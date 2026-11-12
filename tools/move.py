#!/usr/bin/env python3
"""Closed-loop moves with a lidar safety stop.

Runs inside the container:
    ./ros python3 /work/tools/move.py rotate 90      # degrees, + = left (CCW)
    ./ros python3 /work/tools/move.py forward 0.5    # meters, negative = backward

Distance/angle come from the EKF odometry (/odom). Before and during a
straight move, the live /scan is checked in a robot-width corridor along the direction of
travel; the robot stops if anything is closer than STOP_DIST.
"""
from __future__ import annotations

import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scan_anchor import Located, ScanAnchor, anchor_in_odom, plausible  # noqa: E402

LINEAR_SPEED = 0.15      # m/s
ANGULAR_SPEED = 0.5      # rad/s
STOP_DIST = 0.35         # m, measured from the robot center
ROBOT_HALF_WIDTH = 0.12  # m (wheel track 0.171 m + tyres, rounded up)
SIDE_MARGIN = 0.07       # m of extra clearance on each side
CORRIDOR_HALF = ROBOT_HALF_WIDTH + SIDE_MARGIN
# Mirrors and glossy surfaces return no echo (range 0.0), which would look
# like open space. If too many beams near the heading are empty, assume an
# unseen obstacle and cap the clearance.
BLIND_ARC = math.radians(15)
BLIND_FRACTION = 0.3
BLIND_CLEARANCE = 0.6    # m
LASER_X = -0.028         # laser offset behind base_footprint (m)
RATE = 20.0
TIMEOUT_FACTOR = 3.0     # give up after 3x the nominal duration
# The robot keeps moving briefly after the stop command (Wi-Fi latency plus motor
# run-down). Measured 2026-11-11 on hardwood: ~18 mm after a 1 m drive and ~4.6 deg
# after a turn, plus up to one 20 Hz control tick. Stopping this much early lands on target.
LINEAR_COAST = 0.022                 # m
# Straight-line driving holds the starting heading and steers back onto the line
# through the start point. Without it the robot turned ~1 deg and slid 3-20 mm
# sideways on every 1 m out-and-back (mostly while reversing).
HEADING_GAIN = 2.0       # rad/s per rad of heading error
CROSS_TRACK_GAIN = 3.0   # rad/s per m of sideways offset
MAX_STEER = 0.3          # rad/s
ANGULAR_COAST = math.radians(4.0)    # rad; varies 3.0-4.6 deg run to run
def init_ros() -> None:
    """rclpy.init without its Ctrl-C handler, so a `finally: node.stop()` can still publish.

    rclpy's own handler shuts the context down first, and the stop command then fails,
    leaving the robot on its last velocity. SIGTERM (plain pkill) is treated the same way.
    """
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def line_follow_wz(heading_err: float, cross: float, direction: float) -> float:
    """Steering command to stay on a straight line.

    heading_err: line heading minus robot heading (rad); cross: robot's sideways
    offset from the line, left positive (m); direction: +1 forward, -1 reverse.
    Reversing flips the effect of a heading change on the sideways offset, so the
    cross-track term changes sign; the heading term doesn't.
    """
    wz = HEADING_GAIN * heading_err - CROSS_TRACK_GAIN * cross * direction
    return max(-MAX_STEER, min(MAX_STEER, wz))


class Mover(Node):
    def __init__(self) -> None:
        super().__init__("lidarbot_move")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom: Odometry | None = None
        self.scan: LaserScan | None = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)

    def _odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def wait_ready(self, timeout: float = 5.0) -> None:
        end = time.monotonic() + timeout
        while (self.odom is None or self.scan is None) and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None or self.scan is None:
            raise SystemExit("no /odom or /scan; is the robot connected?")

    def clearance(self, heading: float) -> float:
        """Distance to the nearest obstacle in the robot's swept path along heading.

        The path is a straight corridor of half-width CORRIDOR_HALF centered on
        the robot, so side walls of a narrow hallway don't count as blockers.
        heading is in radians in the base frame (0 = forward).
        """
        s = self.scan
        assert s is not None
        ch, sh = math.cos(heading), math.sin(heading)
        best = math.inf
        arc_total = arc_empty = 0
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            valid = s.range_min < r < s.range_max
            if abs(wrap(a - heading)) <= BLIND_ARC:
                arc_total += 1
                arc_empty += 0 if valid else 1
            if not valid:
                continue
            x, y = r * math.cos(a) + LASER_X, r * math.sin(a)
            along, across = x * ch + y * sh, -x * sh + y * ch
            if along > 0 and abs(across) < CORRIDOR_HALF:
                best = min(best, along)
        if arc_total and arc_empty / arc_total > BLIND_FRACTION:
            best = min(best, BLIND_CLEARANCE)
        return best

    def blind_fraction(self, heading: float) -> float:
        """Fraction of beams within BLIND_ARC of heading that returned nothing."""
        s = self.scan
        assert s is not None
        beams = [s.range_min < r < s.range_max for i, r in enumerate(s.ranges)
                 if abs(wrap(s.angle_min + i * s.angle_increment - heading)) <= BLIND_ARC]
        return 1.0 - sum(beams) / len(beams) if beams else 0.0

    def send(self, vx: float = 0.0, wz: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def stop(self) -> None:
        for _ in range(5):
            self.send()
            time.sleep(0.05)

    def pose(self) -> tuple[float, float, float]:
        """Current (x, y, yaw) from /odom; pass it to forward(line=...) to reuse a line."""
        assert self.odom is not None
        p = self.odom.pose.pose.position
        return p.x, p.y, yaw_of(self.odom)

    def fresh_scans(self, n: int, timeout: float = 3.0) -> list[LaserScan]:
        got, last = [], self.scan
        end = time.monotonic() + timeout
        while len(got) < n and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan is not None and self.scan is not last:
                last = self.scan
                got.append(last)
        return got

    def along(self, line: tuple[float, float, float]) -> float:
        """How far the robot is along line (m, from the line's origin in its direction)."""
        x, y, _ = self.pose()
        return (x - line[0]) * math.cos(line[2]) + (y - line[1]) * math.sin(line[2])

    def anchor(self) -> ScanAnchor:
        """Reference scan at the current pose (robot still). See relocate()."""
        return ScanAnchor(self.fresh_scans(1)[0], self.pose())

    def relocate(self, anchor: ScanAnchor, line: tuple[float, float, float] | None = None,
                 scans: int = 5) -> tuple[tuple[float, float, float], Located] | None:
        """Robot still near the anchor spot: the anchor's line in current odometry coordinates
        (pass it to forward(line=...)) and the robot's pose relative to the anchor.

        Odometry drifts slowly from cycle to cycle; this resets the line to where the walls
        say it is. Pass the line from the previous relocate() so matching starts from the
        latest correction. Median over a few scans; None if they don't match, or if the
        match is too far from odometry's guess to be real (scan_anchor.plausible).
        """
        locs = [loc for sc in self.fresh_scans(scans)
                if (loc := anchor.locate(sc, self.pose(), line)) is not None]
        if len(locs) < 3:
            return None
        med = sorted(locs, key=lambda l: l.y)[len(locs) // 2]
        yaw = sorted(l.yaw for l in locs)[len(locs) // 2]
        located = Located(med.x, med.y, yaw, med.rms, med.inliers)
        pose = self.pose()
        guess = anchor.odom_guess(pose, line)
        if not plausible(located, guess):
            print(f"  scan match rejected: {located.x * 1000:+.1f}, {located.y * 1000:+.1f} mm, "
                  f"{math.degrees(located.yaw):+.2f} deg, odometry says {guess[0] * 1000:+.1f}, "
                  f"{guess[1] * 1000:+.1f} mm, {math.degrees(guess[2]):+.2f} deg", flush=True)
            return None
        return anchor_in_odom(pose, located), located

    def forward(self, dist: float, line: tuple[float, float, float] | None = None) -> str:
        """Drive dist meters (negative = reverse) in a straight line.

        line: (x, y, yaw) of the line to follow; default is the current pose. For an
        out-and-back, pass the start pose to both legs so the return steers back onto
        the same line instead of starting a new one from wherever the robot coasted to.
        To stop slow drift over many cycles, re-measure the line with relocate() each time
        the robot is back at the start.
        """
        heading = 0.0 if dist >= 0 else math.pi
        assert self.odom is not None
        p0 = self.odom.pose.pose.position
        lx, ly, yaw0 = line if line is not None else self.pose()
        ux, uy = math.cos(yaw0), math.sin(yaw0)
        direction = math.copysign(1.0, dist)
        speed = direction * LINEAR_SPEED
        end = time.monotonic() + abs(dist) / LINEAR_SPEED * TIMEOUT_FACTOR + 1
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
            p = self.odom.pose.pose.position
            done = ((p.x - p0.x) * ux + (p.y - p0.y) * uy) * direction  # progress along the line
            cross = -(p.x - lx) * uy + (p.y - ly) * ux                 # sideways offset, left positive
            heading_err = wrap(yaw0 - yaw_of(self.odom))
            if done >= abs(dist) - LINEAR_COAST:
                self.stop()
                return f"moved {done:.3f} m (+ coast), {cross * 1000:+.1f} mm off line"
            c = self.clearance(heading)
            if c < STOP_DIST:
                self.stop()
                return f"STOPPED after {done:.2f} m: obstacle {c:.2f} m ahead"
            remaining = abs(dist) - done
            wz = line_follow_wz(heading_err, cross, direction)
            self.send(vx=speed if remaining > 0.05 else speed * 0.5, wz=wz)
        self.stop()
        return "timed out"

    def rotate(self, deg: float) -> str:
        assert self.odom is not None
        target = math.radians(deg)
        turned, last = 0.0, yaw_of(self.odom)
        speed = math.copysign(ANGULAR_SPEED, target)
        end = time.monotonic() + abs(target) / ANGULAR_SPEED * TIMEOUT_FACTOR + 1
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
            y = yaw_of(self.odom)
            turned += wrap(y - last)
            last = y
            remaining = abs(target) - abs(turned)
            if remaining <= ANGULAR_COAST:
                self.stop()
                return f"rotated {math.degrees(turned):.1f} deg (+ coast)"
            self.send(wz=speed if remaining > math.radians(15) else speed * 0.5)
        self.stop()
        return f"timed out after {math.degrees(turned):.1f} deg"


def main() -> None:
    if len(sys.argv) != 3 or sys.argv[1] not in ("forward", "rotate"):
        raise SystemExit(__doc__)
    cmd, value = sys.argv[1], float(sys.argv[2])
    init_ros()
    node = Mover()
    try:
        node.wait_ready()
        if cmd == "forward":
            heading = 0.0 if value >= 0 else math.pi
            c = node.clearance(heading)
            if c < STOP_DIST + abs(value):
                print(f"refusing: only {c:.2f} m clear in that direction")
                return
            result = node.forward(value)
        else:
            result = node.rotate(value)
        node.stop()
        print(result)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
