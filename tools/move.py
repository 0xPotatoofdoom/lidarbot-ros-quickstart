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
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

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
ANGULAR_COAST = math.radians(4.0)    # rad; varies 3.0-4.6 deg run to run


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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

    def forward(self, dist: float) -> str:
        heading = 0.0 if dist >= 0 else math.pi
        assert self.odom is not None
        p0 = self.odom.pose.pose.position
        speed = math.copysign(LINEAR_SPEED, dist)
        end = time.monotonic() + abs(dist) / LINEAR_SPEED * TIMEOUT_FACTOR + 1
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
            p = self.odom.pose.pose.position
            done = math.hypot(p.x - p0.x, p.y - p0.y)
            if done >= abs(dist) - LINEAR_COAST:
                self.stop()
                return f"moved {done:.3f} m (+ coast)"
            c = self.clearance(heading)
            if c < STOP_DIST:
                self.stop()
                return f"STOPPED after {done:.2f} m: obstacle {c:.2f} m ahead"
            remaining = abs(dist) - done
            self.send(vx=speed if remaining > 0.05 else speed * 0.5)
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
    rclpy.init()
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
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
