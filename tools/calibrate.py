#!/usr/bin/env python3
"""Measure odometry scale errors against the lidar.

Runs inside the container (./up must be running, robot on the floor):
    ./ros python3 /work/tools/calibrate.py angular [turns]     default 1 turn each way
    ./ros python3 /work/tools/calibrate.py linear [meters]     default 1.0 m, needs a wall ahead
    ./ros python3 /work/tools/calibrate.py out|back [meters]   one leg only, e.g. to read a tape

angular: turns in place by `turns` full revolutions left, then right. The true
rotation is the EKF's estimate corrected by aligning the lidar scans taken
before and after (accurate to a fraction of a degree, as long as the error is
under 180 degrees).

linear: drives straight toward a wall and back. The true distance is the change
in the lidar range straight ahead, which doesn't depend on the wall's angle.
Mark the robot's starting point on a tape measure if you want a manual check.

Each source is measured on its own: the firmware's /odom_raw pose, its speeds
integrated over arrival time (shown for comparison; Wi-Fi jitter makes this
under-count by about 2%), /imu yaw and gyro, and the /odom EKF. Suggested
factors for config/calibration.yaml use the /odom_raw pose and /imu yaw.
"""
from __future__ import annotations

import math
import statistics
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, "/work/tools")
from move import STOP_DIST, Mover, wrap, yaw_of  # noqa: E402

SCANS_PER_SAMPLE = 7
SETTLE_S = 1.0
MIN_OVERLAP = 0.3
SELF_RANGE = 0.2                # m; closer returns are the robot's own body
AHEAD_ARC = math.radians(2)     # beams averaged for the range straight ahead
MAX_WALL = 5.0                  # m; farther walls are too sparse to trust
MAX_DT = 0.25                   # s; ignore gaps longer than this when integrating


@dataclass(frozen=True)
class Totals:
    """Snapshot of what each source measured during one run."""
    raw_dist: float
    raw_pose_dist: float  # displacement of the firmware's own /odom_raw pose
    raw_turn: float
    raw_pose_turn: float  # heading change of the firmware's own /odom_raw pose
    gyro_turn: float
    imu_yaw_turn: float
    ekf_dist: float
    ekf_turn: float
    counts: tuple[int, int, int]  # raw, imu, ekf messages


def quat_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder:
    """Integrates each motion source independently while recording is on."""

    def __init__(self, node: Mover) -> None:
        self.on = False
        self.reset()
        self._last: dict[str, float] = {}
        self._start_xy: tuple[float, float] | None = None
        self._yaw: dict[str, float] = {}
        node.create_subscription(Odometry, "/odom_raw", self._raw, qos_profile_sensor_data)
        node.create_subscription(Imu, "/imu", self._imu, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odom", self._ekf, 10)

    def reset(self) -> None:
        self.raw_dist = self.raw_turn = self.raw_pose_dist = self.raw_pose_turn = 0.0
        self._raw_start: tuple[float, float] | None = None
        self.gyro_turn = self.imu_yaw_turn = 0.0
        self.ekf_dist = self.ekf_turn = 0.0
        self._start_xy = None
        self.counts = {"raw": 0, "imu": 0, "ekf": 0}

    def snapshot(self) -> Totals:
        c = self.counts
        return Totals(self.raw_dist, self.raw_pose_dist, self.raw_turn,
                      self.raw_pose_turn, self.gyro_turn, self.imu_yaw_turn,
                      self.ekf_dist, self.ekf_turn, (c["raw"], c["imu"], c["ekf"]))

    def _dt(self, key: str) -> float:
        now = time.monotonic()
        last = self._last.get(key)
        self._last[key] = now
        if last is None or not self.on:
            return 0.0
        dt = now - last
        return dt if dt < MAX_DT else 0.0

    def _yaw_step(self, key: str, yaw: float) -> float:
        last = self._yaw.get(key)
        self._yaw[key] = yaw
        return wrap(yaw - last) if (last is not None and self.on) else 0.0

    def _raw(self, m: Odometry) -> None:
        dt = self._dt("raw")
        self.raw_dist += m.twist.twist.linear.x * dt
        self.raw_turn += m.twist.twist.angular.z * dt
        self.raw_pose_turn += self._yaw_step("raw", yaw_of(m))
        if self.on:
            p = m.pose.pose.position
            if self._raw_start is None:
                self._raw_start = (p.x, p.y)
            self.raw_pose_dist = math.hypot(p.x - self._raw_start[0], p.y - self._raw_start[1])
        self.counts["raw"] += self.on

    def _imu(self, m: Imu) -> None:
        dt = self._dt("imu")
        self.gyro_turn += m.angular_velocity.z * dt
        self.imu_yaw_turn += self._yaw_step("imu", quat_yaw(m.orientation))
        self.counts["imu"] += self.on

    def _ekf(self, m: Odometry) -> None:
        p = m.pose.pose.position
        if self.on:
            # straight-line displacement: summing small steps would add up jitter
            if self._start_xy is None:
                self._start_xy = (p.x, p.y)
            self.ekf_dist = math.hypot(p.x - self._start_xy[0], p.y - self._start_xy[1])
        self.ekf_turn += self._yaw_step("ekf", yaw_of(m))
        self.counts["ekf"] += self.on


def grab_scan(node: Mover) -> tuple[np.ndarray, LaserScan]:
    """Median of several scans; invalid ranges are NaN."""
    got: list[LaserScan] = []
    end = time.monotonic() + 3
    last = node.scan
    while len(got) < SCANS_PER_SAMPLE and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.scan is not None and node.scan is not last:
            got.append(node.scan)
            last = node.scan
    if len(got) < 3:
        raise SystemExit("not enough /scan messages")
    r = np.array([list(m.ranges) for m in got], dtype=float)
    r[~np.isfinite(r) | (r <= SELF_RANGE)] = np.nan  # drop returns from the robot itself
    with np.errstate(all="ignore"):
        return np.nanmedian(r, axis=0), got[-1]


def scan_rotation(a: np.ndarray, b: np.ndarray, inc: float) -> tuple[float, float]:
    """Rotation (rad, CCW +) taking scan a to scan b, with sub-beam refinement."""
    n = len(a)
    errs = np.full(n, np.inf)
    for k in range(n):
        d = np.roll(a, k) - b
        m = np.isfinite(d)
        if m.sum() > n * MIN_OVERLAP:
            errs[k] = float(np.mean(np.abs(d[m])))
    k = int(np.argmin(errs))
    e0, em, ep = errs[k], errs[(k - 1) % n], errs[(k + 1) % n]
    frac = 0.0
    denom = em - 2 * e0 + ep
    if np.isfinite(denom) and denom > 1e-9:
        frac = 0.5 * (em - ep) / denom
    shift = k + frac
    if shift > n / 2:
        shift -= n
    # A CCW turn makes features appear at smaller beam angles: negative shift.
    return -shift * inc, float(e0)


def range_ahead(r: np.ndarray, s: LaserScan, heading: float = 0.0) -> float:
    vals = [r[i] for i in range(len(r))
            if abs(wrap(s.angle_min + i * s.angle_increment - heading)) <= AHEAD_ARC
            and np.isfinite(r[i])]
    if len(vals) < 3:
        return math.nan
    return float(np.median(vals))


def settle(node: Mover) -> None:
    node.stop()
    end = time.monotonic() + SETTLE_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def ratio(true: float, measured: float) -> str:
    return f"{true / measured:.4f}" if abs(measured) > 1e-6 else "n/a"


def run(node: Mover, rec: Recorder, action) -> tuple[str, Totals]:
    """Record every source while action() moves the robot, then let it settle."""
    rec.reset()
    rec.on = True
    try:
        result = action()
        settle(node)
    finally:
        rec.on = False
    return result, rec.snapshot()


def angular(node: Mover, rec: Recorder, turns: float) -> None:
    rows: list[tuple[float, Totals]] = []
    for sign, label in ((1, "left (CCW)"), (-1, "right (CW)")):
        a, msg = grab_scan(node)
        result, t = run(node, rec, lambda s=sign: node.rotate(s * turns * 360))
        print(f"turning {label} {turns * 360:.0f} deg: {result}")
        b, _ = grab_scan(node)
        rot, resid = scan_rotation(a, b, msg.angle_increment)
        true = t.ekf_turn + wrap(rot - t.ekf_turn)
        rows.append((true, t))
        print(f"  lidar says {math.degrees(true):+.1f} deg (scan match residual {resid:.3f} m)")
        print(f"  {'source':<22}{'measured':>10}{'true/meas':>11}")
        for name, val in (("/odom_raw speed x time", t.raw_turn), ("/odom_raw pose", t.raw_pose_turn), ("/imu gyro z", t.gyro_turn),
                          ("/imu yaw", t.imu_yaw_turn), ("/odom EKF", t.ekf_turn)):
            print(f"  {name:<22}{math.degrees(val):>+9.1f}°{ratio(true, val):>11}")
        print(f"  messages (raw, imu, ekf): {t.counts}")
    (lt, left), (rt, right) = rows
    print("\nsuggested config/calibration.yaml values:")
    print(f"  angular_scale_left:  {ratio(lt, left.raw_pose_turn)}")
    print(f"  angular_scale_right: {ratio(rt, right.raw_pose_turn)}")
    print(f"  imu_yaw_scale:       left {ratio(lt, left.imu_yaw_turn)}, "
          f"right {ratio(rt, right.imu_yaw_turn)} (use the mean)")


def linear(node: Mover, rec: Recorder, dist: float, legs: tuple[int, ...] = (1, -1)) -> None:
    a, s = grab_scan(node)
    ahead = range_ahead(a, s)
    if not math.isfinite(ahead) or ahead > MAX_WALL:
        raise SystemExit(f"need a wall straight ahead within {MAX_WALL} m (got {ahead:.2f})")
    if 1 in legs and ahead < dist + STOP_DIST + 0.15:
        raise SystemExit(f"wall is {ahead:.2f} m ahead; need {dist + STOP_DIST + 0.15:.2f} m for {dist} m")
    print(f"wall {ahead:.3f} m ahead. Mark the start on your tape now if you want a manual check.")
    scales = []
    for sign in legs:
        label = "forward" if sign > 0 else "back"
        r0 = range_ahead(*grab_scan(node))
        result, t = run(node, rec, lambda s=sign: node.forward(s * dist))
        print(f"driving {label} {dist:.2f} m: {result}")
        true = r0 - range_ahead(*grab_scan(node))
        ekf, pose = t.ekf_dist * sign, t.raw_pose_dist * sign
        print(f"  lidar {true:+.3f} m")
        for name, val in (("/odom_raw speed x time", t.raw_dist), ("/odom_raw pose", pose),
                          ("/odom EKF", ekf)):
            print(f"    {name:<24}{val:+.3f} m   true/meas {ratio(true, val)}")
        if abs(pose) > 1e-3:
            scales.append(true / pose)
        if label == "forward" and len(legs) > 1:
            print("  robot is at the far point: read the tape now if checking manually.")
            input_wait(node, 15.0)
    if scales:
        detail = ", ".join(f"{x:.4f}" for x in scales)
        print(f"\nsuggested linear_scale: {statistics.mean(scales):.4f} (forward, back: {detail})")


def input_wait(node: Mover, seconds: float) -> None:
    """Pause (spinning, so subscriptions stay fresh) to give time for a tape reading."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in ("angular", "linear", "out", "back"):
        raise SystemExit(__doc__)
    mode = sys.argv[1]
    amount = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rclpy.init()
    node = Mover()
    rec = Recorder(node)
    try:
        node.wait_ready()
        settle(node)
        if mode == "angular":
            angular(node, rec, amount)
        else:
            legs = {"linear": (1, -1), "out": (1,), "back": (-1,)}[mode]
            linear(node, rec, amount, legs)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
