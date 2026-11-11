#!/usr/bin/env python3
"""Rotate in place and compare the scan-estimated rotation with odometry.

./ros python3 /work/tools/scan_rotation_check.py [deg] [topic]
Finds the circular index shift that best aligns the scans before and after
the turn. For a correct REP-103 scan the estimate has the same sign as the
odometry turn; the ratio of the two shows the angular calibration error.
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, "/work/tools")
from move import Mover  # noqa: E402

SCANS_PER_SAMPLE = 5
MIN_OVERLAP = 0.3  # fraction of beams that must be valid in both scans


def grab(node: Mover, topic: str) -> tuple[np.ndarray, LaserScan]:
    """Median of a few scans, with invalid ranges as NaN."""
    got: list[LaserScan] = []
    sub = node.create_subscription(LaserScan, topic, got.append, qos_profile_sensor_data)
    end = time.monotonic() + 3
    while len(got) < SCANS_PER_SAMPLE and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    if not got:
        raise SystemExit(f"no messages on {topic}")
    r = np.array([list(m.ranges) for m in got], dtype=float)
    r[~np.isfinite(r) | (r <= 0.05)] = np.nan
    with np.errstate(all="ignore"):
        return np.nanmedian(r, axis=0), got[-1]


def best_shift(a: np.ndarray, b: np.ndarray) -> tuple[int, float]:
    """Signed index shift k minimizing mean |roll(a, k) - b|."""
    n = len(a)
    errs = []
    for k in range(n):
        d = np.roll(a, k) - b
        m = np.isfinite(d)
        errs.append(float(np.mean(np.abs(d[m]))) if m.sum() > n * MIN_OVERLAP else math.inf)
    k = int(np.argmin(errs))
    return (k if k <= n // 2 else k - n), errs[k]


def main() -> None:
    deg = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    topic = sys.argv[2] if len(sys.argv) > 2 else "/scan"
    rclpy.init()
    node = Mover()
    try:
        node.wait_ready()
        a, msg = grab(node, topic)
        print(node.rotate(deg))
        node.stop()
        time.sleep(0.7)
        b, _ = grab(node, topic)
        k, err = best_shift(a, b)
        inc = msg.angle_increment
        print(f"{topic}: N={len(a)}  angle_increment={math.degrees(inc):.3f} deg  "
              f"span={math.degrees(inc * len(a)):.1f} deg")
        print(f"scan says rotation = {-k * math.degrees(inc):+.1f} deg "
              f"(shift {k} samples, mean residual {err:.3f} m)")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
