#!/usr/bin/env python3
"""Drive to a map coordinate in a straight line, in checked steps.

./ros python3 /work/tools/goto.py X Y [max_step]
Turns toward (X, Y) in the map frame, then drives up to max_step (default
1.0 m) at a time with move.py's safety stop, re-aiming between steps.
Stops within ARRIVE_TOL of the goal or when the way ahead is blocked.
"""
from __future__ import annotations

import math
import sys

import rclpy

sys.path.insert(0, "/work/tools")
from explore import MARGIN, Explorer  # noqa: E402
from move import wrap  # noqa: E402

ARRIVE_TOL = 0.15          # m
AIM_TOL = math.radians(4)  # re-aim if heading error exceeds this
MAX_LEGS = 10


def drive_to(node: Explorer, gx: float, gy: float, max_step: float) -> str:
    for _ in range(MAX_LEGS):
        pose = node.map_pose()
        if pose is None:
            return "no map pose available"
        x, y, yaw = pose
        dist = math.hypot(gx - x, gy - y)
        if dist < ARRIVE_TOL:
            return f"arrived at {x:.2f},{y:.2f}"
        turn = wrap(math.atan2(gy - y, gx - x) - yaw)
        if abs(turn) > AIM_TOL:
            print(" ", node.rotate(math.degrees(turn)))
            node.stop()
            node.spin_for(0.7)
        clear = node.clearance(0.0)
        step = min(max_step, dist, clear - MARGIN)
        if step < 0.1:
            return f"blocked: {clear:.2f} m clear, {dist:.2f} m from goal"
        print(f"  {dist:.2f} m to go, driving {step:.2f} m:", node.forward(step))
        node.stop()
        node.spin_for(1.5)
    return f"gave up after {MAX_LEGS} legs"


def main() -> None:
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    max_step = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    rclpy.init()
    node = Explorer()
    try:
        node.wait_ready()
        node.spin_for(1.0)
        print(drive_to(node, gx, gy, max_step))
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
