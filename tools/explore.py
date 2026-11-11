#!/usr/bin/env python3
"""Cautious step-wise exploration for mapping.

Runs inside the container (slam_toolbox must be running):
    ./ros python3 /work/tools/explore.py [steps]      # default 6

Each step:
  1. For 16 headings, measure live clearance from /scan and the number of
     unknown map cells reachable along that heading (information gain).
  2. Skip headings with < MIN_CLEAR of free space; penalize heading back
     toward recently visited positions.
  3. Rotate to the best heading and drive min(MAX_STEP, clearance - MARGIN),
     with move.py's continuous safety stop.
Stops when no heading is both clear and informative.
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, "/work/tools")
from move import Mover, wrap  # noqa: E402

HEADINGS = 16
MIN_CLEAR = 0.9      # m of free space needed to consider a heading
MARGIN = 0.45        # stop this far short of the obstacle ahead
MAX_STEP = 1.0       # m per step
MIN_STEP = 0.3       # don't bother with shorter moves
GAIN_RANGE = 4.0     # m to look along a heading for unknown cells
MIN_GAIN = 15        # unknown cells needed to make a heading worthwhile
VISIT_RADIUS = 0.6   # m; endpoints this close to past positions are penalized


class Explorer(Mover):
    def __init__(self) -> None:
        super().__init__()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.visited: list[tuple[float, float]] = []

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def map_pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def unknown_gain(self, x: float, y: float, heading: float) -> int:
        """Unknown cells along a 3-ray fan, counted until an occupied cell."""
        m = self.map
        assert m is not None
        info = m.info
        grid = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        seen: set[tuple[int, int]] = set()
        for da in (-0.15, 0.0, 0.15):
            a = heading + da
            for r in np.arange(0.2, GAIN_RANGE, res):
                cx, cy = int((x + r * math.cos(a) - ox) / res), int((y + r * math.sin(a) - oy) / res)
                if not (0 <= cx < info.width and 0 <= cy < info.height):
                    seen.add((cx, cy))  # off-map counts as unknown
                    continue
                v = grid[cy, cx]
                if v >= 65:
                    break
                if v < 0:
                    seen.add((cx, cy))
        return len(seen)

    def choose(self) -> tuple[float, float, int] | None:
        pose = self.map_pose()
        if pose is None or self.map is None:
            return None
        x, y, yaw = pose
        best = None
        for i in range(HEADINGS):
            rel = wrap(2 * math.pi * i / HEADINGS)        # heading in base frame
            clear = self.clearance(rel)
            if clear < MIN_CLEAR:
                continue
            step = min(MAX_STEP, clear - MARGIN)
            gain = self.unknown_gain(x, y, yaw + rel)
            ex, ey = x + step * math.cos(yaw + rel), y + step * math.sin(yaw + rel)
            revisit = any(math.hypot(ex - vx, ey - vy) < VISIT_RADIUS for vx, vy in self.visited)
            score = gain * (0.3 if revisit else 1.0) - abs(rel) * 2   # slight preference to not turn
            print(f"    {math.degrees(rel):+6.0f} deg  clear {clear:4.2f} m  gain {gain:4d}"
                  f"{'  (revisit)' if revisit else ''}")
            if gain >= MIN_GAIN and step >= MIN_STEP and (best is None or score > best[3]):
                best = (rel, step, gain, score)
        return None if best is None else best[:3]


def main() -> None:
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rclpy.init()
    node = Explorer()
    try:
        node.wait_ready()
        node.spin_for(1.0)
        for n in range(1, steps + 1):
            pose = node.map_pose()
            if pose:
                node.visited.append(pose[:2])
            print(f"step {n}: at {pose[0]:.2f},{pose[1]:.2f}" if pose else f"step {n}: pose unknown")
            choice = node.choose()
            if choice is None:
                print("nothing left that is both clear and unexplored; done")
                break
            rel, step, gain = choice
            print(f"  -> turn {math.degrees(rel):+.0f} deg, drive {step:.2f} m (gain {gain})")
            if abs(rel) > math.radians(5):
                print("  ", node.rotate(math.degrees(rel)))
                node.stop()
                node.spin_for(0.5)
            # Re-check after turning: the live scan is what matters.
            clear = node.clearance(0.0)
            step = min(step, clear - MARGIN)
            if step < MIN_STEP:
                print(f"   skipped drive: only {clear:.2f} m clear after turning")
                continue
            print("  ", node.forward(step))
            node.stop()
            node.spin_for(1.5)  # let slam_toolbox integrate
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
