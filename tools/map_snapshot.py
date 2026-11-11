#!/usr/bin/env python3
"""Save the current /map as a PNG with the robot pose drawn on it.

Runs inside the container:  ./ros python3 /work/tools/map_snapshot.py [out.png]
Default output: /work/tools/../maps/snapshot.png (i.e. lidarbot/maps/ on the Mac).
Prints map size, robot pose, and the distance to the nearest obstacle per
45-degree sector around the robot (from the map, not the live scan).
"""
from __future__ import annotations

import math
import sys
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

SCALE = 4  # output pixels per map cell
SECTORS = ("front", "front-left", "left", "back-left", "back", "back-right", "right", "front-right")


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__("map_snapshot")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw


def sector_clearance(grid: np.ndarray, info, x: float, y: float, yaw: float) -> list[float]:
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    out = []
    for i in range(len(SECTORS)):
        best = math.inf
        for da in np.linspace(-math.pi / 8, math.pi / 8, 9):
            a = yaw + i * math.pi / 4 + da
            for r in np.arange(0.1, 6.0, res / 2):
                cx = int((x + r * math.cos(a) - ox) / res)
                cy = int((y + r * math.sin(a) - oy) / res)
                if not (0 <= cx < info.width and 0 <= cy < info.height):
                    break
                if grid[cy, cx] >= 65:
                    best = min(best, r)
                    break
        out.append(best)
    return out


def main() -> None:
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/root/map/snapshot.png"
    rclpy.init()
    node = Snapshot()
    deadline = time.monotonic() + 8
    pose = None
    while time.monotonic() < deadline and (node.map is None or pose is None):
        rclpy.spin_once(node, timeout_sec=0.2)
        pose = node.pose()
    if node.map is None:
        raise SystemExit("no /map received")
    info = node.map.info
    grid = np.array(node.map.data, dtype=np.int16).reshape(info.height, info.width)

    img = np.full(grid.shape, 128, np.uint8)  # unknown = grey
    img[(grid >= 0) & (grid < 25)] = 255      # free = white
    img[grid >= 65] = 0                       # occupied = black
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)

    if pose:
        x, y, yaw = pose
        px = int((x - info.origin.position.x) / info.resolution * SCALE)
        py = int((y - info.origin.position.y) / info.resolution * SCALE)
        tip = (int(px + 25 * math.cos(yaw)), int(py + 25 * math.sin(yaw)))
        cv2.circle(img, (px, py), 8, (0, 0, 255), -1)
        cv2.arrowedLine(img, (px, py), tip, (0, 0, 255), 3, tipLength=0.4)
    img = cv2.flip(img, 0)  # map y up
    # 1 m scale bar
    bar = int(1.0 / info.resolution * SCALE)
    cv2.line(img, (10, img.shape[0] - 10), (10 + bar, img.shape[0] - 10), (255, 0, 0), 3)
    cv2.imwrite(out_path, img)

    known = int((grid >= 0).sum())
    print(f"map {info.width}x{info.height} cells @ {info.resolution} m "
          f"({info.width * info.resolution:.1f} x {info.height * info.resolution:.1f} m), "
          f"known {known * info.resolution ** 2:.1f} m^2, saved {out_path}")
    if pose:
        x, y, yaw = pose
        print(f"robot at x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg")
        for name, d in zip(SECTORS, sector_clearance(grid, info, x, y, yaw), strict=True):
            print(f"  {name:<12} {'open/unknown' if math.isinf(d) else f'{d:.2f} m'}")
    else:
        print("robot pose unavailable (no map->base_footprint TF)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
