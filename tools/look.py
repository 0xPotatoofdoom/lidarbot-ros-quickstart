#!/usr/bin/env python3
"""Save a fresh camera frame from /camera/image_raw (camera_driver must be running).

./ros python3 /work/tools/look.py [out.jpg]     default: /root/map/camera.jpg (maps/ on the Mac)
Only frames that arrive after the script starts (plus a short settle time)
are used, so the image is never stale. Prints the camera pan/tilt.
"""
from __future__ import annotations

import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

SETTLE_S = 0.6     # ignore frames already in flight when we start
TIMEOUT_S = 8.0


def to_bgr(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return img if msg.encoding == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/root/map/camera.jpg"
    rclpy.init()
    node = Node("look")
    start = time.monotonic()
    frame: list[Image] = []
    ptz: list[Vector3] = []
    node.create_subscription(
        Image, "/camera/image_raw",
        lambda m: frame.append(m) if time.monotonic() - start > SETTLE_S else None,
        qos_profile_sensor_data)
    node.create_subscription(Vector3, "/ptz_angle", ptz.append, qos_profile_sensor_data)
    while not frame and time.monotonic() - start < TIMEOUT_S:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()
    if not frame:
        print("no fresh frame; is camera_driver running?", file=sys.stderr)
        return 1
    cv2.imwrite(out, to_bgr(frame[-1]))
    pt = f", pan {ptz[-1].x:.0f} tilt {ptz[-1].y:.0f}" if ptz else ""
    print(f"saved {out} ({frame[-1].width}x{frame[-1].height}{pt})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
