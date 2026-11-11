#!/usr/bin/env python3
"""Aim the camera pan/tilt mount and wait until it reports the new angle.

./ros python3 /work/tools/ptz.py PAN TILT     degrees; pan -90..90, tilt -90..20
The firmware ignores this while the USB gamepad receiver is plugged in.
"""
from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

PAN_LIMITS = (-90.0, 90.0)
TILT_LIMITS = (-90.0, 20.0)


def clamp(v: float, lo_hi: tuple[float, float]) -> float:
    return max(lo_hi[0], min(lo_hi[1], v))


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    pan, tilt = clamp(float(sys.argv[1]), PAN_LIMITS), clamp(float(sys.argv[2]), TILT_LIMITS)
    rclpy.init()
    node = Node("ptz_cmd")
    pub = node.create_publisher(Vector3, "/cmd_ptz_angle", qos_profile_sensor_data)
    state: list[Vector3] = []
    node.create_subscription(Vector3, "/ptz_angle", state.append, qos_profile_sensor_data)
    end = time.monotonic() + 4.0
    ok = False
    while time.monotonic() < end:
        pub.publish(Vector3(x=pan, y=tilt, z=0.0))
        rclpy.spin_once(node, timeout_sec=0.1)
        if state and abs(state[-1].x - pan) < 1.5 and abs(state[-1].y - tilt) < 1.5:
            ok = True
            break
    time.sleep(0.4)  # let the servos settle before anyone grabs a frame
    rclpy.shutdown()
    last = f"pan {state[-1].x:.0f} tilt {state[-1].y:.0f}" if state else "no /ptz_angle feedback"
    print(("ok: " if ok else "not confirmed: ") + last)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
