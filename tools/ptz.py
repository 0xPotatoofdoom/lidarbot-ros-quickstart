#!/usr/bin/env python3
"""Aim the camera pan/tilt mount and wait until it reports the new angle.

./ros python3 /work/tools/ptz.py PAN TILT     degrees; pan -90..90, tilt -90..15 (-45, the power-on
                                              position, faces forward on ours since 2026-11-13)
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
# Measured with tools/ptz_range.py after the horn was re-seated (2026-11-13): the tilt moves
# freely from -90 to at least +15; above +15 is untested.
TILT_LIMITS = (-90.0, 15.0)


def clamp(v: float, lo_hi: tuple[float, float]) -> float:
    return float(max(lo_hi[0], min(lo_hi[1], v)))


def aim(node: Node, pan: float, tilt: float, timeout: float = 4.0) -> Vector3 | None:
    """Command pan/tilt on an existing node until /ptz_angle confirms it; the reported angle,
    or None if it never confirmed."""
    pan, tilt = clamp(pan, PAN_LIMITS), clamp(tilt, TILT_LIMITS)
    pub = node.create_publisher(Vector3, "/cmd_ptz_angle", qos_profile_sensor_data)
    state: list[Vector3] = []
    sub = node.create_subscription(Vector3, "/ptz_angle", state.append, qos_profile_sensor_data)
    end = time.monotonic() + timeout
    ok = False
    try:
        while time.monotonic() < end:
            pub.publish(Vector3(x=pan, y=tilt, z=0.0))
            rclpy.spin_once(node, timeout_sec=0.1)
            if state and abs(state[-1].x - pan) < 1.5 and abs(state[-1].y - tilt) < 1.5:
                ok = True
                break
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
    time.sleep(0.4)  # let the servos settle before anyone grabs a frame
    return state[-1] if ok else None


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    rclpy.init()
    node = Node("ptz_cmd")
    got = aim(node, float(sys.argv[1]), float(sys.argv[2]))
    rclpy.shutdown()
    print(f"ok: pan {got.x:.0f} tilt {got.y:.0f}" if got else "not confirmed: no matching /ptz_angle feedback")
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
