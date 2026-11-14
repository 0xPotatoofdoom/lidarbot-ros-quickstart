#!/usr/bin/env python3
"""Find the pan/tilt mount's real end positions with the camera, without limit switches.

    ./ros python3 /work/tools/ptz_range.py [--axis tilt|pan|both] [--step 3] [--center-pan 0]
                                           [--center-tilt -45]

The servos have no position feedback (/ptz_angle only echoes the command). So the camera is
the sensor: step one servo outward from the center a few degrees at a time and measure how far
the image moves between steps (phase correlation, using the size of the shift so the camera's
roll doesn't matter). In the middle of the range every step moves the image about the same
amount. When a step moves it less than 30% of that, the servo has stopped (its own travel end
or the bracket touching something): go straight back to the last position that moved, record
it, and return to the center. Never holds a servo against a stop.

Needs a textured view in both directions; a blank wall or ceiling can end a sweep early (the
match "response" column shows how sure each match was). Frames are only used in memory.
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

HARD_LIMITS = {"pan": (-90.0, 90.0), "tilt": (-90.0, 40.0)}   # never command past these
STOPPED_FRACTION = 0.3
MIN_RESPONSE = 0.5       # phase-correlation peak; below this the two frames didn't match


def camera_ip() -> str:
    return json.load(open("/work/config/robot_info.json"))["camera_ip"]


def grab(ip: str) -> np.ndarray | None:
    """A fresh frame: the first /capture can be an older buffered frame, so take the second."""
    img = None
    for _ in range(2):
        try:
            data = urllib.request.urlopen(f"http://{ip}/capture", timeout=5).read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        except OSError:
            img = None
    if img is None:
        return None
    return cv2.resize(img, (320, 240)).astype(np.float32)


def shift(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    window = cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(a, b, window)
    return float(np.hypot(dx, dy)), float(response)


class Ptz:
    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(Vector3, "/cmd_ptz_angle", qos_profile_sensor_data)
        self.state: list[Vector3] = []
        node.create_subscription(Vector3, "/ptz_angle", self.state.append, qos_profile_sensor_data)

    def aim(self, pan: float, tilt: float, timeout: float = 3.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pub.publish(Vector3(x=pan, y=tilt, z=0.0))
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.state and abs(self.state[-1].x - pan) < 1.5 and abs(self.state[-1].y - tilt) < 1.5:
                time.sleep(0.6)                      # servo travel + settle
                return True
        return False


def sweep(ptz: Ptz, ip: str, axis: str, center: tuple[float, float], direction: int, step: float) -> dict:
    lo, hi = HARD_LIMITS[axis]
    idx = 0 if axis == "pan" else 1
    pos = list(center)
    ptz.aim(*pos)
    prev = grab(ip)
    prev2 = None                                     # frame from two steps back
    moves: list[float] = []
    rows = []
    end, reason = None, "hard limit"
    while True:
        nxt = pos[idx] + direction * step
        if not lo <= nxt <= hi:
            end = pos[idx]
            break
        target = list(pos)
        target[idx] = nxt
        if not ptz.aim(*target):
            end, reason = pos[idx], "firmware did not accept the command"
            break
        cur = grab(ip)
        if cur is None or prev is None:
            end, reason = pos[idx], "camera stopped answering"
            break
        px, resp = shift(prev, cur)
        rows.append((nxt, px, resp, ""))
        typical = statistics.median(moves) if len(moves) >= 3 else None
        if typical and px < STOPPED_FRACTION * typical:
            # Looks stopped. Repeating texture (tiles, cabinet doors) can fake this, so compare
            # with the frame two steps back: one step's shift if the servo stopped, two if not.
            px2, resp2 = shift(prev2, cur) if prev2 is not None else (0.0, 0.0)
            if resp2 >= MIN_RESPONSE and px2 > 1.5 * typical:
                rows[-1] = (nxt, px, resp, f"false stop: {px2:.1f} px over two steps")
                pos, prev, prev2 = target, cur, prev
                continue
            sure = resp >= MIN_RESPONSE and resp2 >= MIN_RESPONSE
            end = pos[idx]
            reason = (f"stopped moving ({px:.1f} px vs typical {typical:.1f}; {px2:.1f} px over two steps)"
                      if sure else
                      f"uncertain: small shift but low match quality ({resp:.2f}/{resp2:.2f}); end is at least here")
            ptz.aim(*pos)                            # back off immediately either way
            break
        moves.append(px)
        pos, prev, prev2 = target, cur, prev
    ptz.aim(*center)
    return {"axis": axis, "direction": direction, "end": end, "reason": reason, "rows": rows,
            "px_per_deg": (statistics.median(moves) / step) if moves else None}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", choices=["tilt", "pan", "both"], default="both")
    ap.add_argument("--step", type=float, default=3.0)
    ap.add_argument("--center-pan", type=float, default=0.0)
    ap.add_argument("--center-tilt", type=float, default=-45.0)   # power-on position
    args = ap.parse_args()
    ip = camera_ip()
    rclpy.init()
    node = Node("ptz_range")
    ptz = Ptz(node)
    center = (args.center_pan, args.center_tilt)
    axes = ["tilt", "pan"] if args.axis == "both" else [args.axis]
    try:
        for axis in axes:
            for direction in (+1, -1):
                r = sweep(ptz, ip, axis, center, direction, args.step)
                print(f"\n{axis} {'+' if direction > 0 else '-'}: end at {r['end']:+.0f} ({r['reason']}); "
                      f"{r['px_per_deg'] or 0:.2f} px/deg at 320x240")
                for cmd, px, resp, note in r["rows"]:
                    print(f"  {cmd:+6.0f}  shift {px:5.1f} px  response {resp:.2f}  {note}")
    finally:
        ptz.aim(*center)
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
