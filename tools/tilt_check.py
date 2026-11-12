#!/usr/bin/env python3
"""Measure how the camera tilt command maps to the camera's real pitch.

Face a wall 1-2 m away with bare floor in front of it (the 1 m tape run facing the door is
fine), then:
    ./ros python3 /work/tools/tilt_check.py               # sweep -15..+6 in 3 deg steps
    ./ros python3 /work/tools/tilt_check.py --cmds -70 -65 -60

For each command it finds the row where the floor meets the wall, and with the lidar's
distance to the wall and the camera's height works out how far below level the camera
points. A straight-line fit gives the command that points the camera level and the real
degrees per commanded degree.

Measured 2026-11-12 before re-seating the tilt horn: level at -60.9, 1.000 scale (0.15 deg rms), mechanical
stop near -80. Don't command below the stop: at -90 the stalled servo dropped the camera.
After re-seating the horn: level at +5.9, 1.008 scale (1.1 deg rms; the servo moves in
uneven 2-4 deg steps).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

CAM_HEIGHT_M = 0.12        # optical center above the floor (URDF: 0.02 + 0.060 + 0.025 + 0.0145)
CAM_AHEAD_OF_LIDAR_M = 0.118   # camera is about 0.09 m ahead of base_link, the lidar 0.028 behind
EDGE_ROWS = 8              # rows averaged on each side of a candidate junction
BORDER_ROWS = 12           # undistortion leaves black rows at the top and bottom
MIN_STEP = 20.0            # redder-plus-darker jump that counts as wall -> floor
STREAM_LAG_S = 3.0         # ignore frames this soon after a move (stream lag, measured)
SAVE_DIR = Path("/root/datasets/camera_cal")


@dataclass(frozen=True)
class TiltFit:
    level_cmd: float   # command that points the camera level
    scale: float       # real degrees per commanded degree
    rms_deg: float


def junction_row(bgr: np.ndarray) -> int | None:
    """Row where a light wall meets a darker orange wood floor, or None.

    Scores each row by how much redder (red minus blue) and darker the rows below are than
    the rows above. Medians over columns ignore blue tape and furniture legs; rows near the
    border, which undistortion leaves black, are skipped.
    """
    img = bgr.astype(np.float32)
    rb = np.median(img[:, :, 2] - img[:, :, 0], axis=1)
    gray = np.median(img.mean(axis=2), axis=1)
    rows = np.arange(BORDER_ROWS + EDGE_ROWS, len(rb) - BORDER_ROWS - EDGE_ROWS + 1)

    def step(profile: np.ndarray) -> np.ndarray:
        c = np.concatenate([[0.0], np.cumsum(profile)])
        return (c[rows + EDGE_ROWS] - c[rows]) / EDGE_ROWS - (c[rows] - c[rows - EDGE_ROWS]) / EDGE_ROWS

    score = step(rb) - step(gray)
    best = int(np.argmax(score))
    return int(rows[best]) if score[best] >= MIN_STEP else None


def pitch_down_deg(row: float, cy: float, fy: float, cam_h: float, dist: float) -> float:
    """Camera pitch below level, from the junction's image row and the wall distance."""
    return math.degrees(math.atan2(cam_h, dist)) - math.degrees(math.atan2(row - cy, fy))


def fit_tilt(cmds: list[float], downs: list[float]) -> TiltFit:
    """Least-squares line down = scale * (level_cmd - cmd)."""
    if len(cmds) < 2:
        raise ValueError("need at least two tilt commands with a visible floor line")
    a, b = np.polyfit(np.asarray(cmds), np.asarray(downs), 1)   # down = a * cmd + b
    resid = np.asarray(downs) - (a * np.asarray(cmds) + b)
    return TiltFit(level_cmd=float(-b / a), scale=float(-a), rms_deg=float(np.sqrt(np.mean(resid ** 2))))


# ---------------------------------------------------------------- robot side

def distance_ahead(node, scans: int = 5) -> float:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    got: list[LaserScan] = []
    sub = node.create_subscription(LaserScan, "/scan", got.append, qos_profile_sensor_data)
    end = time.monotonic() + 5.0
    while len(got) < scans and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    vals = []
    for m in got:
        i = round((0.0 - m.angle_min) / m.angle_increment)
        vals += [r for r in m.ranges[i - 2:i + 3] if math.isfinite(r) and r > 0.15]
    if not vals:
        raise SystemExit("no lidar return straight ahead; face a wall 1-2 m away")
    return float(np.median(vals))


class Frames:
    """Undistorted frames from /camera/image_raw that arrive after the latest move.

    The camera's stream runs 1.5-3 s behind and delivers only 1-2 frames a second
    (measured 2026-11-12), so settled() waits, then wants two frames in a row whose floor
    line agrees."""

    def __init__(self, node, k: np.ndarray, dist: np.ndarray) -> None:
        import cv2
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.node = node
        self.maps = cv2.initUndistortRectifyMap(k, dist, None, k, (640, 480), cv2.CV_16SC2)
        self.since = time.monotonic()
        self.got: list[np.ndarray] = []
        node.create_subscription(Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg) -> None:
        import cv2
        from look import to_bgr
        if msg.width != 640 or msg.height != 480 or time.monotonic() - self.since < STREAM_LAG_S:
            return
        self.got.append(cv2.remap(to_bgr(msg).copy(), *self.maps, cv2.INTER_LINEAR))

    def settled(self, timeout: float = 15.0) -> tuple[np.ndarray, int | None] | None:
        """(frame, floor row) once two consecutive frames agree within 2 px, else None."""
        import rclpy
        self.since, self.got = time.monotonic(), []
        end = self.since + timeout
        rows: list[int | None] = []
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            while len(rows) < len(self.got):
                rows.append(junction_row(self.got[len(rows)]))
            if len(rows) >= 2:
                a, b = rows[-2], rows[-1]
                if (a is None and b is None) or (a is not None and b is not None and abs(a - b) <= 2):
                    return self.got[-1], b
        return None


def sweep(cmds: list[float], pan: float) -> None:
    import cv2
    import rclpy
    from ptz import aim
    from wall_target import load_intrinsics
    k, dist_coef, _ = load_intrinsics()
    rclpy.init()
    node = rclpy.create_node("tilt_check")
    frames = Frames(node, k, dist_coef)
    out = SAVE_DIR / time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    out.mkdir(parents=True, exist_ok=True)
    wall = distance_ahead(node) - CAM_AHEAD_OF_LIDAR_M
    print(f"wall {wall:.3f} m from the camera; frames in {out}")
    used_cmds, downs = [], []
    try:
        for cmd in cmds:
            if aim(node, pan, cmd) is None:
                print(f"  {cmd:+6.1f}: no /ptz_angle confirmation, skipped")
                continue
            got = frames.settled()
            if got is None:
                print(f"  {cmd:+6.1f}: camera image didn't settle, skipped")
                continue
            frame, row = got
            cv2.imwrite(str(out / f"tilt_{cmd:+.0f}.jpg"), frame)
            if row is None:
                print(f"  {cmd:+6.1f}: floor line not in view")
                continue
            down = pitch_down_deg(row, k[1, 2], k[1, 1], CAM_HEIGHT_M, wall)
            used_cmds.append(cmd)
            downs.append(down)
            print(f"  {cmd:+6.1f}: floor line at row {row:3d}, camera {down:+5.1f} deg below level")
        aim(node, pan, 0.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    fit = fit_tilt(used_cmds, downs)
    print(f"\nlevel at command {fit.level_cmd:+.1f}, {fit.scale:.3f} real deg per commanded deg, "
          f"fit rms {fit.rms_deg:.2f} deg ({len(used_cmds)} points)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cmds", type=float, nargs="+", default=[float(c) for c in range(-15, 7, 3)])
    ap.add_argument("--pan", type=float, default=0.0)
    args = ap.parse_args()
    sweep(args.cmds, args.pan)


if __name__ == "__main__":
    main()
