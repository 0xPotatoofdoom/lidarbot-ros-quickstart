#!/usr/bin/env python3
"""Measure the robot's camera position against an ArUco target taped to the wall.

The target (print/wall_target.svg) is a 2x2 board of DICT_4X4_50 markers, IDs 10-13,
taped flat to the wall where the tape line meets it, centered on the line and at about
camera lens height. From one camera frame, solvePnP gives the camera's position and
heading relative to the board. With this narrow lens (about 35 degrees) and a flat
target, a small sideways shift and a small turn look almost the same, so the PnP yaw and
lateral are only good to about 1 degree / 10 mm on their own. What the camera does measure
well is the distance and the bearing to the board center. Heading therefore comes from the
lidar instead: fit_wall() fits a line to the wall ahead. Combined:
    heading_deg  robot heading relative to the wall normal, + = turned right (lidar)
    lateral_mm   camera's sideways offset from the board center, + = right when facing
                 the wall (camera bearing + lidar heading + camera distance)
lateral_mm includes the pan servo's unknown offset from the robot axis, times the distance,
so compare it between stops at the same end of the tape (the offset is fixed for a run).

    ./ros python3 /work/tools/wall_target.py snap        # median of 3 fresh frames
    .venv-hf/bin/python tools/wall_target.py svg print/wall_target.svg   # regenerate the print

Intrinsics come from config/camera_intrinsics.yaml if present, otherwise Yahboom's factory
ost.yaml (focal length about 1000 px, so a narrow ~35 degree view).
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

DICT = cv2.aruco.DICT_4X4_50
BOARD_IDS = (10, 11, 12, 13)   # top-left, top-right, bottom-left, bottom-right
MARKER_MM = 80.0
GAP_MM = 20.0
MIN_MARKERS = 2
LOCAL_INTRINSICS = Path("/work/config/camera_intrinsics.yaml")
SAVE_DIR = Path("/root/datasets/wall")


@dataclass(frozen=True)
class WallPose:
    lateral_mm: float
    distance_mm: float
    height_mm: float
    yaw_deg: float
    pitch_deg: float
    markers: int
    reproj_px: float
    center_px: tuple[float, float]   # where the board center is in the image
    bearing_deg: float               # direction of the board center from the optical axis, + = right


@dataclass(frozen=True)
class WallFit:
    heading_deg: float   # robot heading relative to the wall normal, + = turned right
    distance_m: float    # perpendicular distance from the laser to the wall
    rms_mm: float
    points: int


WALL_ARC = math.radians(25)   # beams either side of straight ahead used for the wall fit


def fit_wall(ranges: np.ndarray, angle_min: float, inc: float) -> WallFit | None:
    """Line through the wall ahead. If its normal lies 2 degrees to the left, the robot is
    turned 2 degrees to the right of square-on (heading +2)."""
    from lidar_wall import fit_line
    f = fit_line(ranges, angle_min, inc, center=0.0, arc=WALL_ARC)
    return None if f is None else WallFit(f.normal_deg, f.distance_m, f.rms_mm, f.points)


def lateral_mm(cam: WallPose, wall: WallFit) -> float:
    """Camera's sideways offset from the board center, + = right when facing the wall."""
    return -cam.distance_mm * math.tan(math.radians(wall.heading_deg + cam.bearing_deg))


def marker_centers() -> dict[int, tuple[float, float]]:
    """Board frame: origin at the board center, x right, y down, z into the wall (mm)."""
    o = (MARKER_MM + GAP_MM) / 2
    return {BOARD_IDS[0]: (-o, -o), BOARD_IDS[1]: (o, -o), BOARD_IDS[2]: (-o, o), BOARD_IDS[3]: (o, o)}


def marker_corners(marker_id: int) -> np.ndarray:
    """4x3 corners in ArUco order (TL, TR, BR, BL), board frame (mm)."""
    cx, cy = marker_centers()[marker_id]
    h = MARKER_MM / 2
    return np.array([[cx - h, cy - h, 0], [cx + h, cy - h, 0], [cx + h, cy + h, 0], [cx - h, cy + h, 0]],
                    np.float64)


def dictionary():
    return cv2.aruco.getPredefinedDictionary(DICT)


def marker_bitmap(marker_id: int, side: int) -> np.ndarray:
    """Marker image (black border included), for OpenCV 4.5 (container) or >= 4.7."""
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary(), marker_id, side)
    return cv2.aruco.drawMarker(dictionary(), marker_id, side)


def detect(gray: np.ndarray) -> dict[int, np.ndarray]:
    """Board markers found in the image: id -> 4x2 corner pixels."""
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary(), params).detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary(), parameters=params)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners) if int(i) in BOARD_IDS}


def solve(img: np.ndarray, k: np.ndarray, dist: np.ndarray) -> WallPose | None:
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found = detect(gray)
    if len(found) < MIN_MARKERS:
        return None
    obj = np.concatenate([marker_corners(i) for i in sorted(found)])
    px = np.concatenate([found[i] for i in sorted(found)]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, px, k, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, px, k, dist, rvec, tvec)
    reproj = cv2.projectPoints(obj, rvec, tvec, k, dist)[0].reshape(-1, 2)
    r = cv2.Rodrigues(rvec)[0]
    cam = (-r.T @ tvec).ravel()           # camera position in the board frame
    axis = r.T @ np.array([0.0, 0.0, 1.0])  # optical axis in the board frame
    center = cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, k, dist)[0].ravel()
    t = tvec.ravel()
    return WallPose(
        lateral_mm=float(cam[0]), distance_mm=float(-cam[2]), height_mm=float(-cam[1]),
        yaw_deg=math.degrees(math.atan2(axis[0], axis[2])),
        pitch_deg=math.degrees(math.atan2(-axis[1], math.hypot(axis[0], axis[2]))),
        markers=len(found), reproj_px=float(np.sqrt(np.mean(np.sum((reproj - px) ** 2, axis=1)))),
        center_px=(float(center[0]), float(center[1])),
        bearing_deg=math.degrees(math.atan2(t[0], t[2])),
    )


def median_pose(poses: list[WallPose]) -> WallPose:
    def med(name: str) -> float:
        return statistics.median(getattr(p, name) for p in poses)
    return WallPose(med("lateral_mm"), med("distance_mm"), med("height_mm"), med("yaw_deg"), med("pitch_deg"),
                    min(p.markers for p in poses), max(p.reproj_px for p in poses),
                    (statistics.median(p.center_px[0] for p in poses),
                     statistics.median(p.center_px[1] for p in poses)),
                    med("bearing_deg"))


def load_intrinsics() -> tuple[np.ndarray, np.ndarray, str]:
    import yaml
    path = LOCAL_INTRINSICS
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("microros_v2_bringup")) / "config" / "ost.yaml"
    cfg = yaml.safe_load(path.read_text())
    k = np.array(cfg["camera_matrix"]["data"], np.float64).reshape(3, 3)
    d = np.array(cfg["distortion_coefficients"]["data"], np.float64)
    if (cfg["image_width"], cfg["image_height"]) != (640, 480) or not 300 < k[0, 0] < 3000:
        raise SystemExit(f"{path}: expected 640x480 intrinsics with a sane focal length")
    return k, d, str(path)


# ---------------------------------------------------------------- robot side

class WallCamera:
    """Collects fresh frames from /camera/image_raw on an existing node."""

    LATENCY_S = 1.0   # the stream lags; frames arriving sooner may predate the stop

    def __init__(self, node) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.k, self.dist, self.source = load_intrinsics()
        self.frames: list[tuple[float, np.ndarray]] = []
        node.create_subscription(Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data)
        self.node = node

    def _on_image(self, msg) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = data[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        img = img if msg.encoding == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.frames = self.frames[-4:] + [(time.monotonic(), img)]

    def measure(self, n: int = 3, timeout: float = 12.0, save: str | None = None) -> WallPose | None:
        """Median pose over n frames that arrive at least LATENCY_S after this call."""
        import rclpy
        after = time.monotonic() + self.LATENCY_S
        seen: set[float] = set()
        poses: list[WallPose] = []
        last = None
        end = time.monotonic() + timeout
        while len(poses) < n and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for t, img in self.frames:
                if t >= after and t not in seen:
                    seen.add(t)
                    last = img
                    pose = solve(img, self.k, self.dist)
                    if pose is not None:
                        poses.append(pose)
        if save and last is not None:
            SAVE_DIR.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(SAVE_DIR / save), last)
        return median_pose(poses) if poses else None


def stop_reading(node, cam: WallCamera, save: str | None = None) -> dict:
    """Camera target pose + lidar wall fit at one stop (robot still). node is a move.Mover."""
    from calibrate import grab_scan
    pose = cam.measure(save=save)
    r, scan = grab_scan(node)
    wall = fit_wall(r, scan.angle_min, scan.angle_increment)
    out: dict = {"camera": pose_dict(pose) if pose else None,
                 "lidar_wall": None if wall is None else {k: round(v, 3) if isinstance(v, float) else v
                                                          for k, v in asdict(wall).items()}}
    if pose and wall:
        out["heading_deg"] = round(wall.heading_deg, 3)
        out["lateral_mm"] = round(lateral_mm(pose, wall), 1)
    return out


def format_reading(rd: dict) -> str:
    if rd["camera"] is None:
        return "wall target: not found (need 2+ of markers 10-13 in view)"
    w = rd["lidar_wall"]
    if w is None:
        return "wall target seen, but no wall line in the lidar ahead"
    return (f"wall: heading {rd['heading_deg']:+.2f} deg, lateral {rd['lateral_mm']:+.1f} mm, camera distance "
            f"{rd['camera']['distance_mm']:.0f} mm (lidar {w['distance_m'] * 1000:.0f} mm, fit rms {w['rms_mm']:.1f} mm)")


def snap(save: bool) -> None:
    import rclpy
    from move import Mover
    rclpy.init()
    node = Mover()
    try:
        node.wait_ready()
        cam = WallCamera(node)
        name = time.strftime("wall_%Y%m%dT%H%M%SZ.jpg", time.gmtime()) if save else None
        rd = stop_reading(node, cam, save=name)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"intrinsics: {cam.source}")
    print(format_reading(rd))
    if rd["camera"]:
        print(format_pose(WallPose(**{**rd["camera"], "center_px": tuple(rd["camera"]["center_px"])})))
    if name:
        print(f"saved datasets/wall/{name}")


def format_pose(p: WallPose) -> str:
    return (f"distance {p.distance_mm:.1f} mm, bearing {p.bearing_deg:+.2f} deg, height {p.height_mm:+.1f} mm "
            f"(PnP only: lateral {p.lateral_mm:+.0f} mm, yaw {p.yaw_deg:+.1f} deg, pitch {p.pitch_deg:+.1f} deg)  "
            f"({p.markers} markers, reprojection {p.reproj_px:.2f} px, board center at "
            f"{p.center_px[0]:.0f},{p.center_px[1]:.0f} px)")


def pose_dict(p: WallPose) -> dict:
    return {k: (round(v, 2) if isinstance(v, float) else v) for k, v in asdict(p).items()}


# ---------------------------------------------------------------- print

def svg(path: Path) -> None:
    """Letter/A4-safe page: the 2x2 board, a center cross and a 100 mm scale bar."""
    page_w, page_h = 200.0, 260.0
    ox, oy = page_w / 2, 110.0     # board center on the page (mm)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w}mm" height="{page_h}mm" '
             f'viewBox="0 0 {page_w} {page_h}"><rect width="{page_w}" height="{page_h}" fill="white"/>',
             '<text x="10" y="10" font-size="4.5" font-family="sans-serif">lidarbot wall target '
             '(ArUco DICT_4X4_50, IDs 10-13, 80 mm markers, 20 mm gaps). Print at 100%.</text>']
    cells = 6  # 4x4 data bits + 1-cell black border on each side
    cell = MARKER_MM / cells
    for marker_id, (cx, cy) in marker_centers().items():
        bmp = marker_bitmap(marker_id, cells * 10)
        x0, y0 = ox + cx - MARKER_MM / 2, oy + cy - MARKER_MM / 2
        for row in range(cells):
            for col in range(cells):
                if bmp[row * 10 + 5, col * 10 + 5] < 128:
                    parts.append(f'<rect x="{x0 + col * cell:.3f}" y="{y0 + row * cell:.3f}" '
                                 f'width="{cell + 0.02:.3f}" height="{cell + 0.02:.3f}" fill="black"/>')
        parts.append(f'<text x="{x0}" y="{y0 + MARKER_MM + 4}" font-size="3" font-family="sans-serif" '
                     f'fill="#888">{marker_id}</text>')
    parts.append(f'<path d="M{ox - 4} {oy} H{ox + 4} M{ox} {oy - 4} V{oy + 4}" stroke="#d00" stroke-width="0.4"/>')
    by = oy + MARKER_MM + GAP_MM / 2 + 25
    parts.append(f'<rect x="{ox - 50}" y="{by}" width="100" height="3" fill="black"/>'
                 f'<text x="{ox - 50}" y="{by + 8}" font-size="4" font-family="sans-serif">'
                 f'this bar is 100 mm; check with a ruler</text>')
    notes = ["Tape it flat to a stiff card, then to the wall where the floor tape meets it:",
             "red cross on the tape line, at camera lens height. Keep it square to the wall."]
    for i, line in enumerate(notes):
        parts.append(f'<text x="10" y="{by + 20 + i * 6}" font-size="4" font-family="sans-serif">{line}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n")
    print(f"wrote {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("snap")
    s.add_argument("--no-save", action="store_true", help="don't save the frame to datasets/wall/")
    g = sub.add_parser("svg")
    g.add_argument("out", type=Path)
    args = ap.parse_args()
    if args.cmd == "svg":
        svg(args.out)
    else:
        snap(save=not args.no_save)


if __name__ == "__main__":
    sys.path.insert(0, "/work/tools")
    main()
