"""Geometry tests for lab/overhead_tracker.py on synthetic overhead images.

Needs OpenCV >= 4.7 (ArucoDetector), so run with the Hugging Face environment:
    .venv-hf/bin/python -m pytest -q tests/test_overhead_tracker.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lab"))
from overhead_tracker import DICT, Tracker, solve_floor  # noqa: E402

W, H = 1920, 1080
F = 1500.0                    # px
CAM_HEIGHT = 2.0              # m
CAM_XY = np.array([0.9, 0.0])  # floor point under the lens
FLOOR = {"1": [0.30, 0.25], "2": [1.52, 0.25], "3": [0.30, -0.25], "4": [1.52, -0.25]}
FLOOR_SIZE = 0.070
ROBOT = {"id": 0, "size_m": 0.060, "height_m": 0.17, "yaw_offset_deg": 0.0}


def camera(tilt_deg: float, roll_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """rvec, tvec, K for a camera looking down, image right = +x, image down = -y."""
    r_cw = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], np.float64).T  # columns: camera axes in world
    tilt = cv2.Rodrigues(np.array([math.radians(tilt_deg), 0, 0]))[0]
    roll = cv2.Rodrigues(np.array([0, 0, math.radians(roll_deg)]))[0]
    r_cw = r_cw @ tilt @ roll
    r_wc = r_cw.T
    c = np.array([CAM_XY[0], CAM_XY[1], CAM_HEIGHT])
    k = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
    return cv2.Rodrigues(r_wc)[0], -r_wc @ c, k


def square(center, size: float, yaw: float, z: float) -> np.ndarray:
    """Marker corners in world coordinates, ArUco order (TL, TR, BR, BL), top edge toward yaw."""
    local = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * size / 2
    c, s = math.cos(yaw), math.sin(yaw)
    xy = local @ np.array([[c, s], [-s, c]]) + np.asarray(center)
    return np.column_stack([xy, np.full(4, z)])


def draw(img: np.ndarray, marker_id: int, corners_world: np.ndarray, cam) -> None:
    rvec, tvec, k = cam
    px = cv2.projectPoints(corners_world, rvec, tvec, k, None)[0].reshape(4, 2)
    side, pad = 120, 30
    bmp = np.full((side + 2 * pad, side + 2 * pad), 255, np.uint8)
    bmp[pad:pad + side, pad:pad + side] = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(DICT), marker_id, side)
    # bitmap corners of the black square -> projected marker corners
    src = np.array([[pad, pad], [pad + side, pad], [pad + side, pad + side], [pad, pad + side]], np.float32)
    m = cv2.getPerspectiveTransform(src, px.astype(np.float32))
    warped = cv2.warpPerspective(bmp, m, (W, H), borderValue=0)
    mask = cv2.warpPerspective(np.full_like(bmp, 255), m, (W, H)) > 0
    img[mask] = warped[mask]


def scene(robot_xy, robot_yaw: float, cam) -> np.ndarray:
    img = np.full((H, W), 150, np.uint8)  # wood-ish grey floor
    rng = np.random.default_rng(0)
    for mid, center in FLOOR.items():
        # laid any of the four ways round
        draw(img, int(mid), square(center, FLOOR_SIZE, rng.integers(4) * math.pi / 2, 0.0), cam)
    draw(img, ROBOT["id"], square(robot_xy, ROBOT["size_m"], robot_yaw, ROBOT["height_m"]), cam)
    return img


def tracker_for(img: np.ndarray, tmp_path: Path) -> Tracker:
    cfg = {"camera_url": "http://unused", "camera_height_m": CAM_HEIGHT, "robot_marker": ROBOT,
           "floor_markers": FLOOR, "floor_marker_size_m": FLOOR_SIZE, "log_dir": str(tmp_path)}
    cfg.update(solve_floor(cfg, [img]))
    return Tracker(cfg)


@pytest.mark.parametrize(("xy", "yaw_deg"), [((0.9, 0.0), 180.0), ((0.5, 0.1), 170.0), ((1.4, -0.15), 45.0)])
def test_straight_down_camera_recovers_pose(tmp_path: Path, xy, yaw_deg: float) -> None:
    cam = camera(0.0, 0.0)
    img = scene(xy, math.radians(yaw_deg), cam)
    pose = tracker_for(img, tmp_path).locate(0.0, img)
    assert pose is not None
    assert math.hypot(pose.x - xy[0], pose.y - xy[1]) < 0.003
    assert abs(math.degrees(math.remainder(pose.yaw - math.radians(yaw_deg), math.tau))) < 0.5


def test_calibration_fit_is_tight(tmp_path: Path) -> None:
    img = scene((0.9, 0.0), math.pi, camera(0.0, 7.0))
    cfg = {"floor_markers": FLOOR, "floor_marker_size_m": FLOOR_SIZE}
    assert solve_floor(cfg, [img])["calibration_rms_mm"] < 1.5


def test_tilted_rotated_camera_stays_within_a_centimeter(tmp_path: Path) -> None:
    # A 3 degree tilt moves the true nadir ~10 cm from the image center; the
    # height correction then errs by about 10 cm * 0.17 / 2.0 = 8.5 mm.
    cam = camera(3.0, 7.0)
    img = scene((1.2, 0.05), math.radians(175), cam)
    pose = tracker_for(img, tmp_path).locate(0.0, img)
    assert pose is not None
    assert math.hypot(pose.x - 1.2, pose.y - 0.05) < 0.010
    assert abs(math.degrees(math.remainder(pose.yaw - math.radians(175), math.tau))) < 1.0


def test_height_correction_matters(tmp_path: Path) -> None:
    """Without the parallax correction the pose is off by several centimeters."""
    cam = camera(0.0, 0.0)
    img = scene((0.6, 0.12), 0.0, cam)
    t = tracker_for(img, tmp_path)
    corrected = t.locate(0.0, img)
    t.parallax = 1.0
    t.last_px = None
    uncorrected = t.locate(0.0, img)
    assert corrected is not None and uncorrected is not None
    assert math.hypot(corrected.x - 0.6, corrected.y - 0.12) < 0.003
    assert math.hypot(uncorrected.x - 0.6, uncorrected.y - 0.12) > 0.02


def test_missing_marker_returns_none(tmp_path: Path) -> None:
    cam = camera(0.0, 0.0)
    img = scene((0.9, 0.0), 0.0, cam)
    t = tracker_for(img, tmp_path)
    blank = np.full_like(img, 150)
    assert t.locate(0.0, blank) is None
