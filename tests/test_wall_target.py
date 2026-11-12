"""tools/wall_target.py on synthetic camera frames and scans (no robot needed).

Works with OpenCV 4.5 (container) and >= 4.7 (.venv-hf):
    ./ros python3 -m pytest -q /work/tests/test_wall_target.py
    .venv-hf/bin/python -m pytest -q tests/test_wall_target.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from wall_target import (BOARD_IDS, MARKER_MM, WallFit, fit_wall, lateral_mm,  # noqa: E402
                         marker_bitmap, marker_centers, solve)

W, H = 640, 480
K = np.array([[1000.0, 0, 320.0], [0, 1000.0, 240.0], [0, 0, 1]])
NO_DIST = np.zeros(5)
PX_PER_MM = 4.0
CANVAS_MM = 260.0
BEAMS = 667
INC = 2 * math.pi / BEAMS


def board_canvas() -> np.ndarray:
    """The board drawn flat, PX_PER_MM pixels per mm, centered on the canvas."""
    side = int(CANVAS_MM * PX_PER_MM)
    canvas = np.full((side, side), 255, np.uint8)
    m = int(MARKER_MM * PX_PER_MM)
    for marker_id, (cx, cy) in marker_centers().items():
        x0 = int((cx - MARKER_MM / 2 + CANVAS_MM / 2) * PX_PER_MM)
        y0 = int((cy - MARKER_MM / 2 + CANVAS_MM / 2) * PX_PER_MM)
        canvas[y0:y0 + m, x0:x0 + m] = marker_bitmap(marker_id, m)
    return canvas


def render(lateral: float, distance: float, height: float, yaw_deg: float) -> np.ndarray:
    """Camera at (lateral, -height, -distance) in the board frame, turned yaw_deg to the right."""
    y = math.radians(yaw_deg)
    r_cw = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    r = r_cw.T
    t = -r @ np.array([lateral, -height, -distance])
    s = np.array([[1 / PX_PER_MM, 0, -CANVAS_MM / 2], [0, 1 / PX_PER_MM, -CANVAS_MM / 2], [0, 0, 1]])
    h = K @ np.column_stack([r[:, 0], r[:, 1], t]) @ s
    img = cv2.warpPerspective(board_canvas(), h, (W, H), flags=cv2.INTER_AREA, borderValue=170)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def wall_scan(distance: float, heading_deg: float, noise: float = 0.0, seed: int = 0) -> np.ndarray:
    """Ranges to a straight wall `distance` m away, robot turned heading_deg to the right."""
    a = -math.pi + INC * np.arange(BEAMS)
    normal = math.radians(heading_deg)        # wall normal appears this far to the left
    cosang = np.cos(a - normal)
    r = np.where(cosang > 0.05, distance / np.maximum(cosang, 1e-9), np.nan)
    r[np.abs(a) > math.radians(80)] = np.nan  # only the wall ahead
    return r + np.random.default_rng(seed).normal(0, noise, BEAMS) if noise else r


@pytest.mark.parametrize("lateral, distance, height, yaw", [
    (0.0, 600.0, 0.0, 0.0),        # far stop, square on
    (25.0, 650.0, -20.0, 2.0),     # drifted right, turned right
    (-40.0, 1600.0, 30.0, -1.5),   # start stop, 1.6 m away
])
def test_camera_distance_and_bearing(lateral: float, distance: float, height: float, yaw: float) -> None:
    pose = solve(render(lateral, distance, height, yaw), K, NO_DIST)
    assert pose is not None and pose.markers == 4 and pose.reproj_px < 0.5
    assert pose.distance_mm == pytest.approx(distance, abs=0.004 * distance)
    expected_bearing = math.degrees(math.atan2(-lateral, distance)) - yaw
    assert pose.bearing_deg == pytest.approx(expected_bearing, abs=0.05)


@pytest.mark.parametrize("lateral, distance, yaw", [(25.0, 650.0, 2.0), (-40.0, 1600.0, -1.5), (5.0, 600.0, 0.5)])
def test_lateral_with_lidar_heading(lateral: float, distance: float, yaw: float) -> None:
    pose = solve(render(lateral, distance, 0.0, yaw), K, NO_DIST)
    assert pose is not None
    wall = WallFit(heading_deg=yaw, distance_m=distance / 1000, rms_mm=0.0, points=100)
    assert lateral_mm(pose, wall) == pytest.approx(lateral, abs=1.0)


def test_flat_target_alone_confuses_shift_and_turn() -> None:
    """Why heading comes from the lidar: PnP can't separate 5 mm from 0.5 deg at 0.6 m."""
    a = solve(render(0.0, 600.0, 0.0, 0.0), K, NO_DIST)
    b = solve(render(5.0, 600.0, 0.0, 0.5), K, NO_DIST)
    assert a is not None and b is not None
    assert b.bearing_deg - a.bearing_deg == pytest.approx(-0.5 - math.degrees(5 / 600), abs=0.05)


@pytest.mark.parametrize("distance, heading", [(0.6, 0.0), (0.65, 1.5), (1.6, -2.0)])
def test_fit_wall_exact(distance: float, heading: float) -> None:
    fit = fit_wall(wall_scan(distance, heading), -math.pi, INC)
    assert fit is not None
    assert fit.heading_deg == pytest.approx(heading, abs=1e-6)
    assert fit.distance_m == pytest.approx(distance, abs=1e-6)


def test_fit_wall_noisy_and_rejects_clutter() -> None:
    r = wall_scan(0.6, 1.0, noise=0.005, seed=1)
    a = -math.pi + INC * np.arange(BEAMS)
    r[np.abs(a - math.radians(10)) < math.radians(2)] = 0.35   # a chair leg in front of the wall
    fit = fit_wall(r, -math.pi, INC)
    assert fit is not None
    assert fit.heading_deg == pytest.approx(1.0, abs=0.5)
    assert fit.distance_m == pytest.approx(0.6, abs=0.003)


def test_needs_enough_markers_and_wall() -> None:
    img = render(0.0, 600.0, 0.0, 0.0)
    covered = img.copy()
    covered[:, 320:] = 170                     # only markers 10 and 12 remain
    pose = solve(covered, K, NO_DIST)
    assert pose is not None and pose.markers == 2
    assert solve(np.full((H, W, 3), 170, np.uint8), K, NO_DIST) is None
    assert fit_wall(np.full(BEAMS, np.nan), -math.pi, INC) is None
    assert set(marker_centers()) == set(BOARD_IDS)
