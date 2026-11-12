"""tools/tilt_check.py on synthetic frames (no robot needed).

    ./ros python3 -m pytest -q /work/tests/test_tilt_check.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from tilt_check import fit_tilt, junction_row, pitch_down_deg  # noqa: E402

WALL = (200, 200, 205)      # BGR, nearly grey: red minus blue about 0
FLOOR = (40, 90, 160)       # orange hardwood: red minus blue about 120
TAPE = (150, 60, 50)        # blue tape


def room(row: int, tape: bool = True) -> np.ndarray:
    """Wall above row, a white baseboard just above it, floor below, tape up the middle."""
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = WALL
    img[row - 25:row] = (235, 235, 235)
    img[row:] = FLOOR
    if tape:
        img[row:, 280:360] = TAPE
        img[:row, 310:330] = TAPE
    rng = np.random.default_rng(1)
    return np.clip(img + rng.normal(0, 6, img.shape), 0, 255).astype(np.uint8)


@pytest.mark.parametrize("row", [40, 240, 300, 440])
def test_junction_row(row: int) -> None:
    found = junction_row(room(row))
    assert found is not None and found == pytest.approx(row, abs=2)


def test_junction_row_none_without_floor() -> None:
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = WALL
    assert junction_row(img) is None


def test_pitch_down() -> None:
    # junction on the optical axis: the camera points straight at it, atan(0.12 / 1.58) down
    assert pitch_down_deg(240.0, 240.0, 1000.0, 0.12, 1.58) == pytest.approx(math.degrees(math.atan(0.12 / 1.58)))
    # junction 60 px below center: camera points about 3.4 deg higher
    assert pitch_down_deg(300.0, 240.0, 1000.0, 0.12, 1.58) == pytest.approx(
        math.degrees(math.atan(0.12 / 1.58)) - math.degrees(math.atan(0.06)))


def test_fit_tilt_finds_level_and_scale() -> None:
    """Measured 2026-11-12 before re-seating the horn: level near -61.5, 1:1 scale (the tool measured -60.9)."""
    cmds = [-59.0, -62.0, -65.0, -68.0, -71.0]
    downs = [-61.5 - c + e for c, e in zip(cmds, [0.2, -0.1, 0.0, 0.1, -0.2])]
    fit = fit_tilt(cmds, downs)
    assert fit.level_cmd == pytest.approx(-61.5, abs=0.3)
    assert fit.scale == pytest.approx(1.0, abs=0.05)
    assert fit.rms_deg < 0.3


def test_fit_tilt_needs_points() -> None:
    with pytest.raises(ValueError):
        fit_tilt([0.0], [1.0])
