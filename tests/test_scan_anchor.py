"""tools/scan_anchor.py on synthetic scans of a room shaped like the test run:
a door 1.7 m ahead, a right wall 0.54 m away with a 10 cm step sticking out beside the
start, and open space to the left. Runs in the container or .venv-hf.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from scan_anchor import Located, ScanAnchor, anchor_in_odom, compose, invert, transform  # noqa: E402

BEAMS = 667
INC = 2 * math.pi / BEAMS
# wall segments in the start frame (x ahead, y left), meters
ROOM = [
    ((1.70, -0.60), (1.70, 1.20)),     # door ahead
    ((-1.0, -0.57), (-0.20, -0.57)),   # right wall behind
    ((-0.20, -0.57), (-0.20, -0.44)),  # step out
    ((-0.20, -0.44), (0.62, -0.44)),   # the thing sticking out
    ((0.62, -0.44), (0.62, -0.54)),    # step back
    ((0.62, -0.54), (1.70, -0.54)),    # right wall ahead
    ((-1.0, -0.57), (-1.0, 1.20)),     # wall behind
    ((-1.0, 1.20), (0.40, 1.20)),      # far left wall, with a gap
    ((0.90, 1.20), (1.70, 1.20)),
]


def raycast(pose: tuple[float, float, float], noise: float = 0.0, seed: int = 0) -> SimpleNamespace:
    """Scan from pose (x, y, yaw) in the start frame."""
    px, py, yaw = pose
    rng = np.random.default_rng(seed)
    ranges = np.full(BEAMS, np.inf)
    for i in range(BEAMS):
        a = yaw - math.pi + i * INC
        dx, dy = math.cos(a), math.sin(a)
        for (x1, y1), (x2, y2) in ROOM:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - px) * ey - (y1 - py) * ex) / den
            u = ((x1 - px) * dy - (y1 - py) * dx) / den
            if t > 0 and 0 <= u <= 1:
                ranges[i] = min(ranges[i], t)
    if noise:
        ranges = ranges + rng.normal(0, noise, BEAMS)
    return SimpleNamespace(ranges=ranges.tolist(), angle_min=-math.pi, angle_increment=INC)


@pytest.mark.parametrize("true_pose", [
    (0.0, 0.0, 0.0),
    (0.5, 0.03, math.radians(2.0)),     # halfway out, drifted 3 cm left, turned 2 deg
    (0.98, -0.02, math.radians(-1.0)),  # at the far stop
])
def test_locate_despite_odometry_drift(true_pose: tuple[float, float, float]) -> None:
    anchor = ScanAnchor(raycast((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    # odometry believes the robot is still on the line, heading straight
    odom = (true_pose[0], 0.0, 0.0)
    loc = anchor.locate(raycast(true_pose, noise=0.005, seed=3), odom)
    assert loc is not None
    assert loc.x == pytest.approx(true_pose[0], abs=0.01)
    assert loc.y == pytest.approx(true_pose[1], abs=0.004)
    assert loc.yaw == pytest.approx(true_pose[2], abs=math.radians(0.3))


def test_anchor_in_a_rotated_odometry_frame() -> None:
    """The anchor's odometry pose can be anywhere; the guess is relative to it."""
    odom0 = (3.0, -2.0, math.radians(120))
    anchor = ScanAnchor(raycast((0.0, 0.0, 0.0)), odom0)
    c, s = math.cos(odom0[2]), math.sin(odom0[2])
    odom = (odom0[0] + 0.5 * c, odom0[1] + 0.5 * s, odom0[2])   # 0.5 m ahead in odom
    loc = anchor.locate(raycast((0.5, 0.02, 0.0)), odom)
    assert loc is not None and loc.y == pytest.approx(0.02, abs=0.004)


def test_rejects_scan_from_elsewhere_and_is_fast() -> None:
    anchor = ScanAnchor(raycast((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    empty = SimpleNamespace(ranges=[math.inf] * BEAMS, angle_min=-math.pi, angle_increment=INC)
    assert anchor.locate(empty, (0.0, 0.0, 0.0)) is None
    scan = raycast((0.5, 0.0, 0.0))
    t0 = time.perf_counter()
    for _ in range(5):
        anchor.locate(scan, (0.5, 0.0, 0.0))
    assert (time.perf_counter() - t0) / 5 < 0.05   # seconds per match


def test_transform_round_trip() -> None:
    pts = np.array([[1.0, 0.0], [0.0, 2.0]])
    out = transform(pts, 0.1, -0.2, math.pi / 2)
    assert out == pytest.approx(np.array([[0.1, 0.8], [-1.9, -0.2]]))


def test_anchor_in_odom_resets_the_line() -> None:
    """Odometry thinks the robot is on its line; the scan says 6 mm left, turned 0.8 deg.
    In the corrected line's frame the robot must sit 6 mm left, turned 0.8 deg."""
    odom = (2.0, 1.0, math.radians(30))
    true = Located(0.004, 0.006, math.radians(0.8), 0.01, 0.9)
    line = anchor_in_odom(odom, true)
    rel = compose(invert(line), odom)
    assert rel == pytest.approx((0.004, 0.006, math.radians(0.8)), abs=1e-9)


def test_guess_follows_the_latest_correction() -> None:
    """After many cycles odometry is 19 cm off. In the real room, matching from that raw
    guess failed every time (2026-11-11 run); starting from the last corrected line, the
    guess is only millimetres off."""
    anchor = ScanAnchor(raycast((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    drifted_odom = (0.0, 0.19, math.radians(3.5))     # robot is really back at the start
    scan = raycast((0.0, 0.0, 0.0), noise=0.005, seed=5)
    line = anchor_in_odom(drifted_odom, Located(0.0, 0.002, 0.0, 0.01, 0.9))  # last correction
    guess = anchor.odom_guess(drifted_odom, line)
    assert abs(guess[1]) < 0.003 and abs(anchor.odom_guess(drifted_odom)[1]) > 0.18
    loc = anchor.locate(scan, drifted_odom, line)
    assert loc is not None and loc.y == pytest.approx(0.0, abs=0.004)
