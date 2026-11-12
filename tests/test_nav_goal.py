"""tools/nav_goal.py helpers (no robot needed)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from nav_goal import arrival_error, quat_to_yaw, yaw_to_quat  # noqa: E402


@pytest.mark.parametrize("yaw", [0.0, 1.0, -2.5, math.pi - 0.01])
def test_quat_round_trip(yaw: float) -> None:
    assert quat_to_yaw(*yaw_to_quat(yaw)) == pytest.approx(yaw)


def test_arrival_error() -> None:
    err = arrival_error((1.0, 2.0, math.radians(90)), (1.03, 1.96, math.radians(95)))
    assert err.dist_m == pytest.approx(0.05)
    assert err.dyaw_deg == pytest.approx(5.0)


def test_arrival_error_wraps_heading() -> None:
    err = arrival_error((0.0, 0.0, math.radians(179)), (0.0, 0.0, math.radians(-179)))
    assert err.dyaw_deg == pytest.approx(2.0)
