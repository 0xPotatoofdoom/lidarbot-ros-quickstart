"""Unit tests for the calibration maths. Run in the container:
    ./ros python3 -m pytest -q /work/tests
"""
from __future__ import annotations

import math
import sys

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan

sys.path.insert(0, "/work/tools")
from calibrate import range_ahead, scan_rotation  # noqa: E402
from odom_calibrator import Scales, YawScaler, quat_of, rpy_of  # noqa: E402

N = 667
INC = 2 * math.pi / N


def room_scan(rotation: float = 0.0) -> np.ndarray:
    """Ranges from the center of an irregular room, after the robot turns by rotation."""
    angles = np.arange(N) * INC + rotation
    # a lumpy, non-symmetric outline so alignment has a unique answer
    return 2.0 + 0.6 * np.sin(angles) + 0.3 * np.cos(3 * angles) + 0.15 * np.sin(5 * angles + 1)


def scan_msg() -> LaserScan:
    return LaserScan(angle_min=0.0, angle_increment=INC, range_min=0.05, range_max=12.0)


@pytest.mark.parametrize("deg", [0.0, 12.0, -30.0, 47.3, 170.0, -170.0])
def test_scan_rotation_recovers_turn(deg: float) -> None:
    rot, resid = scan_rotation(room_scan(), room_scan(math.radians(deg)), INC)
    assert math.degrees(rot) == pytest.approx(deg, abs=0.2)
    assert resid < 0.02


def test_scan_rotation_tolerates_missing_beams() -> None:
    a, b = room_scan(), room_scan(math.radians(20))
    a[::7] = np.nan
    b[100:160] = np.nan  # a mirror
    rot, _ = scan_rotation(a, b, INC)
    assert math.degrees(rot) == pytest.approx(20.0, abs=0.3)


def test_range_ahead_uses_median_of_forward_beams() -> None:
    r = np.full(N, 5.0)
    r[0:3] = 1.50
    r[-3:] = 1.50
    r[3] = 9.0  # one outlier just inside the arc
    assert range_ahead(r, scan_msg()) == pytest.approx(1.50)


def test_range_ahead_nan_when_nothing_ahead() -> None:
    r = np.full(N, np.nan)
    assert math.isnan(range_ahead(r, scan_msg()))


def test_scales_reject_bad_values() -> None:
    with pytest.raises(ValueError):
        Scales(linear=3.0).validated()
    assert Scales(angular_left=1.08).validated().angular_left == 1.08


def test_angular_scale_depends_on_direction() -> None:
    s = Scales(angular_left=1.1, angular_right=1.2)
    assert s.angular(1.0) == pytest.approx(1.1)
    assert s.angular(-1.0) == pytest.approx(-1.2)


def test_yaw_scaler_scales_full_turns_continuously() -> None:
    scale = 1.08
    yaw = YawScaler(scale)
    start = 0.3
    total = 0.0
    out = yaw(start)
    for _ in range(720):  # two full turns in 1-degree steps
        total += math.radians(1)
        out = yaw(math.atan2(math.sin(start + total), math.cos(start + total)))
    expected = start + total * scale
    assert math.cos(out - expected) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("rpy", [(0.0, 0.0, 1.0), (0.05, -0.03, -2.5), (0.0, 0.0, math.pi - 1e-6)])
def test_quaternion_round_trip(rpy: tuple[float, float, float]) -> None:
    assert rpy_of(quat_of(*rpy)) == pytest.approx(rpy, abs=1e-9)


def test_line_follow_steers_back_toward_line() -> None:
    from move import MAX_STEER, line_follow_wz
    # left of the line, driving forward: turn right (clockwise, negative)
    assert line_follow_wz(0.0, 0.02, 1.0) < 0
    # left of the line, reversing: the rear must swing right, so turn left (positive)
    assert line_follow_wz(0.0, 0.02, -1.0) > 0
    # heading error is corrected the same way in both directions
    assert line_follow_wz(0.1, 0.0, 1.0) == line_follow_wz(0.1, 0.0, -1.0) > 0
    assert abs(line_follow_wz(5.0, 1.0, 1.0)) == MAX_STEER


def test_battery_curve_interpolates_and_clamps() -> None:
    from pathlib import Path

    from battery_monitor import load_config
    cells, curve, alarm, capacity = load_config(Path("/work/config/battery.yaml"))
    assert cells == 3 and alarm == 10.2 and capacity == 2.2
    assert curve.percent(4.25) == 100 and curve.percent(3.2) == 0
    assert curve.percent(3.876) == pytest.approx(50)
    assert curve.percent(3.855) == pytest.approx(45)
    assert curve.percent(alarm / cells) == pytest.approx(0, abs=0.5)

