"""Analysis side of tools/battery_drive_test.py (no robot needed). Runs in the container:
    ./ros python3 -m pytest -q /work/tests
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, "/work/tools")
from battery_drive_test import Sample, read_log, rest_points, summarize  # noqa: E402
from battery_monitor import Curve  # noqa: E402

LINEAR = Curve(((3.4, 0.0), (4.2, 100.0)))  # 0.008 V/cell per percent


def segment(t0: float, seconds: float, v: float, phase: str, dist0: float, speed: float) -> list[Sample]:
    return [Sample(t0 + i, v, phase, 0, dist0 + (speed * i if phase == "drive" else 0.0))
            for i in range(int(seconds))]


def synthetic() -> list[Sample]:
    """Rest at 12.0 V, drive 1 h at 0.1 m/s 0.3 V below the resting voltage, rest at 11.52 V (20% used)."""
    out = segment(0, 60, 12.0, "rest", 0.0, 0.0)
    out += segment(60, 3600, 11.22, "drive", 0.0, 0.1)
    out += segment(3660, 60, 11.52, "rest", 360.0, 0.0)
    return out


def test_rest_points_use_settled_rest_and_preceding_drive() -> None:
    pts = rest_points(synthetic())
    assert len(pts) == 2
    assert pts[0].rest_v == 12.0 and pts[0].drive_v is None
    assert pts[1].rest_v == 11.52 and pts[1].drive_v == pytest.approx(11.22)
    assert pts[1].distance == pytest.approx(360.0)


def test_summary_runtime_current_and_sag() -> None:
    s = summarize(synthetic(), LINEAR, cells=3, capacity_ah=2.2)
    assert s.pct_start == pytest.approx(100 * (4.0 - 3.4) / 0.8)
    assert s.pct_start - s.pct_end == pytest.approx(20.0)
    assert s.hours == pytest.approx(3660 / 3600)  # end of first rest to end of last
    assert s.avg_a == pytest.approx(2.2 * 0.2 / s.hours)
    assert s.runtime_h == pytest.approx(s.hours / 0.2)
    assert s.km_per_charge == pytest.approx(0.360 / 0.2)
    assert s.sag_v == pytest.approx(0.30)  # 11.22 loaded vs 11.52 at rest afterwards
    assert s.drive_a is not None and s.drive_a > s.avg_a
    assert s.resistance_ohm == pytest.approx(s.sag_v / (s.drive_a - 0.29))


def test_summary_needs_two_rests() -> None:
    with pytest.raises(ValueError):
        summarize(segment(0, 60, 12.0, "rest", 0, 0), LINEAR, 3, 2.2)


def test_read_log_round_trip(tmp_path: Path) -> None:
    p = tmp_path / "drive.csv"
    p.write_text("unix_time,voltage,phase,cycle,distance_m\n1.00,12.000,rest,0,0.000\n2.00,11.700,drive,1,0.150\n")
    assert read_log(p) == [Sample(1.0, 12.0, "rest", 0, 0.0), Sample(2.0, 11.7, "drive", 1, 0.15)]
