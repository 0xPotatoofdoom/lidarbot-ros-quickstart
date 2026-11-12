#!/usr/bin/env python3
"""Turn a steady-load discharge log into a voltage -> percent curve and a runtime.

Runs on the Mac (plain Python):
    .venv/bin/python tools/battery_report.py datasets/battery/battery_*.csv [--since UNIX] [--until UNIX]

Assumes the load was steady for the whole window (robot powered, sensors on, not
driving), so charge used is proportional to time. Uses resting-voltage rows only.
Prints the runtime from the first sample to the 10.2 V alarm, the voltage at each
10% of elapsed time, and an ocv_curve block for config/battery.yaml.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ALARM_V = 10.2
CELLS = 3
CAPACITY_AH = 2.2        # pack label: 12V 2200mAh 3 cell
NOMINAL_V = 11.1
USABLE = (0.90, 0.95)    # share of rated capacity typically used by 3.4 V/cell (the 10.2 V alarm)
SMOOTH = 25   # rows (~2 minutes at 5 s per row) of median smoothing


def load(paths: list[Path], since: float, until: float) -> list[tuple[float, float]]:
    """Resting rows from one or more logs (files roll over at UTC midnight), sorted by time."""
    rows = []
    for path in paths:
        with path.open() as f:
            for r in csv.DictReader(f):
                t = float(r["unix_time"])
                if since <= t <= until and r["moving"] == "0" and r["rest_voltage"]:
                    rows.append((t, float(r["rest_voltage"])))
    return sorted(rows)


def smooth(rows: list[tuple[float, float]]) -> list[tuple[float, float]]:
    half = SMOOTH // 2
    out = []
    for i in range(len(rows)):
        window = sorted(v for _, v in rows[max(0, i - half):i + half + 1])
        out.append((rows[i][0], window[len(window) // 2]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", type=Path, nargs="+")
    ap.add_argument("--since", type=float, default=0.0)
    ap.add_argument("--until", type=float, default=float("inf"))
    args = ap.parse_args()
    rows = smooth(load(args.logs, args.since, args.until))
    if len(rows) < 2 * SMOOTH:
        sys.exit(f"only {len(rows)} resting samples in the window; need a longer run")
    t0, v_start = rows[0]
    end = next(((t, v) for t, v in rows if v <= ALARM_V), None)
    if end is None:
        hours = (rows[-1][0] - t0) / 3600
        sys.exit(f"didn't reach {ALARM_V} V yet: {v_start:.2f} -> {rows[-1][1]:.2f} V over {hours:.2f} h")
    t_end = end[0]
    run = [(t, v) for t, v in rows if t <= t_end]
    hours = (t_end - t0) / 3600
    print(f"start {v_start:.2f} V ({v_start / CELLS:.3f} V/cell), reached {ALARM_V} V after {hours:.2f} h "
          f"({len(run)} samples)")
    lo, hi = (CAPACITY_AH * u / hours for u in USABLE)
    print(f"average draw about {lo * 1000:.0f}-{hi * 1000:.0f} mA ({lo * NOMINAL_V:.1f}-{hi * NOMINAL_V:.1f} W), "
          f"assuming {USABLE[0]:.0%}-{USABLE[1]:.0%} of the {CAPACITY_AH} Ah rating is used by {ALARM_V} V")
    if v_start < 12.45:
        print("note: started below a full charge (12.6 V); the top of the curve is missing")
    print("\nelapsed  remaining  voltage  V/cell")
    curve = []
    for pct_left in range(100, -1, -10):
        target = t0 + (t_end - t0) * (1 - pct_left / 100)
        v = min(run, key=lambda r: abs(r[0] - target))[1]
        curve.append((v / CELLS, pct_left))
        print(f"{100 - pct_left:6d}%  {pct_left:8d}%  {v:7.2f}  {v / CELLS:6.3f}")
    print("\n# for config/battery.yaml (steady-load fit):")
    print("  ocv_curve:")
    for vpc, pct in sorted(set(curve)):
        print(f"    - [{vpc:.3f}, {pct}]")


if __name__ == "__main__":
    main()
