#!/usr/bin/env python3
"""Check a pan/tilt servo for sticky spots: 1 degree steps across its range, both ways.

    ./ros python3 /work/tools/servo_sweep.py [--axis pan|tilt] [--lo -90] [--hi 90] [--hold -45]

Each step's image shift (phase correlation, tools/ptz_range.py) should be about the same
(8-9 px per degree at 320x240 on ours). A healthy cheap servo has a 1-2 degree deadband:
an occasional step that doesn't move, then a double step. A sticky zone is several steps in
a row that barely move followed by a jump, at the same place in both directions: a worn
position sensor. Stops if the servo stays stuck for 4 steps. Needs a textured view.
"""
from __future__ import annotations

import argparse
import statistics

import ptz_range as pr
import rclpy
from rclpy.node import Node

STUCK_PX = 2.5           # less than this counts as "didn't move" (with a good match)
STUCK_ABORT = 4          # consecutive stuck steps before giving up


def sweep(p: pr.Ptz, ip: str, axis: str, cmds: list[int], hold: float) -> list[tuple[int, float, float]]:
    def aim(c: float) -> bool:
        return p.aim(c, hold) if axis == "pan" else p.aim(hold, c)
    aim(float(cmds[0]))
    prev = pr.grab(ip)
    rows, stuck = [], 0
    for c in cmds[1:]:
        aim(float(c))
        cur = pr.grab(ip)
        if cur is None or prev is None:
            print(f"  camera stopped answering at {c:+d}")
            break
        px, r = pr.shift(prev, cur)
        rows.append((c, px, r))
        prev = cur
        stuck = stuck + 1 if (px < STUCK_PX and r > pr.MIN_RESPONSE) else 0
        if stuck >= STUCK_ABORT:
            print(f"  stuck for {STUCK_ABORT} steps at {c:+d}: stopping")
            break
    return rows


def sticky_zones(rows: list[tuple[int, float, float]]) -> list[str]:
    """Runs of 3+ steps that moved under 30% of typical, ended by a step over 2x typical."""
    typical = statistics.median(px for _, px, _ in rows)
    zones, run = [], []
    for c, px, _ in rows:
        if px < 0.3 * typical:
            run.append(c)
            continue
        if len(run) >= 3 and px > 2.0 * typical:
            zones.append(f"{run[0]:+d} to {run[-1]:+d}, then jumped {px / typical:.1f} steps at {c:+d}")
        run = []
    return zones


def report(label: str, rows: list[tuple[int, float, float]]) -> None:
    moved = [px for _, px, _ in rows]
    zero = sum(1 for px in moved if px < STUCK_PX)
    print(f"{label}: {len(rows)} steps, median {statistics.median(moved):.1f} px/step, "
          f"{zero} steps that didn't move")
    zones = sticky_zones(rows)
    print("  sticky zones: " + ("; ".join(zones) if zones else "none"))
    print("  " + " ".join(f"{c:+d}:{px:.0f}" + ("" if r >= 0.4 else "?") for c, px, r in rows))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", choices=["pan", "tilt"], default="pan")
    ap.add_argument("--lo", type=int, default=-90)
    ap.add_argument("--hi", type=int, default=90)
    ap.add_argument("--hold", type=float, default=None, help="other axis position (pan 0 / tilt -40)")
    args = ap.parse_args()
    hold = args.hold if args.hold is not None else (-40.0 if args.axis == "pan" else 0.0)
    rclpy.init()
    node = Node("servo_sweep")
    p = pr.Ptz(node)
    ip = pr.camera_ip()
    try:
        for label, cmds in ((f"{args.axis} 0 -> {args.lo}", list(range(0, args.lo - 1, -1))),
                            (f"{args.axis} {args.lo} -> 0", list(range(args.lo, 1))),
                            (f"{args.axis} 0 -> {args.hi}", list(range(args.hi + 1))),
                            (f"{args.axis} {args.hi} -> 0", list(range(args.hi, -1, -1)))):
            report(label, sweep(p, ip, args.axis, cmds, hold))
    finally:
        p.aim(0.0, -40.0)
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
