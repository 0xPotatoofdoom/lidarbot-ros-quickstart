#!/usr/bin/env python3
"""Driving-load battery test: drive out and back until the pack is nearly empty.

Runs inside the container (./up running, gamepad receiver unplugged, robot on a
clear straight run of at least --dist + 0.5 m):
    ./ros python3 /work/tools/battery_drive_test.py run [--dist 1.0] [--rest-every 600]
    ./ros python3 /work/tools/battery_drive_test.py report /root/datasets/battery/drive_<time>.csv

`run` drives --dist meters out and back along the same straight line, over and over.
Each time it is back at the start it matches the lidar scan against one taken there at the
beginning (scan_anchor.py) and corrects the line, so the slow cycle-to-cycle drift that
odometry can't see doesn't build up (--no-scan-match to rely on odometry alone).
Every --rest-every seconds it stops for --rest-s seconds so the resting voltage can be
read against the fitted curve in config/battery.yaml (charge left), and compares it with
the voltage while driving just before (sag under load). It stops by itself once the
resting voltage reaches --stop-v (default 10.6 V, above the 10.2 V alarm), if the lidar
safety check stops a leg, or after --max-hours. Ctrl-C stops the robot.

Every /battery sample is logged to datasets/battery/drive_<UTC time>.csv:
    unix_time,voltage,phase,cycle,distance_m
`report` (also printed at the end of `run`) estimates driving runtime, average current,
distance per charge and the voltage sag under load.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, "/work/tools")
from battery_monitor import CONFIG, Curve, load_config  # noqa: E402

LOG_DIR = Path("/root/datasets/battery")
IDLE_A = 0.29          # measured idle draw (2026-11-11 discharge), lidar and camera on
REST_SETTLE_S = 30.0   # ignore the first part of a rest while the voltage recovers
SAG_WINDOW_S = 60.0    # driving samples this long before a rest give the loaded voltage
DRIVE_FLOOR_V = 10.3   # under load; below this, rest now to check the resting voltage
FLOOR_WINDOW_S = 10.0


@dataclass(frozen=True)
class Sample:
    t: float
    v: float
    phase: str      # "drive" or "rest"
    cycle: int
    distance: float


@dataclass(frozen=True)
class RestPoint:
    t: float
    rest_v: float
    drive_v: float | None   # median loaded voltage just before this rest
    distance: float


@dataclass(frozen=True)
class Summary:
    hours: float
    drive_hours: float
    distance_m: float
    pct_start: float
    pct_end: float
    avg_a: float | None
    drive_a: float | None
    runtime_h: float | None
    km_per_charge: float | None
    sag_v: float | None
    resistance_ohm: float | None


def rest_points(samples: list[Sample]) -> list[RestPoint]:
    """One point per rest: settled resting voltage and the loaded voltage before it."""
    points = []
    i = 0
    while i < len(samples):
        if samples[i].phase != "rest":
            i += 1
            continue
        j = i
        while j < len(samples) and samples[j].phase == "rest":
            j += 1
        seg = samples[i:j]
        settled = [s.v for s in seg if s.t - seg[0].t >= REST_SETTLE_S]
        if settled:
            before = [s.v for s in samples[:i] if s.phase == "drive" and seg[0].t - s.t <= SAG_WINDOW_S]
            points.append(RestPoint(seg[-1].t, statistics.median(settled),
                                    statistics.median(before) if before else None, seg[-1].distance))
        i = j
    return points


def drive_seconds(samples: list[Sample]) -> float:
    return sum(b.t - a.t for a, b in zip(samples, samples[1:]) if a.phase == "drive" and b.t - a.t < 5.0)


def summarize(samples: list[Sample], curve: Curve, cells: int, capacity_ah: float) -> Summary:
    pts = rest_points(samples)
    if len(pts) < 2:
        raise ValueError("need at least two completed rests (start and one later)")
    first, last = pts[0], pts[-1]
    hours = (last.t - first.t) / 3600
    window = [s for s in samples if first.t <= s.t <= last.t]
    drive_h = drive_seconds(window) / 3600
    p0, p1 = curve.percent(first.rest_v / cells), curve.percent(last.rest_v / cells)
    used = (p0 - p1) / 100
    dist = last.distance - first.distance
    avg_a = capacity_ah * used / hours if used > 0 and hours > 0 else None
    drive_a = None
    if avg_a is not None and drive_h > 0:
        drive_a = (avg_a * hours - IDLE_A * (hours - drive_h)) / drive_h
    sags = [p.rest_v - p.drive_v for p in pts if p.drive_v is not None]
    sag = statistics.median(sags) if sags else None
    resistance = sag / (drive_a - IDLE_A) if sag is not None and drive_a and drive_a > IDLE_A else None
    return Summary(
        hours=hours, drive_hours=drive_h, distance_m=dist, pct_start=p0, pct_end=p1,
        avg_a=avg_a, drive_a=drive_a,
        runtime_h=hours / used if used > 0 else None,
        km_per_charge=dist / used / 1000 if used > 0 else None,
        sag_v=sag, resistance_ohm=resistance,
    )


def read_log(path: Path) -> list[Sample]:
    with path.open() as f:
        return [Sample(float(r["unix_time"]), float(r["voltage"]), r["phase"], int(r["cycle"]),
                       float(r["distance_m"])) for r in csv.DictReader(f)]


def format_summary(s: Summary) -> str:
    def opt(x: float | None, fmt: str) -> str:
        return "n/a" if x is None else format(x, fmt)
    return "\n".join([
        f"window {s.hours:.2f} h ({s.drive_hours:.2f} h driving), {s.distance_m:.0f} m driven",
        f"charge {s.pct_start:.0f}% -> {s.pct_end:.0f}% (fitted idle curve)",
        f"average draw {opt(s.avg_a, '.2f')} A overall, about {opt(s.drive_a, '.2f')} A while driving",
        f"projected runtime full -> alarm at this duty cycle: {opt(s.runtime_h, '.1f')} h, "
        f"{opt(s.km_per_charge, '.2f')} km",
        f"voltage sag while driving: {opt(s.sag_v, '.2f')} V; rough pack resistance "
        f"{opt(s.resistance_ohm, '.2f')} ohm",
    ])


def report(path: Path) -> None:
    cells, curve, _, capacity = load_config(CONFIG)
    samples = read_log(path)
    for p in rest_points(samples):
        loaded = "" if p.drive_v is None else f", {p.drive_v:.2f} V driving (sag {p.rest_v - p.drive_v:.2f})"
        print(f"{time.strftime('%H:%M', time.localtime(p.t))}  {p.distance:7.0f} m  rest {p.rest_v:.2f} V "
              f"({curve.percent(p.rest_v / cells):.0f}%){loaded}")
    print()
    print(format_summary(summarize(samples, curve, cells, capacity)))


# ---------------------------------------------------------------- robot side

class BatteryLog:
    """Logs every /battery sample with the test's current phase, cycle and distance."""

    def __init__(self, node, path: Path) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float32
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", buffering=1)
        self.file.write("unix_time,voltage,phase,cycle,distance_m\n")
        self.phase, self.cycle, self.distance = "rest", 0, 0.0
        self.recent: deque[tuple[float, float, str]] = deque(maxlen=600)
        node.create_subscription(Float32, "/battery", self._on_voltage, qos_profile_sensor_data)

    def _on_voltage(self, msg) -> None:
        t, v = time.time(), float(msg.data)
        if not 9.0 <= v <= 13.1:   # glitch
            return
        self.recent.append((t, v, self.phase))
        self.file.write(f"{t:.2f},{v:.3f},{self.phase},{self.cycle},{self.distance:.3f}\n")

    def median(self, phase: str, seconds: float) -> float | None:
        now = time.time()
        vs = [v for t, v, p in self.recent if p == phase and now - t <= seconds]
        return statistics.median(vs) if vs else None


def spin_for(node, seconds: float) -> None:
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def rest(node, log: BatteryLog, seconds: float) -> float:
    node.stop()
    log.phase = "rest"
    spin_for(node, seconds)
    v = log.median("rest", seconds - REST_SETTLE_S)
    if v is None:
        raise SystemExit("no /battery samples during the rest; is the robot connected?")
    return v


RELOCATE_SETTLE_S = 0.5
MAX_RELOCATE_FAILS = 3   # in a row; then stop rather than drift on odometry alone
BLOCKED_WAIT_S = 120.0   # a person or pet in the way: wait this long for the path to clear


STOP_DIST_CLEAR = 0.5    # m of clearance needed before resuming after a block


def out_and_back(node, log: BatteryLog, dist: float, line: tuple[float, float, float],
                 anchor, fails: list[int]) -> tuple[str | None, tuple[float, float, float]]:
    """One cycle; returns (reason to stop or None, line for the next cycle).
    fails: one-element counter of consecutive failed relocations."""
    log.phase = "drive"
    results = []
    if anchor is not None:
        spin_for(node, RELOCATE_SETTLE_S)
        found = node.relocate(anchor, line)
        if found is None:
            fails[0] += 1
            if fails[0] >= MAX_RELOCATE_FAILS:
                return f"scan matching failed {fails[0]} times in a row; position unknown", line
            results.append("relocate failed, keeping the line")
        else:
            fails[0] = 0
            line, loc = found
            results.append(f"at start {loc.y * 1000:+.1f} mm, {math.degrees(loc.yaw):+.2f} deg")
    for target in (dist, 0.0):   # fixed points on the line, so coasting errors don't add up
        waited = 0.0
        while True:
            leg = target - node.along(line)
            x0, y0, _ = node.pose()
            result = node.forward(leg, line)
            x1, y1, _ = node.pose()
            log.distance += math.hypot(x1 - x0, y1 - y0)
            if not result.startswith("STOPPED"):
                break
            # Something in the way (the dog, a person): wait for it to clear, then carry on
            # to the same target. Give up if it stays blocked.
            print(f"  blocked: {result}; waiting", flush=True)
            heading = 0.0 if leg >= 0 else math.pi
            start = time.monotonic()
            while node.clearance(heading) < STOP_DIST_CLEAR and time.monotonic() - start < BLOCKED_WAIT_S - waited:
                spin_for(node, 1.0)
            waited += time.monotonic() - start
            if node.clearance(heading) < STOP_DIST_CLEAR:
                return f"leg {leg:+.2f} m: blocked for {waited:.0f} s", line
            spin_for(node, 2.0)   # let it get clear of the path
        results.append(result)
        if result.startswith("timed out"):
            return f"leg {leg:+.2f} m: {result}", line
    print(f"  cycle {log.cycle}: " + " | ".join(results), flush=True)
    return None, line


def drive(args: argparse.Namespace) -> None:
    import rclpy

    from move import STOP_DIST, Mover, init_ros
    cells, curve, _, _ = load_config(CONFIG)
    path = LOG_DIR / f"drive_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.csv"
    init_ros()
    node = Mover()
    log = BatteryLog(node, path)
    reason = "interrupted"
    try:
        node.wait_ready()
        clear = node.clearance(0.0)
        if clear < STOP_DIST + args.dist:
            raise SystemExit(f"refusing: only {clear:.2f} m clear ahead, need {STOP_DIST + args.dist:.2f}")
        line = node.pose()
        anchor = None if args.no_scan_match else node.anchor()
        if anchor is not None:
            print(f"scan anchor: {len(anchor.ref)} reference points")
        start = time.monotonic()
        print(f"logging to {path}")
        reason = loop(node, log, args, line, anchor, start, cells, curve)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()   # rclpy's Ctrl-C handler is off (init_ros), so this still publishes
        log.file.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"\nstopped: {reason}\n")
    try:
        report(path)
    except ValueError as e:
        print(f"no summary: {e}")


def loop(node, log: BatteryLog, args: argparse.Namespace, line: tuple[float, float, float],
         anchor, start: float, cells: int, curve: Curve) -> str:
    next_rest = start  # rest first, for the starting charge
    fails = [0]
    while True:
        if time.monotonic() >= next_rest:
            v = rest(node, log, args.rest_s)
            hours = (time.monotonic() - start) / 3600
            print(f"{hours:5.2f} h  cycle {log.cycle:4d}  {log.distance:7.1f} m  rest {v:.2f} V "
                  f"({curve.percent(v / cells):.0f}%)", flush=True)
            if v <= args.stop_v:
                return f"resting voltage {v:.2f} V reached the {args.stop_v} V stop"
            next_rest = time.monotonic() + args.rest_every
        if time.monotonic() - start > args.max_hours * 3600:
            return f"{args.max_hours} h limit"
        log.cycle += 1
        problem, line = out_and_back(node, log, args.dist, line, anchor, fails)
        if problem:
            return problem
        loaded = log.median("drive", FLOOR_WINDOW_S)
        if loaded is not None and loaded < DRIVE_FLOOR_V:
            next_rest = time.monotonic()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--dist", type=float, default=1.0, help="out-and-back length (m)")
    r.add_argument("--no-scan-match", action="store_true", help="steer by odometry only")
    r.add_argument("--rest-every", type=float, default=600.0, help="seconds of driving between rests")
    r.add_argument("--rest-s", type=float, default=60.0, help="rest length (s), at least 45")
    r.add_argument("--stop-v", type=float, default=10.6, help="stop when the resting voltage reaches this")
    r.add_argument("--max-hours", type=float, default=5.0)
    p = sub.add_parser("report")
    p.add_argument("log", type=Path)
    args = ap.parse_args()
    if args.cmd == "report":
        report(args.log)
        return
    if not 0.2 <= args.dist <= 3.0 or args.rest_s < REST_SETTLE_S + 15 or args.stop_v < 10.2:
        raise SystemExit("need 0.2 <= --dist <= 3, --rest-s >= 45 and --stop-v >= 10.2")
    drive(args)


if __name__ == "__main__":
    main()
