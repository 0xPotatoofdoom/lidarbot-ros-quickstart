#!/usr/bin/env python3
"""One out-and-back cycle with overhead tape-tracker ground truth at each stop.

Runs inside the container (./up running, tape tracker running on the lab Pi):
    ./record run linear-repeat ./ros python3 /work/tools/repeat_linear.py [meters]

At the start, the far point and back at the start, it waits for the robot to
settle and takes the median of fresh tape-tracker readings, alongside what
every odometry source and the lidar say. The last line of output is
GROUND_TRUTH_JSON {...}, which ./record saves as ground_truth.json.
The tracker URL comes from config/lab.json ("tape_url").
"""
from __future__ import annotations

import json
import math
import statistics
import sys
import time
import urllib.request

import rclpy

sys.path.insert(0, "/work/tools")
from calibrate import Recorder, grab_scan, range_ahead, run  # noqa: E402
from move import STOP_DIST, Mover  # noqa: E402

READINGS = 3
SETTLE_S = 1.5
INCH = 0.0254


def tape_url() -> str:
    try:
        cfg = json.load(open("/work/config/lab.json"))
        return cfg["tape_url"].rstrip("/")
    except (OSError, KeyError, ValueError) as e:
        raise SystemExit(f"config/lab.json with a tape_url is needed ({e})") from e


def tape_reading(node: Mover, url: str) -> dict:
    """Median of READINGS tape-tracker readings taken after this call started."""
    t0 = time.time()
    got: list[dict] = []
    deadline = time.monotonic() + 15
    while len(got) < READINGS and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            with urllib.request.urlopen(url + "/reading", timeout=3) as r:
                rd = json.loads(r.read())
        except OSError:
            continue
        if rd and rd["unix_time"] > t0 and (not got or rd["unix_time"] != got[-1]["unix_time"]):
            got.append(rd)
    if len(got) < READINGS:
        raise SystemExit(f"tape tracker gave only {len(got)} fresh readings; is it running?")
    keys = ("front_in", "raw_in", "s_px")
    out = {k: statistics.median(g[k] for g in got) for k in keys}
    out["spread_in"] = max(g["front_in"] for g in got) - min(g["front_in"] for g in got)
    return out


def settle(node: Mover) -> None:
    node.stop()
    end = time.monotonic() + SETTLE_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    url = tape_url()
    rclpy.init()
    node = Mover()
    rec = Recorder(node)
    stops: list[dict] = []
    try:
        node.wait_ready()
        settle(node)
        ahead = range_ahead(*grab_scan(node))
        if not math.isfinite(ahead) or ahead < dist + STOP_DIST + 0.15:
            raise SystemExit(f"need {dist + STOP_DIST + 0.15:.2f} m clear ahead, lidar says {ahead:.2f} m")
        line = node.pose()  # both legs follow the line through the starting pose
        for leg, sign in (("start", 0), ("far", 1), ("return", -1)):
            if sign:
                result, t = run(node, rec, lambda s=sign: node.forward(s * dist, line))
                print(f"drive {sign * dist:+.2f} m: {result}; EKF {t.ekf_dist * sign:+.4f} m, "
                      f"/odom_raw pose {t.raw_pose_dist * sign:+.4f} m")
            settle(node)
            tape = tape_reading(node, url)
            lidar = range_ahead(*grab_scan(node))
            stops.append({"stop": leg, "tape": tape, "lidar_ahead_m": round(lidar, 4)}
                         | ({"ekf_m": round(t.ekf_dist * sign, 4), "odom_raw_pose_m": round(t.raw_pose_dist * sign, 4)}
                            if sign else {}))
            print(f"  {leg:<6} tape {tape['front_in']:.3f} in (spread {tape['spread_in']:.3f}), lidar {lidar:.4f} m")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    out_in = stops[0]["tape"]["front_in"] - stops[1]["tape"]["front_in"]
    back_in = stops[2]["tape"]["front_in"] - stops[1]["tape"]["front_in"]
    home_err = stops[2]["tape"]["front_in"] - stops[0]["tape"]["front_in"]
    print(f"\ntape: out {out_in * INCH:.4f} m, back {back_in * INCH:.4f} m, "
          f"return-to-start error {home_err * INCH * 1000:+.1f} mm")
    gt = {"source": "overhead camera tape tracker (lab/tape_tracker.py)", "units": "inches along tape, zero at the wall",
          "commanded_m": dist, "stops": stops,
          "out_m": round(out_in * INCH, 4), "back_m": round(back_in * INCH, 4),
          "return_error_mm": round(home_err * INCH * 1000, 1)}
    print("GROUND_TRUTH_JSON " + json.dumps(gt))


if __name__ == "__main__":
    main()
