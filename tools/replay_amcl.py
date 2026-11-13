#!/usr/bin/env python3
"""Replay a Nav2 bag through a fresh AMCL and compare it with the AMCL that ran live.

Used for the ROS 2 release migration: the same scans and odometry go into AMCL on the
Humble and the Jazzy image, and each run is compared with the map pose recorded live.

    ./ros python3 /work/tools/replay_amcl.py BAG_DIR OUT.csv [--rate 2.0] [--params FILE] [--nav]

--nav starts the full Nav2 bringup (bringup_launch.py) instead of map_server + AMCL only, and
reports whether every Nav2 node reached the active state: a config check for a new release.
The bag starts first there (Jazzy's costmaps need odom TF at startup), so AMCL is seeded with
the live pose instead of starting at the map origin.

Inside one container (use an isolated one, not the live stack: see docs/09-navigation.md):
plays BAG_DIR with /tf remapped to /tf_bag, relays every transform except map->odom back to
/tf, starts map_server + AMCL (nav2_bringup localization_launch.py, sim time) and samples
both map poses once per second of bag time:
    live   = recorded map->odom  o  odom->base_footprint
    replay = new AMCL map->odom  o  odom->base_footprint
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SAMPLE_PERIOD_S = 1.0
SETTLE_S = 10.0              # skip the first seconds while AMCL converges
MAP_YAML = "/root/map/home.yaml"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pose_of(tr) -> Pose2D:
    q = tr.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Pose2D(tr.transform.translation.x, tr.transform.translation.y, yaw)


def summarize(rows: list[dict]) -> str:
    kept = [r for r in rows if r["t"] >= SETTLE_S]
    if not kept:
        return "no samples after settling"
    d = sorted(r["dist_cm"] for r in kept)
    h = sorted(abs(r["dyaw_deg"]) for r in kept)

    def pct(v: list[float], p: float) -> float:
        return v[min(len(v) - 1, int(p * len(v)))]

    return (f"{len(kept)} samples: position diff median {pct(d, 0.5):.1f} cm, 95% {pct(d, 0.95):.1f} cm, "
            f"max {d[-1]:.1f} cm; heading diff median {pct(h, 0.5):.2f} deg, 95% {pct(h, 0.95):.2f} deg, "
            f"max {h[-1]:.2f} deg")


def run_compare(out: Path, wall_limit_s: float, seed: bool) -> list[dict]:
    """Sample both map poses; with seed, first set AMCL's pose to the live one (5 times, 1 s apart)."""
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node("replay_amcl_compare", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    pub = node.create_publisher(TFMessage, "/tf", qos)
    init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    seeds_left, last_seed = (5 if seed else 0), 0.0
    state: dict[str, Pose2D | None] = {"live": None, "odom": None, "replay": None}

    def on_bag(msg: TFMessage) -> None:
        keep = []
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                state["live"] = pose_of(tr)
                continue
            if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                state["odom"] = pose_of(tr)
            keep.append(tr)
        if keep:
            pub.publish(TFMessage(transforms=keep))

    def on_tf(msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                state["replay"] = pose_of(tr)

    node.create_subscription(TFMessage, "/tf_bag", on_bag, qos)
    node.create_subscription(TFMessage, "/tf", on_tf, qos)

    rows: list[dict] = []
    t0 = next_t = None
    last_progress = time.monotonic()
    wall_start = time.monotonic()
    while rclpy.ok() and time.monotonic() - wall_start < wall_limit_s:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = node.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            continue
        if t0 is None:
            t0, next_t = now, now
        if seeds_left and all(state.values()) and time.monotonic() - last_seed > 1.0:
            live = compose(state["live"], state["odom"])
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = live.x, live.y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(live.yaw / 2), math.cos(live.yaw / 2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
            msg.pose.covariance[35] = math.radians(2.0) ** 2
            init_pub.publish(msg)
            seeds_left, last_seed = seeds_left - 1, time.monotonic()
            continue
        if now >= next_t and all(state.values()):
            live = compose(state["live"], state["odom"])
            rep = compose(state["replay"], state["odom"])
            rows.append({"t": round(now - t0, 2), "live_x": live.x, "live_y": live.y,
                         "live_yaw_deg": math.degrees(wrap(live.yaw)), "replay_x": rep.x, "replay_y": rep.y,
                         "replay_yaw_deg": math.degrees(wrap(rep.yaw)),
                         "dist_cm": 100 * math.hypot(rep.x - live.x, rep.y - live.y),
                         "dyaw_deg": math.degrees(wrap(rep.yaw - live.yaw))})
            next_t += SAMPLE_PERIOD_S
            last_progress = time.monotonic()
        if rows and time.monotonic() - last_progress > 10.0:
            break                       # bag finished: sim time stopped advancing
    node.destroy_node()
    rclpy.shutdown()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as fh:
        if rows:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--params", help="Nav2 params file (default: config/nav2_$ROS_DISTRO.yaml or nav2.yaml)")
    ap.add_argument("--nav", action="store_true", help="full Nav2 bringup, not just localization")
    args = ap.parse_args()
    distro = os.environ.get("ROS_DISTRO", "humble")
    params = args.params or next(p for p in (f"/work/config/nav2_{distro}.yaml", "/work/config/nav2.yaml")
                                 if Path(p).exists())
    bag = Path(args.bag)
    bag = bag / "bag" if (bag / "bag").is_dir() else bag
    if not bag.is_dir():
        raise SystemExit(f"bag not found: {bag}")
    duration = float(subprocess.run(["ros2", "bag", "info", str(bag)], capture_output=True, text=True,
                                    check=True).stdout.split("Duration:")[1].split("s")[0])

    launch_file = "bringup_launch.py" if args.nav else "localization_launch.py"
    launch_log = Path("/tmp/replay_nav2.log" if args.nav else "/tmp/replay_localization.log")
    nav2 = ["ros2", "launch", "nav2_bringup", launch_file, f"map:={MAP_YAML}",
            f"params_file:={params}", "use_sim_time:=True", "autostart:=True"]
    play = ["ros2", "bag", "play", str(bag), "--clock", "--rate", str(args.rate),
            "--topics", "/scan", "/tf", "/tf_static", "/odom", "--remap", "/tf:=/tf_bag"]
    # Localization only: start AMCL first so it sees the first scans. Full Nav2: start the bag
    # first, because Jazzy's costmaps give up if odom->base_footprint isn't there within their
    # transform timeout (on the robot it always is).
    order = [(play, Path("/tmp/replay_bag.log")), (nav2, launch_log)] if args.nav else \
            [(nav2, launch_log), (play, Path("/tmp/replay_bag.log"))]
    procs = []
    for cmd, log_path in order:
        procs.append(subprocess.Popen(cmd, stdout=log_path.open("w"), stderr=subprocess.STDOUT,
                                      start_new_session=True))
        time.sleep(6.0 if cmd is nav2 else 1.0)
    try:
        rows = run_compare(Path(args.out), wall_limit_s=duration / args.rate + 60, seed=args.nav)
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    print(f"{distro}: {launch_file}, params {params}, {args.out}")
    print(summarize(rows))
    # Only count problems before the tool's own SIGINT; shutdown noise isn't a config error.
    log = launch_log.read_text(errors="replace").split("user interrupted with ctrl-c")[0]
    active = log.count("Managed nodes are active")
    errors = [line for line in log.splitlines() if "[ERROR]" in line or "[FATAL]" in line]
    print(f"lifecycle managers reporting active: {active}; error lines: {len(errors)} (log {launch_log})")
    for line in errors[:10]:
        print("  " + line[-200:])
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
