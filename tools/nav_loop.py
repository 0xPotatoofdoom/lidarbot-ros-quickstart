#!/usr/bin/env python3
"""Drive the same Nav2 loop many times and measure every arrival with lidar scan matching.

    ./ros python3 /work/tools/nav_loop.py --laps 5        # ./up nav, robot at the start

Each lap visits the waypoints in order (default: den doorway, living room east end, kitchen
start). Two independent measurements, besides AMCL's own estimate:
- map match: at every waypoint, the scan is matched (ICP) against the saved map's walls near
  the robot, independently of AMCL's particle filter. Matching against a single earlier scan
  instead (the first version) was biased: at the kitchen start it traded about 1.8 deg of
  heading for 3 cm of sideways shift, because the view is down a narrow strip.
- repeat: at every waypoint, the scan is matched against the scan from the first lap's arrival
  there, which gives how far apart repeated arrivals are.
Rows go to /root/datasets/nav/loop_<UTC>.csv. Stops if a goal fails.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav_goal import arrival_error, drive_goal, map_pose, nav_client  # noqa: E402
from scan_anchor import Located, ScanAnchor, plausible  # noqa: E402

MAP_YAML = "/root/map/home.yaml"
MAP_RADIUS = 3.3       # m of map walls around the robot used for matching
MAP_POINTS = 1500
LASER_X = -0.028       # laser frame behind base_footprint (URDF)
SEEDS = [(0.0, 0.0, 0.0), (0.04, 0.0, 0.0), (-0.04, 0.0, 0.0), (0.0, 0.04, 0.0), (0.0, -0.04, 0.0),
         (0.0, 0.0, math.radians(2.0)), (0.0, 0.0, math.radians(-2.0))]
WAYPOINTS = [("den-door", 3.5, -0.45, 0.0), ("living-east", 4.9, 2.6, 0.0), ("kitchen-start", 0.0, 0.0, 0.0)]
MATCH_SCANS = 5
# AMCL can be several cm and degrees off, so allow a wider gap than relocation in move.py.
MAX_JUMP = 0.25
MAX_TURN = math.radians(10.0)
OUT_DIR = Path("/root/datasets/nav")


def map_walls(yaml_path: str):
    """Occupied cells of a saved map as (N, 2) points in map coordinates."""
    import cv2
    import numpy as np
    import yaml
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    img = cv2.imread(str(Path(yaml_path).parent / cfg["image"]), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy, _) = cfg["resolution"], cfg["origin"]
    rows, cols = np.where(img < 50)
    return np.column_stack([ox + (cols + 0.5) * res, oy + (img.shape[0] - 1 - rows + 0.5) * res])


def match_map(walls, scans: list, guess: tuple[float, float, float]) -> Located | None:
    """Robot pose in the map from ICP of each scan against nearby map walls (median), or None
    if most scans fail or the result is implausibly far from guess."""
    import numpy as np
    from scan_anchor import icp, scan_points
    near = walls[np.hypot(walls[:, 0] - guess[0], walls[:, 1] - guess[1]) < MAP_RADIUS]
    if len(near) > MAP_POINTS:
        near = near[np.linspace(0, len(near) - 1, MAP_POINTS).astype(int)]
    locs = []
    for s in scans:
        src = scan_points(s.ranges, s.angle_min, s.angle_increment, 400) + np.array([LASER_X, 0.0])
        # Point-to-point ICP has local minima a few cm apart along narrow spaces (at the kitchen
        # start: 3 cm and 1.3 deg apart), so start from several guesses and keep the best fit.
        fits = [loc for dx, dy, dyaw in SEEDS
                if (loc := icp(src, near, (guess[0] + dx, guess[1] + dy, guess[2] + dyaw))) is not None]
        if fits:
            locs.append(min(fits, key=lambda l: l.rms))
    if len(locs) < (len(scans) + 1) // 2:
        return None
    loc = median_located(locs)
    return loc if plausible(loc, guess, MAX_JUMP, MAX_TURN) else None


def median_located(locs: list[Located]) -> Located:
    def mid(vals: list[float]) -> float:
        return sorted(vals)[len(vals) // 2]
    return Located(mid([l.x for l in locs]), mid([l.y for l in locs]), mid([l.yaw for l in locs]),
                   mid([l.rms for l in locs]), mid([l.inliers for l in locs]))


class Scans:
    def __init__(self, node) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self.node = node
        self.recent: deque = deque(maxlen=MATCH_SCANS * 2)
        self.count = 0
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg) -> None:
        self.recent.append(msg)
        self.count += 1

    def fresh(self, n: int = MATCH_SCANS) -> list:
        import rclpy
        start = self.count
        end = time.monotonic() + 5.0
        while self.count - start < n and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return list(self.recent)[-n:]


def locate(anchor: ScanAnchor, scans: list, guess_pose: tuple[float, float, float]) -> Located | None:
    """Median match of several scans; None if most fail or the result is implausibly far
    from guess_pose (which is in the same frame as anchor.odom0)."""
    locs = [l for s in scans if (l := anchor.locate(s, guess_pose)) is not None]
    if len(locs) < (len(scans) + 1) // 2:
        return None
    loc = median_located(locs)
    return loc if plausible(loc, anchor.odom_guess(guess_pose), MAX_JUMP, MAX_TURN) else None


def fmt(v: float | None, scale: float = 1.0, digits: int = 1) -> str:
    return "" if v is None else f"{v * scale:.{digits}f}"


def run(laps: int, map_yaml: str) -> None:
    import rclpy
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node("nav_loop")
    tf = Buffer()
    TransformListener(tf, node)
    scans = Scans(node)
    walls = map_walls(map_yaml)
    repeat_anchors: dict[str, ScanAnchor] = {}
    client = nav_client(node)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / time.strftime("loop_%Y%m%dT%H%M%SZ.csv", time.gmtime())
    out.write_text("unix_time,lap,name,status,seconds,goal_x,goal_y,goal_yaw_deg,amcl_x,amcl_y,amcl_yaw_deg,"
                   "amcl_err_cm,match_x,match_y,match_yaw_deg,match_err_cm,amcl_vs_match_cm,"
                   "amcl_vs_match_deg,repeat_dx_cm,repeat_dy_cm,repeat_dyaw_deg\n")
    print(f"logging to {out}")
    try:
        for lap in range(1, laps + 1):
            for name, gx, gy, gyaw_deg in WAYPOINTS:
                gyaw = math.radians(gyaw_deg)
                status, secs, amcl = drive_goal(node, tf, client, gx, gy, gyaw)
                if status != "succeeded" or amcl is None:
                    print(f"lap {lap} {name}: {status}; stopping")
                    return
                live = scans.fresh()
                amcl_err = arrival_error((gx, gy, gyaw), amcl)
                truth = match_map(walls, live, amcl)
                rep = None
                if name not in repeat_anchors:
                    repeat_anchors[name] = ScanAnchor(live[-1], amcl)
                else:
                    rep = locate(repeat_anchors[name], live, amcl)
                t_err = arrival_error((gx, gy, gyaw), (truth.x, truth.y, truth.yaw)) if truth else None
                a_vs_t = arrival_error((truth.x, truth.y, truth.yaw), amcl) if truth else None
                line = (f"lap {lap} {name}: {secs:.0f} s, AMCL off by {amcl_err.dist_m * 100:.1f} cm "
                        f"{amcl_err.dyaw_deg:+.1f} deg")
                if t_err:
                    line += (f" | map match off by {t_err.dist_m * 100:.1f} cm {t_err.dyaw_deg:+.1f} deg, "
                             f"AMCL wrong by {a_vs_t.dist_m * 100:.1f} cm {a_vs_t.dyaw_deg:+.1f} deg")
                if rep:
                    line += (f" | vs lap 1: {rep.x * 100:+.1f}, {rep.y * 100:+.1f} cm, "
                             f"{math.degrees(rep.yaw):+.1f} deg")
                print(line, flush=True)
                with out.open("a") as f:
                    f.write(",".join([
                        f"{time.time():.1f}", str(lap), name, status, f"{secs:.1f}", str(gx), str(gy), str(gyaw_deg),
                        f"{amcl[0]:.3f}", f"{amcl[1]:.3f}", f"{math.degrees(amcl[2]):.2f}", f"{amcl_err.dist_m * 100:.1f}",
                        fmt(truth.x if truth else None, 1, 3), fmt(truth.y if truth else None, 1, 3),
                        fmt(math.degrees(truth.yaw) if truth else None, 1, 2),
                        fmt(t_err.dist_m if t_err else None, 100), fmt(a_vs_t.dist_m if a_vs_t else None, 100),
                        fmt(a_vs_t.dyaw_deg if a_vs_t else None, 1, 2),
                        fmt(rep.x if rep else None, 100), fmt(rep.y if rep else None, 100),
                        fmt(math.degrees(rep.yaw) if rep else None, 1, 2)]) + "\n")
    except KeyboardInterrupt:
        print("interrupted; cancel the Nav2 goal with ./up nav (restart) if the robot is still moving")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laps", type=int, default=5)
    ap.add_argument("--map", default=MAP_YAML)
    args = ap.parse_args()
    run(args.laps, args.map)


if __name__ == "__main__":
    main()
