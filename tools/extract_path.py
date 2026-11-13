#!/usr/bin/env python3
"""Pull the robot's map pose and lidar points out of a bag for replays (tools/figures.py gif).

    ./ros python3 /work/tools/extract_path.py BAG_DIR OUT.npz [--step 1.0] [--start 0] [--seconds 200]

Uses /tf (map->odom from AMCL or slam_toolbox, odom->base_footprint from the EKF) and /scan,
sampled every --step seconds of bag time. Points are in map coordinates, within 4 m.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

LASER_X = -0.028
MAX_RANGE = 4.0


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=200.0)
    args = ap.parse_args()
    mcap = sorted(Path(args.bag).glob("**/*.mcap"))[0]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/scan"]))
    map_odom, odom_base = None, None
    poses, points, counts = [], [], []
    t0 = next_t = None
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        if t0 is None:
            t0 = t
            next_t = t0 + args.start
        if t - t0 > args.start + args.seconds:
            break
        if topic == "/tf":
            for tr in deserialize_message(data, TFMessage).transforms:
                p = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    map_odom = p
                elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                    odom_base = p
        elif t >= next_t and map_odom and odom_base:
            scan = deserialize_message(data, LaserScan)
            pose = compose(map_odom, odom_base)
            rng = np.asarray(scan.ranges)
            ang = scan.angle_min + scan.angle_increment * np.arange(len(rng))
            ok = np.isfinite(rng) & (rng > 0.15) & (rng < MAX_RANGE)
            lx, ly = rng[ok] * np.cos(ang[ok]) + LASER_X, rng[ok] * np.sin(ang[ok])
            c, s = math.cos(pose[2]), math.sin(pose[2])
            points.append(np.column_stack([pose[0] + c * lx - s * ly, pose[1] + s * lx + c * ly]))
            counts.append(int(ok.sum()))
            poses.append(pose)
            next_t += args.step
    np.savez_compressed(args.out, poses=np.array(poses), points=np.concatenate(points),
                        counts=np.array(counts), step=np.array([args.step]))
    print(f"{len(poses)} frames -> {args.out}")


if __name__ == "__main__":
    main()
