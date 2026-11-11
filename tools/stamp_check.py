#!/usr/bin/env python3
"""Print how far each topic's header stamp lags (+) or leads (-) the host clock.

./ros python3 /work/tools/stamp_check.py [seconds]
Large or negative lags cause slam_toolbox "Message Filter dropping" warnings.
"""
from __future__ import annotations

import statistics
import sys
import time
from collections.abc import Callable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

TOPICS = {"/scan": LaserScan, "/odom_raw": Odometry, "/imu": Imu, "/odom": Odometry}
TF_KEY = "/tf odom->base"


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    rclpy.init()
    node = Node("stamp_check")
    lag: dict[str, list[float]] = {t: [] for t in [*TOPICS, TF_KEY]}

    def now_s() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def recorder(topic: str) -> Callable:
        return lambda m: lag[topic].append(now_s() - stamp_s(m.header.stamp))

    def on_tf(msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.child_frame_id == "base_footprint":
                lag[TF_KEY].append(now_s() - stamp_s(tr.header.stamp))

    for topic, msg_type in TOPICS.items():
        node.create_subscription(msg_type, topic, recorder(topic), qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    for topic, values in lag.items():
        if not values:
            print(f"{topic:<16} no messages")
            continue
        ms = [v * 1000 for v in values]
        print(f"{topic:<16} n={len(ms):3d}  lag median {statistics.median(ms):7.1f} ms  "
              f"min {min(ms):7.1f}  max {max(ms):7.1f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
