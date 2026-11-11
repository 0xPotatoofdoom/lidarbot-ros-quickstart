#!/usr/bin/env python3
"""Command a sequence of velocities and report what the wheel odometry measured.

Runs inside the container:  ./ros python3 /work/tools/drive_test.py
Each step is (label, linear m/s, angular rad/s, seconds). Always ends stopped.
"""
from __future__ import annotations

import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

STEPS = (
    ("forward", 0.15, 0.0, 2.0),
    ("stop", 0.0, 0.0, 1.0),
    ("reverse", -0.15, 0.0, 2.0),
    ("stop", 0.0, 0.0, 1.0),
    ("turn left", 0.0, 0.8, 2.0),
    ("stop", 0.0, 0.0, 1.0),
    ("turn right", 0.0, -0.8, 2.0),
    ("stop", 0.0, 0.0, 1.0),
)
RATE_HZ = 20.0
SETTLE_S = 0.5  # ignore the ramp-up at the start of each step


class DriveTest(Node):
    def __init__(self) -> None:
        super().__init__("drive_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.samples: list[tuple[float, float, float]] = []
        self.create_subscription(Odometry, "/odom_raw", self._odom, qos_profile_sensor_data)

    def _odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self.samples.append((time.monotonic(), t.linear.x, t.angular.z))

    def send(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def run_step(self, vx: float, wz: float, duration: float) -> tuple[float, float, int]:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send(vx, wz)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)
        window = [s for s in self.samples if s[0] >= start + SETTLE_S]
        if not window:
            return float("nan"), float("nan"), 0
        return (statistics.mean(s[1] for s in window),
                statistics.mean(s[2] for s in window), len(window))


def main() -> None:
    rclpy.init()
    node = DriveTest()
    try:
        # Wait for odometry so we know the robot is connected.
        deadline = time.monotonic() + 5.0
        while not node.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.samples:
            raise SystemExit("No /odom_raw received; is the robot connected?")

        print(f"{'step':<11} {'cmd vx':>7} {'meas vx':>8} {'cmd wz':>7} {'meas wz':>8}  n")
        for label, vx, wz, dur in STEPS:
            mvx, mwz, n = node.run_step(vx, wz, dur)
            print(f"{label:<11} {vx:>7.2f} {mvx:>8.3f} {wz:>7.2f} {mwz:>8.3f}  {n}")
    finally:
        for _ in range(5):
            node.send(0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
