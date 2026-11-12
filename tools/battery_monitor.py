#!/usr/bin/env python3
"""Battery monitor: smoothed voltage, resting voltage, charge estimate and a log.

The firmware publishes the pack voltage on /battery (std_msgs/Float32, ~5 Hz) and
beeps below 10.2 V. This node (started by ./up) adds:
  /battery_state   sensor_msgs/BatteryState at 1 Hz: smoothed voltage, charge estimate
  datasets/battery/battery_YYYYMMDD.csv, every LOG_PERIOD seconds:
      unix_time,voltage,voltage_min,moving,rest_voltage,percent
Voltage sags while the motors draw current, so the charge estimate only updates
from the *resting* voltage: the median once the robot has been still for REST_AFTER s.
The voltage -> percent curve comes from config/battery.yaml (a generic Li-ion
curve until we fit our own from a full discharge).
"""
from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

CONFIG = Path("/work/config/battery.yaml")
LOG_DIR = Path("/root/datasets/battery")
WINDOW_S = 5.0          # smoothing window
REST_AFTER = 20.0       # s without drive commands before the voltage counts as resting
LOG_PERIOD = 5.0        # s between log rows


@dataclass(frozen=True)
class Curve:
    """Resting voltage per cell -> percent, linear between points."""
    points: tuple[tuple[float, float], ...]  # (volts per cell, percent), ascending volts

    def percent(self, volts_per_cell: float) -> float:
        pts = self.points
        if volts_per_cell <= pts[0][0]:
            return pts[0][1]
        if volts_per_cell >= pts[-1][0]:
            return pts[-1][1]
        for (v0, p0), (v1, p1) in zip(pts, pts[1:]):
            if v0 <= volts_per_cell <= v1:
                return p0 + (p1 - p0) * (volts_per_cell - v0) / (v1 - v0)
        raise AssertionError("unreachable")


def load_config(path: Path) -> tuple[int, Curve, float, float]:
    cfg = yaml.safe_load(path.read_text())["battery"]
    pts = tuple(sorted((float(v), float(p)) for v, p in cfg["ocv_curve"]))
    if len(pts) < 2 or any(b[1] < a[1] for a, b in zip(pts, pts[1:])):
        raise ValueError("ocv_curve needs >= 2 points with percent rising with voltage")
    return int(cfg["cells"]), Curve(pts), float(cfg["alarm_v"]), float(cfg.get("capacity_ah", "nan"))


class BatteryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("battery_monitor")
        self.cells, self.curve, self.alarm_v, self.capacity_ah = load_config(CONFIG)
        self.samples: deque[tuple[float, float]] = deque()
        self.last_motion = 0.0
        self.rest_voltage: float | None = None
        self.pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.create_subscription(Float32, "/battery", self._on_voltage, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0, self._publish)
        self.create_timer(LOG_PERIOD, self._log)
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"{self.cells} cells, {self.capacity_ah} Ah, alarm {self.alarm_v} V, log in {LOG_DIR}")

    def _on_voltage(self, msg: Float32) -> None:
        now = time.monotonic()
        if 3.0 * self.cells <= msg.data <= 4.35 * self.cells:  # ignore glitches
            self.samples.append((now, float(msg.data)))
        while self.samples and now - self.samples[0][0] > WINDOW_S:
            self.samples.popleft()

    def _on_cmd(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_motion = time.monotonic()

    def moving(self) -> bool:
        return time.monotonic() - self.last_motion < REST_AFTER

    def voltage(self) -> float | None:
        return statistics.median(v for _, v in self.samples) if self.samples else None

    def percent(self) -> float | None:
        return None if self.rest_voltage is None else self.curve.percent(self.rest_voltage / self.cells)

    def _publish(self) -> None:
        v = self.voltage()
        if v is None:
            return
        if not self.moving():
            self.rest_voltage = v
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = v
        pct = self.percent()
        msg.percentage = float("nan") if pct is None else pct / 100.0
        msg.design_capacity = self.capacity_ah
        msg.capacity = self.capacity_ah
        msg.charge = float("nan") if pct is None else self.capacity_ah * pct / 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.cell_voltage = [v / self.cells] * self.cells
        self.pub.publish(msg)

    def _log(self) -> None:
        v = self.voltage()
        if v is None:
            return
        vmin = min(x for _, x in self.samples)
        path = LOG_DIR / f"battery_{time.strftime('%Y%m%d', time.gmtime())}.csv"
        new = not path.exists()
        with path.open("a") as f:
            if new:
                f.write("unix_time,voltage,voltage_min,moving,rest_voltage,percent\n")
            rest, pct = self.rest_voltage, self.percent()
            f.write(f"{time.time():.1f},{v:.3f},{vmin:.3f},{int(self.moving())},"
                    f"{'' if rest is None else f'{rest:.3f}'},{'' if pct is None else f'{pct:.1f}'}\n")


def main() -> None:
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
