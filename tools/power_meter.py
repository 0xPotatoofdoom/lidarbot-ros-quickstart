"""Battery current and energy from the open firmware's INA228 topics.

    /power          sensor_msgs/BatteryState, 5 Hz: voltage, current (negative = discharging)
    /power_used_wh  std_msgs/Float32, 5 Hz: energy drawn since the board booted (INA228 counter)

PowerMeter keeps a running window that callers read and reset: mean and peak draw (amps,
positive = out of the battery) and energy used since the last reset. Yahboom's firmware has
no current sensor, so with it every value stays None.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


@dataclass(frozen=True)
class PowerWindow:
    samples: int
    mean_a: float | None      # mean draw over the window
    peak_a: float | None      # highest draw in the window
    mean_w: float | None
    wh: float | None          # energy used during the window (INA228 counter difference)
    wh_total: float | None    # counter value at the end of the window (since boot)


class PowerMeter:
    def __init__(self, node: Node) -> None:
        self._sum_a = self._sum_w = 0.0
        self._peak_a: float | None = None
        self._n = 0
        self._wh: float | None = None
        self._wh_start: float | None = None
        node.create_subscription(BatteryState, "/power", self._on_power, qos_profile_sensor_data)
        node.create_subscription(Float32, "/power_used_wh", self._on_wh, qos_profile_sensor_data)

    def _on_power(self, msg: BatteryState) -> None:
        if not (math.isfinite(msg.current) and math.isfinite(msg.voltage)):
            return
        draw = -msg.current
        self._sum_a += draw
        self._sum_w += draw * msg.voltage
        self._peak_a = draw if self._peak_a is None else max(self._peak_a, draw)
        self._n += 1

    def _on_wh(self, msg: Float32) -> None:
        if not math.isfinite(msg.data):
            return
        if self._wh is not None and msg.data < self._wh:
            self._wh_start = None            # board rebooted: its counter restarted at 0
        self._wh = float(msg.data)
        if self._wh_start is None:
            self._wh_start = self._wh

    def take(self) -> PowerWindow:
        """Values since the previous take() (or since start), then start a new window."""
        n = self._n
        wh = None if self._wh is None or self._wh_start is None else self._wh - self._wh_start
        out = PowerWindow(n, self._sum_a / n if n else None, self._peak_a, self._sum_w / n if n else None,
                          wh, self._wh)
        self._sum_a = self._sum_w = 0.0
        self._peak_a = None
        self._n = 0
        self._wh_start = self._wh
        return out


def fmt(v: float | None, digits: int = 3) -> str:
    return "" if v is None else f"{v:.{digits}f}"
