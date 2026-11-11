#!/usr/bin/env python3
"""Apply odometry scale corrections before the EKF.

The firmware has no wheel-geometry settings, so this node rescales its output:
    /odom_raw -> /odom_cal   twist.linear.x * linear_scale,
                             twist.angular.z * angular_scale_left (CCW) or _right (CW)
    /imu      -> /imu_cal    yaw (unwrapped) and angular_velocity.z * imu_yaw_scale
config/ekf.yaml fuses /odom_cal and /imu_cal. Scale factors come from
config/calibration.yaml (measure them with tools/calibrate.py); all default to 1.0.

./up starts it:  python3 /work/tools/odom_calibrator.py --ros-args --params-file /work/config/calibration.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SCALE_LIMITS = (0.5, 2.0)  # anything outside this is a measurement mistake
MAX_STAMP_SKEW = 0.5       # s; messages further than this from the host clock are dropped


@dataclass(frozen=True)
class Scales:
    linear: float = 1.0
    angular_left: float = 1.0
    angular_right: float = 1.0
    imu_yaw: float = 1.0

    def validated(self) -> Scales:
        for name, v in vars(self).items():
            if not SCALE_LIMITS[0] <= v <= SCALE_LIMITS[1]:
                raise ValueError(f"{name}={v} is outside {SCALE_LIMITS}; check config/calibration.yaml")
        return self

    def angular(self, wz: float) -> float:
        return wz * (self.angular_left if wz >= 0 else self.angular_right)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rpy_of(q: Quaternion) -> tuple[float, float, float]:
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quat_of(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


class YawScaler:
    """Scales an absolute yaw about its first reading, keeping it continuous."""

    def __init__(self, scale: float) -> None:
        self.scale = scale
        self.first: float | None = None
        self.last: float | None = None
        self.unwrapped = 0.0

    def __call__(self, yaw: float) -> float:
        if self.first is None or self.last is None:
            self.first = self.last = yaw
            return yaw
        self.unwrapped += wrap(yaw - self.last)
        self.last = yaw
        return wrap(self.first + self.unwrapped * self.scale)


class OdomCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("odom_calibrator")
        p = {name: self.declare_parameter(name, 1.0).value
             for name in ("linear_scale", "angular_scale_left", "angular_scale_right", "imu_yaw_scale")}
        self.scales = Scales(p["linear_scale"], p["angular_scale_left"],
                             p["angular_scale_right"], p["imu_yaw_scale"]).validated()
        self.yaw = YawScaler(self.scales.imu_yaw)
        self.dropped = 0
        self.get_logger().info(f"scales: {self.scales}")
        # Reliable publishers: compatible with both reliable and best-effort subscribers.
        self.odom_pub = self.create_publisher(Odometry, "/odom_cal", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu_cal", 10)
        self.create_subscription(Odometry, "/odom_raw", self._odom, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu", self._imu, qos_profile_sensor_data)

    def _stamp_ok(self, stamp, topic: str) -> bool:
        """Drop messages whose stamp is far from the host clock.

        Right after the agent (re)starts, the robot re-syncs its clock and a few
        messages can carry stamps that are seconds to days off. The EKF would
        integrate the current velocity across that gap and jump hundreds of meters.
        """
        skew = abs(self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9))
        if skew <= MAX_STAMP_SKEW:
            return True
        self.dropped += 1
        self.get_logger().warning(
            f"dropping {topic} with stamp {skew:.1f} s off the host clock ({self.dropped} so far)",
            throttle_duration_sec=5.0)
        return False

    def _odom(self, m: Odometry) -> None:
        if not self._stamp_ok(m.header.stamp, "/odom_raw"):
            return
        m.twist.twist.linear.x *= self.scales.linear
        m.twist.twist.angular.z = self.scales.angular(m.twist.twist.angular.z)
        self.odom_pub.publish(m)

    def _imu(self, m: Imu) -> None:
        if not self._stamp_ok(m.header.stamp, "/imu"):
            return
        roll, pitch, yaw = rpy_of(m.orientation)
        m.orientation = quat_of(roll, pitch, self.yaw(yaw))
        m.angular_velocity.z *= self.scales.imu_yaw
        self.imu_pub.publish(m)


def main() -> None:
    rclpy.init()
    node = OdomCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
