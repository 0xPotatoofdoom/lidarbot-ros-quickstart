#!/usr/bin/env python3
"""Pan/tilt angles (/ptz_angle, degrees) -> /joint_states for robot_state_publisher.

Replaces the one job of Yahboom's base_driver we use (it fed /ptz_status to joint_state_publisher).
cam1 and cam2 (pan, tilt, radians, same as base_driver) are the model's only movable joints, so
publishing /joint_states directly saves the joint_state_publisher process (about 70 MB on the Pi).
"""
import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class PtzJointStates(Node):
    def __init__(self) -> None:
        super().__init__("ptz_joint_states")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        # The firmware publishes best effort.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Vector3, "/ptz_angle", self.on_angle, qos)

    def on_angle(self, msg: Vector3) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["cam1", "cam2"]
        js.position = [math.radians(msg.x), math.radians(msg.y)]
        self.pub.publish(js)


def main() -> None:
    rclpy.init()
    node = PtzJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
