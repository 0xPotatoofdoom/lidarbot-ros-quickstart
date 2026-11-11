#!/usr/bin/env python3
"""Republish /camera/image_raw as JPEG on /camera/image_raw/compressed.

Raw 640x480 frames cost about 8 GB per hour in a recording; JPEG at quality
85 is roughly a tenth of that. ./up starts this alongside the camera driver.
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

JPEG_QUALITY = 85
CHANNELS = {"bgr8": 3, "rgb8": 3, "mono8": 1}


class JpegRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("jpeg_republisher")
        self.pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data)
        self.warned = False

    def _on_image(self, msg: Image) -> None:
        channels = CHANNELS.get(msg.encoding)
        if channels is None:
            if not self.warned:
                self.get_logger().warning(f"unsupported encoding {msg.encoding}; not republishing")
                self.warned = True
            return
        img: np.ndarray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // channels, channels)
        img = img[:, : msg.width]
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            self.get_logger().warning("JPEG encoding failed")
            return
        out = CompressedImage(header=msg.header, format="jpeg", data=buf.tobytes())
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JpegRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
