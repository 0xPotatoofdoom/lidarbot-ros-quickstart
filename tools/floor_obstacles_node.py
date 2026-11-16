"""ROS side of tools/floor_obstacles.py: camera frames in, obstacle and free-floor clouds out."""
from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

from floor_obstacles import DRIVE_TILT, FloorModel, detect, load_camera, pitch_for_tilt

POINT_Z = 0.05          # m: inside the costmap layer's height band
PAD_M = 0.05            # m: each contact point is also marked this far around it, so the
                        # controller's corner cutting (about half the planned detour on
                        # 2026-11-16) still leaves room; marking wins over clearing in Nav2
PAN_TOL = 1.0           # deg: only while the camera looks straight ahead
TILT_TOL = 1.0          # deg around the driving tilt
LOG_EVERY_S = 30.0


def padded(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    ring = [(PAD_M * np.cos(a), PAD_M * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
    return pts + [(x + dx, y + dy) for x, y in pts for dx, dy in ring]


def cloud(header: Header, pts: list[tuple[float, float]]) -> PointCloud2:
    data = np.array([(x, y, POINT_Z) for x, y in pts], np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    return PointCloud2(header=header, height=1, width=len(data), fields=fields, is_bigendian=False,
                       point_step=12, row_step=12 * len(data), is_dense=True, data=data.tobytes())


class FloorObstacles(Node):
    def __init__(self) -> None:
        super().__init__("floor_obstacles")
        self.cam = load_camera(pitch_for_tilt(DRIVE_TILT))
        self.model = FloorModel()
        self.ptz: Vector3 | None = None
        self.pub_obs = self.create_publisher(PointCloud2, "/camera_obstacles", qos_profile_sensor_data)
        self.pub_free = self.create_publisher(PointCloud2, "/camera_floor_free", qos_profile_sensor_data)
        self.create_subscription(Vector3, "/ptz_angle", self._on_ptz, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data)
        self.frames = self.skipped = self.hits = 0
        self.last_log = time.monotonic()
        self.get_logger().info(f"camera pitch {self.cam.pitch_deg:.1f} deg down at tilt {DRIVE_TILT:.0f}")

    def _on_ptz(self, msg: Vector3) -> None:
        self.ptz = msg

    def _aimed(self) -> bool:
        return (self.ptz is not None and abs(self.ptz.x) < PAN_TOL
                and abs(self.ptz.y - DRIVE_TILT) < TILT_TOL)

    def _on_image(self, msg: Image) -> None:
        if not self._aimed() or msg.encoding not in ("bgr8", "rgb8") or (msg.width, msg.height) != (640, 480):
            self.skipped += 1
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[..., ::-1]
        det, _ = detect(np.ascontiguousarray(img), self.cam, self.model)
        header = Header(stamp=msg.header.stamp, frame_id="base_footprint")
        self.pub_obs.publish(cloud(header, padded(det.obstacles)))
        self.pub_free.publish(cloud(header, det.free))
        self.frames += 1
        self.hits += bool(det.obstacles)
        if time.monotonic() - self.last_log > LOG_EVERY_S:
            self.get_logger().info(f"{self.frames} frames, {self.hits} with obstacles, {self.skipped} skipped "
                                   f"(camera not at pan 0 / tilt {DRIVE_TILT:.0f})")
            self.last_log = time.monotonic()


def run() -> None:
    rclpy.init()
    node = FloorObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
