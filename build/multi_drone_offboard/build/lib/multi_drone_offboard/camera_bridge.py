#!/usr/bin/env python3
"""
Gazebo -> ROS 2 camera bridge for multi_drone_offboard

Publishes:
  /drone0/camera/image_raw
  /drone1/camera/image_raw
  /drone2/camera/image_raw

Environment variables:
  WORLD      (default: baylands_collapsed_fixed)
  NUM_DRONES (default: 3)
  MAX_HZ     (default: 5)
"""

import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

try:
    import gz.transport13 as gz_transport
    import gz.msgs10.image_pb2 as image_pb2
except ImportError as exc:
    raise SystemExit(
        "Missing Gazebo Python bindings for this Gazebo version.\n"
        "Expected modules: gz.transport13 and gz.msgs10.image_pb2\n"
        f"Import error: {exc}"
    ) from exc


WORLD = os.environ.get("WORLD", "baylands_collapsed_fixed")
NUM_DRONES = int(os.environ.get("NUM_DRONES", "3"))
MAX_HZ = float(os.environ.get("MAX_HZ", "5"))


# Gazebo pixel format -> ROS encoding
# Only map formats we can honestly pass through without byte conversion.
GZ_FORMAT_MAP = {
    1: "rgb8",    # RGB_INT8
    2: "rgba8",   # RGBA_INT8
    3: "bgr8",    # BGR_INT8
    5: "mono8",   # L_INT8
    6: "mono16",  # L_INT16
    12: "32FC1",  # FLOAT32
    13: "64FC1",  # FLOAT64
}


def gz_to_ros_encoding(gz_img) -> str:
    """Map Gazebo pixel_format_type to a ROS encoding string."""
    return GZ_FORMAT_MAP.get(gz_img.pixel_format_type, "rgb8")


def gz_to_ros_image(gz_img, frame_id: str) -> Image:
    """Convert a Gazebo protobuf image to sensor_msgs/Image."""
    ros_img = Image()
    ros_img.header.stamp = Clock().now().to_msg()
    ros_img.header.frame_id = frame_id

    ros_img.height = gz_img.height
    ros_img.width = gz_img.width
    ros_img.encoding = gz_to_ros_encoding(gz_img)
    ros_img.is_bigendian = 0
    ros_img.step = gz_img.step
    ros_img.data = bytes(gz_img.data)

    return ros_img


class CameraHandler:
    """One Gazebo camera subscription + one ROS image publisher."""

    def __init__(
        self,
        node: Node,
        gz_node,
        drone_id: int,
        gz_topic: str,
        ros_topic: str,
        max_hz: float,
    ) -> None:
        self.node = node
        self.drone_id = drone_id
        self.gz_topic = gz_topic
        self.ros_topic = ros_topic
        self.min_period = 1.0 / max_hz if max_hz > 0.0 else 0.0

        self._last_pub = 0.0
        self._lock = threading.Lock()
        self._rx_count = 0
        self._tx_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = node.create_publisher(Image, ros_topic, qos)

        ok = gz_node.subscribe(image_pb2.Image, gz_topic, self._callback)
        node.get_logger().info(
            f"[drone{drone_id}] subscribe={ok} | gz:{gz_topic} -> ros:{ros_topic} "
            f"| max_hz={max_hz}"
        )

    def _callback(self, gz_img) -> None:
        self._rx_count += 1
        now = time.monotonic()

        with self._lock:
            if self.min_period > 0.0 and (now - self._last_pub) < self.min_period:
                return
            self._last_pub = now

        try:
            ros_img = gz_to_ros_image(gz_img, f"drone{self.drone_id}/camera_link")
            self.publisher.publish(ros_img)
            self._tx_count += 1
        except Exception as exc:
            self.node.get_logger().warn(
                f"[drone{self.drone_id}] frame conversion error: {exc}",
                throttle_duration_sec=5.0,
            )


class GzRosCameraBridge(Node):
    """Bridge Gazebo IMX214 camera topics for multiple x500_depth drones."""

    def __init__(self) -> None:
        super().__init__("gz_ros_camera_bridge")

        self.world = WORLD
        self.num_drones = NUM_DRONES
        self.max_hz = MAX_HZ

        self.gz_node = gz_transport.Node()
        self.handlers = []

        for i in range(self.num_drones):
            gz_topic = (
                f"/world/{self.world}/model/x500_depth_{i}"
                f"/link/camera_link/sensor/IMX214/image"
            )
            ros_topic = f"/drone{i}/camera/image_raw"

            handler = CameraHandler(
                node=self,
                gz_node=self.gz_node,
                drone_id=i,
                gz_topic=gz_topic,
                ros_topic=ros_topic,
                max_hz=self.max_hz,
            )
            self.handlers.append(handler)

        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"Bridge ready | world='{self.world}' | num_drones={self.num_drones} | max_hz={self.max_hz}"
        )

    def _log_stats(self) -> None:
        for h in self.handlers:
            self.get_logger().info(
                f"[drone{h.drone_id}] rx={h._rx_count} published={h._tx_count}"
            )


def main() -> None:
    rclpy.init()
    node = GzRosCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()