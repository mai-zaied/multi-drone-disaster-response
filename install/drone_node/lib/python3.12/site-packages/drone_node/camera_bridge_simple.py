"""
camera_bridge_simple.py

A throttled, single-drone Gazebo-to-ROS2 RGB camera bridge.

Parameterised by 'instance' (integer). All derived names (drone_id,
Gazebo model name, ROS2 output topic) are generated from this single
number using drone_naming.py.

Safety features (unchanged from Task 3.3):
- Hard-capped publish rate (default 2 Hz)
- Single-slot frame queue (drop-on-full)
- Stats every 5 s
"""

import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

from drone_node.drone_naming import drone_id_for, gz_model_name_for


# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
GZ_WORLD_NAME = 'baylands_collapsed_fixed'
GZ_SENSOR_NAME = 'IMX214'
ROS_PUBLISH_HZ = 2.0
FRAME_QUEUE_SIZE = 1


class CameraBridgeSimple(Node):
    def __init__(self):
        super().__init__('camera_bridge_simple')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.declare_parameter('publish_hz', ROS_PUBLISH_HZ)
        self.instance = int(self.get_parameter('instance').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        # ---- Derived names ----
        self.drone_id = drone_id_for(self.instance)
        gz_model = gz_model_name_for(self.instance)
        gz_topic = (
            f'/world/{GZ_WORLD_NAME}/model/{gz_model}'
            f'/link/camera_link/sensor/{GZ_SENSOR_NAME}/image'
        )
        ros_topic = f'/{self.drone_id}/camera/image'

        # ---- State ----
        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
        self.gz_frames_received = 0
        self.gz_frames_dropped = 0
        self.ros_frames_published = 0

        # ---- Publisher ----
        self.ros_pub = self.create_publisher(Image, ros_topic, 10)

        # ---- gz-transport subscriber ----
        self.gz_node = GzNode()
        ok = self.gz_node.subscribe(GzImage, gz_topic, self.gz_callback)
        if not ok:
            self.get_logger().error(f'Failed to subscribe to Gazebo topic: {gz_topic}')
            raise RuntimeError('gz-transport subscription failed')

        # ---- Timers ----
        self.create_timer(1.0 / publish_hz, self.publish_latest_frame)
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            f'[CAM BRIDGE] {self.drone_id} (instance={self.instance}): '
            f'{gz_topic} -> {ros_topic} at {publish_hz} Hz'
        )

    def gz_callback(self, gz_msg):
        self.gz_frames_received += 1
        try:
            self.frame_queue.put_nowait(gz_msg)
        except queue.Full:
            self.gz_frames_dropped += 1

    def publish_latest_frame(self):
        try:
            gz_msg = self.frame_queue.get_nowait()
        except queue.Empty:
            return

        ros_msg = Image()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = f'{self.drone_id}_camera_link'
        ros_msg.height = gz_msg.height
        ros_msg.width = gz_msg.width
        ros_msg.encoding = 'rgb8'
        ros_msg.is_bigendian = 0
        ros_msg.step = gz_msg.step
        ros_msg.data = gz_msg.data

        self.ros_pub.publish(ros_msg)
        self.ros_frames_published += 1

    def log_stats(self):
        self.get_logger().info(
            f'[CAM BRIDGE STATS] {self.drone_id}: '
            f'gz_received={self.gz_frames_received}, '
            f'gz_dropped={self.gz_frames_dropped}, '
            f'ros_published={self.ros_frames_published}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
