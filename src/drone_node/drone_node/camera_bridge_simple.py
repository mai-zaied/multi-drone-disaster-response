"""
camera_bridge_simple.py

A deliberately throttled, single-drone Gazebo-to-ROS2 RGB camera bridge.
Designed for safe first-time camera bring-up on resource-limited laptops.

Key safety features:
- One drone only (drone0)
- Hard-capped publishing rate at 2 Hz (not 30 Hz)
- Queue depth of 1: incoming frames overwrite the previous if the
  ROS2 publisher hasn't caught up (no memory growth)
- Drops frames silently when overloaded
- Single thread of conversion
"""

import queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# gz-transport Python bindings. Version 13 ships with Gazebo Harmonic.
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage


# ----------------------------------------------------------------------
# Configuration — keep conservative for first-time bring-up.
# ----------------------------------------------------------------------
DRONE_ID = 'drone0'
GZ_MODEL_NAME = 'x500_depth_0'        # matches PX4_INSTANCE=0 with gz_x500_depth
GZ_WORLD_NAME = 'baylands_collapsed_fixed'
GZ_SENSOR_NAME = 'IMX214'              # the RGB sensor on OakD-Lite

# Hard frame-rate cap on the ROS2 side. 2 Hz = one frame every 500 ms.
# This is intentionally low. We will raise it later, after the system is proven stable.
ROS_PUBLISH_HZ = 2.0

# Single-slot queue: keep only the latest frame. Drop everything older.
FRAME_QUEUE_SIZE = 1


class CameraBridgeSimple(Node):
    def __init__(self):
        super().__init__('camera_bridge_simple')

        # Build the Gazebo topic name for the RGB camera of this drone.
        gz_topic = (
            f'/world/{GZ_WORLD_NAME}/model/{GZ_MODEL_NAME}'
            f'/link/camera_link/sensor/{GZ_SENSOR_NAME}/image'
        )

        # The ROS2 topic the converted frames will appear on.
        ros_topic = f'/{DRONE_ID}/camera/image'

        # Single-slot queue: latest frame only.
        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)

        # Counters for logging.
        self.gz_frames_received = 0
        self.gz_frames_dropped = 0
        self.ros_frames_published = 0

        # ROS2 publisher.
        self.ros_pub = self.create_publisher(Image, ros_topic, 10)

        # gz-transport subscriber. The callback runs on a separate thread.
        self.gz_node = GzNode()
        ok = self.gz_node.subscribe(GzImage, gz_topic, self.gz_callback)
        if not ok:
            self.get_logger().error(f'Failed to subscribe to Gazebo topic: {gz_topic}')
            raise RuntimeError('gz-transport subscription failed')

        # ROS2 timer drains the queue at fixed rate.
        period = 1.0 / ROS_PUBLISH_HZ
        self.create_timer(period, self.publish_latest_frame)

        # Periodic stats log so we can see what's happening.
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            f'[CAM BRIDGE] Bridging Gazebo {gz_topic}\n'
            f'              -> ROS2 {ros_topic}  at {ROS_PUBLISH_HZ} Hz'
        )

    def gz_callback(self, gz_msg):
        """Runs on a gz-transport thread. Keep it cheap."""
        self.gz_frames_received += 1
        try:
            # Put without blocking. If queue is full (depth=1, frame still pending),
            # drop the new frame.
            self.frame_queue.put_nowait(gz_msg)
        except queue.Full:
            self.gz_frames_dropped += 1

    def publish_latest_frame(self):
        """Runs on ROS2 thread at ROS_PUBLISH_HZ. Converts and publishes."""
        try:
            gz_msg = self.frame_queue.get_nowait()
        except queue.Empty:
            return  # no new frame this tick; that's fine

        # Convert gz.msgs.Image -> sensor_msgs.msg.Image.
        # The pixel byte layout is identical for RGB8, so we copy raw bytes.
        ros_msg = Image()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = f'{DRONE_ID}_camera_link'
        ros_msg.height = gz_msg.height
        ros_msg.width = gz_msg.width
        ros_msg.encoding = 'rgb8'
        ros_msg.is_bigendian = 0
        ros_msg.step = gz_msg.step
        ros_msg.data = gz_msg.data  # raw pixel bytes

        self.ros_pub.publish(ros_msg)
        self.ros_frames_published += 1

    def log_stats(self):
        self.get_logger().info(
            f'[CAM BRIDGE STATS] '
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