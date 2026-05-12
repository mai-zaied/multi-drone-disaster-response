"""
drone_task_publisher.py

Drone-side task generator.

Emits two task types onto /{drone_id}/task:

1. STATUS_REPORT  — produced every 1 second from cached PX4 VehicleStatus.
                    Payload: PX4 nav_state, arming_state, failsafe flags.

2. VICTIM_DETECTION_REQUEST — produced once per received camera frame.
                              Payload: a reference to the frame on the
                              camera topic (sequence, timestamp, topic name,
                              dimensions). The actual image bytes travel
                              separately on /{drone_id}/camera/image.

Design rationale:
- Control plane (Task msgs) and data plane (Image msgs) are kept on
  separate topics. Task messages remain small and routable; image data
  stays on its dedicated topic where the fog (and later, detection
  models) can subscribe natively to sensor_msgs/Image.
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import Image
from task_msgs.msg import Task


# Map drone_id to the PX4 topic that drone publishes its status on.
DRONE_PX4_TOPIC = {
    'drone0': '/fmu/out/vehicle_status_v1',
    'drone1': '/px4_1/fmu/out/vehicle_status_v1',
    'drone2': '/px4_2/fmu/out/vehicle_status_v1',
}


class DroneTaskPublisher(Node):
    def __init__(self):
        super().__init__('drone_task_publisher')

        # drone_id chooses which PX4 topic to listen to and which Task topic to publish on.
        self.declare_parameter('drone_id', 'drone0')
        self.drone_id = self.get_parameter('drone_id').value

        if self.drone_id not in DRONE_PX4_TOPIC:
            raise ValueError(
                f"Unknown drone_id '{self.drone_id}'. "
                f"Allowed: {list(DRONE_PX4_TOPIC.keys())}"
            )

        px4_topic = DRONE_PX4_TOPIC[self.drone_id]
        task_topic = f'/{self.drone_id}/task'
        self.camera_topic = f'/{self.drone_id}/camera/image'

        # ---- PX4 subscriber (BEST_EFFORT + TRANSIENT_LOCAL, must match PX4) ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.latest_status = None
        self.create_subscription(
            VehicleStatus,
            px4_topic,
            self.status_callback,
            px4_qos
        )

        # ---- Camera subscriber (default RELIABLE QoS works for the bridge) ----
        # Depth 1 — we only care about the latest frame's metadata at any moment.
        self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            1
        )

        # ---- Task publisher ----
        self.task_pub = self.create_publisher(Task, task_topic, 10)

        # ---- Counters / sequences ----
        self.status_seq = 0
        self.detection_seq = 0
        self.camera_frame_seq = 0

        # ---- Periodic STATUS_REPORT timer ----
        self.create_timer(1.0, self.generate_status_task)

        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: PX4 status from {px4_topic}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: camera frames from {self.camera_topic}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: tasks published on {task_topic}'
        )

    # ------------------------------------------------------------------
    # PX4 path
    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus):
        """Cache the most recent PX4 status (called at PX4 rate)."""
        self.latest_status = msg

    def generate_status_task(self):
        """Emit one STATUS_REPORT task per second from cached PX4 state."""
        if self.latest_status is None:
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: no PX4 status yet, skipping'
            )
            return

        msg = self.latest_status
        payload = {
            'nav_state': int(msg.nav_state),
            'arming_state': int(msg.arming_state),
            'failsafe': bool(msg.failsafe),
            'pre_flight_checks_pass': bool(msg.pre_flight_checks_pass),
        }

        task = Task()
        task.task_id = f'{self.drone_id}-status-{self.status_seq:04d}'
        task.task_type = 'STATUS_REPORT'
        task.drone_id = self.drone_id
        task.timestamp = self.get_clock().now().to_msg()
        task.priority = 1  # normal priority for routine status
        task.payload = json.dumps(payload)

        self.task_pub.publish(task)
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} (STATUS_REPORT)'
        )
        self.status_seq += 1

    # ------------------------------------------------------------------
    # Camera path
    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        """Per-frame: emit a VICTIM_DETECTION_REQUEST task that references this frame."""
        self.camera_frame_seq += 1

        payload = {
            'frame_seq': self.camera_frame_seq,
            'frame_timestamp_sec': int(msg.header.stamp.sec),
            'frame_timestamp_nsec': int(msg.header.stamp.nanosec),
            'image_topic': self.camera_topic,
            'width': int(msg.width),
            'height': int(msg.height),
            'encoding': msg.encoding,
        }

        task = Task()
        task.task_id = f'{self.drone_id}-detect-{self.detection_seq:04d}'
        task.task_type = 'VICTIM_DETECTION_REQUEST'
        task.drone_id = self.drone_id
        task.timestamp = self.get_clock().now().to_msg()
        task.priority = 2  # higher than routine status
        task.payload = json.dumps(payload)

        self.task_pub.publish(task)
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
            f'(VICTIM_DETECTION_REQUEST, frame_seq={self.camera_frame_seq})'
        )
        self.detection_seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = DroneTaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()