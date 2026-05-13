"""
fog_server.py

Fog node — scalable to N drones via the 'num_drones' parameter.

For each drone in [0, num_drones), the fog subscribes to:
  - The drone's PX4 VehicleStatus topic
  - The drone's fog-tier Task topic   (/{drone_id}/task/fog)
  - The drone's camera topic          (/{drone_id}/camera/image)

And it publishes a Task-2-style decision per drone on:
  - /fog/{drone_id}/decision

The 'num_drones' parameter defaults to 3 for backward compatibility.
Pass -p num_drones:=N to support more.
"""

import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleStatus
from task_msgs.msg import Task

from fog_node.drone_naming import (
    drone_id_for,
    px4_topic_for,
)


class FogServer(Node):
    def __init__(self):
        super().__init__('fog_server')

        # ---- Parameters ----
        self.declare_parameter('num_drones', 3)
        self.num_drones = int(self.get_parameter('num_drones').value)
        if self.num_drones < 1:
            raise ValueError(f'num_drones must be >= 1, got {self.num_drones}')

        # ---- PX4 QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ---- Build per-drone subscriptions dynamically ----
        self.decision_publishers = {}
        self.stats = {}

        for instance in range(self.num_drones):
            drone_id = drone_id_for(instance)
            status_topic = px4_topic_for(instance, 'vehicle_status_v1')
            task_topic = f'/{drone_id}/task/fog'
            camera_topic = f'/{drone_id}/camera/image'

            # PX4 status subscriber
            self.create_subscription(
                VehicleStatus, status_topic,
                lambda msg, d=drone_id: self.status_callback(msg, d),
                px4_qos,
            )

            # Task subscriber (fog-tier only)
            self.create_subscription(
                Task, task_topic,
                lambda msg, d=drone_id: self.task_callback(msg, d),
                10,
            )

            # Camera subscriber
            self.create_subscription(
                Image, camera_topic,
                lambda msg, d=drone_id: self.camera_callback(msg, d),
                1,
            )

            # Decision publisher (Task-2-style commands back to drone)
            decision_topic = f'/fog/{drone_id}/decision'
            self.decision_publishers[drone_id] = self.create_publisher(
                String, decision_topic, 10
            )

            # Stats
            self.stats[drone_id] = {'status': 0, 'tasks': 0, 'frames': 0}

            self.get_logger().info(
                f'[FOG] {drone_id} (instance={instance}): '
                f'status={status_topic}, task={task_topic}, camera={camera_topic}'
            )

        self.get_logger().info(
            f'[FOG] tracking {self.num_drones} drone(s)'
        )

        # Periodic stats
        self.create_timer(5.0, self.log_stats)

    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus, drone_id: str):
        self.stats[drone_id]['status'] += 1

        decision = String()
        if msg.arming_state == 2:
            decision.data = f'{drone_id}: COMMAND_MONITOR (ARMED)'
            self.get_logger().warn(f'[FOG ALERT] {drone_id} is ARMED')
        elif msg.nav_state == 4:
            decision.data = f'{drone_id}: COMMAND_HOLD_POSITION'
        else:
            decision.data = f'{drone_id}: COMMAND_NORMAL_OPERATION'

        time.sleep(0.05)  # short simulated processing — Task 3.7 makes this non-blocking
        self.decision_publishers[drone_id].publish(decision)

    # ------------------------------------------------------------------
    def task_callback(self, msg: Task, drone_id: str):
        self.stats[drone_id]['tasks'] += 1

        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
        latency_ms = (now_ns - sent_ns) / 1e6

        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except json.JSONDecodeError:
            payload = {'_parse_error': True}

        # Highlight dying-drone signals
        if msg.priority == 3:
            self.get_logger().warn(
                f'[FOG TASK CRITICAL] {drone_id} {msg.task_id} '
                f'type={msg.task_type} PRIORITY=3 latency={latency_ms:.1f}ms '
                f'failing={payload.get("drone_failing", False)}'
            )
        else:
            self.get_logger().info(
                f'[FOG TASK] {drone_id} {msg.task_id} '
                f'type={msg.task_type} priority={msg.priority} '
                f'latency={latency_ms:.1f}ms payload_keys={list(payload.keys())}'
            )

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image, drone_id: str):
        self.stats[drone_id]['frames'] += 1
        # Task 4 will run the detection model here.

    # ------------------------------------------------------------------
    def log_stats(self):
        parts = [
            f'{d}[s={c["status"]} t={c["tasks"]} f={c["frames"]}]'
            for d, c in self.stats.items()
        ]
        self.get_logger().info('[FOG STATS] ' + ' '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = FogServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
