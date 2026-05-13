"""
fog_server.py

Fog node: receives data from the drone swarm and processes it locally.

In Task 2, the fog subscribed only to PX4 VehicleStatus and emitted simple
decisions back to each drone.

In Task 3 (this revision), the fog additionally subscribes to:
  - Per-drone Task topics  (control plane, /{drone_id}/task)
  - Per-drone camera topics (data plane,    /{drone_id}/camera/image)

For now the fog only logs what it receives. The actual offloading-decision
logic that uses task metadata to route work to drone/fog/cloud will be added
in Task 3.4.
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


class FogServer(Node):
    def __init__(self):
        super().__init__('fog_server')

        # ---- PX4 QoS: must match PX4's BEST_EFFORT + TRANSIENT_LOCAL ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # ---- Drone roster ----
        # Each drone has three associated topics:
        #   - PX4 VehicleStatus (existing from Task 2)
        #   - Task topic         (Task 3.3, new)
        #   - Camera topic       (Task 3.3, new)
        self.drones = {
            'drone0': {
                'status_topic': '/fmu/out/vehicle_status_v1',
                'task_topic':   '/drone0/task/fog',          
                'camera_topic': '/drone0/camera/image',
            },
            'drone1': {
                'status_topic': '/px4_1/fmu/out/vehicle_status_v1',
                'task_topic':   '/drone1/task/fog',          
                'camera_topic': '/drone1/camera/image',
            },
            'drone2': {
                'status_topic': '/px4_2/fmu/out/vehicle_status_v1',
                'task_topic':   '/drone2/task/fog',         
                'camera_topic': '/drone2/camera/image',
            },
        }

        self.decision_publishers = {}

        # ---- Per-drone counters for logging visibility ----
        self.stats = {
            d: {'status': 0, 'tasks': 0, 'frames': 0}
            for d in self.drones
        }

        for drone_id, topics in self.drones.items():
            # PX4 VehicleStatus subscriber (from Task 2)
            self.create_subscription(
                VehicleStatus,
                topics['status_topic'],
                lambda msg, d=drone_id: self.status_callback(msg, d),
                px4_qos
            )

            # Task subscriber (NEW in Task 3.3)
            self.create_subscription(
                Task,
                topics['task_topic'],
                lambda msg, d=drone_id: self.task_callback(msg, d),
                10
            )

            # Camera subscriber (NEW in Task 3.3)
            # Default RELIABLE QoS to match the bridge's publisher.
            # Depth 1 — we only need the latest frame; we drop older ones.
            self.create_subscription(
                Image,
                topics['camera_topic'],
                lambda msg, d=drone_id: self.camera_callback(msg, d),
                1
            )

            # Decision publisher (from Task 2)
            decision_topic = f'/fog/{drone_id}/decision'
            self.decision_publishers[drone_id] = self.create_publisher(
                String, decision_topic, 10
            )

            self.get_logger().info(
                f'[FOG] {drone_id}: status={topics["status_topic"]}, '
                f'task={topics["task_topic"]}, camera={topics["camera_topic"]}'
            )

        # Periodic summary log
        self.create_timer(5.0, self.log_stats)

    # ------------------------------------------------------------------
    # PX4 VehicleStatus -> simple Task 2 decision logic
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

        # Simulated fog processing delay (Task 2.9). We keep it short to
        # avoid blocking the executor — sleeping inside callbacks isn't
        # ideal but for a graduation demo it's acceptable.
        time.sleep(0.05)

        self.decision_publishers[drone_id].publish(decision)

    # ------------------------------------------------------------------
    # Task control plane
    # ------------------------------------------------------------------
    def task_callback(self, msg: Task, drone_id: str):
        self.stats[drone_id]['tasks'] += 1

        # Compute the delivery latency in milliseconds.
        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
        latency_ms = (now_ns - sent_ns) / 1e6

        # Best-effort payload parsing (payload is a JSON string)
        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except json.JSONDecodeError:
            payload = {'_parse_error': True, '_raw': msg.payload}

        self.get_logger().info(
            f'[FOG TASK] {drone_id} {msg.task_id} '
            f'type={msg.task_type} priority={msg.priority} '
            f'latency={latency_ms:.1f}ms payload_keys={list(payload.keys())}'
        )

    # ------------------------------------------------------------------
    # Camera data plane
    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image, drone_id: str):
        self.stats[drone_id]['frames'] += 1
        # No processing for now — just acknowledge receipt.
        # In Task 4 this is where the detection model will run.

    # ------------------------------------------------------------------
    # Periodic stats
    # ------------------------------------------------------------------
    def log_stats(self):
        parts = []
        for drone_id, c in self.stats.items():
            parts.append(
                f'{drone_id}[s={c["status"]} t={c["tasks"]} f={c["frames"]}]'
            )
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