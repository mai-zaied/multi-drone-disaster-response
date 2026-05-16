"""
fog_server.py

Fog node — scalable to N drones, with end-of-mission cloud archival.

For each drone in [0, num_drones), the fog subscribes to:
  - The drone's PX4 VehicleStatus topic
  - The drone's fog-tier Task topic   (/{drone_id}/task/fog)
  - The drone's camera topic          (/{drone_id}/camera/image)

And publishes a Task-2-style decision per drone on:
  - /fog/{drone_id}/decision

Task 3.8 additions:
- Maintains an in-memory event buffer (bounded, drops oldest on overflow).
- Records each task arrival and each priority-3 alert as an event.
- Exposes a /fog/end_mission service. When called, flushes the buffer
  to the cloud as chunked batches on /fog/cloud/mission_log.
- During the mission the cloud topic carries zero traffic, keeping all
  fog compute focused on real-time work.
"""

import time
import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleStatus
from task_msgs.msg import Task

from fog_node.drone_naming import (
    drone_id_for,
    px4_topic_for,
)


# ----------------------------------------------------------------------
# Event buffer limits
# ----------------------------------------------------------------------
EVENT_BUFFER_SOFT_CAP = 10000   # drop oldest beyond this
BATCH_CHUNK_SIZE = 1000          # split flush into chunks of this size


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

        # ---- Per-drone state ----
        self.decision_publishers = {}
        self.stats = {}

        for instance in range(self.num_drones):
            drone_id = drone_id_for(instance)
            status_topic = px4_topic_for(instance, 'vehicle_status_v1')
            task_topic = f'/{drone_id}/task/fog'
            camera_topic = f'/{drone_id}/camera/image'

            self.create_subscription(
                VehicleStatus, status_topic,
                lambda msg, d=drone_id: self.status_callback(msg, d),
                px4_qos,
            )
            self.create_subscription(
                Task, task_topic,
                lambda msg, d=drone_id: self.task_callback(msg, d),
                10,
            )
            self.create_subscription(
                Image, camera_topic,
                lambda msg, d=drone_id: self.camera_callback(msg, d),
                1,
            )

            decision_topic = f'/fog/{drone_id}/decision'
            self.decision_publishers[drone_id] = self.create_publisher(
                String, decision_topic, 10
            )
            self.stats[drone_id] = {'status': 0, 'tasks': 0, 'frames': 0}

            self.get_logger().info(
                f'[FOG] {drone_id} (instance={instance}): '
                f'status={status_topic}, task={task_topic}, camera={camera_topic}'
            )

        # ---- Cloud archival infrastructure ----
        # bounded buffer of event dicts; oldest are evicted automatically
        self.event_buffer = deque(maxlen=EVENT_BUFFER_SOFT_CAP)
        self.events_dropped_on_overflow = 0
        self._prev_buffer_len = 0

        self.cloud_pub = self.create_publisher(
            String, '/fog/cloud/mission_log', 10
        )

        # end-of-mission service
        self.end_mission_srv = self.create_service(
            Trigger, '/fog/end_mission', self.end_mission_callback
        )

        self.get_logger().info(
            f'[FOG] tracking {self.num_drones} drone(s)'
        )
        self.get_logger().info(
            '[FOG] cloud archival: buffer accumulates during mission, '
            'flushes via /fog/end_mission service'
        )

        # Periodic stats
        self.create_timer(5.0, self.log_stats)

    # ------------------------------------------------------------------
    # Event buffering helper
    # ------------------------------------------------------------------
    def _record_event(self, event_type: str, drone_id: str, payload: dict):
        """Append an event to the bounded buffer. Track silent overflow."""
        before_len = len(self.event_buffer)
        if before_len >= EVENT_BUFFER_SOFT_CAP:
            self.events_dropped_on_overflow += 1
        event = {
            'event_type': event_type,
            'drone_id': drone_id,
            'fog_received_at': time.time(),
            'payload': payload,
        }
        self.event_buffer.append(event)

    # ------------------------------------------------------------------
    # PX4 status path (Task 2 style decision)
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

        time.sleep(0.05)  # short simulated processing — Task 3.7 will async-ify
        self.decision_publishers[drone_id].publish(decision)

    # ------------------------------------------------------------------
    # Task control plane (records every arrival as a mission-log event)
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

        # Record event for future cloud archival
        self._record_event(
            event_type='TASK_RECEIVED',
            drone_id=drone_id,
            payload={
                'task_id': msg.task_id,
                'task_type': msg.task_type,
                'priority': int(msg.priority),
                'latency_ms': round(latency_ms, 2),
                'task_timestamp_sec': int(msg.timestamp.sec),
                'task_timestamp_nsec': int(msg.timestamp.nanosec),
                'payload_keys': list(payload.keys()),
            },
        )

        if msg.priority == 3:
            self.get_logger().warn(
                f'[FOG TASK CRITICAL] {drone_id} {msg.task_id} '
                f'type={msg.task_type} PRIORITY=3 latency={latency_ms:.1f}ms '
                f'failing={payload.get("drone_failing", False)}'
            )
            # Also record a separate ALERT event so it's easy to find in archives
            self._record_event(
                event_type='PRIORITY3_ALERT',
                drone_id=drone_id,
                payload={
                    'task_id': msg.task_id,
                    'task_type': msg.task_type,
                    'drone_failing': payload.get('drone_failing', False),
                    'position': payload.get('position'),
                },
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
        # Task 4 places the detection model here.

    # ------------------------------------------------------------------
    # End-of-mission service: flush buffer to cloud as chunked batches
    # ------------------------------------------------------------------
    def end_mission_callback(self, request, response):
        events = list(self.event_buffer)
        total_events = len(events)

        if total_events == 0:
            response.success = True
            response.message = 'No events to archive.'
            self.get_logger().info(
                '[FOG END_MISSION] Buffer empty, nothing to flush.'
            )
            return response

        # Chunk the events
        chunks = [
            events[i:i + BATCH_CHUNK_SIZE]
            for i in range(0, total_events, BATCH_CHUNK_SIZE)
        ]
        total_batches = len(chunks)
        fog_timestamp = time.time()

        self.get_logger().info(
            f'[FOG END_MISSION] Flushing {total_events} events in '
            f'{total_batches} batch(es) to cloud.'
        )

        for idx, chunk in enumerate(chunks):
            batch = {
                'fog_timestamp': fog_timestamp,
                'batch_index': idx + 1,
                'total_batches': total_batches,
                'event_count': len(chunk),
                'events': chunk,
            }
            msg = String()
            msg.data = json.dumps(batch)
            self.cloud_pub.publish(msg)
            self.get_logger().info(
                f'[FOG END_MISSION] Published batch {idx + 1}/{total_batches} '
                f'({len(chunk)} events).'
            )

        # Clear the buffer after flush
        self.event_buffer.clear()
        dropped = self.events_dropped_on_overflow
        self.events_dropped_on_overflow = 0

        response.success = True
        response.message = (
            f'Flushed {total_events} events in {total_batches} batch(es). '
            f'Dropped during mission due to overflow: {dropped}.'
        )
        return response

    # ------------------------------------------------------------------
    def log_stats(self):
        parts = [
            f'{d}[s={c["status"]} t={c["tasks"]} f={c["frames"]}]'
            for d, c in self.stats.items()
        ]
        self.get_logger().info('[FOG STATS] ' + ' '.join(parts))

        # Periodic buffer status (only log when it changes meaningfully)
        cur_len = len(self.event_buffer)
        if cur_len != self._prev_buffer_len:
            self.get_logger().info(
                f'[FOG BUFFER] {cur_len} events buffered '
                f'(soft_cap={EVENT_BUFFER_SOFT_CAP}, '
                f'overflow_drops={self.events_dropped_on_overflow})'
            )
            self._prev_buffer_len = cur_len


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
