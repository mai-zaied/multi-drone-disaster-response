"""
cloud_server.py

Simulated cloud archival node.

In this project the cloud is strictly an archival destination. It receives
mission logs (and later, detection records and metrics) from the fog node
AFTER the mission ends. The cloud:

- Applies a randomised simulated WAN delay per received batch (500 ms - 5 s
  by default), implemented non-blocking via ROS2 one-shot timers so the node
  can handle multiple in-flight batches concurrently.
- Writes each archived batch to disk as a JSON file in a session directory.
- Logs each receipt, each scheduled archival, and each completed archival.

Topics subscribed:
- /fog/cloud/mission_log   (std_msgs/String, JSON-encoded batch)

Files written:
- <archive_dir>/batch_<NNNNNN>_<recv_ts>.json   per received batch

Parameters:
- archive_dir       (string, default '/tmp/cloud_archive_<startup_ts>')
- delay_min_sec     (double, default 0.5)
- delay_max_sec     (double, default 5.0)
"""

import json
import os
import random
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CloudServer(Node):
    def __init__(self):
        super().__init__('cloud_server')

        # ---- Parameters ----
        startup_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_archive_dir = f'/tmp/cloud_archive_{startup_ts}'
        self.declare_parameter('archive_dir', default_archive_dir)
        self.declare_parameter('delay_min_sec', 0.5)
        self.declare_parameter('delay_max_sec', 5.0)

        self.archive_dir = str(self.get_parameter('archive_dir').value)
        self.delay_min_sec = float(self.get_parameter('delay_min_sec').value)
        self.delay_max_sec = float(self.get_parameter('delay_max_sec').value)

        if self.delay_min_sec < 0 or self.delay_max_sec < self.delay_min_sec:
            raise ValueError(
                f'Invalid delay range: [{self.delay_min_sec}, {self.delay_max_sec}]'
            )

        os.makedirs(self.archive_dir, exist_ok=True)

        # ---- State ----
        self.batches_received = 0
        self.batches_archived = 0
        self.events_archived = 0
        # Holds (timer, batch_metadata) pairs so we can keep timers alive.
        self._pending_timers = {}

        # ---- Subscribers ----
        # std_msgs/String JSON envelope, default reliable QoS (must not drop).
        self.create_subscription(
            String,
            '/fog/cloud/mission_log',
            self.mission_log_callback,
            10,
        )

        # ---- Periodic stats ----
        self.create_timer(10.0, self.log_stats)

        self.get_logger().info(
            f'[CLOUD] Started. archive_dir={self.archive_dir}, '
            f'delay_range=[{self.delay_min_sec:.2f}s, {self.delay_max_sec:.2f}s]'
        )
        self.get_logger().info('[CLOUD] Subscribed to /fog/cloud/mission_log')

    # ------------------------------------------------------------------
    def mission_log_callback(self, msg: String):
        """Fog has sent a mission-log batch. Schedule its archival with delay."""
        recv_time = time.time()
        self.batches_received += 1

        try:
            batch = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'[CLOUD] Malformed batch JSON: {e}')
            return

        events = batch.get('events', [])
        batch_index = batch.get('batch_index', -1)
        total_batches = batch.get('total_batches', -1)

        # Pick a random WAN delay for this batch.
        delay_sec = random.uniform(self.delay_min_sec, self.delay_max_sec)

        self.get_logger().info(
            f'[CLOUD] Received batch {batch_index}/{total_batches} '
            f'({len(events)} events). Scheduling archival in {delay_sec:.2f}s.'
        )

        # Build a closure capturing the batch and recv time.
        timer_key = id(batch)  # stable handle to remove timer later

        def deferred_archival():
            self._archive_batch(batch, recv_time, delay_sec)
            # Free the timer reference so it can be GC'd.
            t = self._pending_timers.pop(timer_key, None)
            if t is not None:
                t.cancel()

        # Non-blocking delay via one-shot timer.
        timer = self.create_timer(delay_sec, deferred_archival)
        self._pending_timers[timer_key] = timer

    # ------------------------------------------------------------------
    def _archive_batch(self, batch: dict, recv_time: float, delay_sec: float):
        """Write a batch to disk and update counters."""
        batch_index = batch.get('batch_index', -1)
        total_batches = batch.get('total_batches', -1)
        events = batch.get('events', [])

        archive_record = {
            'batch_index': batch_index,
            'total_batches': total_batches,
            'fog_timestamp': batch.get('fog_timestamp'),
            'cloud_received_at': recv_time,
            'cloud_archived_at': time.time(),
            'simulated_wan_delay_sec': round(delay_sec, 3),
            'event_count': len(events),
            'events': events,
        }

        filename = (
            f'batch_{self.batches_archived:06d}_'
            f'recv{int(recv_time*1000)}.json'
        )
        path = os.path.join(self.archive_dir, filename)

        try:
            with open(path, 'w') as f:
                json.dump(archive_record, f, indent=2)
        except OSError as e:
            self.get_logger().error(f'[CLOUD] Failed to write {path}: {e}')
            return

        self.batches_archived += 1
        self.events_archived += len(events)

        self.get_logger().info(
            f'[CLOUD ARCHIVED] batch {batch_index}/{total_batches} '
            f'({len(events)} events) after {delay_sec:.2f}s delay -> {path}'
        )

    # ------------------------------------------------------------------
    def log_stats(self):
        self.get_logger().info(
            f'[CLOUD STATS] received={self.batches_received} '
            f'archived={self.batches_archived} '
            f'events_archived={self.events_archived} '
            f'pending={len(self._pending_timers)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloudServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
