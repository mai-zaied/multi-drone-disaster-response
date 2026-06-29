#!/usr/bin/env python3
"""
battery_simulator.py — activity-coupled battery / energy model.

Drain is driven by the drone's REAL flight state (from /{drone}/mission_feedback)
plus a PROCESSING SURCHARGE that depends on what compute the drone is doing:

    LOCAL detection  -> the drone runs YOLO itself      -> ai_surcharge   (high)
    CLOUD offload     -> the drone only uploads frames   -> upload_surcharge (low)
    FOG offload       -> the drone does neither          -> no surcharge

This is what makes the Task 6 energy comparison meaningful: in LOCAL mode the drone
pays for on-board AI; in FOG mode it offloads that cost; CLOUD sits in between
(upload cost, no inference). The surcharge is triggered by keywords on
/{drone}/task_status, published by victim_detector (local) / cloud_detector (cloud).

Output line format is UNCHANGED so metrics_collector still parses `battery=NN.NN`:
    droneX: battery=NN.NN% | state=SCANNING+AI | drain=0.18%/1.0s
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


AVIONICS_BASE_RATE = 0.02   # always-present avionics draw (%/s)

FLIGHT_RATE = {             # added on top, by flight state (%/s)
    'IDLE': 0.00,
    'EN_ROUTE': 0.12,       # CLIMB + TRANSIT + DESCEND (highest)
    'SCANNING': 0.10,       # lawnmower sweep
    'ARRIVED': 0.07,        # holding over a target
    'HOLDING': 0.07,
    'RETURNING': 0.11,      # RTL
    'FAILED': 0.07,
}
DEFAULT_FLIGHT_RATE = 0.05

AI_SURCHARGE_RATE = 0.06        # on-board YOLO (LOCAL mode)
UPLOAD_SURCHARGE_RATE = 0.025   # frame upload to cloud (CLOUD mode)

AI_KEYWORDS = ('DETECTION', 'INFERENCE', 'PROCESSING', 'AI', 'LOCAL_AI')
UPLOAD_KEYWORDS = ('CLOUD', 'UPLOAD', 'OFFLOAD_CLOUD')


def flight_drain_rate(state):
    """Pure: commander flight state -> flight drain (%/s)."""
    return FLIGHT_RATE.get(str(state).upper(), DEFAULT_FLIGHT_RATE)


def classify_surcharge(task_text, ai_rate=AI_SURCHARGE_RATE,
                       upload_rate=UPLOAD_SURCHARGE_RATE):
    """Pure: map a task_status string -> (label, surcharge_rate) or (None, 0.0).
    AI keywords win over upload if both appear."""
    t = str(task_text).upper()
    if any(k in t for k in AI_KEYWORDS):
        return 'AI', ai_rate
    if any(k in t for k in UPLOAD_KEYWORDS):
        return 'UP', upload_rate
    return None, 0.0


def total_drain_per_tick(state, surcharge_rate, period,
                         avionics=AVIONICS_BASE_RATE):
    """Pure: percent of pack consumed in one `period`-second tick."""
    return (avionics + flight_drain_rate(state) + surcharge_rate) * period


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('initial_battery', 100.0)
        self.declare_parameter('period', 1.0)
        self.declare_parameter('low_threshold', 30.0)
        self.declare_parameter('critical_threshold', 10.0)
        self.declare_parameter('avionics_base_rate', AVIONICS_BASE_RATE)
        self.declare_parameter('ai_surcharge_rate', AI_SURCHARGE_RATE)
        self.declare_parameter('upload_surcharge_rate', UPLOAD_SURCHARGE_RATE)
        self.declare_parameter('processing_timeout', 4.0)

        self.drone_id = self.get_parameter('drone_id').value
        self.battery = float(self.get_parameter('initial_battery').value)
        self.period = float(self.get_parameter('period').value)
        self.low_threshold = float(self.get_parameter('low_threshold').value)
        self.critical_threshold = float(self.get_parameter('critical_threshold').value)
        self.avionics_base = float(self.get_parameter('avionics_base_rate').value)
        self.ai_rate = float(self.get_parameter('ai_surcharge_rate').value)
        self.upload_rate = float(self.get_parameter('upload_surcharge_rate').value)
        self.processing_timeout = float(self.get_parameter('processing_timeout').value)

        self.state = 'IDLE'
        self._surcharge_rate = 0.0
        self._surcharge_label = None
        self._surcharge_until = 0.0

        self.low_sent = False
        self.critical_sent = False
        self.dead_sent = False

        self.battery_pub = self.create_publisher(
            String, f'/{self.drone_id}/battery_status', 10)
        self.decision_pub = self.create_publisher(String, '/decision/status', 10)

        self.feedback_sub = self.create_subscription(
            String, f'/{self.drone_id}/mission_feedback',
            self.feedback_callback, 10)
        self.task_sub = self.create_subscription(
            String, f'/{self.drone_id}/task_status',
            self.task_callback, 10)

        self.timer = self.create_timer(self.period, self.update_battery)

        self.get_logger().info(
            f'[BATTERY SIM] {self.drone_id} | initial={self.battery:.1f}% '
            f'| ai+={self.ai_rate}%/s upload+={self.upload_rate}%/s '
            f'| driven by /{self.drone_id}/mission_feedback')

    # ------------------------------------------------------------------
    def feedback_callback(self, msg):
        state = None
        try:
            fb = json.loads(msg.data)
            state = fb.get('state')
        except Exception:
            state = msg.data.strip() or None
        if state:
            new_state = str(state).upper()
            if new_state != self.state:
                self.get_logger().info(
                    f'[BATTERY STATE] {self.drone_id} {self.state} -> {new_state}')
            self.state = new_state

    def task_callback(self, msg):
        label, rate = classify_surcharge(msg.data, self.ai_rate, self.upload_rate)
        if label is not None:
            self._surcharge_label = label
            self._surcharge_rate = rate
            self._surcharge_until = time.time() + self.processing_timeout

    # ------------------------------------------------------------------
    def update_battery(self):
        if self.battery <= 0:
            return

        if time.time() < self._surcharge_until:
            surcharge = self._surcharge_rate
            tag = '+' + (self._surcharge_label or 'PROC')
        else:
            surcharge = 0.0
            tag = ''

        drain = total_drain_per_tick(self.state, surcharge, self.period,
                                     avionics=self.avionics_base)
        self.battery = max(0.0, self.battery - drain)

        status = String()
        status.data = (
            f'{self.drone_id}: battery={self.battery:.2f}% | '
            f'state={self.state}{tag} | drain={drain:.2f}%/{self.period:.1f}s')
        self.battery_pub.publish(status)
        self.get_logger().info(f'[BATTERY] {status.data}')

        if self.battery <= self.low_threshold and not self.low_sent:
            self.low_sent = True
            self._publish_decision_event('LOW_BATTERY')
        if self.battery <= self.critical_threshold and not self.critical_sent:
            self.critical_sent = True
            self._publish_decision_event('DRONE_FAILING_RETURN_HOME')
        if self.battery <= 0 and not self.dead_sent:
            self.dead_sent = True
            self._publish_decision_event('BATTERY_DEAD')

    def _publish_decision_event(self, event_type):
        msg = String()
        msg.data = (
            f'{self.drone_id}: {event_type} | battery={self.battery:.2f}% | '
            f'state={self.state} | timestamp={time.time()}')
        self.decision_pub.publish(msg)
        self.get_logger().warn(f'[BATTERY EVENT] {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
