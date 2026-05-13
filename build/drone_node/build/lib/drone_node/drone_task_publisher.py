"""
drone_task_publisher.py

Drone-side task generator with onboard intelligence.

This node:
1. Filters camera frames locally (brightness / inter-frame diff / blur).
2. Attaches the drone's PX4 local position to every task payload.
3. Decides which tier processes each task (local / fog / cloud) and
   publishes on tier-specific topics.
4. Supports a 'simulate_low_battery' flag that escalates STATUS_REPORT
   priority to 3 and includes a drone_failing flag in the payload.

Parameterised by 'instance' (integer PX4 instance index). All derived
names (drone_id, PX4 topic names, camera topic, tier topics) are
generated from this single number using drone_naming.py.
"""

import json

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from sensor_msgs.msg import Image
from task_msgs.msg import Task

from drone_node.drone_naming import (
    drone_id_for,
    px4_topic_for,
)


# ----------------------------------------------------------------------
# Frame filter thresholds
# ----------------------------------------------------------------------
BRIGHTNESS_MIN = 20.0
BRIGHTNESS_MAX = 240.0
DIFF_MIN = 0.1        # tuned for stationary-drone testing in Gazebo
BLUR_MIN = 100.0


# ----------------------------------------------------------------------
# Rates
# ----------------------------------------------------------------------
STATUS_PERIOD_SEC = 5.0    # 1 status report every 5 s (0.2 Hz)
FILTER_STATS_PERIOD_SEC = 10.0


# ----------------------------------------------------------------------
# Offloading decision module
# ----------------------------------------------------------------------
DEFAULT_TARGET = {
    'STATUS_REPORT':            'fog',
    'VICTIM_DETECTION_REQUEST': 'fog',
    'BATTERY_CHECK':            'local',
    'MISSION_LOG_UPLOAD':       'cloud',
    'DETECTION_RECORD_ARCHIVAL':'cloud',
    'METRICS_REPORT':           'cloud',
}


def decide_target(task_type: str, priority: int, drone_failing: bool) -> str:
    """Pure decision function. See README for routing table and overrides."""
    target = DEFAULT_TARGET.get(task_type, 'fog')
    if drone_failing and task_type == 'STATUS_REPORT':
        target = 'fog'
    return target


# ----------------------------------------------------------------------
# Main node
# ----------------------------------------------------------------------
class DroneTaskPublisher(Node):
    def __init__(self):
        super().__init__('drone_task_publisher')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.declare_parameter('simulate_low_battery', False)
        self.instance = int(self.get_parameter('instance').value)
        self.simulate_low_battery = bool(self.get_parameter('simulate_low_battery').value)

        # ---- Derived names (single source: drone_naming.py) ----
        self.drone_id = drone_id_for(self.instance)
        px4_status_topic = px4_topic_for(self.instance, 'vehicle_status_v1')
        px4_position_topic = px4_topic_for(self.instance, 'vehicle_local_position_v1')
        self.camera_topic = f'/{self.drone_id}/camera/image'

        self.task_topics = {
            'local': f'/{self.drone_id}/task/local',
            'fog':   f'/{self.drone_id}/task/fog',
            'cloud': f'/{self.drone_id}/task/cloud',
        }

        # ---- PX4 QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ---- State caches ----
        self.latest_status: VehicleStatus = None
        self.latest_position: VehicleLocalPosition = None
        self.prev_gray_frame: np.ndarray = None

        # ---- Subscribers ----
        self.create_subscription(VehicleStatus, px4_status_topic,
                                 self.status_callback, px4_qos)
        self.create_subscription(VehicleLocalPosition, px4_position_topic,
                                 self.position_callback, px4_qos)
        self.create_subscription(Image, self.camera_topic,
                                 self.camera_callback, 1)

        # ---- Publishers ----
        self.task_pubs = {
            tier: self.create_publisher(Task, topic, 10)
            for tier, topic in self.task_topics.items()
        }

        # ---- Counters ----
        self.status_seq = 0
        self.detection_seq = 0
        self.camera_frame_seq = 0
        self.filter_stats = {
            'in': 0, 'passed': 0,
            'drop_dark': 0, 'drop_bright': 0,
            'drop_static': 0, 'drop_blur': 0,
        }

        # ---- Timers ----
        self.create_timer(STATUS_PERIOD_SEC, self.generate_status_task)
        self.create_timer(FILTER_STATS_PERIOD_SEC, self.log_filter_stats)

        # ---- Startup logs ----
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id} (instance={self.instance}): '
            f'simulate_low_battery={self.simulate_low_battery}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: PX4 status from {px4_status_topic}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: PX4 position from {px4_position_topic}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: camera frames from {self.camera_topic}'
        )
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: status rate = 1/{STATUS_PERIOD_SEC:.0f}s'
        )
        for tier, topic in self.task_topics.items():
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: tier "{tier}" -> {topic}'
            )

    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus):
        self.latest_status = msg

    def position_callback(self, msg: VehicleLocalPosition):
        self.latest_position = msg

    # ------------------------------------------------------------------
    def generate_status_task(self):
        if self.latest_status is None:
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: no PX4 status yet, skipping'
            )
            return

        msg = self.latest_status
        drone_failing = self.simulate_low_battery
        priority = 3 if drone_failing else 0

        payload = {
            'nav_state': int(msg.nav_state),
            'arming_state': int(msg.arming_state),
            'failsafe': bool(msg.failsafe),
            'pre_flight_checks_pass': bool(msg.pre_flight_checks_pass),
            'drone_failing': drone_failing,
            'position': self._build_position_dict(),
        }

        task = Task()
        task.task_id = f'{self.drone_id}-status-{self.status_seq:04d}'
        task.task_type = 'STATUS_REPORT'
        task.drone_id = self.drone_id
        task.timestamp = self.get_clock().now().to_msg()
        task.priority = priority
        task.payload = json.dumps(payload)

        target = decide_target(task.task_type, task.priority, drone_failing)
        self.task_pubs[target].publish(task)

        log = self.get_logger().warn if drone_failing else self.get_logger().info
        flag = ', DRONE_FAILING' if drone_failing else ''
        log(
            f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
            f'(STATUS_REPORT, priority={priority}{flag}) -> {target}'
        )
        self.status_seq += 1

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        self.camera_frame_seq += 1
        self.filter_stats['in'] += 1

        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
        except ValueError:
            return

        keep, scores, reason = self._filter_frame(arr)
        if not keep:
            self.filter_stats[f'drop_{reason}'] += 1
            return

        self.filter_stats['passed'] += 1

        payload = {
            'frame_seq': self.camera_frame_seq,
            'frame_timestamp_sec': int(msg.header.stamp.sec),
            'frame_timestamp_nsec': int(msg.header.stamp.nanosec),
            'image_topic': self.camera_topic,
            'width': int(msg.width),
            'height': int(msg.height),
            'encoding': msg.encoding,
            'filter_scores': scores,
            'position': self._build_position_dict(),
        }

        task = Task()
        task.task_id = f'{self.drone_id}-detect-{self.detection_seq:04d}'
        task.task_type = 'VICTIM_DETECTION_REQUEST'
        task.drone_id = self.drone_id
        task.timestamp = self.get_clock().now().to_msg()
        task.priority = 0
        task.payload = json.dumps(payload)

        target = decide_target(task.task_type, task.priority, False)
        self.task_pubs[target].publish(task)

        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
            f'(VICTIM_DETECTION_REQUEST, frame_seq={self.camera_frame_seq}) -> {target}'
        )
        self.detection_seq += 1

    # ------------------------------------------------------------------
    def _filter_frame(self, rgb_frame: np.ndarray):
        mean_brightness = float(rgb_frame.mean())
        if mean_brightness < BRIGHTNESS_MIN:
            return False, {'brightness': mean_brightness}, 'dark'
        if mean_brightness > BRIGHTNESS_MAX:
            return False, {'brightness': mean_brightness}, 'bright'

        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)

        diff_score = None
        if self.prev_gray_frame is not None and self.prev_gray_frame.shape == gray.shape:
            diff_score = float(np.abs(
                gray.astype(np.int16) - self.prev_gray_frame.astype(np.int16)
            ).mean())
            if diff_score < DIFF_MIN:
                self.prev_gray_frame = gray
                return False, {
                    'brightness': mean_brightness,
                    'diff': diff_score
                }, 'static'
        self.prev_gray_frame = gray

        blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if blur_score < BLUR_MIN:
            return False, {
                'brightness': mean_brightness,
                'diff': diff_score,
                'blur': blur_score,
            }, 'blur'

        return True, {
            'brightness': round(mean_brightness, 2),
            'diff': round(diff_score, 2) if diff_score is not None else None,
            'blur': round(blur_score, 2),
        }, ''

    # ------------------------------------------------------------------
    def _build_position_dict(self):
        if self.latest_position is None:
            return {'valid': False, 'x': None, 'y': None, 'z': None}
        p = self.latest_position
        valid = bool(p.xy_valid and p.z_valid)
        return {
            'valid': valid,
            'x': float(p.x),
            'y': float(p.y),
            'z': float(p.z),
        }

    # ------------------------------------------------------------------
    def log_filter_stats(self):
        s = self.filter_stats
        total_dropped = s['drop_dark'] + s['drop_bright'] + s['drop_static'] + s['drop_blur']
        pass_rate = (100.0 * s['passed'] / s['in']) if s['in'] else 0.0
        self.get_logger().info(
            f'[FILTER STATS] {self.drone_id}: '
            f"in={s['in']} passed={s['passed']} ({pass_rate:.1f}%) "
            f"dropped={total_dropped} "
            f"[dark={s['drop_dark']} bright={s['drop_bright']} "
            f"static={s['drop_static']} blur={s['drop_blur']}]"
        )


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
