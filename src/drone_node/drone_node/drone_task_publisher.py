"""
drone_task_publisher.py

Drone-side task generator with onboard intelligence.

This node does three things the previous Task 3.3 version did not:

1. FRAME FILTERING — cheap onboard checks (brightness, inter-frame
   difference, blur) drop uninteresting frames before they ever become
   tasks. This is the drone-side preprocessing layer described in
   Section 4.5 / Table 4.4 of the system design.

2. POSITION ATTACHMENT — every detection request carries the drone's
   local position at the time of capture, so the fog can build a
   victim map (Task 5).

3. OFFLOADING DECISION — a lookup-table-based decision function
   determines which tier (drone-local / fog / cloud) each task should
   go to. The result is encoded by publishing the task on a
   tier-specific topic:
     /{drone_id}/task/local
     /{drone_id}/task/fog
     /{drone_id}/task/cloud
   This is the ROS2-native "topic = routing" pattern; no field in the
   Task message is needed.

The decision module also supports a critical-battery override: when
the drone is about to die, status reports are emitted with priority=3
so the fog can reallocate other drones to cover the area (Task 5).
Real battery integration is deferred; a 'simulate_low_battery'
parameter is provided for demo purposes.
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


# ----------------------------------------------------------------------
# Per-drone topic configuration
# ----------------------------------------------------------------------
DRONE_PX4_STATUS_TOPIC = {
    'drone0': '/fmu/out/vehicle_status_v1',
    'drone1': '/px4_1/fmu/out/vehicle_status_v1',
    'drone2': '/px4_2/fmu/out/vehicle_status_v1',
}
DRONE_PX4_POSITION_TOPIC = {
    'drone0': '/fmu/out/vehicle_local_position_v1',
    'drone1': '/px4_1/fmu/out/vehicle_local_position_v1',
    'drone2': '/px4_2/fmu/out/vehicle_local_position_v1',
}


# ----------------------------------------------------------------------
# Frame filter thresholds
# ----------------------------------------------------------------------
# Tuned conservatively. We'd rather pass a marginal frame to fog than
# silently drop a real victim.
BRIGHTNESS_MIN = 20.0    # average pixel intensity below this -> too dark
BRIGHTNESS_MAX = 240.0   # above this -> washed out / sky
DIFF_MIN = 0           # mean absolute difference from previous frame    // i turned this to 0 for now for stationary testing, but it should be >0 to filter out static frames
BLUR_MIN = 100.0         # variance-of-Laplacian below this -> too blurry


# ----------------------------------------------------------------------
# Offloading decision module
# ----------------------------------------------------------------------
# Default routing by task type. STATUS_REPORT goes to FOG because the
# fog uses it for swarm coordination; the drone produces it locally
# but the fog needs to see it.
DEFAULT_TARGET = {
    'STATUS_REPORT':            'fog',
    'VICTIM_DETECTION_REQUEST': 'fog',
    'BATTERY_CHECK':            'local',
    'MISSION_LOG_UPLOAD':       'cloud',
    'DETECTION_RECORD_ARCHIVAL':'cloud',
    'METRICS_REPORT':           'cloud',
}


def decide_target(task_type: str, priority: int, drone_failing: bool) -> str:
    """
    Decide which tier processes this task.

    Rules:
    - Default routing is by task_type lookup.
    - If the drone is failing (priority=3), STATUS_REPORT MUST go to fog
      even if the default would route elsewhere -- fog needs to know
      immediately so it can reallocate other drones.
    - Unknown task types default to fog (safest fallback).
    """
    target = DEFAULT_TARGET.get(task_type, 'fog')

    if drone_failing and task_type == 'STATUS_REPORT':
        target = 'fog'  # always fog when dying, regardless of default

    return target


# ----------------------------------------------------------------------
# Main node
# ----------------------------------------------------------------------
class DroneTaskPublisher(Node):
    def __init__(self):
        super().__init__('drone_task_publisher')

        # ---- Parameters ----
        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('simulate_low_battery', False)
        self.drone_id = self.get_parameter('drone_id').value
        self.simulate_low_battery = self.get_parameter('simulate_low_battery').value

        if self.drone_id not in DRONE_PX4_STATUS_TOPIC:
            raise ValueError(
                f"Unknown drone_id '{self.drone_id}'. "
                f"Allowed: {list(DRONE_PX4_STATUS_TOPIC.keys())}"
            )

        # ---- Topic strings ----
        px4_status_topic   = DRONE_PX4_STATUS_TOPIC[self.drone_id]
        px4_position_topic = DRONE_PX4_POSITION_TOPIC[self.drone_id]
        self.camera_topic  = f'/{self.drone_id}/camera/image'

        # Three task topics, one per tier (ROS2-native routing).
        self.task_topics = {
            'local': f'/{self.drone_id}/task/local',
            'fog':   f'/{self.drone_id}/task/fog',
            'cloud': f'/{self.drone_id}/task/cloud',
        }

        # ---- PX4 QoS profile (must match BEST_EFFORT + TRANSIENT_LOCAL) ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
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

        # ---- Publishers (one per tier) ----
        self.task_pubs = {
            tier: self.create_publisher(Task, topic, 10)
            for tier, topic in self.task_topics.items()
        }

        # ---- Sequence counters ----
        self.status_seq = 0
        self.detection_seq = 0
        self.camera_frame_seq = 0

        # ---- Filter statistics ----
        self.filter_stats = {
            'in':            0,
            'passed':        0,
            'drop_dark':     0,
            'drop_bright':   0,
            'drop_static':   0,
            'drop_blur':     0,
        }

        # ---- Periodic timers ----
        self.create_timer(1.0, self.generate_status_task)
        self.create_timer(10.0, self.log_filter_stats)

        # ---- Startup logs ----
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: '
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
        for tier, topic in self.task_topics.items():
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: tasks for tier "{tier}" -> {topic}'
            )

    # ------------------------------------------------------------------
    # PX4 callbacks
    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus):
        self.latest_status = msg

    def position_callback(self, msg: VehicleLocalPosition):
        self.latest_position = msg

    # ------------------------------------------------------------------
    # Status task generator (1 Hz timer)
    # ------------------------------------------------------------------
    def generate_status_task(self):
        if self.latest_status is None:
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: no PX4 status yet, skipping'
            )
            return

        msg = self.latest_status

        # Determine priority and dying-flag based on simulated low-battery state.
        drone_failing = bool(self.simulate_low_battery)
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

        if drone_failing:
            self.get_logger().warn(
                f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
                f'(STATUS_REPORT, priority=3, DRONE_FAILING) -> {target}'
            )
        else:
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
                f'(STATUS_REPORT, priority={priority}) -> {target}'
            )

        self.status_seq += 1

    # ------------------------------------------------------------------
    # Camera callback: filter -> (maybe) generate detection task
    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        self.camera_frame_seq += 1
        self.filter_stats['in'] += 1

        # Reconstruct the RGB frame as numpy array. The image arrives as
        # raw bytes in row-major rgb8 order; reshape is essentially free
        # because it's a view, not a copy.
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
        except ValueError:
            # Frame size mismatch; drop and move on.
            return

        # Run the three filter checks.
        keep, scores, reason = self._filter_frame(arr)
        if not keep:
            self.filter_stats[f'drop_{reason}'] += 1
            return

        self.filter_stats['passed'] += 1

        # Build the detection task.
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
        task.priority = 0  # detections are normal priority unless escalated later
        task.payload = json.dumps(payload)

        target = decide_target(task.task_type, task.priority, False)
        self.task_pubs[target].publish(task)

        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: published {task.task_id} '
            f'(VICTIM_DETECTION_REQUEST, frame_seq={self.camera_frame_seq}) -> {target}'
        )
        self.detection_seq += 1

    # ------------------------------------------------------------------
    # Frame filter
    # ------------------------------------------------------------------
    def _filter_frame(self, rgb_frame: np.ndarray):
        """
        Return (keep: bool, scores: dict, reason: str).
        reason is one of: 'dark', 'bright', 'static', 'blur' when keep=False.
        """
        # Check 1: brightness (fastest, do first)
        mean_brightness = float(rgb_frame.mean())
        if mean_brightness < BRIGHTNESS_MIN:
            return False, {'brightness': mean_brightness}, 'dark'
        if mean_brightness > BRIGHTNESS_MAX:
            return False, {'brightness': mean_brightness}, 'bright'

        # Convert to grayscale once for the next two checks.
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)

        # Check 2: inter-frame difference
        diff_score = None
        if self.prev_gray_frame is not None and self.prev_gray_frame.shape == gray.shape:
            diff_score = float(np.abs(
                gray.astype(np.int16) - self.prev_gray_frame.astype(np.int16)
            ).mean())
            if diff_score < DIFF_MIN:
                # Update prev so we don't get stuck if scene barely changes.
                self.prev_gray_frame = gray
                return False, {
                    'brightness': mean_brightness,
                    'diff': diff_score
                }, 'static'
        self.prev_gray_frame = gray

        # Check 3: blur (variance of Laplacian)
        blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if blur_score < BLUR_MIN:
            return False, {
                'brightness': mean_brightness,
                'diff': diff_score,
                'blur': blur_score
            }, 'blur'

        # All checks passed.
        return True, {
            'brightness': round(mean_brightness, 2),
            'diff': round(diff_score, 2) if diff_score is not None else None,
            'blur': round(blur_score, 2),
        }, ''

    # ------------------------------------------------------------------
    # Position helper
    # ------------------------------------------------------------------
    def _build_position_dict(self):
        """Convert cached PX4 local position to a JSON-friendly dict."""
        if self.latest_position is None:
            return {'valid': False, 'x': None, 'y': None, 'z': None}

        p = self.latest_position
        # PX4's xy_valid / z_valid tell us if the EKF has converged.
        valid = bool(p.xy_valid and p.z_valid)
        return {
            'valid': valid,
            'x': float(p.x),
            'y': float(p.y),
            'z': float(p.z),
        }

    # ------------------------------------------------------------------
    # Periodic filter stats
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