"""
drone_task_publisher.py

Drone-side task generator with an ADAPTIVE offloading decision engine.

This node:
1. Filters camera frames locally (brightness / inter-frame diff / blur).
2. Attaches the drone's PX4 local position to every task payload.
3. Runs an offloading DECISION ENGINE that weighs four live signals —
   task urgency, onboard CPU load, battery level, and link quality (RSSI,
   modelled from drone<->fog distance) — and routes each task to the best
   tier (local / fog / cloud) using documented thresholds.
4. Orders outgoing tasks through a PRIORITY QUEUE so critical tasks
   (e.g. a failing drone's status) overtake routine ones under load.

The previous static lookup is kept as `decide_target_static` and can be
re-enabled with `enable_adaptive_offload:=false`, so the report can compare
static routing against the adaptive engine (an A/B baseline for Task 6).

Parameterised by 'instance' (integer PX4 instance index). All derived names
are generated from this single number using drone_naming.py.
"""

import json
import math
import os
import re
import heapq
import itertools

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from sensor_msgs.msg import Image
from std_msgs.msg import String
from task_msgs.msg import Task

from drone_node.drone_naming import drone_id_for, px4_topic_for


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
STATUS_PERIOD_SEC = 5.0       # 1 status report every 5 s (0.2 Hz)
FILTER_STATS_PERIOD_SEC = 10.0
CPU_SAMPLE_PERIOD_SEC = 1.0
QUEUE_DRAIN_HZ = 50.0         # priority-queue service rate

# ----------------------------------------------------------------------
# Default spawn positions (ENU East=x, North=y) — mirror drone_commander.
# Used only to convert PX4 local NED -> world ENU for the RSSI model.
# ----------------------------------------------------------------------
DEFAULT_SPAWNS = {0: (18.0, 25.0), 1: (23.0, 25.0), 2: (30.0, 25.0)}

# ----------------------------------------------------------------------
# RSSI / link model (log-distance path loss)
#   rssi(d) = RSSI_REF_DBM - 10*n*log10(max(1,d))     [dBm]
#   link_quality maps [RSSI_MIN_DBM .. RSSI_MAX_DBM] -> [0 .. 1]
# ----------------------------------------------------------------------
RSSI_REF_DBM = -40.0          # received power at 1 m
RSSI_MIN_DBM = -100.0         # unusable link
RSSI_MAX_DBM = -50.0          # strong link

# Static fallback routing (legacy behaviour; kept for A/B comparison)
DEFAULT_TARGET = {
    'STATUS_REPORT':            'fog',
    'VICTIM_DETECTION_REQUEST': 'fog',
    'VICTIM_DETECTION':         'fog',
    'BATTERY_CHECK':            'local',
    'MISSION_LOG_UPLOAD':       'cloud',
    'DETECTION_RECORD_ARCHIVAL': 'cloud',
    'METRICS_REPORT':           'cloud',
}
ARCHIVAL_TASKS = ('MISSION_LOG_UPLOAD', 'DETECTION_RECORD_ARCHIVAL', 'METRICS_REPORT')
HEAVY_TASKS = ('VICTIM_DETECTION_REQUEST', 'VICTIM_DETECTION')


def rssi_from_distance(dist_m: float, pathloss_exp: float) -> float:
    return RSSI_REF_DBM - 10.0 * pathloss_exp * math.log10(max(1.0, dist_m))


def link_quality_from_rssi(rssi_dbm: float) -> float:
    q = (rssi_dbm - RSSI_MIN_DBM) / (RSSI_MAX_DBM - RSSI_MIN_DBM)
    return max(0.0, min(1.0, q))


def decide_target_static(task_type: str, drone_failing: bool) -> str:
    """Legacy static lookup (kept for the static-vs-adaptive comparison)."""
    target = DEFAULT_TARGET.get(task_type, 'fog')
    if drone_failing and task_type == 'STATUS_REPORT':
        target = 'fog'
    return target


def decide_target_adaptive(task_type, ctx, th):
    """
    Adaptive offloading decision engine.

    Inputs (ctx): urgency (0..3), battery (0..1), cpu_load (0..1),
                  link_quality (0..1).
    Thresholds (th): battery_low, cpu_high, link_ok.

    Returns (target, reason). Cloud is reserved for non-real-time/archival
    traffic only — consistent with the architecture's "cloud = archival"
    commitment; real-time work is routed local or fog.
    """
    # 1) Archival / non-real-time -> cloud.
    if task_type in ARCHIVAL_TASKS:
        return 'cloud', 'archival/non-real-time -> cloud'

    # 2) Critical urgency -> fog fast path (coordinator must see it now).
    if ctx['urgency'] >= 3:
        return 'fog', 'critical priority -> fog fast path'

    # 3) Trivial local checks stay on the drone.
    if task_type == 'BATTERY_CHECK':
        return 'local', 'lightweight check -> local'

    # 4) Status reports -> fog coordinator, but hold local if the link is poor.
    if task_type == 'STATUS_REPORT':
        if ctx['link_quality'] < th['link_ok']:
            return 'local', f"status: weak link (q={ctx['link_quality']:.2f}) -> local"
        return 'fog', 'status -> fog coordinator'

    # 5) Heavy compute (victim detection) — the core offloading decision.
    if task_type in HEAVY_TASKS:
        q = ctx['link_quality']
        batt = ctx['battery']
        cpu = ctx['cpu_load']
        # Can't reliably offload over a weak link -> process onboard.
        if q < th['link_ok']:
            return 'local', f'heavy: weak link (q={q:.2f}) -> onboard'
        # Link is usable. Offload to relieve a resource-constrained edge.
        if batt < th['battery_low']:
            return 'fog', f'heavy: low battery ({batt:.2f}) -> offload to fog'
        if cpu > th['cpu_high']:
            return 'fog', f'heavy: high CPU ({cpu:.2f}) -> offload to fog'
        # Healthy edge + good link -> process onboard (lowest latency, no fog load).
        return 'local', f'heavy: edge healthy (batt={batt:.2f}, cpu={cpu:.2f}) -> onboard'

    return 'fog', 'default -> fog'


# ----------------------------------------------------------------------
# Main node
# ----------------------------------------------------------------------
class DroneTaskPublisher(Node):
    def __init__(self):
        super().__init__('drone_task_publisher')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.declare_parameter('simulate_low_battery', False)
        self.declare_parameter('enable_adaptive_offload', True)
        self.declare_parameter('use_priority_queue', True)
        self.declare_parameter('pathloss_exp', 2.0)
        self.declare_parameter('base_x', 25.0)  # fog/base station ENU East
        self.declare_parameter('base_y', 25.0)  # fog/base station ENU North
        self.declare_parameter('battery_low', 0.25)
        self.declare_parameter('cpu_high', 0.80)
        self.declare_parameter('link_ok', 0.35)

        self.instance = int(self.get_parameter('instance').value)
        self.simulate_low_battery = bool(self.get_parameter('simulate_low_battery').value)
        self.adaptive = bool(self.get_parameter('enable_adaptive_offload').value)
        self.use_pq = bool(self.get_parameter('use_priority_queue').value)
        self.pathloss_exp = float(self.get_parameter('pathloss_exp').value)
        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.th = {
            'battery_low': float(self.get_parameter('battery_low').value),
            'cpu_high': float(self.get_parameter('cpu_high').value),
            'link_ok': float(self.get_parameter('link_ok').value),
        }

        dsx, dsy = DEFAULT_SPAWNS.get(self.instance, (0.0, 0.0))
        self.declare_parameter('spawn_x', dsx)
        self.declare_parameter('spawn_y', dsy)
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        # ---- Derived names ----
        self.drone_id = drone_id_for(self.instance)
        px4_status_topic = px4_topic_for(self.instance, 'vehicle_status_v1')
        px4_position_topic = px4_topic_for(self.instance, 'vehicle_local_position_v1')
        self.camera_topic = f'/{self.drone_id}/camera/image'
        battery_topic = f'/{self.drone_id}/battery_status'

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

        # ---- State ----
        self.latest_status: VehicleStatus = None
        self.latest_position: VehicleLocalPosition = None
        self.prev_gray_frame: np.ndarray = None
        self.battery_frac = 1.0          # 0..1, updated from battery_status
        self.cpu_load = 0.0              # 0..1, sampled from load average

        # ---- Subscribers ----
        self.create_subscription(VehicleStatus, px4_status_topic,
                                 self.status_callback, px4_qos)
        self.create_subscription(VehicleLocalPosition, px4_position_topic,
                                 self.position_callback, px4_qos)
        self.create_subscription(Image, self.camera_topic,
                                 self.camera_callback, 1)
        self.create_subscription(String, battery_topic,
                                 self.battery_callback, 10)

        # ---- Publishers ----
        self.task_pubs = {
            tier: self.create_publisher(Task, topic, 10)
            for tier, topic in self.task_topics.items()
        }

        # ---- Priority queue ----
        # min-heap of (-priority, seq) so highest priority / oldest drains first
        self._pq = []
        self._pq_counter = itertools.count()
        self._pq_items = {}              # seq -> (task_msg, target)
        self.PQ_MAX = 500

        # ---- Counters ----
        self.status_seq = 0
        self.detection_seq = 0
        self.camera_frame_seq = 0
        self.route_counts = {'local': 0, 'fog': 0, 'cloud': 0}
        self.filter_stats = {
            'in': 0, 'passed': 0,
            'drop_dark': 0, 'drop_bright': 0,
            'drop_static': 0, 'drop_blur': 0,
        }

        # ---- Timers ----
        self.create_timer(STATUS_PERIOD_SEC, self.generate_status_task)
        self.create_timer(FILTER_STATS_PERIOD_SEC, self.log_filter_stats)
        self.create_timer(CPU_SAMPLE_PERIOD_SEC, self.sample_cpu)
        if self.use_pq:
            self.create_timer(1.0 / QUEUE_DRAIN_HZ, self.drain_queue)

        # ---- Startup logs ----
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id} (instance={self.instance}): '
            f'adaptive_offload={self.adaptive}, priority_queue={self.use_pq}, '
            f'simulate_low_battery={self.simulate_low_battery}')
        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: spawn ENU=({self.spawn_x},{self.spawn_y}), '
            f'fog/base ENU=({self.base_x},{self.base_y}), '
            f"thresholds batt<{self.th['battery_low']} cpu>{self.th['cpu_high']} "
            f"link<{self.th['link_ok']}")
        for tier, topic in self.task_topics.items():
            self.get_logger().info(f'[DRONE TASK PUB] {self.drone_id}: tier "{tier}" -> {topic}')

    # ------------------------------------------------------------------
    # Signal inputs
    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus):
        self.latest_status = msg

    def position_callback(self, msg: VehicleLocalPosition):
        self.latest_position = msg

    def battery_callback(self, msg: String):
        m = re.search(r'battery=([\d.]+)', msg.data)
        if m:
            try:
                self.battery_frac = max(0.0, min(1.0, float(m.group(1)) / 100.0))
            except ValueError:
                pass

    def sample_cpu(self):
        try:
            load1 = os.getloadavg()[0]
            ncpu = os.cpu_count() or 1
            self.cpu_load = max(0.0, min(1.0, load1 / ncpu))
        except (OSError, AttributeError):
            self.cpu_load = 0.0

    # ------------------------------------------------------------------
    # Decision context (the four live signals)
    # ------------------------------------------------------------------
    def _link_quality(self):
        """Model RSSI from 3-D drone<->fog distance. Returns (rssi_dbm, quality)."""
        if self.latest_position is None or not (
                self.latest_position.xy_valid and self.latest_position.z_valid):
            return RSSI_MAX_DBM, 1.0    # optimistic until EKF converges
        p = self.latest_position
        # PX4 local NED relative to spawn -> world ENU.
        world_x = self.spawn_x + float(p.y)     # East
        world_y = self.spawn_y + float(p.x)     # North
        alt = -float(p.z)
        dist = math.sqrt((world_x - self.base_x) ** 2
                         + (world_y - self.base_y) ** 2
                         + alt ** 2)
        rssi = rssi_from_distance(dist, self.pathloss_exp)
        return rssi, link_quality_from_rssi(rssi)

    def _battery(self):
        if self.simulate_low_battery:
            return 0.08
        return self.battery_frac

    def _build_context(self, priority):
        rssi, q = self._link_quality()
        return {
            'urgency': int(priority),
            'battery': self._battery(),
            'cpu_load': self.cpu_load,
            'rssi_dbm': round(rssi, 1),
            'link_quality': round(q, 3),
        }

    # ------------------------------------------------------------------
    # Dispatch: decide tier, annotate payload, enqueue/publish
    # ------------------------------------------------------------------
    def _dispatch(self, task_type, priority, payload, task_id, drone_failing=False):
        ctx = self._build_context(priority)
        if self.adaptive:
            target, reason = decide_target_adaptive(task_type, ctx, self.th)
        else:
            target = decide_target_static(task_type, drone_failing)
            reason = 'static routing table'

        payload['routing'] = {
            'target': target, 'reason': reason,
            'battery': round(ctx['battery'], 3),
            'cpu_load': round(ctx['cpu_load'], 3),
            'rssi_dbm': ctx['rssi_dbm'],
            'link_quality': ctx['link_quality'],
        }

        task = Task()
        task.task_id = task_id
        task.task_type = task_type
        task.drone_id = self.drone_id
        task.timestamp = self.get_clock().now().to_msg()
        task.priority = int(priority)
        task.payload = json.dumps(payload)

        self.route_counts[target] += 1
        if self.use_pq:
            self._enqueue(task, target, priority)
        else:
            self.task_pubs[target].publish(task)
        return target, reason, ctx

    def _enqueue(self, task, target, priority):
        seq = next(self._pq_counter)
        self._pq_items[seq] = (task, target)
        heapq.heappush(self._pq, (-int(priority), seq))
        # Overflow guard: drop the lowest-priority oldest item.
        if len(self._pq) > self.PQ_MAX:
            worst = max(self._pq)            # largest (-prio, seq) = lowest prio, oldest
            self._pq.remove(worst)
            heapq.heapify(self._pq)
            self._pq_items.pop(worst[1], None)

    def drain_queue(self):
        if not self._pq:
            return
        _, seq = heapq.heappop(self._pq)
        item = self._pq_items.pop(seq, None)
        if item is None:
            return
        task, target = item
        self.task_pubs[target].publish(task)

    # ------------------------------------------------------------------
    # Task generators
    # ------------------------------------------------------------------
    def generate_status_task(self):
        if self.latest_status is None:
            self.get_logger().info(
                f'[DRONE TASK PUB] {self.drone_id}: no PX4 status yet, skipping')
            return

        msg = self.latest_status
        drone_failing = self.simulate_low_battery or self._battery() <= self.th['battery_low']
        priority = 3 if drone_failing else 0

        payload = {
            'nav_state': int(msg.nav_state),
            'arming_state': int(msg.arming_state),
            'failsafe': bool(msg.failsafe),
            'pre_flight_checks_pass': bool(msg.pre_flight_checks_pass),
            'drone_failing': drone_failing,
            'position': self._build_position_dict(),
        }

        task_id = f'{self.drone_id}-status-{self.status_seq:04d}'
        target, reason, ctx = self._dispatch(
            'STATUS_REPORT', priority, payload, task_id, drone_failing)

        log = self.get_logger().warn if drone_failing else self.get_logger().info
        flag = ', DRONE_FAILING' if drone_failing else ''
        log(f'[DRONE TASK PUB] {self.drone_id}: {task_id} (STATUS_REPORT, p={priority}{flag}) '
            f'-> {target} [{reason}]')
        self.status_seq += 1

    def camera_callback(self, msg: Image):
        self.camera_frame_seq += 1
        self.filter_stats['in'] += 1

        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
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

        task_id = f'{self.drone_id}-detect-{self.detection_seq:04d}'
        target, rreason, ctx = self._dispatch(
            'VICTIM_DETECTION_REQUEST', 0, payload, task_id)

        self.get_logger().info(
            f'[DRONE TASK PUB] {self.drone_id}: {task_id} '
            f'(VICTIM_DETECTION_REQUEST, frame={self.camera_frame_seq}) '
            f'-> {target} [{rreason}]')
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
                return False, {'brightness': mean_brightness, 'diff': diff_score}, 'static'
        self.prev_gray_frame = gray

        blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if blur_score < BLUR_MIN:
            return False, {'brightness': mean_brightness, 'diff': diff_score,
                           'blur': blur_score}, 'blur'

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
        return {'valid': valid, 'x': float(p.x), 'y': float(p.y), 'z': float(p.z)}

    # ------------------------------------------------------------------
    def log_filter_stats(self):
        s = self.filter_stats
        total_dropped = s['drop_dark'] + s['drop_bright'] + s['drop_static'] + s['drop_blur']
        pass_rate = (100.0 * s['passed'] / s['in']) if s['in'] else 0.0
        rc = self.route_counts
        self.get_logger().info(
            f'[FILTER STATS] {self.drone_id}: '
            f"in={s['in']} passed={s['passed']} ({pass_rate:.1f}%) "
            f"dropped={total_dropped} "
            f"[dark={s['drop_dark']} bright={s['drop_bright']} "
            f"static={s['drop_static']} blur={s['drop_blur']}]")
        self.get_logger().info(
            f"[ROUTE STATS] {self.drone_id}: local={rc['local']} fog={rc['fog']} "
            f"cloud={rc['cloud']} | battery={self._battery():.2f} cpu={self.cpu_load:.2f} "
            f"pq_depth={len(self._pq)}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneTaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
