"""
fog_server.py

Fog node — scalable to ANY number of drones, with DISASTER-ZONE partitioning,
two-altitude mission profiles, camera-footprint COVERAGE TRACKING, and
end-of-mission cloud archival.

ZONE MODE (use_zones:=true)
Instead of partitioning one big rectangle, the fog takes a list of disaster
site locations (zones_x/zones_y, default = the three collapsed buildings) and
builds a square scan box of half-size zone_half_size around each. Drones are
assigned to zones round-robin; if several drones share a zone, the zone is
subdivided among them with the same recursive bisection. If there are more
zones than drones, the extra zones are NOT scanned (a warning is printed).

TWO-ALTITUDE PROFILE
Every START_MISSION carries both:
  - transit_alt : high, obstacle-safe altitude for crossing the map (~35 m)
  - alt         : low scan altitude for victim detection inside the zone (~18 m)
The commander climbs to transit_alt, crosses to the zone, DESCENDS over the
zone's first corner, and sweeps at the scan altitude. Low flight only ever
happens inside the (rubble-only) disaster zones, so trees on the way are
cleared at transit altitude.

RECTANGLE MODE (use_zones:=false, default)
Same behaviour as before: one rectangle partitioned by recursive bisection.

COVERAGE
A point is covered when it falls inside the camera footprint of a captured
frame (2*h*tan(FOV/2), using the drone's ACTUAL altitude, so the smaller
low-altitude footprint is modelled correctly). Needs the camera bridge per
drone. Coverage % is reported every 5 s and a final report + ASCII map is
printed and written to /tmp/fog_coverage_<ts>.txt on end_mission.

DRONE-FAILURE REPARTITIONING (Task 6.9 reliability scenario)
If a drone becomes unreachable, the fog drops it from the active set and
REPARTITIONS the search area among the survivors, then re-sends START_MISSION so
the remaining drones expand their sweep to cover the whole area — the mission
continues instead of leaving a hole. A failure is triggered either:
  * deterministically for experiments:  fail_drone_id + fail_after_sec, or
  * realistically by a telemetry watchdog:  heartbeat_timeout_sec > 0
Already-covered ground is carried over into the new partition, so survivors don't
re-sweep what was already done. Failure/repartition events are logged, buffered
to the cloud archive, and published on /fog/reliability.

All existing contracts (Task message, /fog/{drone}/decision, topic and
service names) are unchanged.
"""

import os
import time
import json
import math
from collections import deque
from datetime import datetime

os.environ['CUDA_VISIBLE_DEVICES'] = ''  # force CPU

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from task_msgs.msg import Task

from fog_node.drone_naming import (
    drone_id_for,
    px4_topic_for,
)


# ----------------------------------------------------------------------
EVENT_BUFFER_SOFT_CAP = 10000
BATCH_CHUNK_SIZE = 1000

WORLD_ORIGIN_LAT = 47.397971057728974
WORLD_ORIGIN_LON = 8.546163739800146
_EARTH_RADIUS_M = 6371000.0


def enu_to_global(world_x, world_y):
    origin_lat_rad = math.radians(WORLD_ORIGIN_LAT)
    d_lat = world_y / _EARTH_RADIUS_M
    d_lon = world_x / (_EARTH_RADIUS_M * math.cos(origin_lat_rad))
    return (WORLD_ORIGIN_LAT + math.degrees(d_lat),
            WORLD_ORIGIN_LON + math.degrees(d_lon))


def partition_area(min_x, max_x, min_y, max_y, n):
    """Recursive bisection -> n gap-free, area-balanced, near-square cells."""
    if n <= 1:
        return [(min_x, max_x, min_y, max_y)]
    a = n // 2
    b = n - a
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x >= span_y:
        cut = min_x + span_x * (a / n)
        return (partition_area(min_x, cut, min_y, max_y, a)
                + partition_area(cut, max_x, min_y, max_y, b))
    else:
        cut = min_y + span_y * (a / n)
        return (partition_area(min_x, max_x, min_y, cut, a)
                + partition_area(min_x, max_x, cut, max_y, b))


class CoverageGrid:
    """Binary occupancy grid over one cell (a set of (i,j) bins)."""

    def __init__(self, min_x, max_x, min_y, max_y, cell_m):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.cell = max(cell_m, 0.5)
        self.nx = max(1, math.ceil((max_x - min_x) / self.cell))
        self.ny = max(1, math.ceil((max_y - min_y) / self.cell))
        self.total = self.nx * self.ny
        self.covered = set()

    def contains(self, x, y):
        return (self.min_x <= x < self.max_x) and (self.min_y <= y < self.max_y)

    def mark_footprint(self, cx, cy, half_w, half_l):
        x0 = max(self.min_x, cx - half_w)
        x1 = min(self.max_x, cx + half_w)
        y0 = max(self.min_y, cy - half_l)
        y1 = min(self.max_y, cy + half_l)
        if x1 <= x0 or y1 <= y0:
            return
        i0 = int((x0 - self.min_x) // self.cell)
        i1 = int((x1 - self.min_x) // self.cell)
        j0 = int((y0 - self.min_y) // self.cell)
        j1 = int((y1 - self.min_y) // self.cell)
        for i in range(max(0, i0), min(self.nx - 1, i1) + 1):
            for j in range(max(0, j0), min(self.ny - 1, j1) + 1):
                self.covered.add((i, j))

    def is_covered(self, x, y):
        if not self.contains(x, y):
            return False
        i = int((x - self.min_x) // self.cell)
        j = int((y - self.min_y) // self.cell)
        return (i, j) in self.covered

    def pct(self):
        return 100.0 * len(self.covered) / self.total if self.total else 0.0


class FogServer(Node):
    def __init__(self):
        super().__init__('fog_server')

        # ---- Parameters ----
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('enable_detection', False)
        # Cloud tier: whether to archive the mission log to the cloud at the
        # end. True for fog + cloud scenarios (cloud reachable); set false for
        # the local / fog+cloud-down fallback so the cloud tier is correctly
        # reported as inactive.
        self.declare_parameter('archive_to_cloud', True)
        self.archive_to_cloud = bool(self.get_parameter('archive_to_cloud').value)
        self.num_drones = int(self.get_parameter('num_drones').value)
        self.enable_detection = bool(self.get_parameter('enable_detection').value)
        if self.num_drones < 1:
            raise ValueError(f'num_drones must be >= 1, got {self.num_drones}')

        # Rectangle mode bounds (used when use_zones is false)
        self.declare_parameter('area_min_x', -80.0)
        self.declare_parameter('area_max_x', 100.0)
        self.declare_parameter('area_min_y', -40.0)
        self.declare_parameter('area_max_y', 75.0)

        # Altitudes: high transit (clears trees/buildings), low scan (detection)
        self.declare_parameter('transit_altitude', 35.0)
        self.declare_parameter('scan_altitude', 18.0)

        # Zone mode: scan boxes around disaster sites only.
        # Defaults = the three collapsed buildings in baylands_collapsed_fixed.
        self.declare_parameter('use_zones', False)
        self.declare_parameter('zones_x', [35.0, 80.0, -60.0])
        self.declare_parameter('zones_y', [-20.0, 40.0, 55.0])
        self.declare_parameter('zone_half_size', 25.0)

        self.declare_parameter('spawns_x', [18.0, 23.0, 30.0])
        self.declare_parameter('spawns_y', [25.0, 25.0, 25.0])

        # Coverage params. VERIFY camera_hfov_deg against your model SDF.
        self.declare_parameter('camera_hfov_deg', 60.0)
        self.declare_parameter('camera_vfov_deg', 0.0)   # 0 -> derive (4:3)
        self.declare_parameter('coverage_cell_m', 2.0)
        self.declare_parameter('coverage_overlap', 0.2)
        # "Fully covered" threshold. When a drone's cell reaches this %, the
        # fog sends THAT drone RTL; when all cells reach it, the mission ends
        # automatically (report + cloud flush). Set 0 to disable auto-ending
        # (drones then loop until a manual /fog/end_mission).
        self.declare_parameter('coverage_target_pct', 98.0)
        # Mean-coverage auto-finish: when the bin-weighted OVERALL coverage across
        # all active cells reaches this %, end the mission (RTL all + report +
        # cloud flush). Independent of the per-cell coverage_target_pct above; set
        # coverage_target_pct:=0.0 to use only this mean-based finish. 0 = disabled.
        self.declare_parameter('auto_finish_coverage_pct', 0.0)

        # ---- Drone-failure / repartition scenario (Task 6.9) ----
        # fail_drone_id >= 0 injects a deterministic failure of that instance
        # fail_after_sec seconds after START_MISSION (for repeatable experiments).
        # heartbeat_timeout_sec > 0 additionally declares any active drone failed
        # when its PX4 telemetry goes silent for that long (realistic "unreachable").
        # On failure the area is repartitioned among the survivors and re-dispatched.
        self.declare_parameter('fail_drone_id', -1)
        self.declare_parameter('fail_after_sec', 60.0)
        self.declare_parameter('fail_action', 'RTL')          # sent to the failed drone
        self.declare_parameter('heartbeat_timeout_sec', 0.0)  # 0 = auto-detect off
        # Grace window (s) after a drone is (re)dispatched before the heartbeat
        # watchdog may fail it, so a not-yet-streaming or momentarily-starved
        # drone isn't falsely declared dead. Should exceed heartbeat_timeout_sec.
        self.declare_parameter('heartbeat_grace_sec', 20.0)
        self.declare_parameter('repartition_on_failure', True)

        self.area_min_x = float(self.get_parameter('area_min_x').value)
        self.area_max_x = float(self.get_parameter('area_max_x').value)
        self.area_min_y = float(self.get_parameter('area_min_y').value)
        self.area_max_y = float(self.get_parameter('area_max_y').value)
        self.transit_altitude = float(self.get_parameter('transit_altitude').value)
        self.scan_altitude = float(self.get_parameter('scan_altitude').value)

        self.use_zones = bool(self.get_parameter('use_zones').value)
        self.zones_x = [float(v) for v in self.get_parameter('zones_x').value]
        self.zones_y = [float(v) for v in self.get_parameter('zones_y').value]
        self.zone_half = float(self.get_parameter('zone_half_size').value)
        if len(self.zones_x) != len(self.zones_y):
            raise ValueError('zones_x and zones_y must have the same length.')

        self.spawns_x = [float(v) for v in self.get_parameter('spawns_x').value]
        self.spawns_y = [float(v) for v in self.get_parameter('spawns_y').value]
        if len(self.spawns_x) != len(self.spawns_y):
            raise ValueError('spawns_x and spawns_y must have the same length.')
        if len(self.spawns_x) < self.num_drones:
            self.get_logger().warn(
                f'[FOG] only {len(self.spawns_x)} spawn(s) for '
                f'{self.num_drones} drone(s); coverage needs a spawn per drone.')

        hfov = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        vfov_deg = float(self.get_parameter('camera_vfov_deg').value)
        vfov = (2.0 * math.atan(math.tan(hfov / 2.0) / (4.0 / 3.0))
                if vfov_deg <= 0.0 else math.radians(vfov_deg))
        self._tan_half_h = math.tan(hfov / 2.0)
        self._tan_half_v = math.tan(vfov / 2.0)
        self.coverage_cell_m = float(self.get_parameter('coverage_cell_m').value)
        self.coverage_overlap = float(self.get_parameter('coverage_overlap').value)
        self.coverage_target_pct = float(self.get_parameter('coverage_target_pct').value)
        self.auto_finish_coverage_pct = float(
            self.get_parameter('auto_finish_coverage_pct').value)

        self.fail_drone_id = int(self.get_parameter('fail_drone_id').value)
        self.fail_after_sec = float(self.get_parameter('fail_after_sec').value)
        self.fail_action = str(self.get_parameter('fail_action').value)
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout_sec').value)
        self.heartbeat_grace = float(self.get_parameter('heartbeat_grace_sec').value)
        self.repartition = bool(self.get_parameter('repartition_on_failure').value)

        # ---- Detection model (loaded only if enabled) ----
        self.declare_parameter('detect_period', 0.4)   # inference cadence (timer)
        self.declare_parameter('detect_imgsz', 640)    # lower on weak CPUs
        self.detect_period = float(self.get_parameter('detect_period').value)
        self.detect_imgsz = int(self.get_parameter('detect_imgsz').value)
        self.model = None
        self._latest_frame = {}   # drone_id -> newest frame (detection buffer)
        self._detect_rr = -1      # round-robin pointer across drones
        if self.enable_detection:
            from ultralytics import YOLO
            self.model = YOLO('yolov8n.pt')
            # Warm up so the first real inference isn't a multi-second stall.
            self.model(np.zeros((480, 640, 3), dtype=np.uint8),
                       verbose=False, device='cpu', imgsz=self.detect_imgsz)
            self.get_logger().info(
                f'[FOG] YOLOv8n detection ENABLED (CPU, imgsz={self.detect_imgsz}, '
                f'infer every {self.detect_period}s over the newest frame per drone)')
        else:
            self.get_logger().info(
                '[FOG] Detection disabled (use enable_detection:=true to enable)')

        # ---- PX4 QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ---- Per-drone state ----
        self.decision_publishers = {}
        self.mission_cmd_publishers = {}
        self.util_task_pubs = {}      # Task 6 fog-utilisation (see loop below)
        self.stats = {}
        self.spawn_of = {}
        self.local_pos = {}
        self.cov_grid = {}
        self.coverage_pct = {}
        self._cell_done = {}      # drone_id -> True once its cell hit the target
        self._armed = {}
        self._coverage_done = False
        # ---- Drone-failure state ----
        self.active_drones = set()      # drone_ids currently in the mission
        self.failed_drones = set()      # drone_ids declared unreachable
        self.last_seen = {}             # drone_id -> last telemetry wall-clock
        self._dispatch_grace_until = {} # drone_id -> wall-clock until which the
                                        # heartbeat watchdog must not fail it
        self._mission_start_t = None    # set on START_MISSION (for fail_after_sec)
        # Bounds used for the report's ASCII map (set at start_mission)
        self.map_bounds = (self.area_min_x, self.area_max_x,
                           self.area_min_y, self.area_max_y)

        for instance in range(self.num_drones):
            drone_id = drone_id_for(instance)
            status_topic = px4_topic_for(instance, 'vehicle_status_v1')
            pos_topic = px4_topic_for(instance, 'vehicle_local_position_v1')
            task_topic = f'/{drone_id}/task/fog'
            camera_topic = f'/{drone_id}/camera/image'

            self.create_subscription(
                VehicleStatus, status_topic,
                lambda msg, d=drone_id: self.status_callback(msg, d), px4_qos)
            self.create_subscription(
                VehicleLocalPosition, pos_topic,
                lambda msg, d=drone_id: self.local_pos_callback(msg, d), px4_qos)
            self.create_subscription(
                Task, task_topic,
                lambda msg, d=drone_id: self.task_callback(msg, d), 10)
            self.create_subscription(
                Image, camera_topic,
                lambda msg, d=drone_id: self.camera_callback(msg, d), 1)

            self.decision_publishers[drone_id] = self.create_publisher(
                String, f'/fog/{drone_id}/decision', 10)
            self.mission_cmd_publishers[drone_id] = self.create_publisher(
                String, f'/{drone_id}/mission_command', 10)
            # Task 6 utilisation (fog tier): metrics_collector counts one unit
            # of fog utilisation per Task on /{drone}/task/fog — the SAME topic
            # this node subscribes to for the Task-4 drone->fog contract.
            # Nothing ever published there in fog mode, so every fog-mode
            # summary showed utilisation fog=0 while cloud mode (whose
            # cloud_detector publishes /{drone}/task/cloud per processed
            # frame) counted fine. camera_callback now publishes one
            # FOG_PROCESSING task per frame the fog actually infers, and
            # task_callback filters that type out to avoid a self-receive
            # loop (see both).
            self.util_task_pubs[drone_id] = self.create_publisher(
                Task, task_topic, 10)
            self.stats[drone_id] = {'status': 0, 'tasks': 0, 'frames': 0}
            self._armed[drone_id] = False

            if instance < len(self.spawns_x):
                self.spawn_of[drone_id] = (self.spawns_x[instance],
                                           self.spawns_y[instance])

            self.get_logger().info(
                f'[FOG] {drone_id} (instance={instance}): '
                f'status={status_topic}, pos={pos_topic}, '
                f'task={task_topic}, camera={camera_topic}')

        # ---- Cloud archival ----
        self.event_buffer = deque(maxlen=EVENT_BUFFER_SOFT_CAP)
        self.events_dropped_on_overflow = 0
        self._prev_buffer_len = 0
        self.cloud_pub = self.create_publisher(String, '/fog/cloud/mission_log', 10)

        # ---- Victim alert publisher (Task 4 detection output) ----
        self.victim_alert_pub = self.create_publisher(
            String, '/fog/victim_alerts', 10)

        # ---- Coverage telemetry (Task 6 metric: Coverage Efficiency) ----
        self.coverage_pub = self.create_publisher(String, '/fog/coverage', 10)

        # Track in-flight rescues so auto-finish waits for a dispatched drone to
        # actually REACH its victim before ending the mission. decision_node
        # publishes ASSIGNED/RESOLVED lifecycle on /fog/decision_log; an event
        # assigned but not yet resolved is a rescue still in progress. Without
        # this, a victim detected late (near the coverage target) is dispatched
        # but the mission auto-finishes before arrival, so it shows as
        # created/assigned-but-not-resolved with no completion time.
        self.declare_parameter('rescue_grace_sec', 30.0)
        self.rescue_grace_sec = float(self.get_parameter('rescue_grace_sec').value)
        self._open_rescues = {}          # event_id -> first_seen wall-clock
        self._finish_deferred_since = None
        self.create_subscription(
            String, '/fog/decision_log', self._decision_log_cb, 10)

        # ---- Reliability telemetry (Task 6.9: drone failure / repartition) ----
        self.reliability_pub = self.create_publisher(String, '/fog/reliability', 10)

        self.end_mission_srv = self.create_service(
            Trigger, '/fog/end_mission', self.end_mission_callback)
        self.start_mission_srv = self.create_service(
            Trigger, '/fog/start_mission', self.start_mission_callback)

        mode = 'ZONES (disaster sites)' if self.use_zones else 'RECTANGLE'
        self.get_logger().info(
            f'[FOG] tracking {self.num_drones} drone(s), partition mode: {mode}')
        self.get_logger().info(
            f'[FOG] transit_alt={self.transit_altitude}m (obstacle-safe), '
            f'scan_alt={self.scan_altitude}m (detection)')
        self.get_logger().info(
            '[FOG] coverage tracking ON (needs the camera bridge per drone)')

        self.create_timer(5.0, self.log_stats)
        # Detection runs on a timer over the newest frame per drone (see
        # camera_callback / _detect_latest) so a slow inference never processes
        # a stale backlog. Harmless no-op when detection is disabled.
        self.create_timer(self.detect_period, self._detect_latest)
        # Drone-failure watchdog (injected schedule + telemetry heartbeat).
        self.create_timer(1.0, self._failure_watchdog)
        if self.fail_drone_id >= 0:
            self.get_logger().warn(
                f'[FOG RELIABILITY] injected failure ARMED: '
                f'{drone_id_for(self.fail_drone_id)} will fail '
                f'{self.fail_after_sec:.0f}s after START_MISSION '
                f'(repartition={self.repartition}).')
        if self.heartbeat_timeout > 0:
            self.get_logger().info(
                f'[FOG RELIABILITY] telemetry watchdog ON '
                f'(declare failed after {self.heartbeat_timeout:.0f}s silence).')

    # ------------------------------------------------------------------
    # Assignment builders
    # ------------------------------------------------------------------
    def _build_assignments(self, active_ids=None):
        """
        Returns dict { drone_id : {min_x,max_x,min_y,max_y,cx,cy,zone} }.
        Zone mode: drones assigned to zones round-robin; a zone shared by k
        drones is subdivided into k cells. Rectangle mode: bisection of the
        one rectangle. `active_ids` (list of drone_ids) partitions among exactly
        those drones — used after a failure to repartition among survivors.
        Default (None) = all drones 0..num_drones-1, i.e. the original behaviour.
        """
        if active_ids is None:
            active_ids = [drone_id_for(i) for i in range(self.num_drones)]
        n = len(active_ids)
        assignments = {}
        if n == 0:
            return assignments
        if self.use_zones and self.zones_x:
            num_zones = len(self.zones_x)
            if num_zones > n:
                self.get_logger().warn(
                    f'[FOG] {num_zones} zones but only {n} '
                    f'active drone(s): zones {n}..{num_zones - 1} '
                    f'will NOT be scanned.')
            # Round-robin active drones onto zones
            groups = {}   # zone_idx -> [drone_ids]
            for k, did in enumerate(active_ids):
                z = k % num_zones
                groups.setdefault(z, []).append(did)
            for z, members in sorted(groups.items()):
                zx, zy = self.zones_x[z], self.zones_y[z]
                h = self.zone_half
                cells = partition_area(zx - h, zx + h, zy - h, zy + h,
                                       len(members))
                for did, (x0, x1, y0, y1) in zip(members, cells):
                    assignments[did] = {
                        'min_x': round(x0, 2), 'max_x': round(x1, 2),
                        'min_y': round(y0, 2), 'max_y': round(y1, 2),
                        'cx': round((x0 + x1) / 2, 2),
                        'cy': round((y0 + y1) / 2, 2),
                        'zone': z,
                    }
        else:
            cells = partition_area(self.area_min_x, self.area_max_x,
                                   self.area_min_y, self.area_max_y, n)
            for idx, (did, (x0, x1, y0, y1)) in enumerate(zip(active_ids, cells)):
                assignments[did] = {
                    'min_x': round(x0, 2), 'max_x': round(x1, 2),
                    'min_y': round(y0, 2), 'max_y': round(y1, 2),
                    'cx': round((x0 + x1) / 2, 2),
                    'cy': round((y0 + y1) / 2, 2),
                    'zone': idx,
                }
        return assignments

    # ------------------------------------------------------------------
    def _record_event(self, event_type, drone_id, payload):
        if len(self.event_buffer) >= EVENT_BUFFER_SOFT_CAP:
            self.events_dropped_on_overflow += 1
        self.event_buffer.append({
            'event_type': event_type, 'drone_id': drone_id,
            'fog_received_at': time.time(), 'payload': payload,
        })

    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus, drone_id: str):
        self.stats[drone_id]['status'] += 1
        self.last_seen[drone_id] = time.time()

        is_armed = (msg.arming_state == 2)
        if is_armed and not self._armed[drone_id]:
            self.get_logger().warn(f'[FOG ALERT] {drone_id} ARMED (in flight)')
        elif not is_armed and self._armed[drone_id]:
            self.get_logger().info(f'[FOG] {drone_id} DISARMED')
        self._armed[drone_id] = is_armed

        decision = String()
        if is_armed:
            decision.data = f'{drone_id}: COMMAND_MONITOR (ARMED)'
        elif msg.nav_state == 4:
            decision.data = f'{drone_id}: COMMAND_HOLD_POSITION'
        else:
            decision.data = f'{drone_id}: COMMAND_NORMAL_OPERATION'

        time.sleep(0.05)
        self.decision_publishers[drone_id].publish(decision)

    def local_pos_callback(self, msg: VehicleLocalPosition, drone_id: str):
        self.local_pos[drone_id] = msg
        self.last_seen[drone_id] = time.time()

    # ------------------------------------------------------------------
    def task_callback(self, msg: Task, drone_id: str):
        if msg.task_type == 'FOG_PROCESSING':
            # Our OWN per-frame utilisation marker (published in
            # camera_callback for metrics_collector). This node subscribes to
            # the same topic for the drone->fog Task-4 contract, so it
            # receives its own messages — without this filter every inferred
            # frame would flood _record_event/the cloud archive at 2 Hz per
            # drone. Skip entirely: don't count it in stats['tasks'] either,
            # so that stat keeps meaning "drone-originated tasks".
            return
        self.stats[drone_id]['tasks'] += 1

        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
        latency_ms = (now_ns - sent_ns) / 1e6

        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except json.JSONDecodeError:
            payload = {'_parse_error': True}

        self._record_event('TASK_RECEIVED', drone_id, {
            'task_id': msg.task_id, 'task_type': msg.task_type,
            'priority': int(msg.priority), 'latency_ms': round(latency_ms, 2),
            'task_timestamp_sec': int(msg.timestamp.sec),
            'task_timestamp_nsec': int(msg.timestamp.nanosec),
            'payload_keys': list(payload.keys()),
        })

        if msg.priority == 3:
            self.get_logger().warn(
                f'[FOG TASK CRITICAL] {drone_id} {msg.task_id} '
                f'type={msg.task_type} PRIORITY=3 latency={latency_ms:.1f}ms '
                f'failing={payload.get("drone_failing", False)}')
            self._record_event('PRIORITY3_ALERT', drone_id, {
                'task_id': msg.task_id, 'task_type': msg.task_type,
                'drone_failing': payload.get('drone_failing', False),
                'position': payload.get('position'),
            })
        else:
            self.get_logger().info(
                f'[FOG TASK] {drone_id} {msg.task_id} type={msg.task_type} '
                f'priority={msg.priority} latency={latency_ms:.1f}ms '
                f'payload_keys={list(payload.keys())}')

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image, drone_id: str):
        self.stats[drone_id]['frames'] += 1

        # Coverage tracking always runs (area-partitioning). Cheap: no YOLO.
        self._mark_coverage(drone_id)

        # Victim detection only when enabled (Task 4).
        if not self.enable_detection or self.model is None:
            return

        # Store ONLY the newest frame for this drone; inference happens on a
        # timer (_detect_latest). Running YOLO synchronously here — for THREE
        # cameras, on one single-threaded executor — built a stale-frame
        # backlog: the fog analysed frames seconds old, so a drone could pass
        # directly over the victim and the fog never inferred a frame that
        # actually contained it. Storing-and-timing (same latest-frame pattern
        # as cloud_detector / victim_detector) means the fog always infers the
        # CURRENT view, which is what makes detection reliable under load.
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        except ValueError:
            return
        self._latest_frame[drone_id] = frame

    def _detect_latest(self):
        """Timer: run YOLO on the newest frame of ONE drone per tick (round
        robin), so a slow inference can never back up a stale queue and the
        three cameras share the fog's inference budget fairly."""
        if not self.enable_detection or self.model is None:
            return
        drones = [d for d in sorted(self._latest_frame)
                  if self._latest_frame[d] is not None]
        if not drones:
            return
        # Round-robin pointer so no single drone monopolises inference.
        self._detect_rr = (self._detect_rr + 1) % len(drones)
        drone_id = drones[self._detect_rr]
        frame = self._latest_frame[drone_id]

        start = time.time()
        results = self.model(frame, verbose=False, device='cpu',
                             imgsz=self.detect_imgsz)
        inference_ms = (time.time() - start) * 1000

        # Task 6 utilisation: one FOG_PROCESSING task per frame the fog tier
        # actually inferred, mirroring cloud_detector's per-processed-frame
        # /{drone}/task/cloud. metrics_collector counts these into
        # utilisation.per_drone[drone].fog. task_callback ignores this type
        # (self-receive filter).
        ut = Task()
        ut.task_id = f'fog-{drone_id}-proc-{self.stats[drone_id]["frames"]:05d}'
        ut.task_type = 'FOG_PROCESSING'
        ut.drone_id = drone_id
        ut.timestamp = self.get_clock().now().to_msg()
        ut.priority = 1
        ut.payload = json.dumps({'inference_ms': round(inference_ms, 1)})
        self.util_task_pubs[drone_id].publish(ut)

        # Extract person detections
        detections = []
        boxes = results[0].boxes
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i])
            conf = float(boxes.conf[i])
            if cls_id == 0 and conf >= 0.25:  # person class, conf > 25%
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                detections.append({
                    'bbox': [round(x1, 1), round(y1, 1),
                             round(x2, 1), round(y2, 1)],
                    'confidence': round(conf, 3),
                    'label': 'person',
                })

        if detections:
            # Publish detection as Task message
            task = Task()
            task.task_id = f'fog-{drone_id}-detect-{self.stats[drone_id]["frames"]:04d}'
            task.task_type = 'VICTIM_DETECTION'
            task.drone_id = drone_id
            task.timestamp = self.get_clock().now().to_msg()
            task.priority = 2
            task.payload = json.dumps({
                'detections': detections,
                'num_persons': len(detections),
                'inference_time_ms': round(inference_ms, 1),
                'processed_at': 'fog',
            })

            # Record event
            self._record_event('VICTIM_DETECTED', drone_id, {
                'num_persons': len(detections),
                'inference_ms': round(inference_ms, 1),
                'detections': detections,
            })

            self.get_logger().warn(
                f'[FOG DETECTION] {drone_id}: {len(detections)} person(s) '
                f'detected (inference={inference_ms:.0f}ms)'
            )

            # Publish victim alert
            alert = String()
            alert.data = json.dumps({
                'drone_id': drone_id,
                'num_persons': len(detections),
                'detections': detections,
                'inference_time_ms': round(inference_ms, 1),
                'processed_at': 'fog',
                'timestamp': time.time(),
            })
            self.victim_alert_pub.publish(alert)

    def _decision_log_cb(self, msg):
        """Track rescues in flight from decision_node's lifecycle log so
        auto-finish can wait for them (see the auto-finish gate in log_stats)."""
        try:
            rec = json.loads(msg.data)
        except Exception:
            return
        kind = rec.get('kind')
        eid = rec.get('event_id')
        if not eid:
            return
        if kind == 'ASSIGNED':
            # a rescuer was dispatched to this victim; it's now in flight
            self._open_rescues.setdefault(eid, time.time())
        elif kind == 'RESOLVED':
            # rescuer arrived; no longer holding the mission open for it
            self._open_rescues.pop(eid, None)
            # reset the defer clock once the last open rescue closes
            if not self._open_rescues:
                self._finish_deferred_since = None

    def _mark_coverage(self, drone_id):
        grid = self.cov_grid.get(drone_id)
        spawn = self.spawn_of.get(drone_id)
        lp = self.local_pos.get(drone_id)
        if grid is None or spawn is None or lp is None:
            return
        if not (lp.xy_valid and lp.z_valid):
            return
        spawn_x, spawn_y = spawn
        world_x = spawn_x + lp.y          # NED East  -> ENU East
        world_y = spawn_y + lp.x          # NED North -> ENU North
        # Use the drone's ACTUAL altitude so the low-scan footprint is correct.
        h = -lp.z if lp.z < 0 else self.scan_altitude
        grid.mark_footprint(world_x, world_y,
                            h * self._tan_half_h, h * self._tan_half_v)
        self.coverage_pct[drone_id] = grid.pct()
        self._check_cell_complete(drone_id)

    def _check_cell_complete(self, drone_id):
        """Scan-until-covered: when this drone's cell hits the target, send it
        home; when every cell has hit the target, end the mission."""
        if self.coverage_target_pct <= 0.0 or self._coverage_done:
            return
        if self._cell_done.get(drone_id):
            return
        pct = self.coverage_pct.get(drone_id, 0.0)
        if pct < self.coverage_target_pct:
            return

        self._cell_done[drone_id] = True
        self.get_logger().info(
            f'[FOG COVERAGE] {drone_id} cell FULLY COVERED ({pct:.1f}% >= '
            f'{self.coverage_target_pct:.0f}%) — sending RTL to {drone_id}.')
        m = String()
        m.data = json.dumps({'command': 'RTL'})
        self.mission_cmd_publishers[drone_id].publish(m)

        if self.cov_grid and all(self._cell_done.get(d) for d in self.cov_grid):
            self._coverage_done = True
            self.get_logger().info(
                '[FOG COVERAGE] ALL cells fully covered — mission complete.')
            self._finish_mission()

    # ------------------------------------------------------------------
    def _flush_to_cloud(self):
        events = list(self.event_buffer)
        total_events = len(events)
        if total_events == 0:
            self.get_logger().info('[FOG END_MISSION] Buffer empty, nothing to flush.')
            return 0, 0, 0
        chunks = [events[i:i + BATCH_CHUNK_SIZE]
                  for i in range(0, total_events, BATCH_CHUNK_SIZE)]
        total_batches = len(chunks)
        fog_timestamp = time.time()
        self.get_logger().info(
            f'[FOG END_MISSION] Flushing {total_events} events in '
            f'{total_batches} batch(es) to cloud.')
        for idx, chunk in enumerate(chunks):
            msg = String()
            msg.data = json.dumps({
                'fog_timestamp': fog_timestamp,
                'batch_index': idx + 1, 'total_batches': total_batches,
                'event_count': len(chunk), 'events': chunk})
            self.cloud_pub.publish(msg)
            self.get_logger().info(
                f'[FOG END_MISSION] Published batch {idx + 1}/{total_batches} '
                f'({len(chunk)} events).')
        self.event_buffer.clear()
        dropped = self.events_dropped_on_overflow
        self.events_dropped_on_overflow = 0
        return total_events, total_batches, dropped

    def _command_all_rtl(self):
        cmd = json.dumps({'command': 'RTL'})
        for pub in self.mission_cmd_publishers.values():
            m = String()
            m.data = cmd
            pub.publish(m)
        self.get_logger().info(
            f'[FOG END_MISSION] Sent RTL to {len(self.mission_cmd_publishers)} drone(s).')

    # ------------------------------------------------------------------
    def _ascii_map(self, cols=60, rows=24):
        """' ' = outside every scan cell, '.' = in-cell uncovered, '#' = covered."""
        mnx, mxx, mny, mxy = self.map_bounds
        out = []
        dx = (mxx - mnx) / cols
        dy = (mxy - mny) / rows
        for r in range(rows):
            y = mxy - (r + 0.5) * dy
            line = []
            for c in range(cols):
                x = mnx + (c + 0.5) * dx
                ch = ' '
                for g in self.cov_grid.values():
                    if g.contains(x, y):
                        ch = '#' if g.is_covered(x, y) else '.'
                        if ch == '#':
                            break
                line.append(ch)
            out.append(''.join(line))
        return out

    def _coverage_report(self):
        mnx, mxx, mny, mxy = self.map_bounds
        mode = 'ZONES' if self.use_zones else 'RECTANGLE'
        lines = ['=' * 64, f'COVERAGE REPORT  (mode: {mode})',
                 f'Map bounds: X[{mnx},{mxx}] Y[{mny},{mxy}]']
        total_bins = covered_bins = 0
        for d in sorted(self.cov_grid):
            g = self.cov_grid[d]
            total_bins += g.total
            covered_bins += len(g.covered)
            lines.append(
                f'  {d}: cell X[{g.min_x},{g.max_x}] Y[{g.min_y},{g.max_y}]  '
                f'covered {g.pct():5.1f}%  ({len(g.covered)}/{g.total} bins)')
        overall = 100.0 * covered_bins / total_bins if total_bins else 0.0
        lines.append(f'OVERALL COVERAGE OF ASSIGNED CELLS: {overall:.1f}%  '
                     f'({covered_bins}/{total_bins} bins)')
        lines.append("MAP  '#'=captured  '.'=assigned, not yet captured  "
                     "' '=outside scan cells   (N at top, W at left)")
        lines += self._ascii_map()
        lines.append('=' * 64)
        return '\n'.join(lines), overall

    def _write_report(self, text):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = f'/tmp/fog_coverage_{ts}.txt'
        try:
            with open(path, 'w') as f:
                f.write(text + '\n')
        except OSError as e:
            self.get_logger().error(f'[FOG COVERAGE] could not write report: {e}')
            return '(write failed)'
        return path

    def _finish_mission(self):
        self._command_all_rtl()
        if self.cov_grid:
            text, overall = self._coverage_report()
            path = self._write_report(text)
            self.get_logger().info(
                '[FOG COVERAGE] FINAL REPORT (saved to %s):\n%s' % (path, text))
        # The end-of-mission archive is the CLOUD tier's characteristic work.
        # In scenarios where the cloud is meant to be unavailable (the local /
        # fog+cloud-down fallback), set archive_to_cloud:=false so the run
        # truthfully shows the cloud tier as inactive instead of archiving.
        if not self.archive_to_cloud:
            self.get_logger().info(
                '[FOG END_MISSION] archive_to_cloud=false -> skipping cloud '
                'flush (cloud tier represented as unavailable this run).')
            return 0, 0, 0
        return self._flush_to_cloud()

    def end_mission_callback(self, request, response):
        total_events, total_batches, dropped = self._finish_mission()
        response.success = True
        response.message = (
            f'RTL sent + coverage report written. Flushed {total_events} events in '
            f'{total_batches} batch(es). Overflow drops: {dropped}.')
        return response

    # ------------------------------------------------------------------
    def _dispatch_assignments(self, assignments, header='START_MISSION'):
        """Build a fresh coverage grid per assigned drone, set map bounds, and
        publish START_MISSION to each. Shared by the initial start and by a
        post-failure repartition, so both use identical mission geometry."""
        if not assignments:
            return

        # Map bounds = union of all assigned cells (plus a small border)
        mnx = min(a['min_x'] for a in assignments.values()) - 5
        mxx = max(a['max_x'] for a in assignments.values()) + 5
        mny = min(a['min_y'] for a in assignments.values()) - 5
        mxy = max(a['max_y'] for a in assignments.values()) + 5
        self.map_bounds = (mnx, mxx, mny, mxy)

        swath_w = 2.0 * self.scan_altitude * self._tan_half_h
        rec_lane = swath_w * (1.0 - self.coverage_overlap)
        # Along-track capture spacing: footprint LENGTH x (1 - overlap).
        footprint_l = 2.0 * self.scan_altitude * self._tan_half_v
        capture_spacing = round(footprint_l * (1.0 - self.coverage_overlap), 1)

        self.get_logger().info(f'[FOG {header}] ' + '=' * 50)
        if self.use_zones:
            zones_str = ', '.join(
                f'zone{z}@({self.zones_x[z]},{self.zones_y[z]})'
                for z in range(len(self.zones_x)))
            self.get_logger().info(
                f'[FOG {header}] DISASTER ZONES: {zones_str} '
                f'(each a {2 * self.zone_half:.0f}x{2 * self.zone_half:.0f} m box)')
        else:
            self.get_logger().info(
                f'[FOG {header}] AREA TO COVER: '
                f'X[{self.area_min_x},{self.area_max_x}] '
                f'Y[{self.area_min_y},{self.area_max_y}]')
        self.get_logger().info(
            f'[FOG {header}] transit_alt={self.transit_altitude}m, '
            f'scan_alt={self.scan_altitude}m, scan swath={swath_w:.1f}m '
            f'-> set commander lane_spacing <= {rec_lane:.1f}m; '
            f'capture_spacing={capture_spacing}m (for stop_and_go).')
        self.get_logger().info(
            f'[FOG {header}] ASSIGNMENTS ({len(assignments)} drone(s)):')

        for drone_id, a in assignments.items():
            self.cov_grid[drone_id] = CoverageGrid(
                a['min_x'], a['max_x'], a['min_y'], a['max_y'],
                self.coverage_cell_m)
            self.coverage_pct[drone_id] = 0.0
            # Give this drone a telemetry grace window from now: it was just
            # (re)dispatched and may not have streamed fresh PX4 telemetry yet.
            self._dispatch_grace_until[drone_id] = time.time() + self.heartbeat_grace

            lat, lon = enu_to_global(a['cx'], a['cy'])
            tgt = {
                'world_x': a['cx'], 'world_y': a['cy'],
                'alt': self.scan_altitude,
                'transit_alt': self.transit_altitude,
                'capture_spacing': capture_spacing,
                'lat': lat, 'lon': lon,
                'area': {'min_x': a['min_x'], 'max_x': a['max_x'],
                         'min_y': a['min_y'], 'max_y': a['max_y']}}
            spawn_str = 'auto-calibrate'
            if drone_id in self.spawn_of:
                sx, sy = self.spawn_of[drone_id]
                tgt['spawn'] = {'x': sx, 'y': sy}
                spawn_str = f'spawn=({sx},{sy})'

            m = String()
            m.data = json.dumps({'command': 'START_MISSION', 'target': tgt})
            self.mission_cmd_publishers[drone_id].publish(m)
            self.get_logger().info(
                f'[FOG {header}]   {drone_id} -> zone {a["zone"]}: '
                f'cell X[{a["min_x"]},{a["max_x"]}] Y[{a["min_y"]},{a["max_y"]}] '
                f'transit@{self.transit_altitude}m scan@{self.scan_altitude}m '
                f'{spawn_str}')

        self.get_logger().info(f'[FOG {header}] ' + '=' * 50)

    def start_mission_callback(self, request, response):
        assignments = self._build_assignments()

        self.cov_grid = {}
        self.coverage_pct = {}
        self._cell_done = {}
        self._coverage_done = False

        # Reset failure/repartition tracking for this mission.
        self.active_drones = set(assignments.keys())
        self.failed_drones = set()
        self._mission_start_t = time.time()
        now = self._mission_start_t
        for d in assignments:
            self.last_seen[d] = now

        self._dispatch_assignments(assignments, header='START_MISSION')

        response.success = True
        response.message = (
            f'Mission started: {len(assignments)} drone(s) assigned '
            f'({"zones" if self.use_zones else "rectangle"} mode).')
        return response

    # ------------------------------------------------------------------
    # Drone-failure handling + repartition (Task 6.9 reliability)
    # ------------------------------------------------------------------
    def _failure_watchdog(self):
        """Fires at 1 Hz. Triggers the injected failure on schedule and/or
        declares a drone failed when its telemetry goes silent."""
        if self._mission_start_t is None:
            return
        now = time.time()
        elapsed = now - self._mission_start_t

        # (1) Deterministic injected failure (repeatable experiments)
        if self.fail_drone_id >= 0:
            did = drone_id_for(self.fail_drone_id)
            if did in self.active_drones and elapsed >= self.fail_after_sec:
                self._handle_drone_failure(did, f'injected @T+{elapsed:.0f}s')

        # (2) Telemetry heartbeat (realistic "unreachable").
        # Two guards make this robust on a CPU-loaded laptop, where a healthy
        # drone's PX4 telemetry can stall for several seconds:
        #   - never fail the LAST survivor (a cascade that empties the mission
        #     is never what "one drone fails" is meant to test); and
        #   - honour a per-drone grace window after (re)dispatch, since a drone
        #     that was just told to START_MISSION/REPARTITION hasn't necessarily
        #     streamed fresh telemetry yet.
        # Without these, a starved-but-alive drone gets marked failed on top of
        # the scheduled failure -> 2+ "failures" and the whole area repartitioned
        # onto a single drone (the catastrophic 4% coverage case).
        if self.heartbeat_timeout > 0 and elapsed > self.heartbeat_timeout:
            for did in list(self.active_drones):
                if len(self.active_drones) <= 1:
                    break  # keep at least one drone flying the mission
                grace = self._dispatch_grace_until.get(did, 0.0)
                if now < grace:
                    continue
                ls = self.last_seen.get(did)
                if ls is not None and (now - ls) > self.heartbeat_timeout:
                    self._handle_drone_failure(
                        did, f'no telemetry for {now - ls:.1f}s')

    def _handle_drone_failure(self, drone_id, reason):
        if drone_id not in self.active_drones:
            return  # idempotent — already handled

        # Capture progress-so-far BEFORE we tear anything down, so survivors
        # inherit already-covered ground (including the failed drone's).
        old_grids = list(self.cov_grid.values())

        self.active_drones.discard(drone_id)
        self.failed_drones.add(drone_id)
        self._cell_done.pop(drone_id, None)

        self.get_logger().error('[FOG RELIABILITY] ' + '!' * 50)
        self.get_logger().error(
            f'[FOG RELIABILITY] DRONE FAILURE: {drone_id} unreachable '
            f'({reason}). Survivors: {sorted(self.active_drones) or "NONE"}.')
        self._record_event('DRONE_FAILED', drone_id, {
            'reason': reason, 'active_remaining': sorted(self.active_drones)})
        self._publish_reliability('DRONE_FAILED', drone_id, reason)

        # Best-effort: tell the failed drone to leave the airspace (it may be
        # unreachable, in which case this simply goes nowhere).
        if self.fail_action and drone_id in self.mission_cmd_publishers:
            m = String()
            m.data = json.dumps({'command': self.fail_action})
            self.mission_cmd_publishers[drone_id].publish(m)

        if self.repartition and self.active_drones:
            self._repartition(old_grids, reason)
        elif not self.active_drones:
            self.get_logger().error(
                '[FOG RELIABILITY] no active drones left — ending mission.')
            self._finish_mission()
        else:
            # No repartition: just drop the failed drone from coverage stats so
            # completion no longer waits on its cell.
            self.cov_grid.pop(drone_id, None)
            self.coverage_pct.pop(drone_id, None)
        self.get_logger().error('[FOG RELIABILITY] ' + '!' * 50)

    def _repartition(self, old_grids, reason):
        """Repartition the whole area among the surviving drones and re-dispatch,
        carrying over already-covered ground so they don't re-sweep it."""
        active = sorted(self.active_drones)
        assignments = self._build_assignments(active_ids=active)

        # Fresh grids for the survivors' NEW (larger) cells.
        self.cov_grid = {}
        self.coverage_pct = {}
        self._cell_done = {}
        self._coverage_done = False

        self.get_logger().warn(
            f'[FOG REPARTITION] area split among {len(active)} survivor(s): '
            f'{active}')
        self._dispatch_assignments(assignments, header='REPARTITION')

        # Carry over previously-covered ground into the new grids.
        seeded = 0
        for og in old_grids:
            for (i, j) in og.covered:
                x = og.min_x + (i + 0.5) * og.cell
                y = og.min_y + (j + 0.5) * og.cell
                for g in self.cov_grid.values():
                    if g.contains(x, y):
                        g.mark_footprint(x, y, g.cell * 0.4, g.cell * 0.4)
                        seeded += 1
                        break
        for d, g in self.cov_grid.items():
            self.coverage_pct[d] = g.pct()
        if seeded:
            self.get_logger().info(
                f'[FOG REPARTITION] carried over {seeded} covered bin(s); '
                f'survivor coverage now '
                + ' '.join(f'{d}={p:.0f}%' for d, p in
                           sorted(self.coverage_pct.items())))

        self._record_event('REPARTITION', 'fog', {
            'reason': reason, 'active': active,
            'cells': {d: [a['min_x'], a['max_x'], a['min_y'], a['max_y']]
                      for d, a in assignments.items()}})
        self._publish_reliability('REPARTITION', ','.join(active), reason)

    def _publish_reliability(self, event, who, reason):
        msg = String()
        msg.data = json.dumps({
            'event': event, 'drone': who, 'reason': reason,
            'active': sorted(self.active_drones),
            'failed': sorted(self.failed_drones),
            'ts': time.time()})
        self.reliability_pub.publish(msg)

    # ------------------------------------------------------------------
    def log_stats(self):
        parts = [f'{d}[s={c["status"]} t={c["tasks"]} f={c["frames"]}]'
                 for d, c in self.stats.items()]
        self.get_logger().info('[FOG STATS] ' + ' '.join(parts))

        if self.coverage_pct:
            cov = [f'{d}={p:.0f}%{"✓" if self._cell_done.get(d) else ""}'
                   for d, p in sorted(self.coverage_pct.items())]
            overall = sum(self.coverage_pct.values()) / len(self.coverage_pct)
            self.get_logger().info('[FOG COVERAGE] ' + ' '.join(cov)
                                   + f' | mean={overall:.0f}%')

        # Publish coverage telemetry for the evaluation harness (Task 6).
        # Overall is bin-weighted (same definition as the end-of-mission report).
        if self.cov_grid:
            total_bins = sum(g.total for g in self.cov_grid.values())
            covered_bins = sum(len(g.covered) for g in self.cov_grid.values())
            overall_pct = (100.0 * covered_bins / total_bins) if total_bins else 0.0
            # Scanned area in m^2 = sum of each assigned cell's real extent (so
            # zone mode and rectangle mode both report the true area covered).
            area_m2 = sum((g.max_x - g.min_x) * (g.max_y - g.min_y)
                          for g in self.cov_grid.values())
            # Edge-tier activity: total camera frames captured/streamed by the
            # drones so far (the edge tier's characteristic work in EVERY mode).
            frames_total = sum(s['frames'] for s in self.stats.values())
            cov_msg = String()
            cov_msg.data = json.dumps({
                'overall_pct': round(overall_pct, 2),
                'covered_bins': covered_bins,
                'total_bins': total_bins,
                'area_m2': round(area_m2, 1),
                'frames_total': frames_total,
                'per_drone': {d: round(g.pct(), 2) for d, g in self.cov_grid.items()},
                'ts': time.time(),
            })
            self.coverage_pub.publish(cov_msg)

            # Mean-coverage auto-finish (Task 6): end once overall coverage of the
            # active cells reaches the threshold. Works with survivor-only cells
            # after a repartition, so the drone-failure run still ends cleanly.
            if (self.auto_finish_coverage_pct > 0.0 and not self._coverage_done
                    and overall_pct >= self.auto_finish_coverage_pct):
                # Hold the finish while a dispatched rescuer is still en route to
                # a victim, so its completion time gets recorded — but only up to
                # rescue_grace_sec so a stuck/never-arriving rescue can't hang the
                # mission forever.
                now = time.time()
                open_ids = list(self._open_rescues)
                if open_ids:
                    if self._finish_deferred_since is None:
                        self._finish_deferred_since = now
                        self.get_logger().info(
                            f'[FOG COVERAGE] {overall_pct:.1f}% >= target, but '
                            f'holding finish up to {self.rescue_grace_sec:.0f}s for '
                            f'{len(open_ids)} in-flight rescue(s): {open_ids}')
                    waited = now - self._finish_deferred_since
                    if waited < self.rescue_grace_sec:
                        return   # keep flying; check again next telemetry tick
                    self.get_logger().warn(
                        f'[FOG COVERAGE] rescue grace {self.rescue_grace_sec:.0f}s '
                        f'elapsed with {len(open_ids)} rescue(s) still open '
                        f'{open_ids} — finishing anyway.')
                self._coverage_done = True
                self.get_logger().info(
                    f'[FOG COVERAGE] overall {overall_pct:.1f}% >= '
                    f'{self.auto_finish_coverage_pct:.0f}% target — '
                    f'mission complete, finishing.')
                self._finish_mission()

        cur_len = len(self.event_buffer)
        if cur_len != self._prev_buffer_len:
            self.get_logger().info(
                f'[FOG BUFFER] {cur_len} events buffered '
                f'(overflow_drops={self.events_dropped_on_overflow})')
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