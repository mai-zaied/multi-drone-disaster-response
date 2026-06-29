"""
decision_node.py  —  Task 5: Threat Decision Logic (core intelligence & coordination)

This is the swarm's decision engine. It closes the loop:

    Drone -> Detection (Task 4) -> Offloading -> Fog -> DECISION NODE -> Drone Action

It is a fog-tier ROS2 node (runs alongside fog_server, separate process) that:

  5.4  Subscribes to detection results (Task 4's /fog/victim_alerts JSON alerts and,
       as a fallback, VICTIM_DETECTION Task messages on /{drone_id}/task/fog).
  5.5  Aggregates detections into a global event map (spatially clustered so several
       reports of one victim collapse into one event).
  5.6  Prioritises events:  score = confidence*W_CONF + norm_reports*W_REPORTS
  5.7  Selects the best drone per event (distance + battery + availability).
  5.8  Generates commands: GO_TO / HOVER / SCAN_AREA / RETURN_HOME.
  5.9  Sends each command to the correct drone on /{drone_id}/mission_command.
  5.11 Coordinates multiple drones (one drone per victim; extras keep searching).
  5.12 Avoids conflict/redundancy (unique assignment, no idle-while-work-pending).
  5.13 Consumes feedback on /{drone_id}/mission_feedback to mark events resolved.
  5.14 Handles drone failure (reassigns the orphaned event, sends the failing drone home).
  5.15 Logs every decision with a clear, greppable label.
  5.16 Tracks response time, completion time, and drone utilisation.

WHY A DERIVED LOCATION:
Task 4's detection messages carry image-pixel bounding boxes + confidence, but no
world location. The decision node tracks each drone's live PX4 local position
(VehicleLocalPosition, NED relative to spawn) and converts it to world ENU using the
known spawn points. A detection from droneX is tagged with droneX's current world
position — the drone is flying over the victim, so its position is the victim's
location to within the camera footprint. This is also exactly the data nearest-drone
selection (5.7) needs, so spatial reasoning is centralised here.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from task_msgs.msg import Task

from fog_node.drone_naming import drone_id_for, px4_topic_for


# ----------------------------------------------------------------------
# Tunables (all overridable as ROS parameters)
# ----------------------------------------------------------------------
# Spawn points (ENU East = x, ENU North = y) — must match the PX4_GZ_MODEL_POSE
# used at launch and DEFAULT_SPAWNS in drone_commander.py.
DEFAULT_SPAWNS = {
    0: (18.0, 25.0),
    1: (23.0, 25.0),
    2: (30.0, 25.0),
}

W_CONF = 0.7              # 5.6 weight on detection confidence
W_REPORTS = 0.3           # 5.6 weight on number of corroborating reports
REPORT_SATURATION = 3     # this many reports counts as "fully corroborated"

CLUSTER_RADIUS_M = 6.0    # 5.5 detections within this distance = same victim
MIN_CONFIDENCE = 0.40     # below this -> SCAN_AREA (re-scan) instead of dispatch
NOISE_CONFIDENCE = 0.20   # below this -> ignore entirely

SELECT_W_DIST = 0.6       # 5.7 selection: weight on distance
SELECT_W_BATTERY = 0.4    # 5.7 selection: weight on (1 - battery)
DIST_NORMALISER_M = 100.0 # distance scale for normalisation

LOW_BATTERY_FRAC = 0.20   # at/below this a drone is treated as failing
ARRIVAL_RADIUS_M = 3.0    # event considered reached within this distance
SCAN_ALTITUDE = 12.0      # default GO_TO altitude (m above spawn)

COORDINATE_PERIOD_SEC = 1.0   # how often the planner runs
STATS_PERIOD_SEC = 5.0        # metrics log cadence


class Event:
    """A clustered real-world event the swarm may need to act on."""
    _next_id = 0

    def __init__(self, world_x, world_y, confidence, drone_id, now):
        Event._next_id += 1
        self.event_id = f'E{Event._next_id:03d}'
        self.world_x = world_x
        self.world_y = world_y
        self.confidence = confidence            # max confidence seen
        self.reporting_drones = {drone_id}
        self.num_reports = 1
        self.status = 'UNASSIGNED'              # UNASSIGNED / ASSIGNED / RESOLVED
        self.assigned_drone = None
        self.first_detection_at = now           # 5.16 response-time clock start
        self.assigned_at = None
        self.resolved_at = None
        self.first_command_at = None

    def merge(self, world_x, world_y, confidence, drone_id):
        # Confidence-weighted position update keeps the centroid sensible.
        total = self.confidence + confidence
        if total > 0:
            self.world_x = (self.world_x * self.confidence + world_x * confidence) / total
            self.world_y = (self.world_y * self.confidence + world_y * confidence) / total
        self.confidence = max(self.confidence, confidence)
        self.reporting_drones.add(drone_id)
        self.num_reports += 1

    def priority(self):
        norm_reports = min(self.num_reports / REPORT_SATURATION, 1.0)
        return W_CONF * self.confidence + W_REPORTS * norm_reports


class DroneState:
    """Live view of one drone, assembled from PX4 + feedback."""
    def __init__(self, drone_id, spawn_x, spawn_y):
        self.drone_id = drone_id
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.world_x = spawn_x          # best-known world position (ENU)
        self.world_y = spawn_y
        self.pos_valid = False
        self.armed = False
        self.battery = 1.0              # 0..1; updated from feedback / failing flag
        self.failing = False
        self.assigned_event = None      # event_id this drone is committed to
        self.last_command = None        # (command, round(x,1), round(y,1)) for dedup
        self.last_feedback_state = 'UNKNOWN'

    def available(self):
        # Flyable, not failing, not already committed.
        return (not self.failing) and self.battery > LOW_BATTERY_FRAC \
            and self.assigned_event is None


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # ---- Parameters ----
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('scan_altitude', SCAN_ALTITUDE)
        self.num_drones = int(self.get_parameter('num_drones').value)
        self.scan_altitude = float(self.get_parameter('scan_altitude').value)
        if self.num_drones < 1:
            raise ValueError(f'num_drones must be >= 1, got {self.num_drones}')

        # Per-drone spawn overrides: spawn_x_0, spawn_y_0, spawn_x_1, ...
        self.drones = {}
        for inst in range(self.num_drones):
            did = drone_id_for(inst)
            dsx, dsy = DEFAULT_SPAWNS.get(inst, (0.0, 0.0))
            self.declare_parameter(f'spawn_x_{inst}', dsx)
            self.declare_parameter(f'spawn_y_{inst}', dsy)
            sx = float(self.get_parameter(f'spawn_x_{inst}').value)
            sy = float(self.get_parameter(f'spawn_y_{inst}').value)
            self.drones[did] = DroneState(did, sx, sy)

        # ---- QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ---- Detection inputs (5.4) ----
        # Primary: Task 4's fog-side victim alerts.
        self.create_subscription(
            String, '/fog/victim_alerts', self.victim_alert_callback, 10)

        # Per-drone inputs
        self.command_pubs = {}
        for inst in range(self.num_drones):
            did = drone_id_for(inst)

            # Fallback detection path: drone-side VICTIM_DETECTION Tasks, and
            # STATUS_REPORT (carries the drone_failing flag we use for 5.14).
            self.create_subscription(
                Task, f'/{did}/task/fog',
                lambda msg, d=did: self.task_callback(msg, d), 10)

            # Live position -> world ENU (for victim location + nearest-drone).
            self.create_subscription(
                VehicleLocalPosition,
                px4_topic_for(inst, 'vehicle_local_position_v1'),
                lambda msg, d=did: self.local_pos_callback(msg, d), px4_qos)

            # Armed / availability.
            self.create_subscription(
                VehicleStatus, px4_topic_for(inst, 'vehicle_status_v1'),
                lambda msg, d=did: self.status_callback(msg, d), px4_qos)

            # Feedback loop (5.13).
            self.create_subscription(
                String, f'/{did}/mission_feedback',
                lambda msg, d=did: self.feedback_callback(msg, d), 10)

            # Command output (5.9) — same topic drone_commander already listens on.
            self.command_pubs[did] = self.create_publisher(
                String, f'/{did}/mission_command', 10)

        # Structured decision stream for logging / visualisation (5.15).
        self.decision_log_pub = self.create_publisher(String, '/fog/decision_log', 10)

        # ---- Event store (5.5) ----
        self.events = []                 # list[Event]
        self._recent_dedup = {}          # (drone_id, frame) -> ts, to skip dup alerts

        # ---- Metrics (5.16) ----
        self.metrics = {
            'detections_seen': 0,
            'events_created': 0,
            'events_resolved': 0,
            'commands_sent': 0,
            'response_times': [],        # detection -> first command (s)
            'completion_times': [],      # assignment -> arrival (s)
        }

        # ---- Planner + stats timers ----
        self.create_timer(COORDINATE_PERIOD_SEC, self.coordinate)
        self.create_timer(STATS_PERIOD_SEC, self.log_stats)

        self.get_logger().info(
            f'[DECISION] engine up for {self.num_drones} drone(s). '
            f'priority = {W_CONF}*conf + {W_REPORTS}*reports, '
            f'cluster={CLUSTER_RADIUS_M}m, min_conf={MIN_CONFIDENCE}')
        for did, d in self.drones.items():
            self.get_logger().info(
                f'[DECISION] {did}: spawn ENU=({d.spawn_x}, {d.spawn_y})')

    # ==================================================================
    # Inputs
    # ==================================================================
    def local_pos_callback(self, msg: VehicleLocalPosition, drone_id: str):
        d = self.drones[drone_id]
        # PX4 NED local (x=North, y=East) relative to spawn -> world ENU.
        d.world_x = d.spawn_x + float(msg.y)     # East
        d.world_y = d.spawn_y + float(msg.x)     # North
        d.pos_valid = bool(msg.xy_valid)

    def status_callback(self, msg: VehicleStatus, drone_id: str):
        self.drones[drone_id].armed = (msg.arming_state == 2)

    def feedback_callback(self, msg: String, drone_id: str):
        """5.13 — drones report their state back; we close events here."""
        try:
            fb = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        d = self.drones[drone_id]
        state = fb.get('state', 'UNKNOWN')
        d.last_feedback_state = state

        if 'battery' in fb and fb['battery'] is not None:
            d.battery = float(fb['battery'])
        if d.battery <= LOW_BATTERY_FRAC:
            d.failing = True

        if state in ('FAILED', 'RETURNING'):
            d.failing = d.failing or (state == 'FAILED')

        # Arrival closes the assigned event (5.13 + 5.16 completion time).
        if state in ('ARRIVED', 'HOLDING') and d.assigned_event is not None:
            ev = self._event_by_id(d.assigned_event)
            if ev is not None and ev.status != 'RESOLVED':
                dist = self._dist(d.world_x, d.world_y, ev.world_x, ev.world_y)
                if dist <= ARRIVAL_RADIUS_M or state == 'HOLDING':
                    self._resolve_event(ev, d)

    def task_callback(self, msg: Task, drone_id: str):
        """Fallback detection path + drone_failing signal."""
        if msg.task_type == 'STATUS_REPORT':
            try:
                payload = json.loads(msg.payload) if msg.payload else {}
            except json.JSONDecodeError:
                payload = {}
            if payload.get('drone_failing'):
                self.drones[drone_id].failing = True
                self.drones[drone_id].battery = min(self.drones[drone_id].battery, 0.1)
            return

        if msg.task_type != 'VICTIM_DETECTION':
            return
        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except json.JSONDecodeError:
            return
        conf = self._max_conf(payload.get('detections', []))
        n = int(payload.get('num_persons', len(payload.get('detections', [])) or 1))
        self._ingest_detection(drone_id, conf, n, payload.get('frame_seq'))

    def victim_alert_callback(self, msg: String):
        """Primary detection path: Task 4 /fog/victim_alerts JSON."""
        try:
            alert = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        drone_id = alert.get('drone_id')
        if drone_id not in self.drones:
            return
        conf = self._max_conf(alert.get('detections', []))
        n = int(alert.get('num_persons', 1))
        self._ingest_detection(drone_id, conf, n, alert.get('frame'))

    # ==================================================================
    # Detection ingest + clustering (5.5)
    # ==================================================================
    def _ingest_detection(self, drone_id, confidence, num_persons, frame):
        if confidence < NOISE_CONFIDENCE:
            return
        # Cheap dedup: same drone + same frame within 2 s -> skip.
        key = (drone_id, frame)
        now = time.time()
        if frame is not None and now - self._recent_dedup.get(key, 0.0) < 2.0:
            return
        if frame is not None:
            self._recent_dedup[key] = now

        d = self.drones[drone_id]
        wx, wy = d.world_x, d.world_y
        self.metrics['detections_seen'] += 1

        self.get_logger().warn(
            f'[DECISION DETECT] {drone_id} reports {num_persons} person(s) '
            f'conf={confidence:.2f} at world=({wx:.1f}, {wy:.1f})')

        # Cluster into an existing nearby active event, else create one.
        ev = self._nearest_active_event(wx, wy)
        if ev is not None:
            ev.merge(wx, wy, confidence, drone_id)
            self._emit_decision('EVENT_UPDATED', event=ev)
            self.get_logger().info(
                f'[DECISION EVENT] {ev.event_id} updated: reports={ev.num_reports} '
                f'conf={ev.confidence:.2f} priority={ev.priority():.2f}')
        else:
            ev = Event(wx, wy, confidence, drone_id, now)
            self.events.append(ev)
            self.metrics['events_created'] += 1
            self._emit_decision('EVENT_CREATED', event=ev)
            self.get_logger().info(
                f'[DECISION EVENT] {ev.event_id} created at ({wx:.1f}, {wy:.1f}) '
                f'conf={ev.confidence:.2f} priority={ev.priority():.2f}')

    def _nearest_active_event(self, wx, wy):
        best, best_d = None, CLUSTER_RADIUS_M
        for ev in self.events:
            if ev.status == 'RESOLVED':
                continue
            dd = self._dist(wx, wy, ev.world_x, ev.world_y)
            if dd <= best_d:
                best, best_d = ev, dd
        return best

    # ==================================================================
    # Planner: prioritise (5.6) -> select (5.7) -> command (5.8/5.9),
    # with conflict/redundancy handling (5.12) and failure handling (5.14).
    # ==================================================================
    def coordinate(self):
        # 5.14: a drone that just failed forfeits its event so it can be reassigned.
        for d in self.drones.values():
            if d.failing and d.assigned_event is not None:
                ev = self._event_by_id(d.assigned_event)
                if ev is not None and ev.status != 'RESOLVED':
                    self.get_logger().warn(
                        f'[DECISION FAILURE] {d.drone_id} failing — releasing '
                        f'{ev.event_id} for reassignment')
                    ev.status = 'UNASSIGNED'
                    ev.assigned_drone = None
                d.assigned_event = None
                self._send_command(d.drone_id, 'RETURN_HOME', {})

        # 5.6: rank unassigned, actionable events by priority (desc).
        pending = [e for e in self.events if e.status == 'UNASSIGNED']
        pending.sort(key=lambda e: e.priority(), reverse=True)

        for ev in pending:
            # Low-confidence event -> re-scan rather than commit a rescuer (5.2).
            if ev.confidence < MIN_CONFIDENCE:
                drone = self._select_drone(ev, allow_low_conf=True)
                if drone is not None:
                    self._assign(ev, drone, action='SCAN_AREA')
                continue

            drone = self._select_drone(ev)
            if drone is None:
                self.get_logger().info(
                    f'[DECISION WAIT] {ev.event_id} (p={ev.priority():.2f}) '
                    f'has no free drone — queued')
                continue
            self._assign(ev, drone, action='GO_TO')

    def _select_drone(self, ev, allow_low_conf=False):
        """5.7 — lowest-cost available drone: distance + battery."""
        best, best_cost = None, math.inf
        for d in self.drones.values():
            if not d.available():
                continue
            dist = self._dist(d.world_x, d.world_y, ev.world_x, ev.world_y)
            norm_dist = min(dist / DIST_NORMALISER_M, 1.0)
            cost = SELECT_W_DIST * norm_dist + SELECT_W_BATTERY * (1.0 - d.battery)
            if cost < best_cost:
                best, best_cost = d, cost
        if best is not None:
            best._last_cost = best_cost   # for logging
        return best

    def _assign(self, ev, drone, action):
        now = time.time()
        ev.status = 'ASSIGNED'
        ev.assigned_drone = drone.drone_id
        ev.assigned_at = now
        drone.assigned_event = ev.event_id

        if ev.first_command_at is None:
            ev.first_command_at = now
            self.metrics['response_times'].append(now - ev.first_detection_at)

        self.get_logger().warn(
            f'[DECISION ASSIGN] {ev.event_id} -> {drone.drone_id} '
            f'(action={action}, cost={getattr(drone, "_last_cost", 0):.2f}, '
            f'priority={ev.priority():.2f})')
        self._emit_decision('ASSIGNED', event=ev, drone=drone.drone_id, action=action)

        self._send_command(drone.drone_id, action, {
            'world_x': round(ev.world_x, 2),
            'world_y': round(ev.world_y, 2),
            'alt': self.scan_altitude,
            'event_id': ev.event_id,
        })

    # ==================================================================
    # Command generation + dispatch (5.8 / 5.9 / 5.12 dedup)
    # ==================================================================
    def _send_command(self, drone_id, command, target):
        d = self.drones[drone_id]
        sig = (command, round(target.get('world_x', 0), 1),
               round(target.get('world_y', 0), 1))
        if d.last_command == sig:
            return                        # 5.12: don't re-spam identical commands
        d.last_command = sig

        cmd = {'command': command}
        if target:
            cmd['target'] = target
        msg = String()
        msg.data = json.dumps(cmd)
        self.command_pubs[drone_id].publish(msg)
        self.metrics['commands_sent'] += 1
        self.get_logger().info(f'[DECISION CMD] {drone_id} <- {command} {target}')

    # ==================================================================
    # Resolution + metrics (5.13 / 5.16)
    # ==================================================================
    def _resolve_event(self, ev, drone):
        ev.status = 'RESOLVED'
        ev.resolved_at = time.time()
        # arrival   = assignment -> arrival (what we already tracked internally)
        # completion = detection  -> arrival (Task 6 "task completion time")
        # response   = detection  -> first command
        arrival = (ev.resolved_at - ev.assigned_at
                   if ev.assigned_at is not None else None)
        completion = (ev.resolved_at - ev.first_detection_at
                      if ev.first_detection_at is not None else None)
        response = (ev.first_command_at - ev.first_detection_at
                    if ev.first_command_at is not None else None)
        if arrival is not None:
            self.metrics['completion_times'].append(arrival)
        self.metrics['events_resolved'] += 1
        drone.assigned_event = None
        self.get_logger().warn(
            f'[DECISION RESOLVED] {ev.event_id} reached by {drone.drone_id} '
            f'in {arrival if arrival is not None else 0.0:.1f}s '
            f'(completion={completion if completion is not None else 0.0:.1f}s)')
        self._emit_decision('RESOLVED', event=ev, drone=drone.drone_id, extra={
            'arrival_time': round(arrival, 4) if arrival is not None else None,
            'completion_time': round(completion, 4) if completion is not None else None,
            'response_time': round(response, 4) if response is not None else None,
        })
        # Send the rescuer into a hold over the victim.
        self._send_command(drone.drone_id, 'HOVER', {
            'world_x': round(ev.world_x, 2),
            'world_y': round(ev.world_y, 2),
            'alt': self.scan_altitude,
        })

    def log_stats(self):
        active = sum(1 for e in self.events if e.status != 'RESOLVED')
        assigned = sum(1 for d in self.drones.values() if d.assigned_event)
        util = assigned / self.num_drones if self.num_drones else 0.0
        rt = self.metrics['response_times']
        ct = self.metrics['completion_times']
        avg_rt = sum(rt) / len(rt) if rt else 0.0
        avg_ct = sum(ct) / len(ct) if ct else 0.0
        self.get_logger().info(
            f'[DECISION STATS] events: active={active} '
            f'resolved={self.metrics["events_resolved"]} '
            f'created={self.metrics["events_created"]} | '
            f'utilisation={util*100:.0f}% ({assigned}/{self.num_drones}) | '
            f'avg_response={avg_rt:.2f}s avg_completion={avg_ct:.2f}s | '
            f'cmds={self.metrics["commands_sent"]}')

    # ==================================================================
    # Helpers
    # ==================================================================
    def _emit_decision(self, kind, event=None, drone=None, action=None, extra=None):
        rec = {'kind': kind, 'ts': time.time()}
        if event is not None:
            rec.update({
                'event_id': event.event_id,
                'world_x': round(event.world_x, 2),
                'world_y': round(event.world_y, 2),
                'confidence': round(event.confidence, 3),
                'num_reports': event.num_reports,
                'priority': round(event.priority(), 3),
                'status': event.status,
                # The drone(s) that REPORTED this event. The metrics collector
                # uses this to link the original detection row to the event,
                # even when the dispatched (nearest) drone differs from the
                # detector. Without it, detector != rescuer loses completion.
                'reporting_drones': sorted(event.reporting_drones),
            })
        if drone is not None:
            rec['drone'] = drone
        if action is not None:
            rec['action'] = action
        if extra:
            rec.update(extra)
        m = String()
        m.data = json.dumps(rec)
        self.decision_log_pub.publish(m)

    def _event_by_id(self, event_id):
        for ev in self.events:
            if ev.event_id == event_id:
                return ev
        return None

    @staticmethod
    def _max_conf(detections):
        if not detections:
            return 0.0
        return max(float(d.get('confidence', 0.0)) for d in detections)

    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
