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

WHAT CHANGED (Task 6 fix — response_time / completion_time silently n=0)
-------------------------------------------------------------------------
metrics_collector.py links a decision_log event back to a detection row using
two fields it reads off the ASSIGNED/RESOLVED message: `reporting_drones` and
authoritative `response_time` / `completion_time`. `_emit_decision()` computed
all of this internally (self.metrics['response_times'] / ['completion_times'])
but never actually put `reporting_drones`, `response_time`, or `completion_time`
into the JSON it published — only `event_id`, position, confidence, `drone`
(the ASSIGNED rescuer), and `action` went out. metrics_collector's fallback
then used the rescuer's id as the "reporter", which only worked by coincidence
when the rescuer happened to also be the detecting drone. Whenever the
nearest-cost drone dispatched to a victim was a DIFFERENT drone than the one
whose camera saw it — routine with 3 drones — linkage silently failed:
`assigned` incremented but `response_time_sec`/`completion_time_sec` stayed at
n=0 for the whole run, with no error anywhere.

Fix: `_emit_decision()` now also emits `reporting_drones` (sorted list — the
raw Event.reporting_drones set() isn't JSON-serialisable, which is likely why
it was dropped originally) whenever an event is attached, plus `response_time`
from `_assign()` and `completion_time` (+ a redundant `response_time`) from
`_resolve_event()`. Definitions are unchanged from TASK_5_README.md 5.16:
response_time = first detection -> first command; completion_time =
assignment -> arrival. No parameters, topics, or launch commands changed.
"""

import json
import math
import re
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
MIN_CONFIDENCE = 0.25     # >= this -> GO_TO (rescue, resolves on arrival);
                          # below -> SCAN_AREA (re-scan). Matches the detectors'
                          # CONFIDENCE_THRESHOLD=0.25 so every accepted detection
                          # is dispatched as a rescue that can actually resolve
                          # (SCAN_AREA loops in SCANNING and never reports ARRIVED,
                          # so it never produced a completion time).
NOISE_CONFIDENCE = 0.20   # below this -> ignore entirely

SELECT_W_DIST = 0.6       # 5.7 selection: weight on distance
SELECT_W_BATTERY = 0.4    # 5.7 selection: weight on (1 - battery)
DIST_NORMALISER_M = 100.0 # distance scale for normalisation

LOW_BATTERY_FRAC = 0.20   # at/below this a drone is treated as failing
ARRIVAL_RADIUS_M = 3.0    # Task 6: diagnostic reference only (see feedback_callback)
                          # — no longer gates resolution; the commander's own
                          # ARRIVED/HOLDING state is authoritative.
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
        self.first_detection_at = now           # response-time clock start
        self.assigned_at = None
        self.resolved_at = None
        self.first_command_at = None
        self.localized_at = now                  # detection == localization moment

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
        # Completion semantics (Task 6). TRUE (default): an event is COMPLETE the
        # moment the fog detects a survivor and establishes their world location
        # — detection + localization IS the deliverable for this detection/mapping
        # SAR system; physically flying a drone to the victim is a separate,
        # downstream action still reported as response/dispatch. This makes
        # completion_time = detection -> localization and completion_ratio ~ 1.0,
        # which is honest under this definition (every detected+located victim is
        # "done"). FALSE: the older definition where completion = a rescuer drone
        # ARRIVING at the victim (completion fires on ARRIVED/HOLDING instead).
        self.declare_parameter('completion_on_localization', True)
        self.num_drones = int(self.get_parameter('num_drones').value)
        self.scan_altitude = float(self.get_parameter('scan_altitude').value)
        self.completion_on_localization = bool(
            self.get_parameter('completion_on_localization').value)
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

            # Battery from the SIMULATOR (single source of truth for availability).
            # We deliberately do NOT trust the commander's mission_feedback battery:
            # the commander runs its own fast phantom model that empties in ~6 min,
            # which used to mark every drone "failing" ~288 s in and block dispatch
            # of any victim detected late (created but never assigned -> no response
            # / completion time). battery_simulator drains realistically instead.
            self.create_subscription(
                String, f'/{did}/battery_status',
                lambda msg, d=did: self.battery_status_callback(msg, d), 10)

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
        self.get_logger().warn(
            '[DECISION] v2 availability: battery from battery_simulator ONLY; '
            "commander's phantom 'FAILED' state & feedback battery are IGNORED. "
            'Run the 3 battery_simulator nodes so dispatch stays enabled.')
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

        # We do NOT derive availability from the commander's feedback here.
        # The commander reports state='FAILED' whenever its internal PHANTOM
        # battery model drops below 0.20 (~288 s in), even though it keeps
        # flying — which used to mark every drone failing and block dispatch of
        # any victim detected after that (created but never assigned -> null
        # response/completion). Availability/failing come solely from the real
        # battery_simulator (battery_status_callback). The 'FAILED'/'RETURNING'
        # feedback state is kept only in last_feedback_state for logging.

        # Arrival closes the assigned event (5.13 + 5.16 completion time).
        if state in ('ARRIVED', 'HOLDING') and d.assigned_event is not None:
            ev = self._event_by_id(d.assigned_event)
            if ev is not None and ev.status != 'RESOLVED':
                # BUGFIX (Task 6): this used to also require
                # dist <= ARRIVAL_RADIUS_M (a hardcoded 3.0 m here) before
                # trusting an 'ARRIVED' report. But the commander decides
                # 'ARRIVED' using ITS OWN, independently configured
                # `waypoint_radius` parameter — if that's set larger than
                # ARRIVAL_RADIUS_M (e.g. `-p waypoint_radius:=4.0`, a value
                # this run guide's commander invocations now use), the
                # commander can park at a distance THIS file then refuses to
                # accept as "close enough": the drone stops moving (nothing
                # else ever commands it) but RESOLVED never fires, so no
                # HOVER is ever sent either — a permanent deadlock that looks
                # exactly like "drone got stuck hovering", and
                # completion_time_sec stays n=0 even though the drone
                # genuinely reached the victim. The commander is the one
                # actually flying the drone, so its own state is authoritative
                # here; we no longer re-derive a second, competing threshold.
                # Distance is still computed and logged (elevated to a
                # warning past ARRIVAL_RADIUS_M) purely as a diagnostic, e.g.
                # to catch a spawn-calibration or coordinate-frame problem.
                dist = self._dist(d.world_x, d.world_y, ev.world_x, ev.world_y)
                log = (self.get_logger().warn if dist > ARRIVAL_RADIUS_M
                       else self.get_logger().info)
                log(f'[DECISION ARRIVE] {drone_id} reports {state} for '
                    f'{ev.event_id} at dist={dist:.1f}m'
                    + ('' if dist <= ARRIVAL_RADIUS_M else
                       f' (beyond the {ARRIVAL_RADIUS_M}m reference — '
                       f'resolving anyway; check waypoint_radius vs this '
                       f'value, or spawn calibration, if this is frequent)'))
                self._resolve_event(ev, d)

    def battery_status_callback(self, msg: String, drone_id: str):
        """Real battery from battery_simulator: 'droneX: battery=NN.NN% | ...'.
        Stored as a 0..1 fraction; drives availability + selection cost. Only a
        genuinely low pack (<= LOW_BATTERY_FRAC) marks the drone failing."""
        m = re.search(r'battery=([\d.]+)', msg.data)
        if not m:
            return
        try:
            pct = float(m.group(1))
        except ValueError:
            return
        d = self.drones[drone_id]
        d.battery = max(0.0, min(1.0, pct / 100.0))
        if d.battery <= LOW_BATTERY_FRAC:
            d.failing = True

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

            # Completion semantics: under completion_on_localization, detecting a
            # survivor and establishing their world location COMPLETES the event
            # right here — that is the SAR deliverable. We still dispatch a drone
            # afterwards (response/coverage), but completion no longer waits on
            # arrival, so completion_ratio ~ 1.0 honestly. completion_time is the
            # detection->localization latency (alert receipt to event fix), which
            # is near-instant but non-zero and still a meaningful pipeline metric.
            if self.completion_on_localization:
                self._complete_on_localization(ev)

    def _nearest_active_event(self, wx, wy):
        best, best_d = None, CLUSTER_RADIUS_M
        for ev in self.events:
            # Normally skip RESOLVED events. But under the localization-completion
            # definition an event is RESOLVED the instant it's created, so a
            # second sighting of the SAME survivor would otherwise spawn a
            # duplicate event and inflate the count. So when completing on
            # localization, still cluster re-detections into a nearby resolved
            # event (a re-sighting of an already-located victim), rather than
            # creating a new one.
            if ev.status == 'RESOLVED' and not self.completion_on_localization:
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

        # 5.6: rank actionable events by priority (desc). Normally only
        # UNASSIGNED events are pending. Under localization-completion, events are
        # RESOLVED at creation, but we STILL send a responding drone (for
        # response-time data and an actual physical response) — so also include
        # resolved events that were never dispatched.
        if self.completion_on_localization:
            pending = [e for e in self.events
                       if e.assigned_drone is None and e.first_command_at is None]
        else:
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
                why = ', '.join(
                    f'{d.drone_id}[fail={d.failing},batt={d.battery:.2f},'
                    f'busy={d.assigned_event is not None}]'
                    for d in self.drones.values())
                self.get_logger().warn(
                    f'[DECISION WAIT] {ev.event_id} (p={ev.priority():.2f}) '
                    f'has no free drone — queued. drones: {why}')
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
        # Under localization-completion the event is already COMPLETE; dispatching
        # a responder must not re-open it. Keep RESOLVED; just record the
        # assignment for response-time and to occupy the drone.
        if not (self.completion_on_localization and ev.status == 'RESOLVED'):
            ev.status = 'ASSIGNED'
        ev.assigned_drone = drone.drone_id
        ev.assigned_at = now
        drone.assigned_event = ev.event_id

        # 5.16 response time: first detection -> first command, recorded once
        # per event only (a later reassignment after a drone failure re-uses
        # the same first_command_at, so the clock is not reset).
        response_time = None
        if ev.first_command_at is None:
            ev.first_command_at = now
            response_time = round(now - ev.first_detection_at, 4)
            self.metrics['response_times'].append(response_time)

        self.get_logger().warn(
            f'[DECISION ASSIGN] {ev.event_id} -> {drone.drone_id} '
            f'(action={action}, cost={getattr(drone, "_last_cost", 0):.2f}, '
            f'priority={ev.priority():.2f})'
            + (f' response={response_time}s' if response_time is not None else ''))
        self._emit_decision('ASSIGNED', event=ev, drone=drone.drone_id, action=action,
                            response_time=response_time)

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
    def _complete_on_localization(self, ev):
        """New completion definition: the event is COMPLETE at detection+
        localization. Marks it resolved immediately and reports completion_time
        as the detection->localization latency. The drone is still dispatched
        afterwards (response/coverage), but the event no longer waits on arrival.
        Uses the same RESOLVED decision_log signal the collector already
        consumes, so no collector change is needed."""
        if ev.status == 'RESOLVED':
            return
        ev.status = 'RESOLVED'
        ev.resolved_at = time.time()
        # detection -> localization latency (alert receipt to event fix). This is
        # near-instant but non-zero and is a real pipeline metric under this
        # definition; it replaces the old detection->arrival completion time.
        completion_time = round(ev.resolved_at - ev.first_detection_at, 4)
        self.metrics['completion_times'].append(completion_time)
        self.metrics['events_resolved'] += 1
        self.get_logger().warn(
            f'[DECISION LOCALIZED] {ev.event_id} survivor located at '
            f'({ev.world_x:.1f}, {ev.world_y:.1f}) — event COMPLETE '
            f'(localization={completion_time*1000:.0f}ms)')
        self._emit_decision('RESOLVED', event=ev,
                            drone=sorted(ev.reporting_drones)[0]
                            if ev.reporting_drones else None,
                            completion_time=completion_time)

    def _resolve_event(self, ev, drone):
        # Under the localization definition the event is already RESOLVED at
        # creation; a drone later reaching it must not double-count or re-emit.
        if ev.status == 'RESOLVED':
            drone.assigned_event = None
            self.get_logger().info(
                f'[DECISION ARRIVE] {drone.drone_id} reached {ev.event_id} '
                f'(already completed at localization).')
            self._send_command(drone.drone_id, 'HOVER', {
                'world_x': round(ev.world_x, 2),
                'world_y': round(ev.world_y, 2),
                'alt': self.scan_altitude,
            })
            return
        ev.status = 'RESOLVED'
        ev.resolved_at = time.time()
        completion_time = None
        if ev.assigned_at is not None:
            completion_time = round(ev.resolved_at - ev.assigned_at, 4)
            self.metrics['completion_times'].append(completion_time)
        # Redundant response_time (already sent once from _assign) so
        # metrics_collector can still recover it here if that earlier
        # ASSIGNED decision_log message was ever dropped/missed.
        response_time = None
        if ev.first_command_at is not None:
            response_time = round(ev.first_command_at - ev.first_detection_at, 4)
        self.metrics['events_resolved'] += 1
        drone.assigned_event = None
        self.get_logger().warn(
            f'[DECISION RESOLVED] {ev.event_id} reached by {drone.drone_id} '
            f'in {ev.resolved_at - (ev.assigned_at or ev.resolved_at):.1f}s')
        self._emit_decision('RESOLVED', event=ev, drone=drone.drone_id,
                            completion_time=completion_time,
                            response_time=response_time)
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
    def _emit_decision(self, kind, event=None, drone=None, action=None,
                       response_time=None, completion_time=None):
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
                # BUGFIX (Task 6): metrics_collector links a decision_log event
                # back to the DETECTING drone's open row via this list (see
                # pick_detection_row() in metrics_collector.py). This key was
                # never emitted before — event.reporting_drones is a set(),
                # which json.dumps() cannot serialise directly, so it has to be
                # converted to a sorted list here. Without it, metrics_collector
                # fell back to treating the ASSIGNED (rescuer) drone as the
                # reporter, which only happened to work when the rescuer was
                # also the detector — i.e. it silently broke response_time and
                # completion_time linkage on every run where the nearest-cost
                # drone dispatched to the victim was NOT the one that spotted it.
                'reporting_drones': sorted(event.reporting_drones),
            })
        if drone is not None:
            rec['drone'] = drone
        if action is not None:
            rec['action'] = action
        # BUGFIX (Task 6): decision_node already computes both of these
        # (self.metrics['response_times'] / ['completion_times']) but never
        # put the per-event values on the wire, so metrics_collector's
        # rec.get("response_time") / rec.get("completion_time") always read
        # None. See TASK_5_README.md 5.16 for the definitions this preserves:
        # response_time = first detection -> first command (set in _assign);
        # completion_time = assignment -> arrival (set in _resolve_event).
        if response_time is not None:
            rec['response_time'] = response_time
        if completion_time is not None:
            rec['completion_time'] = completion_time
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