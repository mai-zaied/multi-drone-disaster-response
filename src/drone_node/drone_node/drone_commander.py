"""
drone_commander.py  —  MERGED (area-partitioning phased flight  +  Task 5 feedback)

This is the integration of two branches:

  * area-partitioning  (Mai): phased offboard flight executor with CONTINUOUS
    full-area lawnmower scan, two-altitude transit/scan profile, OFFBOARD
    auto-recovery, lead-capped ("carrot") setpoints, and spawn auto-calibration.
    Drives the fog's START_MISSION (area + spawn + transit_alt) and RTL.

  * task5             (Doaa): the decision-node action interface — extra verbs
    GO_TO / HOVER / SCAN_AREA / RETURN_HOME, a 1 Hz /{drone_id}/mission_feedback
    publisher, and a simple battery model for battery-aware drone selection and
    the drone-failure scenario.

Both command vocabularies are accepted on /{drone_id}/mission_command and BOTH
are routed through the SAME validated phased flight core, so the decision node's
GO_TO/HOVER/SCAN_AREA get the same gentle, failsafe-safe behaviour as
START_MISSION (climb vertically first, lead-capped setpoints, OFFBOARD recovery).

PHASES (per mission):
    CLIMB    -> straight up over the spawn to the high transit altitude
    TRANSIT  -> across to the first waypoint at transit altitude (obstacle-safe)
    DESCEND  -> sink to the low scan altitude over the cell's first corner
    SCAN     -> boustrophedon ("lawnmower") sweep, looping until RTL
    HOLD     -> single-target hold (GO_TO / do_scan=false)

FEEDBACK (Task 5): /{drone_id}/mission_feedback at 1 Hz, JSON:
    {drone_id, state, command, target, battery, dist_to_target, ts}
    state in: IDLE, EN_ROUTE, ARRIVED, HOLDING, SCANNING, RETURNING, FAILED

COORDINATE FRAMES:
    world ENU target (Ex, Ey) -> local NED setpoint (N, E, D)
      N = Ey - spawn_North ;  E = Ex - spawn_East ;  D = -altitude

Parameters:
- instance (int)          : PX4 instance index. Derives all topic names.
- do_scan (bool)          : True = lawnmower-sweep the cell (looping);
                            False = fly to centroid and hold. Default True.
- lane_spacing (double)   : spacing between sweep lanes (m). Default 15.0.
- waypoint_radius (double): arrival threshold per waypoint (m). Default 3.0.
- max_step (double)       : max distance the commanded setpoint leads the drone
                            by (m). Smaller = gentler/slower. Default 25.0.
- default_alt (double)    : fallback altitude if a command omits one. Default 12.0.
- stop_and_go (bool)      : hover at each capture point. Default False.
- hover_sec (double)      : hover dwell for stop_and_go. Default 2.0.
- spawn_x / spawn_y (double): optional pre-seed of the spawn (ENU East/North).
                            The fog command and global-ref auto-calibration both
                            override / cross-check this. Defaults per instance.
- simulate_low_battery (bool): force the battery model low (drone-failure demo).
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
from std_msgs.msg import String

from drone_node.drone_naming import drone_id_for


# World ENU origin (Gazebo world SDF <spherical_coordinates>). MUST match the
# value the fog uses. Only needed for auto-calibration / spawn cross-check.
WORLD_ORIGIN_LAT = 47.397971057728974
WORLD_ORIGIN_LON = 8.546163739800146
_EARTH_RADIUS_M = 6371000.0

# Optional per-instance spawn pre-seed (used only if spawn_x/spawn_y params are
# left at their per-instance defaults; fog command + auto-calibration override).
DEFAULT_SPAWNS = {
    0: (18.0, 25.0),
    1: (23.0, 25.0),
    2: (30.0, 25.0),
}

ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14   # PX4 navigation_state for OFFBOARD

CONTROL_PERIOD_SEC = 0.1   # 10 Hz control loop
TICKS_BEFORE_MODE = 20     # 2.0 s of streaming before switching to OFFBOARD
TICKS_BEFORE_ARM = 30      # 3.0 s before arming

SCAN_LEG_M = 10.0          # SCAN_AREA box half-side (Task 5 verb)
FEEDBACK_PERIOD_SEC = 1.0  # Task 5 feedback rate

# Simple battery model: full at start, ~6 min to empty while flying.
BATTERY_DRAIN_PER_SEC = 1.0 / 360.0
BATTERY_FAIL_THRESHOLD = 0.20

# Phases
PHASE_IDLE = 'IDLE'
PHASE_CLIMB = 'CLIMB'
PHASE_TRANSIT = 'TRANSIT'
PHASE_DESCEND = 'DESCEND'
PHASE_SCAN = 'SCAN'
PHASE_HOLD = 'HOLD'


class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)

        self.declare_parameter('do_scan', True)
        self.do_scan = bool(self.get_parameter('do_scan').value)
        self.declare_parameter('lane_spacing', 15.0)
        self.lane_spacing = float(self.get_parameter('lane_spacing').value)
        self.declare_parameter('waypoint_radius', 3.0)
        self.waypoint_radius = float(self.get_parameter('waypoint_radius').value)
        self.declare_parameter('max_step', 25.0)
        self.max_step = float(self.get_parameter('max_step').value)
        self.declare_parameter('default_alt', 12.0)
        self.default_alt = float(self.get_parameter('default_alt').value)
        self.declare_parameter('stop_and_go', False)
        self.stop_and_go = bool(self.get_parameter('stop_and_go').value)
        self.declare_parameter('hover_sec', 2.0)
        self.hover_sec = float(self.get_parameter('hover_sec').value)

        # Optional spawn pre-seed (Task-5 interface). Default per instance.
        default_sx, default_sy = DEFAULT_SPAWNS.get(self.instance, (0.0, 0.0))
        self.declare_parameter('spawn_x', default_sx)
        self.declare_parameter('spawn_y', default_sy)
        self.declare_parameter('simulate_low_battery', False)
        self.simulate_low_battery = bool(
            self.get_parameter('simulate_low_battery').value)

        # PX4 SITL: instance N uses MAVLink system id N+1.
        self.sysid = self.instance + 1

        # ---- Topic prefixes ----
        if self.instance == 0:
            in_prefix = '/fmu/in'
            out_prefix = '/fmu/out'
        else:
            in_prefix = f'/px4_{self.instance}/fmu/in'
            out_prefix = f'/px4_{self.instance}/fmu/out'

        cmd_in_topic = f'/{self.drone_id}/mission_command'
        feedback_topic = f'/{self.drone_id}/mission_feedback'

        # ---- QoS ----
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Publishers to PX4 ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{in_prefix}/offboard_control_mode', pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{in_prefix}/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{in_prefix}/vehicle_command', pub_qos)

        # ---- Feedback publisher (Task 5) ----
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)

        # ---- Subscribers ----
        self.create_subscription(
            String, cmd_in_topic, self.mission_command_callback, 10)
        self.create_subscription(
            VehicleStatus, f'{out_prefix}/vehicle_status_v1',
            self.status_callback, sub_qos)
        self.create_subscription(
            VehicleLocalPosition, f'{out_prefix}/vehicle_local_position_v1',
            self.local_pos_callback, sub_qos)

        # ---- Spawn state (world ENU East/North) ----
        seed_x = float(self.get_parameter('spawn_x').value)
        seed_y = float(self.get_parameter('spawn_y').value)
        self.spawn_x = seed_x
        self.spawn_y = seed_y
        # If a non-trivial seed was given, treat it as calibrated but keep the
        # global-ref cross-check armed so a wrong seed still gets a warning.
        if seed_x != 0.0 or seed_y != 0.0:
            self.spawn_calibrated = True
            self.spawn_source = 'param'
            self._spawn_checked = False
        else:
            self.spawn_calibrated = False
            self.spawn_source = None
            self._spawn_checked = False

        # ---- Flight state ----
        self.active = False
        self.phase = PHASE_IDLE
        self.alt = self.default_alt          # scan altitude
        self.transit_alt = self.default_alt  # high obstacle-safe altitude
        self.waypoints = [(0.0, 0.0)]        # (world_x, world_y)
        self.wp_idx = 0
        self.scan_loops = 0
        self._hover_until = None             # stop_and_go: hover deadline
        self.counter = 0
        self.mode_sent = False
        self.arm_sent = False
        self.latest_status = None
        self.latest_local_pos = None
        self._wait_log_counter = 0

        # ---- Task-5 feedback / battery state ----
        self.command = 'NONE'
        self.state = 'IDLE'
        self.cur_target_world = None         # (x, y, alt) reported in feedback
        self.battery = 1.0
        self.flying_since = None

        # ---- Loops ----
        self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)
        self.create_timer(FEEDBACK_PERIOD_SEC, self.publish_feedback)

        self.get_logger().info(
            f'[COMMANDER] {self.drone_id} (instance={self.instance}, '
            f'sysid={self.sysid}) ready.')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: do_scan={self.do_scan}, '
            f'lane_spacing={self.lane_spacing}m, max_step={self.max_step}m, '
            f'simulate_low_battery={self.simulate_low_battery}.')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: commands on {cmd_in_topic}, '
            f'feedback on {feedback_topic}, PX4 input prefix {in_prefix}')

    # ------------------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def status_callback(self, msg):
        self.latest_status = msg

    def _ref_based_spawn(self, msg):
        origin_lat_rad = math.radians(WORLD_ORIGIN_LAT)
        north = math.radians(float(msg.ref_lat) - WORLD_ORIGIN_LAT) * _EARTH_RADIUS_M
        east = (math.radians(float(msg.ref_lon) - WORLD_ORIGIN_LON)
                * _EARTH_RADIUS_M * math.cos(origin_lat_rad))
        return east, north

    def local_pos_callback(self, msg):
        self.latest_local_pos = msg
        if not getattr(msg, 'xy_global', False):
            return

        if not self.spawn_calibrated:
            self.spawn_x, self.spawn_y = self._ref_based_spawn(msg)
            self.spawn_calibrated = True
            self.spawn_source = 'auto (global ref)'
            self._spawn_checked = True
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: spawn auto-calibrated from global '
                f'ref -> world ENU=({self.spawn_x:.1f}, {self.spawn_y:.1f})')
        elif (self.spawn_source
              and self.spawn_source.startswith(('fog', 'param'))
              and not self._spawn_checked):
            ex, ny = self._ref_based_spawn(msg)
            err = math.hypot(ex - self.spawn_x, ny - self.spawn_y)
            self._spawn_checked = True
            if err > 3.0:
                self.get_logger().warn(
                    f'[COMMANDER] {self.drone_id}: configured spawn '
                    f'({self.spawn_x:.1f}, {self.spawn_y:.1f}) disagrees with '
                    f'actual spawn ({ex:.1f}, {ny:.1f}) by {err:.1f} m. The drone '
                    f'will reach the WRONG place. Fix spawns_x/spawns_y on the fog '
                    f'(or spawn_x/spawn_y here) to match PX4_GZ_MODEL_POSE.')

    # ------------------------------------------------------------------
    def ned_of(self, world_x, world_y, alt):
        """World ENU (x=East, y=North) -> local NED [N, E, D] at altitude alt."""
        return [world_y - self.spawn_y, world_x - self.spawn_x, -alt]

    def build_lawnmower(self, min_x, max_x, min_y, max_y):
        """
        Boustrophedon ("lawnmower") sweep. Lanes run along the cell's LONGER
        dimension and step across the shorter one by lane_spacing. Alternate
        lanes reverse direction so the path is continuous; a small inset keeps
        it off the boundary.
        """
        inset = 2.0
        lane = max(self.lane_spacing, 1.0)
        span_x = max_x - min_x
        span_y = max_y - min_y
        wps = []

        if span_x >= span_y:
            lines = []
            y = min_y + lane / 2.0
            while y < max_y:
                lines.append(y)
                y += lane
            if not lines:
                lines = [(min_y + max_y) / 2.0]
            for j, yv in enumerate(lines):
                if j % 2 == 0:
                    wps.append((min_x + inset, yv))
                    wps.append((max_x - inset, yv))
                else:
                    wps.append((max_x - inset, yv))
                    wps.append((min_x + inset, yv))
        else:
            lines = []
            x = min_x + lane / 2.0
            while x < max_x:
                lines.append(x)
                x += lane
            if not lines:
                lines = [(min_x + max_x) / 2.0]
            for j, xv in enumerate(lines):
                if j % 2 == 0:
                    wps.append((xv, min_y + inset))
                    wps.append((xv, max_y - inset))
                else:
                    wps.append((xv, max_y - inset))
                    wps.append((xv, min_y + inset))
        return wps

    def densify(self, wps, spacing):
        """
        Insert intermediate capture points along every path segment, spaced by
        `spacing` (camera footprint length minus overlap). Turns the
        lane-endpoint lawnmower into the grid of capture points for stop-and-go.
        """
        if spacing <= 0.5 or len(wps) < 2:
            return wps
        out = [wps[0]]
        for a, b in zip(wps, wps[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, math.ceil(seg / spacing))
            for k in range(1, n + 1):
                t = k / n
                out.append((a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t))
        return out

    # ------------------------------------------------------------------
    def _begin_mission(self, command, target, area=None):
        """
        Shared setup for START_MISSION / GO_TO / SCAN_AREA. Sets altitudes,
        optional spawn, waypoints, and (re)starts the phased arm sequence.
        `target` carries world_x/world_y/alt (+ optional transit_alt, spawn,
        capture_spacing). `area` (dict min_x/max_x/min_y/max_y) triggers a
        lawnmower sweep; otherwise a single hold waypoint.
        """
        self.alt = float(target.get('alt', self.default_alt))
        self.transit_alt = float(target.get('transit_alt', self.alt))
        if self.transit_alt < self.alt:
            self.transit_alt = self.alt

        spawn = target.get('spawn')
        if spawn is not None:
            self.spawn_x = float(spawn['x'])
            self.spawn_y = float(spawn['y'])
            self.spawn_calibrated = True
            self.spawn_source = 'fog command'
            self._spawn_checked = False
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: spawn from fog -> world ENU='
                f'({self.spawn_x:.1f}, {self.spawn_y:.1f})')

        if self.do_scan and area:
            self.waypoints = self.build_lawnmower(
                float(area['min_x']), float(area['max_x']),
                float(area['min_y']), float(area['max_y']))
            if self.stop_and_go:
                spacing = float(target.get('capture_spacing', 10.0))
                self.waypoints = self.densify(self.waypoints, spacing)
        else:
            self.waypoints = [(float(target['world_x']), float(target['world_y']))]

        self.cur_target_world = (float(target['world_x']),
                                 float(target['world_y']), self.alt)
        self.wp_idx = 0
        self.scan_loops = 0
        self._hover_until = None
        self.active = True
        self.phase = PHASE_CLIMB
        self.counter = 0
        self.mode_sent = False
        self.arm_sent = False
        self.command = command
        self.state = 'EN_ROUTE'
        if self.flying_since is None:
            self.flying_since = time.time()

        first = self.waypoints[0]
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: {command} '
            f'transit@{self.transit_alt}m scan@{self.alt}m, '
            f'{len(self.waypoints)} waypoint(s), first=ENU{first} '
            f'-> CLIMB, TRANSIT, DESCEND, SCAN/HOLD.')

    def mission_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'[COMMANDER] {self.drone_id}: bad command JSON')
            return

        command = cmd.get('command', '')
        target = cmd.get('target', {})

        if command in ('START_MISSION', 'GO_TO'):
            # START_MISSION carries an 'area' (lawnmower). GO_TO is a single
            # point (decision-node action). Both share the phased core.
            area = target.get('area')
            self._begin_mission(command, target, area=area)

        elif command == 'SCAN_AREA':
            wx = float(target['world_x'])
            wy = float(target['world_y'])
            h = SCAN_LEG_M
            area = {'min_x': wx - h, 'max_x': wx + h,
                    'min_y': wy - h, 'max_y': wy + h}
            self._begin_mission('SCAN_AREA', target, area=area)
            self.state = 'SCANNING'

        elif command == 'HOVER':
            # Prefer the commanded hold point (e.g. the decision node parks the
            # rescuer over the victim's world_x/world_y); otherwise hold the
            # current position. Altitude follows the command, else keeps scan alt.
            if 'world_x' in target and 'world_y' in target:
                wx = float(target['world_x'])
                wy = float(target['world_y'])
                self.alt = float(target.get('alt', self.alt))
                self.waypoints = [(wx, wy)]
                self.cur_target_world = (wx, wy, self.alt)
            elif self.latest_local_pos is not None and self.spawn_calibrated:
                wx = self.spawn_x + self.latest_local_pos.y   # NED E -> ENU E
                wy = self.spawn_y + self.latest_local_pos.x   # NED N -> ENU N
                self.waypoints = [(wx, wy)]
                self.cur_target_world = (wx, wy, self.alt)
            self.wp_idx = 0
            self.phase = PHASE_HOLD
            self.active = True
            self.command = 'HOVER'
            self.state = 'HOLDING'
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: HOVER (holding)')

        elif command in ('RTL', 'RETURN_HOME'):
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: {command} — returning to launch')
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.active = False
            self.phase = PHASE_IDLE
            self._hover_until = None
            self.command = command
            self.state = 'RETURNING'

        else:
            self.get_logger().warn(
                f'[COMMANDER] {self.drone_id}: unknown command "{command}"')

    # ------------------------------------------------------------------
    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = self.sysid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    # ------------------------------------------------------------------
    def target_ned(self):
        """The TRUE target for the current phase (uncapped), in local NED."""
        if self.phase == PHASE_CLIMB:
            return [0.0, 0.0, -self.transit_alt]
        idx = min(self.wp_idx, len(self.waypoints) - 1)
        wx, wy = self.waypoints[idx]
        if self.phase == PHASE_TRANSIT:
            return self.ned_of(wx, wy, self.transit_alt)
        return self.ned_of(wx, wy, self.alt)

    def current_setpoint(self):
        """True target, capped so it never leads the drone by > max_step (m)."""
        raw = self.target_ned()
        if self.phase != PHASE_CLIMB and self.latest_local_pos is not None:
            cn = self.latest_local_pos.x
            ce = self.latest_local_pos.y
            dn = raw[0] - cn
            de = raw[1] - ce
            dist = math.hypot(dn, de)
            if dist > self.max_step:
                s = self.max_step / dist
                return [cn + dn * s, ce + de * s, raw[2]]
        return raw

    def update_phase(self):
        """Advance the phase machine using the local position estimate."""
        if self.latest_local_pos is None:
            return
        alt_now = -self.latest_local_pos.z

        if self.phase == PHASE_CLIMB:
            if alt_now < self.transit_alt - 1.0:
                return
            if not self.spawn_calibrated:
                self._wait_log_counter += 1
                if self._wait_log_counter % 50 == 0:
                    self.get_logger().warn(
                        f'[COMMANDER] {self.drone_id}: at altitude, waiting for a '
                        f'spawn (fog command or global reference) before TRANSIT.')
                return
            self.phase = PHASE_TRANSIT
            self.state = 'EN_ROUTE'
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: reached {alt_now:.1f}m, '
                f'TRANSIT to cell at {self.transit_alt}m.')
            return

        if self.phase == PHASE_DESCEND:
            if abs(alt_now - self.alt) > 1.0:
                return
            if len(self.waypoints) > 1:
                self.phase = PHASE_SCAN
                self.state = 'SCANNING'
                self.wp_idx = 1
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: at scan altitude '
                    f'{alt_now:.1f}m, SCAN ({len(self.waypoints)} waypoints, '
                    f'looping until RTL).')
            else:
                self.phase = PHASE_HOLD
                # Single-target hold: GO_TO -> ARRIVED, explicit HOVER -> HOLDING.
                self.state = 'HOLDING' if self.command == 'HOVER' else 'ARRIVED'
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: at scan altitude, HOLD.')
            return

        if self.phase in (PHASE_TRANSIT, PHASE_SCAN):
            tgt = self.target_ned()
            dn = tgt[0] - self.latest_local_pos.x
            de = tgt[1] - self.latest_local_pos.y
            dist = (dn * dn + de * de) ** 0.5
            if dist >= self.waypoint_radius:
                return

            if self.phase == PHASE_TRANSIT:
                self.phase = PHASE_DESCEND
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: over cell, DESCEND '
                    f'{self.transit_alt}m -> {self.alt}m.')
            else:  # PHASE_SCAN — loop forever
                if self.stop_and_go:
                    now_s = self.get_clock().now().nanoseconds / 1e9
                    if self._hover_until is None:
                        self._hover_until = now_s + self.hover_sec
                        return
                    if now_s < self._hover_until:
                        return
                    self._hover_until = None
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.wp_idx = 0
                    self.scan_loops += 1
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: scan pass '
                        f'{self.scan_loops} complete, restarting sweep.')

    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.active:
            return

        sp = self.current_setpoint()

        offboard = OffboardControlMode()
        offboard.timestamp = self.now_us()
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        tsp = TrajectorySetpoint()
        tsp.timestamp = self.now_us()
        tsp.position = [float(sp[0]), float(sp[1]), float(sp[2])]
        tsp.yaw = 0.0
        self.setpoint_pub.publish(tsp)

        if self.counter == TICKS_BEFORE_MODE and not self.mode_sent:
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: setting OFFBOARD mode')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.mode_sent = True

        if self.counter == TICKS_BEFORE_ARM and not self.arm_sent:
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: arming')
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_sent = True

        armed = (self.latest_status is not None
                 and self.latest_status.arming_state == ARMING_STATE_ARMED)

        # OFFBOARD auto-recovery: if armed but PX4 left OFFBOARD, re-command it.
        if (self.arm_sent and armed and self.latest_status is not None
                and self.latest_status.nav_state != NAV_STATE_OFFBOARD
                and self.counter % 10 == 0):
            self.get_logger().warn(
                f'[COMMANDER] {self.drone_id}: not in OFFBOARD '
                f'(nav_state={self.latest_status.nav_state}); re-commanding.')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        if armed:
            self.update_phase()

        self.counter += 1

    # ------------------------------------------------------------------
    def publish_feedback(self):
        """Task 5 — report state back to the decision node at 1 Hz."""
        if self.simulate_low_battery:
            self.battery = 0.08
        elif self.active and self.flying_since is not None:
            elapsed = time.time() - self.flying_since
            self.battery = max(0.0, 1.0 - BATTERY_DRAIN_PER_SEC * elapsed)

        state = self.state
        if self.battery <= BATTERY_FAIL_THRESHOLD:
            state = 'FAILED'

        dist = None
        if self.active and self.latest_local_pos is not None:
            tgt = self.target_ned()
            dn = tgt[0] - self.latest_local_pos.x
            de = tgt[1] - self.latest_local_pos.y
            dist = round((dn * dn + de * de) ** 0.5, 2)

        fb = {
            'drone_id': self.drone_id,
            'state': state,
            'command': self.command,
            'target': self.cur_target_world,
            'battery': round(self.battery, 3),
            'dist_to_target': dist,
            'ts': time.time(),
        }
        msg = String()
        msg.data = json.dumps(fb)
        self.feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()