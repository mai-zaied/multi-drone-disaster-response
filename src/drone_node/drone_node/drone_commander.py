"""
drone_commander.py

Drone-side flight executor (offboard-streaming, phased, with CONTINUOUS
full-area scan). Works for ANY number of drones. Spawn locations are NOT
hardcoded here: they are configured on the FOG node and delivered to each
drone inside the START_MISSION command (auto-calibration fallback included).

Per drone, on START_MISSION:

    CLIMB    -> straight up over the spawn point to scan altitude
    TRANSIT  -> across to the first corner of the assigned cell
    SCAN     -> a boustrophedon ("lawnmower") sweep that LOOPS continuously,
                covering the whole cell again and again
    HOLD     -> (only if there is a single target, e.g. do_scan=false)

The scan keeps running until the fog sends an RTL command (which it does on
/fog/end_mission), at which point the drone returns to launch.

ROBUSTNESS (added after observing failsafe-induced loss of control):
- OFFBOARD auto-recovery: if PX4 leaves OFFBOARD (e.g. a transient failsafe),
  the commander re-commands OFFBOARD so control is regained.
- Lead/"carrot" setpoints: the commanded position never sits more than
  `max_step` metres ahead of the drone, so it accelerates gently instead of
  rolling hard toward a far waypoint (which trips the attitude failure check).

WHY OFFBOARD STREAMING + PHASING:
PX4 only accepts OFFBOARD mode and arming once it is already receiving a
steady setpoint stream, and it rejects DO_REPOSITION (command 192) in this
SITL build. We stream OffboardControlMode + TrajectorySetpoint continuously,
switch to OFFBOARD, then arm. We climb VERTICALLY first (the setpoint stays
over the spawn) so takeoff is controlled.

COORDINATE FRAMES:
Fog targets are world ENU (x=East, y=North from the Gazebo world origin).
PX4 TrajectorySetpoint is the drone's LOCAL NED frame, origin = the drone's
spawn point. With the spawn known (from the fog command):

    N = world_y - spawn_North
    E = world_x - spawn_East
    D = -altitude

Parameters:
- instance (int)          : PX4 instance index. Derives all topic names.
- do_scan (bool)          : True = lawnmower-sweep the cell (looping);
                            False = fly to the cell centroid and hold. Default True.
- lane_spacing (double)   : spacing between sweep lanes (m). Default 15.0.
- waypoint_radius (double): arrival threshold per waypoint (m). Default 3.0.
- max_step (double)       : max distance the commanded setpoint leads the drone
                            by (m). Smaller = gentler/slower. Default 25.0.
- default_alt (double)    : fallback altitude if a command omits one. Default 12.0.
"""

import json
import math

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

ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14   # PX4 navigation_state for OFFBOARD

CONTROL_PERIOD_SEC = 0.1   # 10 Hz control loop
TICKS_BEFORE_MODE = 20     # 2.0 s of streaming before switching to OFFBOARD
TICKS_BEFORE_ARM = 30      # 3.0 s before arming

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
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_calibrated = False
        self.spawn_source = None
        self._spawn_checked = False

        # ---- Flight state ----
        self.active = False
        self.phase = PHASE_IDLE
        self.alt = self.default_alt          # scan altitude
        self.transit_alt = self.default_alt  # high obstacle-safe altitude
        self.waypoints = [(0.0, 0.0)]   # (world_x, world_y)
        self.wp_idx = 0
        self.scan_loops = 0
        self.counter = 0
        self.mode_sent = False
        self.arm_sent = False
        self.latest_status = None
        self.latest_local_pos = None
        self._wait_log_counter = 0

        # ---- Control loop ----
        self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)

        self.get_logger().info(
            f'[COMMANDER] {self.drone_id} (instance={self.instance}, '
            f'sysid={self.sysid}) ready.')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: do_scan={self.do_scan}, '
            f'lane_spacing={self.lane_spacing}m, max_step={self.max_step}m. '
            f'Spawn comes from the fog command.')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: listening on {cmd_in_topic}, '
            f'PX4 input prefix {in_prefix}')

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
        elif self.spawn_source and self.spawn_source.startswith('fog') and not self._spawn_checked:
            ex, ny = self._ref_based_spawn(msg)
            err = math.hypot(ex - self.spawn_x, ny - self.spawn_y)
            self._spawn_checked = True
            if err > 3.0:
                self.get_logger().warn(
                    f'[COMMANDER] {self.drone_id}: fog spawn '
                    f'({self.spawn_x:.1f}, {self.spawn_y:.1f}) disagrees with '
                    f'actual spawn ({ex:.1f}, {ny:.1f}) by {err:.1f} m. The drone '
                    f'will reach the WRONG place. Fix spawns_x/spawns_y on the fog '
                    f'to match this drone\'s PX4_GZ_MODEL_POSE.')

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

    # ------------------------------------------------------------------
    def mission_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'[COMMANDER] {self.drone_id}: bad command JSON')
            return

        command = cmd.get('command', '')
        if command == 'START_MISSION':
            t = cmd.get('target', {})
            self.alt = float(t.get('alt', self.default_alt))
            # Transit altitude: high enough to clear trees/buildings on the way.
            self.transit_alt = float(t.get('transit_alt', self.alt))
            if self.transit_alt < self.alt:
                self.transit_alt = self.alt

            spawn = t.get('spawn')
            if spawn is not None:
                self.spawn_x = float(spawn['x'])
                self.spawn_y = float(spawn['y'])
                self.spawn_calibrated = True
                self.spawn_source = 'fog command'
                self._spawn_checked = False
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: spawn from fog -> world ENU='
                    f'({self.spawn_x:.1f}, {self.spawn_y:.1f})')

            area = t.get('area')
            if self.do_scan and area:
                self.waypoints = self.build_lawnmower(
                    float(area['min_x']), float(area['max_x']),
                    float(area['min_y']), float(area['max_y']))
            else:
                self.waypoints = [(float(t['world_x']), float(t['world_y']))]

            self.wp_idx = 0
            self.scan_loops = 0
            self.active = True
            self.phase = PHASE_CLIMB
            self.counter = 0
            self.mode_sent = False
            self.arm_sent = False

            first = self.waypoints[0]
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: START_MISSION '
                f'transit@{self.transit_alt}m scan@{self.alt}m, '
                f'{len(self.waypoints)} waypoint(s), first=ENU{first} '
                f'-> CLIMB, TRANSIT, DESCEND, SCAN (loops until RTL).')
        elif command == 'RTL':
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: RTL received — returning to launch')
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.active = False
            self.phase = PHASE_IDLE
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
            # Straight up over the spawn, to the HIGH transit altitude.
            return [0.0, 0.0, -self.transit_alt]
        idx = min(self.wp_idx, len(self.waypoints) - 1)
        wx, wy = self.waypoints[idx]
        if self.phase == PHASE_TRANSIT:
            # Cross the map at the high, obstacle-safe altitude.
            return self.ned_of(wx, wy, self.transit_alt)
        # DESCEND / SCAN / HOLD happen at the low scan altitude.
        return self.ned_of(wx, wy, self.alt)

    def current_setpoint(self):
        """
        The setpoint we actually stream: the true target, but capped so it
        never leads the drone by more than max_step metres horizontally. This
        keeps accelerations gentle and avoids the hard rolls that trip PX4's
        attitude failure check.
        """
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
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: reached {alt_now:.1f}m, '
                f'TRANSIT to cell at {self.transit_alt}m.')
            return

        if self.phase == PHASE_DESCEND:
            # Holding XY over the cell's first corner, sinking to scan altitude.
            if abs(alt_now - self.alt) > 1.0:
                return
            if len(self.waypoints) > 1:
                self.phase = PHASE_SCAN
                self.wp_idx = 1
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: at scan altitude '
                    f'{alt_now:.1f}m, SCAN ({len(self.waypoints)} waypoints, '
                    f'looping until RTL).')
            else:
                self.phase = PHASE_HOLD
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
                # Arrived over the cell's first corner: descend before scanning.
                self.phase = PHASE_DESCEND
                self.get_logger().info(
                    f'[COMMANDER] {self.drone_id}: over cell, DESCEND '
                    f'{self.transit_alt}m -> {self.alt}m.')
            else:  # PHASE_SCAN — loop forever
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

        # OFFBOARD auto-recovery: if we've armed but PX4 is not in OFFBOARD
        # (e.g. a transient failsafe kicked us out), re-command it. Throttled.
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