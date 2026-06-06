"""
drone_commander.py

Drone-side flight executor (offboard-streaming, phased, with full-area scan).
Works for ANY number of drones. Spawn locations are NOT hardcoded here:
they are configured on the FOG node and delivered to each drone inside the
START_MISSION command. (If a command omits the spawn, the commander falls
back to auto-calibrating it from PX4's reported global reference.)

Per drone, on START_MISSION:

    CLIMB    -> straight up over the spawn point to scan altitude
    TRANSIT  -> across to the first corner of the assigned cell
    SCAN     -> a boustrophedon ("lawnmower") sweep covering the whole cell
    HOLD     -> loiter at the last waypoint

WHY OFFBOARD STREAMING + PHASING:
PX4 only accepts OFFBOARD mode and arming once it is already receiving a
steady setpoint stream, and it rejects DO_REPOSITION (command 192) in this
SITL build. We stream OffboardControlMode + TrajectorySetpoint continuously,
switch to OFFBOARD, then arm. We climb VERTICALLY first (the setpoint stays
over the spawn) instead of commanding the far target off the ground, which
keeps the takeoff controlled.

COORDINATE FRAMES:
Fog targets are world ENU (x=East, y=North from the Gazebo world origin).
PX4 TrajectorySetpoint is the drone's LOCAL NED frame, origin = the drone's
spawn point. With the spawn known (from the fog command), conversion is:

    N = world_y - spawn_North
    E = world_x - spawn_East
    D = -altitude

Parameters:
- instance (int)          : PX4 instance index. Derives all topic names.
- do_scan (bool)          : True = lawnmower-sweep the cell; False = fly to the
                            cell centroid and hold. Default True.
- lane_spacing (double)   : spacing between sweep lanes (m). Default 15.0.
- waypoint_radius (double): arrival threshold per waypoint (m). Default 3.0.
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
# value the fog uses, so both sides share one coordinate frame. Only needed
# for the optional auto-calibration / spawn cross-check.
WORLD_ORIGIN_LAT = 47.397971057728974
WORLD_ORIGIN_LON = 8.546163739800146
_EARTH_RADIUS_M = 6371000.0

ARMING_STATE_ARMED = 2

CONTROL_PERIOD_SEC = 0.1   # 10 Hz control loop
TICKS_BEFORE_MODE = 20     # 2.0 s of streaming before switching to OFFBOARD
TICKS_BEFORE_ARM = 30      # 3.0 s before arming

# Phases
PHASE_IDLE = 'IDLE'
PHASE_CLIMB = 'CLIMB'
PHASE_TRANSIT = 'TRANSIT'
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

        # ---- Spawn state (world ENU East/North). Filled from the fog command
        #      or, as a fallback, auto-calibrated from the global reference. ----
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_calibrated = False
        self.spawn_source = None
        self._spawn_checked = False   # have we cross-checked vs PX4 truth yet?

        # ---- Flight state ----
        self.active = False
        self.phase = PHASE_IDLE
        self.alt = self.default_alt
        self.waypoints = [(0.0, 0.0)]   # (world_x, world_y)
        self.wp_idx = 0
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
            f'lane_spacing={self.lane_spacing}m. Spawn comes from the fog command.')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: listening on {cmd_in_topic}, '
            f'PX4 input prefix {in_prefix}')

    # ------------------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def status_callback(self, msg):
        self.latest_status = msg

    def _ref_based_spawn(self, msg):
        """Spawn (world ENU East, North) derived from the EKF global reference."""
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
            # Fallback: no spawn from the fog yet — derive it from PX4.
            self.spawn_x, self.spawn_y = self._ref_based_spawn(msg)
            self.spawn_calibrated = True
            self.spawn_source = 'auto (global ref)'
            self._spawn_checked = True
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: spawn auto-calibrated from global '
                f'ref -> world ENU=({self.spawn_x:.1f}, {self.spawn_y:.1f})')
        elif self.spawn_source and self.spawn_source.startswith('fog') and not self._spawn_checked:
            # Cross-check the fog-provided spawn against PX4's actual spawn.
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
    def ned_of(self, world_x, world_y):
        """World ENU (x=East, y=North) -> local NED [N, E, D] at scan alt."""
        return [world_y - self.spawn_y, world_x - self.spawn_x, -self.alt]

    def build_lawnmower(self, min_x, max_x, min_y, max_y):
        """
        Boustrophedon ("lawnmower") sweep of the rectangle. Lanes run along the
        cell's LONGER dimension and step across the shorter one by lane_spacing,
        so coverage uses fewer, longer passes. Alternate lanes reverse direction
        so the path is continuous. A small inset keeps the path off the boundary.
        """
        inset = 2.0
        lane = max(self.lane_spacing, 1.0)
        span_x = max_x - min_x
        span_y = max_y - min_y
        wps = []

        if span_x >= span_y:
            # Wide cell: lanes run East-West (vary X), step North (vary Y).
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
            # Tall cell: lanes run North-South (vary Y), step East (vary X).
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

            # Spawn from the fog (preferred). Resets the cross-check so we
            # re-verify against PX4 truth.
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
            self.active = True
            self.phase = PHASE_CLIMB
            self.counter = 0
            self.mode_sent = False
            self.arm_sent = False

            first = self.waypoints[0]
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: START_MISSION alt={self.alt}m, '
                f'{len(self.waypoints)} waypoint(s), first=ENU{first} '
                f'-> CLIMB then TRANSIT.')
        elif command == 'RTL':
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: RTL received')
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
    def current_setpoint(self):
        """NED setpoint for the active phase."""
        if self.phase == PHASE_CLIMB:
            return [0.0, 0.0, -self.alt]   # straight up over the spawn
        idx = min(self.wp_idx, len(self.waypoints) - 1)
        wx, wy = self.waypoints[idx]
        return self.ned_of(wx, wy)

    def update_phase(self):
        """Advance the phase machine using the local position estimate."""
        if self.latest_local_pos is None:
            return
        alt_now = -self.latest_local_pos.z

        if self.phase == PHASE_CLIMB:
            if alt_now < self.alt - 1.0:
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
                f'TRANSIT to cell.')
            return

        if self.phase in (PHASE_TRANSIT, PHASE_SCAN):
            tgt = self.current_setpoint()
            dn = tgt[0] - self.latest_local_pos.x
            de = tgt[1] - self.latest_local_pos.y
            dist = (dn * dn + de * de) ** 0.5
            if dist >= self.waypoint_radius:
                return

            if self.phase == PHASE_TRANSIT:
                if len(self.waypoints) > 1:
                    self.phase = PHASE_SCAN
                    self.wp_idx = 1
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: arrived at cell, '
                        f'SCAN ({len(self.waypoints)} waypoints).')
                else:
                    self.phase = PHASE_HOLD
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: arrived at cell, HOLD.')
            else:  # PHASE_SCAN
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.wp_idx = len(self.waypoints) - 1
                    self.phase = PHASE_HOLD
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: scan complete, HOLD.')
                else:
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: waypoint '
                        f'{self.wp_idx}/{len(self.waypoints) - 1}.')

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