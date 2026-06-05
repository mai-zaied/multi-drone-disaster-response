"""
drone_commander.py  —  extended for Task 5 (Drone Action + feedback loop)

Closes the loop between the decision node's commands and PX4 flight control,
using the same offboard-streaming method as before (stream OffboardControlMode +
TrajectorySetpoint, then switch to offboard, then arm — so arming succeeds).

TASK 5 ADDITIONS (on top of the START_MISSION / RTL commander):
  - New commands (5.8/5.10): GO_TO, HOVER, SCAN_AREA, RETURN_HOME.
        START_MISSION is kept as an alias of GO_TO.
  - Feedback loop (5.13): publishes /{drone_id}/mission_feedback at 1 Hz with
        {drone_id, state, command, target, battery, dist_to_target, ts}.
        state in: IDLE, EN_ROUTE, ARRIVED, HOLDING, SCANNING, RETURNING, FAILED.
  - Simple battery model so the decision node's battery-aware selection (5.7)
        and the drone-failure scenario (5.14) are meaningful in SITL.

COORDINATE FRAMES (unchanged):
    world ENU target (Ex, Ey) -> local NED setpoint (N, E, D)
      N = Ey - spawn_Ey ;  E = Ex - spawn_Ex ;  D = -altitude
"""

import json
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


DEFAULT_SPAWNS = {
    0: (18.0, 25.0),
    1: (23.0, 25.0),
    2: (30.0, 25.0),
}

ARMING_STATE_ARMED = 2
CONTROL_PERIOD_SEC = 0.1          # 10 Hz control loop
TICKS_BEFORE_MODE = 20            # 2.0 s of streaming before offboard mode
TICKS_BEFORE_ARM = 30            # 3.0 s before arming

ARRIVAL_RADIUS_M = 2.0
SCAN_LEG_M = 10.0                 # SCAN_AREA pattern half-side
FEEDBACK_PERIOD_SEC = 1.0

# Simple battery model: full at start, ~6 min to empty while flying.
BATTERY_DRAIN_PER_SEC = 1.0 / 360.0


class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)

        default_sx, default_sy = DEFAULT_SPAWNS.get(self.instance, (0.0, 0.0))
        self.declare_parameter('spawn_x', default_sx)
        self.declare_parameter('spawn_y', default_sy)
        self.declare_parameter('simulate_low_battery', False)
        self.spawn_x = float(self.get_parameter('spawn_x').value)   # ENU East
        self.spawn_y = float(self.get_parameter('spawn_y').value)   # ENU North
        self.simulate_low_battery = bool(
            self.get_parameter('simulate_low_battery').value)

        self.sysid = self.instance + 1

        if self.instance == 0:
            topic_prefix, out_prefix = '/fmu/in', '/fmu/out'
        else:
            topic_prefix = f'/px4_{self.instance}/fmu/in'
            out_prefix = f'/px4_{self.instance}/fmu/out'

        cmd_in_topic = f'/{self.drone_id}/mission_command'
        feedback_topic = f'/{self.drone_id}/mission_feedback'

        # ---- QoS ----
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---- Publishers to PX4 ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{topic_prefix}/offboard_control_mode', pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{topic_prefix}/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{topic_prefix}/vehicle_command', pub_qos)

        # ---- Feedback publisher (5.13) ----
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)

        # ---- Subscribers ----
        self.create_subscription(String, cmd_in_topic,
                                 self.mission_command_callback, 10)
        self.create_subscription(VehicleStatus, f'{out_prefix}/vehicle_status_v1',
                                 self.status_callback, sub_qos)
        self.create_subscription(VehicleLocalPosition,
                                 f'{out_prefix}/vehicle_local_position_v1',
                                 self.local_pos_callback, sub_qos)

        # ---- State ----
        self.active = False
        self.target_ned = None            # [N, E, D]
        self.counter = 0
        self.mode_sent = False
        self.arm_sent = False
        self.latest_status = None
        self.latest_local_pos = None
        self.state = 'IDLE'               # reported in feedback
        self.command = 'NONE'
        self.cur_target_world = None      # (x, y, alt) for feedback

        # SCAN_AREA pattern state
        self.scan_waypoints = []
        self.scan_idx = 0

        # Battery model
        self.battery = 1.0
        self.flying_since = None

        # ---- Loops ----
        self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)
        self.create_timer(FEEDBACK_PERIOD_SEC, self.publish_feedback)

        self.get_logger().info(
            f'[COMMANDER] {self.drone_id} (instance={self.instance}, '
            f'sysid={self.sysid}) ready. spawn ENU=({self.spawn_x}, {self.spawn_y})')
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: commands on {cmd_in_topic}, '
            f'feedback on {feedback_topic}')

    # ------------------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def status_callback(self, msg):
        self.latest_status = msg

    def local_pos_callback(self, msg):
        self.latest_local_pos = msg

    # ------------------------------------------------------------------
    def _world_to_ned(self, world_x, world_y, alt):
        n = world_y - self.spawn_y
        e = world_x - self.spawn_x
        d = -float(alt)
        return [n, e, d]

    def _set_world_target(self, world_x, world_y, alt):
        self.target_ned = self._world_to_ned(world_x, world_y, alt)
        self.cur_target_world = (world_x, world_y, alt)
        if not self.active:
            # First mission for this drone: run the arm sequence.
            self.counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.flying_since = time.time()
        self.active = True

    # ------------------------------------------------------------------
    def mission_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'[COMMANDER] {self.drone_id}: bad command JSON')
            return

        command = cmd.get('command', '')
        target = cmd.get('target', {})

        if command in ('GO_TO', 'START_MISSION'):
            wx = float(target['world_x'])
            wy = float(target['world_y'])
            alt = float(target.get('alt', 12.0))
            self._set_world_target(wx, wy, alt)
            self.command = command
            self.state = 'EN_ROUTE'
            self.scan_waypoints = []
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: {command} -> world=({wx}, {wy}) '
                f'alt={alt}m  local_NED=({self.target_ned[0]:.1f}, '
                f'{self.target_ned[1]:.1f}, {self.target_ned[2]:.1f})')

        elif command == 'HOVER':
            # Hold the current position (or target if not airborne yet).
            if self.latest_local_pos is not None:
                self.target_ned = [self.latest_local_pos.x,
                                   self.latest_local_pos.y,
                                   self.target_ned[2] if self.target_ned else -12.0]
            self.command = 'HOVER'
            self.state = 'HOLDING'
            self.scan_waypoints = []
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: HOVER (holding)')

        elif command == 'SCAN_AREA':
            wx = float(target['world_x'])
            wy = float(target['world_y'])
            alt = float(target.get('alt', 12.0))
            self.scan_waypoints = self._build_scan_pattern(wx, wy, alt)
            self.scan_idx = 0
            self._set_world_target(*self.scan_waypoints[0])
            self.command = 'SCAN_AREA'
            self.state = 'SCANNING'
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: SCAN_AREA around ({wx}, {wy}), '
                f'{len(self.scan_waypoints)} waypoints')

        elif command in ('RETURN_HOME', 'RTL'):
            self.command = command
            self.state = 'RETURNING'
            self.scan_waypoints = []
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: {command}')
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.active = False

        else:
            self.get_logger().warn(
                f'[COMMANDER] {self.drone_id}: unknown command "{command}"')

    # ------------------------------------------------------------------
    def _build_scan_pattern(self, cx, cy, alt):
        """A small box lawnmower around (cx, cy) in world ENU."""
        h = SCAN_LEG_M
        return [
            (cx - h, cy - h, alt),
            (cx + h, cy - h, alt),
            (cx + h, cy + h, alt),
            (cx - h, cy + h, alt),
        ]

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
    def control_loop(self):
        if not self.active or self.target_ned is None:
            return

        offboard = OffboardControlMode()
        offboard.timestamp = self.now_us()
        offboard.position = True
        self.offboard_pub.publish(offboard)

        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [float(self.target_ned[0]),
                       float(self.target_ned[1]),
                       float(self.target_ned[2])]
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)

        if self.counter == TICKS_BEFORE_MODE and not self.mode_sent:
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: OFFBOARD mode')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.mode_sent = True

        if self.counter == TICKS_BEFORE_ARM and not self.arm_sent:
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: arming')
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_sent = True

        # Distance to current target.
        if self.arm_sent and self.latest_local_pos is not None:
            dn = self.target_ned[0] - self.latest_local_pos.x
            de = self.target_ned[1] - self.latest_local_pos.y
            dd = self.target_ned[2] - self.latest_local_pos.z
            dist = (dn * dn + de * de + dd * dd) ** 0.5

            if dist < ARRIVAL_RADIUS_M:
                if self.command == 'SCAN_AREA' and self.scan_waypoints:
                    # Advance to next scan waypoint (loop the pattern).
                    self.scan_idx = (self.scan_idx + 1) % len(self.scan_waypoints)
                    self._set_world_target(*self.scan_waypoints[self.scan_idx])
                elif self.state == 'EN_ROUTE':
                    self.state = 'ARRIVED'
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: arrived (within {dist:.1f}m)')

        self.counter += 1

    # ------------------------------------------------------------------
    def publish_feedback(self):
        """5.13 — report state back to the decision node."""
        # Battery model: drain while active; honour the failure-injection flag.
        if self.simulate_low_battery:
            self.battery = 0.08
        elif self.active and self.flying_since is not None:
            elapsed = time.time() - self.flying_since
            self.battery = max(0.0, 1.0 - BATTERY_DRAIN_PER_SEC * elapsed)

        state = self.state
        if self.battery <= 0.20:
            state = 'FAILED'

        dist = None
        if self.target_ned is not None and self.latest_local_pos is not None:
            dn = self.target_ned[0] - self.latest_local_pos.x
            de = self.target_ned[1] - self.latest_local_pos.y
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
