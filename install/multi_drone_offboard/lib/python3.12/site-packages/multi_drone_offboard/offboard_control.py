#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    BatteryStatus,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DRONE0_MISSION = [
    { 'type': 'takeoff',  'altitude': 15.0 },
    { 'type': 'hover',    'duration': 5.0  },
    { 'type': 'forward',  'distance': 10.0 },
    { 'type': 'rotate',   'angle':    90.0 },
    { 'type': 'forward',  'distance': 10.0 },
    { 'type': 'hover',    'duration': 5.0  },
]

DRONE1_MISSION = [
    { 'type': 'takeoff',  'altitude': 15.0 },
    { 'type': 'hover',    'duration': 3.0  },
    { 'type': 'rotate',   'angle':    45.0 },
    { 'type': 'forward',  'distance': 15.0 },
    { 'type': 'hover',    'duration': 5.0  },
]

DRONE2_MISSION = [
    { 'type': 'takeoff',  'altitude': 15.0 },
    { 'type': 'forward',  'distance': 10.0 },
    { 'type': 'rotate',   'angle':   -90.0 },
    { 'type': 'forward',  'distance': 10.0 },
    { 'type': 'rotate',   'angle':   -90.0 },
    { 'type': 'hover',    'duration': 5.0  },
]

POSITION_THRESHOLD = 0.5
ANGLE_THRESHOLD    = 2.0


class DroneController(Node):

    def __init__(self, namespace, start_n, start_e, start_d, sysid, mission, use_namespace=True):
        super().__init__(f'{namespace}_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.ns      = namespace
        self.sysid   = sysid
        self.mission = mission
        self.pos_n   = start_n
        self.pos_e   = start_e
        self.pos_d   = start_d
        self.yaw     = 0.0
        self.target     = [start_n, start_e, start_d]
        self.target_yaw = 0.0
        self.step              = 0
        self.hover_timer       = 0.0
        self.mission_done      = False
        self._step_initialized = False
        self.current_task      = 'IDLE'
        self.battery_pct       = -1.0
        self.counter           = 0
        self.mode_sent         = False
        self.arm_sent          = False
        self.position_received = False

        if use_namespace:
            topic_prefix = f'/{namespace}/fmu/in'
            out_prefix   = f'/{namespace}/fmu/out'
        else:
            topic_prefix = '/fmu/in'
            out_prefix   = '/fmu/out'

        self.create_subscription(VehicleLocalPosition, f'{out_prefix}/vehicle_local_position_v1', self.position_callback, qos)
        self.create_subscription(BatteryStatus, f'{out_prefix}/battery_status_v1', self.battery_callback, qos)

        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{topic_prefix}/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint,  f'{topic_prefix}/trajectory_setpoint',  qos)
        self.command_pub  = self.create_publisher(VehicleCommand,      f'{topic_prefix}/vehicle_command',      qos)
        self.status_pub   = self.create_publisher(String, f'/{namespace}/status', 10)

        self.timer        = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'{namespace} controller started | sysid={sysid} | steps={len(mission)}')

    def position_callback(self, msg):
        self.pos_n = msg.x
        self.pos_e = msg.y
        self.pos_d = msg.z
        self.yaw   = msg.heading
        if not self.position_received:
            self.position_received = True
            self.get_logger().info(f'{self.ns}: Real position data received. Ready.')

    def battery_callback(self, msg):
        self.battery_pct = msg.remaining * 100.0

    def publish_status(self):
        battery_str = f'{self.battery_pct:.1f}%' if self.battery_pct >= 0 else 'N/A'
        status = (
            f'drone_id={self.ns} | '
            f'battery={battery_str} | '
            f'position=({self.pos_n:.1f}, {self.pos_e:.1f}, {self.pos_d:.1f}) | '
            f'task={self.current_task}'
        )
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def now(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = self.sysid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def distance_to_target(self):
        dn = self.target[0] - self.pos_n
        de = self.target[1] - self.pos_e
        dd = self.target[2] - self.pos_d
        return math.sqrt(dn*dn + de*de + dd*dd)

    def angle_error(self):
        diff = math.degrees(self.target_yaw - self.yaw)
        diff = (diff + 180) % 360 - 180
        return abs(diff)

    def reached_target(self):
        return self.distance_to_target() < POSITION_THRESHOLD

    def reached_yaw(self):
        return self.angle_error() < ANGLE_THRESHOLD

    def takeoff(self, altitude):
        self.target       = [self.pos_n, self.pos_e, -altitude]
        self.target_yaw   = self.yaw
        self.current_task = f'TAKEOFF({altitude}m)'
        self.get_logger().info(f'{self.ns}: TAKEOFF to {altitude}m')

    def hover(self):
        self.target       = [self.pos_n, self.pos_e, self.pos_d]
        self.target_yaw   = self.yaw
        self.current_task = 'HOVER'
        self.get_logger().info(f'{self.ns}: HOVER')

    def move_forward(self, distance):
        new_n = self.pos_n + distance * math.cos(self.yaw)
        new_e = self.pos_e + distance * math.sin(self.yaw)
        self.target       = [new_n, new_e, self.pos_d]
        self.target_yaw   = self.yaw
        self.current_task = f'FORWARD({distance}m)'
        self.get_logger().info(f'{self.ns}: MOVE FORWARD {distance}m -> target=({new_n:.1f}, {new_e:.1f})')

    def rotate(self, angle_deg):
        new_yaw = self.yaw + math.radians(angle_deg)
        new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi
        self.target_yaw   = new_yaw
        self.target       = [self.pos_n, self.pos_e, self.pos_d]
        self.current_task = f'ROTATE({angle_deg}deg)'
        self.get_logger().info(f'{self.ns}: ROTATE {angle_deg}deg -> target_yaw={math.degrees(new_yaw):.1f}deg')

    def run_mission(self):
        if self.mission_done:
            return
        if self.step >= len(self.mission):
            if self.current_task != 'MISSION COMPLETE':
                self.get_logger().info(f'{self.ns}: Mission complete.')
                self.current_task = 'MISSION COMPLETE'
            self.mission_done = True
            return

        cmd = self.mission[self.step]

        if cmd['type'] == 'takeoff':
            if not self._step_initialized:
                self.takeoff(cmd['altitude'])
                self._step_initialized = True
            if self.reached_target():
                self.get_logger().info(f'{self.ns}: Takeoff complete.')
                self._advance_step()

        elif cmd['type'] == 'hover':
            if not self._step_initialized:
                self.hover()
                self.hover_timer = 0.0
                self._step_initialized = True
            self.hover_timer += 0.1
            if self.hover_timer >= cmd['duration']:
                self.get_logger().info(f'{self.ns}: Hover complete ({cmd["duration"]}s).')
                self._advance_step()

        elif cmd['type'] == 'forward':
            if not self._step_initialized:
                self.move_forward(cmd['distance'])
                self._step_initialized = True
            if self.reached_target():
                self.get_logger().info(f'{self.ns}: Move forward complete.')
                self._advance_step()

        elif cmd['type'] == 'rotate':
            if not self._step_initialized:
                self.rotate(cmd['angle'])
                self._step_initialized = True
            if self.reached_yaw():
                self.get_logger().info(f'{self.ns}: Rotate complete.')
                self._advance_step()

    def _advance_step(self):
        self.step += 1
        self._step_initialized = False

    def control_loop(self):
        offboard = OffboardControlMode()
        offboard.timestamp    = self.now()
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        sp = TrajectorySetpoint()
        sp.timestamp = self.now()
        sp.position  = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        sp.yaw       = float(self.target_yaw)
        self.setpoint_pub.publish(sp)

        if self.counter == 20 and not self.mode_sent:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.mode_sent = True
            self.get_logger().info(f'{self.ns}: Offboard mode set.')

        if self.counter == 30 and not self.arm_sent:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_sent = True
            self.get_logger().info(f'{self.ns}: Arm command sent.')

        if self.counter > 30 and self.position_received:
            self.run_mission()

        self.counter += 1


def main():
    rclpy.init()

    drone0 = DroneController('px4_0', 18.0, 25.0, 0.0, sysid=1, mission=DRONE0_MISSION, use_namespace=False)
    drone1 = DroneController('px4_1', 23.0, 25.0, 0.0, sysid=2, mission=DRONE1_MISSION, use_namespace=True)
    drone2 = DroneController('px4_2', 30.0, 25.0, 0.0, sysid=3, mission=DRONE2_MISSION, use_namespace=True)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(drone0)
    executor.add_node(drone1)
    executor.add_node(drone2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\nShutting down...')

    drone0.destroy_node()
    drone1.destroy_node()
    drone2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()