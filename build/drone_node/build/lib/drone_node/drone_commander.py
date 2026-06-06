"""
drone_commander.py

Drone-side flight command executor.

This node closes the loop between the fog's mission commands and PX4's
flight controller. It listens for high-level mission commands from the fog
on /{drone_id}/mission_command and translates them into PX4 VehicleCommand
messages that actually arm the drone, take it off, and fly it to a target.

For Step 1, the supported command is START_MISSION with a target position
expressed in global coordinates (latitude, longitude, altitude). The node
runs the sequence:

    arm -> takeoff -> (wait for climb) -> reposition to target -> hold

The fog computes targets in world ENU coordinates and converts them to
global lat/lon before sending, so this node receives absolute global
coordinates and does not need to know its own spawn offset. Using a global
frame means every drone shares the same coordinate system, which avoids
per-drone NED-origin bookkeeping.

Parameters:
- instance (int): PX4 instance index (0, 1, 2, ...). Derives all topic names.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
)
from std_msgs.msg import String

from drone_node.drone_naming import drone_id_for, px4_topic_for


# PX4 nav_state values we care about
NAV_STATE_AUTO_TAKEOFF = 17
NAV_STATE_AUTO_LOITER = 4
NAV_STATE_OFFBOARD = 14

# PX4 arming_state values
ARMING_STATE_ARMED = 2

# Mission phase machine
PHASE_IDLE = 'IDLE'
PHASE_ARMING = 'ARMING'
PHASE_TAKEOFF = 'TAKEOFF'
PHASE_ENROUTE = 'ENROUTE'
PHASE_HOLDING = 'HOLDING'


class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # ---- Parameters ----
        self.declare_parameter('instance', 0)
        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)

        # ---- Topic names ----
        cmd_in_topic = f'/{self.drone_id}/mission_command'
        px4_vehicle_command_topic = px4_topic_for(self.instance, 'vehicle_command').replace(
            '/fmu/out/', '/fmu/in/'
        )
        # Note: vehicle_command is an INPUT topic (we publish TO px4),
        # so it lives under /fmu/in/, not /fmu/out/.
        px4_status_topic = px4_topic_for(self.instance, 'vehicle_status_v1')
        px4_local_pos_topic = px4_topic_for(self.instance, 'vehicle_local_position_v1')
        px4_global_pos_topic = px4_topic_for(self.instance, 'vehicle_global_position')

        # ---- PX4 QoS (BEST_EFFORT + TRANSIENT_LOCAL for status/position) ----
        px4_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- State ----
        self.phase = PHASE_IDLE
        self.latest_status = None
        self.latest_local_pos = None
        self.latest_global_pos = None
        self.target = None              # dict {lat, lon, alt}
        self.takeoff_altitude = None    # meters AMSL we commanded for takeoff
        self.phase_entered_time = self.get_clock().now()

        # ---- Subscribers ----
        self.create_subscription(String, cmd_in_topic,
                                 self.mission_command_callback, 10)
        self.create_subscription(VehicleStatus, px4_status_topic,
                                 self.status_callback, px4_sub_qos)
        self.create_subscription(VehicleLocalPosition, px4_local_pos_topic,
                                 self.local_pos_callback, px4_sub_qos)
        self.create_subscription(VehicleGlobalPosition, px4_global_pos_topic,
                                 self.global_pos_callback, px4_sub_qos)

        # ---- Publisher to PX4 ----
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, px4_vehicle_command_topic, 10
        )

        # ---- Phase machine timer (2 Hz) ----
        self.create_timer(0.5, self.phase_machine_step)

        self.get_logger().info(
            f'[COMMANDER] {self.drone_id} (instance={self.instance}) ready.'
        )
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: listening for commands on {cmd_in_topic}'
        )
        self.get_logger().info(
            f'[COMMANDER] {self.drone_id}: sending VehicleCommand to {px4_vehicle_command_topic}'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def status_callback(self, msg):
        self.latest_status = msg

    def local_pos_callback(self, msg):
        self.latest_local_pos = msg

    def global_pos_callback(self, msg):
        self.latest_global_pos = msg

    def mission_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'[COMMANDER] {self.drone_id}: bad command JSON')
            return

        command = cmd.get('command', '')
        if command == 'START_MISSION':
            target = cmd.get('target', {})
            self.target = {
                'lat': float(target['lat']),
                'lon': float(target['lon']),
                'alt': float(target['alt']),   # meters above takeoff point
            }
            self.get_logger().info(
                f'[COMMANDER] {self.drone_id}: START_MISSION received, '
                f"target lat={self.target['lat']:.7f} lon={self.target['lon']:.7f} "
                f"alt={self.target['alt']:.1f}m"
            )
            self._enter_phase(PHASE_ARMING)
        elif command == 'RTL':
            self.get_logger().info(f'[COMMANDER] {self.drone_id}: RTL received')
            self._send_rtl()
            self._enter_phase(PHASE_IDLE)
        else:
            self.get_logger().warn(
                f'[COMMANDER] {self.drone_id}: unknown command "{command}"'
            )

    # ------------------------------------------------------------------
    # Phase machine
    # ------------------------------------------------------------------
    def _enter_phase(self, phase):
        self.phase = phase
        self.phase_entered_time = self.get_clock().now()
        self.get_logger().info(f'[COMMANDER] {self.drone_id}: phase -> {phase}')

    def _seconds_in_phase(self):
        return (self.get_clock().now() - self.phase_entered_time).nanoseconds / 1e9

    def phase_machine_step(self):
        if self.phase == PHASE_IDLE:
            return

        if self.latest_status is None:
            self.get_logger().warn(
                f'[COMMANDER] {self.drone_id}: no PX4 status yet, waiting...'
            )
            return

        if self.phase == PHASE_ARMING:
            # Send arm command, then move to takeoff after it confirms armed.
            self._send_arm()
            if self.latest_status.arming_state == ARMING_STATE_ARMED:
                self.get_logger().info(f'[COMMANDER] {self.drone_id}: armed.')
                self._enter_phase(PHASE_TAKEOFF)
            elif self._seconds_in_phase() > 10.0:
                self.get_logger().error(
                    f'[COMMANDER] {self.drone_id}: failed to arm after 10s. '
                    f'arming_state={self.latest_status.arming_state}. '
                    f'Check EKF convergence and pre-flight checks.'
                )
                self._enter_phase(PHASE_IDLE)

        elif self.phase == PHASE_TAKEOFF:
            # Command takeoff to target altitude.
            self._send_takeoff(self.target['alt'])
            # Consider takeoff complete when we're within 1.5 m of target alt.
            if self.latest_local_pos is not None:
                current_alt = -self.latest_local_pos.z  # NED z is down-positive
                if current_alt >= self.target['alt'] - 1.5:
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: reached takeoff altitude '
                        f'({current_alt:.1f}m).'
                    )
                    self._enter_phase(PHASE_ENROUTE)
            if self._seconds_in_phase() > 30.0:
                self.get_logger().warn(
                    f'[COMMANDER] {self.drone_id}: takeoff timeout, proceeding to enroute anyway.'
                )
                self._enter_phase(PHASE_ENROUTE)

        elif self.phase == PHASE_ENROUTE:
            # Send reposition to the global target.
            self._send_reposition(
                self.target['lat'], self.target['lon'], self.target['alt']
            )
            # Consider arrival when horizontal distance is small.
            if self.latest_global_pos is not None:
                dist = self._horizontal_distance_to_target()
                if dist is not None and dist < 3.0:
                    self.get_logger().info(
                        f'[COMMANDER] {self.drone_id}: arrived at target '
                        f'(within {dist:.1f}m).'
                    )
                    self._enter_phase(PHASE_HOLDING)
            if self._seconds_in_phase() > 60.0:
                self.get_logger().warn(
                    f'[COMMANDER] {self.drone_id}: enroute timeout, holding at current position.'
                )
                self._enter_phase(PHASE_HOLDING)

        elif self.phase == PHASE_HOLDING:
            # Re-send reposition periodically to keep it loitering at the target.
            if int(self._seconds_in_phase()) % 5 == 0:
                self._send_reposition(
                    self.target['lat'], self.target['lon'], self.target['alt']
                )

    # ------------------------------------------------------------------
    # Distance helper
    # ------------------------------------------------------------------
    def _horizontal_distance_to_target(self):
        if self.latest_global_pos is None or self.target is None:
            return None
        # Equirectangular approximation, fine for small distances.
        lat1 = math.radians(self.latest_global_pos.lat)
        lon1 = math.radians(self.latest_global_pos.lon)
        lat2 = math.radians(self.target['lat'])
        lon2 = math.radians(self.target['lon'])
        R = 6371000.0
        x = (lon2 - lon1) * math.cos((lat1 + lat2) / 2.0)
        y = (lat2 - lat1)
        return math.sqrt(x * x + y * y) * R

    # ------------------------------------------------------------------
    # VehicleCommand senders
    # ------------------------------------------------------------------
    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        # target this specific vehicle: PX4 SITL instance N uses sysid N+1
        msg.target_system = self.instance + 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _send_arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,   # 1 = arm
        )

    def _send_takeoff(self, altitude_m):
        # MAV_CMD_NAV_TAKEOFF. param7 is target altitude (AMSL).
        # We use the current global position's altitude + desired height.
        if self.latest_global_pos is not None:
            target_amsl = self.latest_global_pos.alt + altitude_m
        else:
            target_amsl = altitude_m
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=target_amsl,
        )

    def _send_reposition(self, lat, lon, altitude_m):
        # MAV_CMD_DO_REPOSITION. param5/6/7 are lat/lon/alt.
        # Altitude is AMSL; convert desired-height-above-home to AMSL using
        # the takeoff point's altitude if known.
        if self.latest_global_pos is not None and self.takeoff_altitude is None:
            # Capture home altitude the first time we know our global alt.
            pass
        target_amsl = altitude_m
        if self.latest_global_pos is not None:
            # Reposition altitude is AMSL; we want `altitude_m` above ground.
            # Use current alt minus current height-above-takeoff as ground ref.
            current_height = 0.0
            if self.latest_local_pos is not None:
                current_height = -self.latest_local_pos.z
            ground_amsl = self.latest_global_pos.alt - current_height
            target_amsl = ground_amsl + altitude_m
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
            param1=-1.0,   # ground speed: -1 = default
            param5=lat,
            param6=lon,
            param7=target_amsl,
        )

    def _send_rtl(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        )


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