"""
fog_server.py

Fog node — scalable to ANY number of drones, with end-of-mission cloud archival.

For each drone in [0, num_drones), the fog subscribes to:
  - The drone's PX4 VehicleStatus topic
  - The drone's fog-tier Task topic   (/{drone_id}/task/fog)
  - The drone's camera topic          (/{drone_id}/camera/image)

And publishes a Task-2-style decision per drone on:
  - /fog/{drone_id}/decision

Mission control:
- /fog/start_mission partitions the search area into one cell per drone using
  RECURSIVE BISECTION (repeatedly cut the longer side, split the drones in
  half, recurse). This tiles the rectangle into N gap-free, area-balanced,
  near-square cells for ANY N. Each START_MISSION command carries the cell's
  full world-ENU rectangle plus its centroid, so the commander can sweep the
  whole cell.

Task 3.8 additions:
- Maintains an in-memory event buffer (bounded, drops oldest on overflow).
- Records each task arrival and each priority-3 alert as an event.
- Exposes a /fog/end_mission service. When called, flushes the buffer
  to the cloud as chunked batches on /fog/cloud/mission_log.
- During the mission the cloud topic carries zero traffic, keeping all
  fog compute focused on real-time work.
"""

import time
import json
from collections import deque
import math


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleStatus
from task_msgs.msg import Task

from fog_node.drone_naming import (
    drone_id_for,
    px4_topic_for,
)


# ----------------------------------------------------------------------
# Event buffer limits
# ----------------------------------------------------------------------
EVENT_BUFFER_SOFT_CAP = 10000   # drop oldest beyond this
BATCH_CHUNK_SIZE = 1000          # split flush into chunks of this size

# World origin from the Gazebo world SDF <spherical_coordinates>.
# This is the lat/lon of the ENU world frame origin (0,0). MUST match the
# value the commander uses.
WORLD_ORIGIN_LAT = 47.397971057728974
WORLD_ORIGIN_LON = 8.546163739800146

# Earth radius for the local-tangent-plane conversion.
_EARTH_RADIUS_M = 6371000.0


def enu_to_global(world_x, world_y):
    """
    Convert a world ENU position (meters East, meters North from the world
    origin) to global latitude/longitude using an equirectangular
    (local-tangent-plane) approximation. Accurate to well under a meter for
    the ~100 m distances used here.

    world_x = East (meters), world_y = North (meters).
    """
    origin_lat_rad = math.radians(WORLD_ORIGIN_LAT)
    d_lat = world_y / _EARTH_RADIUS_M
    d_lon = world_x / (_EARTH_RADIUS_M * math.cos(origin_lat_rad))
    lat = WORLD_ORIGIN_LAT + math.degrees(d_lat)
    lon = WORLD_ORIGIN_LON + math.degrees(d_lon)
    return lat, lon


def partition_area(min_x, max_x, min_y, max_y, n):
    """
    Recursive bisection of a rectangle into n gap-free, area-balanced,
    near-square cells, for ANY n >= 1.

    At each step we give floor(n/2) drones to one part and the rest to the
    other, cutting the rectangle's LONGER side proportionally so each part's
    area is proportional to its drone count. Cutting the longer side keeps the
    resulting cells close to square, which makes the per-cell lawnmower sweep
    efficient. The union of all returned cells is exactly the input rectangle
    (no gaps, no overlaps).

    Returns a list of n tuples: (cell_min_x, cell_max_x, cell_min_y, cell_max_y).
    """
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


def divide_area(area_min_x, area_max_x, area_min_y, area_max_y,
                scan_altitude, num_drones):
    """
    Partition the search area into one cell per drone (recursive bisection)
    and return an assignment per drone: the cell centroid (for a quick fly-to)
    AND the cell's full rectangle (for area coverage).

    Returns: dict { drone_id : {'world_x','world_y','lat','lon','alt',
                                'cell_index',
                                'min_x','max_x','min_y','max_y'} }
    """
    cells = partition_area(area_min_x, area_max_x,
                           area_min_y, area_max_y, num_drones)

    assignments = {}
    for i, (cmin_x, cmax_x, cmin_y, cmax_y) in enumerate(cells):
        cx = (cmin_x + cmax_x) / 2.0
        cy = (cmin_y + cmax_y) / 2.0
        lat, lon = enu_to_global(cx, cy)
        drone_id = f'drone{i}'
        assignments[drone_id] = {
            'world_x': round(cx, 2),
            'world_y': round(cy, 2),
            'lat': lat,
            'lon': lon,
            'alt': float(scan_altitude),
            'cell_index': i,
            'min_x': round(cmin_x, 2),
            'max_x': round(cmax_x, 2),
            'min_y': round(cmin_y, 2),
            'max_y': round(cmax_y, 2),
        }
    return assignments


class FogServer(Node):
    def __init__(self):
        super().__init__('fog_server')

        # ---- Parameters ----
        self.declare_parameter('num_drones', 3)
        self.num_drones = int(self.get_parameter('num_drones').value)
        if self.num_drones < 1:
            raise ValueError(f'num_drones must be >= 1, got {self.num_drones}')

        self.declare_parameter('area_min_x', -80.0)
        self.declare_parameter('area_max_x', 100.0)
        self.declare_parameter('area_min_y', -40.0)
        self.declare_parameter('area_max_y', 75.0)
        self.declare_parameter('scan_altitude', 12.0)   # meters above takeoff

        # Drone spawn locations in world ENU (East=x, North=y), matching each
        # drone's PX4_GZ_MODEL_POSE at launch. Defaults to the current 3-drone
        # setup; override with -p spawns_x:="[...]" -p spawns_y:="[...]".
        # There should be (at least) num_drones entries in each list.
        self.declare_parameter('spawns_x', [18.0, 23.0, 30.0])
        self.declare_parameter('spawns_y', [25.0, 25.0, 25.0])

        self.area_min_x = float(self.get_parameter('area_min_x').value)
        self.area_max_x = float(self.get_parameter('area_max_x').value)
        self.area_min_y = float(self.get_parameter('area_min_y').value)
        self.area_max_y = float(self.get_parameter('area_max_y').value)
        self.scan_altitude = float(self.get_parameter('scan_altitude').value)

        self.spawns_x = [float(v) for v in self.get_parameter('spawns_x').value]
        self.spawns_y = [float(v) for v in self.get_parameter('spawns_y').value]
        if len(self.spawns_x) != len(self.spawns_y):
            raise ValueError(
                f'spawns_x ({len(self.spawns_x)}) and spawns_y '
                f'({len(self.spawns_y)}) must have the same length.'
            )
        if len(self.spawns_x) < self.num_drones:
            self.get_logger().warn(
                f'[FOG] only {len(self.spawns_x)} spawn(s) given for '
                f'{self.num_drones} drone(s); drones beyond that will '
                f'auto-calibrate their spawn from PX4.'
            )


        # ---- PX4 QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ---- Per-drone state ----
        self.decision_publishers = {}
        self.mission_cmd_publishers = {}
        self.stats = {}

        for instance in range(self.num_drones):
            drone_id = drone_id_for(instance)
            status_topic = px4_topic_for(instance, 'vehicle_status_v1')
            task_topic = f'/{drone_id}/task/fog'
            camera_topic = f'/{drone_id}/camera/image'

            self.create_subscription(
                VehicleStatus, status_topic,
                lambda msg, d=drone_id: self.status_callback(msg, d),
                px4_qos,
            )
            self.create_subscription(
                Task, task_topic,
                lambda msg, d=drone_id: self.task_callback(msg, d),
                10,
            )
            self.create_subscription(
                Image, camera_topic,
                lambda msg, d=drone_id: self.camera_callback(msg, d),
                1,
            )

            decision_topic = f'/fog/{drone_id}/decision'
            self.decision_publishers[drone_id] = self.create_publisher(
                String, decision_topic, 10
            )

            # Mission command publisher (fog -> drone commander)
            mission_cmd_topic = f'/{drone_id}/mission_command'
            self.mission_cmd_publishers[drone_id] = self.create_publisher(
                String, mission_cmd_topic, 10
            )

            self.stats[drone_id] = {'status': 0, 'tasks': 0, 'frames': 0}

            self.get_logger().info(
                f'[FOG] {drone_id} (instance={instance}): '
                f'status={status_topic}, task={task_topic}, camera={camera_topic}'
            )


        # ---- Cloud archival infrastructure ----
        # bounded buffer of event dicts; oldest are evicted automatically
        self.event_buffer = deque(maxlen=EVENT_BUFFER_SOFT_CAP)
        self.events_dropped_on_overflow = 0
        self._prev_buffer_len = 0

        self.cloud_pub = self.create_publisher(
            String, '/fog/cloud/mission_log', 10
        )

        # end-of-mission service
        self.end_mission_srv = self.create_service(
            Trigger, '/fog/end_mission', self.end_mission_callback
        )

        self.start_mission_srv = self.create_service(
            Trigger, '/fog/start_mission', self.start_mission_callback
        )

        self.get_logger().info(
            f'[FOG] tracking {self.num_drones} drone(s)'
        )
        self.get_logger().info(
            '[FOG] cloud archival: buffer accumulates during mission, '
            'flushes via /fog/end_mission service'
        )

        # Periodic stats
        self.create_timer(5.0, self.log_stats)

    # ------------------------------------------------------------------
    # Event buffering helper
    # ------------------------------------------------------------------
    def _record_event(self, event_type: str, drone_id: str, payload: dict):
        """Append an event to the bounded buffer. Track silent overflow."""
        before_len = len(self.event_buffer)
        if before_len >= EVENT_BUFFER_SOFT_CAP:
            self.events_dropped_on_overflow += 1
        event = {
            'event_type': event_type,
            'drone_id': drone_id,
            'fog_received_at': time.time(),
            'payload': payload,
        }
        self.event_buffer.append(event)

    # ------------------------------------------------------------------
    # PX4 status path (Task 2 style decision)
    # ------------------------------------------------------------------
    def status_callback(self, msg: VehicleStatus, drone_id: str):
        self.stats[drone_id]['status'] += 1

        decision = String()
        if msg.arming_state == 2:
            decision.data = f'{drone_id}: COMMAND_MONITOR (ARMED)'
            self.get_logger().warn(f'[FOG ALERT] {drone_id} is ARMED')
        elif msg.nav_state == 4:
            decision.data = f'{drone_id}: COMMAND_HOLD_POSITION'
        else:
            decision.data = f'{drone_id}: COMMAND_NORMAL_OPERATION'

        time.sleep(0.05)  # short simulated processing — Task 3.7 will async-ify
        self.decision_publishers[drone_id].publish(decision)

    # ------------------------------------------------------------------
    # Task control plane (records every arrival as a mission-log event)
    # ------------------------------------------------------------------
    def task_callback(self, msg: Task, drone_id: str):
        self.stats[drone_id]['tasks'] += 1

        now_ns = self.get_clock().now().nanoseconds
        sent_ns = msg.timestamp.sec * 1_000_000_000 + msg.timestamp.nanosec
        latency_ms = (now_ns - sent_ns) / 1e6

        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except json.JSONDecodeError:
            payload = {'_parse_error': True}

        # Record event for future cloud archival
        self._record_event(
            event_type='TASK_RECEIVED',
            drone_id=drone_id,
            payload={
                'task_id': msg.task_id,
                'task_type': msg.task_type,
                'priority': int(msg.priority),
                'latency_ms': round(latency_ms, 2),
                'task_timestamp_sec': int(msg.timestamp.sec),
                'task_timestamp_nsec': int(msg.timestamp.nanosec),
                'payload_keys': list(payload.keys()),
            },
        )

        if msg.priority == 3:
            self.get_logger().warn(
                f'[FOG TASK CRITICAL] {drone_id} {msg.task_id} '
                f'type={msg.task_type} PRIORITY=3 latency={latency_ms:.1f}ms '
                f'failing={payload.get("drone_failing", False)}'
            )
            # Also record a separate ALERT event so it's easy to find in archives
            self._record_event(
                event_type='PRIORITY3_ALERT',
                drone_id=drone_id,
                payload={
                    'task_id': msg.task_id,
                    'task_type': msg.task_type,
                    'drone_failing': payload.get('drone_failing', False),
                    'position': payload.get('position'),
                },
            )
        else:
            self.get_logger().info(
                f'[FOG TASK] {drone_id} {msg.task_id} '
                f'type={msg.task_type} priority={msg.priority} '
                f'latency={latency_ms:.1f}ms payload_keys={list(payload.keys())}'
            )

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image, drone_id: str):
        self.stats[drone_id]['frames'] += 1
        # Task 4 places the detection model here.

    # ------------------------------------------------------------------
    # End-of-mission service: flush buffer to cloud as chunked batches
    # ------------------------------------------------------------------
    def end_mission_callback(self, request, response):
        events = list(self.event_buffer)
        total_events = len(events)

        if total_events == 0:
            response.success = True
            response.message = 'No events to archive.'
            self.get_logger().info(
                '[FOG END_MISSION] Buffer empty, nothing to flush.'
            )
            return response

        # Chunk the events
        chunks = [
            events[i:i + BATCH_CHUNK_SIZE]
            for i in range(0, total_events, BATCH_CHUNK_SIZE)
        ]
        total_batches = len(chunks)
        fog_timestamp = time.time()

        self.get_logger().info(
            f'[FOG END_MISSION] Flushing {total_events} events in '
            f'{total_batches} batch(es) to cloud.'
        )

        for idx, chunk in enumerate(chunks):
            batch = {
                'fog_timestamp': fog_timestamp,
                'batch_index': idx + 1,
                'total_batches': total_batches,
                'event_count': len(chunk),
                'events': chunk,
            }
            msg = String()
            msg.data = json.dumps(batch)
            self.cloud_pub.publish(msg)
            self.get_logger().info(
                f'[FOG END_MISSION] Published batch {idx + 1}/{total_batches} '
                f'({len(chunk)} events).'
            )

        # Clear the buffer after flush
        self.event_buffer.clear()
        dropped = self.events_dropped_on_overflow
        self.events_dropped_on_overflow = 0

        response.success = True
        response.message = (
            f'Flushed {total_events} events in {total_batches} batch(es). '
            f'Dropped during mission due to overflow: {dropped}.'
        )
        return response


    # ------------------------------------------------------------------
    # Start-of-mission service
    # ------------------------------------------------------------------
    def start_mission_callback(self, request, response):
        """
        Partition the search area among the drones and command each one to fly
        to (and sweep) its assigned cell. The command carries both the cell
        centroid and the cell's full world-ENU rectangle.
        """
        assignments = divide_area(
            self.area_min_x, self.area_max_x,
            self.area_min_y, self.area_max_y,
            self.scan_altitude, self.num_drones,
        )

        self.get_logger().info(
            f'[FOG START_MISSION] Partitioning area '
            f'X[{self.area_min_x},{self.area_max_x}] '
            f'Y[{self.area_min_y},{self.area_max_y}] '
            f'into {self.num_drones} cell(s).'
        )

        for drone_id, target in assignments.items():
            idx = target['cell_index']
            tgt = {
                'world_x': target['world_x'],   # ENU East  centroid
                'world_y': target['world_y'],   # ENU North centroid
                'alt': target['alt'],
                'lat': target['lat'],            # kept for reference / logging
                'lon': target['lon'],
                'area': {                        # full cell rectangle (ENU)
                    'min_x': target['min_x'],
                    'max_x': target['max_x'],
                    'min_y': target['min_y'],
                    'max_y': target['max_y'],
                },
            }
            # Attach this drone's spawn (from the fog params) so the commander
            # can convert world ENU -> its local NED frame.
            spawn_str = 'auto-calibrate'
            if idx < len(self.spawns_x):
                tgt['spawn'] = {'x': self.spawns_x[idx], 'y': self.spawns_y[idx]}
                spawn_str = f'({self.spawns_x[idx]}, {self.spawns_y[idx]})'

            cmd = {'command': 'START_MISSION', 'target': tgt}
            msg = String()
            msg.data = json.dumps(cmd)
            self.mission_cmd_publishers[drone_id].publish(msg)
            self.get_logger().info(
                f'[FOG START_MISSION] {drone_id} -> cell {idx} '
                f"centroid=({target['world_x']}, {target['world_y']}) "
                f"area X[{target['min_x']},{target['max_x']}] "
                f"Y[{target['min_y']},{target['max_y']}] "
                f"alt={target['alt']}m spawn={spawn_str}"
            )

        response.success = True
        response.message = (
            f'Mission started: {len(assignments)} drone(s) commanded to their cells.'
        )
        return response

    # ------------------------------------------------------------------
    def log_stats(self):
        parts = [
            f'{d}[s={c["status"]} t={c["tasks"]} f={c["frames"]}]'
            for d, c in self.stats.items()
        ]
        self.get_logger().info('[FOG STATS] ' + ' '.join(parts))

        # Periodic buffer status (only log when it changes meaningfully)
        cur_len = len(self.event_buffer)
        if cur_len != self._prev_buffer_len:
            self.get_logger().info(
                f'[FOG BUFFER] {cur_len} events buffered '
                f'(soft_cap={EVENT_BUFFER_SOFT_CAP}, '
                f'overflow_drops={self.events_dropped_on_overflow})'
            )
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