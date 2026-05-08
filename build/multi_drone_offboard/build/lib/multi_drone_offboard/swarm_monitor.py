#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# SWARM MONITOR NODE — Task 1.7
#
# This node subscribes to the /status topic of each drone and prints
# a live swarm summary to the terminal every 2 seconds.
#
# Usage:
#   ros2 run multi_drone_offboard monitor
#
# It reads from these topics (published by offboard_control.py):
#   /px4_0/status
#   /px4_1/status
#   /px4_2/status
#
# To add more drones, add their namespace to the DRONE_NAMESPACES list below.
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# List of all drone namespaces to monitor.
# Add or remove namespaces here to match your swarm size.
DRONE_NAMESPACES = ['px4_0', 'px4_1', 'px4_2']

# How often to print the swarm summary (in seconds)
PRINT_INTERVAL = 2.0


class SwarmMonitor(Node):

    def __init__(self):
        super().__init__('swarm_monitor')

        # Dictionary to store the latest status string from each drone.
        # Key   = drone namespace (e.g. 'px4_0')
        # Value = latest status string received, or 'NO DATA' if nothing yet
        self.drone_status = {ns: 'NO DATA' for ns in DRONE_NAMESPACES}

        # Subscribe to the /status topic of every drone
        for ns in DRONE_NAMESPACES:
            self.create_subscription(
                String,
                f'/{ns}/status',
                self.make_callback(ns),   # each drone gets its own callback
                10)

        # Timer: print the swarm summary every PRINT_INTERVAL seconds
        self.create_timer(PRINT_INTERVAL, self.print_summary)

        self.get_logger().info(
            f'Swarm Monitor started. Watching {len(DRONE_NAMESPACES)} drones.')

    def make_callback(self, namespace):
        """
        Creates and returns a unique callback function for a given drone.
        This is necessary because all 3 subscriptions need separate callbacks
        that each know which drone they belong to.
        """
        def callback(msg):
            self.drone_status[namespace] = msg.data
        return callback

    def print_summary(self):
        """
        Prints a clean swarm status table to the terminal.
        Called every PRINT_INTERVAL seconds.
        """
        active_count = sum(
            1 for status in self.drone_status.values()
            if status != 'NO DATA'
        )

        print('\n' + '=' * 62)
        print('              SWARM STATUS MONITOR')
        print('=' * 62)

        for ns in DRONE_NAMESPACES:
            raw = self.drone_status[ns]

            if raw == 'NO DATA':
                print(f'  [{ns}]  *** NOT CONNECTED ***')
            else:
                # Parse the status string into parts for clean display
                # Format: drone_id=X | battery=X | position=(X,X,X) | task=X
                parts = {}
                for part in raw.split(' | '):
                    if '=' in part:
                        key, val = part.split('=', 1)
                        parts[key.strip()] = val.strip()

                battery  = parts.get('battery',  'N/A')
                position = parts.get('position', 'N/A')
                task     = parts.get('task',     'N/A')

                print(f'  [{ns}]  battery={battery:<8}  pos={position:<30}  task={task}')

        print('=' * 62)
        print(f'  Active drones: {active_count}/{len(DRONE_NAMESPACES)}')
        print('=' * 62)


def main():
    rclpy.init()
    node = SwarmMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nMonitor shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()