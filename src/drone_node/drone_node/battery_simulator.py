import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('initial_battery', 100.0)
        self.declare_parameter('period', 2.0)
        self.declare_parameter('low_threshold', 30.0)
        self.declare_parameter('critical_threshold', 10.0)

        self.drone_id = self.get_parameter('drone_id').value
        self.battery = float(self.get_parameter('initial_battery').value)
        self.period = float(self.get_parameter('period').value)
        self.low_threshold = float(self.get_parameter('low_threshold').value)
        self.critical_threshold = float(self.get_parameter('critical_threshold').value)

        self.current_mode = "IDLE"
        self.low_sent = False
        self.critical_sent = False
        self.dead_sent = False

        self.battery_pub = self.create_publisher(
            String,
            f'/{self.drone_id}/battery_status',
            10
        )

        self.decision_pub = self.create_publisher(
            String,
            '/decision/status',
            10
        )

        self.command_sub = self.create_subscription(
            String,
            f'/{self.drone_id}/mission_command',
            self.command_callback,
            10
        )

        self.task_sub = self.create_subscription(
            String,
            f'/{self.drone_id}/task_status',
            self.task_callback,
            10
        )

        self.timer = self.create_timer(self.period, self.update_battery)

        self.get_logger().info(
            f'[BATTERY SIM] Started for {self.drone_id} | initial={self.battery}%'
        )

    def command_callback(self, msg):
        command = msg.data.upper()

        if "GO_TO" in command:
            self.current_mode = "FLYING_TO_TARGET"
        elif "SCAN_AREA" in command:
            self.current_mode = "SCANNING"
        elif "HOVER" in command:
            self.current_mode = "HOVERING"
        elif "RETURN_HOME" in command:
            self.current_mode = "RETURNING_HOME"
        else:
            self.current_mode = "IDLE"

        self.get_logger().info(
            f'[BATTERY MODE] {self.drone_id} mode changed to {self.current_mode}'
        )

    def task_callback(self, msg):
        task = msg.data.upper()

        if "DETECTION" in task or "AI" in task or "PROCESSING" in task:
            self.current_mode = "HEAVY_PROCESSING"
        elif "CLOUD" in task or "UPLOAD" in task:
            self.current_mode = "CLOUD_UPLOAD"
        elif "SEARCH" in task:
            self.current_mode = "SCANNING"

        self.get_logger().info(
            f'[BATTERY TASK] {self.drone_id} task mode = {self.current_mode}'
        )

    def calculate_drain(self):
        base_drain = 0.20

        mode_drain = {
            "IDLE": 0.10,
            "HOVERING": 0.35,
            "SCANNING": 0.55,
            "FLYING_TO_TARGET": 0.75,
            "RETURNING_HOME": 0.65,
            "HEAVY_PROCESSING": 1.10,
            "CLOUD_UPLOAD": 0.85
        }

        return base_drain + mode_drain.get(self.current_mode, 0.20)

    def update_battery(self):
        if self.battery <= 0:
            return

        drain = self.calculate_drain()
        self.battery = max(0.0, self.battery - drain)

        status_msg = String()
        status_msg.data = (
            f'{self.drone_id}: battery={self.battery:.2f}% | '
            f'mode={self.current_mode} | drain={drain:.2f}%/{self.period}s'
        )

        self.battery_pub.publish(status_msg)
        self.get_logger().info(f'[BATTERY] {status_msg.data}')

        if self.battery <= self.low_threshold and not self.low_sent:
            self.low_sent = True
            self.publish_decision_event('LOW_BATTERY')

        if self.battery <= self.critical_threshold and not self.critical_sent:
            self.critical_sent = True
            self.publish_decision_event('DRONE_FAILING_RETURN_HOME')

        if self.battery <= 0 and not self.dead_sent:
            self.dead_sent = True
            self.publish_decision_event('BATTERY_DEAD')

    def publish_decision_event(self, event_type):
        msg = String()
        msg.data = (
            f'{self.drone_id}: {event_type} | '
            f'battery={self.battery:.2f}% | '
            f'mode={self.current_mode} | '
            f'timestamp={time.time()}'
        )

        self.decision_pub.publish(msg)
        self.get_logger().warn(f'[BATTERY EVENT] {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
