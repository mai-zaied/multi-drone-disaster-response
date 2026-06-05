import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DroneReactor(Node):
    def __init__(self):
        super().__init__('drone_reactor')

        self.declare_parameter('drone_id', 'drone0')
        self.drone_id = self.get_parameter('drone_id').value

        topic = f'/fog/{self.drone_id}/decision'

        self.subscription = self.create_subscription(
            String,
            topic,
            self.command_callback,
            10
        )

        self.get_logger().info(f'[DRONE REACTOR] {self.drone_id} listening to {topic}')

    def command_callback(self, msg):
        self.get_logger().warn(f'[DRONE ACTION] {self.drone_id} executing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DroneReactor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()