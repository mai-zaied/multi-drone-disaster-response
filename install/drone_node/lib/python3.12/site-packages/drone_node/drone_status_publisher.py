import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DroneStatusPublisher(Node):
    def __init__(self):
        super().__init__('drone_status_publisher')

        self.declare_parameter('drone_id', 'drone1')
        self.drone_id = self.get_parameter('drone_id').value

        topic_name = f'/{self.drone_id}/status'
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.counter = 0

        self.get_logger().info(f'[SIM DRONE] {self.drone_id} publishing on {topic_name}')

    def publish_status(self):
        msg = String()
        msg.data = (f"drone_id={self.drone_id}, battery=90, "
                    f"position=(1,2,3), state=SEARCHING, count={self.counter}")
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = DroneStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()