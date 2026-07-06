import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
import random


class FakeDetectionPublisher(Node):

    def __init__(self):
        super().__init__('fake_detection_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/detection/results',
            10
        )

        self.timer = self.create_timer(5.0, self.publish_detection)

        self.get_logger().info(
            'Fake Detection Publisher Started...'
        )

    def publish_detection(self):

        drone_id = random.choice([
            'drone0',
            'drone1',
            'drone2'
        ])

        confidence = round(
            random.uniform(0.5, 0.99),
            2
        )

        detection_data = {
            "drone_id": drone_id,
            "detected": True,
            "confidence": confidence,
            "x": round(random.uniform(0, 50), 2),
            "y": round(random.uniform(0, 50), 2),
            "battery": random.randint(40, 100),
            "status": "available"
        }

        msg = String()

        msg.data = json.dumps(detection_data)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'[DETECTION] {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = FakeDetectionPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
