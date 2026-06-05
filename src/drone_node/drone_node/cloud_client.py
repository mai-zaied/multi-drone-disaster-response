import json
import time
import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CloudClient(Node):
    def __init__(self):
        super().__init__('cloud_client')

        self.declare_parameter('cloud_url', 'http://10.211.55.5:5000/upload')
        self.cloud_url = self.get_parameter('cloud_url').value

        self.subscription = self.create_subscription(
            String,
            '/decision/status',
            self.status_callback,
            10
        )

        self.get_logger().info(f'[CLOUD CLIENT] Sending data to: {self.cloud_url}')

    def status_callback(self, msg):
        payload = {
            "source": "fog_decision_system",
            "timestamp": time.time(),
            "message": msg.data
        }

        start_time = time.time()

        try:
            response = requests.post(self.cloud_url, json=payload, timeout=5)
            delay = time.time() - start_time

            self.get_logger().info(
                f'[CLOUD UPLOAD] success | delay={delay:.3f}s | response={response.status_code}'
            )

        except Exception as e:
            self.get_logger().error(f'[CLOUD UPLOAD] failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CloudClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
