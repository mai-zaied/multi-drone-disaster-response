"""
cloud_detector.py

Simulated cloud-based victim detection.

Demonstrates why cloud processing is unsuitable for real-time detection:
adds a random WAN delay (1–3 seconds) before running inference.

Used for performance comparison only (local vs fog vs cloud).
"""

import os
import json
import time
import random

os.environ['CUDA_VISIBLE_DEVICES'] = ''

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from drone_node.drone_naming import drone_id_for


CONFIDENCE_THRESHOLD = 0.25
DELAY_MIN = 1.0  # seconds
DELAY_MAX = 3.0  # seconds


class CloudDetector(Node):
    def __init__(self):
        super().__init__('cloud_detector')

        self.declare_parameter('instance', 0)
        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)

        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(
            f'[CLOUD DETECTOR] {self.drone_id}: YOLOv8n loaded, '
            f'simulated WAN delay {DELAY_MIN}–{DELAY_MAX}s')

        camera_topic = f'/{self.drone_id}/camera/image'
        self.create_subscription(Image, camera_topic, self.camera_callback, 1)

        self.result_pub = self.create_publisher(
            String, f'/{self.drone_id}/cloud/detection', 10)

        self.frame_count = 0

    def camera_callback(self, msg: Image):
        self.frame_count += 1

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return

        # Simulate WAN delay
        delay = random.uniform(DELAY_MIN, DELAY_MAX)
        time.sleep(delay)

        # Run inference
        start = time.time()
        results = self.model(frame, verbose=False, device='cpu')
        inference_ms = (time.time() - start) * 1000
        total_ms = delay * 1000 + inference_ms

        # Extract person detections
        detections = []
        boxes = results[0].boxes
        for i in range(len(boxes)):
            if int(boxes.cls[i]) == 0 and float(boxes.conf[i]) >= CONFIDENCE_THRESHOLD:
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                detections.append({
                    'bbox': [round(x1, 1), round(y1, 1),
                             round(x2, 1), round(y2, 1)],
                    'confidence': round(float(boxes.conf[i]), 3),
                    'label': 'person',
                })

        result = String()
        result.data = json.dumps({
            'drone_id': self.drone_id,
            'frame': self.frame_count,
            'wan_delay_ms': round(delay * 1000, 0),
            'inference_ms': round(inference_ms, 1),
            'total_ms': round(total_ms, 1),
            'num_persons': len(detections),
            'detections': detections,
            'processed_at': 'cloud',
        })
        self.result_pub.publish(result)

        self.get_logger().info(
            f'[CLOUD DETECTOR] {self.drone_id}: frame {self.frame_count} '
            f'delay={delay*1000:.0f}ms + inference={inference_ms:.0f}ms = '
            f'total={total_ms:.0f}ms, detections={len(detections)}')


def main(args=None):
    rclpy.init(args=args)
    node = CloudDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
