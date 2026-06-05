"""
victim_detector.py

Victim detection node using YOLOv8n on CPU.

Subscribes to /droneN/camera/image, runs person detection,
publishes annotated image and detection Task messages.

Parameterised by 'instance' (PX4 instance index).
"""

import os
import json
import time

os.environ['CUDA_VISIBLE_DEVICES'] = ''  # force CPU

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from task_msgs.msg import Task

from drone_node.drone_naming import drone_id_for


CONFIDENCE_THRESHOLD = 0.25
PERSON_CLASS_ID = 0


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')

        # Parameters
        self.declare_parameter('instance', 0)
        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)

        # State
        self.model = None
        self.det_seq = 0

        # Subscriber (created before model load so ROS connects immediately)
        camera_topic = f'/{self.drone_id}/camera/image'
        self.create_subscription(Image, camera_topic, self.camera_callback, 10)

        # Publishers
        self.det_image_pub = self.create_publisher(
            Image, f'/{self.drone_id}/detection/image', 10)
        self.task_pub = self.create_publisher(
            Task, f'/{self.drone_id}/task/fog', 10)

        self.get_logger().info(
            f'[DETECTOR] {self.drone_id}: listening on {camera_topic}')

        # Load model via one-shot timer so ROS executor starts spinning first
        self.create_timer(0.1, self._load_model_once)

    def _load_model_once(self):
        """Load YOLO model after executor is running."""
        if self.model is not None:
            return  # already loaded

        from ultralytics import YOLO
        self.model = YOLO('yolov8n.pt')
        # Warmup
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model(dummy, verbose=False, device='cpu')
        self.get_logger().info(
            f'[DETECTOR] {self.drone_id}: YOLOv8n loaded and warmed up')

    def camera_callback(self, msg: Image):
        # Skip if model not ready yet
        if self.model is None:
            return

        # Convert ROS Image to numpy
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return

        # Run inference
        start = time.time()
        results = self.model(frame, verbose=False, device='cpu')
        inference_ms = (time.time() - start) * 1000

        # Extract person detections above threshold
        detections = []
        boxes = results[0].boxes
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i])
            conf = float(boxes.conf[i])
            if cls_id == PERSON_CLASS_ID and conf >= CONFIDENCE_THRESHOLD:
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                detections.append({
                    'bbox': [round(x1, 1), round(y1, 1),
                             round(x2, 1), round(y2, 1)],
                    'confidence': round(conf, 3),
                    'label': 'person',
                })

        # Draw bounding boxes on frame
        annotated = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"person {det['confidence']:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish annotated image
        det_msg = Image()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.header.frame_id = f'{self.drone_id}_detection'
        det_msg.height = annotated.shape[0]
        det_msg.width = annotated.shape[1]
        det_msg.encoding = 'rgb8'
        det_msg.step = annotated.shape[1] * 3
        det_msg.data = annotated.tobytes()
        self.det_image_pub.publish(det_msg)

        # Publish Task message if detections found
        if detections:
            task = Task()
            task.task_id = f'{self.drone_id}-victim-{self.det_seq:04d}'
            task.task_type = 'VICTIM_DETECTION'
            task.drone_id = self.drone_id
            task.timestamp = self.get_clock().now().to_msg()
            task.priority = 2  # high priority
            task.payload = json.dumps({
                'detections': detections,
                'frame_seq': self.det_seq,
                'inference_time_ms': round(inference_ms, 1),
                'num_persons': len(detections),
            })
            self.task_pub.publish(task)

            self.get_logger().warn(
                f'[DETECTION] {self.drone_id}: {len(detections)} person(s) '
                f'detected (inference={inference_ms:.0f}ms)')
        else:
            # Log periodically even when no detections
            if self.det_seq % 10 == 0:
                self.get_logger().info(
                    f'[DETECTOR] {self.drone_id}: frame {self.det_seq}, '
                    f'no persons (inference={inference_ms:.0f}ms)')

        self.det_seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
