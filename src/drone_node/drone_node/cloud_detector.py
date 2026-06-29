#!/usr/bin/env python3
"""
cloud_detector.py — simulated CLOUD-tier victim detection (Task 6 baseline).

Models the cloud path: the drone uploads a frame over a slow WAN, the cloud runs
YOLO, and the result comes back a round-trip later. Used for the local-vs-fog-vs
-cloud comparison and the "fog unavailable -> offload to cloud" scenario.

WHAT CHANGED vs the old version
-------------------------------
1. NON-BLOCKING WAN delay. The old node did `time.sleep(1-3 s)` inside the camera
   callback, freezing the whole executor every frame. Now the latest frame is
   cached, inference runs on a timer, and the result is queued for delayed
   publication (a 20 Hz pump releases it when its WAN round-trip elapses). The
   executor never blocks.

2. FEEDS THE COORDINATOR. The old node published ONLY /{drone}/cloud/detection,
   so decision_node never saw cloud detections and cloud-mode completion could
   never close. Now it ALSO publishes /fog/victim_alerts (after the WAN delay),
   so decision_node dispatches a rescuer and completion time is recorded.

3. ENERGY ACCOUNTING. Emits a CLOUD_UPLOAD marker on /{drone}/task_status so the
   battery sim adds the (small) upload surcharge, and a /{drone}/task/cloud Task
   for utilisation counting (Task 6.10).

Topics out:
  /{drone}/cloud/detection   JSON {total_ms, inference_ms, wan_delay_ms, detections}
  /fog/victim_alerts         JSON {drone_id, num_persons, detections, inference_time_ms}
  /{drone}/task_status       'CLOUD_UPLOAD ...'   (battery surcharge)
  /{drone}/task/cloud        Task                  (utilisation)

Params: instance, wan_min (1.0), wan_max (3.0), process_period (1.0),
        conf_threshold (0.25), emit_victim_alerts (true).
"""

import os
import json
import time
import random
from collections import deque

os.environ['CUDA_VISIBLE_DEVICES'] = ''

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from task_msgs.msg import Task
    _HAVE_TASK = True
except Exception:
    Task = None
    _HAVE_TASK = False

from drone_node.drone_naming import drone_id_for


def total_latency_ms(wan_delay_ms, inference_ms):
    """Pure: end-to-end cloud latency = round-trip WAN + cloud inference."""
    return round(wan_delay_ms + inference_ms, 1)


class CloudDetector(Node):
    def __init__(self):
        super().__init__('cloud_detector')

        self.declare_parameter('instance', 0)
        self.declare_parameter('wan_min', 1.0)
        self.declare_parameter('wan_max', 3.0)
        self.declare_parameter('process_period', 1.0)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('emit_victim_alerts', True)

        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)
        self.wan_min = float(self.get_parameter('wan_min').value)
        self.wan_max = float(self.get_parameter('wan_max').value)
        self.process_period = float(self.get_parameter('process_period').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.emit_alerts = bool(self.get_parameter('emit_victim_alerts').value)

        from ultralytics import YOLO
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info(
            f'[CLOUD DETECTOR] {self.drone_id}: YOLOv8n loaded, '
            f'WAN {self.wan_min}-{self.wan_max}s (non-blocking), '
            f'inference every {self.process_period}s')

        self.create_subscription(
            Image, f'/{self.drone_id}/camera/image', self.camera_callback, 1)

        self.result_pub = self.create_publisher(
            String, f'/{self.drone_id}/cloud/detection', 10)
        self.alert_pub = self.create_publisher(String, '/fog/victim_alerts', 10)
        self.task_status_pub = self.create_publisher(
            String, f'/{self.drone_id}/task_status', 10)
        if _HAVE_TASK:
            self.util_pub = self.create_publisher(
                Task, f'/{self.drone_id}/task/cloud', 10)

        self.latest_frame = None
        self.frame_count = 0
        self.pending = deque()   # (due_at, result_dict)

        self.create_timer(self.process_period, self.process_latest)
        self.create_timer(0.05, self.pump_pending)   # 20 Hz release

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        self.frame_count += 1
        try:
            self.latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        except ValueError:
            self.latest_frame = None

    def process_latest(self):
        frame = self.latest_frame
        if frame is None:
            return

        # The drone is uploading a frame right now -> battery upload surcharge.
        ts = String()
        ts.data = 'CLOUD_UPLOAD frame to cloud'
        self.task_status_pub.publish(ts)

        start = time.time()
        results = self.model(frame, verbose=False, device='cpu')
        inference_ms = (time.time() - start) * 1000

        detections = []
        boxes = results[0].boxes
        for i in range(len(boxes)):
            if int(boxes.cls[i]) == 0 and float(boxes.conf[i]) >= self.conf_th:
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                detections.append({
                    'bbox': [round(x1, 1), round(y1, 1),
                             round(x2, 1), round(y2, 1)],
                    'confidence': round(float(boxes.conf[i]), 3),
                    'label': 'person',
                })

        wan_delay = random.uniform(self.wan_min, self.wan_max)
        wan_ms = round(wan_delay * 1000, 0)
        total_ms = total_latency_ms(wan_ms, inference_ms)

        result = {
            'drone_id': self.drone_id,
            'frame': self.frame_count,
            'wan_delay_ms': wan_ms,
            'inference_ms': round(inference_ms, 1),
            'total_ms': total_ms,
            'num_persons': len(detections),
            'detections': detections,
            'processed_at': 'cloud',
        }
        # Release after the WAN round-trip so latency AND completion reflect it.
        self.pending.append((time.time() + wan_delay, result))

        if _HAVE_TASK:
            t = Task()
            t.task_id = f'{self.drone_id}-cloud-{self.frame_count:04d}'
            t.task_type = 'CLOUD_OFFLOAD'
            t.drone_id = self.drone_id
            t.timestamp = self.get_clock().now().to_msg()
            t.priority = 2
            t.payload = json.dumps({'total_ms': total_ms})
            self.util_pub.publish(t)

    def pump_pending(self):
        now = time.time()
        while self.pending and self.pending[0][0] <= now:
            _, result = self.pending.popleft()
            msg = String()
            msg.data = json.dumps(result)
            self.result_pub.publish(msg)
            self.get_logger().info(
                f'[CLOUD DETECTOR] {self.drone_id}: total={result["total_ms"]:.0f}ms '
                f'(wan={result["wan_delay_ms"]:.0f}+inf={result["inference_ms"]:.0f}) '
                f'persons={result["num_persons"]}')

            # Feed the coordinator so a rescuer is dispatched (completion time).
            if self.emit_alerts and result['detections']:
                alert = String()
                alert.data = json.dumps({
                    'drone_id': result['drone_id'],
                    'num_persons': result['num_persons'],
                    'detections': result['detections'],
                    'inference_time_ms': result['total_ms'],  # full cloud latency
                    'processed_at': 'cloud',
                    'timestamp': now,
                })
                self.alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = CloudDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
