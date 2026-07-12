"""
victim_detector.py

LOCAL-tier victim detection node using YOLOv8n on CPU (Task 6 baseline C:
"fog + cloud unavailable -> detect on the drone itself").

Subscribes to /droneN/camera/image, runs person detection, and publishes:
  /{drone}/detection/image   annotated frame
  /{drone}/task/fog          Task VICTIM_DETECTION  (utilisation: local tier)
  /{drone}/task_status       'LOCAL_AI_PROCESSING'  (battery AI surcharge)
  /fog/victim_alerts         JSON alert  <-- THIS IS THE FIX

WHY THE /fog/victim_alerts PUBLICATION MATTERS (the time-metric fix)
-------------------------------------------------------------------
metrics_collector reads detection LATENCY from a single unified contract:
    /fog/victim_alerts -> inference_time_ms
and it closes COMPLETION time when the dispatched rescuer arrives. The old
local detector only emitted /{drone}/task/fog, so in LOCAL runs the collector
saw no inference latency at all (latency n=0) and completion rarely closed.

Publishing /fog/victim_alerts here puts LOCAL mode on the exact same measurement
path as FOG mode, so latency / response / completion are all recorded. The only
difference vs fog is `processed_at: 'local'` and that inference_time_ms is the
ON-DRONE inference time (no fog hop), which is precisely the metric we want to
compare. decision_node's existing dedup guards against re-dispatching the same
static victim across consecutive frames.

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
from std_msgs.msg import String
from task_msgs.msg import Task

from drone_node.drone_naming import drone_id_for


CONFIDENCE_THRESHOLD = 0.25
PERSON_CLASS_ID = 0


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')

        # Parameters
        self.declare_parameter('instance', 0)
        self.declare_parameter('emit_victim_alerts', True)
        # Same CPU-thrash fix as cloud_detector: three victim_detector
        # processes each letting torch grab every core starve one another
        # (plus Gazebo + 3x PX4) until the effective frame rate is so low the
        # victim is never in a processed frame. Cap threads per process.
        self.declare_parameter('torch_threads', 2)
        # Inference input size (same knob as cloud_detector). Default 640 =
        # YOLO/fog default. On a CPU-starved machine (the [LOCAL RATE] warning
        # firing with multi-second inference), THIS is the effective lever:
        # 320 typically cuts inference ~3-4x, 256 more, at the cost of some
        # small-object sensitivity. Thread-capping cannot rescue an 8000ms
        # inference; a smaller imgsz can.
        self.declare_parameter('imgsz', 640)
        # Inference cadence. camera_callback now only STORES the newest frame
        # (cheap) and this timer runs YOLO on whatever the latest frame is, so
        # a slow inference can never build a backlog of stale frames — the
        # detector always analyses the CURRENT view. 0.5s = try 2 Hz; on a
        # CPU-starved host the real rate is bounded by inference time anyway
        # (the [LOCAL RATE] watchdog reports the achieved rate).
        self.declare_parameter('process_period', 0.5)
        self.instance = int(self.get_parameter('instance').value)
        self.emit_alerts = bool(self.get_parameter('emit_victim_alerts').value)
        self.torch_threads = max(1, int(self.get_parameter('torch_threads').value))
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.process_period = float(self.get_parameter('process_period').value)
        self.drone_id = drone_id_for(self.instance)

        # State
        self.model = None
        self.det_seq = 0
        self._infer_ms_sum = 0.0
        self._rate_prev_seq = 0
        self._rate_prev_t = time.time()
        # Latest-frame buffer (single slot, newest wins) + frame bookkeeping.
        self._latest_frame = None
        self._frames_seen = 0
        self._last_frame_t = None

        # Subscriber (created before model load so ROS connects immediately).
        # Depth 1: if inference falls behind, ROS keeps only the newest frame
        # instead of queuing a stale backlog.
        camera_topic = f'/{self.drone_id}/camera/image'
        self.create_subscription(Image, camera_topic, self.camera_callback, 1)

        # Publishers
        self.det_image_pub = self.create_publisher(
            Image, f'/{self.drone_id}/detection/image', 10)
        self.task_pub = self.create_publisher(
            Task, f'/{self.drone_id}/task/fog', 10)
        # Task 6 utilisation (LOCAL tier): metrics_collector counts one unit
        # of local utilisation per Task on /{drone}/task/local — and nothing
        # ever published there, so every local-mode summary showed
        # utilisation local=0 (the /{drone}/task/fog VICTIM_DETECTION above
        # is the local-mode LATENCY path and only fires on detections; it is
        # counted as *fog* utilisation by the collector). One LOCAL_PROCESSING
        # task per inferred frame, mirroring cloud_detector's
        # /{drone}/task/cloud.
        self.local_util_pub = self.create_publisher(
            Task, f'/{self.drone_id}/task/local', 10)
        # Energy accounting: tells battery_simulator the drone is running YOLO
        # on-board (LOCAL mode), so it adds the AI surcharge. This is the whole
        # point of the local-vs-fog energy comparison.
        self.task_status_pub = self.create_publisher(
            String, f'/{self.drone_id}/task_status', 10)
        # THE FIX: feed the coordinator + the metrics collector on the unified
        # detection contract, so LOCAL mode records latency/response/completion.
        self.alert_pub = self.create_publisher(String, '/fog/victim_alerts', 10)

        self.get_logger().info(
            f'[DETECTOR] {self.drone_id}: listening on {camera_topic} '
            f'| victim_alerts={"on" if self.emit_alerts else "off"}')

        # Load model via one-shot timer so ROS executor starts spinning first
        self.create_timer(0.1, self._load_model_once)
        self.create_timer(10.0, self._report_rate)
        # Inference runs on a timer over the LATEST frame, not per-frame, so a
        # slow inference never processes stale backlog (see camera_callback).
        self.create_timer(self.process_period, self.process_latest)

    def _report_rate(self):
        """Every 10 s: achieved inference rate; WARN when CPU-starved so a
        0-detection local run can never end silently (same watchdog idea as
        cloud_detector's [CLOUD RATE])."""
        if self.model is None or self.det_seq == 0:
            return
        now = time.time()
        dt = max(1e-6, now - self._rate_prev_t)
        done = self.det_seq - self._rate_prev_seq
        eff_fps = done / dt
        mean_ms = (self._infer_ms_sum / done) if done else 0.0
        self._rate_prev_seq = self.det_seq
        self._rate_prev_t = now
        self._infer_ms_sum = 0.0
        line = (f'[LOCAL RATE] {self.drone_id}: {eff_fps:.2f} fps effective '
                f'(bridge publishes 2.0), mean inference {mean_ms:.0f}ms')
        if eff_fps < 0.5:
            self.get_logger().warn(
                line + ' -> CPU-STARVED: victims can be missed. Close other '
                       'CPU-heavy processes or lower torch_threads contention.')
        else:
            self.get_logger().info(line)

    def _load_model_once(self):
        """Load YOLO model after executor is running."""
        if self.model is not None:
            return  # already loaded

        # Thread caps must be in place before torch/ultralytics initialise
        # their pools (see torch_threads parameter above).
        os.environ.setdefault('OMP_NUM_THREADS', str(self.torch_threads))
        os.environ.setdefault('MKL_NUM_THREADS', str(self.torch_threads))
        import torch
        torch.set_num_threads(self.torch_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass   # pool already initialised; intra-op cap above still applies

        from ultralytics import YOLO
        self.model = YOLO('yolov8n.pt')
        # Warmup
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model(dummy, verbose=False, device='cpu', imgsz=self.imgsz)
        self.get_logger().info(
            f'[DETECTOR] {self.drone_id}: YOLOv8n loaded and warmed up '
            f'(threads={self.torch_threads}, imgsz={self.imgsz})')

    def camera_callback(self, msg: Image):
        # Cheap: just keep the NEWEST frame. No inference here, so a slow YOLO
        # can't back up a queue of stale frames. process_latest() (timer) does
        # the work on whatever is freshest, which is what lets a starved host
        # still detect the victim while the drone is actually over it.
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        except ValueError:
            return
        self._latest_frame = frame
        self._frames_seen += 1
        self._last_frame_t = time.time()

    def process_latest(self):
        # Skip if model not ready or no frame yet.
        if self.model is None or self._latest_frame is None:
            return
        frame = self._latest_frame

        # Run inference
        start = time.time()
        results = self.model(frame, verbose=False, device='cpu',
                             imgsz=self.imgsz)
        inference_ms = (time.time() - start) * 1000
        self._infer_ms_sum += inference_ms

        # Every processed frame runs YOLO on the drone -> keep the battery AI
        # surcharge on.
        st = String()
        st.data = f'LOCAL_AI_PROCESSING inference={inference_ms:.0f}ms'
        self.task_status_pub.publish(st)

        # Task 6 utilisation: one LOCAL_PROCESSING task per inferred frame
        # (see publisher comment in __init__).
        ut = Task()
        ut.task_id = f'{self.drone_id}-localproc-{self.det_seq:05d}'
        ut.task_type = 'LOCAL_PROCESSING'
        ut.drone_id = self.drone_id
        ut.timestamp = self.get_clock().now().to_msg()
        ut.priority = 1
        ut.payload = json.dumps({'inference_ms': round(inference_ms, 1)})
        self.local_util_pub.publish(ut)

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

        # Publish Task message + victim alert if detections found
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

            # Unified detection contract -> collector latency + coordinator
            # dispatch (-> response + completion time). inference_time_ms is the
            # ON-DRONE inference time: exactly the LOCAL-tier latency we compare.
            if self.emit_alerts:
                alert = String()
                alert.data = json.dumps({
                    'drone_id': self.drone_id,
                    'num_persons': len(detections),
                    'detections': detections,
                    'inference_time_ms': round(inference_ms, 1),
                    'processed_at': 'local',
                    'timestamp': time.time(),
                })
                self.alert_pub.publish(alert)

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
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()