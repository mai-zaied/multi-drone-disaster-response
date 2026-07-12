#!/usr/bin/env python3
"""
cloud_detector.py — simulated CLOUD-tier victim detection (Task 6 baseline B:
"fog unavailable -> offload + detect on the cloud").

Models the cloud path: the drone uploads a frame over a slow WAN, the cloud runs
YOLO, and the result comes back a round-trip later.

WHAT CHANGED vs the previous version (the time-metric fix)
---------------------------------------------------------
The previous node sampled the camera at 1 Hz (process_period=1.0) while the
camera bridge publishes at 2 Hz. Over a ~7 min run it processed only ~30 frames
per drone and therefore CAUGHT THE STATIC VICTIM ZERO TIMES -> zero
/fog/victim_alerts -> the collector recorded latency/completion/response = null
for the whole cloud scenario.

Fixes:
  1. PROCESS AT THE BRIDGE RATE. process_period now defaults to 0.5 s (= 2 Hz),
     so every published frame is inferred, matching how fog_server detects. This
     is what makes the victim reliably caught and the cloud latency recorded.
  2. FRAME-STARVATION WATCHDOG. If no camera frames arrive for a few seconds the
     node logs a clear warning, so a silent "0 detections" run can never happen
     again without an obvious reason on screen (check the bridges / QoS / topic).
  3. Still NON-BLOCKING WAN delay (timer + delayed-release pump) and still feeds
     the coordinator via /fog/victim_alerts so completion time closes.
  4. CPU THREAD CAP + THROUGHPUT WATCHDOG (the "cloud run had utilisation but
     zero detections" fix). Even at process_period=0.5 the ACHIEVED rate
     collapsed in real runs: torch defaults to using every CPU core, so three
     cloud_detector processes + Gazebo + 3x PX4 thrash each other and one
     inference stretches to many seconds (observed: 51 processed frames per
     drone over 460 s = one per ~9 s — with frames that sparse the static
     victim is simply never inside a processed frame, hence detections = 0
     while utilisation still counts). torch_threads (default 2) caps each
     process BEFORE torch/ultralytics spin up their pools; imgsz (default 640,
     the YOLO/fog default) is exposed for weaker machines. A [CLOUD RATE] log
     every 10 s reports achieved fps vs target and WARNS when CPU-starved, so
     this failure mode can never again end a run silently. Rationale for
     making the simulated cloud's inference FAST rather than throttling it:
     the real cloud tier is a GPU server — its inference is quick and the WAN
     round-trip (still simulated, wan_min-wan_max, unchanged) is what makes
     the cloud path slow. A laptop-starved CPU inference made the simulated
     cloud slower than the simulated fog for the wrong reason.

Topics out:
  /{drone}/cloud/detection   JSON {total_ms, inference_ms, wan_delay_ms, detections}
  /fog/victim_alerts         JSON {drone_id, num_persons, detections, inference_time_ms}
  /{drone}/task_status       'CLOUD_UPLOAD ...'   (battery upload surcharge)
  /{drone}/task/cloud        Task                  (utilisation, Task 6.10)

Params: instance, wan_min (1.0), wan_max (3.0), process_period (0.5),
        conf_threshold (0.25), emit_victim_alerts (true), starvation_warn_sec (4.0),
        torch_threads (2), imgsz (640).
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
        self.declare_parameter('process_period', 0.5)   # was 1.0; match 2 Hz bridge
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('emit_victim_alerts', True)
        self.declare_parameter('starvation_warn_sec', 4.0)
        # THE THROUGHPUT FIX (see WHAT CHANGED #4 in the docstring): cap the
        # CPU threads each detector process uses. Without a cap, all three
        # cloud_detector processes let torch grab EVERY core, and together
        # with Gazebo + 3x PX4 they thrash each other so badly that one
        # "0.5 s-period" inference takes many seconds — a whole run can end
        # with ~0.1 effective fps per drone and the victim never inside a
        # processed frame (observed: utilisation cloud=51 per drone over a
        # 460 s run = one frame per ~9 s, detections = 0).
        self.declare_parameter('torch_threads', 2)
        # Inference input size. 640 = YOLO default (same as the fog tier —
        # keeps detection ability identical). If [CLOUD RATE] still reports
        # starvation on a weaker machine, 416 or 320 trades small-object
        # sensitivity for a large speedup.
        self.declare_parameter('imgsz', 640)

        self.instance = int(self.get_parameter('instance').value)
        self.drone_id = drone_id_for(self.instance)
        self.wan_min = float(self.get_parameter('wan_min').value)
        self.wan_max = float(self.get_parameter('wan_max').value)
        self.process_period = float(self.get_parameter('process_period').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.emit_alerts = bool(self.get_parameter('emit_victim_alerts').value)
        self.starvation_warn = float(self.get_parameter('starvation_warn_sec').value)
        self.torch_threads = max(1, int(self.get_parameter('torch_threads').value))
        self.imgsz = int(self.get_parameter('imgsz').value)

        # Thread caps MUST be set before torch/ultralytics initialise their
        # thread pools, which is why the import happens down here, after the
        # parameters are read, and the env vars are set first.
        os.environ.setdefault('OMP_NUM_THREADS', str(self.torch_threads))
        os.environ.setdefault('MKL_NUM_THREADS', str(self.torch_threads))
        import torch
        torch.set_num_threads(self.torch_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass   # already initialised elsewhere in this process; cap above still applies
        from ultralytics import YOLO
        self.model = YOLO('yolov8n.pt')
        # Warmup so the first real inference isn't a multi-second stall.
        self.model(np.zeros((480, 640, 3), dtype=np.uint8),
                   verbose=False, device='cpu', imgsz=self.imgsz)
        self.get_logger().info(
            f'[CLOUD DETECTOR] {self.drone_id}: YOLOv8n loaded '
            f'(threads={self.torch_threads}, imgsz={self.imgsz}), '
            f'WAN {self.wan_min}-{self.wan_max}s (non-blocking), '
            f'inference every {self.process_period}s (matches 2 Hz bridge)')

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
        self.processed_count = 0
        self.last_frame_time = None
        self.last_starv_warn = 0.0
        self.pending = deque()   # (due_at, result_dict)
        # Effective-rate watchdog state (WHAT CHANGED #4): the old frame
        # watchdog only caught "no frames arriving"; it could NOT catch
        # "frames arriving fine but inference too slow to keep up", which
        # produces the same silent 0-detection run.
        self._rate_prev_processed = 0
        self._rate_prev_t = time.time()
        self._infer_ms_sum = 0.0

        self.create_timer(self.process_period, self.process_latest)
        self.create_timer(0.05, self.pump_pending)        # 20 Hz release
        self.create_timer(2.0, self.check_starvation)     # frame watchdog
        self.create_timer(10.0, self.report_rate)         # throughput watchdog

    # ------------------------------------------------------------------
    def report_rate(self):
        """Every 10 s: log the ACHIEVED inference rate vs the target, and WARN
        loudly when the CPU can't keep up — so a starved run can never again
        end silently with detections=0."""
        now = time.time()
        dt = max(1e-6, now - self._rate_prev_t)
        done = self.processed_count - self._rate_prev_processed
        eff_fps = done / dt
        target_fps = 1.0 / self.process_period
        mean_ms = (self._infer_ms_sum / done) if done else 0.0
        self._rate_prev_processed = self.processed_count
        self._rate_prev_t = now
        self._infer_ms_sum = 0.0
        if self.processed_count == 0:
            return   # frame watchdog already covers the no-frames case
        line = (f'[CLOUD RATE] {self.drone_id}: {eff_fps:.2f} fps effective '
                f'(target {target_fps:.1f}), mean inference {mean_ms:.0f}ms, '
                f'received={self.frame_count} processed={self.processed_count}')
        if eff_fps < 0.5 * target_fps:
            self.get_logger().warn(
                line + ' -> CPU-STARVED: inference cannot keep up; victims can '
                       'be missed. Lower imgsz (e.g. -p imgsz:=416) or close '
                       'other CPU-heavy processes.')
        else:
            self.get_logger().info(line)

    # ------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        self.frame_count += 1
        self.last_frame_time = time.time()
        try:
            self.latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        except ValueError:
            self.latest_frame = None

    def check_starvation(self):
        """Make a silent 'no frames -> 0 detections' run impossible to miss."""
        now = time.time()
        if self.last_frame_time is None:
            if now - self.last_starv_warn > self.starvation_warn:
                self.last_starv_warn = now
                self.get_logger().warn(
                    f'[CLOUD DETECTOR] {self.drone_id}: NO camera frames yet on '
                    f'/{self.drone_id}/camera/image -> cloud detection cannot run. '
                    f'Is camera_bridge_simple (instance={self.instance}) up?')
            return
        gap = now - self.last_frame_time
        if gap > self.starvation_warn and now - self.last_starv_warn > self.starvation_warn:
            self.last_starv_warn = now
            self.get_logger().warn(
                f'[CLOUD DETECTOR] {self.drone_id}: no camera frame for {gap:.1f}s '
                f'(received={self.frame_count}, processed={self.processed_count}).')

    def process_latest(self):
        frame = self.latest_frame
        if frame is None:
            return
        self.processed_count += 1

        # The drone is uploading a frame right now -> battery upload surcharge.
        ts = String()
        ts.data = 'CLOUD_UPLOAD frame to cloud'
        self.task_status_pub.publish(ts)

        start = time.time()
        results = self.model(frame, verbose=False, device='cpu',
                             imgsz=self.imgsz)
        inference_ms = (time.time() - start) * 1000
        self._infer_ms_sum += inference_ms

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