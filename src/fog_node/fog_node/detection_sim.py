"""
detection_sim.py  —  Task 5.14 scenario driver.

Publishes synthetic Task-4-style victim alerts on /fog/victim_alerts so the
decision node can be exercised deterministically without running YOLO or flying.
The real Task 4 detector publishes the *same* contract, so nothing in the
decision node changes between simulated and real detections.

Scenarios (parameter 'scenario'):
  single     - one victim, one drone responds
  multiple   - several victims at once, multiple drones assigned by priority
  none       - no detections (drones keep searching)
  low_conf   - a weak detection that should trigger SCAN_AREA, not a rescue
  corroborate- two drones report the same spot; reports raise the priority

Note: world location of a victim is derived by the decision node from the
*reporting drone's* live position, so this sim just names a drone + confidence;
make sure that drone's PX4 instance is alive (or run alongside the offboard sim).
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def alert(drone_id, confidence, n=1):
    return json.dumps({
        'drone_id': drone_id,
        'num_persons': n,
        'detections': [{'bbox': [10, 10, 50, 80], 'confidence': confidence,
                        'label': 'person'} for _ in range(n)],
        'inference_time_ms': 42.0,
        'processed_at': 'sim',
        'timestamp': time.time(),
    })


SCENARIOS = {
    'single':      [(2.0, 'drone0', 0.82, 1)],
    'multiple':    [(2.0, 'drone0', 0.80, 1), (2.5, 'drone1', 0.74, 1),
                    (3.0, 'drone2', 0.69, 2)],
    'none':        [],
    'low_conf':    [(2.0, 'drone0', 0.30, 1)],
    'corroborate': [(2.0, 'drone0', 0.55, 1), (4.0, 'drone1', 0.61, 1),
                    (6.0, 'drone0', 0.58, 1)],
}


class DetectionSim(Node):
    def __init__(self):
        super().__init__('detection_sim')
        self.declare_parameter('scenario', 'single')
        self.scenario = str(self.get_parameter('scenario').value)
        self.pub = self.create_publisher(String, '/fog/victim_alerts', 10)

        script = SCENARIOS.get(self.scenario)
        if script is None:
            self.get_logger().error(f'[SIM] unknown scenario "{self.scenario}"')
            script = []
        self.get_logger().info(
            f'[SIM] scenario="{self.scenario}", {len(script)} alert(s) queued')

        self.start = time.time()
        self.script = sorted(script, key=lambda r: r[0])
        self.idx = 0
        self.create_timer(0.2, self.tick)

    def tick(self):
        t = time.time() - self.start
        while self.idx < len(self.script) and self.script[self.idx][0] <= t:
            _, drone_id, conf, n = self.script[self.idx]
            msg = String()
            msg.data = alert(drone_id, conf, n)
            self.pub.publish(msg)
            self.get_logger().warn(
                f'[SIM] t={t:.1f}s emit {drone_id} conf={conf} n={n}')
            self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
