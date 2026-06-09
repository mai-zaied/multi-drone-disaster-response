import csv
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MetricsCollector(Node):
    def __init__(self):
        super().__init__("metrics_collector")

        self.declare_parameter("mode", "fog")
        self.declare_parameter("scenario", "medium")
        self.declare_parameter("run_id", "run_01")
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("out_dir", "evaluation/results_real")

        self.mode = self.get_parameter("mode").value
        self.scenario = self.get_parameter("scenario").value
        self.run_id = self.get_parameter("run_id").value
        self.num_drones = int(self.get_parameter("num_drones").value)
        self.out_dir = self.get_parameter("out_dir").value

        os.makedirs(self.out_dir, exist_ok=True)

        self.events = {}
        self.rows = []

        self.create_subscription(String, "/fog/victim_alerts", self.detection_callback, 10)
        self.create_subscription(String, "/fog/decision_log", self.decision_callback, 10)
        self.create_subscription(String, "/decision/status", self.status_callback, 10)

        for i in range(self.num_drones):
            self.create_subscription(String, f"/drone{i}/feedback", self.feedback_callback, 10)
            self.create_subscription(String, f"/drone{i}/battery_status", self.battery_callback, 10)

        self.get_logger().info(
            f"[METRICS] Started | mode={self.mode} | scenario={self.scenario} | run={self.run_id}"
        )

    def now(self):
        return time.time()

    def make_task_id(self, text):
        return str(abs(hash(text)) % 1000000)

    def detection_callback(self, msg):
        t = self.now()
        task_id = self.make_task_id(msg.data)

        self.events[task_id] = {
            "task_id": task_id,
            "mode": self.mode,
            "scenario": self.scenario,
            "run_id": self.run_id,
            "detection_time": t,
            "decision_time": None,
            "feedback_time": None,
            "status_time": None,
            "battery_event_time": None,
            "detection_msg": msg.data,
            "decision_msg": "",
            "feedback_msg": "",
            "status_msg": "",
            "battery_msg": "",
        }

        self.get_logger().info(f"[DETECTION LOGGED] task={task_id} | {msg.data}")

    def decision_callback(self, msg):
        t = self.now()

        if not self.events:
            task_id = self.make_task_id(msg.data)
            self.events[task_id] = {
                "task_id": task_id,
                "mode": self.mode,
                "scenario": self.scenario,
                "run_id": self.run_id,
                "detection_time": None,
                "decision_time": t,
                "feedback_time": None,
                "status_time": None,
                "battery_event_time": None,
                "detection_msg": "",
                "decision_msg": msg.data,
                "feedback_msg": "",
                "status_msg": "",
                "battery_msg": "",
            }
        else:
            task_id = list(self.events.keys())[-1]
            self.events[task_id]["decision_time"] = t
            self.events[task_id]["decision_msg"] = msg.data

        self.get_logger().info(f"[DECISION LOGGED] {msg.data}")

    def feedback_callback(self, msg):
        t = self.now()

        if not self.events:
            return

        task_id = list(self.events.keys())[-1]
        self.events[task_id]["feedback_time"] = t
        self.events[task_id]["feedback_msg"] = msg.data

        self.get_logger().info(f"[FEEDBACK LOGGED] {msg.data}")

    def status_callback(self, msg):
        t = self.now()

        if not self.events:
            task_id = self.make_task_id(msg.data)
            self.events[task_id] = {
                "task_id": task_id,
                "mode": self.mode,
                "scenario": self.scenario,
                "run_id": self.run_id,
                "detection_time": None,
                "decision_time": None,
                "feedback_time": None,
                "status_time": t,
                "battery_event_time": None,
                "detection_msg": "",
                "decision_msg": "",
                "feedback_msg": "",
                "status_msg": msg.data,
                "battery_msg": "",
            }
        else:
            task_id = list(self.events.keys())[-1]
            self.events[task_id]["status_time"] = t
            self.events[task_id]["status_msg"] = msg.data

        self.get_logger().info(f"[STATUS LOGGED] {msg.data}")

    def battery_callback(self, msg):
        t = self.now()

        if not self.events:
            return

        task_id = list(self.events.keys())[-1]
        self.events[task_id]["battery_event_time"] = t
        self.events[task_id]["battery_msg"] = msg.data

    def compute_rows(self):
        rows = []

        for task_id, e in self.events.items():
            detection_time = e["detection_time"]
            decision_time = e["decision_time"]
            feedback_time = e["feedback_time"]

            latency = ""
            completion_time = ""

            if detection_time is not None and decision_time is not None:
                latency = round(decision_time - detection_time, 4)

            if detection_time is not None and feedback_time is not None:
                completion_time = round(feedback_time - detection_time, 4)

            completed = 1 if feedback_time is not None else 0
            detected = 1 if detection_time is not None else 0

            rows.append({
                "task_id": task_id,
                "mode": e["mode"],
                "scenario": e["scenario"],
                "run_id": e["run_id"],
                "latency_sec": latency,
                "completion_time_sec": completion_time,
                "detected": detected,
                "completed": completed,
                "detection_msg": e["detection_msg"],
                "decision_msg": e["decision_msg"],
                "feedback_msg": e["feedback_msg"],
                "status_msg": e["status_msg"],
                "battery_msg": e["battery_msg"],
            })

        return rows

    def save_csv(self):
        rows = self.compute_rows()

        output_file = os.path.join(
            self.out_dir,
            f"{self.mode}_{self.scenario}_{self.run_id}.csv"
        )

        fieldnames = [
            "task_id",
            "mode",
            "scenario",
            "run_id",
            "latency_sec",
            "completion_time_sec",
            "detected",
            "completed",
            "detection_msg",
            "decision_msg",
            "feedback_msg",
            "status_msg",
            "battery_msg",
        ]

        with open(output_file, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        self.get_logger().info(f"[METRICS SAVED] {output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
