#!/usr/bin/env python3
"""
metrics_collector.py — Task 6 instrumentation (local / fog / cloud).

Runs as a passive observer node alongside one experiment. It listens to the
real ROS 2 topics the system already publishes and records, per detected
victim, the Task 6 metrics:

    latency_sec           detection -> decision (or detection->result)
    comm_delay_sec        transport/processing portion of the latency
    completion_time_sec   detection -> drone arrival at victim
    detected / completed  success flags
    utilisation           tasks routed local / fog / cloud (per drone)

One run = one (mode, scenario, run_id) combination. Launch it, run the
mission, Ctrl-C it; it writes:

    <out_dir>/<mode>_<scenario>_<run_id>.csv          (one row per detection)
    <out_dir>/<mode>_<scenario>_<run_id>.summary.json (per-run aggregates)

Parameters
----------
mode         local | fog | cloud      which processing path is under test
scenario     small | medium | large | congestion | failure   (free label)
run_id       e.g. run_01 ... run_10
num_drones   number of drone instances launched
num_victims  GROUND TRUTH victim count in the scene (for success rate; 0=unknown)
fault        none | fog_down | drone_down | comm_delay   (reliability tag)
out_dir      output directory (default evaluation/results_real)

Topic sources (all real, already emitted by the system)
-------------------------------------------------------
fog detection   /fog/victim_alerts        JSON {drone_id, num_persons, detections[]}
fog decision    /fog/decision_log         JSON {kind: ASSIGNED|RESOLVED, drone, ts}
completion      /{drone}/mission_feedback JSON {state: ARRIVED|HOLDING|...}
local detection /{drone}/task/fog Task     VICTIM_DETECTION payload.inference_time_ms
cloud detection /{drone}/cloud/detection  JSON {total_ms, inference_ms, detections[]}
utilisation     /{drone}/task/{local,fog,cloud} Task message counts
battery/status  /{drone}/battery_status, /decision/status   (reliability markers)
"""

import csv
import json
import os
import statistics
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from task_msgs.msg import Task
    _HAVE_TASK = True
except Exception:  # task_msgs not built / workspace not sourced
    Task = None
    _HAVE_TASK = False


CSV_FIELDS = [
    "task_id", "mode", "scenario", "run_id", "fault", "num_drones", "drone",
    "latency_sec", "comm_delay_sec", "completion_time_sec",
    "detected", "completed", "confidence", "num_persons", "detail",
]

ARRIVAL_STATES = ("ARRIVED", "HOLDING")


class MetricsCollector(Node):
    def __init__(self):
        super().__init__("metrics_collector")

        self.declare_parameter("mode", "fog")
        self.declare_parameter("scenario", "medium")
        self.declare_parameter("run_id", "run_01")
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("num_victims", 0)
        self.declare_parameter("fault", "none")
        self.declare_parameter("out_dir", "evaluation/results_real")

        self.mode = str(self.get_parameter("mode").value).lower()
        self.scenario = str(self.get_parameter("scenario").value)
        self.run_id = str(self.get_parameter("run_id").value)
        self.num_drones = int(self.get_parameter("num_drones").value)
        self.num_victims = int(self.get_parameter("num_victims").value)
        self.fault = str(self.get_parameter("fault").value)
        self.out_dir = str(self.get_parameter("out_dir").value)
        os.makedirs(self.out_dir, exist_ok=True)

        # records: one dict per detection event
        self.records = []
        # per-drone FIFO of record indices still awaiting decision/completion
        self.open_by_drone = defaultdict(deque)
        # utilisation counters: drone -> {local, fog, cloud}
        self.util = defaultdict(lambda: {"local": 0, "fog": 0, "cloud": 0})
        self.seq = 0
        self.t_start = time.time()

        # ---- Fog path ----
        self.create_subscription(String, "/fog/victim_alerts", self.fog_alert_cb, 10)
        self.create_subscription(String, "/fog/decision_log", self.decision_cb, 10)
        self.create_subscription(String, "/decision/status", self.status_cb, 10)

        # ---- Per-drone ----
        for i in range(self.num_drones):
            did = f"drone{i}"
            self.create_subscription(
                String, f"/{did}/cloud/detection",
                lambda m, d=did: self.cloud_cb(m, d), 10)
            self.create_subscription(
                String, f"/{did}/mission_feedback",
                lambda m, d=did: self.feedback_cb(m, d), 10)
            self.create_subscription(
                String, f"/{did}/battery_status",
                lambda m, d=did: self.battery_cb(m, d), 10)
            if _HAVE_TASK:
                self.create_subscription(
                    Task, f"/{did}/task/local",
                    lambda m, d=did: self.util_cb(d, "local"), 10)
                self.create_subscription(
                    Task, f"/{did}/task/fog",
                    lambda m, d=did: self.taskfog_cb(m, d), 10)
                self.create_subscription(
                    Task, f"/{did}/task/cloud",
                    lambda m, d=did: self.util_cb(d, "cloud"), 10)

        self.get_logger().info(
            f"[METRICS] mode={self.mode} scenario={self.scenario} run={self.run_id} "
            f"drones={self.num_drones} victims={self.num_victims} fault={self.fault} "
            f"-> {self.out_dir}")
        if not _HAVE_TASK:
            self.get_logger().warn(
                "[METRICS] task_msgs not importable -> utilisation + local-mode "
                "latency disabled. Source the workspace before launching.")

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _max_conf(dets):
        c = 0.0
        for d in dets or []:
            try:
                c = max(c, float(d.get("confidence", 0.0)))
            except Exception:
                pass
        return c

    def _new_record(self, drone, t_detect, latency=None,
                    confidence="", num_persons="", detail=""):
        rec = {
            "task_id": f"{self.mode}-{self.seq:05d}",
            "mode": self.mode, "scenario": self.scenario, "run_id": self.run_id,
            "fault": self.fault, "num_drones": self.num_drones, "drone": drone,
            "t_detect": t_detect, "t_decision": None, "t_complete": None,
            "latency_sec": "" if latency is None else round(latency, 4),
            "comm_delay_sec": "", "completion_time_sec": "",
            "detected": 1, "completed": 0,
            "confidence": confidence, "num_persons": num_persons,
            "detail": detail,
        }
        self.seq += 1
        idx = len(self.records)
        self.records.append(rec)
        self.open_by_drone[drone].append(idx)
        return rec

    def _oldest_open(self, drone, field):
        """Oldest open record for a drone whose `field` is still unset."""
        for idx in self.open_by_drone.get(drone, ()):
            if self.records[idx][field] is None:
                return self.records[idx]
        return None

    # ------------------------------------------------------------------
    # detection handlers (mode-specific)
    # ------------------------------------------------------------------
    def fog_alert_cb(self, msg):
        if self.mode != "fog":
            return
        try:
            a = json.loads(msg.data)
        except Exception:
            return
        drone = a.get("drone_id", "drone?")
        conf = round(self._max_conf(a.get("detections", [])), 3)
        n = int(a.get("num_persons", 1))
        self._new_record(drone, time.time(), confidence=conf,
                         num_persons=n, detail="fog_alert")
        self.get_logger().info(f"[DETECT/fog] {drone} conf={conf}")

    def cloud_cb(self, msg, drone):
        if self.mode != "cloud":
            return
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        dets = r.get("detections", [])
        if not dets:
            return
        total_ms = float(r.get("total_ms", 0.0))
        infer_ms = float(r.get("inference_ms", 0.0))
        rec = self._new_record(
            drone, time.time(), latency=total_ms / 1000.0,
            confidence=round(self._max_conf(dets), 3),
            num_persons=len(dets), detail="cloud_detection")
        # comm delay = WAN portion (total - local inference)
        rec["comm_delay_sec"] = round(max(0.0, (total_ms - infer_ms) / 1000.0), 4)
        self.get_logger().info(
            f"[DETECT/cloud] {drone} latency={rec['latency_sec']}s "
            f"(wan={rec['comm_delay_sec']}s)")

    def taskfog_cb(self, msg, drone):
        # utilisation count for the fog tier
        self.util[drone]["fog"] += 1
        # local-mode detection: VICTIM_DETECTION carries on-drone inference time
        if self.mode != "local":
            return
        if getattr(msg, "task_type", "") != "VICTIM_DETECTION":
            return
        try:
            payload = json.loads(msg.payload) if msg.payload else {}
        except Exception:
            payload = {}
        infer_ms = float(payload.get("inference_time_ms", 0.0))
        n = int(payload.get("num_persons", 1))
        conf = round(self._max_conf(payload.get("detections", [])), 3)
        self._new_record(drone, time.time(), latency=infer_ms / 1000.0,
                         confidence=conf, num_persons=n, detail="local_inference")
        self.get_logger().info(
            f"[DETECT/local] {drone} latency={infer_ms/1000.0:.4f}s")

    def util_cb(self, drone, tier):
        self.util[drone][tier] += 1

    # ------------------------------------------------------------------
    # decision + completion handlers
    # ------------------------------------------------------------------
    def decision_cb(self, msg):
        if self.mode != "fog":
            return
        try:
            rec = json.loads(msg.data)
        except Exception:
            return
        kind = str(rec.get("kind", "")).upper()
        drone = rec.get("drone")
        if drone is None:
            return
        if kind == "ASSIGNED":
            r = self._oldest_open(drone, "t_decision")
            if r is not None:
                r["t_decision"] = time.time()
                lat = r["t_decision"] - r["t_detect"]
                r["latency_sec"] = round(lat, 4)
                r["comm_delay_sec"] = round(lat, 4)  # alert->command transit+compute
                self.get_logger().info(
                    f"[DECISION/fog] {drone} latency={r['latency_sec']}s")
        elif kind == "RESOLVED":
            self._close(drone)

    def feedback_cb(self, msg, drone):
        try:
            fb = json.loads(msg.data)
        except Exception:
            return
        if str(fb.get("state", "")).upper() in ARRIVAL_STATES:
            self._close(drone)

    def _close(self, drone):
        r = self._oldest_open(drone, "t_complete")
        if r is None:
            return
        r["t_complete"] = time.time()
        r["completion_time_sec"] = round(r["t_complete"] - r["t_detect"], 4)
        r["completed"] = 1
        # pop this index from the open queue
        q = self.open_by_drone.get(drone)
        if q:
            for k, idx in enumerate(q):
                if self.records[idx] is r:
                    del q[k]
                    break
        self.get_logger().info(
            f"[COMPLETE] {drone} completion={r['completion_time_sec']}s")

    def status_cb(self, msg):
        pass  # reliability marker; presence noted via logs

    def battery_cb(self, msg, drone):
        pass  # reliability marker

    # ------------------------------------------------------------------
    # output
    # ------------------------------------------------------------------
    def _finalise_rows(self):
        rows = []
        for r in self.records:
            rows.append({k: r.get(k, "") for k in CSV_FIELDS})
        return rows

    def save(self):
        rows = self._finalise_rows()
        base = f"{self.mode}_{self.scenario}_{self.run_id}"
        csv_path = os.path.join(self.out_dir, base + ".csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)

        # ---- summary aggregates ----
        lats = [r["latency_sec"] for r in self.records
                if isinstance(r["latency_sec"], (int, float))]
        comps = [r["completion_time_sec"] for r in self.records
                 if isinstance(r["completion_time_sec"], (int, float))]
        detections = len(self.records)
        completed = sum(1 for r in self.records if r["completed"])

        def agg(xs):
            if not xs:
                return {"n": 0, "mean": None, "median": None,
                        "min": None, "max": None, "stdev": None}
            return {
                "n": len(xs),
                "mean": round(statistics.fmean(xs), 4),
                "median": round(statistics.median(xs), 4),
                "min": round(min(xs), 4), "max": round(max(xs), 4),
                "stdev": round(statistics.pstdev(xs), 4) if len(xs) > 1 else 0.0,
            }

        util_total = {"local": 0, "fog": 0, "cloud": 0}
        for d in self.util.values():
            for k in util_total:
                util_total[k] += d[k]

        summary = {
            "mode": self.mode, "scenario": self.scenario, "run_id": self.run_id,
            "fault": self.fault, "num_drones": self.num_drones,
            "num_victims": self.num_victims,
            "duration_sec": round(time.time() - self.t_start, 2),
            "detections": detections, "completed": completed,
            "detection_rate": (round(min(1.0, detections / self.num_victims), 4)
                               if self.num_victims > 0 else None),
            "success_rate": (round(min(1.0, completed / self.num_victims), 4)
                             if self.num_victims > 0 else None),
            "completion_ratio": (round(completed / detections, 4)
                                 if detections else None),
            "latency_sec": agg(lats),
            "completion_time_sec": agg(comps),
            "utilisation": {"per_drone": dict(self.util), "total": util_total},
        }
        json_path = os.path.join(self.out_dir, base + ".summary.json")
        with open(json_path, "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(
            f"[METRICS SAVED] {csv_path}  ({detections} detections, "
            f"{completed} completed)  + {json_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
