#!/usr/bin/env python3
"""
metrics_collector.py — Task 6 instrumentation (local / fog / cloud).

Passive observer node. Listens to the real ROS 2 topics the system already
publishes and records, per detection, the Task 6 metrics, then auto-saves a CSV
+ summary.json when the mission ends.

WHAT CHANGED vs the previous version (completion-time wiring)
------------------------------------------------------------
Completion time used to be keyed by the *detecting* drone. But decision_node
dispatches the *nearest* drone (`_select_drone`), which is often — not always —
the detector. When detector != rescuer, the arrival was reported by a drone with
no open detection row, so completion silently never closed (this is one reason
completion came back empty even in coordinated runs).

Now completion is linked by EVENT, using the `reporting_drones` and authoritative
`completion_time` that decision_node emits on /fog/decision_log:

    detection (drone A)            -> open row, keyed by A
    ASSIGNED  (event E, [A], ->B)  -> link A's oldest open row to event E
    RESOLVED  (event E, completion_time) -> close that row with the real time

A drone-keyed feedback path (ARRIVED/HOLDING) is kept as a FALLBACK so runs
without a healthy decision_log still record completion when detector == rescuer.

The summary now reports BOTH:
  * raw per-detection latency (the latency dataset, one row per alert), and
  * event-level lifecycle from decision_log (events_created/assigned/resolved,
    completion_ratio, authoritative completion_time / response_time aggregates).

Parameters
----------
mode local|fog|cloud · scenario · run_id · num_drones · fault
auto_finish (default true) · grace_sec (default 90) · out_dir
"""

import csv
import json
import os
import re
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
    "event_id", "latency_sec", "comm_delay_sec", "completion_time_sec",
    "response_time_sec", "detected", "completed", "confidence", "num_persons",
    "detail",
]

ARRIVAL_STATES = ("ARRIVED", "HOLDING")


# ----------------------------------------------------------------------
# Pure helpers (no ROS) — unit-testable in isolation
# ----------------------------------------------------------------------
def pick_detection_row(records, open_by_drone, reporting_drones):
    """Oldest still-open, not-yet-linked detection row reported by any of
    `reporting_drones`. Returns the record index, or None."""
    best = None
    for drone in reporting_drones or ():
        for idx in open_by_drone.get(drone, ()):
            rec = records[idx]
            if rec.get("event_id"):
                continue
            if best is None or idx < best:
                best = idx
    return best


def remove_from_open(open_by_drone, drone, idx):
    q = open_by_drone.get(drone)
    if not q:
        return
    for k, j in enumerate(q):
        if j == idx:
            del q[k]
            return


# ----------------------------------------------------------------------
class MetricsCollector(Node):
    def __init__(self):
        super().__init__("metrics_collector")

        self.declare_parameter("mode", "fog")
        self.declare_parameter("scenario", "medium")
        self.declare_parameter("run_id", "run_01")
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("fault", "none")
        self.declare_parameter("auto_finish", True)
        self.declare_parameter("grace_sec", 90.0)
        self.declare_parameter("out_dir", "evaluation/results_real")

        self.mode = str(self.get_parameter("mode").value).lower()
        self.scenario = str(self.get_parameter("scenario").value)
        self.run_id = str(self.get_parameter("run_id").value)
        self.num_drones = int(self.get_parameter("num_drones").value)
        self.fault = str(self.get_parameter("fault").value)
        self.auto_finish = bool(self.get_parameter("auto_finish").value)
        self.grace_sec = float(self.get_parameter("grace_sec").value)
        self.out_dir = os.path.abspath(str(self.get_parameter("out_dir").value))
        os.makedirs(self.out_dir, exist_ok=True)

        self._saved = False
        self._end_seen = False
        self._mismatch_warned = set()

        self.records = []                       # one dict per detection alert
        self.open_by_drone = defaultdict(deque)  # drone -> indices not linked/closed
        self.row_by_event = {}                  # event_id -> record index
        self.util = defaultdict(lambda: {"local": 0, "fog": 0, "cloud": 0})
        self.coverage = None
        self.batt_first = {}
        self.batt_last = {}
        self.seq = 0
        self.t_start = time.time()

        # Event-level lifecycle from decision_log (authoritative for completion)
        self.events_created = set()
        self.events_assigned = set()
        self.events_resolved = set()
        self.event_completion_times = []   # authoritative detection->arrival (s)
        self.event_response_times = []     # authoritative detection->command (s)

        # ---- Fog / decision path ----
        self.create_subscription(String, "/fog/victim_alerts", self.fog_alert_cb, 10)
        self.create_subscription(String, "/fog/decision_log", self.decision_cb, 10)
        self.create_subscription(String, "/decision/status", self.status_cb, 10)
        self.create_subscription(String, "/fog/coverage", self.coverage_cb, 10)
        self.create_subscription(
            String, "/fog/cloud/mission_log", self.mission_end_cb, 10)

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
            f"drones={self.num_drones} fault={self.fault} "
            f"auto_finish={self.auto_finish} grace={self.grace_sec:.0f}s")
        self.get_logger().warn(
            f"[METRICS] WILL SAVE TO: "
            f"{os.path.join(self.out_dir, f'{self.mode}_{self.scenario}_{self.run_id}')}"
            f".csv / .summary.json")
        self._hb = self.create_timer(15.0, self._heartbeat)
        if not _HAVE_TASK:
            self.get_logger().warn(
                "[METRICS] task_msgs not importable -> utilisation + local-mode "
                "latency disabled. Source the workspace before launching.")

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
            "event_id": "",
            "t_detect": t_detect, "t_decision": None, "t_complete": None,
            "latency_sec": "" if latency is None else round(latency, 4),
            "comm_delay_sec": "", "completion_time_sec": "",
            "response_time_sec": "",
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
        """Oldest open (not linked) record for a drone whose `field` is unset."""
        for idx in self.open_by_drone.get(drone, ()):
            rec = self.records[idx]
            if rec.get("event_id"):
                continue
            if rec[field] is None:
                return idx
        return None

    # ------------------------------------------------------------------
    # detection handlers (mode-specific)
    # ------------------------------------------------------------------
    def _heartbeat(self):
        self.get_logger().info(
            f"[METRICS HEARTBEAT] mode={self.mode} recorded_detections={len(self.records)} "
            f"events c/a/r={len(self.events_created)}/{len(self.events_assigned)}"
            f"/{len(self.events_resolved)} batt_drones={len(self.batt_last)} "
            f"coverage={'yes' if self.coverage else 'no'}")

    def _warn_mismatch(self, src_mode, topic):
        if (src_mode, topic) in self._mismatch_warned:
            return
        self._mismatch_warned.add((src_mode, topic))
        self.get_logger().error(
            f"[METRICS] receiving {src_mode.upper()} detections on {topic} but this "
            f"collector is mode={self.mode} -> NOT recording them. Restart it with "
            f"-p mode:={src_mode}.")

    def fog_alert_cb(self, msg):
        if self.mode != "fog":
            self._warn_mismatch("fog", "/fog/victim_alerts")
            return
        try:
            a = json.loads(msg.data)
        except Exception:
            return
        drone = a.get("drone_id", "drone?")
        conf = round(self._max_conf(a.get("detections", [])), 3)
        n = int(a.get("num_persons", 1))
        infer_ms = float(a.get("inference_time_ms", 0.0))
        lat = (infer_ms / 1000.0) if infer_ms > 0 else None
        self._new_record(drone, time.time(), latency=lat, confidence=conf,
                         num_persons=n, detail="fog_alert")
        self.get_logger().info(f"[DETECT/fog] {drone} conf={conf} infer={infer_ms:.0f}ms")

    def cloud_cb(self, msg, drone):
        if self.mode != "cloud":
            self._warn_mismatch("cloud", f"/{drone}/cloud/detection")
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
        rec["comm_delay_sec"] = round(max(0.0, (total_ms - infer_ms) / 1000.0), 4)
        self.get_logger().info(
            f"[DETECT/cloud] {drone} latency={rec['latency_sec']}s "
            f"(wan={rec['comm_delay_sec']}s)")

    def taskfog_cb(self, msg, drone):
        self.util[drone]["fog"] += 1
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
    # decision lifecycle (link + close by EVENT)
    # ------------------------------------------------------------------
    def decision_cb(self, msg):
        try:
            rec = json.loads(msg.data)
        except Exception:
            return
        kind = str(rec.get("kind", "")).upper()
        ev_id = rec.get("event_id")

        if kind == "EVENT_CREATED" and ev_id:
            self.events_created.add(ev_id)
            return

        if kind == "ASSIGNED" and ev_id:
            self.events_assigned.add(ev_id)
            reporting = rec.get("reporting_drones") or []
            # Fall back to the assigned drone if the reporter list is missing.
            if not reporting and rec.get("drone"):
                reporting = [rec["drone"]]
            idx = pick_detection_row(self.records, self.open_by_drone, reporting)
            if idx is not None:
                r = self.records[idx]
                r["event_id"] = ev_id
                r["t_decision"] = time.time()
                # Coordination latency (only if no inference latency was captured)
                if r["latency_sec"] in ("", None):
                    lat = round(r["t_decision"] - r["t_detect"], 4)
                    r["latency_sec"] = lat
                    r["comm_delay_sec"] = lat
                remove_from_open(self.open_by_drone, r["drone"], idx)
                self.row_by_event[ev_id] = idx
                self.get_logger().info(
                    f"[ASSIGN/{self.mode}] {ev_id} <- detection by {r['drone']}")
            return

        if kind == "RESOLVED" and ev_id:
            self.events_resolved.add(ev_id)
            ct = rec.get("completion_time")
            rt = rec.get("response_time")
            if ct is not None:
                self.event_completion_times.append(float(ct))
            if rt is not None:
                self.event_response_times.append(float(rt))
            idx = self.row_by_event.pop(ev_id, None)
            if idx is not None:
                r = self.records[idx]
                # Authoritative time from decision_node; else our own clock.
                comp = float(ct) if ct is not None else (time.time() - r["t_detect"])
                r["completion_time_sec"] = round(comp, 4)
                if rt is not None:
                    r["response_time_sec"] = round(float(rt), 4)
                r["t_complete"] = time.time()
                r["completed"] = 1
                self.get_logger().info(
                    f"[COMPLETE/{self.mode}] {ev_id} {r['drone']} "
                    f"completion={r['completion_time_sec']}s")
            return

    def feedback_cb(self, msg, drone):
        """Fallback: close an UNLINKED open row when its own drone arrives.
        (Primary close path is decision_log RESOLVED, by event.)"""
        try:
            fb = json.loads(msg.data)
        except Exception:
            return
        if str(fb.get("state", "")).upper() not in ARRIVAL_STATES:
            return
        idx = self._oldest_open(drone, "t_complete")
        if idx is None:
            return
        r = self.records[idx]
        r["t_complete"] = time.time()
        r["completion_time_sec"] = round(r["t_complete"] - r["t_detect"], 4)
        r["completed"] = 1
        remove_from_open(self.open_by_drone, drone, idx)
        self.get_logger().info(
            f"[COMPLETE/fallback] {drone} completion={r['completion_time_sec']}s")

    def status_cb(self, msg):
        pass  # reliability marker; presence noted via logs

    def coverage_cb(self, msg):
        try:
            self.coverage = json.loads(msg.data)
        except Exception:
            pass

    # ------------------------------------------------------------------
    def mission_end_cb(self, msg):
        if self._end_seen:
            return
        self._end_seen = True
        self.get_logger().info(
            "[METRICS] end-of-mission detected (fog flush). "
            f"Collecting trailing events for {self.grace_sec:.0f}s, then saving.")
        if self.auto_finish:
            self._grace_timer = self.create_timer(
                self.grace_sec, self._auto_finish_once)

    def _auto_finish_once(self):
        if self._saved:
            return
        try:
            self._grace_timer.cancel()
        except Exception:
            pass
        self.get_logger().info(
            "[METRICS] grace window elapsed -> saving and shutting down.")
        self.save()
        if rclpy.ok():
            rclpy.shutdown()

    def battery_cb(self, msg, drone):
        m = re.search(r'battery=([\d.]+)', msg.data)
        if not m:
            return
        try:
            pct = float(m.group(1))
        except ValueError:
            return
        if drone not in self.batt_first:
            self.batt_first[drone] = pct
        self.batt_last[drone] = pct

    # ------------------------------------------------------------------
    def _finalise_rows(self):
        return [{k: r.get(k, "") for k in CSV_FIELDS} for r in self.records]

    def save(self):
        if self._saved:
            return
        self._saved = True
        rows = self._finalise_rows()
        base = f"{self.mode}_{self.scenario}_{self.run_id}"
        csv_path = os.path.join(self.out_dir, base + ".csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)

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

        energy = {}
        total_consumed = 0.0
        for d in self.batt_first:
            start = self.batt_first[d]
            end = self.batt_last.get(d, start)
            consumed = max(0.0, start - end)
            energy[d] = {"start_pct": round(start, 2), "end_pct": round(end, 2),
                         "consumed_pct": round(consumed, 2)}
            total_consumed += consumed
        n_batt = len(energy)

        cov_overall = (self.coverage.get("overall_pct")
                       if isinstance(self.coverage, dict) else None)

        # Event-level lifecycle (authoritative for completion)
        n_assigned = len(self.events_assigned)
        n_resolved = len(self.events_resolved)
        event_completion_ratio = (round(n_resolved / n_assigned, 4)
                                  if n_assigned else None)

        summary = {
            "mode": self.mode, "scenario": self.scenario, "run_id": self.run_id,
            "fault": self.fault, "num_drones": self.num_drones,
            "duration_sec": round(time.time() - self.t_start, 2),
            "detection_events": detections,
            "completed": completed,
            # completion_ratio is now EVENT-based (resolved / assigned), so many
            # alerts of one victim no longer dilute it. Falls back to row-based
            # if no events were assigned (e.g. detector==rescuer feedback path).
            "completion_ratio": (event_completion_ratio
                                 if event_completion_ratio is not None
                                 else (round(completed / detections, 4)
                                       if detections else None)),
            "events": {
                "created": len(self.events_created),
                "assigned": n_assigned,
                "resolved": n_resolved,
            },
            "latency_sec": agg(lats),
            # Prefer authoritative detection->arrival times from decision_node;
            # fall back to per-row completion (feedback path) if none.
            "completion_time_sec": agg(self.event_completion_times or comps),
            "response_time_sec": agg(self.event_response_times),
            "utilisation": {"per_drone": dict(self.util), "total": util_total},
            "coverage": self.coverage,
            "coverage_overall_pct": cov_overall,
            "energy": {"per_drone": energy,
                       "total_consumed_pct": round(total_consumed, 2),
                       "mean_consumed_pct": (round(total_consumed / n_batt, 2)
                                             if n_batt else None)},
            "energy_total_pct": round(total_consumed, 2) if energy else None,
        }
        json_path = os.path.join(self.out_dir, base + ".summary.json")
        with open(json_path, "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(
            f"[METRICS SAVED] {csv_path}  ({detections} detection events, "
            f"{n_resolved} resolved / {n_assigned} assigned)  + {json_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()

    # Save even if killed by SIGTERM (e.g. a launch shutdown), which would not
    # otherwise raise KeyboardInterrupt or run the finally block.
    import signal

    def _on_term(signum, frame):
        try:
            node.save()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    try:
        signal.signal(signal.SIGTERM, _on_term)
    except Exception:
        pass

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