#!/usr/bin/env python3
"""
battery_logger.py — record a per-drone battery time series for Task 6.

The metrics_collector only stores start/end battery (energy delta). To draw
"battery consumption over time" we need the full trace, so this passive node
subscribes to every /{drone}/battery_status, parses the `battery=NN.NN` value
(same regex the collector uses) plus the `state=...` tag, and appends a row per
message to:

    <out_dir>/<mode>_<scenario>_<run_id>_battery.csv
    columns: t_rel_sec, mode, scenario, run_id, fault, drone, battery_pct, state

Run it in ITS OWN terminal alongside the collector, with the SAME mode/scenario/
run_id, so analyze_results.py can pair it with the run. Rows are flushed as they
arrive, so Ctrl-C (or a launch SIGTERM) never loses data.

Usage:
  python3 evaluation/battery_logger.py --ros-args \
      -p mode:=fog -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=none
"""

import csv
import os
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BATT_RE = re.compile(r'battery=([\d.]+)')
STATE_RE = re.compile(r'state=([^\s|]+)')


class BatteryLogger(Node):
    def __init__(self):
        super().__init__('battery_logger')

        self.declare_parameter('mode', 'fog')
        self.declare_parameter('scenario', 'medium')
        self.declare_parameter('run_id', 'run_01')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('fault', 'none')
        self.declare_parameter('out_dir', 'evaluation/results_real')

        self.mode = str(self.get_parameter('mode').value).lower()
        self.scenario = str(self.get_parameter('scenario').value)
        self.run_id = str(self.get_parameter('run_id').value)
        self.num_drones = int(self.get_parameter('num_drones').value)
        self.fault = str(self.get_parameter('fault').value)
        self.out_dir = str(self.get_parameter('out_dir').value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.t_start = time.time()
        self.rows = 0

        base = f'{self.mode}_{self.scenario}_{self.run_id}_battery.csv'
        self.path = os.path.join(self.out_dir, base)
        self._fh = open(self.path, 'w', newline='')
        self._w = csv.writer(self._fh)
        self._w.writerow(['t_rel_sec', 'mode', 'scenario', 'run_id', 'fault',
                          'drone', 'battery_pct', 'state'])
        self._fh.flush()

        for i in range(self.num_drones):
            did = f'drone{i}'
            self.create_subscription(
                String, f'/{did}/battery_status',
                lambda m, d=did: self.batt_cb(m, d), 10)

        self.get_logger().info(
            f'[BATTERY LOG] mode={self.mode} scenario={self.scenario} '
            f'run={self.run_id} drones={self.num_drones} -> {self.path}')

    def batt_cb(self, msg, drone):
        m = BATT_RE.search(msg.data)
        if not m:
            return
        try:
            pct = float(m.group(1))
        except ValueError:
            return
        sm = STATE_RE.search(msg.data)
        state = sm.group(1) if sm else ''
        t_rel = round(time.time() - self.t_start, 2)
        self._w.writerow([t_rel, self.mode, self.scenario, self.run_id,
                          self.fault, drone, round(pct, 2), state])
        self._fh.flush()   # survive SIGTERM / Ctrl-C
        self.rows += 1

    def close(self):
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        self.get_logger().info(
            f'[BATTERY LOG] saved {self.rows} samples -> {self.path}')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()