# Task 6 — Experiment Protocol & Report Scaffold

This is the runbook that turns the instrumentation into Task-6 deliverables.
The harness (`metrics_collector.py` + `analyze_results.py`) is complete; the one
thing only your VM can produce is the **data**. Follow the matrix below, then run
the analyzer — it emits every required table and all five graphs.

> Honesty note: do **not** hand-author result rows. Every number in the report
> must come from a real run captured by `metrics_collector.py`. The single
> pre-existing `fog_medium_run_01.csv` was a hand-made placeholder (its
> `detection_msg`/`decision_msg` are plain text, but the system publishes JSON) —
> delete it before collecting real data.

---

## 6.1 Metrics (what the harness records)

| Metric | Where it comes from |
|---|---|
| Latency | `latency_sec` — detection→decision (fog), or detection→result (local/cloud) |
| Task completion time | `completion_time_sec` — detection→drone arrival (`mission_feedback` ARRIVED) |
| Communication delay | `comm_delay_sec` — alert→command (fog) or WAN portion `total_ms−inference_ms` (cloud) |
| Detection success rate | `summary.json success_rate` = resolved victims / `num_victims` (ground truth) |
| System reliability | `completed` ratio under each `fault` (none / fog_down / drone_down / comm_delay) |
| Resource utilisation | `summary.json utilisation` — Tasks routed local / fog / cloud per drone |

## 6.2 Baselines (how each mode is realised in YOUR system)

| Mode | Detection node | Decision | What latency measures |
|---|---|---|---|
| **Local** | `victim_detector` (YOLO on drone) | on-drone (inline) | on-board inference time (`inference_time_ms`) |
| **Cloud** | `cloud_detector` (YOLO + 1–3 s WAN sleep) | after WAN round-trip | `total_ms` (WAN + inference) |
| **Fog** | `victim_detector`→`/fog/victim_alerts` + `decision_node` | fog `decision_node` | alert→`ASSIGNED` command |

## 6.3 Scenarios

| Scenario | Knob | Suggested setting |
|---|---|---|
| Small area | `decision_node`/`fog_server` area params | X[-20,20] Y[-15,15], `num_victims:=2` |
| Medium area | area params | X[-80,100] Y[-40,75] (default), `num_victims:=5` |
| Large area | area params | X[-150,180] Y[-90,120], `num_victims:=8` |
| Congestion | `fault:=comm_delay` | inject delay (see §Reliability) |
| Drone failure | `fault:=drone_down` | launch N−1 drones, or one with `simulate_low_battery:=true` |

---

## Run matrix

Minimum per Task 6: **each (mode × scenario) ≥ 10 repetitions** (run_01…run_10).
Plus a scalability sweep and a reliability set. That's the data; the analyzer does
the rest.

```
modes      = local, fog, cloud
scenarios  = small, medium, large
reps       = run_01 .. run_10
scalability= fog @ num_drones ∈ {3,5,8,10}, medium
reliability= fog @ fault ∈ {none, fog_down, drone_down, comm_delay}, medium
```

---

## How to run ONE experiment

Bring up the base flight stack exactly as in `RUN_GUIDE_jazzy_vm.md`
(agent → PX4 ×N → camera bridges → commanders), then add the mode-specific
detection node, the fog `decision_node`, and the collector.

**Common (every run), in their own terminals — each sources the workspace:**
```bash
# decision/coordination engine (fog brain; needed for fog completion timing)
ros2 run fog_node decision_node --ros-args -p num_drones:=3

# metrics collector (CHANGE mode/scenario/run_id/num_victims/fault per run)
python3 evaluation/metrics_collector.py --ros-args \
  -p mode:=fog -p scenario:=medium -p run_id:=run_01 \
  -p num_drones:=3 -p num_victims:=5 -p fault:=none
```

**Detection node — pick ONE per the mode under test:**
```bash
# LOCAL mode (per drone instance 0..N-1)
ros2 run drone_node victim_detector --ros-args -p instance:=0

# CLOUD mode (per drone instance) — adds the simulated WAN delay
ros2 run drone_node cloud_detector  --ros-args -p instance:=0

# FOG mode  = victim_detector feeding the fog + decision_node (already running)
ros2 run drone_node victim_detector --ros-args -p instance:=0
```

> Never run two detection paths at once (e.g. `victim_detector` **and**
> `cloud_detector`) — it double-counts. One mode per run.

**Then:** trigger the mission (`ros2 service call /fog/start_mission ...`), let it
run until victims are reached, and **Ctrl-C the collector** — it writes
`evaluation/results_real/<mode>_<scenario>_<run_id>.csv` + `.summary.json`.

### Looping the reps (collector side)
The mission still starts manually, but you can script the collector lifetime:
```bash
for r in $(seq -w 1 10); do
  echo ">> run_$r — start the mission now, ~60s window"
  timeout 60 python3 evaluation/metrics_collector.py --ros-args \
    -p mode:=fog -p scenario:=medium -p run_id:=run_$r \
    -p num_drones:=3 -p num_victims:=5 -p fault:=none
done
```

### Scalability sweep (Task 6.8)
Repeat the fog run at `num_drones ∈ {3,5,8,10}` (match PX4 instances launched),
`run_id:=run_d<N>`. On the Apple-Silicon VM, 8–10 drones will likely fall below
real-time — record what you can and note the RTF as a limitation (6.15).

### Reliability set (Task 6.9)
- `none` — baseline.
- `fog_down` — kill the `decision_node` terminal ~halfway; tag `fault:=fog_down`.
- `drone_down` — launch one fewer drone (or `drone_commander ... -p simulate_low_battery:=true`); tag `fault:=drone_down`.
- `comm_delay` — inject latency, tag `fault:=comm_delay`. Quickest inside the VM:
  `sudo tc qdisc add dev lo root netem delay 200ms` (remove with
  `sudo tc qdisc del dev lo root netem`). Cloud mode already has built-in delay.

---

## Analyse (Task 6.11–6.14)

```bash
python3 evaluation/analyze_results.py
```
Produces in `evaluation/`:
- `tables/table_mode_comparison.{md,csv}` (latency / completion / success per mode)
- `tables/table_scalability.{md,csv}`, `tables/table_reliability.{md,csv}`
- `tables/table_objectives.md` (auto-filled where numbers exist)
- `plots/g1…g5_*.png` (latency, completion, success, scalability, reliability)
- console: % latency reduction of fog vs local and fog vs cloud

---

## Report scaffold (fill from analyzer output — no invented numbers)

- **6.13 Interpret:** state the measured fog-vs-cloud and fog-vs-local latency
  deltas (from the console line), and whether completion/ success favour fog.
- **6.14 Objectives:** paste `table_objectives.md`, complete the two `<fill>` rows.
- **6.15 Limitations:** Apple-Silicon-VM software rendering caps RTF and the drone
  ceiling; cloud delay is simulated (`time.sleep`), not a real WAN; success rate
  uses resolved/ground-truth victims, so victim placement must be controlled.
- **6.16 Future work:** GPU passthrough / bare-metal for higher swarm sizes; real
  network emulation; learned task-routing; multi-victim clustering for success rate.

## Task 6 completion checklist

- [ ] Delete placeholder `fog_medium_run_01.csv`
- [ ] local/fog/cloud × small/medium/large × 10 reps collected
- [ ] scalability sweep (3/5/8/10 fog) collected
- [ ] reliability set (4 faults) collected
- [ ] `analyze_results.py` run → 5 graphs + 4 tables generated
- [ ] 6.13–6.16 written from the generated numbers
