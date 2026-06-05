# Task 5 — Threat Decision Logic (Core Intelligence & Coordination)

**Pipeline completed by this task:**

```
Drone → Detection (Task 4) → Offloading → Fog → Decision Node → Drone Action
```

Task 5 adds the **decision engine** that interprets detections, decides what to do,
and coordinates the drones — plus the **drone-side reactions** and the **feedback
loop** that closes the cycle.

## Files in this deliverable

| File | Goes to | Role |
|---|---|---|
| `decision_node.py` | `src/fog_node/fog_node/decision_node.py` | The decision engine (5.1–5.16) |
| `drone_commander.py` | `src/drone_node/drone_node/drone_commander.py` | Drone Action executor + feedback (5.10, 5.13) — extends the existing commander |
| `detection_sim.py` | `src/fog_node/fog_node/detection_sim.py` | Scenario driver for deterministic testing (5.14) |

> The decision engine is a **fog-tier** node but runs as its **own ROS2 process**,
> matching the "Decision Node" box in the pipeline. It is kept separate from
> `fog_server` so real-time fog work (status decisions, archival) and coordination
> intelligence don't share a callback group.

## ⚠️ Branch merge note (read first)

Your work currently lives on two branches:

- **`task4-victim-detection`** — has `victim_detector.py`, `cloud_detector.py`,
  fog-side YOLO, and publishes detections on `/fog/victim_alerts` (and as
  `VICTIM_DETECTION` `Task`s). **No** `drone_commander` / `start_mission`.
- **current project** — has `drone_commander.py` and the fog `start_mission`
  area-division. **No** detection.

Task 5 sits between them and needs **both**. Build it in a workspace that contains:
the Task 4 detection nodes **and** the commander/`start_mission` fog. The decision
node itself touches nothing in Task 4 — it only consumes `/fog/victim_alerts`.

## Detection contract consumed (Task 4 → Task 5)

Reused as-is, no new message type:

- **Primary** `/fog/victim_alerts` (`std_msgs/String`, JSON):
  `{drone_id, num_persons, detections:[{bbox, confidence, label}], ...}`
- **Fallback** `VICTIM_DETECTION` `Task` on `/{drone_id}/task/fog` (drone-side detector).

Detections carry pixel bboxes + confidence but **no world location**. The decision
node derives location from the **reporting drone's live position**: it tracks each
drone's PX4 `VehicleLocalPosition` (NED, relative to spawn) and converts to world
ENU using the spawn points (`world_x = spawn_x + pos.y`, `world_y = spawn_y + pos.x`).
The drone is over the victim, so its position is the victim's location to within the
camera footprint — and it is exactly what nearest-drone selection needs.

## Subtask coverage

| Subtask | Where |
|---|---|
| 5.1 Define threat/event | `Event` class + ingest rules (victim = person ≥ `NOISE_CONFIDENCE`) |
| 5.2 Decision rules | `coordinate()`: dispatch if conf ≥ `MIN_CONFIDENCE`, else SCAN_AREA |
| 5.3 Decision node | `decision_node.py` |
| 5.4 Subscribe to detections | `/fog/victim_alerts` + `VICTIM_DETECTION` tasks |
| 5.5 Event aggregation | spatial clustering (`CLUSTER_RADIUS_M`) into `self.events` |
| 5.6 Prioritise | `Event.priority() = 0.7*conf + 0.3*norm_reports` |
| 5.7 Best drone | `_select_drone()`: `0.6*norm_dist + 0.4*(1-battery)` over available drones |
| 5.8 Generate commands | `GO_TO`, `HOVER`, `SCAN_AREA`, `RETURN_HOME` |
| 5.9 Send to specific drone | `/{drone_id}/mission_command` |
| 5.10 Drone reaction | `drone_commander.py` executes the four commands |
| 5.11 Multi-drone coordination | one drone per event, ranked by priority |
| 5.12 Conflict/redundancy | `assigned_event` lock per drone; command dedup |
| 5.13 Feedback loop | `/{drone_id}/mission_feedback` → events resolved on arrival |
| 5.14 Scenarios | `detection_sim.py` + failure handling in `coordinate()` |
| 5.15 Logging | `[DECISION …]` labels |
| 5.16 Metrics | `[DECISION STATS]`: response/completion time, utilisation |

## Decision rules (5.1 / 5.2)

- **Event** = a clustered cluster of person detections. `confidence` = max seen;
  `num_reports` = corroborating detections within `CLUSTER_RADIUS_M`.
- `confidence < NOISE_CONFIDENCE (0.20)` → ignored as noise.
- `NOISE ≤ confidence < MIN_CONFIDENCE (0.40)` → **SCAN_AREA** (re-scan to confirm).
- `confidence ≥ MIN_CONFIDENCE` → **GO_TO** with the nearest suitable drone.
- A drone marked **failing** (battery ≤ 20% or `drone_failing` flag) → forfeits its
  event (reassigned) and is sent **RETURN_HOME**.
- No actionable events → drones continue their existing search (no spam commands).

## Coordination strategy

Each 1 s planner cycle: handle failures → rank unassigned events by priority →
for each, pick the lowest-cost **available** drone and dispatch. A drone is locked to
at most one event; an event to at most one drone; identical commands are not resent.
On arrival feedback the event is **RESOLVED**, the drone freed, and held over the
victim (`HOVER`). Lower-priority events wait (and are logged as queued) when all
drones are busy.

## Topics

| Topic | Type | Dir | Purpose |
|---|---|---|---|
| `/fog/victim_alerts` | String | in | detections (Task 4) |
| `/{drone_id}/task/fog` | Task | in | fallback detections + `drone_failing` |
| `/px4_N/fmu/out/vehicle_local_position_v1` | VehicleLocalPosition | in | live world position |
| `/px4_N/fmu/out/vehicle_status_v1` | VehicleStatus | in | armed/availability |
| `/{drone_id}/mission_feedback` | String | in | drone state (5.13) |
| `/{drone_id}/mission_command` | String | out | commands (5.8/5.9) |
| `/fog/decision_log` | String | out | structured decisions for viz |

## Integration

Add entry points, then build.

`src/fog_node/setup.py` → `console_scripts`:
```python
'fog_server = fog_node.fog_server:main',
'decision_node = fog_node.decision_node:main',
'detection_sim = fog_node.detection_sim:main',
```

`src/drone_node/setup.py` → `console_scripts` (the commander has no entry point yet on either branch):
```python
'drone_commander = drone_node.drone_commander:main',
```

```bash
cd ~/ros2_ws
colcon build --packages-select fog_node drone_node
source install/setup.bash   # in every terminal, after sourcing /opt/ros/humble
```

## Run procedure

On top of the Task 3 launch (MicroXRCEAgent → PX4 instances → camera bridges →
`fog_server` → task publishers), add:

```bash
# Detection (Task 4) — fog-side, or per-drone victim_detector per instance
ros2 run fog_node fog_server --ros-args -p num_drones:=3 -p enable_detection:=true

# One commander per drone (Drone Action)
ros2 run drone_node drone_commander --ros-args -p instance:=0
ros2 run drone_node drone_commander --ros-args -p instance:=1
ros2 run drone_node drone_commander --ros-args -p instance:=2

# The decision engine
ros2 run fog_node decision_node --ros-args -p num_drones:=3
```

Trigger the search-area spread first (existing service), then let detections flow:
```bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger {}
```

### Deterministic scenario testing (5.14), no YOLO needed

```bash
ros2 run fog_node detection_sim --ros-args -p scenario:=single
# scenarios: single | multiple | none | low_conf | corroborate
```

Drone-failure case: add `-p simulate_low_battery:=true` to one commander — the
decision node will release its event, reassign it, and send that drone home.

## Example logs (5.15)

```
[DECISION DETECT] drone0 reports 1 person(s) conf=0.82 at world=(31.4, 40.2)
[DECISION EVENT]  E001 created at (31.4, 40.2) conf=0.82 priority=0.67
[DECISION ASSIGN] E001 -> drone1 (action=GO_TO, cost=0.21, priority=0.67)
[DECISION CMD]    drone1 <- GO_TO {'world_x': 31.4, 'world_y': 40.2, ...}
[DECISION RESOLVED] E001 reached by drone1 in 7.3s
[DECISION STATS]  events: active=0 resolved=1 created=1 | utilisation=33% (1/3) | avg_response=0.42s avg_completion=7.3s
```

## Metrics (5.16)

- **Response time** — first detection → first command (per event).
- **Completion time** — assignment → arrival (from feedback).
- **Utilisation** — fraction of drones currently committed to an event.

All three are summarised every 5 s in `[DECISION STATS]`.

## Points to defend in the viva

1. **Reused Task 4's `/fog/victim_alerts` contract** instead of a new message — the
   decision node is detector-agnostic and Task 4 needed no changes.
2. **Location derived from the reporting drone**, not the detection message —
   detections only have pixel bboxes, and this is also the data selection needs.
3. **Separate node, fog tier** — coordination is decoupled from real-time fog work.
4. **Planner on a timer, not per-detection** — lets priority compare *across*
   accumulated events rather than reacting greedily to whichever arrived last.
5. **Explicit assignment locks** give conflict/redundancy avoidance for free (5.12).
6. **Feedback-driven resolution** — the loop is closed by the drone, not assumed.
