# Task 3 — Task Offloading Mechanism (Complete Reference)

**Project:** Fog-Enabled UAV Swarm System for Low-Latency Disaster Response
**Course:** Birzeit University ENCS5300 Graduation Project, 2025–2026
**Team:** Lina Abureesh (1211985), Mai Beitnoba (1210260), Doaa Hatu (1211088)
**Supervisor:** Dr. Ibrahim Nemer

**Scope of this document:** Everything built in Task 3 — sub-tasks 3.1 through 3.13, including the
simulated cloud archival of Task 3.8. This is the single source of truth for Task 3 and supersedes
the per-sub-task READMEs (`TASK_3.3`, `TASK_3.4`, `TASK_3.8`, and the earlier 3.1–3.6 master). If you
just cloned the repo, read this end-to-end before running anything.

---

## Table of Contents

1. [What Task 3 is about](#1-what-task-3-is-about)
2. [Design decisions (Tasks 3.1, 3.2)](#2-design-decisions-tasks-31-32)
3. [System architecture](#3-system-architecture)
4. [What we built](#4-what-we-built)
   - 4.1 Custom message package `task_msgs`
   - 4.2 Camera bridge (`camera_bridge_simple`)
   - 4.3 Drone task publisher (`drone_task_publisher`)
   - 4.4 Fog server (`fog_server`)
   - 4.5 Cloud server (`cloud_server`)
   - 4.6 Helper module `drone_naming`
5. [How offloading works in this system](#5-how-offloading-works-in-this-system)
6. [Cloud archival in depth (Task 3.8)](#6-cloud-archival-in-depth-task-38)
7. [How each Task 3 sub-task is satisfied](#7-how-each-task-3-sub-task-is-satisfied)
8. [Repository layout](#8-repository-layout)
9. [Prerequisites](#9-prerequisites)
10. [Build instructions](#10-build-instructions)
11. [Run instructions — 1 drone](#11-run-instructions--1-drone)
12. [Run instructions — 3 drones (validated)](#12-run-instructions--3-drones-validated)
13. [Run instructions — N drones](#13-run-instructions--n-drones)
14. [Run instructions — cloud archival](#14-run-instructions--cloud-archival)
15. [Inspecting the cloud archive](#15-inspecting-the-cloud-archive)
16. [Verification checklists](#16-verification-checklists)
17. [Tuning notes and lessons learned](#17-tuning-notes-and-lessons-learned)
18. [Troubleshooting](#18-troubleshooting)
19. [Design decisions worth defending in the viva](#19-design-decisions-worth-defending-in-the-viva)
20. [Quick reference — file map](#20-quick-reference--file-map)

---

## 1. What Task 3 is about

The objective of Task 3 is to design and implement a **task-offloading mechanism** that decides where
each unit of work runs: on the drone (edge), on the fog node, or on the cloud (simulated). Without
offloading, the system has only two unsatisfactory options — process everything onboard and exhaust the
drone, or push everything to the cloud and accept latencies that make the drone too slow to react. Fog
computing resolves this by absorbing the bulk of mission-critical work close to the drones.

By the end of Task 3:

- The drone is an active **task producer** with onboard intelligence (frame filtering, position
  tagging, decision-making).
- Tasks flow on a **typed control plane** (custom ROS2 message), and sensor data flows on a parallel
  **data plane** (standard ROS2 image messages).
- The fog receives, parses, and acknowledges every task with measured end-to-end latency, and buffers
  every event for later archival.
- The cloud exists as a real, separate node that archives mission logs after the mission ends, applying
  a simulated WAN delay.
- The architecture scales from 1 drone to N drones with a single parameter, no code changes.

---

## 2. Design decisions (Tasks 3.1, 3.2)

### Three-tier architecture (Task 3.1)

Each tier has a **distinct, non-overlapping responsibility**:

- **Drone (edge):** lightweight, latency-critical operations that must run regardless of network
  conditions. Examples: telemetry generation, battery/health checks, camera-frame pre-filtering,
  attaching the drone's position to outgoing data.
- **Fog (swarm coordinator):** medium-complexity, mission-critical workloads. Examples: victim detection
  from camera frames, swarm-wide status aggregation, mission-decision logic. This is the **brain of the
  swarm**.
- **Cloud (archival only):** strictly **non-real-time** data storage. The cloud receives logs,
  aggregated maps, performance metrics, and confirmed detection records from the fog. The cloud **never
  runs live processing** and **never returns commands** during a mission. This matches Section 4.9 of the
  project design.

### Communication pattern

- **Drone → Fog:** sensor data, telemetry, task offload requests.
- **Fog → Drone:** decisions, area assignments, commands (e.g., re-tasking when a drone is failing).
- **Fog → Cloud:** logs, archives, performance metrics (one-way, non-real-time).
- **Drone ↔ Cloud:** never. The drone has no business talking to the cloud directly.

### Task catalog (Task 3.2)

| ID | Task Name | Producer | Executed At | Status in Task 3 |
|---|---|---|---|---|
| T1 | Telemetry Status Generation | Drone | Drone (local), forwarded to Fog | ✅ Implemented |
| T2 | Battery & Health Check | Drone | Drone (local) | Catalogued; runs as part of T1 |
| T3 | Camera Frame Pre-filtering | Drone | Drone (local) | ✅ Implemented (continuous) |
| T4 | Victim Detection | Drone (request) | Fog | ✅ Request side; model pending Task 4 |
| T5 | Swarm Status Aggregation | Fog | Fog | Catalogued; full impl in Task 3.7 |
| T6 | Threat / Mission Decision | Fog | Fog | Catalogued; full impl in Task 5 |
| T7 | Mission Log Upload | Fog | Cloud (simulated) | ✅ Implemented (Task 3.8) |
| T8 | Detection Record Archival | Fog | Cloud (simulated) | Catalogued; needs Task 4 |
| T9 | Performance Metrics Report | Fog | Cloud (simulated) | Catalogued; Task 3.13 |

### Where offloading decisions live (Tasks 3.4, 3.5)

Each tier knows what it can do and what it must offload. The drone has a `decide_target()` function that
maps task types to tiers, but **the heart of offloading is that each tier produces tasks only for work it
cannot do itself**. The drone never produces cloud tasks because the drone never talks to the cloud. The
fog never produces drone *tasks* because the fog talks down via commands, not tasks. The architecture is
implicit in the responsibilities of each tier; the explicit function is there for the assignment
requirement and to leave room for future task types.

### Local processing (Task 3.6)

Local processing on the drone is implemented as **continuous background operations**, not as discrete
tasks pulled off a queue. Filtering frames, generating status, attaching position — these are not "tasks"
that get scheduled; they are simply what being a drone means. The `/{drone_id}/task/local` topic exists in
the architecture (for future expansion) but is currently unused. This is a deliberate architectural
choice: a drone sending tasks to itself would be indirection without value.

---

## 3. System architecture

### Mission-time data and control flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              GAZEBO (Harmonic)                              │
│                                                                             │
│   PX4 SITL drone0, drone1, drone2 ...                                       │
│   ├─ VehicleStatus            ──► ROS2 (uxrce-dds bridge)                   │
│   ├─ VehicleLocalPosition     ──► ROS2 (uxrce-dds bridge)                   │
│   └─ IMX214 RGB camera        ──► gz-transport ──► (camera bridge)          │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
            ┌──────────────────────────┼──────────────────────────┐
            ▼                          ▼                          ▼
        Drone 0                     Drone 1                    Drone N
   ┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
   │ camera_bridge   │         │ camera_bridge   │         │ camera_bridge   │
   │ (2 Hz throttle) │         │                 │         │                 │
   └────────┬────────┘         └─────────────────┘         └─────────────────┘
            │
            ▼ /drone0/camera/image (sensor_msgs/Image, 2 Hz)
   ┌──────────────────────────────────────────────────────────────────────┐
   │ drone_task_publisher (one per drone)                                 │
   │                                                                      │
   │   Subscribes to:                                                     │
   │     • /fmu/out/vehicle_status_v1            (or /px4_N/...)          │
   │     • /fmu/out/vehicle_local_position_v1    (or /px4_N/...)          │
   │     • /drone0/camera/image                                           │
   │                                                                      │
   │   Per camera frame:                                                  │
   │     1. Filter (brightness / inter-frame diff / blur)                 │
   │     2. Attach position from cached PX4 local position                │
   │     3. Run decide_target() → choose tier                             │
   │     4. Publish to /drone0/task/<tier>                                │
   │                                                                      │
   │   Every 5 s: STATUS_REPORT with position + drone_failing flag        │
   │                                                                      │
   │   Three task topics per drone:                                       │
   │     /drone0/task/local   (no consumer — reserved for future)         │
   │     /drone0/task/fog     (consumed by fog_server)                    │
   │     /drone0/task/cloud   (no consumer — fog→cloud goes via service)  │
   └──────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼ Task messages (fog tier)
   ┌──────────────────────────────────────────────────────────────────────┐
   │ fog_server (one node, subscribes to all N drones)                    │
   │                                                                      │
   │   For each drone in [0, num_drones):                                 │
   │     • Subscribe to PX4 status, fog-tier task topic, camera topic     │
   │     • Publish decisions on /fog/{drone_id}/decision                  │
   │                                                                      │
   │   Logs per-drone task arrivals with measured end-to-end latency.     │
   │   Highlights priority=3 (dying-drone) tasks as CRITICAL warnings.    │
   │   Buffers every event in memory for end-of-mission archival.         │
   │   Stats line every 5 s.                                              │
   └──────────────────────────────────────────────────────────────────────┘
```

**Key insight: topic = routing.** Instead of stuffing a `target_layer` field into the `Task` message, we
publish to three different topics based on the decision. Each tier subscribes only to its own topic. This
makes Task 3.9 (Task Routing) practically free and makes the fog/cloud consumers a clean drop-in.

The pipeline is fully parallel: status, tasks, and camera frames each travel on their own topic and are
handled by independent callbacks on the fog side. A burst of detection requests doesn't slow down camera
frames, and vice versa.

### Cloud archival flow (Task 3.8)

```
                  DURING MISSION
                  ──────────────

  Drones ─tasks─► Fog ─decisions─► Drones
                   │
                   ▼
            Event buffer (in memory, bounded to 10000)
                   │
                   ▼
        /fog/cloud/mission_log
            ▲ (silent — zero traffic)
            │
           Cloud   (idle, ready to receive)


                  AT END OF MISSION
                  ─────────────────

  Operator ──ros2 service call /fog/end_mission──► Fog
                                                    │  chunks buffer into
                                                    │  batches of 1000 events,
                                                    ▼  publishes them all
                              /fog/cloud/mission_log
                                                    │
                                                    ▼
                                                Cloud — per batch:
                                                    1. Pick random delay [0.5, 5.0] s
                                                    2. Schedule one-shot timer
                                                    3. When timer fires, write to disk
                                                    ▼
                              /tmp/cloud_archive_<startup_ts>/
                                  batch_000000_recvNNN.json ...
```

---

## 4. What we built

### 4.1 Custom message package `task_msgs`

A dedicated ROS2 `ament_cmake` package containing a single message:

```
# Task.msg
string task_id              # e.g., "drone0-detect-0042"
string task_type            # STATUS_REPORT, VICTIM_DETECTION_REQUEST, ...
string drone_id             # which drone produced the task
builtin_interfaces/Time timestamp
uint8 priority              # 0 (default) ... 3 (critical)
string payload              # JSON-encoded task-specific data
```

| Field | Purpose |
|---|---|
| `task_id` | Unique instance identifier, e.g. `drone0-detect-0042`. Used in logs to track a specific task. |
| `task_type` | Task category, used by the offloading decision module to route tasks. |
| `drone_id` | Which drone produced the task. The fog uses this to send results back to the right drone. |
| `timestamp` | When the task was created at the drone. The fog computes `now - timestamp` to measure end-to-end latency. |
| `priority` | 0 (default) to 3 (critical). |
| `payload` | Free-form JSON string. Schema depends on `task_type`. Fixed metadata envelope + variable payload. |

**Why custom-typed, not plain `String`:** a typed message gives compile-time field validation, integrates
with `ros2 topic info` / `ros2 interface show`, and produces self-documenting output in `ros2 topic echo`.
The schema lives in an `ament_cmake` package because ROS2 message generation runs on CMake — mixing message
definitions into a Python node package silently fails.

**Why JSON payload inside a string field:** the metadata envelope (task_id, type, drone_id, timestamp,
priority) is stable forever. Different task types carry different payloads — `STATUS_REPORT` carries PX4
state, `VICTIM_DETECTION_REQUEST` carries a frame reference. JSON-in-string keeps the schema stable as new
task types are added (`BATTERY_CHECK`, `MISSION_LOG_UPLOAD`, etc.) without ever modifying `Task.msg`.

**Files involved:** `task_msgs/msg/Task.msg` (schema), `task_msgs/CMakeLists.txt` (invokes
`rosidl_generate_interfaces`), `task_msgs/package.xml` (declares `rosidl_default_generators` and
`builtin_interfaces`).

### 4.2 Camera bridge (`camera_bridge_simple`)

Gazebo's camera sensor publishes on `gz-transport`, not ROS2 DDS. The bridge reads RGB frames from Gazebo
and republishes them as standard `sensor_msgs/msg/Image` on a ROS2 topic.

**Why a custom bridge:** Gazebo Harmonic uses its own transport protocol. The official `ros_gz_bridge` has
reliability issues on certain Gazebo versions and network configs. A small custom bridge with explicit
drop policies is simpler and more robust for our use case.

**Safety features:**

- **Hard-capped publish rate** (default 2 Hz, configurable via `publish_hz`). The camera natively produces
  ~30 Hz; converting 30 frames/s in Python while running PX4, Gazebo, MicroXRCEAgent, and ROS2 spikes CPU
  to the point of freezing the laptop — we learned this the hard way. 2 Hz is fast enough for victim
  detection (humans don't disappear in 500 ms) and slow enough for any developer laptop.
- **Single-slot frame queue (`maxsize=1`), drop-on-full.** Between each publish tick only the most recent
  frame is held. If the ROS2 side is busy, new frames are dropped, not queued. Blocking would back-pressure
  into Gazebo's callback thread and destabilise the simulation. Dropping is the correct behaviour for a
  real-time data plane.
- **Stats every 5 s** (`[CAM BRIDGE STATS]`): received / dropped / published counters — the at-a-glance
  health check. Expect `ros_published` to increase by exactly 10 every 5 s (= 2 Hz), with a large
  `gz_dropped` count (~28 of every 30 frames dropped). **The high drop count is intentional and healthy.**

**Pixel format:** both `gz.msgs.Image` and `sensor_msgs/Image` store RGB pixels as a flat row-major byte
array, so the conversion is a raw byte copy — no decoding, no OpenCV, no Pillow. Fastest possible passthrough.

The bridge is parameterised by `instance` (the PX4 instance index). All derived names — Gazebo model,
Gazebo topic, ROS2 topic — come from this single integer via `drone_naming.py`.

### 4.3 Drone task publisher (`drone_task_publisher`)

The heart of the drone-side intelligence. One node per drone, parameterised by `instance`.

**Per camera frame:**

1. **Filter** — three cheap CV checks in order of cost. The first failed check drops the frame; the
   rest are skipped.

   | Check | What it catches | Cost | Threshold |
   |---|---|---|---|
   | Brightness | All-black / washed-out frames (glitch, looking into the sun) | < 1 ms (numpy `.mean()`) | mean ∈ [20, 240] |
   | Inter-frame diff | Frames identical to previous (stationary drone, static scene) | ~1 ms (numpy subtraction on grayscale) | mean ‖diff‖ ≥ 0.1 |
   | Blur (variance of Laplacian) | Frames too blurry for detection | ~3 ms (cv2.Laplacian on grayscale) | variance ≥ 100 |

   Grayscale is computed **once** and reused by both the diff and blur checks. No model is required —
   each check is a few lines of numpy/OpenCV; heavy CV belongs on the fog.

2. **Attach position** — PX4 local position (NED relative to spawn) is cached from
   `/fmu/out/vehicle_local_position_v1` and included in every task payload, with a `valid` flag derived
   from the EKF's `xy_valid` and `z_valid` outputs:

   ```json
   "position": {"valid": true, "x": 18.21, "y": 25.13, "z": -0.04}
   ```

   We use **local position, not global lat/lon**: it's always present once the EKF converges (~10–20 s
   after boot), accurate enough to map relative victim positions, and directly interpretable against the
   spawn coordinates configured in `PX4_GZ_MODEL_POSE`. Global position would require GPS lock and add
   complexity; it can be added later if absolute geolocation is needed.

3. **Decide tier** — `decide_target(task_type, priority, drone_failing)` returns `'local'`, `'fog'`, or
   `'cloud'`. A pure function with a lookup table by task type, plus one override (see §5).

4. **Publish** to the topic for that tier: `/drone0/task/local`, `/drone0/task/fog`, or
   `/drone0/task/cloud`. When a frame passes, the task carries the actual scores so filter behaviour is
   visible in the message itself: `"filter_scores": {"brightness": 142.5, "diff": 18.7, "blur": 245.3}`.

**Every 5 seconds (0.2 Hz)**, the node also emits a `STATUS_REPORT` task containing PX4 nav state, arming
state, failsafe flag, pre-flight check status, `drone_failing` flag, and current position.

**Cache-and-republish pattern for status:** the PX4 `VehicleStatus` subscriber callback only updates
`self.latest_status` — it doesn't publish. A separate 5 s timer publishes the `STATUS_REPORT` from whatever
is currently cached. This **decouples task-generation rate from sensor rate**: if PX4 stops publishing,
status tasks keep emitting at 0.2 Hz with the last known state; if PX4 floods at 20 Hz, we still emit
exactly 0.2 Hz. Predictable rate matters for the offloading logic.

**Why we don't put image bytes inside the Task message:** a 640×480 RGB frame is ~900 KB. Putting it in a
Task message would make every task ~1 MB (and ~1.2 MB once base64-encoded into JSON), make `ros2 topic echo`
unreadable, and break the standard `sensor_msgs/Image` topology that every ROS2 vision tool expects. Instead
the payload carries a **reference** (frame_seq, timestamps, image_topic, width, height, encoding); the fog
correlates the reference with the matching frame on `/drone0/camera/image` using the timestamp. Control
plane stays small; data plane stays standard.

**QoS choices:**
- PX4 subscribers (`VehicleStatus`, `VehicleLocalPosition`): `BEST_EFFORT + TRANSIENT_LOCAL`. This matches
  PX4's internal QoS; any other choice causes the subscription to silently receive nothing (same lesson as
  Task 2).
- Camera subscriber: depth 1 — we only need the latest frame.
- Task publishers: `RELIABLE`. Tasks must not be dropped because routing/coordination decisions depend on them.

**Filter telemetry:** every 10 s, a `[FILTER STATS]` line shows received / passed / dropped counts broken
down by reason. This is the empirical evidence that the drone is doing real local preprocessing, and it is
gold for the report.

**`simulate_low_battery` parameter:** when true, status reports are emitted with priority=3 and
`drone_failing: true` in the payload, routed to fog, logged at WARN level. The fog flags these as critical.
The real reaction logic — reallocating other drones to cover the dying drone's area — is Task 5. For now we
confirm the signal propagates correctly. Real PX4 SITL batteries don't drain realistically, which is why
this is a manual flag.

### 4.4 Fog server (`fog_server`)

One node, parameterised by `num_drones`. At startup it builds subscriptions in a
`for instance in range(num_drones)` loop. For each drone it subscribes to:

- PX4 `VehicleStatus` (`BEST_EFFORT + TRANSIENT_LOCAL` QoS — **critical**; default `RELIABLE` silently fails).
- `/{drone_id}/task/fog` for fog-tier tasks.
- `/{drone_id}/camera/image` for raw camera frames.

And publishes Task-2-style decisions on `/fog/{drone_id}/decision`.

**`status_callback`** — generates a Task-2 decision (`COMMAND_HOLD_POSITION` when `nav_state == 4`,
`COMMAND_MONITOR (ARMED)` when `arming_state == 2` with a `[FOG ALERT]` warning, else
`COMMAND_NORMAL_OPERATION`). A short blocking `time.sleep(0.05)` simulates fog processing cost — a holdover
from Task 2 that Task 3.7 will replace with a non-blocking pattern. (This 50 ms value matters: the original
Task 2 `time.sleep(1)` blocked the executor and would starve the task/camera callbacks.)

**`task_callback`** — parses the JSON payload, computes end-to-end latency (`now() - task.timestamp` in ms),
and records the arrival in the event buffer. Priority-3 tasks are logged as `[FOG TASK CRITICAL]` at WARN
level (and recorded as a separate `PRIORITY3_ALERT` event); everything else is logged as `[FOG TASK]` at
INFO.

**`camera_callback`** — currently just increments a counter. Task 4 places the YOLO detection model here.

**`[FOG STATS]` every 5 s** gives per-drone counters: `s` = status messages, `t` = tasks, `f` = camera
frames, e.g. `drone0[s=12 t=72 f=60] drone1[...] drone2[...]`.

The fog also maintains the **end-of-mission cloud archival infrastructure** (event buffer + `/fog/end_mission`
service); see §6.

### 4.5 Cloud server (`cloud_server`)

A new `ament_python` package, `cloud_node`, containing a single node, `cloud_server`. Keeping
`drone_node` / `fog_node` / `cloud_node` as three distinct packages makes the three-tier story visible at
the package level; the cloud could in principle run on a separate machine, and the package boundary is the
natural place for that split.

The node's whole job: subscribe to `/fog/cloud/mission_log`, apply a randomised simulated WAN delay per
batch (non-blocking, via ROS2 one-shot timers), and write each batch to disk as JSON. No detection, no
aggregation, no analysis — the cloud is the historian, not the brain (Section 4.9 of the design).

| Parameter | Default | Purpose |
|---|---|---|
| `archive_dir` | `/tmp/cloud_archive_<startup_timestamp>` | Where to write archived batches |
| `delay_min_sec` | `0.5` | Lower bound for random WAN delay |
| `delay_max_sec` | `5.0` | Upper bound for random WAN delay |

Full detail in §6.

### 4.6 Helper module `drone_naming`

A tiny module **intentionally duplicated** into both `drone_node` and `fog_node`. Four functions translate
a PX4 instance index into every derived name:

```python
drone_id_for(0)                          # "drone0"
px4_namespace_for(2)                     # "/px4_2"   ("" for instance 0)
px4_topic_for(2, "vehicle_status_v1")    # "/px4_2/fmu/out/vehicle_status_v1"
gz_model_name_for(1)                     # "x500_depth_1"
```

Sharing Python code between ROS2 packages requires either a custom install hook or an extra "common"
package, both of which add complexity. These four functions are simple enough that duplication has lower
long-term cost than abstraction. This module is the **single source of truth** for naming: adding a drone
is one extra integer in `num_drones` / `instance`; nothing else changes.

---

## 5. How offloading works in this system

In one paragraph: **each tier produces a task only for work it cannot do itself.** The drone produces
`STATUS_REPORT` and `VICTIM_DETECTION_REQUEST` tasks for the fog (it can't run detection, and the fog needs
status for coordination). The drone's own filtering and position tracking are not tasks — they are
continuous local processing. The drone never produces cloud tasks because the drone doesn't talk to the
cloud. The fog produces cloud-tier work (mission logs) at end of mission. The cloud never produces tasks
because it is passive storage.

The `decide_target()` function on the drone implements this rule explicitly:

| Task type | Tier | Why |
|---|---|---|
| `STATUS_REPORT` | fog | Fog uses it for swarm coordination |
| `VICTIM_DETECTION_REQUEST` | fog | Fog runs the CV detector |
| `BATTERY_CHECK` *(future)* | local | Drone reads its own battery |
| `MISSION_LOG_UPLOAD` *(future, fog-produced)* | cloud | Archival |
| `DETECTION_RECORD_ARCHIVAL` *(future, fog-produced)* | cloud | Archival |
| `METRICS_REPORT` *(future, fog-produced)* | cloud | Archival |

Unknown task types fall back to `fog` (safest default). Plus **one override**: if the drone is failing
(`drone_failing == True`), `STATUS_REPORT` is forced to fog regardless of the default, so the dying-drone
signal is never lost.

**Priority scheme:** all tasks default to **priority 0**. The only escalation is a dying drone, which makes
`STATUS_REPORT` jump to **priority 3** (critical), adds `"drone_failing": true` to the payload, forces fog
routing, and logs at WARN. Detection tasks stay at priority 0 even when the drone is dying — only the status
channel escalates, because that's the channel the fog uses for coordination decisions.

**Why a pure function with a lookup table, not a smart algorithm:** separating policy from plumbing makes
the function trivial to unit-test, and when policy changes (e.g., adding a network-condition override later)
only this one function changes. For a disaster-response system, deterministic and explainable beats clever
and opaque — a lookup table is something you can put in a single-row table in the report and defend in the
viva.

**Topic-based routing** advertises all three tier topics at startup whether or not they carry traffic, so
`ros2 topic list | grep /drone0/task` shows all three immediately, ready for subscribers. Adding a new tier
(e.g., a "peer drone" tier for D2D handoff) is one new entry in a dict — no schema changes.

---

## 6. Cloud archival in depth (Task 3.8)

The cloud's only role is **non-real-time archival**. Three architectural points it makes visible:

1. **The drone never talks to the cloud.** The fog is the only producer on `/fog/cloud/mission_log`.
2. **The cloud topic carries zero traffic during the mission.** `[CLOUD STATS]` shows `received=0`
   consistently until `end_mission` is called — the architectural principle made observable.
3. **The cloud handles multiple in-flight batches concurrently and archives them out-of-order** based on
   each batch's independent random delay — realistic for a WAN with jitter.

### 6.1 End-of-mission service on the fog

**Service:** `/fog/end_mission`, **type:** `std_srvs/srv/Trigger` (empty request; response is
`bool success` + `string message`). When called, the fog:

1. Snapshots the current event buffer (`list(self.event_buffer)`).
2. Splits it into chunks of 1000 events.
3. Builds a batch dict per chunk with `batch_index`, `total_batches`, `event_count`, `fog_timestamp`, and
   the events.
4. Publishes each batch as a JSON-encoded `std_msgs/String` on `/fog/cloud/mission_log`.
5. Clears the buffer.
6. Returns the total event count, batch count, and how many events were dropped during the mission due to
   buffer overflow.

Call it from the terminal:

```bash
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

**Why a service, not a topic:** a service gives clean request/response semantics with a confirmation
message (events flushed and dropped). A topic is fire-and-forget and could be missed if it arrives before
the fog is ready.

### 6.2 The fog-side event buffer

Implemented as a `collections.deque` with `maxlen=10000`: bounded by construction (oldest evicted at the
cap), O(1) `append()` on the hot path, and an O(n) drain into a list only at `end_mission`.

Every task that arrives generates a `TASK_RECEIVED` event:

```json
{
  "event_type": "TASK_RECEIVED",
  "drone_id": "drone0",
  "fog_received_at": 1778700123.456,
  "payload": {
    "task_id": "drone0-detect-0042",
    "task_type": "VICTIM_DETECTION_REQUEST",
    "priority": 0,
    "latency_ms": 4.7,
    "task_timestamp_sec": 1778700123,
    "task_timestamp_nsec": 451000000,
    "payload_keys": ["frame_seq", "frame_timestamp_sec", "filter_scores", "position", "..."]
  }
}
```

Priority-3 (dying-drone) tasks additionally generate a `PRIORITY3_ALERT` event so they're easy to find in
the archive (carrying `task_id`, `drone_failing`, and `position`).

**Overflow tracking:** if the buffer is full and a new event arrives, the oldest is silently dropped and
`events_dropped_on_overflow` is incremented; that count is returned in the `end_mission` response. The
10,000-event cap covers roughly 3 hours of operation at current per-drone task rates — far more than any
demo needs. A `[FOG BUFFER]` line logs the buffer length whenever it changes meaningfully.

**Why `std_msgs/String` with JSON, not a typed message:** mission-log events are heterogeneous
(`TASK_RECEIVED` vs `PRIORITY3_ALERT` have different fields). A typed message would need every possible
field (most empty) or one message type per event. The same JSON-in-String pattern used for `Task.payload`
is the cleanest answer.

### 6.3 The non-blocking delay pattern

The naive approach (`time.sleep(delay)`) blocks the entire executor — no other callback runs during the
sleep. Instead the cloud schedules a ROS2 one-shot timer per batch:

```python
delay_sec = random.uniform(0.5, 5.0)
timer = self.create_timer(delay_sec, deferred_archival)
self._pending_timers[id(batch)] = timer
```

When the timer fires, `deferred_archival` does the disk write and removes its `_pending_timers` entry. The
callback returns immediately after scheduling, the executor stays free, multiple delays run concurrently
with independent expiry, and because timers run on the same executor as callbacks there is no threading or
synchronisation complexity.

If the fog flushes 3 batches and the cloud picks delays of 4.2 s, 1.1 s, 3.0 s, the batches **archive
out-of-order** (1.1 s first, then 3.0 s, then 4.2 s). This out-of-order archival is the headline visual for
Task 3.8: it demonstrates concurrent batch handling, realistic WAN jitter, and the non-blocking pattern in
action.

### 6.4 On-disk archive format

Each batch is one JSON file: `batch_<NNNNNN>_recv<RECV_MS>.json`, where `NNNNNN` is a zero-padded counter
assigned in **archival order** (so files sort chronologically by when they hit disk) and `RECV_MS` is the
millisecond receive timestamp (keeping filenames unique across sessions).

```json
{
  "batch_index": 1,
  "total_batches": 3,
  "fog_timestamp": 1778700100.123,
  "cloud_received_at": 1778700100.125,
  "cloud_archived_at": 1778700104.347,
  "simulated_wan_delay_sec": 4.222,
  "event_count": 1000,
  "events": [ { "event_type": "TASK_RECEIVED", "drone_id": "drone0", "...": "..." } ]
}
```

The four timestamps form a measurable timeline. The difference
`cloud_archived_at - cloud_received_at` equals `simulated_wan_delay_sec` to within milliseconds — the proof
that the delay actually happened. The fog→cloud delivery time (`cloud_received_at - fog_timestamp`) reflects
real ROS2 IPC, typically a few ms on one machine; in a distributed deployment it would be larger, but the
simulated delay still dominates.

**Why write to disk (not an in-memory archive):** disk is more useful for the demo and report — you can
show actual JSON files with real data afterward. An in-memory archive would need another retrieval service
and would be lost on cloud-node restart.

---

## 7. How each Task 3 sub-task is satisfied

| Sub-task | Status | Implementation |
|---|---|---|
| **3.1 Understand Offloading Concept** | ✅ | Three-tier architecture defined; drone–fog–cloud responsibilities documented |
| **3.2 Define Task Types** | ✅ | Nine task types catalogued (T1–T9), tier assignments fixed |
| **3.3 Extend Drone Node to Generate Tasks** | ✅ | `Task.msg` schema; `drone_task_publisher` generates STATUS_REPORT and VICTIM_DETECTION_REQUEST; camera bridge |
| **3.4 Create Offloading Decision Module** | ✅ | `decide_target()` pure function, lookup table + dying-drone override; frame filter; position attachment |
| **3.5 Integrate Decision into Drone Node** | ✅ | Decision called inline in both task-generation paths; drone publishes to tier-specific topic |
| **3.6 Implement Local Processing (Drone Side)** | ✅ | Frame filter + status generation + position attachment run continuously (~7 ms/frame) |
| **3.7 Reuse Fog Node for Fog Processing** | ✅  | Fog logs/parses tasks; swarm aggregation and non-blocking decision delay still to come |
| **3.8 Simulate Cloud Processing** | ✅ | `cloud_node` + end-of-mission flush + non-blocking WAN delay + on-disk JSON archive |
| **3.9 Implement Task Routing** | ✅ | Topic-based routing — routing IS the topic; no separate routing module |
| **3.10 Handle Multi-Drone Offloading** | ✅ | `num_drones` scales the fog; `instance` scales drone-side nodes |
| **3.11 Add Logging for Decisions** | ✅ | Every task log shows `-> <tier>`; filter stats every 10 s; fog stats every 5 s; buffer logs |
| **3.12 Test Different Scenarios** | Partial | 1-drone and 3-drone runs validated; dying-drone path validated; more cloud scenarios ongoing |
| **3.13 Measure Basic Performance** | Partial | Per-task latency logged on fog; full fog-only vs cloud-only comparison pending |

---

## 8. Repository layout

```
ros2_ws/
└── src/
    ├── task_msgs/                          # custom message package (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       └── Task.msg
    │
    ├── drone_node/                         # ament_python
    │   ├── package.xml
    │   ├── setup.py
    │   └── drone_node/
    │       ├── __init__.py
    │       ├── drone_naming.py             # helper (duplicated in fog_node)
    │       ├── camera_bridge_simple.py     # Gazebo → ROS2 RGB bridge
    │       ├── drone_task_publisher.py     # task generator: filter, position, decision, routing
    │       ├── drone_status_publisher.py   # legacy from Task 2
    │       └── drone_reactor.py            # legacy from Task 2
    │
    ├── fog_node/                           # ament_python
    │   ├── package.xml                     # adds <exec_depend>std_srvs</exec_depend>
    │   ├── setup.py
    │   └── fog_node/
    │       ├── __init__.py
    │       ├── drone_naming.py             # duplicated copy of helper
    │       └── fog_server.py               # multi-drone subscriptions + event buffer + end_mission
    │
    ├── cloud_node/                         # ament_python (NEW in 3.8)
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/cloud_node
    │   └── cloud_node/
    │       ├── __init__.py
    │       └── cloud_server.py             # simulated cloud archive
    │
    ├── multi_drone_offboard/               # from Task 1, unchanged
    └── px4_msgs/                           # PX4 message definitions, unchanged
```

---

## 9. Prerequisites

- **Ubuntu 22.04** (LTS)
- **ROS2 Humble** (full desktop install recommended; ships `std_srvs`)
- **Gazebo Harmonic** (gz-sim 8.x)
- **PX4-Autopilot** built with SITL (`make px4_sitl_default`) at `~/PX4-Autopilot`
- **MicroXRCEAgent** at `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`
- **Python bindings:** `python3-gz-transport13`, `python3-gz-msgs10`, `python3-opencv`, `numpy`
- **px4_msgs** matching your PX4 branch, cloned into `ros2_ws/src/`
- The `baylands_collapsed_fixed` world available in your PX4 install (from Task 1)

Quick verification:

```bash
ros2 --version
gz sim --version
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
python3 -c "import cv2, numpy; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print('all ok')"
ros2 interface show std_srvs/srv/Trigger    # should print: bool success / string message
```

If any check fails, install the missing component before continuing.

---

## 10. Build instructions

From a fresh clone:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

The first build takes a few minutes because `task_msgs` and `px4_msgs` compile message bindings in C++.

Rebuild only Task 3 packages:

```bash
colcon build --packages-select task_msgs drone_node fog_node cloud_node
source install/setup.bash
```

Verify the custom message and the cloud entry point:

```bash
ros2 interface show task_msgs/msg/Task     # should list the six fields
ros2 pkg executables cloud_node            # should print: cloud_node cloud_server
```

**Rule of thumb — every new terminal needs BOTH:**

```bash
source /opt/ros/humble/setup.bash       # ROS2 itself
source ~/ros2_ws/install/setup.bash     # your custom packages and messages
```

Skip the second and `ros2 topic echo`, `ros2 topic hz`, and `ros2 interface show` all break for any topic
using `task_msgs/msg/Task` with `The message type 'task_msgs/msg/Task' is invalid` — the runtime nodes are
unaffected; this only hits CLI tools. A handy alias:

```bash
echo "alias rosws='source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

Then `rosws` in any new terminal sets everything up. **After every build, re-source `install/setup.bash`
in every terminal where you run nodes**, or new code/entry points won't be picked up.

---

## 11. Run instructions — 1 drone

Use this for first bring-up or debugging. Six terminals; `rosws` in every one.

### T1 — MicroXRCEAgent
```bash
cd ~/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```
Leave it running. Expected: `... init | running... | port: 8888`.

### T2 — PX4 drone0 with camera
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=0 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed \
PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```
The key piece is `PX4_SIM_MODEL=gz_x500_depth` — the x500 with the OakD-Lite camera. Plain `gz_x500` has no
camera. A Gazebo window opens on the Baylands scene; after ~20 s you reach a `pxh>` prompt. Then:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```
Glance at T1 — new client connection lines confirm the PX4↔ROS2 bridge is alive.

### T3 — Verify topics
```bash
rosws
ros2 topic list | grep -E "(vehicle_status|local_position|camera)"
```
Expected: `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`, and the Gazebo camera topic
(`gz topic -l | grep IMX214`).

### T4 — Camera bridge
```bash
rosws
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0
```

### T5 — Fog server (start BEFORE the task publisher)
```bash
rosws
ros2 run fog_node fog_server --ros-args -p num_drones:=1
```

### T6 — Drone task publisher
```bash
rosws
ros2 run drone_node drone_task_publisher --ros-args -p instance:=0
```

You should see roughly one `VICTIM_DETECTION_REQUEST` per passed camera frame (~2 Hz) plus one
`STATUS_REPORT` every 5 s, each logged with `-> fog`. The fog logs matching `[FOG TASK]` lines with
`latency=...ms`.

---

## 12. Run instructions — 3 drones (validated)

Eleven terminals (twelve with the cloud). Use Terminator with a tiling grid. `rosws` in every terminal.

| T# | Command |
|---|---|
| T1 | `cd ~/Micro-XRCE-DDS-Agent/build && ./MicroXRCEAgent udp4 -p 8888` |
| T2 | PX4 drone0 (below) |
| T3 | PX4 drone1 (below) |
| T4 | PX4 drone2 (below) |
| T5 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0` |
| T6 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=1` |
| T7 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=2` |
| T8 | `ros2 run fog_node fog_server --ros-args -p num_drones:=3` |
| T9 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=0` |
| T10 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=1` |
| T11 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=2` |
| (T12) | `ros2 run cloud_node cloud_server` |

### PX4 launch commands

```bash
# Drone 0
cd ~/PX4-Autopilot
PX4_INSTANCE=0 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0

# Drone 1
PX4_INSTANCE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="23,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1

# Drone 2
PX4_INSTANCE=2 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="30,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 2
```

After each PX4 reaches `pxh>`:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

### Launch order
1. T1 (agent) first.
2. T2–T4 (PX4 instances) — wait for each `pxh>` and re-run the `uxrce_dds_client` lines before starting
   the next.
3. T5–T7 (camera bridges).
4. T8 (fog) — **before** the task publishers, so it's subscribed when tasks start arriving.
5. T9–T11 (task publishers).
6. (Optional) T12 (cloud).

### Tuning if your laptop struggles
If `htop` shows sustained 100% across cores, drop bridge rate from 2 Hz to 1 Hz:
```bash
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0 -p publish_hz:=1.0
```
(Same for instances 1 and 2.)

### Dying-drone demo
Kill drone0's task publisher and restart with the flag:
```bash
ros2 run drone_node drone_task_publisher --ros-args -p instance:=0 -p simulate_low_battery:=true
```
Verify: drone0's status logs appear at WARN with `DRONE_FAILING`; the fog logs `[FOG TASK CRITICAL]` with
`PRIORITY=3 failing=True` for drone0 only; drone1/drone2 stay at priority 0.

---

## 13. Run instructions — N drones

The system scales to any positive N:

1. Launch N PX4 instances using indices 0 through N−1.
2. Launch N camera bridges with `instance:=0` … `instance:=N-1`.
3. Launch the fog with `num_drones:=N`.
4. Launch N task publishers with `instance:=0` … `instance:=N-1`.

The naming convention (`drone0`…`droneN-1` mapped to PX4 instances 0…N−1) is encoded in `drone_naming.py`
and used uniformly across all nodes.

**Real-world limits:** four drones is comfortable on most developer laptops. Six pushes Gazebo's real-time
factor below 0.3 and PX4 starts throwing `Accel #0 TIMEOUT` errors. Validated up to 3 drones.

---

## 14. Run instructions — cloud archival

### Quick standalone test (no drones)

Confirms the fog↔cloud integration without the full stack.

```bash
# T1 — Fog (1 drone so it doesn't complain about num_drones)
rosws
ros2 run fog_node fog_server --ros-args -p num_drones:=1

# T2 — Cloud  (note the archive_dir it prints)
rosws
ros2 run cloud_node cloud_server

# T3 — Trigger end_mission with an empty buffer
rosws
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

Expected response: `success=True, message='No events to archive.'`. Both nodes keep running (normal). No
files appear. This confirms the service is wired correctly and handles the empty case.

### Full end-to-end

Bring up the standard 3-drone stack (§12), then add the cloud:

```bash
# T12 — Cloud server
rosws
ros2 run cloud_node cloud_server
```

`[CLOUD STATS]` appears every 10 s with `received=0` consistently — **the key architectural observation:
no cloud traffic during the mission.** Meanwhile the fog's `[FOG BUFFER]` lines show the buffer growing:

```
[FOG BUFFER] 56 events buffered (soft_cap=10000, overflow_drops=0)
[FOG BUFFER] 134 events buffered (soft_cap=10000, overflow_drops=0)
```

(Optional) restart drone0's publisher with `-p simulate_low_battery:=true` so `PRIORITY3_ALERT` events also
land in the archive.

End the mission:
```bash
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

Fog:
```
[FOG END_MISSION] Flushing 220 events in 1 batch(es) to cloud.
[FOG END_MISSION] Published batch 1/1 (220 events).
```

Cloud (within ~5 s):
```
[CLOUD] Received batch 1/1 (220 events). Scheduling archival in 2.73s.
[CLOUD ARCHIVED] batch 1/1 (220 events) after 2.73s delay -> /tmp/cloud_archive_.../batch_000000_recvNNN.json
[CLOUD STATS] received=1 archived=1 events_archived=220 pending=0
```

### Multi-batch demo (the headline visual)

Let the system run 3–5 minutes so the buffer accumulates 2000+ events. On `end_mission`, the fog splits
into chunks of 1000 and the cloud archives them **out of order** by random delay:

```
[CLOUD] Received batch 1/3 (1000 events). Scheduling archival in 4.21s.
[CLOUD] Received batch 2/3 (1000 events). Scheduling archival in 1.13s.
[CLOUD] Received batch 3/3 (456 events).  Scheduling archival in 3.07s.
[CLOUD ARCHIVED] batch 2/3 ... after 1.13s delay -> ...
[CLOUD ARCHIVED] batch 3/3 ... after 3.07s delay -> ...
[CLOUD ARCHIVED] batch 1/3 ... after 4.21s delay -> ...
```

Batch 2 (shortest delay) archives first — proving concurrent batch handling, WAN jitter, and the
non-blocking delay pattern.

---

## 15. Inspecting the cloud archive

```bash
ls /tmp/cloud_archive_*/
# batch_000000_recv1778700104221.json  batch_000001_...  batch_000002_...
```

Filename order (by counter) matches **archival** order, not `batch_index` — files sort chronologically by
when they hit disk.

```bash
# Pretty-print one file
cat /tmp/cloud_archive_*/batch_000000_*.json | python3 -m json.tool | head -40
```

Count events by type and drone:

```bash
python3 <<'EOF'
import json, glob, collections
counts, drones = collections.Counter(), collections.Counter()
for path in sorted(glob.glob('/tmp/cloud_archive_*/batch_*.json')):
    with open(path) as f:
        batch = json.load(f)
    for ev in batch['events']:
        counts[ev['event_type']] += 1
        drones[ev['drone_id']] += 1
print('Events by type:', dict(counts))
print('Events by drone:', dict(drones))
EOF
# Events by type: {'TASK_RECEIVED': 220, 'PRIORITY3_ALERT': 6}
# Events by drone: {'drone0': 80, 'drone1': 73, 'drone2': 73}
```

Verify the delay was actually applied:

```bash
python3 <<'EOF'
import json, glob
for path in sorted(glob.glob('/tmp/cloud_archive_*/batch_*.json')):
    with open(path) as f:
        b = json.load(f)
    measured = b['cloud_archived_at'] - b['cloud_received_at']
    print(f"Batch {b['batch_index']}/{b['total_batches']}: "
          f"sim_delay={b['simulated_wan_delay_sec']:.3f}s, "
          f"measured={measured:.3f}s, events={b['event_count']}")
EOF
# Batch 1/3: sim_delay=4.221s, measured=4.222s, events=1000
# Batch 2/3: sim_delay=1.131s, measured=1.131s, events=1000
# Batch 3/3: sim_delay=3.073s, measured=3.073s, events=456
```

`simulated_wan_delay_sec` matches `measured` to within milliseconds — the cloud is genuinely delaying.

---

## 16. Verification checklists

### Drone + fog (steady state, N drones, `simulate_low_battery=False`)

After ~60 s of operation:

- [ ] All N PX4 `pxh>` prompts alive
- [ ] `ros2 topic list | grep vehicle_status_v1` shows N topics
- [ ] `ros2 topic list | grep /drone.*/task/fog` shows N topics
- [ ] `ros2 topic list | grep /drone0/task` shows all three tier topics (local/fog/cloud)
- [ ] Each camera bridge's `ros_published` increases by ~10 every 5 s (2 Hz)
- [ ] Fog `[FOG STATS]` shows non-zero `s`, `t`, `f` for **all N drones** growing in lockstep
- [ ] Each task publisher shows `[FILTER STATS]` every 10 s with non-zero `in` and `passed`
- [ ] Each publisher emits both STATUS_REPORT and VICTIM_DETECTION_REQUEST lines, each ending `-> fog`
- [ ] `ros2 topic hz /drone0/task/local` and `.../task/cloud` show **no** messages (correct)
- [ ] One sample fog task (`ros2 topic echo /drone1/task/fog --once --full-length`) shows `position` with
      `valid: true`, reasonable X/Y/Z near the spawn pose, and `filter_scores` with all three keys

### Dying-drone (`simulate_low_battery:=true` on one drone)

- [ ] That drone's status logs come through at `[WARN]` with `DRONE_FAILING`
- [ ] Fog logs `[FOG TASK CRITICAL]` with `PRIORITY=3 failing=True` for that drone only
- [ ] Status task payload contains `"drone_failing": true`
- [ ] Detection tasks stay at priority 0 (no over-escalation)
- [ ] Status still routes to fog (no routing change); other drones unaffected

### Cloud archival

- [ ] `ros2 pkg executables cloud_node` lists `cloud_server`
- [ ] Cloud prints its archive directory at startup
- [ ] During the mission, `[CLOUD STATS]` shows `received=0` consistently (zero traffic)
- [ ] `[FOG BUFFER]` shows the event count growing during the mission
- [ ] `ros2 service list | grep end_mission` shows `/fog/end_mission`
- [ ] Empty-buffer `end_mission` returns `success=True, "No events to archive"`
- [ ] Non-empty `end_mission` returns the correct event/batch counts; fog logs `Published batch X/Y`
- [ ] Cloud logs `[CLOUD] Received batch X/Y` immediately, then `[CLOUD ARCHIVED]` after each delay
- [ ] With multiple batches, archival order does NOT match send order (random delays working)
- [ ] JSON files exist in the archive dir, each with real `task_id`/`task_type`/`drone_id` events
- [ ] `simulated_wan_delay_sec` matches `cloud_archived_at - cloud_received_at`
- [ ] Fog and cloud keep running after `end_mission` (no exit — expected)

---

## 17. Tuning notes and lessons learned

### Status report rate
Initial: 1 Hz. Final: every 5 s (0.2 Hz). 1 Hz produced excessive log noise; at 5 s the fog still detects a
failed drone within 5–10 s and bandwidth scales ~5× better as drones are added. Industry norms span 1 Hz
(MAVLink heartbeat) to 0.1 Hz (telemetry summaries); 0.2 Hz sits comfortably in range.

### Filter `DIFF_MIN` threshold
Initial: 2.0. Final: 0.1. The 2.0 value assumed natural sensor noise between frames. In Gazebo with a
stationary drone, consecutive frames are nearly bit-identical, and the filter dropped **99.9%** as "static":
```
[FILTER STATS] drone0: in=860 passed=1 (0.1%) dropped=859 [dark=0 bright=0 static=859 blur=0]
```
At 0.1 the filter still drops bit-identical frames (frozen-camera check) but passes everything else
(~90% pass rate on a stationary drone). In a deployed, moving drone the threshold can be raised back to
1.0–5.0; it should arguably become a launch parameter (currently a module-level constant).

### Camera resolution and per-frame cost
The OakD-Lite defaults to 640×480 at 30 Hz. We throttle to 2 Hz in the bridge and filter at that rate, so
each frame costs ~7 ms of CPU (brightness + diff + blur). Comfortable for a single drone; multi-drone may
need further optimisation.

### EKF convergence timing
PX4's EKF takes 10–20 s after boot to converge. During this window `position.valid == false` and `x/y/z`
are `None`. Tasks generated in this window still flow correctly but with invalid positions. Downstream
consumers should treat invalid positions as "position unknown" rather than dropping the task.

### Per-terminal sourcing
Every new terminal needs both `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash`.
The CLI tools can't introspect custom messages without the workspace source; runtime nodes are unaffected.

### `ros2 topic echo` truncation
Long payloads are truncated unless `--full-length` is passed; image topics flood unless `--no-arr` is used:
```bash
ros2 topic echo /drone0/task/fog --once --full-length
ros2 topic echo /drone0/camera/image --once --no-arr
```

### Camera bridge load
One bridge at 2 Hz adds ~10–20% CPU on one core. Three concurrently are sustainable. For larger swarms or
weaker hardware, drop to 1 Hz with `-p publish_hz:=1.0`.

### Buffer sizing
The 10,000-event cap covers ~3 hours at current per-drone task rates — never a concern for a 5-minute demo.
For very long missions, raise `EVENT_BUFFER_SOFT_CAP` (a one-line change) or trigger `end_mission` between
mission phases.

---

## 18. Troubleshooting

**`The message type 'task_msgs/msg/Task' is invalid` / `ModuleNotFoundError: No module named 'task_msgs'`**
Workspace not sourced in this terminal. Run `source ~/ros2_ws/install/setup.bash` (or `rosws`). Runtime
nodes are unaffected; this only hits CLI tools.

**`Failed to subscribe to Gazebo topic` in the camera bridge**
The Gazebo topic doesn't exist. Verify with `gz topic -l | grep IMX214`. If empty, PX4 wasn't launched with
the camera model — confirm `PX4_SIM_MODEL=gz_x500_depth` (not `gz_x500`).

**Bridge runs but `gz_received` stays at 0**
gz-transport discovery issue. Either let auto-detection work (default — don't set `GZ_IP`) or pin to your
LAN IP before importing `gz.transport13`:
```python
import os
os.environ['GZ_IP'] = '192.168.1.8'   # your actual IP
```

**`[FILTER STATS]` shows `passed=0` (all frames dropped)**
If `static` is high → `DIFF_MIN` too aggressive; set to 0.1. If `dark`/`bright` is high → unusual lighting
in the world. If everything is `blur` → genuinely blurry frames; check the SDF lens settings.

**`ros2 topic echo /drone0/task/fog --field task_type` shows only one type**
Only STATUS_REPORT → the filter is dropping every camera frame (see `DIFF_MIN`). Only
VICTIM_DETECTION_REQUEST → the status timer isn't firing; check that `latest_status` isn't stuck at `None`
(PX4 status not arriving).

**`position` is always `{"valid": false, ...}`**
EKF hasn't converged. Wait 10–20 s after boot. If it never goes valid, review PX4 sim parameters
(mag/GPS/baro checks).

**Fog shows `s=` increasing but `t=0` for one drone**
That drone's task publisher isn't running, or has a mismatched `instance` vs the fog's subscription.

**One drone missing from `ros2 topic list`**
`uxrce_dds_client` wasn't restarted in that PX4's terminal. At its `pxh>`: `uxrce_dds_client stop` then
`uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888`.

**Fog latency consistently grows instead of staying small**
A callback is blocking the executor. Ensure `fog_server.py` uses `time.sleep(0.05)`, not the old Task 2
`time.sleep(1)`.

**`Accel #0 fail: TIMEOUT` flooding the PX4 console**
Gazebo's real-time factor dropped due to overload. Close other apps. If it persists 30+ s, kill everything
(`pkill -9 px4 ruby gz MicroXRCEAgent`) and retry with fewer drones or a lower bridge rate.

**`ImportError: No module named cv2`** → `sudo apt install python3-opencv`.

**Cloud node starts but never receives batches**
The fog only publishes on `/fog/cloud/mission_log` at `end_mission`. Confirm `ros2 topic list | grep
mission_log` shows the topic (else the fog isn't running / wasn't rebuilt).

**`end_mission` service call hangs**
`ros2 service list | grep end_mission` — if absent, the fog isn't running or wasn't rebuilt. Re-run
`colcon build --packages-select fog_node` and re-source.

**Cloud disk write fails** (`[CLOUD] Failed to write ...`)
Archive dir unwritable. Point it somewhere writable:
```bash
ros2 run cloud_node cloud_server --ros-args -p archive_dir:=$HOME/cloud_archive
```

**Fog/cloud "kept working" after `end_mission`** — expected. Both are long-lived; `end_mission` is a
one-shot action. Ctrl+C to stop.

**Buffer overflow during mission** (`Dropped during mission due to overflow: N`, N > 0)
The mission exceeded the 10,000-event cap. Raise `EVENT_BUFFER_SOFT_CAP` and rebuild, or trigger
`end_mission` more frequently.

**Laptop freezes during launch**
Overloaded machine. Drop to a TTY (Ctrl+Alt+F3), `pkill -9 px4 ruby gz MicroXRCEAgent python3`, reboot if
needed, and close browsers/other apps before the next attempt; run bridges at 1 Hz.

---

## 19. Design decisions worth defending in the viva

These are the points where the design pushed back against the assignment text or a previous draft, and the
reasoning the supervisor should appreciate:

1. **Cloud is archival only, not a backend.** Section 4.9 of the report commits to this; Task 3.8
   implements it literally — no live processing, no return path to drones, no traffic during the mission.
2. **Local "tier" exists but isn't task-based.** A drone sending tasks to itself would be architectural
   theater; local work runs inline as continuous processing.
3. **Topic-based routing instead of a `target_layer` field.** The ROS2-native pattern — the topic *is* the
   routing decision; each tier subscribes only to its own topic.
4. **Custom typed message + JSON-in-string payload** instead of plain `String` or typed message variants.
   Typed envelope gives validation and tooling; JSON payload keeps the schema stable as task types grow.
5. **Control plane / data plane separation.** Task descriptors (small, typed) and camera frames (standard
   `sensor_msgs/Image`) travel on separate channels; tasks carry a frame *reference*, not image bytes.
6. **End-of-mission flush** instead of periodic batch upload. Keeps the fog's network/CPU focused on
   real-time work during the mission; "press a button when the mission ends" is operationally unambiguous.
7. **Non-blocking simulated WAN delay** via one-shot timers, not `time.sleep`. Enables concurrent in-flight
   batches and out-of-order archival with no threading complexity.
8. **Service (not topic) for `end_mission`.** Clean request/response with a confirmation message (events
   flushed and dropped); a topic could be missed and gives no acknowledgement.
9. **Disk archive (not in-memory).** Real JSON files to show in the report; survives cloud-node restart.
10. **Pure decision function with a lookup table**, not a clever algorithm. Deterministic, explainable,
    unit-testable; one place to change when policy evolves.
11. **Integer-based scalability** (`instance`, `num_drones`) instead of hardcoded drone IDs. One number to
    add a drone; all names derive from `drone_naming.py`.
12. **Status rate 0.2 Hz**, not 1 Hz. Within industry norms, reduces log noise, scales bandwidth with the
    swarm.
13. **`drone_naming.py` duplicated, not shared.** Sharing Python across ROS2 packages costs more than
    duplicating four trivial functions.

---
---

## 20. Quick reference — file map

| File | Purpose | Notes |
|---|---|---|
| `src/task_msgs/msg/Task.msg` | Custom Task message schema | — |
| `src/task_msgs/CMakeLists.txt` | Message generation config | — |
| `src/task_msgs/package.xml` | Message package manifest | — |
| `src/drone_node/drone_node/drone_naming.py` | Instance → names helper | duplicated in fog_node |
| `src/drone_node/drone_node/camera_bridge_simple.py` | Gazebo → ROS2 RGB bridge | 2 Hz throttle, drop-on-full |
| `src/drone_node/drone_node/drone_task_publisher.py` | Task generator: filter, position, decision, routing | — |
| `src/drone_node/setup.py` / `package.xml` | Entry points / dependencies | — |
| `src/fog_node/fog_node/drone_naming.py` | Duplicated copy of helper | — |
| `src/fog_node/fog_node/fog_server.py` | Multi-drone subscriptions + event buffer + `end_mission` | — |
| `src/fog_node/package.xml` | Adds `<exec_depend>std_srvs</exec_depend>` | modified in 3.8 |
| `src/cloud_node/cloud_node/cloud_server.py` | Simulated cloud archive | new in 3.8 |
| `src/cloud_node/package.xml` / `setup.py` / `setup.cfg` | Cloud package build config | new in 3.8 |

### Log labels emitted by the system

| Label | Source | When |
|---|---|---|
| `[CAM BRIDGE]` / `[CAM BRIDGE STATS]` | camera_bridge_simple | startup / every 5 s |
| `[DRONE TASK PUB]` | drone_task_publisher | per published task / startup |
| `[FILTER STATS]` | drone_task_publisher | every 10 s |
| `[FOG]` | fog_server | startup / config |
| `[FOG TASK]` / `[FOG TASK CRITICAL]` | fog_server | per task arrival (CRITICAL = priority 3) |
| `[FOG ALERT]` | fog_server | when a drone is detected as ARMED |
| `[FOG STATS]` | fog_server | every 5 s |
| `[FOG BUFFER]` | fog_server | when the buffer length changes |
| `[FOG END_MISSION]` | fog_server | on the service call |
| `[CLOUD]` / `[CLOUD ARCHIVED]` / `[CLOUD STATS]` | cloud_server | per receive / per archived batch / every 10 s |

---

*Consolidated Task 3 reference — covers sub-tasks 3.1–3.13 including cloud archival (3.8). Supersedes the
per-sub-task READMEs. Next: Task 3.7 (fog-side swarm aggregation with non-blocking delay) and Task 4 (real
victim detection model).*
