# Task 3 — Task Offloading Mechanism (Complete Implementation)

**Project:** Fog-Enabled UAV Swarm System for Low-Latency Disaster Response  
**Scope of this document:** Tasks 3.1 through 3.6, plus the scalability refactor

This document is the single source of truth for everything we built in Task 3. It supersedes the earlier per-sub-task READMEs (TASK_3.3 and TASK_3.4). If you just cloned the repo, read this end-to-end before running anything.

---

## Table of Contents

1. [What Task 3 is about](#1-what-task-3-is-about)
2. [Design decisions (Tasks 3.1, 3.2)](#2-design-decisions-tasks-31-32)
3. [System architecture](#3-system-architecture)
4. [What we built](#4-what-we-built)
   - 4.1 Custom message package `task_msgs`
   - 4.2 Camera bridge
   - 4.3 Drone task publisher
   - 4.4 Fog server
   - 4.5 Helper module: `drone_naming`
5. [How offloading works in this system](#5-how-offloading-works-in-this-system)
6. [How each Task 3 sub-task is satisfied](#6-how-each-task-3-sub-task-is-satisfied)
7. [Repository layout](#7-repository-layout)
8. [Prerequisites](#8-prerequisites)
9. [Build instructions](#9-build-instructions)
10. [Run instructions — 1 drone](#10-run-instructions--1-drone)
11. [Run instructions — 3 drones (validated)](#11-run-instructions--3-drones-validated)
12. [Run instructions — N drones](#12-run-instructions--n-drones)
13. [Verification checklist](#13-verification-checklist)
14. [Tuning notes and lessons learned](#14-tuning-notes-and-lessons-learned)
15. [Troubleshooting](#15-troubleshooting)
16. [What's intentionally NOT done yet](#16-whats-intentionally-not-done-yet)

---

## 1. What Task 3 is about

The objective of Task 3 is to design and implement a task-offloading mechanism that decides where each unit of work runs: on the drone (edge), on the fog node, or on the cloud (simulated). Without offloading, the system has only two unsatisfactory options — process everything onboard and exhaust the drone, or push everything to the cloud and accept latencies that make the drone too slow to react. Fog computing resolves this by absorbing the bulk of mission-critical work close to the drones.

By the end of Task 3:

- The drone is an active **task producer** with onboard intelligence (frame filtering, position tagging, decision-making).
- Tasks flow on a **typed control plane** (custom ROS2 message), and sensor data flows on a parallel **data plane** (standard ROS2 image messages).
- The fog receives, parses, and acknowledges every task with measured end-to-end latency.
- The architecture scales from 1 drone to N drones with a single parameter, no code changes.

---

## 2. Design decisions (Tasks 3.1, 3.2)

### Three-tier architecture (Task 3.1)

Each tier has a **distinct, non-overlapping responsibility**:

- **Drone (edge):** lightweight, latency-critical operations that must run regardless of network conditions. Examples: telemetry generation, battery/health checks, camera-frame pre-filtering, attaching the drone's position to outgoing data.
- **Fog (swarm coordinator):** medium-complexity, mission-critical workloads. Examples: victim detection from camera frames, swarm-wide status aggregation, mission-decision logic. This is the **brain of the swarm**.
- **Cloud (archival only):** strictly **non-real-time** data storage. The cloud receives logs, aggregated maps, performance metrics, and confirmed detection records from the fog. The cloud **never runs live processing** and **never returns commands** during a mission. This matches Section 4.9 of the project design.

### Communication pattern

- **Drone → Fog:** sensor data, telemetry, task offload requests.
- **Fog → Drone:** decisions, area assignments, commands (e.g., re-tasking when a drone is failing).
- **Fog → Cloud:** logs, archives, performance metrics (one-way, non-real-time).
- **Drone ↔ Cloud:** never. The drone has no business talking to the cloud directly.

### Task catalog (Task 3.2)

| ID | Task Name | Producer | Executed At | Status in Task 3 |
|---|---|---|---|---|
| T1 | Telemetry Status Generation | Drone | Drone (local) | ✅ Implemented |
| T2 | Battery & Health Check | Drone | Drone (local) | Catalogued; runs as part of T1 |
| T3 | Camera Frame Pre-filtering | Drone | Drone (local) | ✅ Implemented |
| T4 | Victim Detection | Drone (request) | Fog | ✅ Request side implemented (model in Task 4) |
| T5 | Swarm Status Aggregation | Fog | Fog | Catalogued; full impl in Task 3.7 |
| T6 | Threat / Mission Decision | Fog | Fog | Catalogued; full impl in Task 5 |
| T7 | Mission Log Upload | Fog | Cloud (simulated) | Catalogued; Task 3.8 |
| T8 | Detection Record Archival | Fog | Cloud (simulated) | Catalogued; Task 3.8 |
| T9 | Performance Metrics Report | Fog | Cloud (simulated) | Catalogued; Task 3.13 |

### Where offloading decisions live (Tasks 3.4, 3.5)

Each tier knows what it can do and what it must offload. The drone has a `decide_target()` function that maps task types to tiers, but **the heart of offloading is that each tier produces tasks only for work it cannot do itself**. The drone never produces cloud tasks because the drone never talks to the cloud. The fog never produces drone tasks because the fog talks down via commands, not tasks. The architecture is implicit in the responsibilities of each tier; the explicit function is there for the assignment requirement and to leave room for future task types.

### Local processing (Task 3.6)

Local processing on the drone is implemented as **continuous background operations**, not as discrete tasks pulled off a queue. Filtering frames, generating status, attaching position — these are not "tasks" that get scheduled; they are simply what being a drone means. The `/{drone_id}/task/local` topic exists in the architecture (for future expansion) but is currently unused. This is a deliberate architectural choice: a drone sending tasks to itself would be indirection without value.

---

## 3. System architecture

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
   │     /drone0/task/cloud   (no consumer — Task 3.8)                    │
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
   │   Stats line every 5 s.                                              │
   └──────────────────────────────────────────────────────────────────────┘
```

**Key insight: topic = routing.** Instead of stuffing a `target_layer` field into the Task message, we publish to three different topics based on the decision. Each tier subscribes only to its own topic. This makes Task 3.9 (Task Routing) practically free and makes Task 3.7/3.8 a clean drop-in.

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

**Why custom-typed, not plain `String`:** typed message gives compile-time field validation, integrates with `ros2 topic info` / `ros2 interface show`, and produces self-documenting output in `ros2 topic echo`. The schema lives in `ament_cmake` because ROS2 message generation requires CMake.

**Why JSON payload inside a string field:** the metadata envelope (task_id, type, drone_id, timestamp, priority) is stable forever. Different task types carry different payloads — STATUS_REPORT carries PX4 state, VICTIM_DETECTION_REQUEST carries a frame reference. JSON-in-string keeps the schema stable as new task types are added without ever modifying `Task.msg`.

### 4.2 Camera bridge (`camera_bridge_simple`)

Gazebo's camera sensor publishes on `gz-transport`, not ROS2 DDS. The bridge reads RGB frames from Gazebo and republishes them as standard `sensor_msgs/msg/Image` on a ROS2 topic, with two safety features:

- **Hard-capped publish rate** (default 2 Hz, configurable). The camera natively produces 30 Hz; we throttle aggressively to keep CPU usage manageable on developer laptops.
- **Single-slot frame queue, drop-on-full.** Between each ROS2 publish tick, only the most recent frame is held in memory. If the ROS2 side is busy, new frames are dropped, not queued. Blocking would back-pressure into Gazebo and destabilise the simulation.

The bridge is parameterised by `instance` (the PX4 instance index). All derived names — Gazebo model, Gazebo topic, ROS2 topic — are generated from this single integer.

### 4.3 Drone task publisher (`drone_task_publisher`)

The heart of the drone-side intelligence. One node per drone. Per camera frame:

1. **Filter** — three cheap CV checks in order of cost. First failed check drops the frame.

   | Check | What it catches | Cost | Threshold |
   |---|---|---|---|
   | Brightness | Black or washed-out frames | < 1 ms (numpy mean) | mean ∈ [20, 240] |
   | Inter-frame diff | Frames identical to previous | ~1 ms (numpy subtraction on grayscale) | mean ‖diff‖ ≥ 0.1 |
   | Blur | Frames too blurry for detection | ~3 ms (cv2.Laplacian) | variance ≥ 100 |

2. **Attach position** — PX4 local position (NED coordinates relative to spawn point) is cached from `/fmu/out/vehicle_local_position_v1` and included in every task payload, along with a `valid` flag derived from the EKF's `xy_valid` and `z_valid` outputs.

3. **Decide tier** — `decide_target(task_type, priority, drone_failing)` returns `'local'`, `'fog'`, or `'cloud'`. It's a pure function with a lookup table by task type, with one override: if the drone is failing, force STATUS_REPORT to fog.

4. **Publish** to the topic for that tier: `/drone0/task/local`, `/drone0/task/fog`, or `/drone0/task/cloud`.

Every 5 seconds, the node also emits a `STATUS_REPORT` task containing PX4 nav state, arming state, failsafe flag, pre-flight check status, drone_failing flag, and current position.

**Filter telemetry:** every 10 seconds, a `[FILTER STATS]` log line shows received/passed/dropped counts broken down by reason. This is the empirical evidence that the drone is doing real local preprocessing.

**`simulate_low_battery` parameter:** when true, status reports are emitted with priority=3 and `drone_failing: true` in the payload. The fog flags these as critical (WARN-level logs). The real reaction logic — reallocating other drones to cover the dying drone's area — is the subject of Task 5. For now we just confirm the signal propagates correctly.

### 4.4 Fog server (`fog_server`)

One node, parameterised by `num_drones`. At startup it builds subscriptions in a `for instance in range(num_drones)` loop. For each drone:

- Subscribes to PX4 `VehicleStatus` topic with BEST_EFFORT + TRANSIENT_LOCAL QoS.
- Subscribes to `/{drone_id}/task/fog` for fog-tier tasks.
- Subscribes to `/{drone_id}/camera/image` for raw camera frames.
- Publishes Task-2-style decisions on `/fog/{drone_id}/decision`.

The task callback:

- Parses the JSON payload.
- Computes end-to-end latency: `now() - task.timestamp` in milliseconds.
- Logs each task with all fields. Priority=3 tasks are logged as `[FOG TASK CRITICAL]` at WARN level, making the dying-drone signal visible at a glance.

The camera callback currently just increments a counter. Task 4 will place the YOLO detection model here.

A `[FOG STATS]` line every 5 seconds gives per-drone counters (`s` = status messages, `t` = tasks, `f` = camera frames).

### 4.5 Helper module `drone_naming`

A tiny module duplicated into both `drone_node` and `fog_node`. Four functions translate between a PX4 instance index and all derived names:

```python
drone_id_for(0)             # "drone0"
drone_id_for(3)             # "drone3"
px4_namespace_for(0)        # ""
px4_namespace_for(2)        # "/px4_2"
px4_topic_for(2, "vehicle_status_v1")  # "/px4_2/fmu/out/vehicle_status_v1"
gz_model_name_for(1)        # "x500_depth_1"
```

The module is **intentionally duplicated** instead of shared across packages. Sharing Python code between ROS2 packages requires either a custom installation hook or an extra "common" package, both of which add complexity. The four functions are simple enough that duplication has lower long-term cost than abstraction.

This module is the **single source of truth** for naming. Adding a new drone is one extra integer in `num_drones`; nothing else changes.

---

## 5. How offloading works in this system

In one paragraph: **each tier produces a task only for work it cannot do itself.** The drone produces `STATUS_REPORT` and `VICTIM_DETECTION_REQUEST` tasks for fog (it can't do detection, and the fog needs status for coordination). The drone's own filtering and position tracking are not tasks — they are continuous local processing. The drone never produces cloud tasks because the drone doesn't talk to the cloud. The fog will produce cloud-tier tasks (logs, archives) in Task 3.8. The cloud will never produce tasks because the cloud is passive storage.

The `decide_target()` function on the drone implements this rule explicitly, with a lookup table by task type:

| Task type | Tier | Why |
|---|---|---|
| STATUS_REPORT | fog | Fog uses it for swarm coordination |
| VICTIM_DETECTION_REQUEST | fog | Fog runs the CV detector |
| BATTERY_CHECK *(future)* | local | Drone reads its own battery |
| MISSION_LOG_UPLOAD *(future, fog-produced)* | cloud | Archival |
| DETECTION_RECORD_ARCHIVAL *(future, fog-produced)* | cloud | Archival |
| METRICS_REPORT *(future, fog-produced)* | cloud | Archival |

Plus one override: if the drone is failing (`drone_failing == True`), STATUS_REPORT is forced to fog regardless of the default, ensuring the dying-drone signal is never lost.

---

## 6. How each Task 3 sub-task is satisfied

| Sub-task | Status | Implementation |
|---|---|---|
| **3.1 Understand Offloading Concept** | ✅ | Three-tier architecture defined; drone-fog-cloud responsibilities documented |
| **3.2 Define Task Types** | ✅ | Nine task types catalogued (T1–T9), tier assignments fixed |
| **3.3 Extend Drone Node to Generate Tasks** | ✅ | `Task.msg` schema, `drone_task_publisher` generates STATUS_REPORT and VICTIM_DETECTION_REQUEST |
| **3.4 Create Offloading Decision Module** | ✅ | `decide_target()` pure function, lookup table + dying-drone override |
| **3.5 Integrate Decision into Drone Node** | ✅ | Decision is called inline in both task-generation paths; drone publishes to tier-specific topic |
| **3.6 Implement Local Processing (Drone Side)** | ✅ | Frame filter + status generation + position attachment run continuously on the drone with measurable per-frame cost (~7 ms) |
| **3.7 Reuse Fog Node for Fog Processing** | ⏳ Next | Fog currently logs tasks; Task 3.7 adds real processing logic and a proper non-blocking simulated delay |
| **3.8 Simulate Cloud Processing** | ⏳ Next | Cloud node + cloud-tier producer not yet built |
| **3.9 Implement Task Routing** | ✅ Implicit | Topic-based routing means routing IS the topic; no separate routing module needed |
| **3.10 Handle Multi-Drone Offloading** | ✅ | `num_drones` parameter scales the fog; `instance` parameter scales drone-side nodes |
| **3.11 Add Logging for Decisions** | ✅ | Every task log line shows `-> <tier>`; filter stats every 10 s; fog stats every 5 s |
| **3.12 Test Different Scenarios** | Partial | 1-drone and 3-drone validation runs completed; cloud scenarios pending Task 3.8 |
| **3.13 Measure Basic Performance** | Partial | Per-task latency logged on fog; full performance comparison pending Task 3.8 + 4 |

---

## 7. Repository layout

```
ros2_ws/
└── src/
    ├── task_msgs/                          # custom message package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       └── Task.msg
    │
    ├── drone_node/
    │   ├── package.xml
    │   ├── setup.py
    │   └── drone_node/
    │       ├── __init__.py
    │       ├── drone_naming.py              # helper (duplicated in fog_node)
    │       ├── camera_bridge_simple.py
    │       ├── drone_task_publisher.py
    │       ├── drone_status_publisher.py    # legacy from Task 2
    │       └── drone_reactor.py             # legacy from Task 2
    │
    ├── fog_node/
    │   ├── package.xml
    │   ├── setup.py
    │   └── fog_node/
    │       ├── __init__.py
    │       ├── drone_naming.py              # helper (duplicated copy)
    │       └── fog_server.py
    │
    ├── multi_drone_offboard/                # from Task 1, unchanged
    └── px4_msgs/                            # from Task 1, unchanged
```

---

## 8. Prerequisites

- **Ubuntu 22.04** (LTS)
- **ROS2 Humble**
- **Gazebo Harmonic** (gz-sim 8.x)
- **PX4-Autopilot** built with SITL at `~/PX4-Autopilot`
- **MicroXRCEAgent** at `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`
- **Python**: `python3-gz-transport13`, `python3-gz-msgs10`, `python3-opencv`, `numpy`, `Pillow` (optional, for screenshots)
- **px4_msgs** matching your PX4 branch, cloned into `ros2_ws/src/`

Quick verification:
```bash
ros2 --version
gz sim --version
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
python3 -c "import cv2, numpy; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print('all ok')"
```

If any check fails, install the missing component before continuing.

---

## 9. Build instructions

From a fresh clone:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To rebuild only Task 3 packages:
```bash
colcon build --packages-select task_msgs drone_node fog_node
source install/setup.bash
```

**Rule of thumb:** every new terminal needs **both**
```bash
source /opt/ros/humble/setup.bash       # ROS2 itself
source ~/ros2_ws/install/setup.bash     # YOUR custom packages and messages
```
Skip the second one and `ros2 topic echo`, `ros2 topic hz`, `ros2 interface show` all break for any topic using `task_msgs/msg/Task`.

A handy alias:
```bash
echo "alias rosws='source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```
Then `rosws` in any new terminal sets everything up.

---

## 10. Run instructions — 1 drone

Use this when first bringing things up or debugging. Six terminals.

### T1 — MicroXRCEAgent
```bash
cd ~/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

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

At the `pxh>` prompt:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

### T3 — Verify topics
```bash
rosws
ros2 topic list | grep vehicle_status_v1
```
Expected:
```
/fmu/out/vehicle_status_v1
```

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

---

## 11. Run instructions — 3 drones (validated)

Eleven terminals. Use Terminator with a 3×4 grid.

**Source `rosws` in every terminal.**

| Terminal | Command |
|---|---|
| T1 | `cd ~/Micro-XRCE-DDS-Agent/build && ./MicroXRCEAgent udp4 -p 8888` |
| T2 | PX4 drone0 (see below) |
| T3 | PX4 drone1 (see below) |
| T4 | PX4 drone2 (see below) |
| T5 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0` |
| T6 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=1` |
| T7 | `ros2 run drone_node camera_bridge_simple --ros-args -p instance:=2` |
| T8 | `ros2 run fog_node fog_server --ros-args -p num_drones:=3` |
| T9 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=0` |
| T10 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=1` |
| T11 | `ros2 run drone_node drone_task_publisher --ros-args -p instance:=2` |

### PX4 launch commands

**Drone 0:**
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=0 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

**Drone 1:**
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="23,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1
```

**Drone 2:**
```bash
cd ~/PX4-Autopilot
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
2. T2, T3, T4 (PX4 instances) — wait for each `pxh>` and re-run the `uxrce_dds_client` lines before launching the next.
3. T5, T6, T7 (camera bridges).
4. T8 (fog) — must come before task publishers so it's subscribed when tasks start arriving.
5. T9, T10, T11 (task publishers).

### Tuning if your laptop struggles
If `htop` shows sustained 100% CPU on multiple cores, drop bridge rate from 2 Hz to 1 Hz:
```bash
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0 -p publish_hz:=1.0
```
Same for instances 1 and 2.

---

## 12. Run instructions — N drones

The system scales to any positive N. Steps:

1. Launch N PX4 instances using indices 0 through N−1.
2. Launch N camera bridges with `instance:=0` through `instance:=N-1`.
3. Launch the fog with `num_drones:=N`.
4. Launch N task publishers with `instance:=0` through `instance:=N-1`.

The naming convention (`drone0`, `drone1`, ..., `droneN-1` mapped to PX4 instances 0 through N−1) is encoded in `drone_naming.py` and used uniformly across all nodes.

**Real-world limits:** four drones is doable on most developer laptops. Six is the practical limit before Gazebo's real-time factor drops below 0.3 and PX4 starts throwing `Accel #0 TIMEOUT` errors.

---

## 13. Verification checklist

After 60 seconds of steady-state operation with N drones:

- [ ] All N PX4 `pxh>` prompts alive
- [ ] `ros2 topic list | grep vehicle_status_v1` shows N topics
- [ ] `ros2 topic list | grep /drone.*/task/fog` shows N topics
- [ ] Each camera bridge `ros_published` increases by 10 every 5 s (at 2 Hz)
- [ ] Fog `[FOG STATS]` shows non-zero `s`, `t`, `f` for **all N drones** growing in lockstep
- [ ] Each task publisher shows `[FILTER STATS]` with non-zero `passed` count
- [ ] Each task publisher emits both STATUS_REPORT and VICTIM_DETECTION_REQUEST log lines
- [ ] Fog logs `[FOG TASK]` lines for both task types from all N drones
- [ ] One sample fog task payload (`ros2 topic echo /drone1/task/fog --once --full-length`) shows position with `valid: true` and reasonable X/Y/Z values

### Dying-drone test (optional)

Kill drone0's task publisher and restart with the flag:
```bash
ros2 run drone_node drone_task_publisher --ros-args -p instance:=0 -p simulate_low_battery:=true
```

Verify:
- Drone0's status logs appear as `[WARN]` with `DRONE_FAILING`
- Fog logs `[FOG TASK CRITICAL]` with `PRIORITY=3 failing=True` for drone0 only
- Drone1, drone2 status reports continue at priority 0 (no spillover)

---

## 14. Tuning notes and lessons learned

These are findings from development that matter to anyone re-running the system.

### Status report rate

Initial implementation: 1 Hz. Final: every 5 s (0.2 Hz).

Reasoning: status reports tell fog "drone is alive and here is its position." 1 Hz produced excessive log noise. At 5 s the fog still detects a failed drone within 5–10 seconds (Task 5 will add "stale drone" detection), and bandwidth scales 5× better as drones are added. Industry norms range from 1 Hz (MAVLink heartbeat) to 0.1 Hz (telemetry summaries); 0.2 Hz sits in this range.

### Filter `DIFF_MIN` threshold

Initial implementation: 2.0. Final: 0.1.

The 2.0 threshold was based on an expectation of natural sensor noise between frames. In Gazebo's simulated camera with a stationary drone, consecutive frames are nearly bit-identical, and the filter dropped 99.9% of frames as "static":
```
[FILTER STATS] drone0: in=860 passed=1 (0.1%) dropped=859 [dark=0 bright=0 static=859 blur=0]
```

Lowering to 0.1 still drops bit-identical frames (camera-frozen check) but passes everything else. In a deployed system where the drone is moving over a search area, the threshold can be raised back to 1.0–5.0 because natural drone motion generates substantial inter-frame variation. The threshold should arguably become a launch parameter; for now it lives as a module-level constant in `drone_task_publisher.py`.

### EKF convergence timing

PX4's EKF takes 10–20 seconds after boot to converge. During this window, `position.valid == false` and `x/y/z` are `None`. Tasks generated during warmup still flow correctly but with invalid positions. Downstream consumers should treat invalid positions as "drone position unknown" rather than dropping the task.

### Per-terminal sourcing

Every new terminal needs both `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash`. The CLI tools (`ros2 topic echo`, `ros2 topic hz`, `ros2 interface show`) cannot introspect custom messages without the workspace source. The runtime nodes are unaffected — they pick up the message definitions through their package dependencies.

### `ros2 topic echo` truncation

Long payload strings are truncated unless `--full-length` is passed:
```bash
ros2 topic echo /drone0/task/fog --once --full-length
```

### Camera bridge load

One camera bridge at 2 Hz adds ~10–20% CPU on a single core. Three bridges concurrently are sustainable. For larger swarms or weaker hardware, drop to 1 Hz with `-p publish_hz:=1.0`.

---

## 15. Troubleshooting

### `The message type 'task_msgs/msg/Task' is invalid`
Workspace not sourced in this terminal. Run `source ~/ros2_ws/install/setup.bash` (or the `rosws` alias). The actual nodes are unaffected; this only affects CLI tools.

### `ModuleNotFoundError: No module named 'task_msgs'`
Same as above. Source the workspace.

### `Failed to subscribe to Gazebo topic` in camera bridge
The Gazebo topic doesn't exist. Verify with `gz topic -l | grep IMX214`. If empty, PX4 wasn't launched with the camera model. Confirm `PX4_SIM_MODEL=gz_x500_depth` (not `gz_x500`).

### Bridge runs but `gz_received` stays at 0
gz-transport network discovery issue. Either let auto-detection work (default — don't set `GZ_IP`) or pin to your LAN IP:
```python
os.environ['GZ_IP'] = '192.168.1.X'   # before importing gz.transport13
```

### `Accel #0 fail: TIMEOUT` flooding PX4 console
Gazebo's real-time factor has dropped due to overload. Close other applications. If errors persist for 30+ seconds, kill everything (`pkill -9 px4 ruby gz MicroXRCEAgent`) and try again with fewer drones or lower bridge rate.

### `[FILTER STATS]` shows `passed=0` (all frames dropped)
If `static` is high: `DIFF_MIN` too aggressive — set to 0.1.
If `dark` or `bright` is high: lighting in the simulated world is unusual — check the Gazebo render.
If everything is `blur`: camera is genuinely producing blurry frames — check the SDF for unusual lens settings.

### Fog shows `s=` increasing but `t=0` for one drone
Task publisher for that drone isn't running, or is connected to a topic the fog isn't subscribed to (mismatched `instance` parameter on either side).

### One drone missing from `ros2 topic list`
`uxrce_dds_client` wasn't restarted in that PX4's terminal. At its `pxh>`:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

### Laptop freezes during launch
Drop everything (TTY → `pkill -9 ...`). On next attempt, close browsers and other apps before starting Gazebo. Run with bridge rate at 1 Hz instead of 2 Hz.

---

## 16. What's intentionally NOT done yet

These are deliberately deferred to later sub-tasks. **This is by design** — each sub-task should have one clear deliverable.

- **Task 3.7 — Fog processing logic.** The fog currently parses and logs tasks but doesn't actually *do* anything with them beyond Task 2's status-based decisions. Task 3.7 will add real processing — at minimum, a non-blocking simulated delay using ROS2 timers instead of `time.sleep()`.
- **Task 3.8 — Cloud node.** `/drone0/task/cloud` has a publisher but no consumer. The cloud node will subscribe to a fog-produced topic (not drone-produced), apply a long simulated WAN delay (3–5 s), and archive incoming logs/metrics/detection records. No live processing on the cloud.
- **Task 4 — Real victim detection.** The fog's `camera_callback` currently just counts frames. The YOLO/MobileNet inference replaces the body of this callback in Task 4.
- **Task 5 — Threat decision logic and swarm reallocation.** The dying-drone signal arrives at the fog as a priority-3 STATUS_REPORT, but the fog doesn't yet act on it. Task 5 will implement the reallocation logic and the swarm-coordination decisions.
- **Drone-side preprocessing improvements.** The current filter is conservative. A real deployment would add JPEG compression before sending frames to fog, object-proposal generation (cheap "anything interesting here?" check), and possibly a tiny edge model.
- **Network-condition override.** The current decision module assumes the fog is always reachable. Network-condition awareness is deferred until we have a meaningful network signal.
- **Real battery integration.** `simulate_low_battery` is a manual flag. Wiring it to a real PX4 battery threshold awaits a realistic discharge model.

---

## Quick reference — file map

| File | Purpose |
|---|---|
| `src/task_msgs/msg/Task.msg` | Custom Task message schema |
| `src/task_msgs/CMakeLists.txt` | Message generation config |
| `src/task_msgs/package.xml` | Message package manifest |
| `src/drone_node/drone_node/drone_naming.py` | Instance → names helper |
| `src/drone_node/drone_node/camera_bridge_simple.py` | Gazebo → ROS2 RGB bridge |
| `src/drone_node/drone_node/drone_task_publisher.py` | Task generator with filter, position, decision |
| `src/drone_node/setup.py` | Entry points |
| `src/drone_node/package.xml` | Dependencies |
| `src/fog_node/fog_node/drone_naming.py` | (duplicated copy of helper) |
| `src/fog_node/fog_node/fog_server.py` | Fog with multi-drone subscription |
| `src/fog_node/setup.py` | Entry points |
| `src/fog_node/package.xml` | Dependencies |

---

*Last updated: end of Task 3.6 with scalability refactor and 3-drone validation. Next: Task 3.7 — fog-side processing logic with non-blocking simulated delay.*
