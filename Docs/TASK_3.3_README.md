# Task 3.3 — Drone Task Generation with Camera Integration

This document explains everything we added on top of Task 2 to complete Task 3.3 of the **Fog-Enabled UAV Swarm System for Low-Latency Disaster Response** project. By the end of this sub-task, each drone:

- Generates structured **Task messages** describing units of work.
- Publishes them on a dedicated ROS2 topic.
- Streams raw RGB camera frames from Gazebo into ROS2 in parallel.
- Has its tasks and camera frames received and logged at the fog node.

This document is written for someone who just cloned the repository and has never seen the project before. Read it top to bottom.

---

## Table of Contents

1. [What we built and why](#1-what-we-built-and-why)
2. [System architecture](#2-system-architecture)
3. [Repository layout](#3-repository-layout)
4. [Detailed explanation of each new component](#4-detailed-explanation-of-each-new-component)
   - 4.1 The `task_msgs` package
   - 4.2 The camera bridge (`camera_bridge_simple`)
   - 4.3 The drone task publisher (`drone_task_publisher`)
   - 4.4 The extended fog server (`fog_server`)
5. [Prerequisites](#5-prerequisites)
6. [Build instructions](#6-build-instructions)
7. [Run instructions — step by step](#7-run-instructions--step-by-step)
8. [Verification checklist](#8-verification-checklist)
9. [Troubleshooting](#9-troubleshooting)
10. [What's intentionally NOT done yet](#10-whats-intentionally-not-done-yet)

---

## 1. What we built and why

### The problem we're solving

In Task 2, drones published raw status data and the fog interpreted it directly. There was no concept of a "task" — it was just data flowing. In a real fog-assisted UAV system, drones don't merely emit telemetry. They generate **structured units of work** that describe *what* should be processed and *what data* is involved. Where each task actually runs (drone, fog, or cloud) is decided by an offloading mechanism that we'll build in Task 3.4.

For Task 3.3 specifically, we need to:

- Turn the drone from a passive data source into an active **task producer**.
- Establish two parallel communication paths between drone and fog:
  - **Control plane** — small structured task descriptors.
  - **Data plane** — large sensor data (camera frames).
- Verify that the fog can receive and parse both planes simultaneously.

### Why this separation matters

Real distributed systems separate the *description of work* from the *bulk data the work operates on*. The same pattern shows up in HTTP (headers vs body), gRPC (metadata vs payload), and every production fog/edge platform. We do it here for three concrete reasons:

1. **Task messages stay small and routable.** A `Task` message is a few hundred bytes — easy to filter, prioritise, queue, and log.
2. **Camera frames stay on their native type.** `sensor_msgs/Image` is the standard ROS2 image type. Any tool (`rqt_image_view`, OpenCV, YOLO inference) can consume it directly without unpacking JSON or custom envelopes.
3. **Throughput is independent.** A burst of detection requests doesn't slow down camera frames, and vice versa.

---

## 2. System architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          GAZEBO (Harmonic)                               │
│                                                                          │
│   PX4 SITL drone0                                                        │
│   ├─ VehicleStatus  ──(uxrce-dds)──► ROS2 topic                          │
│   │                                  /fmu/out/vehicle_status_v1          │
│   │                                                                      │
│   └─ Camera sensor IMX214 ──► gz-transport topic                         │
│                              /world/.../sensor/IMX214/image              │
└──────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
        ┌──────────────────────────────────────────────────────┐
        │              camera_bridge_simple                    │
        │  Subscribes to gz-transport, throttles to 2 Hz,      │
        │  drops old frames, publishes on ROS2.                │
        └──────────────────────────────────────────────────────┘
                                  │
                                  ▼ /drone0/camera/image  (Image, 2 Hz)
        ┌──────────────────────────────────────────────────────┐
        │              drone_task_publisher                    │
        │                                                      │
        │  Subscribes to:                                      │
        │    • /fmu/out/vehicle_status_v1                      │
        │    • /drone0/camera/image                            │
        │                                                      │
        │  Emits tasks on /drone0/task:                        │
        │    • STATUS_REPORT          (every 1 s, PX4-driven)  │
        │    • VICTIM_DETECTION_REQUEST (per camera frame)     │
        └──────────────────────────────────────────────────────┘
                                  │
                                  ▼ /drone0/task  (task_msgs/Task)
        ┌──────────────────────────────────────────────────────┐
        │                    fog_server                        │
        │                                                      │
        │  Subscribes (per drone) to:                          │
        │    • /fmu/out/vehicle_status_v1   (PX4 status)       │
        │    • /drone0/task                 (control plane)    │
        │    • /drone0/camera/image         (data plane)       │
        │                                                      │
        │  Logs each task with end-to-end latency.             │
        │  Counts camera frames received.                      │
        │  Publishes Task 2 decisions back to drones.          │
        └──────────────────────────────────────────────────────┘
```

The pipeline is fully parallel: status, tasks, and camera frames each travel on their own topic and are handled by independent callbacks on the fog side.

---

## 3. Repository layout

After cloning, your workspace should look like this:

```
ros2_ws/
└── src/
    ├── task_msgs/                          # NEW — custom message package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       └── Task.msg
    │
    ├── drone_node/                         # MODIFIED — added 2 new nodes
    │   ├── package.xml
    │   ├── setup.py
    │   └── drone_node/
    │       ├── __init__.py
    │       ├── drone_status_publisher.py   # (from Task 2)
    │       ├── drone_reactor.py            # (from Task 2)
    │       ├── drone_task_publisher.py     # NEW
    │       └── camera_bridge_simple.py     # NEW
    │
    ├── fog_node/                           # MODIFIED — extended fog server
    │   ├── package.xml
    │   ├── setup.py
    │   └── fog_node/
    │       ├── __init__.py
    │       └── fog_server.py               # MODIFIED
    │
    ├── multi_drone_offboard/               # (unchanged, from Task 1)
    └── px4_msgs/                           # (unchanged, from Task 1)
```

---

## 4. Detailed explanation of each new component

### 4.1 The `task_msgs` package

**What it is:** A ROS2 package that defines a single custom message type, `Task.msg`. It contains zero source code — only the message definition and the build configuration to generate Python/C++ bindings.

**Why it's a separate package:** ROS2 custom messages must live in an `ament_cmake` package, not a Python package, because the message generator runs on CMake. Mixing message definitions into a Python node package will silently fail. Keeping messages in a dedicated package is the standard practice.

**The Task.msg schema:**

```
string task_id
string task_type
string drone_id
builtin_interfaces/Time timestamp
uint8 priority
string payload
```

| Field | Purpose |
|---|---|
| `task_id` | Unique instance identifier, e.g. `drone0-detect-0042`. Used in logs to track a specific task. |
| `task_type` | Task category. Used by the offloading decision module (Task 3.4) to route tasks. Examples: `STATUS_REPORT`, `VICTIM_DETECTION_REQUEST`. |
| `drone_id` | Which drone produced the task. The fog uses this to send results back to the right drone. |
| `timestamp` | When the task was created at the drone. The fog computes `now - timestamp` to measure end-to-end latency. |
| `priority` | 0 (low) to 3 (critical). Used for routing in later sub-tasks. |
| `payload` | Free-form JSON string. Schema depends on `task_type`. This is the **envelope-plus-payload** pattern: fixed metadata, variable payload. |

**Why JSON inside a string field instead of typed fields?**
Different task types carry different data. `STATUS_REPORT` carries PX4 state; `VICTIM_DETECTION_REQUEST` carries a frame reference. A single message with optional fields for every task type would balloon as we add types. A typed metadata envelope plus a JSON payload keeps the schema stable forever. When we add `LOG_UPLOAD`, `BATTERY_CHECK`, or `THERMAL_FRAME` tasks later, we don't touch `Task.msg`.

**Files involved:**
- `task_msgs/msg/Task.msg` — the schema.
- `task_msgs/CMakeLists.txt` — invokes `rosidl_generate_interfaces` to build Python bindings.
- `task_msgs/package.xml` — declares `rosidl_default_generators` and `builtin_interfaces` dependencies.

---

### 4.2 The camera bridge (`camera_bridge_simple`)

**What it does:** Reads RGB camera frames from Gazebo's internal transport (`gz-transport`) and republishes them as standard `sensor_msgs/msg/Image` on ROS2, throttled to 2 Hz.

**Why we need a bridge:** Gazebo Harmonic uses its own transport protocol (`gz-transport`), not ROS2 DDS. The official `ros_gz_bridge` package exists but has reliability issues on certain Gazebo versions and network configurations. Writing a small custom bridge with explicit drop policies is simpler and more robust for our use case.

**Why we throttle to 2 Hz:**
The camera sensor produces 30 frames per second. Converting 30 frames/s in Python on a laptop while simultaneously running PX4, Gazebo, MicroXRCEAgent, and ROS2 will spike CPU usage to the point of freezing the machine — we learned this the hard way. 2 Hz is fast enough for victim detection demos (humans don't disappear in 500 ms) and slow enough that any developer laptop can sustain it.

**Key safety features in the bridge code:**

1. **Single-slot queue (`maxsize=1`).** Between each ROS2 publish tick, only one frame is held in memory. If a new frame arrives before the previous one is published, the new one is **dropped**, not queued. This is a deliberate choice — queuing would cause RAM to grow without bound under load.

2. **Drop-on-full instead of block-on-full.** If we blocked when the queue was full, the Gazebo callback thread would stall, which would back-pressure into Gazebo itself and could destabilise the simulation. Dropping frames is the correct behaviour for a real-time data plane.

3. **Stats every 5 seconds.** The bridge prints a count of received vs published vs dropped frames. This is your at-a-glance health check.

4. **Single drone only (drone0).** The bridge intentionally serves one drone. Multi-drone support is added later when we know the system is stable.

**Pixel format details:** Both `gz.msgs.Image` and `sensor_msgs/msg/Image` store RGB pixels as a flat byte array in the same row-major order, so the conversion is a raw byte copy — no decoding, no OpenCV, no Pillow. Fastest possible passthrough.

**What its log output looks like:**
```
[CAM BRIDGE] Bridging Gazebo /world/.../IMX214/image -> ROS2 /drone0/camera/image at 2.0 Hz
[CAM BRIDGE STATS] gz_received=152, gz_dropped=141, ros_published=10
[CAM BRIDGE STATS] gz_received=303, gz_dropped=282, ros_published=20
```

You should see `ros_published` increase by exactly 10 every 5 seconds (= 2 Hz). The high drop count is expected and healthy — we want to drop ~28 of every 30 frames to maintain the throttle.

---

### 4.3 The drone task publisher (`drone_task_publisher`)

**What it does:** Generates `Task` messages from the drone side. Two task types in this sub-task:

| Task type | Producer trigger | Rate | Priority | Payload |
|---|---|---|---|---|
| `STATUS_REPORT` | 1 Hz timer | 1 Hz | 1 (normal) | PX4 nav state, arming state, failsafe flags |
| `VICTIM_DETECTION_REQUEST` | Each received camera frame | 2 Hz (matches bridge) | 2 (high) | Frame reference: sequence, timestamp, image topic, dimensions |

**How it handles two different rate sources:**

The publisher uses the **cache-and-republish** pattern for PX4 status:
- A subscriber callback runs every time PX4 publishes a `VehicleStatus` message (variable rate, often ~2 Hz).
- The callback only updates `self.latest_status` — it doesn't publish anything itself.
- A separate 1 Hz timer fires every second and publishes a `STATUS_REPORT` using whatever is currently in `self.latest_status`.

This **decouples task generation rate from sensor rate**. If PX4 stops publishing for a few seconds, the publisher keeps emitting status tasks at 1 Hz with the last known state. If PX4 publishes 20 times per second, the publisher still emits exactly 1 Hz. The rate is predictable, which matters for the offloading decision module.

For camera frames, the pattern is different — every frame produces a task. The camera is the trigger, not a timer. The frame's metadata becomes the task's payload.

**Why we don't put image bytes inside the Task message:**

Three reasons:

1. **Size.** A 640×480 RGB frame is 921,600 bytes (~900 KB). Putting that inside a Task message would make every task ~1 MB. `ros2 topic echo /drone0/task` would become unreadable.
2. **Serialisation overhead.** Encoding 900 KB of binary as a JSON string field requires base64 (33% overhead). Total: ~1.2 MB per task, encoded.
3. **Topology.** The fog (and later, the detection model) wants to subscribe to a continuous image stream, not unpack images out of task envelopes. The standard `sensor_msgs/Image` type plays well with every ROS2 vision tool.

Instead, the task payload contains a **reference**:

```json
{
  "frame_seq": 42,
  "frame_timestamp_sec": 1778599512,
  "frame_timestamp_nsec": 100000000,
  "image_topic": "/drone0/camera/image",
  "width": 640,
  "height": 480,
  "encoding": "rgb8"
}
```

The fog (or any future processor) can correlate this reference with the corresponding frame on `/drone0/camera/image` using the timestamp. Control plane stays small; data plane stays standard.

**QoS choices:**
- PX4 subscriber: `BEST_EFFORT + TRANSIENT_LOCAL`. This is the QoS PX4 uses internally; any other choice causes the subscription to silently receive nothing. This is the same lesson from Task 2.
- Camera subscriber: default `RELIABLE`. The bridge publishes with `RELIABLE`. Depth of 1 — we only need the latest frame.
- Task publisher: default `RELIABLE`. Tasks must not be dropped because routing decisions depend on them.

**`drone_id` parameter:**
The script is parameterised by `drone_id`. A single source file serves all three drones; you launch three instances with `drone0`, `drone1`, `drone2` and each one subscribes to its own PX4 / camera topic and publishes to its own task topic. No code duplication.

---

### 4.4 The extended fog server (`fog_server`)

**What changed from Task 2:** The Task 2 fog server only subscribed to PX4 `VehicleStatus`. In Task 3.3, it additionally subscribes to:

- The task topic of each drone (`/{drone_id}/task`)
- The camera topic of each drone (`/{drone_id}/camera/image`)

The Task 2 functionality (PX4 status → simple decision → publish back on `/fog/{drone_id}/decision`) is preserved unchanged.

**Three callbacks per drone:**

1. **`status_callback`** — PX4 VehicleStatus arrives. Generates a Task 2 decision (`HOLD_POSITION`, `MONITOR`, `NORMAL_OPERATION`). Same logic as Task 2.

2. **`task_callback`** — A `Task` message arrives.
   - Increments the per-drone task counter.
   - Parses the JSON payload.
   - Computes end-to-end latency: `now() - task.timestamp` in milliseconds.
   - Logs everything in a single structured line.

3. **`camera_callback`** — A camera frame arrives. Currently just increments a counter. In Task 4 this is where the victim-detection model will run.

**The 5-second stats line:**

```
[FOG STATS] drone0[s=148 t=60 f=150] drone1[s=0 t=0 f=0] drone2[s=0 t=0 f=0]
```

- `s` = PX4 status messages received
- `t` = task messages received
- `f` = camera frames received

This is your at-a-glance health check. All three channels growing = healthy.

**Important change vs Task 2:** The Task 2 `time.sleep(1)` simulated delay was shortened to 50 ms in this sub-task. The 1-second sleep was **blocking the executor**, which would have starved the new task and camera callbacks. A proper non-blocking simulated processing delay (using a timer-based delayed publish) is the subject of Task 3.7. For now, 50 ms keeps the system responsive while still representing some fog processing cost.

**Why the fog subscribes to the camera topic at all:**

This is a design choice we deliberately made. There are two valid options:

- **Option A (what we did):** The fog subscribes to the camera topic directly. Task messages contain a reference to the matching frame. The fog correlates task ↔ frame using the timestamp.
- **Option B:** Image bytes travel inside Task messages. The fog only subscribes to the task topic.

We chose Option A because it's how production systems work: control plane and data plane on separate channels. Task messages stay small and easy to read in `ros2 topic echo`; image data uses the standard ROS2 image type that every vision tool understands.

---

## 5. Prerequisites

Before building, confirm you have:

- **Ubuntu 22.04** (LTS)
- **ROS2 Humble** (full desktop install recommended)
- **Gazebo Harmonic** (`gz-sim 8.x`)
- **PX4-Autopilot** built with SITL (`make px4_sitl_default`) at `~/PX4-Autopilot`
- **MicroXRCEAgent** built at `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`
- **Python bindings for gz-transport**: `python3-gz-transport13`
- **Python bindings for gz-msgs**: `python3-gz-msgs10`
- **Pillow** (for the verification screenshot tool): `pip3 install --user Pillow`

Check each one:

```bash
ros2 --version                                      # should show Humble
gz sim --version                                    # should show 8.x
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4   # file must exist
ls ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent      # file must exist
python3 -c "from gz.transport13 import Node; print('OK')"
python3 -c "from gz.msgs10.image_pb2 import Image; print('OK')"
```

If any check fails, install the missing component before proceeding.

You should also have these from earlier sub-tasks:

- A `px4_msgs` package cloned into `ros2_ws/src/` matching your PX4 branch.
- The `baylands_collapsed_fixed` world available in your PX4 install (from Task 1 / Chapter 6).

---

## 6. Build instructions

From a fresh clone:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

The first build will take a few minutes because `task_msgs` and `px4_msgs` compile message bindings in C++.

To rebuild only the new packages later:

```bash
colcon build --packages-select task_msgs drone_node fog_node
source install/setup.bash
```

After every build, **re-source `install/setup.bash` in every terminal where you'll run nodes**, or the new entry points won't be visible.

Verify the custom message is registered:

```bash
ros2 interface show task_msgs/msg/Task
```

You should see the six fields of the schema. If you get "package not found," the build didn't succeed or you didn't source `install/setup.bash`.

---

## 7. Run instructions — step by step

We use 6 terminals. Open all 6 first and label them mentally. A tiling terminal like Terminator helps a lot.

**In every terminal, source ROS2 first:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1 — MicroXRCEAgent

The bridge between PX4 and ROS2. Must start before PX4.

```bash
cd ~/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

**Expected output:**
```
[1234567890] info | UDPv4AgentLinux.cpp | init | running... | port: 8888
```

Leave this terminal running. Don't type anything else in it.

### Terminal 2 — PX4 SITL drone0 (with camera)

```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=0 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed \
PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

The key piece is `PX4_SIM_MODEL=gz_x500_depth` — this is the x500 quadcopter with the OakD-Lite camera attached. Plain `gz_x500` has no camera.

**Expected:**
- A Gazebo window opens showing the Baylands disaster scene with a drone on the ground.
- After ~20 seconds, the PX4 console reaches a `pxh>` prompt.

If you see repeated `Accel #0 fail: TIMEOUT` errors, wait 30 seconds. They usually self-resolve once Gazebo's real-time factor stabilises.

At the `pxh>` prompt:
```
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

Glance at Terminal 1 — you should see new client connection lines appearing. That confirms the PX4-to-ROS2 bridge is alive.

### Terminal 3 — Verify Gazebo camera is publishing

Before bridging anything, confirm Gazebo itself is producing camera frames:

```bash
gz topic -l | grep camera
```

You should see, among others:
```
/world/baylands_collapsed_fixed/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

That's the RGB camera. If it's not in the list, the camera sensor isn't active — check the launch command.

### Terminal 4 — Camera bridge

```bash
ros2 run drone_node camera_bridge_simple
```

**Expected:**
```
[CAM BRIDGE] Bridging Gazebo .../IMX214/image -> ROS2 /drone0/camera/image at 2.0 Hz
```

After 5 seconds:
```
[CAM BRIDGE STATS] gz_received=152, gz_dropped=141, ros_published=10
```

Numbers to verify:
- `gz_received` is growing (Gazebo is feeding us frames).
- `ros_published` adds exactly 10 every 5 seconds (we're publishing at 2 Hz).
- `gz_dropped` ≈ `gz_received` − `ros_published`, and it should be large. **This is intentional** — the throttle is working.

If `gz_received` stays at 0, jump to [Troubleshooting](#9-troubleshooting).

### Terminal 5 — Fog server

**Start this BEFORE the task publisher**, so the fog is already subscribed when the first task arrives.

```bash
ros2 run fog_node fog_server
```

**Expected:**
```
[FOG] drone0: status=/fmu/out/vehicle_status_v1, task=/drone0/task, camera=/drone0/camera/image
[FOG] drone1: status=/px4_1/fmu/out/vehicle_status_v1, task=/drone1/task, camera=/drone1/camera/image
[FOG] drone2: status=/px4_2/fmu/out/vehicle_status_v1, task=/drone2/task, camera=/drone2/camera/image
[FOG STATS] drone0[s=10 t=0 f=10] drone1[s=0 t=0 f=0] drone2[s=0 t=0 f=0]
```

The fog subscribes for all three drones; only drone0 has data at this stage, so `drone1` and `drone2` stay at zero — that's correct.

`s` and `f` start growing immediately (PX4 status and camera frames are already flowing). `t` stays at 0 until we start the next terminal.

### Terminal 6 — Drone task publisher

```bash
ros2 run drone_node drone_task_publisher --ros-args -p drone_id:=drone0
```

**Expected publisher output:**
```
[DRONE TASK PUB] drone0: PX4 status from /fmu/out/vehicle_status_v1
[DRONE TASK PUB] drone0: camera frames from /drone0/camera/image
[DRONE TASK PUB] drone0: tasks published on /drone0/task
[DRONE TASK PUB] drone0: published drone0-detect-0000 (VICTIM_DETECTION_REQUEST, frame_seq=1)
[DRONE TASK PUB] drone0: published drone0-status-0000 (STATUS_REPORT)
[DRONE TASK PUB] drone0: published drone0-detect-0001 (VICTIM_DETECTION_REQUEST, frame_seq=2)
[DRONE TASK PUB] drone0: published drone0-detect-0002 (VICTIM_DETECTION_REQUEST, frame_seq=3)
[DRONE TASK PUB] drone0: published drone0-status-0001 (STATUS_REPORT)
```

You should see roughly **twice as many `VICTIM_DETECTION_REQUEST` as `STATUS_REPORT`** because camera is 2 Hz and status is 1 Hz.

**Expected fog output (Terminal 5):**
```
[FOG TASK] drone0 drone0-detect-0000 type=VICTIM_DETECTION_REQUEST priority=2 latency=1.0ms payload_keys=['frame_seq', 'frame_timestamp_sec', 'frame_timestamp_nsec', 'image_topic', 'width', 'height', 'encoding']
[FOG TASK] drone0 drone0-status-0000 type=STATUS_REPORT priority=1 latency=0.7ms payload_keys=['nav_state', 'arming_state', 'failsafe', 'pre_flight_checks_pass']
...
[FOG STATS] drone0[s=148 t=60 f=150] drone1[s=0 t=0 f=0] drone2[s=0 t=0 f=0]
```

Read this carefully:
- `latency` should be single-digit ms most of the time, occasionally spiking to 20–40 ms.
- `payload_keys` should match the schema (`frame_seq, frame_timestamp_sec, ...` for detection, `nav_state, arming_state, ...` for status).
- The 5-second stats show all three channels (`s`, `t`, `f`) growing for drone0.

---

## 8. Verification checklist

After running for 30+ seconds, all of these should be true:

- [ ] Terminal 4 (bridge) `ros_published` adds 10 per 5-second window
- [ ] Terminal 5 (fog) `[FOG STATS]` shows `s`, `t`, and `f` all growing for drone0
- [ ] Terminal 5 shows both `VICTIM_DETECTION_REQUEST` and `STATUS_REPORT` task log lines
- [ ] All `[FOG TASK]` lines include a `latency=...ms` field with a small number
- [ ] All `[FOG TASK]` payload_keys match the expected schema for each type
- [ ] Per 5-second window: ~10 status tasks + ~10 detection tasks ≈ 15 tasks per stats line
- [ ] Your laptop is stable, fans not screaming

You can also independently verify:

```bash
# In a new terminal:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Confirm topics exist
ros2 topic list | grep -E "(task|camera)"

# Confirm task topic uses custom message
ros2 topic info /drone0/task --verbose

# See one full task message (use --no-arr in case of large payloads later)
ros2 topic echo /drone0/task --once

# Confirm camera publish rate
ros2 topic hz /drone0/camera/image
```

Expected:
```
/drone0/camera/image
/drone0/task
```

```
Type: task_msgs/msg/Task
Publisher count: 1
Reliability: RELIABLE
```

```
average rate: 2.000
```

---

## 9. Troubleshooting

### `ModuleNotFoundError: No module named 'task_msgs'`
You didn't source `install/setup.bash` in that terminal. Run:
```bash
source ~/ros2_ws/install/setup.bash
```

### `Failed to subscribe to Gazebo topic` in the bridge
The Gazebo topic doesn't exist yet. Check:
```bash
gz topic -l | grep IMX214
```
If empty, PX4 didn't launch with the camera model. Confirm `PX4_SIM_MODEL=gz_x500_depth` (not `gz_x500`).

### Bridge runs but `gz_received` stays at 0
Network interface mismatch. Gazebo binds to your LAN IP, gz-transport tries auto-detect. Either:

**Option 1 — let auto-detect work** (default in our code):
Make sure `os.environ['GZ_IP']` is **not** set anywhere in `camera_bridge_simple.py`.

**Option 2 — pin to your actual IP:**
Find your IP with `ip addr | grep "inet "` (look for something like `192.168.x.x` on `wlan0` or `eth0`). Then at the top of `camera_bridge_simple.py`, before any imports:
```python
import os
os.environ['GZ_IP'] = '192.168.1.8'   # your actual IP
```

### Fog logs show `latency=...` consistently growing instead of staying small
A callback in the fog is blocking the executor. Make sure `fog_server.py` uses `time.sleep(0.05)`, **not** `time.sleep(1)` (an old line from Task 2).

### `Accel #0 fail: TIMEOUT` errors flooding PX4 console
Gazebo's real-time factor is too low because your machine is overloaded. Close other applications. If the errors persist for more than 30 seconds, kill Gazebo and PX4 (`pkill -9 px4 ruby gz`) and try again with fewer drones.

### Laptop freezes during camera bridge startup
Same root cause — overloaded machine. Drop to a TTY (Ctrl+Alt+F3), log in, and run:
```bash
pkill -9 px4 ruby gz MicroXRCEAgent python3
```
Reboot if necessary. On next attempt, close all browsers and other apps before starting Gazebo.

### `ros2 topic echo /drone0/camera/image` floods terminal
Always use `--no-arr` for image topics:
```bash
ros2 topic echo /drone0/camera/image --once --no-arr
```

---

## 10. What's intentionally NOT done yet

The following are deferred to later sub-tasks. **This is by design.** Each sub-task should have one clear deliverable.

- **Offloading decision module (Task 3.4)** — right now every task is just logged at the fog. Routing tasks to drone-local vs fog vs cloud comes next.
- **Drone-side preprocessing (Task 3.6 / Task 4)** — the drone currently forwards every raw 640×480 RGB frame to the fog. In a proper deployment we'd filter for quality, possibly compress to JPEG, and only forward frames that pass a cheap "anything interesting?" check. The architecture is built to support this; the intelligence comes later.
- **Real victim detection (Task 4)** — the fog's `camera_callback` currently just counts frames. The YOLO/MobileNet inference replaces the body of this callback.
- **Cloud archival node (Task 3.8)** — a separate node simulating long WAN delay for log/archive tasks. We've designed the task taxonomy to support cloud tasks (`LOG_UPLOAD`, `DETECTION_ARCHIVE`, `METRICS_REPORT`) but haven't implemented the cloud receiver yet.
- **Thermal camera (later in Task 3 or Task 4)** — the OakD-Lite has only RGB and depth sensors. A simulated thermal feed will be added later.
- **Multi-drone simultaneous run** — the code supports drones 0, 1, 2 with no changes. We tested drone0 only to keep the laptop stable during bring-up. Running all three at once works; it just stresses the machine more.

When you start a new sub-task, refer back to this document for the existing infrastructure — don't re-build what's already there.

---

## Quick reference — file map

| File | Purpose | New in 3.3? |
|---|---|---|
| `src/task_msgs/msg/Task.msg` | Message schema | ✅ |
| `src/task_msgs/CMakeLists.txt` | Message build config | ✅ |
| `src/task_msgs/package.xml` | Message package manifest | ✅ |
| `src/drone_node/drone_node/camera_bridge_simple.py` | Gazebo→ROS2 RGB bridge | ✅ |
| `src/drone_node/drone_node/drone_task_publisher.py` | Task generator | ✅ |
| `src/drone_node/setup.py` | Entry points (updated) | Modified |
| `src/drone_node/package.xml` | Dependencies (updated) | Modified |
| `src/fog_node/fog_node/fog_server.py` | Fog with task+camera reception | Modified |
| `src/fog_node/package.xml` | Dependencies (updated) | Modified |

---

*Last updated: Task 3.3 — drone task generation with RGB camera integration. Next: Task 3.4 — offloading decision module.*
