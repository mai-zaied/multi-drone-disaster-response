# Task 3.4 — Offloading Decision Module + Frame Filter + Position Tracking

This document explains everything we added on top of Task 3.3 to complete Task 3.4 of the **Fog-Enabled UAV Swarm System for Low-Latency Disaster Response** project.

By the end of this sub-task, each drone:

- Generates structured **Task messages** as before (Task 3.3),
- Filters its own camera frames locally before turning them into tasks,
- Attaches its current position to every task it produces,
- Decides *where* each task should be processed (drone, fog, or cloud),
- Publishes the task on the matching tier-specific topic.

The fog continues to receive the tasks that are routed to it and now sees richer payloads (position, filter scores, priority levels).

This document assumes the Task 3.3 README has already been read.

---

## Table of Contents

1. [What we built and why](#1-what-we-built-and-why)
2. [System architecture](#2-system-architecture)
3. [Repository changes](#3-repository-changes)
4. [Detailed explanation of each new component](#4-detailed-explanation-of-each-new-component)
   - 4.1 Drone-side frame filter
   - 4.2 Position attachment
   - 4.3 Offloading decision module
   - 4.4 Topic-based routing
   - 4.5 Priority and the dying-drone override
   - 4.6 Updated fog subscription
5. [Prerequisites](#5-prerequisites)
6. [Build instructions](#6-build-instructions)
7. [Run instructions — step by step](#7-run-instructions--step-by-step)
8. [Verification checklist](#8-verification-checklist)
9. [Tuning notes and lessons learned](#9-tuning-notes-and-lessons-learned)
10. [Troubleshooting](#10-troubleshooting)
11. [What's intentionally NOT done yet](#11-whats-intentionally-not-done-yet)

---

## 1. What we built and why

### The problem

In Task 3.3 we built the plumbing: the drone produced `Task` messages, the fog received them, the camera bridge forwarded images. But the drone had **no intelligence** of its own:

- It forwarded **every** camera frame, including completely blank ones, blurry ones, and frames that hadn't changed since the previous tick.
- It had **no concept of location** — tasks carried no spatial reference, so the fog couldn't tell *where* a detection would later be.
- It had **no concept of routing** — every task went to a single topic, and the fog had to handle everything regardless of where it belonged.

Task 3.4 fixes all three:

- **Frame filter** drops uninteresting frames before they ever become tasks. Saves fog compute, saves network bandwidth, makes the architecture honest.
- **Position attachment** lets the fog build the victim map in later tasks.
- **Offloading decision module** routes each task to the tier (drone-local / fog / cloud) where it belongs, using the ROS2-native pattern of *topic = routing*.

### Why this matters architecturally

The drone is now a genuine **edge node**. It does cheap onboard work (filtering, position tagging, routing decisions) before sending anything to fog. The fog is no longer being asked to look at every raw frame; it only sees the ones the drone considers worth analysing. This is the standard fog-edge division of labor described in Section 4.5 of the project design.

---

## 2. System architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          GAZEBO (Harmonic)                               │
│                                                                          │
│   PX4 SITL drone0                                                        │
│   ├─ VehicleStatus         ──► /fmu/out/vehicle_status_v1                │
│   ├─ VehicleLocalPosition  ──► /fmu/out/vehicle_local_position_v1   NEW  │
│   └─ Camera IMX214         ──► gz-transport ──► (camera bridge)          │
└──────────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
        ┌──────────────────────────────────────────────────────┐
        │              camera_bridge_simple                    │
        │           (unchanged from Task 3.3)                  │
        └──────────────────────────────────────────────────────┘
                                  │
                                  ▼ /drone0/camera/image  (Image, 2 Hz)
        ┌──────────────────────────────────────────────────────┐
        │              drone_task_publisher                    │
        │                       (HEAVILY EXTENDED)             │
        │                                                      │
        │  Subscribes to:                                      │
        │   • /fmu/out/vehicle_status_v1                       │
        │   • /fmu/out/vehicle_local_position_v1   NEW         │
        │   • /drone0/camera/image                             │
        │                                                      │
        │  Per camera frame:                                   │
        │   1. Filter (brightness / diff / blur)    NEW        │
        │   2. Attach position                      NEW        │
        │   3. Decide tier                          NEW        │
        │   4. Publish to /drone0/task/<tier>       NEW        │
        │                                                      │
        │  Every 1 s:                                          │
        │   STATUS_REPORT with position + dying-flag           │
        │                                                      │
        │  Publishes on THREE topics (one per tier):           │
        │   • /drone0/task/local                     NEW       │
        │   • /drone0/task/fog                       NEW       │
        │   • /drone0/task/cloud                     NEW       │
        └──────────────────────────────────────────────────────┘
                                  │
                ┌─────────────────┼──────────────────┐
                ▼                 ▼                  ▼
       /drone0/task/local   /drone0/task/fog   /drone0/task/cloud
        (no subscriber           │              (no subscriber yet,
         yet, Task 3.6)          ▼               Task 3.8)
                       ┌──────────────────┐
                       │    fog_server    │
                       │  (subscribes to  │
                       │   the fog tier   │
                       │   only)          │
                       └──────────────────┘
```

### Key insight: topic = routing

We use ROS2's native pattern: instead of stuffing a `target_layer` field into the `Task` message and asking every subscriber to filter for "their" tasks, we publish to **three different topics** based on the decision. Each tier subscribes only to its own topic.

- `/drone0/task/local` — drone processes these itself (Task 3.6 will implement the consumer)
- `/drone0/task/fog` — fog subscribes
- `/drone0/task/cloud` — cloud node subscribes (Task 3.8 will implement the consumer)

This makes Task 3.9 ("Task Routing") almost free — the routing IS the topic name.

---

## 3. Repository changes

| File | Action |
|---|---|
| `src/drone_node/drone_node/drone_task_publisher.py` | **Heavily modified** — adds filter, position, decision, three-topic routing |
| `src/fog_node/fog_node/fog_server.py` | **One-line change** — subscribes to `/{drone_id}/task/fog` instead of `/{drone_id}/task` |

No new packages. No `Task.msg` schema changes.

---

## 4. Detailed explanation of each new component

### 4.1 Drone-side frame filter

**What it does:** Three cheap onboard checks on every camera frame, in order from fastest to slowest. First failed check drops the frame; the rest are skipped.

| Check | What it catches | Cost | Threshold |
|---|---|---|---|
| **Brightness** | Completely dark or completely washed-out frames (camera glitch, looking into the sun) | Sub-millisecond (numpy `.mean()`) | mean ∈ [20, 240] |
| **Inter-frame difference** | Frames that look identical to the previous one (drone is stationary, scene isn't changing) | ~1 ms (numpy subtraction on grayscale) | mean ‖diff‖ ≥ 0.1 |
| **Blur (variance of Laplacian)** | Frames too blurry for detection | ~3 ms (cv2.Laplacian on grayscale) | variance ≥ 100 |

Frames that pass all three checks become `VICTIM_DETECTION_REQUEST` tasks. Frames that fail are silently dropped; no task is created.

**Why these specific checks:**
- **No model required.** Each check is a few lines of numpy/OpenCV. The drone shouldn't waste battery running heavy CV models locally — that's what the fog is for.
- **Order matters.** Brightness is the cheapest, so it's first. Blur is the most expensive, so it's last. We never compute the expensive check if a cheap one already disqualified the frame.
- **Convert to grayscale once.** The diff check and the blur check both need grayscale. We compute it once and reuse.

**Filter telemetry:**

Every 10 seconds, the publisher logs a stats line:

```
[FILTER STATS] drone0: in=860 passed=18 (2.1%) dropped=842
   [dark=0 bright=0 static=842 blur=0]
```

These numbers are gold for the project report: they show the drone is doing real local preprocessing with measurable outcomes. They also help tune thresholds for different operating conditions.

**Filter scores in the task payload:**

When a frame passes, the task carries the actual scores:

```json
"filter_scores": {"brightness": 142.5, "diff": 18.7, "blur": 245.3}
```

This makes the filter behavior visible in the message itself, not just in logs. The fog can use these scores later (Task 4) if it wants to prioritise sharper frames for detection.

---

### 4.2 Position attachment

**What it does:** The drone subscribes to PX4's local-position topic (`/fmu/out/vehicle_local_position_v1`), caches the latest position, and attaches it to every task payload.

**Why local position and not global (lat/lon):**
Local position is what PX4's EKF produces in NED coordinates (North, East, Down) relative to the drone's spawn point. It's:
- Always present once the EKF converges (~10 s after PX4 boot).
- Accurate enough to map relative positions of victims within the search area.
- Simple to interpret (matches the spawn coordinates we configure in `PX4_GZ_MODEL_POSE`).

Global position (lat/lon) requires GPS lock and adds complexity. We'll add it later if the deployment scenario needs absolute geolocation.

**The position dict in every payload:**

```json
"position": {
  "valid": true,
  "x": 18.21,
  "y": 25.13,
  "z": -0.04
}
```

The `valid` flag matters: PX4's local position can be `NaN` during the EKF warmup. We check the EKF's own `xy_valid` and `z_valid` flags and pass them through. The fog can decide what to do with invalid positions (drop the task, queue it, log it).

**Both task types carry position:**
- `STATUS_REPORT` carries position so the fog knows where each drone is (essential for swarm coordination).
- `VICTIM_DETECTION_REQUEST` carries position so when the fog detects a victim, it knows where the drone was at capture time (essential for the victim map in Task 5).

---

### 4.3 Offloading decision module

**What it is:** A pure function:

```python
decide_target(task_type, priority, drone_failing) -> 'local' | 'fog' | 'cloud'
```

**How it decides:**

The core is a lookup table by `task_type`:

| Task type | Default target | Rationale |
|---|---|---|
| `STATUS_REPORT` | fog | The drone produces it, but the fog needs it for swarm coordination |
| `VICTIM_DETECTION_REQUEST` | fog | The fog runs the CV detector |
| `BATTERY_CHECK` (future) | local | The drone reads its own battery |
| `MISSION_LOG_UPLOAD` (future) | cloud | Archival, non-real-time |
| `DETECTION_RECORD_ARCHIVAL` (future) | cloud | Archival, non-real-time |
| `METRICS_REPORT` (future) | cloud | Archival, non-real-time |

Unknown task types fall back to fog (safest default).

**Plus one override:**
- If the drone is failing (`drone_failing == True`), force `STATUS_REPORT` to fog regardless. This ensures the fog never misses a dying-drone signal.

**Why a pure function:**
Separating policy from plumbing has two practical benefits. The function is easy to unit-test in isolation. And when policy needs to change (e.g., adding a network-condition override later), only this function changes — the rest of the publisher stays untouched.

**Why a lookup table and not a smart algorithm:**
For a graduation-project disaster-response system, deterministic and explainable beats clever and opaque. A lookup table is something we can write in a single-row table in the report and defend in the viva.

---

### 4.4 Topic-based routing

**What it is:** Instead of one task topic per drone, there are now three:

```
/drone0/task/local   ← LOCAL-tier tasks
/drone0/task/fog     ← FOG-tier tasks (the only one currently consumed)
/drone0/task/cloud   ← CLOUD-tier tasks (no consumer yet)
```

The decision function picks the tier; the publisher dispatches to the matching topic. The topic name *is* the routing decision.

**Why this and not a field in `Task.msg`:**
- **No subscriber filtering.** Each tier just subscribes to its own topic. Cleaner code, fewer bugs.
- **No schema changes when adding tiers.** Adding a new tier (e.g., a "peer drone" tier for D2D handoff) is one new entry in a dict.
- **Implicit and visible.** `ros2 topic echo /drone0/task/cloud` immediately tells you what's going to the cloud. No deserialisation required.

The three topics are advertised at startup whether or not they have any traffic:

```
[DRONE TASK PUB] drone0: tasks for tier "local" -> /drone0/task/local
[DRONE TASK PUB] drone0: tasks for tier "fog"   -> /drone0/task/fog
[DRONE TASK PUB] drone0: tasks for tier "cloud" -> /drone0/task/cloud
```

When you start the publisher and check `ros2 topic list | grep /drone0/task`, all three appear immediately, ready to accept subscribers.

---

### 4.5 Priority and the dying-drone override

**Priority scheme (changed from Task 3.3):**

All tasks default to **priority 0**. The only thing that escalates priority is the drone being about to die. In that case:
- `STATUS_REPORT` jumps to **priority 3** (critical).
- Status payload includes `"drone_failing": true`.
- Status routing is forced to fog regardless of the default mapping.
- The log line is emitted as a `WARN`, not `INFO`.

Detection tasks stay at priority 0 even when the drone is dying — only the status channel escalates, because that's the channel the fog uses for swarm coordination decisions.

**Simulating low battery:**

Real PX4 SITL batteries don't drain realistically. To demo the dying-drone path, we added a launch parameter:

```bash
ros2 run drone_node drone_task_publisher --ros-args \
    -p drone_id:=drone0 \
    -p simulate_low_battery:=true
```

This artificially sets the dying flag. You'll see:

```
[WARN] [DRONE TASK PUB] drone0: published drone0-status-0000
    (STATUS_REPORT, priority=3, DRONE_FAILING) -> fog
```

Real battery integration is deferred. The signal path is what matters: when fog receives a priority-3 status, it can act (Task 5 will use this to reallocate the surviving drones).

---

### 4.6 Updated fog subscription

The fog server changes by **one field** per drone in its drone roster dict:

```python
# Before (Task 3.3):
'task_topic': '/drone0/task',

# After (Task 3.4):
'task_topic': '/drone0/task/fog',
```

That's the entire change to `fog_server.py`. Everything else stays — the same callback parses payloads, computes latency, and logs structured task lines.

---

## 5. Prerequisites

Same as Task 3.3, **plus:**

- **OpenCV for Python:** `python3 -c "import cv2; print(cv2.__version__)"` must succeed.
  If not: `sudo apt install python3-opencv`
- **Numpy for Python:** already installed via ROS2.

Check both:
```bash
python3 -c "import cv2, numpy; print('cv2', cv2.__version__, 'numpy', numpy.__version__)"
```

---

## 6. Build instructions

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_node fog_node
source install/setup.bash
```

**After every build, re-source `install/setup.bash` in every terminal where you run nodes**, or new code won't be picked up.

---

## 7. Run instructions — step by step

This builds on Task 3.3's run procedure. Terminals 1–4 are unchanged: MicroXRCEAgent, PX4 with `gz_x500_depth`, camera bridge.

Open six terminals. Source ROS2 + workspace in every one:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1 — MicroXRCEAgent
```bash
cd ~/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — PX4 drone0 with camera
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

### Terminal 3 — (optional) verify topics
```bash
ros2 topic list | grep -E "(vehicle_status|local_position|camera)"
```
Expected:
```
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v1
```

### Terminal 4 — Camera bridge
```bash
ros2 run drone_node camera_bridge_simple
```

### Terminal 5 — Fog server
**Start the fog BEFORE the task publisher** so it's already subscribed when tasks arrive.
```bash
ros2 run fog_node fog_server
```

Expected:
```
[FOG] drone0: status=/fmu/out/vehicle_status_v1, task=/drone0/task/fog, camera=/drone0/camera/image
```

Note `task=/drone0/task/fog` — the new tier-specific topic.

### Terminal 6 — Drone task publisher (normal operation)
```bash
ros2 run drone_node drone_task_publisher --ros-args -p drone_id:=drone0
```

Expected startup:
```
[DRONE TASK PUB] drone0: simulate_low_battery=False
[DRONE TASK PUB] drone0: PX4 status from /fmu/out/vehicle_status_v1
[DRONE TASK PUB] drone0: PX4 position from /fmu/out/vehicle_local_position_v1
[DRONE TASK PUB] drone0: camera frames from /drone0/camera/image
[DRONE TASK PUB] drone0: tasks for tier "local" -> /drone0/task/local
[DRONE TASK PUB] drone0: tasks for tier "fog"   -> /drone0/task/fog
[DRONE TASK PUB] drone0: tasks for tier "cloud" -> /drone0/task/cloud
```

Then continuous output:
```
[DRONE TASK PUB] drone0: published drone0-detect-0000 (VICTIM_DETECTION_REQUEST, frame_seq=1) -> fog
[DRONE TASK PUB] drone0: published drone0-status-0000 (STATUS_REPORT, priority=0) -> fog
[DRONE TASK PUB] drone0: published drone0-detect-0001 (VICTIM_DETECTION_REQUEST, frame_seq=2) -> fog
```

Every 10 seconds:
```
[FILTER STATS] drone0: in=20 passed=18 (90.0%) dropped=2 [dark=0 bright=0 static=2 blur=0]
```

### Terminal 6 (alternative) — Demo the dying-drone override
Stop the publisher (Ctrl+C) and restart with the simulation flag:
```bash
ros2 run drone_node drone_task_publisher --ros-args \
    -p drone_id:=drone0 \
    -p simulate_low_battery:=true
```

Expected (note the WARN level):
```
[WARN] [DRONE TASK PUB] drone0: published drone0-status-0000
    (STATUS_REPORT, priority=3, DRONE_FAILING) -> fog
```

In T5 (fog), the priority-3 status appears with `drone_failing: true` in its payload. The reaction logic (drone reallocation) is Task 5.

---

## 8. Verification checklist

With `simulate_low_battery=False`:

- [ ] Publisher logs show `-> fog` on every task line
- [ ] `[FILTER STATS]` lines appear every 10 s with non-zero `in` and `passed` counts
- [ ] `ros2 topic list | grep /drone0/task` shows all three tier topics
- [ ] `ros2 topic hz /drone0/task/fog` shows ~3 Hz (one detection per ~2 frames, plus 1 Hz status)
- [ ] `ros2 topic hz /drone0/task/local` shows no messages (correct — Task 3.6)
- [ ] `ros2 topic hz /drone0/task/cloud` shows no messages (correct — Task 3.8)
- [ ] `ros2 topic echo /drone0/task/fog --field task_type` shows BOTH `VICTIM_DETECTION_REQUEST` and `STATUS_REPORT` interleaved
- [ ] Inspecting one full task message (`ros2 topic echo /drone0/task/fog --once --full-length`) shows `position` with `valid: true` and real X/Y/Z values
- [ ] Detection task payload includes `filter_scores` with all three keys (`brightness`, `diff`, `blur`)
- [ ] Position values approximately match the spawn pose `(18, 25, 0.5)`

With `simulate_low_battery:=true`:

- [ ] Status logs come through as `[WARN]` with `DRONE_FAILING` text
- [ ] Fog logs show `priority=3` on status tasks
- [ ] Status task payload contains `"drone_failing": true`
- [ ] Detection tasks stay at priority 0 (no over-escalation)
- [ ] Status still goes to fog tier (no routing change)

If all are green, Task 3.4 is sealed.

---

## 9. Tuning notes and lessons learned

### The DIFF_MIN threshold

Initial tuning of `DIFF_MIN = 2.0` was based on the expectation of natural sensor noise between frames. In Gazebo's simulated camera with a stationary drone, this assumption fails completely — consecutive frames are nearly identical, and **99.9% of frames were dropped as "static"** in our first test:

```
[FILTER STATS] drone0: in=860 passed=1 (0.1%) dropped=859 [dark=0 bright=0 static=859 blur=0]
```

The fix was to lower the threshold to `DIFF_MIN = 0.1`. At this value the filter still drops bit-identical frames (which would indicate a frozen camera) but passes everything else.

In a deployed system where the drone is moving over a search area, the threshold can be raised back to 1.0 – 5.0 because natural drone motion generates substantial inter-frame variation. The threshold is one of those values that should be a launch parameter in a future revision.

### Camera resolution

The OakD-Lite model defaults to 640×480 at 30 Hz. We throttle this to 2 Hz in the bridge (Task 3.3) and apply filtering at that rate, so each frame gets ~7 ms of CPU spent on filtering (brightness + diff + blur). This is comfortable for a single drone on a laptop. For multi-drone operation, additional optimization may be needed.

### Position validity timing

PX4's EKF takes ~10 seconds after boot to converge. During this period, `position.valid == false` and the position dict has `x, y, z == None`. Tasks generated in this window still flow correctly but with invalid positions. The fog should treat invalid positions as "drone position unknown" rather than dropping the task.

---

## 10. Troubleshooting

### `[FILTER STATS]` shows `passed=0` (all frames dropped)

Almost certainly `DIFF_MIN` is too aggressive for your test scenario. If the drone is stationary, lower `DIFF_MIN` to `0.1`:

```python
DIFF_MIN = 0.1
```

Rebuild `drone_node` and restart the publisher.

### `The message type 'task_msgs/msg/Task' is invalid`

The terminal you're running `ros2 topic echo` / `ros2 topic hz` from didn't source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Rule: every new terminal needs **both** `source /opt/ros/humble/setup.bash` AND `source ~/ros2_ws/install/setup.bash`.

### `ros2 topic echo` truncates the payload

Use `--full-length`:

```bash
ros2 topic echo /drone0/task/fog --once --full-length
```

Or pipe to a file:

```bash
ros2 topic echo /drone0/task/fog --once --full-length > /tmp/task.yaml
cat /tmp/task.yaml
```

### `position` is always `{"valid": false, ...}`

The EKF hasn't converged yet, or it failed to converge. Wait 10–20 seconds after PX4 boot. If it never goes valid, your PX4 simulation parameters need review (mag/GPS/baro checks may be active).

### `ros2 topic echo /drone0/task/fog --field task_type` shows only one task type

If only `STATUS_REPORT` appears — the filter is dropping every camera frame. Check the `[FILTER STATS]` line in the publisher. See the `DIFF_MIN` note above.

If only `VICTIM_DETECTION_REQUEST` appears — the 1 Hz status timer isn't firing. Check that the status callback is receiving PX4 messages (`latest_status` shouldn't stay `None`).

### `ImportError: No module named cv2` when the publisher starts

Install OpenCV:
```bash
sudo apt install python3-opencv
```

---

## 11. What's intentionally NOT done yet

These belong to subsequent sub-tasks and are deliberately deferred.

- **Local-tier processing (Task 3.6)** — `/drone0/task/local` has a publisher but no consumer. Local-tier tasks like `BATTERY_CHECK` will be added in Task 3.6 along with the drone-side processor that handles them.
- **Cloud node (Task 3.8)** — `/drone0/task/cloud` has a publisher but no consumer. The cloud node will simulate WAN latency and archive logs/metrics/detection records. No live processing in the cloud (per Section 4.9 of the design).
- **Victim detection model (Task 4)** — the fog still just counts camera frames and parses task metadata. The actual CV inference (YOLO / MobileNet) goes into the fog camera callback in Task 4.
- **Network-condition override** — the current decision module assumes the fog is always reachable. Network-condition awareness is deferred until we have a meaningful network signal to act on.
- **Real battery integration** — `simulate_low_battery` is a manual flag. Wiring it to a real PX4 battery threshold is deferred until we have a realistic battery discharge model.
- **Victim map** — the position data is now in every task, but the fog doesn't yet aggregate detections into a spatial map. That's Task 5.
- **Multi-drone simultaneous operation** — the code supports drones 0, 1, 2 with no changes (`-p drone_id:=drone1` etc.). We tested drone0 only to keep the laptop stable during development. Multi-drone runs work but stress the machine more.

---

## Quick reference — file map

| File | Purpose | Modified in 3.4? |
|---|---|---|
| `src/task_msgs/msg/Task.msg` | Message schema (unchanged) | No |
| `src/drone_node/drone_node/camera_bridge_simple.py` | Gazebo→ROS2 RGB bridge (unchanged) | No |
| `src/drone_node/drone_node/drone_task_publisher.py` | Task generator with filter, position, decision | **Yes** |
| `src/fog_node/fog_node/fog_server.py` | Subscribes to `/drone0/task/fog` instead of `/drone0/task` | **One line** |

---

*Last updated: Task 3.4 — offloading decision module with frame filter and position tracking. Next: Task 3.5 (integrate decision into autonomous drone behavior) or Task 3.6 (local-tier processing).*
