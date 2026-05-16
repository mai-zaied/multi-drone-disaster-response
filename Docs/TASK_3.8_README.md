# Task 3.8 — Simulated Cloud Processing (End-of-Mission Archival)

This document covers everything added on top of Task 3.6 to complete Task 3.8 of the **Fog-Enabled UAV Swarm System for Low-Latency Disaster Response** project.

By the end of this sub-task:

- A new ROS2 package `cloud_node` exists and acts as a simulated cloud archive.
- The fog buffers events in memory during the mission, with no cloud traffic.
- An end-of-mission service flushes the buffer to the cloud in chunked batches.
- The cloud receives batches, applies a randomised simulated WAN delay (500 ms – 5 s) per batch using non-blocking ROS2 timers, and writes each batch to disk as a JSON file.

This document assumes the Task 3.6 README has already been read.

---

## Table of Contents

1. [What we built and why](#1-what-we-built-and-why)
2. [Architecture](#2-architecture)
3. [Repository changes](#3-repository-changes)
4. [Detailed explanation of each component](#4-detailed-explanation-of-each-component)
   - 4.1 The `cloud_node` package
   - 4.2 The end-of-mission service on the fog
   - 4.3 The fog-side event buffer
   - 4.4 The non-blocking delay pattern
   - 4.5 The on-disk archive format
5. [Prerequisites](#5-prerequisites)
6. [Build instructions](#6-build-instructions)
7. [Run instructions — quick standalone test](#7-run-instructions--quick-standalone-test)
8. [Run instructions — full 3-drone end-to-end](#8-run-instructions--full-3-drone-end-to-end)
9. [Inspecting the archive](#9-inspecting-the-archive)
10. [Verification checklist](#10-verification-checklist)
11. [Design rationale and lessons learned](#11-design-rationale-and-lessons-learned)
12. [Troubleshooting](#12-troubleshooting)
13. [What's intentionally NOT done yet](#13-whats-intentionally-not-done-yet)

---

## 1. What we built and why

### The problem

Up to Task 3.6 the system had two of the three architectural tiers wired up: drones produce tasks, the fog receives and parses them. The cloud tier existed only as a topic name and a slot in the design document. Nothing was archiving anything, and nothing was simulating the long latency cost of reaching a real cloud.

### What Task 3.8 adds

**A real cloud node.** A separate ROS2 package and node that subscribes to a cloud-bound topic, applies a simulated WAN delay, and writes received data to disk. It runs as a long-lived archive server.

**End-of-mission archival pattern.** The fog does **not** send anything to the cloud during the mission. Instead, every fog event (task arrivals, priority-3 alerts) is buffered in memory. When the mission ends, an explicit service call triggers the fog to flush the buffer to the cloud in chunked batches. This keeps the fog's CPU and network capacity focused on real-time work for the duration of the mission — exactly what Section 4.9 of the project design commits to.

**Non-blocking simulated WAN delay.** The cloud applies a delay using ROS2 one-shot timers, not `time.sleep()`. Multiple batches can be in flight concurrently with independent delays, and the node never blocks. This is the right pattern and we use it here instead of in Task 3.7 because the cloud is the first place where the delay magnitude matters enough to make blocking obviously wrong.

### Why this matters architecturally

The cloud is **not** a heavy-compute backend in this project. Following Section 4.9 of the design, the cloud's only role is **non-real-time data archiving**. Task 3.8 honours this by making the cloud literally just receive, delay, write to disk, and nothing else. There is no detection on the cloud, no aggregation, no analysis — those belong to the fog. The cloud is the historian, not the brain.

---

## 2. Architecture

```
                  DURING MISSION
                  ──────────────

  Drones ─tasks─► Fog ─decisions─► Drones
                   │
                   ▼
            Event buffer
            (in memory,
             bounded to 10000)
                   │
                   ▼
        /fog/cloud/mission_log
            ▲ (silent — zero traffic)
            │
           Cloud   (idle, ready to receive)


                  AT END OF MISSION
                  ─────────────────

  Operator ──ros2 service call /fog/end_mission──► Fog
                                                    │
                                  Fog chunks buffer │ into batches of 1000 events
                                                    │ and publishes them all
                                                    ▼
                              /fog/cloud/mission_log
                                                    │
                                                    ▼
                                                Cloud
                                                    │ Per batch:
                                                    │  1. Pick random delay [0.5, 5.0] s
                                                    │  2. Schedule one-shot timer
                                                    │  3. When timer fires, write to disk
                                                    │
                                                    ▼
                              /tmp/cloud_archive_<startup_ts>/
                                  batch_000000_recvNNN.json
                                  batch_000001_recvNNN.json
                                  ...
```

### Three architectural points worth highlighting

1. **The drone never talks to the cloud.** The fog is the only thing producing on the cloud topic. The drone has no business reaching across two tiers.

2. **The cloud topic carries zero traffic during the mission.** All `[CLOUD STATS]` lines show `received=0` until end_mission is called. This is the architectural principle made visible.

3. **The cloud handles multiple in-flight batches concurrently.** Because each batch's delay is implemented as a one-shot timer, the cloud can receive 5 batches simultaneously and archive them out-of-order (whichever delay fires first wins). This is realistic for a WAN with jitter.

---

## 3. Repository changes

| File | Action |
|---|---|
| `src/cloud_node/` | **NEW package** |
| `src/cloud_node/package.xml` | new |
| `src/cloud_node/setup.py` | new |
| `src/cloud_node/setup.cfg` | new |
| `src/cloud_node/resource/cloud_node` | new (empty marker file) |
| `src/cloud_node/cloud_node/__init__.py` | new (empty) |
| `src/cloud_node/cloud_node/cloud_server.py` | new |
| `src/fog_node/fog_node/fog_server.py` | modified — event buffer, end-of-mission service, batch chunking |
| `src/fog_node/package.xml` | modified — adds `<exec_depend>std_srvs</exec_depend>` |

No changes to `task_msgs`, `drone_node`, or any drone-side code.

---

## 4. Detailed explanation of each component

### 4.1 The `cloud_node` package

**What it is:** A new ROS2 Python (`ament_python`) package containing a single node, `cloud_server`. Mirrors the structure of `drone_node` and `fog_node`.

**The node's responsibilities:**

- Subscribe to `/fog/cloud/mission_log` for incoming batches.
- For each batch, pick a random WAN delay uniformly in `[delay_min_sec, delay_max_sec]`.
- Schedule a one-shot ROS2 timer with that delay.
- When the timer fires, write the batch + metadata to a JSON file in the session archive directory.
- Log statistics every 10 seconds.

**Parameters:**

| Parameter | Default | Purpose |
|---|---|---|
| `archive_dir` | `/tmp/cloud_archive_<startup_timestamp>` | Where to write archived batches |
| `delay_min_sec` | `0.5` | Lower bound for random WAN delay |
| `delay_max_sec` | `5.0` | Upper bound for random WAN delay |

**Why a separate package and not part of `fog_node`:**

The cloud is its own conceptual tier in the architecture. Having `drone_node`, `fog_node`, `cloud_node` as three distinct packages makes the three-tier story visible at the package level. The cloud could in principle run on a different machine (or container), and the package boundary is the natural place for that separation.

### 4.2 The end-of-mission service on the fog

**Service:** `/fog/end_mission`  
**Type:** `std_srvs/srv/Trigger`

The `std_srvs/Trigger` type is the standard ROS2 service for "do a thing, tell me if it worked." The request is empty (no arguments). The response contains:
- `bool success` — whether the flush completed
- `string message` — a human-readable summary

**Service implementation:**

When called, the fog:
1. Snapshots the current event buffer (`list(self.event_buffer)`).
2. Splits into chunks of 1000 events.
3. Builds a batch dict for each chunk with `batch_index`, `total_batches`, `event_count`, `fog_timestamp`, and the events.
4. Publishes each batch as a JSON-encoded `std_msgs/String` on `/fog/cloud/mission_log`.
5. Clears the buffer.
6. Returns a response with the total event count, batch count, and how many events were dropped during the mission due to buffer overflow.

**Why a service, not a topic:**

- A service gives a clean request/response semantics — the operator gets confirmation that the flush succeeded.
- Topics are fire-and-forget; a topic message could be missed if it arrives before the fog is ready.
- `ros2 service call` is a one-liner from the terminal, making the demo trivial.

**Calling it manually:**

```bash
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

**Calling it from a script** (e.g., the operator's mission control panel in a future iteration):

```python
from std_srvs.srv import Trigger
client = node.create_client(Trigger, '/fog/end_mission')
client.wait_for_service()
future = client.call_async(Trigger.Request())
```

### 4.3 The fog-side event buffer

**Implementation:** Python `collections.deque` with `maxlen=10000`.

Why `deque`:
- Bounded by construction. When `maxlen` is reached, the oldest items are silently evicted.
- `append()` is O(1) — no memory reallocation as events come in.
- Iterating into a list for batching is O(n) but only happens at end_mission, not on the hot path.

**What gets recorded as an event:**

Every task that arrives at the fog generates one `TASK_RECEIVED` event:
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
    "payload_keys": ["frame_seq", "frame_timestamp_sec", "filter_scores", "position", ...]
  }
}
```

Priority-3 (dying-drone) tasks additionally generate a `PRIORITY3_ALERT` event so they're easy to find in the archive:
```json
{
  "event_type": "PRIORITY3_ALERT",
  "drone_id": "drone0",
  "fog_received_at": 1778700200.123,
  "payload": {
    "task_id": "drone0-status-0017",
    "drone_failing": true,
    "position": {"valid": true, "x": 18.2, "y": 25.1, "z": -0.05}
  }
}
```

**Overflow tracking:**

If the buffer is at 10000 events and a new one arrives, the oldest is silently dropped. The fog tracks how many times this happened in `events_dropped_on_overflow`. This count is included in the end_mission response so the operator knows if any data was lost during a long mission.

For typical short missions (under ~30 minutes at current task rates) the buffer never overflows. Long-duration missions might; if that becomes a problem, raising the cap to 50,000 is a one-line change.

**Buffer status log:**

The fog logs a `[FOG BUFFER]` line whenever the buffer length changes meaningfully, alongside its 5-second stats:
```
[FOG BUFFER] 142 events buffered (soft_cap=10000, overflow_drops=0)
```

This makes it visible during a demo that the buffer is filling.

### 4.4 The non-blocking delay pattern

The naive approach would be `time.sleep(delay)` in the callback. That blocks the entire executor — no other callbacks run during the sleep, including `[CLOUD STATS]` and any new incoming batches.

Instead the cloud uses ROS2 one-shot timers:

```python
delay_sec = random.uniform(0.5, 5.0)
timer = self.create_timer(delay_sec, deferred_archival)
self._pending_timers[id(batch)] = timer
```

When the timer fires, `deferred_archival` runs, does the disk write, and removes its entry from `_pending_timers`.

**Why this is the right pattern:**

- The callback returns immediately after scheduling the timer.
- The executor is free to handle other batches in parallel.
- Multiple delays can be in flight at once with independent expiry times.
- ROS2 timer scheduling is handled by the same executor — no threading complexity.

**Demonstrating this in the demo:**

If the fog flushes 3 batches at once and the cloud picks delays of 4.2 s, 1.1 s, and 3.0 s, the cloud logs look like:

```
[CLOUD] Received batch 1/3 (1000 events). Scheduling archival in 4.21s.
[CLOUD] Received batch 2/3 (1000 events). Scheduling archival in 1.13s.
[CLOUD] Received batch 3/3 (456 events). Scheduling archival in 3.07s.
[CLOUD ARCHIVED] batch 2/3 (1000 events) after 1.13s delay -> ...
[CLOUD ARCHIVED] batch 3/3 (456 events) after 3.07s delay -> ...
[CLOUD ARCHIVED] batch 1/3 (1000 events) after 4.21s delay -> ...
```

Note batch 2 arrives first (shortest delay), then batch 3, then batch 1. Out-of-order archival is realistic for a degraded WAN connection.

### 4.5 The on-disk archive format

Each batch is written as a single JSON file in the archive directory.

**Filename convention:**

```
batch_<NNNNNN>_recv<RECV_MS>.json
```

- `NNNNNN` — zero-padded counter assigned in order of archival (so files sort chronologically by archival time)
- `RECV_MS` — millisecond timestamp of when the batch was received (so files are unique even if NNNNNN matches across sessions)

**File contents:**

```json
{
  "batch_index": 1,
  "total_batches": 3,
  "fog_timestamp": 1778700100.123,
  "cloud_received_at": 1778700100.125,
  "cloud_archived_at": 1778700104.347,
  "simulated_wan_delay_sec": 4.222,
  "event_count": 1000,
  "events": [
    {
      "event_type": "TASK_RECEIVED",
      "drone_id": "drone0",
      "fog_received_at": 1778700090.456,
      "payload": { ... }
    },
    ...
  ]
}
```

**The four timestamps form a measurable timeline:**

- `fog_timestamp` — when the fog created the batch
- `cloud_received_at` — when the cloud first saw the batch
- `cloud_archived_at` — when the cloud wrote it to disk
- `simulated_wan_delay_sec` — the delay applied

The difference `cloud_archived_at - cloud_received_at` equals `simulated_wan_delay_sec` to within milliseconds. This is the proof that the simulated delay actually happened. You can show this in the report.

The fog-to-cloud delivery time (`cloud_received_at - fog_timestamp`) reflects real ROS2 IPC, which is typically a few milliseconds on the same machine. In a distributed deployment this would be larger, but the simulated delay still dominates.

---

## 5. Prerequisites

Same as Task 3.6. No new system packages. The cloud_node uses only `rclpy` and `std_msgs`, both already present.

Just confirm `std_srvs` is available (it ships with `ros-humble-desktop` by default):

```bash
ros2 interface show std_srvs/srv/Trigger
```

Expected:
```
---
bool success
string message
```

---

## 6. Build instructions

From a fresh state:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cloud_node fog_node
source install/setup.bash
```

Both packages should finish without errors.

Verify the cloud_node entry point:

```bash
ros2 pkg executables cloud_node
```

Expected:
```
cloud_node cloud_server
```

If that doesn't show up, the install didn't link the entry point. Re-run `colcon build` and `source install/setup.bash`.

---

## 7. Run instructions — quick standalone test

Use this to confirm the fog/cloud integration works without launching the full drone stack.

### T1 — Fog (1 drone, so it doesn't complain)

```bash
rosws
ros2 run fog_node fog_server --ros-args -p num_drones:=1
```

You'll see drone0 subscriptions advertised but no PX4 status arriving — that's fine, we're just testing fog↔cloud.

### T2 — Cloud

```bash
rosws
ros2 run cloud_node cloud_server
```

**Expected first lines:**
```
[CLOUD] Started. archive_dir=/tmp/cloud_archive_20260516_142315, delay_range=[0.50s, 5.00s]
[CLOUD] Subscribed to /fog/cloud/mission_log
[CLOUD STATS] received=0 archived=0 events_archived=0 pending=0
```

**Note the archive directory** — write it down. That's where files will land.

### T3 — Trigger end_mission with empty buffer

```bash
rosws
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

**Expected:**
```
response:
std_srvs.srv.Trigger_Response(success=True, message='No events to archive.')
```

The fog and cloud both keep running (this is normal — see Section 12). No files appear in the archive directory.

This confirms the service is wired up correctly and handles the empty case gracefully. Kill T1 and T2 (Ctrl+C in each) and move on to the full test.

---

## 8. Run instructions — full 3-drone end-to-end

This builds on the Task 3.6 run procedure. Use Terminator with a tiling layout.

### Terminals 1–11: standard 3-drone bring-up

Follow Section 11 of the Task 3 README. After completion you should have:

- T1: MicroXRCEAgent
- T2, T3, T4: PX4 instances 0, 1, 2 (with uxrce_dds_client started)
- T5, T6, T7: Camera bridges for instances 0, 1, 2
- T8: Fog server with `num_drones:=3`
- T9, T10, T11: Task publishers for instances 0, 1, 2

### T12 — Cloud server

```bash
rosws
ros2 run cloud_node cloud_server
```

Watch this terminal. `[CLOUD STATS]` should appear every 10 seconds with `received=0` consistently. **This is the key architectural observation** — during the mission, no cloud traffic happens.

### Let the system run for 30–60 seconds

Watch T8 (fog) for `[FOG BUFFER]` lines showing buffer growth:
```
[FOG BUFFER] 56 events buffered (soft_cap=10000, overflow_drops=0)
[FOG BUFFER] 134 events buffered (soft_cap=10000, overflow_drops=0)
[FOG BUFFER] 220 events buffered (soft_cap=10000, overflow_drops=0)
```

Meanwhile T12 (cloud) stays silent — proving zero cloud traffic during the mission.

### Optional — trigger a dying drone for richer archive contents

In T9 (drone0 task publisher), Ctrl+C and restart with the simulate flag:
```bash
ros2 run drone_node drone_task_publisher --ros-args -p instance:=0 -p simulate_low_battery:=true
```

Now drone0's status reports come in at priority 3 and the fog records them as `PRIORITY3_ALERT` events. These will appear in the archive and prove the priority signal flows through to disk.

### T13 — End the mission

```bash
rosws
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```

**Expected response:**
```
response:
std_srvs.srv.Trigger_Response(success=True, message='Flushed 220 events in 1 batch(es). Dropped during mission due to overflow: 0.')
```

**Expected in T8 (fog):**
```
[FOG END_MISSION] Flushing 220 events in 1 batch(es) to cloud.
[FOG END_MISSION] Published batch 1/1 (220 events).
```

**Expected in T12 (cloud) within ~5 seconds:**
```
[CLOUD] Received batch 1/1 (220 events). Scheduling archival in 2.73s.
[CLOUD ARCHIVED] batch 1/1 (220 events) after 2.73s delay -> /tmp/cloud_archive_.../batch_000000_recvNNN.json
[CLOUD STATS] received=1 archived=1 events_archived=220 pending=0
```

### Multi-batch demo

For a more dramatic demo, let the system run longer (3–5 minutes) so the buffer accumulates 2000+ events. When you trigger end_mission, the fog splits into multiple chunks of 1000:

```
[FOG END_MISSION] Flushing 2456 events in 3 batch(es) to cloud.
[FOG END_MISSION] Published batch 1/3 (1000 events).
[FOG END_MISSION] Published batch 2/3 (1000 events).
[FOG END_MISSION] Published batch 3/3 (456 events).
```

And the cloud receives all three at once but archives them out-of-order based on random delays:

```
[CLOUD] Received batch 1/3 (1000 events). Scheduling archival in 4.21s.
[CLOUD] Received batch 2/3 (1000 events). Scheduling archival in 1.13s.
[CLOUD] Received batch 3/3 (456 events). Scheduling archival in 3.07s.
[CLOUD ARCHIVED] batch 2/3 (1000 events) after 1.13s delay -> ...
[CLOUD ARCHIVED] batch 3/3 (456 events) after 3.07s delay -> ...
[CLOUD ARCHIVED] batch 1/3 (1000 events) after 4.21s delay -> ...
```

**The out-of-order archival is the headline visual for Task 3.8.** It demonstrates:
- Real concurrent batch handling
- Realistic WAN jitter
- The non-blocking delay pattern in action

---

## 9. Inspecting the archive

After end_mission completes, the archive directory contains one JSON file per batch.

```bash
ls /tmp/cloud_archive_*/
```

**Expected:**
```
batch_000000_recv1778700104221.json
batch_000001_recv1778700102351.json
batch_000002_recv1778700106887.json
```

Note that filename order (by counter) matches archival order, NOT batch_index. This is intentional — files sort chronologically by when they hit disk.

### Open a file

```bash
cat /tmp/cloud_archive_*/batch_000000_*.json | python3 -m json.tool | head -40
```

You should see the four timestamps, the simulated delay, the event count, and the first events from the batch.

### Count events by type

```bash
cat /tmp/cloud_archive_*/batch_*.json | python3 -c "
import json, sys
counts = {}
for line in sys.stdin:
    pass  # just a quick demo
"
```

For something more useful:
```bash
python3 <<'EOF'
import json, glob, collections
counts = collections.Counter()
drones = collections.Counter()
for path in sorted(glob.glob('/tmp/cloud_archive_*/batch_*.json')):
    with open(path) as f:
        batch = json.load(f)
    for ev in batch['events']:
        counts[ev['event_type']] += 1
        drones[ev['drone_id']] += 1
print('Events by type:', dict(counts))
print('Events by drone:', dict(drones))
EOF
```

**Expected output:**
```
Events by type: {'TASK_RECEIVED': 220, 'PRIORITY3_ALERT': 6}
Events by drone: {'drone0': 80, 'drone1': 73, 'drone2': 73}
```

This is real archive analysis — exactly what the cloud is supposed to enable for post-mission review.

### Verify the delay was actually applied

```bash
python3 <<'EOF'
import json, glob
for path in sorted(glob.glob('/tmp/cloud_archive_*/batch_*.json')):
    with open(path) as f:
        b = json.load(f)
    measured = b['cloud_archived_at'] - b['cloud_received_at']
    print(f"Batch {b['batch_index']}/{b['total_batches']}: "
          f"sim_delay={b['simulated_wan_delay_sec']:.3f}s, "
          f"measured={measured:.3f}s, "
          f"events={b['event_count']}")
EOF
```

**Expected:**
```
Batch 1/3: sim_delay=4.221s, measured=4.222s, events=1000
Batch 2/3: sim_delay=1.131s, measured=1.131s, events=1000
Batch 3/3: sim_delay=3.073s, measured=3.073s, events=456
```

`simulated_wan_delay_sec` matches `measured` to within milliseconds. The cloud is genuinely delaying.

---

## 10. Verification checklist

- [ ] `ros2 pkg executables cloud_node` lists `cloud_server`
- [ ] `ros2 interface show std_srvs/srv/Trigger` returns the expected fields
- [ ] Cloud node starts and prints its archive directory path
- [ ] During the mission, cloud `[CLOUD STATS]` shows `received=0` consistently (zero traffic)
- [ ] Fog `[FOG BUFFER]` shows event count growing during the mission
- [ ] `ros2 service list | grep end_mission` shows `/fog/end_mission`
- [ ] Calling end_mission with empty buffer returns success=True and "No events to archive"
- [ ] Calling end_mission with N>0 events returns success=True with N reported correctly
- [ ] Fog logs `[FOG END_MISSION] Published batch X/Y` lines
- [ ] Cloud logs `[CLOUD] Received batch X/Y` immediately for all batches
- [ ] Cloud logs `[CLOUD ARCHIVED]` lines after their respective delays
- [ ] When multiple batches are sent at once, archival order does NOT match send order (random delays are working)
- [ ] JSON files exist in the archive directory after end_mission
- [ ] Each JSON file contains real events with `task_id`, `task_type`, `drone_id` etc.
- [ ] `simulated_wan_delay_sec` matches the measured `cloud_archived_at - cloud_received_at`
- [ ] Fog and cloud nodes keep running after end_mission completes (no exit)

---

## 11. Design rationale and lessons learned

### Why end-of-mission and not periodic batching

The original Task 3.8 plan was for the fog to periodically batch events (every 10 s) and ship them to cloud continuously. This was rejected in favour of end-of-mission archival because:

1. **Architectural principle.** Section 4.9 of the project design specifies the cloud as exclusively non-real-time. Periodic batching during the mission is real-time-adjacent and uses bandwidth that could be spent on detection/coordination.

2. **CPU focus.** During the mission, every cycle of fog compute should be available for real-time work. Serialising and publishing batches mid-mission steals cycles.

3. **Operationally clearer.** "Press a button when the mission ends" is unambiguous. Periodic uploads raise questions like "what if the upload is in flight when something critical happens?"

The trade-off is buffer size: a long mission needs more memory to hold events. The 10,000-event cap covers ~3 hours of operation at current task rates per drone, which is more than enough for a graduation demo.

### Why `std_msgs/String` with JSON, not a typed message

Mission log events are heterogeneous: a `TASK_RECEIVED` event has different fields from a `PRIORITY3_ALERT` event. A typed ROS2 message would either need every possible field (most empty) or one message per event type (lots of files). The same JSON-in-String pattern used for `Task.payload` is the cleanest answer.

### Why one-shot timers and not a thread

`time.sleep()` would block. A background thread for delays would require thread-safety on the archive counters. ROS2 timers run on the same executor as the callbacks, so no synchronisation is needed. The pattern scales naturally to many concurrent delays.

### Why a service and not a topic for end-of-mission

A topic message could be missed if it arrives at the wrong moment, and there's no acknowledgement. A service gives a clean request/response with a confirmation message including the number of events flushed and dropped. From the operator's perspective, calling a service is a one-line `ros2 service call`.

### Writing to disk vs in-memory archive

Disk is more useful for the demo and for the report. After the run, you can show actual JSON files with real data. An in-memory archive would require another service to retrieve it, and the data would be lost when the cloud node restarts.

---

## 12. Troubleshooting

### Cloud node starts but never receives batches

The fog isn't publishing on `/fog/cloud/mission_log` until end_mission is called. Check:

```bash
ros2 topic list | grep mission_log
```

You should see `/fog/cloud/mission_log`. If not, the fog isn't running or wasn't rebuilt after the Task 3.8 changes.

### Service call hangs

```bash
ros2 service list | grep end_mission
```

If `/fog/end_mission` doesn't appear, the fog isn't running or wasn't rebuilt. Re-run `colcon build --packages-select fog_node` and source `install/setup.bash`.

### `[CLOUD STATS] received=N archived=0` and stays there

A bug in the timer scheduling would look like this — batches received but never archived. In practice this hasn't happened in testing. If you see it, check the cloud node's terminal for Python exceptions; the most likely cause is disk write failure (permissions on the archive directory).

### Disk write fails

If the archive directory is unwritable, the cloud logs an error per failed batch:
```
[CLOUD] Failed to write /tmp/.../batch_000000_recvNNN.json: [Errno X] ...
```

Fix by giving the cloud a writable directory:
```bash
ros2 run cloud_node cloud_server --ros-args -p archive_dir:=$HOME/cloud_archive
```

### Fog and cloud "kept working" after end_mission

This is expected. Both nodes are designed to run for the entire system uptime. The end_mission service is a one-shot action that the fog handles inline and then continues spinning. To stop a node, Ctrl+C in its terminal.

### Buffer overflow during mission

If the fog reports `Dropped during mission due to overflow: N` with N > 0, the mission exceeded the 10,000-event buffer cap. Two options:

1. Increase the cap by editing `EVENT_BUFFER_SOFT_CAP` in `fog_server.py` and rebuilding.
2. Trigger end_mission more frequently (e.g., between mission phases).

For a typical 5-minute demo this is never a concern.

---

## 13. What's intentionally NOT done yet

- **Detection record archival.** The fog currently archives `TASK_RECEIVED` events (which include detection-request metadata) but not actual confirmed detections. Detection records become possible only after Task 4 adds a real CV model that produces "victim found" results worth archiving.
- **Aggregated map archival.** The fog doesn't yet build a swarm-wide map. That's Task 3.7 (swarm aggregation) plus Task 4 (detections to plot on the map) plus a small wire-up to the cloud.
- **Performance metrics archival.** Task 3.13 will produce performance reports. The cloud archive will be extended to handle a separate metrics topic at that time.
- **Cloud download / retrieval.** The cloud writes to disk but offers no way to read archives back via ROS2. Operators inspect files directly with `cat`, `jq`, or Python. A retrieval service could be added if needed; for now, file-on-disk is the simplest deliverable.
- **Network failure simulation.** The current delay is just a delay — every batch eventually arrives. Real WAN behaviour includes packet loss and duplicate delivery. Modeling these is out of scope for Task 3.8.
- **Compression.** Mission log batches are uncompressed JSON. For a real deployment we'd gzip before sending and decompress at the cloud. For the demo, plain JSON is more readable.

---

## Quick reference — file map

| File | Purpose | Status |
|---|---|---|
| `src/cloud_node/package.xml` | Package manifest | New |
| `src/cloud_node/setup.py` | Build config | New |
| `src/cloud_node/setup.cfg` | Build config | New |
| `src/cloud_node/resource/cloud_node` | ROS2 resource marker | New |
| `src/cloud_node/cloud_node/__init__.py` | Python package marker | New |
| `src/cloud_node/cloud_node/cloud_server.py` | Cloud archival node | New |
| `src/fog_node/fog_node/fog_server.py` | Adds buffer + service + batching | Modified |
| `src/fog_node/package.xml` | Adds `<exec_depend>std_srvs</exec_depend>` | Modified |

---

*Last updated: end of Task 3.8 — simulated cloud archival with end-of-mission flush. Next: Task 3.7 — fog-side swarm status aggregation.*
