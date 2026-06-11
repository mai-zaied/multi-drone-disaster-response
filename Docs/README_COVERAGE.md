# Coverage Subsystem — Fog-Coordinated Multi-Drone Area Search

**Project:** Fog-Enabled UAV Swarm System for Low-Latency Disaster Response
**Course:** Birzeit University ENCS5300 Graduation Project, 2025–2026
**Team:** Lina Abureesh (1211985), Mai Beitnoba (1210260), Doaa Hatu (1211088)
**Supervisor:** Dr. Ibrahim Nemer

This document describes the **area-coverage subsystem**: the fog node divides a
search area among an arbitrary number of drones, each drone flies to and scans
its assigned cell, and the fog proves — from the camera footage itself — that
the whole area was captured.

---

## Table of contents
1. [Design decisions](#1-design-decisions)
2. [System architecture](#2-system-architecture)
3. [What we built](#3-what-we-built)
4. [How partitioning works](#4-how-partitioning-works)
5. [How coverage works](#5-how-coverage-works)
6. [The flight profile](#6-the-flight-profile)
7. [Build instructions](#7-build-instructions)
8. [Run instructions](#8-run-instructions)
9. [Parameters reference](#9-parameters-reference)
10. [Outputs and proof](#10-outputs-and-proof)
11. [Known limitations and future work](#11-known-limitations-and-future-work)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Design decisions

These are the choices that shaped the subsystem, with the reasoning behind each
(the points worth defending in the viva).

1. **The fog is the coordinator; drones are executors.** The fog has the global
   view — the search area, every drone's position, and the camera model — so it
   owns partitioning, assignment, coverage bookkeeping, and the
   mission-complete decision. Drones only fly the cell they are given. This
   keeps the swarm logic in one place and the drone code simple.

2. **"Covered" means *captured by the camera*, not *flown over*.** A point is
   covered the instant it falls inside a camera footprint. Coverage is the
   union of footprints over the flight, not the flight path. This matches the
   real mission (you need to *see* every point) and means a single pass with a
   wide footprint can cover a strip without overflying every metre of it.

3. **Coverage is computed at the fog from camera frames + PX4 position.** The
   fog already receives each drone's camera stream and subscribes to its
   position, so it can project each frame's footprint onto a grid with no extra
   drone-side work. Coverage is a global/coordination concern, so it belongs at
   the fog, not split across drones.

4. **Partition by recursive bisection, not a fixed grid.** A near-square grid
   (`ceil(sqrt(n))` columns) leaves empty cells whenever `n` is not a tidy grid
   number (3 drones → a 2×2 grid with one cell unscanned). Recursive bisection
   produces exactly `n` gap-free, area-balanced cells for **any** `n`.

5. **Everything is parameterised — nothing is hardcoded to a world.** Area
   bounds, drone count, altitudes, camera FOV, and spawn locations are all fog
   parameters. The same code runs on any rectangle with any number of drones.

6. **Two-altitude flight: transit high, scan low.** Crossing the map happens at
   a high obstacle-safe altitude; the low, detection-grade scanning is confined
   to the assigned cell. This avoids trees/buildings on the way without
   sacrificing image resolution where it matters.

7. **Offboard streaming, never `DO_REPOSITION`.** The target PX4 SITL build
   rejects `MAV_CMD_DO_REPOSITION` (command 192). The robust method is to stream
   `OffboardControlMode` + `TrajectorySetpoint` continuously, switch to OFFBOARD,
   then arm — with vertical takeoff first and OFFBOARD auto-recovery if a
   transient failsafe drops it.

8. **Footprint-based lane and capture spacing (standard survey practice).**
   Lane spacing (sidelap) and capture spacing (frontlap) are derived from the
   camera footprint at scan altitude, with a ~20% overlap margin so there are no
   gaps. Continuous and stop-and-go scan styles are both supported.

9. **Scan-until-covered, then auto-complete.** Each drone loops its sweep until
   its cell crosses a coverage threshold (default 98%), then returns home;
   when the last cell completes, the fog auto-saves the report and flushes the
   mission log. No manual end call needed (one is still available as an abort).

10. **Stable contracts so nothing downstream breaks.** The `Task` message, the
    `/fog/{drone}/decision` string, and all topic/service names are unchanged
    from the base system, so the task publisher, cloud server, reactor, and
    detection nodes keep working.

---

## 2. System architecture

Three-tier fog architecture. The coverage subsystem lives in the **fog**
(coordination) and **drone** (execution) tiers.

```
                      /fog/start_mission (std_srvs/Trigger)
                                    │
                                    ▼
        ┌──────────────────── FOG SERVER ─────────────────────┐
        │  • partition area -> one cell per drone              │
        │  • send each drone its cell, spawn, both altitudes,  │
        │    and the capture spacing                           │
        │  • subscribe to each drone's camera + PX4 position   │
        │  • mark footprint coverage per frame                 │
        │  • when a cell is covered -> RTL that drone          │
        │  • when all covered -> report + cloud flush          │
        └───────┬───────────────────────────────┬─────────────┘
   /droneN/mission_command            /fog/{droneN}/decision
                │                                 ▲
                ▼                                 │
        ┌──────────────┐   OffboardControlMode +  │ VehicleStatus,
        │  COMMANDER N │   TrajectorySetpoint @10Hz│ VehicleLocalPosition
        │ CLIMB→TRANSIT│ ───────────────────────► │ (PX4 outputs)
        │ →DESCEND→SCAN│                          │
        └──────┬───────┘                          │
               ▼                                  │
           PX4 SITL N ──────────────────────────► fog (status/pos)
               │
               ▼  (Gazebo camera topic)
        ┌──────────────┐   /droneN/camera/image (sensor_msgs/Image)
        │ CAMERA BRIDGE│ ────────────────────────► fog (coverage frames)
        │  N  (gz→ROS) │                            + rqt_image_view (you)
        └──────────────┘
```

**Data flow for coverage:** Gazebo renders the downward camera → camera bridge
republishes it as a ROS2 image at 2 Hz → fog's `camera_callback` fires → fog
reads that drone's latest PX4 position → projects the footprint → marks the
grid → updates the coverage %.

---

## 3. What we built

| File | Package | Role |
|---|---|---|
| `fog_server.py` | `fog_node` | Partitioning, per-drone assignment, coverage grid + report, mission lifecycle, cloud archival |
| `drone_commander.py` | `drone_node` | Offboard flight executor: phased CLIMB/TRANSIT/DESCEND/SCAN, lawnmower sweep, stop-and-go, RTL |
| `camera_bridge_simple.py` | `drone_node` | Bridges the Gazebo camera topic to `/{drone}/camera/image` at 2 Hz |
| `drone_naming.py` | both | Single source of truth mapping `instance` → drone_id / model / topic names |

New capabilities added on top of the base task/fog/cloud system:

- **Recursive-bisection partitioner** (`partition_area`) and per-drone assignment.
- **`CoverageGrid`** class — a numpy-free occupancy grid with footprint marking,
  point queries, and percentage.
- **Camera-driven coverage tracking** in the fog (subscribe to position +
  camera; mark per frame).
- **Two-altitude flight profile** with a `DESCEND` phase in the commander.
- **Footprint-based lane/capture spacing**, computed by the fog and sent to the
  commander.
- **Stop-and-go scan mode** (hover + capture at each footprint-spaced point).
- **Scan-until-covered lifecycle**: per-drone RTL on completion, automatic
  mission end, coverage report written to disk.
- **Optional zone mode** for scanning boxes around named disaster sites.

---

## 4. How partitioning works

The fog is given a search rectangle and `num_drones`. It splits the rectangle
into exactly `num_drones` cells using **recursive bisection**:

1. If one drone remains, it gets the whole (sub)rectangle.
2. Otherwise, split the drones into two groups of `floor(n/2)` and the rest.
3. Cut the rectangle's **longer side** proportionally to each group's drone
   count, so areas stay balanced and cells stay near-square.
4. Recurse on each half.

```python
def partition_area(min_x, max_x, min_y, max_y, n):
    if n <= 1:
        return [(min_x, max_x, min_y, max_y)]
    a, b = n // 2, n - n // 2
    if (max_x - min_x) >= (max_y - min_y):
        cut = min_x + (max_x - min_x) * (a / n)
        return (partition_area(min_x, cut, min_y, max_y, a)
                + partition_area(cut, max_x, min_y, max_y, b))
    else:
        cut = min_y + (max_y - min_y) * (a / n)
        return (partition_area(min_x, max_x, min_y, cut, a)
                + partition_area(min_x, max_x, cut, max_y, b))
```

**Properties (verified for n = 1..8):** the cells exactly tile the rectangle —
their areas sum to the whole area, with zero overlap — and every drone's cell
has equal area. Examples on the default area `X[-80,100] Y[-40,75]`:

| n | Layout |
|---|---|
| 1 | whole area |
| 2 | two halves along the longer axis |
| 3 | three vertical strips (centroids −50, 10, 70 at y=17.5) |
| 4 | 2×2 |
| 6 | 3×2 |

Each drone receives its cell rectangle plus its centroid, spawn, both
altitudes, and the capture spacing inside a single JSON `START_MISSION` command.

**Coordinate frames.** The fog works in **world ENU** (x=East, y=North, origin =
Gazebo world origin). PX4's setpoints are in the drone's **local NED** (origin =
its spawn). The conversion uses the spawn the fog supplies:
`N = world_y − spawn_y`, `E = world_x − spawn_x`, `D = −altitude`.

**Zone mode (optional, `use_zones:=true`).** Instead of one rectangle, the fog
takes a list of disaster-site coordinates and builds a square scan box around
each; drones are assigned to zones round-robin, and a zone shared by several
drones is subdivided with the same bisection. Default is off.

---

## 5. How coverage works

**Footprint.** For a downward camera at altitude `h` with horizontal FOV
`HFOV`, the ground footprint is `W = 2·h·tan(HFOV/2)` wide and
`L = 2·h·tan(VFOV/2)` long.

**Grid.** Each cell is discretised into `coverage_cell_m`-sized bins (default
2 m). The fog keeps one `CoverageGrid` per drone.

**Marking.** Every camera frame the fog receives for a drone triggers a mark: it
converts the drone's PX4 local-NED position to world ENU (using the spawn),
computes the footprint at the drone's **actual** altitude, and flips every bin
inside that footprint rectangle to "covered."

**Percentage.** `coverage % = covered bins / total bins in the cell`. It is
reported live every 5 s as `[FOG COVERAGE] droneN=NN% … | mean=NN%`, and a final
per-cell + overall report with an ASCII map is printed and written to disk on
completion.

**Why the metric also tunes the flight.** For gap-free coverage the lane spacing
must be ≤ the swath `W` (with overlap). If lanes are too far apart, footprints
don't overlap and the coverage % **plateaus below 100** — so the number itself
diagnoses whether the spacing is small enough. The fog prints the recommended
`lane_spacing` and `capture_spacing` at start so you can set the commander
correctly.

**Spacing follows standard survey practice.** Lane spacing = `W·(1 − overlap)`
(sidelap) and capture spacing = `L·(1 − overlap)` (frontlap), with a ~20%
overlap margin to absorb position error and camera tilt — gap-free without the
heavy 70–80% overlap that photo-stitching (which this mission does not need)
would require.

---

## 6. The flight profile

Each commander runs a phase machine, streaming offboard setpoints at 10 Hz:

| Phase | Action | Altitude | Exits when |
|---|---|---|---|
| `CLIMB` | Straight up over the spawn | → transit | at transit altitude |
| `TRANSIT` | Fly to the cell's first corner | transit (high, obstacle-safe) | within `waypoint_radius` |
| `DESCEND` | Sink over the cell | transit → scan | at scan altitude |
| `SCAN` | Boustrophedon ("lawnmower") sweep, looping | scan (low, detection) | RTL on cell complete |
| `HOLD` | Loiter (single-target / `do_scan=false` only) | scan | — |

**Robustness.** Vertical takeoff first (so PX4 doesn't roll hard off the
ground); lead-capped setpoints (the commanded point never sits more than
`max_step` ahead) to keep accelerations gentle; OFFBOARD auto-recovery if a
transient failsafe drops the mode.

**Scan styles.**
- **Continuous** (default): fly the lanes; the camera streams while moving —
  faster.
- **Stop-and-go** (`stop_and_go:=true`): the lawnmower is densified into a grid
  of footprint-spaced points (using the fog's `capture_spacing`); the drone
  hovers `hover_sec` at each so every frame is sharp, level, and truly nadir —
  better for detection with a body-fixed camera, at the cost of time.

**Lifecycle.** A drone loops its sweep until its cell reaches
`coverage_target_pct`; the fog then RTLs that drone. When all cells are done the
fog auto-completes the mission (report + cloud flush). The drones return to
launch under PX4's own RTL (which climbs to a safe return altitude).

---

## 7. Build instructions

Prerequisites (already set up on the project machine): Ubuntu 22.04, ROS2
Humble, Gazebo Harmonic, PX4-Autopilot SITL, MicroXRCEAgent, OpenCV, the
`task_msgs` / `drone_node` / `fog_node` / `cloud_node` workspace packages.

```bash
cd ~/ros2_ws
colcon build --packages-select drone_node fog_node --symlink-install
```

Source in **every** terminal you open:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**One-time world setup for valid coverage:** the camera must point **down**. In
`~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf`, give the
OakD-Lite a 90° pitch in both the include pose and the joint pose:
```xml
<pose>.12 .03 .242 0 1.5708 0</pose>
```
Verify in `rqt_image_view` that the feed shows the ground, not the horizon.

---

## 8. Run instructions

> A clean restart matters: a **leftover Gazebo server** from a previous run
> desyncs PX4's lockstep clock and causes endless `Accel/Gyro/Baro TIMEOUT`
> errors. Always kill all of px4 + gz + agent together between runs.

**Pre-run cleanup**
```bash
pkill -9 -f px4 ; pkill -9 -f 'gz sim' ; pkill -9 -f gz-sim
pkill -9 -f ruby ; pkill -9 -f MicroXRCEAgent
pkill -9 -f drone_commander ; pkill -9 -f fog_server ; pkill -9 -f camera_bridge
ps aux | grep -E 'px4|gz sim|gz-sim|MicroXRCE' | grep -v grep   # must be empty
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/{0,1,2}/parameters*.bson
```

**Launch order** (one terminal each; source ROS + workspace in every one):

1. **Agent:** `MicroXRCEAgent udp4 -p 8888`
2. **Gazebo server** (wait 10–20 s; needs `gz_env.sh`):
   ```bash
   cd ~/PX4-Autopilot
   source build/px4_sitl_default/rootfs/gz_env.sh
   __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
   gz sim -s -r Tools/simulation/gz/worlds/baylands_collapsed_fixed.sdf
   ```
3. **Gazebo GUI:** `gz sim -g` — confirm the clock is advancing (RTF > 0).
4. **PX4 ×3** (poses are yours; keep the fog's `spawns_x/y` matching them):
   ```bash
   cd ~/PX4-Autopilot
   PX4_INSTANCE=0 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
   PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
   ./build/px4_sitl_default/bin/px4 -i 0
   ```
   (drone 1 → `…"23,25,0.5,0,0,0" -i 1`; drone 2 → `…"30,25,0.5,0,0,0" -i 2`)
   In each `pxh>` after "Ready for takeoff":
   ```
   param set COM_ARM_WO_GPS 1
   param set COM_PREARM_MODE 0
   param set COM_DISARM_PRFLT 0
   param set FD_IMB_PROP_THR 0
   param save
   ```
5. **Camera bridges ×3** (REQUIRED — no frames, no coverage):
   ```bash
   ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0   # :=1, :=2
   ```
   Each should show non-zero `gz_received`. View a feed any time:
   `ros2 run rqt_image_view rqt_image_view` → pick `/droneN/camera/image`.
6. **Fog** (the two inputs are area + num_drones):
   ```bash
   ros2 run fog_node fog_server --ros-args \
     -p num_drones:=3 \
     -p area_min_x:=-80.0 -p area_max_x:=100.0 \
     -p area_min_y:=-40.0 -p area_max_y:=75.0 \
     -p transit_altitude:=35.0 -p scan_altitude:=18.0 \
     -p camera_hfov_deg:=60.0 \
     -p spawns_x:="[18.0, 23.0, 30.0]" -p spawns_y:="[25.0, 25.0, 25.0]"
   ```
   Note the `lane_spacing <= …` and `capture_spacing=…` it prints.
7. **Commanders ×3** — set lane spacing to the fog's recommendation:
   ```bash
   # continuous (smoother flight)
   ros2 run drone_node drone_commander --ros-args -p instance:=0 \
     -p lane_spacing:=16.0 -p max_step:=12.0 -p waypoint_radius:=6.0
   # OR stop-and-go (sharper frames)
   ros2 run drone_node drone_commander --ros-args -p instance:=0 \
     -p lane_spacing:=16.0 -p waypoint_radius:=3.0 -p stop_and_go:=true -p hover_sec:=2.0
   ```
   (repeat with `instance:=1`, `instance:=2`)
8. **Start the mission** (after all EKFs show `local position: 1`):
   ```bash
   ros2 service call /fog/start_mission std_srvs/srv/Trigger {}
   ```

The mission ends itself when all cells are covered. To abort early:
`ros2 service call /fog/end_mission std_srvs/srv/Trigger {}`.

---

## 9. Parameters reference

### Fog (`fog_node fog_server`)
| Parameter | Default | Meaning |
|---|---|---|
| `num_drones` | 3 | Drones to partition for (any N) |
| `area_min_x/max_x/min_y/max_y` | -80/100/-40/75 | Search rectangle (world ENU, m) |
| `transit_altitude` | 35.0 | High obstacle-safe crossing altitude (m) |
| `scan_altitude` | 18.0 | Low scan/detection altitude (m) |
| `camera_hfov_deg` | 60.0 | **Verify vs model SDF** — drives footprint math |
| `camera_vfov_deg` | 0.0 | 0 = derive from HFOV (4:3 sensor) |
| `coverage_cell_m` | 2.0 | Coverage grid bin size (m) |
| `coverage_overlap` | 0.2 | Sidelap/frontlap margin for spacing recommendations |
| `coverage_target_pct` | 98.0 | "Fully covered" threshold; 100 = strict, 0 = loop until manual end |
| `spawns_x` / `spawns_y` | [18,23,30]/[25,25,25] | Per-drone spawn (world ENU), matching `PX4_GZ_MODEL_POSE` |
| `use_zones` | false | Scan boxes around disaster sites instead of one rectangle |
| `zones_x` / `zones_y` / `zone_half_size` | buildings / 25 | Zone-mode parameters |

### Commander (`drone_node drone_commander`)
| Parameter | Default | Meaning |
|---|---|---|
| `instance` | 0 | PX4 instance index (derives all topic names) |
| `do_scan` | true | true = sweep the cell; false = fly to centroid and hold |
| `lane_spacing` | 15.0 | Distance between sweep lanes (m); set to fog recommendation |
| `waypoint_radius` | 3.0 | Arrival threshold (m); larger rounds corners (smoother) |
| `max_step` | 25.0 | Setpoint lead cap (m); smaller = gentler/steadier flight |
| `stop_and_go` | false | Hover + capture at each footprint-spaced grid point |
| `hover_sec` | 2.0 | Hover time per capture point in stop-and-go |
| `default_alt` | 12.0 | Fallback altitude if a command omits one |

### Camera bridge (`drone_node camera_bridge_simple`)
| Parameter | Default | Meaning |
|---|---|---|
| `instance` | 0 | PX4 instance index |
| `publish_hz` | 2.0 | ROS republish rate of the Gazebo camera |

---

## 10. Outputs and proof

- **Live:** `[FOG COVERAGE] drone0=NN% drone1=NN% … | mean=NN%` every 5 s; a
  ✓ marks each cell as it completes.
- **Per-drone completion:** `[FOG COVERAGE] droneN cell FULLY COVERED … —
  sending RTL`.
- **Final report:** printed and written to `/tmp/fog_coverage_<timestamp>.txt`,
  containing per-cell and overall coverage % and an ASCII map
  (`#` captured, `.` assigned-but-not-yet, blank outside cells). Read the latest:
  ```bash
  ls -t /tmp/fog_coverage_*.txt | head -1 | xargs cat
  ```
- **Cloud archive:** mission events flushed to `/fog/cloud/mission_log` and
  written by the cloud node.

A representative run on the default area reaches ~99% overall, with the map
showing a fully filled rectangle except a thin bottom seam (the lawnmower's 2 m
edge inset) — which is exactly why the default threshold is 98%, not 100%.

---

## 11. Known limitations and future work

- **No reactive obstacle avoidance.** Obstacles are avoided by geometry (fly the
  transit at a safe altitude; scan only where obstacles are known/short), not by
  sensing. Future work: feed the depth camera into PX4 collision prevention, or
  use it during the descent as a clearance check.
- **The descent happens over the cell's first corner with no obstacle check** —
  fine for clear corners, but a tall obstacle exactly there would be a hazard.
- **Body-fixed camera couples image quality to flight smoothness.** A moving
  multirotor tilts to translate, so continuous-mode frames are slightly
  off-nadir, and stop-and-go (which fixes that) makes the flight look rockier. A
  **gimbal** (the `x500_gimbal` model) decouples the two and is the recommended
  upgrade.
- **Detection altitude is resolution-limited.** Whether 18 m detects a person
  depends on the camera's render width (pixels-on-target). Verify the width and
  lower the altitude or use a larger YOLO model if targets are too small.
- **`num_drones` is set manually**, not auto-discovered from live PX4 instances.
- **Coverage assumes a downward camera and a flat-ish ground footprint** — valid
  here, but uneven terrain would need a terrain-aware footprint model.

---

## 12. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Endless `Accel/Gyro/Baro TIMEOUT` at PX4 start | Stale Gazebo server from a previous run. Kill ALL px4+gz+agent, restart the sim, confirm the GUI clock advances **before** launching PX4. |
| `f=0` for a drone in `[FOG STATS]` | Its camera bridge isn't feeding frames — restart it (with `gz_env.sh` sourced). Coverage can't progress without frames. |
| "Didn't descend to scan altitude" | Confirm the commander logs `DESCEND … -> …` then `at scan altitude …`. If missing, you're on an old build. Verify live with `ros2 topic echo /fmu/out/vehicle_local_position_v1 --field z`. |
| Coverage plateaus below target, drone scans forever | `lane_spacing` too wide, or wrong `camera_hfov_deg` — footprints don't overlap. Use the fog's recommended lane spacing. |
| Coverage map covers ground the camera never saw | Camera not pointing down — fix the model SDF pitch; verify in rqt_image_view. |
| Flight looks like it swings/rocks while moving | Normal for a multirotor (it tilts to translate). Reduce with `max_step:=12`, `waypoint_radius:=6`, continuous mode; or PX4 `MPC_TILTMAX_AIR`/`MPC_XY_VEL_MAX`. |
| `command 192 unsupported` | Stale commander build — rebuild, `pkill -9 -f drone_commander`, relaunch in sourced terminals. |
| Spawn-mismatch warning from a commander | Fog `spawns_x/y` entry ≠ that drone's `PX4_GZ_MODEL_POSE` — make them equal. |
| Drone won't arm | Run the `param set` block in that drone's `pxh>`. |
| `RTPS_TRANSPORT_SHM … open_and_lock_file failed` | Harmless stale shared-memory file; Fast-DDS falls back to UDP. Clear with `rm -f /dev/shm/fastrtps_* /tmp/fastrtps_*`. |
| `task_msgs/msg/Task is invalid` on `ros2 topic echo` | That terminal didn't source the workspace (CLI-only; nodes unaffected). |

---

*Coverage subsystem — area partitioning, two-altitude scanning, and
footprint-based coverage proof for the Fog-Enabled UAV Swarm.*
