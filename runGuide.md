# RUN GUIDE — Task 6 complete (4 scenarios, detection + timing + battery)

Fog-enabled UAV swarm, three-tier disaster response. This one document runs the
whole evaluation: **four scenarios**, **victim detection**, **latency / response /
completion** logging, and **battery-vs-time** logging — with every node as its
own terminal so nothing is silently skipped.
 
| # | Scenario | `mode` | `fault` | fog detection | tier detector |
|---|----------|--------|---------|---------------|---------------|
| **A** | Fog (proposed 3-tier) | `fog` | `none` | ON | — (fog detects) |
| **B** | Fog fails → Cloud | `cloud` | `fog_down` | OFF | `cloud_detector` ×3 |
| **C** | Fog+Cloud fail → Local | `local` | `fog_cloud_down` | OFF | `victim_detector` ×3 |
| **D** | One drone fails → repartition | `fog` | `drone_down` | ON | — (fog detects) |
 
Run order per scenario: **Part 0** (once) → **Part 1** → **Part 2** → the
scenario section → **Part 8 (analyze)**.

================================================================
# PART 0 —  build (once)
================================================================

0.1 Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select task_msgs drone_node fog_node cloud_node
source ~/ros2_ws/install/setup.bash
```
 
### 0.2 VERIFY executables exist

```bash
ros2 pkg executables drone_node | grep -E "cloud_detector|victim_detector|battery_simulator|drone_commander|camera_bridge_simple"
ros2 pkg executables fog_node   | grep -E "fog_server|decision_node"
```
All must be listed. Missing `cloud_detector` → Scenario B detects nobody.
 
> Re-source `install/setup.bash` in EVERY terminal after EVERY build.

================================================================
# PART 1 — Pre-flight reset (before EVERY run)
================================================================
```bash
pkill -9 px4 ruby gz MicroXRCEAgent python3 2>/dev/null; sleep 2
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/{0,1,2}/parameters*.bson 2>/dev/null
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson 2>/dev/null
pgrep -a px4; pgrep -a gz; pgrep -a MicroXRCEAgent   # should print nothing
```

================================================================
# PART 2 — Common bring-up (Terminals 1–10, identical for every scenario)
================================================================


**T1 — agent**
```bash
MicroXRCEAgent udp4 -p 8888
```
**T2 - QGroundControl**
```bash
# Launch the AppImage you downloaded (typical location):
~/QGroundControl.AppImage
# (first time only: chmod +x ~/QGroundControl.AppImage)
```

**T3 — Gazebo server** (wait 10–20 s to load)
```bash
cd ~/PX4-Autopilot
source build/px4_sitl_default/rootfs/gz_env.sh
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
gz sim -s -r Tools/simulation/gz/worlds/baylands_collapsed_fixed.sdf
```
**T4 — Gazebo GUI**
```bash
gz sim -g
```

**T5 / T6 / T7 -PX4**

```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=0 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```
- drone 1: `PX4_INSTANCE=1 … PX4_GZ_MODEL_POSE="23,25,0.5,0,0,0" … -i 1`
- drone 2: `PX4_INSTANCE=2 … PX4_GZ_MODEL_POSE="30,25,0.5,0,0,0" … -i 2`

---

**T8 / T9 / T10 — Camera bridges (drone 0 / 1 / 2)**
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=0
```
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=1
```
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node camera_bridge_simple --ros-args -p instance:=2
```

**T11 / T12 / T13 — Commanders (drone 0 / 1 / 2)**
These use **stop-and-go**
coverage (hover briefly at each capture point) for more reliable detection —
see the note below the blocks for the tradeoff and how to switch to a
continuous sweep instead.
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node drone_commander --ros-args -p instance:=0 \
  -p lane_spacing:=13.0 -p waypoint_radius:=4.0 \
  -p stop_and_go:=true -p hover_sec:=2.0 -p divert_resume_sec:=12.0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node drone_commander --ros-args -p instance:=1 \
  -p lane_spacing:=13.0 -p waypoint_radius:=4.0 \
  -p stop_and_go:=true -p hover_sec:=2.0 -p divert_resume_sec:=12.0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node drone_commander --ros-args -p instance:=2 \
  -p lane_spacing:=13.0 -p waypoint_radius:=4.0 \
  -p stop_and_go:=true -p hover_sec:=2.0 -p divert_resume_sec:=12.0
```
**stop_and_go vs continuous sweep (which to use):**
> `stop_and_go:=true` densifies each lawnmower lane into a grid of capture
> points (spaced by `capture_spacing`, default 10 m — sent by the fog in
> START_MISSION) and **hovers `hover_sec` at each one**. The drone climbs and
> descends into the search cell only ONCE at the start; it does *not* climb/
> descend between every point — it dwells at scan altitude and pauses at each
> capture point. This gives more camera frames per point, so victims are caught
> more reliably (helpful given detections are sparse), at the cost of a slower
> sweep and more battery per unit area. For a faster, lower-battery run that
> favours coverage speed over dwell time, just drop the last two flags
> (`stop_and_go` defaults to `false` = continuous sweep). Both fully cover the
> area; pick based on whether you're optimising for detection reliability
> (stop-and-go) or coverage speed / battery (continuous).


================================================================
# SCENARIO A — Fog (proposed 3-tier)
================================================================

### T14 — fog_server (detection ON)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node fog_server --ros-args \
  -p num_drones:=3 -p enable_detection:=true \
  -p coverage_target_pct:=0.0 -p auto_finish_coverage_pct:=95.0 \
  -p area_min_x:=-80.0 -p area_max_x:=100.0 -p area_min_y:=-40.0 -p area_max_y:=75.0 \
  -p transit_altitude:=35.0 -p scan_altitude:=15.0 -p camera_hfov_deg:=60.0 \
  -p spawns_x:="[18.0, 23.0, 30.0]" -p spawns_y:="[25.0, 25.0, 25.0]"
```
### T15 — decision_node
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node decision_node --ros-args -p num_drones:=3
```
### T16 / T17 / T18 — battery simulators (drone 0 / 1 / 2)
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone2
```
### T19 — metrics collector  (latency + response + completion)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/metrics_collector.py --ros-args \
  -p mode:=fog -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=none
```
### T20 — battery logger  (battery vs time)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/battery_logger.py --ros-args \
  -p mode:=fog -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=none
```
### Start the mission 
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger
```

================================================================
# SCENARIO B — Fog fails → Cloud   (runs the CLOUD DETECTOR explicitly)
================================================================
Do Part 1 + Part 2 first.
 
### T14 — fog_server 
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node fog_server --ros-args \
  -p num_drones:=3 -p enable_detection:=false \
  -p coverage_target_pct:=0.0 -p auto_finish_coverage_pct:=95.0 \
  -p area_min_x:=-80.0 -p area_max_x:=100.0 -p area_min_y:=-40.0 -p area_max_y:=75.0 \
  -p transit_altitude:=35.0 -p scan_altitude:=15.0 -p camera_hfov_deg:=60.0 \
  -p spawns_x:="[18.0, 23.0, 30.0]" -p spawns_y:="[25.0, 25.0, 25.0]"
```
### T15 — decision_node
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node decision_node --ros-args -p num_drones:=3
```
### T16 / T17 / T18 — CLOUD DETECTORS (drone 0 / 1 / 2)
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node cloud_detector --ros-args -p instance:=0 -r __node:=cloud_detector_0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node cloud_detector --ros-args -p instance:=1 -r __node:=cloud_detector_1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node cloud_detector --ros-args -p instance:=2 -r __node:=cloud_detector_2
```
### T19 / T20 / T21 — battery simulators (drone 0 / 1 / 2)
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone2
```
### T22 — collector  (fault:=fog_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/metrics_collector.py --ros-args \
  -p mode:=cloud -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=fog_down
```
### T23 — battery logger  (fault:=fog_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/battery_logger.py --ros-args \
  -p mode:=cloud -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=fog_down
```
### Start the mission 
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger
```
### VERIFY the cloud detectors are alive (every cloud run)
```bash
ros2 node list | grep -c cloud_detector     # MUST print 3
ros2 topic hz /drone0/cloud/detection        # should tick
ros2 topic echo /fog/victim_alerts           # processed_at:"cloud" after 1-3 s WAN
```
Each cloud-detector terminal should log `persons=1` over the victim (not always
`persons=0`). `NO camera frames yet ...` = that drone's bridge (T5–T7) is down.


================================================================
# SCENARIO C — Fog+Cloud fail → Local   (runs the LOCAL DETECTOR explicitly)
================================================================
Do Part 1 + Part 2 first.
 
### T14 — fog_server
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node fog_server --ros-args \
  -p num_drones:=3 -p enable_detection:=false \
  -p coverage_target_pct:=0.0 -p auto_finish_coverage_pct:=95.0 \
  -p area_min_x:=-80.0 -p area_max_x:=100.0 -p area_min_y:=-40.0 -p area_max_y:=75.0 \
  -p transit_altitude:=35.0 -p scan_altitude:=15.0 -p camera_hfov_deg:=60.0 \
  -p spawns_x:="[18.0, 23.0, 30.0]" -p spawns_y:="[25.0, 25.0, 25.0]" \
  -p archive_to_cloud:=false
```
> `archive_to_cloud:=false` so this local (fog+cloud-down) run reports
> `tiers_active: ["edge"]` — cloud is unavailable here, so it must not
> archive (UPDATE 7). Scenario B (cloud) keeps the default true.

### T15 — decision_node
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node decision_node --ros-args -p num_drones:=3
```
### T16 / T17 / T18 — LOCAL (on-drone) DETECTORS (drone 0 / 1 / 2)
`imgsz:=192` on all three keeps the 3-drone local run from CPU-starving on this
machine (see UPDATE 4-C / UPDATE 6). If a `[LOCAL RATE] ... CPU-STARVED` warning
still appears or a drone never logs `[DETECTION] ... person(s) detected` when it
passes a victim, that drone is still too loaded — close other apps, or accept
sparser local detections and pool more reps.
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node victim_detector --ros-args -p instance:=0 -p imgsz:=192 -r __node:=victim_detector_0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node victim_detector --ros-args -p instance:=1 -p imgsz:=192 -r __node:=victim_detector_1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node victim_detector --ros-args -p instance:=2 -p imgsz:=192 -r __node:=victim_detector_2
```
### T19 / T20 / T21 — battery simulators (drone 0 / 1 / 2)
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone2
```
### T22 — collector  (fault:=fog_cloud_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/metrics_collector.py --ros-args \
  -p mode:=local -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=fog_cloud_down
```
### T23 — battery logger  (fault:=fog_cloud_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/battery_logger.py --ros-args \
  -p mode:=local -p scenario:=medium -p run_id:=run_01 -p num_drones:=3 -p fault:=fog_cloud_down
```
### Start the mission 
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger
```
### VERIFY
```bash
ros2 node list | grep -c victim_detector     # MUST print 3
ros2 topic echo /fog/victim_alerts            # processed_at:"local"
```

================================================================
# SCENARIO D — One drone fails → repartition
================================================================
Proposed fog system; drone1 made unreachable at T+90 s; the fog repartitions the
area among the survivors and the mission still completes. Do Part 1 + Part 2.
Fog detects; no tier detector.
 
### T14 — fog_server (detection ON + injected failure)
Deterministic single failure: the scheduled failure of drone1 at T+90 s, with
the telemetry watchdog OFF so machine load can't inject a second failure (see
UPDATE 4-D). This is the reliable way to run Scenario D on this laptop.

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node fog_server --ros-args \
  -p num_drones:=3 -p enable_detection:=true \
  -p coverage_target_pct:=0.0 -p auto_finish_coverage_pct:=95.0 \
  -p area_min_x:=-80.0 -p area_max_x:=100.0 -p area_min_y:=-40.0 -p area_max_y:=75.0 \
  -p transit_altitude:=35.0 -p scan_altitude:=15.0 -p camera_hfov_deg:=60.0 \
  -p spawns_x:="[18.0, 23.0, 30.0]" -p spawns_y:="[25.0, 25.0, 25.0]" \
  -p fail_drone_id:=1 -p fail_after_sec:=90.0 -p repartition_on_failure:=true \
  -p heartbeat_timeout_sec:=0.0
```
> To demonstrate the *realistic* "drone goes silent" path instead (the
> `pkill px4 -i 1` option below), swap the last two lines for
> `-p fail_drone_id:=-1 -p repartition_on_failure:=true` and
> `-p heartbeat_timeout_sec:=5.0 -p heartbeat_grace_sec:=20.0`. Only the drone
> you kill will fail — the new last-survivor guard and post-dispatch grace
> window prevent a starved survivor from being culled too.
### T15 — decision_node
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run fog_node decision_node --ros-args -p num_drones:=3
```
### T16 / T17 / T18 — battery simulators (drone 0 / 1 / 2)
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone0
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone1
```
```bash

source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run drone_node battery_simulator --ros-args -p drone_id:=drone2
```
### Start the mission (REQUIRED — the failure clock starts here)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger
```
### T19 — collector  (fault:=drone_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/metrics_collector.py --ros-args \
  -p mode:=fog -p scenario:=medium -p run_id:=drone_fail_01 -p num_drones:=3 -p fault:=drone_down
```
### T20 — battery logger  (fault:=drone_down)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/battery_logger.py --ros-args \
  -p mode:=fog -p scenario:=medium -p run_id:=drone_fail_01 -p num_drones:=3 -p fault:=drone_down
```
### Optional — trigger a REAL failure instead of the scheduled one
```bash
pkill -9 -f "bin/px4 -i 1"     # drone1 goes silent → fog repartitions in ~5 s
```
### VERIFY
At ~T+90 s fog_server prints `DRONE FAILURE: drone1 ... Survivors: ['drone0','drone2']`
then a `[FOG REPARTITION]` block, then:
```bash
ros2 topic echo /fog/reliability     # DRONE_FAILED then REPARTITION events
```
drone1 returns; drone0/drone2 expand their sweep; coverage still reaches 95 % and
the mission finishes. Run a baseline Scenario A (fault=none) too for the graph.
 

================================================================
# PART 4 — End a run + analyze
================================================================
**Ending:** at 95 % mean coverage the fog auto-finishes (RTL all + report + cloud
flush) and the collector saves + exits. Then Ctrl-C the battery logger, then the
rest. If a run stalls below 95 % (e.g. a rescuer is holding over a victim), end it
manually — this still writes the summary:
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /fog/end_mission std_srvs/srv/Trigger
```
Files per run in `evaluation/results_real/`:
`<mode>_<scenario>_<run_id>.csv`, `.summary.json`, `_battery.csv`.
 
**Repetitions (≥10):** repeat Parts 1→2→scenario with `run_id:=run_02 … run_10`
(change it in the collector + battery_logger together).
 
**Analyze:**
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 evaluation/analyze_results.py
```
Plots → `evaluation/plots/`: g1 latency, g2 completion, g3 response, g4 grouped
time, g5 energy, g6 coverage, **g7 battery-over-time**, g8 cloud WAN/inference,
g9 scalability, g10 reliability. 

































