# How to Run — Dynamic Area Partitioning & Coverage

This guide walks through running the area-coverage mission: the **fog** node
partitions a search area into one cell per drone and each **commander** flies
its drone up, out to its cell, and sweeps it.

- **Number of drones is dynamic** — set `num_drones` on the fog and launch
  that many PX4 instances. The example below uses 3.
- **Spawn locations are not hardcoded.** They are fog parameters
  (`spawns_x`, `spawns_y`) that default to the 3-drone setup below; override
  them to match wherever you actually spawn the drones.

---

## 0. One-time build

After copying `fog_server.py` into `fog_node` and `drone_commander.py` into
`drone_node`:

```bash
cd ~/ros2_ws
colcon build --packages-select drone_node fog_node --symlink-install
```

**In every terminal you open below, source first:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

(PX4 / Gazebo terminals don't need the ROS workspace, but it does no harm.)

---

## 1. Run procedure (3 drones)

Open one terminal per step.

### T1 — micro-ROS agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### T2 — QGroundControl (optional, clears the GCS preflight warning)
```bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

### T3 — Gazebo server
```bash
cd ~/PX4-Autopilot
source build/px4_sitl_default/rootfs/gz_env.sh
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
gz sim -s -r Tools/simulation/gz/worlds/baylands_collapsed_fixed.sdf
```

### T4 — Gazebo GUI
```bash
gz sim -g
```

### T5, T6, T7 — PX4 instances (one per drone)

**Drone 0** (spawn 18, 25):
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=0 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="18,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

**Drone 1** (spawn 23, 25):
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="23,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1
```

**Drone 2** (spawn 30, 25):
```bash
cd ~/PX4-Autopilot
PX4_INSTANCE=2 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_WORLD=baylands_collapsed_fixed PX4_GZ_MODEL_POSE="30,25,0.5,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 2
```

> **The fog's default `spawns_x`/`spawns_y` match these exact poses
> (18/23/30 at y=25).** If you change a `PX4_GZ_MODEL_POSE`, update the fog
> params to match (see Section 2) — otherwise that drone will fly to the
> wrong place, and the commander will print a spawn-mismatch warning.

### In each `pxh>` prompt (once it shows "Ready for takeoff")

Lets SITL arm without a strict GPS/GCS gate. Set once; `param save` persists it.
```
param set COM_ARM_WO_GPS 1
param set COM_PREARM_MODE 0
param set COM_DISARM_PRFLT 0
param set FD_IMB_PROP_THR 0
param save
```
`FD_IMB_PROP_THR 0` disables PX4's "imbalanced propeller" failure detector,
which fires spuriously on the SITL `gz_x500` during normal flight and can kick
the drone out of OFFBOARD.

### T8 — Fog server
```bash
ros2 run fog_node fog_server --ros-args -p num_drones:=3
```
Expect `[FOG] tracking 3 drone(s)` and three `[FOG] droneN ...` lines.

### T9, T10, T11 — Commanders (one per drone)
```bash
ros2 run drone_node drone_commander --ros-args -p instance:=0   # T9
ros2 run drone_node drone_commander --ros-args -p instance:=1   # T10
ros2 run drone_node drone_commander --ros-args -p instance:=2   # T11
```
Each prints its PX4 input prefix: `/fmu/in` for drone 0, `/px4_1/fmu/in` and
`/px4_2/fmu/in` for the others.

### T12 — Start the mission
Only after all three drones' EKFs report `local position: 1`:
```bash
ros2 service call /fog/start_mission std_srvs/srv/Trigger {}
```

---

## 2. Configuring spawn locations (fog parameters)

Spawn locations live on the fog as two equal-length lists, in **world ENU**
(x = East, y = North), one entry per drone, in instance order. They default to:

```
spawns_x = [18.0, 23.0, 30.0]
spawns_y = [25.0, 25.0, 25.0]
```

To use different spawns, pass them when launching the fog — and use the
**same** values in each drone's `PX4_GZ_MODEL_POSE`:

```bash
ros2 run fog_node fog_server --ros-args \
  -p num_drones:=3 \
  -p spawns_x:="[18.0, 23.0, 30.0]" \
  -p spawns_y:="[25.0, 25.0, 25.0]"
```

If a drone has no spawn entry (fewer entries than `num_drones`), that drone
auto-calibrates its spawn from PX4's reported global position instead.

---

## 3. Changing the number of drones

The partition adapts automatically (recursive bisection → equal-area,
gap-free cells for any count). To run N drones:

1. Launch N PX4 instances (T5…), each with a unique `PX4_INSTANCE`, `-i`, and
   `PX4_GZ_MODEL_POSE`.
2. Give the fog N spawns and `num_drones:=N`:
   ```bash
   ros2 run fog_node fog_server --ros-args \
     -p num_drones:=4 \
     -p spawns_x:="[18.0, 24.0, 30.0, 36.0]" \
     -p spawns_y:="[25.0, 25.0, 25.0, 25.0]"
   ```
3. Launch N commanders (`instance:=0 … N-1`). They can be backgrounded from
   one terminal:
   ```bash
   for i in 0 1 2 3; do
     ros2 run drone_node drone_commander --ros-args -p instance:=$i &
   done
   ```

> On this laptop, ~4 drones is the comfortable ceiling; 6+ pushes Gazebo's
> real-time factor down and PX4 starts emitting `Accel #0 TIMEOUT`.

---

## 4. Configuring the search area

The area to cover is a rectangle in world ENU, default `X[-80,100] Y[-40,75]`.
Override per run:
```bash
ros2 run fog_node fog_server --ros-args -p num_drones:=3 \
  -p area_min_x:=-80.0 -p area_max_x:=100.0 \
  -p area_min_y:=-40.0 -p area_max_y:=75.0 \
  -p scan_altitude:=12.0
```
Make sure these bounds frame the part of `baylands_collapsed_fixed` you want
searched.

---

## 5. Commander knobs

| Parameter | Default | Effect |
|---|---|---|
| `do_scan` | true | `false` = fly to the cell centroid and just hold (good for a first sanity check before enabling full sweeps) |
| `lane_spacing` | 15.0 | Metres between sweep lanes — smaller = denser, more thorough coverage, longer flight |
| `waypoint_radius` | 3.0 | How close counts as "reached" a waypoint (m) |
| `max_step` | 25.0 | Max distance the commanded setpoint leads the drone by (m). Smaller = gentler, slower flight; reduces the hard rolls that trip PX4's attitude-failure check |
| `default_alt` | 12.0 | Altitude used only if a command omits one |

The sweep **loops continuously** — each drone keeps re-scanning its cell until
you end the mission (Section 6b), at which point it returns to launch.

Example — verify clean flight first, no sweeping:
```bash
ros2 run drone_node drone_commander --ros-args -p instance:=0 -p do_scan:=false
```

---

## 6. Expected output

**Fog (T8), after `start_mission`:**
```
[FOG START_MISSION] Partitioning area X[-80.0,100.0] Y[-40.0,75.0] into 3 cell(s).
[FOG START_MISSION] drone0 -> cell 0 centroid=(-50.0, 17.5) area X[-80.0,-20.0] Y[-40.0,75.0] alt=12.0m spawn=(18.0, 25.0)
[FOG START_MISSION] drone1 -> cell 1 centroid=(10.0, 17.5)  area X[-20.0,40.0]  Y[-40.0,75.0] alt=12.0m spawn=(23.0, 25.0)
[FOG START_MISSION] drone2 -> cell 2 centroid=(70.0, 17.5)  area X[40.0,100.0]  Y[-40.0,75.0] alt=12.0m spawn=(30.0, 25.0)
```

**Each commander:**
```
[COMMANDER] droneN: spawn from fog -> world ENU=(...)
[COMMANDER] droneN: START_MISSION alt=12.0m, 8 waypoint(s), first=ENU(...) -> CLIMB then TRANSIT (scan loops until RTL).
[COMMANDER] droneN: setting OFFBOARD mode
[COMMANDER] droneN: arming
[COMMANDER] droneN: reached 12.0m, TRANSIT to cell.
[COMMANDER] droneN: arrived at cell, SCAN (8 waypoints, looping until RTL).
[COMMANDER] droneN: scan pass 1 complete, restarting sweep.
[COMMANDER] droneN: scan pass 2 complete, restarting sweep.
...
```

The three drones climb straight up, fan out west / center / east, and keep
sweeping their strips. The whole area is covered with no gaps, and the sweep
repeats until you end the mission.

## 6b. Ending the mission

When you want the drones to stop scanning and come home:
```bash
ros2 service call /fog/end_mission std_srvs/srv/Trigger {}
```
The fog sends every drone an RTL (return to launch) and flushes its buffered
mission events to the cloud. Each commander prints:
```
[COMMANDER] droneN: RTL received — returning to launch
```
and PX4 flies that drone back to its takeoff point and lands.

> To stop and hover in place instead of returning home, change the `RTL`
> command in the commander's `mission_command_callback` to hold the current
> position — but RTL is the standard "mission complete" behaviour.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `WARN [commander] command 192 unsupported` (in a PX4 terminal) | An **old** commander build is still running. Rebuild, then kill stale nodes (`pkill -9 -f drone_commander`) and relaunch all commanders in freshly sourced terminals. The current commander never sends command 192. |
| Drone arms but won't take off | Make sure the commander prints `setting OFFBOARD mode` then `arming`. If not, it's the stale-build issue above. |
| Drone flies to the wrong place; commander warns about spawn mismatch | The fog's `spawns_x`/`spawns_y` entry for that drone doesn't match its real `PX4_GZ_MODEL_POSE`. Make them equal. |
| Commander stuck after climbing, "waiting for a spawn" | The fog didn't send a spawn for that drone (fewer spawn entries than drones) **and** PX4 hasn't reported a global fix yet. Add the spawn to the fog params, or wait for GPS lock. |
| `The message type 'task_msgs/msg/Task' is invalid` on `ros2 topic echo` | The terminal didn't source `~/ros2_ws/install/setup.bash`. (Nodes are unaffected — this only hits CLI tools.) |
| PX4 won't arm | Run the `param set COM_ARM_WO_GPS 1` block in that drone's `pxh>` and `param save`. |
| `Attitude failure (roll)` / drone goes unstable and stops following commands | PX4's failure detector tripped and dropped OFFBOARD. The commander now auto-recovers OFFBOARD and flies with capped (gentle) setpoints, but if it still happens: raise `scan_altitude` so the drones clear collapsed-building geometry (try 20–30 m), lower `max_step` (e.g. `-p max_step:=15.0`) for gentler motion, and set `FD_IMB_PROP_THR 0` in each `pxh>`. |
| `Imbalanced propeller detected` | SITL nuisance from the `gz_x500` model; set `param set FD_IMB_PROP_THR 0` in each `pxh>` (already in the param block above). |
| Drones reach the cell then drift / stop with no target | You're on the old single-pass build. Rebuild — the current commander loops the sweep until `end_mission`. |
