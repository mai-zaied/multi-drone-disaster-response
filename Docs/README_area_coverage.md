# Area Partitioning & Coverage Subsystem

**Project:** Fog-Enabled UAV Swarm System for Low-Latency Disaster Response
**This document covers:** how the fog node divides a search area among an
arbitrary number of drones, and how each drone flies to and sweeps its
assigned cell.

This is the mission-control layer that sits on top of the existing
three-tier (drone / fog / cloud) system. It uses two nodes:

| Node | File | Package | Role |
|---|---|---|---|
| Fog server | `fog_server.py` | `fog_node` | Partitions the area, sends each drone its cell |
| Drone commander | `drone_commander.py` | `drone_node` | Flies the drone to its cell and sweeps it |

Everything scales with a single number — `num_drones` on the fog. There is
**no per-drone configuration**: the same launch command works for drone 0,
drone 7, or drone 20.

---

## 1. End-to-end flow

```
                /fog/start_mission (std_srvs/Trigger)
                            |
                            v
   +------------------- FOG SERVER -------------------+
   |  partition the search rectangle into N cells     |
   |  (recursive bisection, one cell per drone)       |
   |  for each drone: publish START_MISSION with its  |
   |  cell rectangle + centroid + altitude            |
   +--------------------------------------------------+
        |  /drone0/mission_command   |  /droneN/mission_command
        v                            v
   +--------------+             +--------------+
   |  COMMANDER 0 |    ...      |  COMMANDER N |
   |  CLIMB ->    |             |  CLIMB ->    |
   |  TRANSIT ->  |             |  TRANSIT ->  |
   |  SCAN ->     |             |  SCAN ->     |
   |  HOLD        |             |  HOLD        |
   +------+-------+             +------+-------+
          |  OffboardControlMode + TrajectorySetpoint (10 Hz)
          v                            v
        PX4 instance 0      ...      PX4 instance N
```

The fog does the *thinking* (where each drone should go). The commander does
the *flying* (closing the loop with PX4). They communicate with one JSON
message type on a per-drone topic.

---

## 2. The partition: recursive bisection

The fog splits the rectangular search area into exactly one cell per drone
using **recursive bisection** (`partition_area` in `fog_server.py`):

1. If there is only one drone, that drone gets the whole rectangle.
2. Otherwise, split the drones into two groups of `floor(n/2)` and the rest.
3. Cut the rectangle's **longer side** so that each piece's area is
   proportional to the number of drones it will hold.
4. Recurse on each piece.

```python
def partition_area(min_x, max_x, min_y, max_y, n):
    if n <= 1:
        return [(min_x, max_x, min_y, max_y)]
    a = n // 2
    b = n - a
    if (max_x - min_x) >= (max_y - min_y):
        cut = min_x + (max_x - min_x) * (a / n)
        return (partition_area(min_x, cut, min_y, max_y, a)
                + partition_area(cut, max_x, min_y, max_y, b))
    else:
        cut = min_y + (max_y - min_y) * (a / n)
        return (partition_area(min_x, max_x, min_y, cut, a)
                + partition_area(min_x, max_x, cut, max_y, b))
```

### Why this method

- **Works for any N.** A near-square grid (`ceil(sqrt(n))` columns) leaves
  empty cells whenever N is not a tidy grid number — e.g. 3 drones gives a
  2x2 grid with one cell unassigned, so a quarter of the area is never
  scanned. Bisection always produces exactly N cells.
- **Gap-free and non-overlapping.** The cells tile the rectangle exactly;
  their areas sum to the whole area with zero overlap (verified for N = 1..8).
- **Area-balanced.** The proportional cut gives every drone an equal share of
  the area (`total_area / N`).
- **Near-square cells.** Always cutting the longer side keeps cells from
  becoming long thin slivers, which keeps each drone's sweep efficient.

### What the cells look like

For the default area `X[-80, 100]  Y[-40, 75]`:

| N | Resulting layout |
|---|---|
| 1 | whole area |
| 2 | two halves (split along X, the longer axis) |
| 3 | three vertical strips: centroids (-50, 17.5), (10, 17.5), (70, 17.5) |
| 4 | 2x2 |
| 6 | 3x2 |

---

## 3. Per-drone flight: the commander phase machine

Once a drone receives its cell, `drone_commander.py` runs a four-phase
sequence using **PX4 offboard control**:

| Phase | What the drone does | Exit condition |
|---|---|---|
| `CLIMB` | Climbs straight up over its spawn point to scan altitude | within 1 m of target altitude (and spawn known) |
| `TRANSIT` | Flies to the first corner of its cell | within `waypoint_radius` of the first waypoint |
| `SCAN` | Sweeps the cell along a lawnmower path, **looping continuously** | never on its own — runs until an RTL command |
| `HOLD` | Loiters at the point | only entered when there is a single target (`do_scan=false`) |

The scan keeps repeating until the fog issues an RTL (which it does on
`/fog/end_mission`). The commander also **auto-recovers OFFBOARD** if PX4 drops
it (a transient failsafe no longer means permanent loss of control), and it
**leads toward each waypoint by a capped step** (`max_step`) so the drone
accelerates gently instead of rolling hard toward a far point — which is what
was tripping PX4's attitude-failure check.

### Why offboard streaming (and not NAV commands)

PX4 in this SITL build **rejects `DO_REPOSITION` (command 192)** — that is the
`command 192 unsupported` warning. It also refuses to enter OFFBOARD mode or
arm unless it is *already* receiving a steady setpoint stream. So the
commander:

1. Streams `OffboardControlMode` + `TrajectorySetpoint` at 10 Hz from the
   moment a mission starts.
2. After ~2 s, switches to OFFBOARD mode.
3. After ~3 s, arms.

Because the stream has been running, mode-switch and arming succeed, and the
drone flies to whatever setpoint is being streamed.

### Why we climb straight up first

If the commander streams the *far* cell target the instant it arms, the drone
tries to climb and shoot sideways simultaneously off the ground — which looks
like it is losing control. Instead, the `CLIMB` phase streams a setpoint
directly above the spawn (`N=0, E=0` in the local frame), so takeoff is a
clean vertical climb. Only after reaching altitude does it translate to the
cell.

### The lawnmower sweep

`build_lawnmower` lays parallel lanes across the cell, spaced `lane_spacing`
metres apart, reversing direction on alternate lanes so the path is
continuous (a boustrophedon / ox-plough pattern). Lanes run along the cell's
**longer dimension** so coverage uses fewer, longer passes. Setting
`do_scan:=false` skips the sweep and the drone simply holds at the cell
centroid — useful for first verifying clean flight before enabling coverage.

---

## 4. Coordinate frames & where spawns come from

There are two frames in play:

- **World ENU** — shared across the whole swarm. Origin = the Gazebo world
  origin (`WORLD_ORIGIN_LAT/LON`). `x` = metres East, `y` = metres North. The
  fog computes every cell in this frame.
- **Local NED** — each PX4 instance's own frame. Origin = that drone's spawn
  point. `x` = North, `y` = East, `z` = Down. PX4's `TrajectorySetpoint`
  lives here.

To fly to a world-ENU target, the commander must know its spawn's position in
world ENU. That spawn is **configured on the fog** as the `spawns_x` /
`spawns_y` parameters (defaults match the 3-drone setup; one entry per drone
in instance order). The fog sends each drone its spawn inside the
START_MISSION command, and the commander converts:

```
N = world_y - spawn_North
E = world_x - spawn_East
D = -altitude
```

Keeping spawns on the fog means there is one configurable source of truth and
nothing hardcoded in the commander. Each spawn entry must match that drone's
`PX4_GZ_MODEL_POSE` at launch.

**Fallback + safety check.** If the fog sends no spawn for a drone (fewer
spawn entries than drones), the commander auto-calibrates its spawn from PX4's
reported global reference (`ref_lat`/`ref_lon` in `VehicleLocalPosition`,
available once the EKF has a GPS fix). And whenever the fog *does* supply a
spawn, the commander cross-checks it against that same PX4 reference and warns
if they disagree by more than 3 m — catching a spawn parameter that does not
match the actual launch pose before it sends a drone to the wrong place.

> **Important:** `WORLD_ORIGIN_LAT/LON` must be identical in `fog_server.py`
> and `drone_commander.py`. It comes from the world SDF's
> `<spherical_coordinates>` block.

---

## 5. The mission command message

The fog publishes one `std_msgs/String` (JSON) per drone on
`/{drone_id}/mission_command`:

```json
{
  "command": "START_MISSION",
  "target": {
    "world_x": -50.0,
    "world_y": 17.5,
    "alt": 12.0,
    "lat": 47.39812...,
    "lon": 8.54550...,
    "area": { "min_x": -80.0, "max_x": -20.0, "min_y": -40.0, "max_y": 75.0 },
    "spawn": { "x": 18.0, "y": 25.0 }
  }
}
```

- `world_x/world_y` — cell centroid (used when `do_scan=false`).
- `alt` — scan altitude, metres above the spawn point.
- `lat/lon` — centroid in global coords (informational).
- `area` — the cell rectangle in world ENU; the commander builds its sweep
  inside these bounds.
- `spawn` — the drone's spawn in world ENU, from the fog's `spawns_x/spawns_y`
  parameters. Omitted if the fog has no spawn entry for that drone, in which
  case the commander auto-calibrates.

An `{"command": "RTL"}` message tells a drone to return to launch.

---

## 6. Parameters

### Fog (`fog_node fog_server`)

| Parameter | Default | Meaning |
|---|---|---|
| `num_drones` | 3 | Number of drones to partition for. **Set this to however many you launched.** |
| `spawns_x` / `spawns_y` | `[18,23,30]` / `[25,25,25]` | Drone spawn locations in world ENU (East / North), one entry per drone in instance order. Must match each `PX4_GZ_MODEL_POSE`. |
| `area_min_x` / `area_max_x` | -80 / 100 | Search area X bounds (ENU East, m) |
| `area_min_y` / `area_max_y` | -40 / 75 | Search area Y bounds (ENU North, m) |
| `scan_altitude` | 12.0 | Altitude sent to every drone (m) |

### Commander (`drone_node drone_commander`)

| Parameter | Default | Meaning |
|---|---|---|
| `instance` | 0 | PX4 instance index. Derives all topic names. |
| `do_scan` | true | true = sweep the whole cell; false = fly to centroid and hold |
| `lane_spacing` | 15.0 | Distance between sweep lanes (m). Smaller = denser coverage |
| `waypoint_radius` | 3.0 | How close counts as "arrived" at a waypoint (m) |
| `max_step` | 25.0 | Max distance the commanded setpoint leads the drone by (m). Smaller = gentler/slower flight |
| `default_alt` | 12.0 | Fallback altitude if a command omits one |

(The commander has no spawn parameter — its spawn comes from the fog command,
or is auto-calibrated from PX4 as a fallback.)

---

## 7. Log labels

| Label | Source | When |
|---|---|---|
| `[FOG]` | fog | Startup / per-drone config |
| `[FOG START_MISSION]` | fog | Partition summary + per-drone cell |
| `[FOG ALERT]` | fog | When a drone reports ARMED |
| `[COMMANDER]` | commander | Phase changes, calibration, waypoints |

---

## 8. Known limitations & future work

- **No inter-drone collision avoidance.** Drones fly direct paths at the same
  altitude; in dense swarms their transit paths can cross. Mitigations:
  per-drone altitude offsets, or assigning the spatially-nearest cell to each
  drone.
- **`num_drones` is set manually**, not auto-detected from live PX4
  instances. It must match the number of drones actually launched.
- **Coverage assumes a single rectangle.** Non-rectangular or
  obstacle-filled areas would need a finer decomposition.
- **Scan completeness depends on `lane_spacing`** relative to the camera's
  ground footprint at `scan_altitude`. Set `lane_spacing` no larger than the
  swath width for true full coverage.
