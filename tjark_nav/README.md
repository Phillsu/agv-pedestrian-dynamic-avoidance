# Obstacle Avoidance Navigation for Mobile Robots in Dynamic Environments

Autonomous navigation demo for the **tjark_agv** differential-drive robot using
ROS Noetic + Gazebo 11 on Ubuntu 20.04.

The robot navigates a 26 × 16 m arena from start to goal, encountering two
separate crowds of pedestrians (5 per crowd, i.e. > 4) and avoiding each one
using AMCL localisation and the DWA local planner. The start and end points
are recognised by the on-board RGB-D camera during a prior RTAB-Map mapping
phase (green panel = start, red panel = end).

---

## Project background

### Motivation

Service and warehouse robots increasingly have to share their workspace with
people who move unpredictably. A practical mobile robot therefore needs to do
two things well: (1) understand *where it is* and *where it must go* without a
human hand-feeding it coordinates, and (2) react in real time to obstacles —
especially people — that were never on the map. This project demonstrates an
end-to-end pipeline for both, in simulation, on the **tjark_agv** platform.

### The robot

`tjark_agv` is a differential-drive robot (wheel separation 0.34 m, wheel
diameter 0.12 m) carrying two perception sensors:

| Sensor | Role |
|--------|------|
| 360° RPLidar (`/scan`) | 2-D obstacle sensing for SLAM, localisation and the local costmap |
| RGB-D depth camera (`/my_camera/...`) | Visual recognition of the start/end signs and RGB-D input to SLAM |

It is driven entirely through `/cmd_vel` (the Gazebo differential-drive plugin)
and reports wheel odometry on `/odom`.

### The use case — recognise the route, then run it

The mission deliberately mirrors how a real deployment is commissioned: a
mapping pass first, an autonomous run second.

**Phase 1 — map & recognise.** The operator drives the robot around the arena
once. While driving, **RTAB-Map** fuses the wheel odometry, the LiDAR and the
RGB-D camera into a 2-D occupancy grid. At the same time, the camera looks for
two coloured cues mounted in the world:

- a **green square** beside the **start** point, and
- a **red square** beside the **end** point.

`marker_detector.py` recognises each square in the RGB image, reads its true
3-D position from the depth point cloud, transforms it into the map frame, and
records it. The recognised points are marked in RViz (**green = start,
red = end**) and saved to a waypoints file. No coordinates are typed by hand.

**Phase 2 — localise & navigate.** On the next boot the robot loads the saved
map, **AMCL** matches the live laser scan against the map walls to work out its
exact pose, and the global planner computes a collision-free route from the
recognised start to the recognised end. As the robot follows that route, the
**DWA local planner** reads the laser dozens of times per second and continually
re-plans a fine evasion trajectory around anything that appears — here, two
crowds of **five pedestrians each** crossing the aisle. The route is laid out so
the robot must avoid a crowd of more-than-four people at **two** separate points
before reaching the goal.

### Pipeline at a glance

```
Phase 1 (mapping.launch)                  Phase 2 (demo.launch)
─────────────────────────                 ─────────────────────────
 LiDAR + RGB-D + odom                       map + waypoints (from Phase 1)
        │                                          │
   RTAB-Map SLAM ── 2-D grid ──► map.pgm      map_server ─► /map
        │                                          │
   marker_detector (green/red)                 AMCL  ─► map→odom (localise)
        │                                          │
   waypoints.yaml (start, end)               move_base: global plan (NavFn)
                                                   + local plan (DWA, dynamic
                                                     pedestrian avoidance)
                                                   └─► /cmd_vel
```

### Running it

```bash
./run_demo.sh mapping     # Phase 1 — then './run_demo.sh teleop' to drive
./run_demo.sh demo        # Phase 2 — autonomous start→end with avoidance
./run_demo.sh test        # headless smoke test of the whole stack
```

---

## System requirements

| Component | Version |
|-----------|---------|
| Ubuntu    | 20.04 LTS |
| ROS       | Noetic  |
| Gazebo    | 11 (ships with ROS Noetic) |
| Python    | 3.8     |

### ROS packages

Install all navigation dependencies in one command:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-navigation \
  ros-noetic-gmapping \
  ros-noetic-rtabmap-ros \
  ros-noetic-cv-bridge \
  ros-noetic-vision-opencv \
  ros-noetic-image-geometry \
  python3-opencv \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-teleop-twist-keyboard
```

---

## Repository layout

```
proj/
├── tjark_agv/                  ← existing robot description package
│   ├── urdf/                   ← URDF + xacro fragments
│   ├── meshes/                 ← STL files
│   └── launch/                 ← original launch files
└── tjark_nav/                  ← navigation package (this package)
    ├── launch/
    │   ├── spawn_world.launch  ← Gazebo + robot only
    │   ├── mapping.launch      ← RTAB-Map RGB-D SLAM + camera marker detection
    │   ├── navigation.launch   ← map_server + AMCL + move_base
    │   └── demo.launch         ← full demo (everything at once)
    ├── config/
    │   ├── amcl.yaml
    │   ├── costmap_common.yaml
    │   ├── global_costmap.yaml
    │   ├── local_costmap.yaml
    │   ├── dwa_local_planner.yaml
    │   └── move_base.yaml
    ├── worlds/
    │   └── obstacle_avoidance.world
    ├── maps/
    │   ├── generate_map.py     ← generates map.pgm from world geometry
    │   └── map.yaml
    ├── scripts/
    │   ├── pedestrian_controller.py
    │   ├── marker_detector.py      ← RGB-D green/red start-end recognition
    │   └── navigation_demo.py
    └── rviz/
        └── navigation.rviz
```

---

## Architecture

### Scenario design

```
                    Zone A               Zone B
                   (x ≈ -4)             (x ≈ +4)
                  ╫ ╫ ╫ ╫               ╫ ╫ ╫ ╫
 Start ──────────────────────────────────────────── Goal
(-9, 0)                 Waypoint (0,0)           (9, 0)
                   5 pedestrians          5 pedestrians
                  cross path here        cross path here
```

The robot is given **two sequential goals**:
1. `(0, 0)` — must avoid **Zone A** (5 pedestrians, x ≈ −4)
2. `(9, 0)` — must avoid **Zone B** (5 pedestrians, x ≈ +4)

Pedestrians oscillate in the y-direction with amplitude 4.5 m and period 9 s.
Adjacent pedestrians have a π/2 phase offset so the crowd is always spread out.

### ROS node graph

```
Gazebo
 ├─ /scan           →  amcl  →  TF(map→odom)
 ├─ /odom           →  amcl
 └─ /tf             →  move_base

map_server  →  /map  →  move_base (global costmap)
                      →  amcl

move_base
 ├─ global planner:  NavfnROS      (uses /map)
 ├─ local planner:   DWAPlannerROS (uses /scan, local costmap)
 └─ /cmd_vel        →  Gazebo diff-drive plugin

pedestrian_controller  →  /gazebo/set_model_state (10 cylinders)
navigation_demo        →  /move_base (actionlib)
```

### TF tree

```
map  ──[amcl]──►  odom  ──[diff-drive]──►  base_link
                                            ├──[fixed]──►  base_body
                                            │               ├──►  laser
                                            │               └──►  camera_link ──►  camera_optical_link
                                            ├──►  left_wheel
                                            ├──►  right_wheel
                                            └──►  LF/LB/RF/RB_link (casters)
```

### Pedestrian simulation

Cylinders (radius 0.30 m, height 1.80 m) are spawned via
`/gazebo/spawn_sdf_model` and repositioned at 10 Hz via
`/gazebo/set_model_state`.  Because they are real Gazebo physics models:

- They appear in the **LiDAR scan** (`/scan`)
- The **local costmap** inflates around them and clears when they move away
- **DWA** replans around the updated costmap in real time

---

## Quick start (pre-generated map)

### Step 1 — build the catkin workspace

```bash
# Assuming your workspace src/ is at ~/catkin_ws/src
cd ~/catkin_ws/src
# link or copy the two packages if not already there
ln -s /home/phill/proj/tjark_agv .
ln -s /home/phill/proj/tjark_nav .

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 2 — generate the map

```bash
python3 $(rospack find tjark_nav)/maps/generate_map.py
```

This writes `maps/map.pgm` (520 × 320 px, 0.05 m/px) matching the arena walls.

### Step 3 — run the demo

```bash
roslaunch tjark_nav demo.launch
```

Gazebo, RViz, AMCL, move_base, the pedestrian controller and the goal script
all start automatically. Watch the robot in RViz navigate around both crowds.

---

## Phase 1 — RTAB-Map SLAM + camera start/end detection

Build the map with RTAB-Map RGB-D SLAM while the camera automatically
recognises the START (green panel) and END (red panel) signs:

```bash
# Terminal 1 – Gazebo + RTAB-Map + marker_detector + RViz
roslaunch tjark_nav mapping.launch

# Terminal 2 – teleop the robot around the arena
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

While driving, point the camera at each coloured sign from within ~3 m
(the depth far-clip):

- **Green panel** (beside the start, at −9, −2) → locks the **START** waypoint
- **Red panel** (beside the end, at 9, +2)     → locks the **END** waypoint

Each lock prints `✓ LOCKED …`, drops a coloured sphere on `/detected_markers`
in RViz, and writes `maps/obstacle_waypoints.yaml`
(`maps/cafe_waypoints.yaml` for `scene:=cafe`).

```bash
# Terminal 3 – save the occupancy grid when coverage is complete
rosrun map_server map_saver -f $(rospack find tjark_nav)/maps/map

# Then Ctrl-C everything and run Phase 2 (demo.launch) — it loads the saved
# map + detected waypoints and navigates start → end automatically.
```

### How the camera detection works

`scripts/marker_detector.py`:

1. HSV-thresholds the RGB image for the green / red square (largest contour).
2. Reads the matching 3-D point directly from the **organised depth point
   cloud** at the blob's centroid pixel — this carries correct geometry in the
   `camera_optical_link` frame, sidestepping any optical-convention guesswork.
3. Transforms the point into the `map` frame via TF.
4. After enough stable consecutive frames, **locks** the position, publishes
   the green/red RViz marker, and saves the waypoints YAML.

> **Camera frame note:** the depth plugin publishes in REP-103 optical
> convention, so `tjark_agv.sensor.xacro` defines a dedicated
> `camera_optical_link` and points the plugin's `frameName` at it. Without this
> the projected detections land in the floor.

---

## Running components separately

```bash
# Gazebo + robot only (no navigation):
roslaunch tjark_nav spawn_world.launch

# Navigation stack only (assumes Gazebo already running):
roslaunch tjark_nav navigation.launch

# Spawn pedestrians manually:
rosrun tjark_nav pedestrian_controller.py

# Send goals manually:
rosrun tjark_nav navigation_demo.py
```

---

## Key parameters

| File | Parameter | Default | Effect |
|------|-----------|---------|--------|
| `pedestrian_controller.py` | `AMPLITUDE` | 4.5 m | Pedestrian y-range |
| `pedestrian_controller.py` | `PERIOD`    | 9 s   | Oscillation period |
| `dwa_local_planner.yaml` | `max_vel_x` | 0.45 m/s | Maximum forward speed |
| `dwa_local_planner.yaml` | `sim_time`  | 2.0 s | DWA forward simulation horizon |
| `costmap_common.yaml` | `inflation_radius` | 0.60 m | Obstacle buffer size |
| `amcl.yaml` | `max_particles` | 3000 | Localisation accuracy vs. CPU |

---

## Troubleshooting

### Robot does not move
- Check `rostopic echo /scan` — if empty, the laser plugin may not be active.
  Verify that `tjark_agv.xacro` includes `tjark_agv.sensor.xacro`.
- Check `rostopic echo /odom` — if empty, the diff-drive plugin is not
  publishing. Confirm Gazebo loaded the robot model correctly.

### AMCL not localising
- The initial pose is published automatically to `/initialpose` at spawn
  coordinates (-9, 0).  If AMCL still diverges, open RViz → use the
  **2D Pose Estimate** tool to click the robot's actual location.
- Run `rosrun tf tf_echo map odom` — if TF is missing, AMCL is not running.

### Pedestrians not visible in RViz / not blocking the robot
- Run `rosservice call /gazebo/get_model_state '{model_name: ped_A1, relative_entity_name: world}'`
  to verify pedestrians exist in Gazebo.
- The laser scan angle range (90°–270° in sensor frame) combined with the
  laser joint rotation covers the forward hemisphere.  If the robot approaches
  from an unscanned direction, extend the scan range in
  `tjark_agv/urdf/tjark_agv.sensor.xacro`:
  ```xml
  <min_angle>-3.14159</min_angle>
  <max_angle> 3.14159</max_angle>
  ```

### move_base oscillates / robot gets stuck
- Increase `controller_patience` in `config/move_base.yaml`.
- Reduce `min_vel_trans` in `dwa_local_planner.yaml` to allow slower creeping.
- Use the **2D Nav Goal** tool in RViz to test with a custom goal while the
  demo is running.

### Gazebo crash / model spawn failure
- Ensure `GAZEBO_MODEL_PATH` includes the standard Gazebo model directory:
  ```bash
  export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
  ```
- If `model://ground_plane` or `model://sun` fail, add the above export to
  your `~/.bashrc`.

---

## Extending the demo

| Idea | How |
|------|-----|
| Add more pedestrians | Extend `ZONE_A_X` / `ZONE_B_X` lists in `pedestrian_controller.py` |
| Add a third avoidance zone | Add a `ZONE_C_X` list and a third entry in `GOALS` in `navigation_demo.py` |
| Use TEB planner (better dynamic avoidance) | `sudo apt install ros-noetic-teb-local-planner`; set `base_local_planner: teb_local_planner/TebLocalPlannerROS` in `move_base.yaml` |
| Record a rosbag | `rosbag record /scan /odom /amcl_pose /move_base/global_costmap/costmap /tf` |
| Export an occupancy video | Play back the bag in RViz and use `ros-noetic-image-transport` to capture |
