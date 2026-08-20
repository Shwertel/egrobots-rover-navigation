# egrobots-rover-navigation — Custom Robot URDF, Gazebo Bridging & TF2 Localization (ROS 2 Humble)

A ROS 2 package that models Egrobots' own rover from scratch in URDF, bridges it
into Gazebo with a working drivetrain and LiDAR, and reuses the obstacle
avoidance + goal navigation logic from Task 1 — now running on TF2-based
localization instead of raw odometry math.

Built as part of the Egrobots ROS 2 Week 2 Task (Robot Localization and
Coordinate Frames), extending Week 1's Autonomous Obstacle Avoidance package.

---

## 1. Overview

This package builds a custom mobile robot from scratch — not TurtleBot3 — matching
Egrobots' real rover: a wide chassis, four wheels, and a raised sensor mast, sized
from a reference photo of the physical robot. The robot is:

- **Modeled in URDF/xacro** — chassis, four wheels, mast, and sensor mount, each
  as a proper link/joint with visual, collision, and inertial properties.
- **Bridged into Gazebo** — a LiDAR (ray sensor) plugin publishes real `/scan`
  data from the sensor mount, and a genuine **4-wheel-drive** drivetrain (via
  `gazebo_ros2_control` + `ros2_controllers`' `DiffDriveController`) drives all
  four wheels together, rather than only a front pair.
- **Located via TF2** — the robot's position is obtained dynamically from the
  `odom → base_link` transform (published by the drive controller), not
  hardcoded, and the full frame tree is visualizable in RViz.
- **Running the same avoidance + navigation node from Task 1**, unmodified in
  its core logic, driving this new robot instead of TurtleBot3 — proving the
  original design was correctly decoupled from any specific robot.

---

## 2. Package Contents

| File | Purpose |
|---|---|
| `urdf/egrobots_rover.urdf.xacro` | The custom robot: chassis, 4 wheels, mast, sensor mount, materials, LiDAR sensor, and `ros2_control` hardware interface |
| `worlds/egrobots_world.world` | Minimal Gazebo world with increased physics solver iterations (see Section 5) |
| `config/ros2_controllers.yaml` | `controller_manager` config: `joint_state_broadcaster` and a 4-wheel `diff_drive_controller` |
| `config/task2_egrobots_rover_params.yaml` | Avoidance/navigation parameters tuned for this robot's size and sensor placement |
| `launch/egrobots_rover.launch.py` | Single launch file: Gazebo (custom world), robot spawn, controllers, and the avoidance node |
| `obstacle_avoider/obstacle_avoider_node.py` | Shared avoidance + goal navigation node (from Task 1), now angle-aware so it works correctly regardless of a sensor's `angle_min` |
| `obstacle_avoider/manual_controller.py` | Keyboard teleop node, for manually driving and testing the custom robot |

---

## 3. Build & Run — From a Clean Workspace

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-urdf-tutorial
sudo apt install ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-controller-manager
```

### Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place this package here as obstacle_avoider/

cd ~/ros2_ws
colcon build --packages-select obstacle_avoider
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Run everything with one command
```bash
ros2 launch obstacle_avoider egrobots_rover.launch.py
```

This starts Gazebo (custom world), spawns the rover, loads and activates the
drive controllers, and starts `obstacle_avoider_node` with parameters from
`task2_egrobots_rover_params.yaml`. Then, in another terminal:
```bash
ros2 service call /start_avoidance std_srvs/srv/Trigger
```
```bash
ros2 service call /stop_avoidance std_srvs/srv/Trigger
```

### Manual driving (for testing)
The drive controller expects commands on a different topic than the node
defaults to, so the launch file remaps `obstacle_avoider_node`'s `/cmd_vel`
internally. For manual testing outside the launch file, remap explicitly:
```bash
ros2 run obstacle_avoider manual_controller --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Visualizing coordinate frames (R5)
```bash
ros2 run tf2_tools view_frames
```
generates a PDF of the full frame tree. To watch it live:
```bash
rviz2
```
Set **Fixed Frame** to `odom`, then **Add → TF** (and optionally **RobotModel**).

---

## 4. ROS 2 Components

### Custom robot (`egrobots_rover.urdf.xacro`)

**Links:** `base_link` (chassis), `wheel_front_left`, `wheel_front_right`,
`wheel_rear_left`, `wheel_rear_right`, `mast`, `sensor_mount`.

**Sensors:** a `ray`-type LiDAR on `sensor_mount`, publishing `sensor_msgs/msg/LaserScan`
on `/scan` — 360° coverage, 0.12–10.0 m range, matching the same message shape
`obstacle_avoider_node` already consumed for TurtleBot3.

**Drivetrain:** `gazebo_ros2_control` loads a `ros2_controllers` `DiffDriveController`
with `left_wheel_names: [wheel_front_left_joint, wheel_rear_left_joint]` and
`right_wheel_names: [wheel_front_right_joint, wheel_rear_right_joint]` — all four
wheels are genuinely torque-driven, not just the front pair.

**Coordinate frames published:** `odom → base_link` (by the drive controller,
with `publish_odom_tf` behavior built into `ros2_controllers`), plus the full
static tree `base_link → mast → sensor_mount` and per-wheel joint transforms
(by `robot_state_publisher` and `joint_state_broadcaster`).

### Node: `obstacle_avoider_node` (shared with Task 1)

Same topics, services, and parameters as documented in the Task 1 README — see
that repository for the full reference. The only code change made for this
robot was making the forward-detection cone **angle-aware** (computed from the
scan message's own `angle_min`/`angle_increment`) instead of assuming index 0
is always straight ahead — a fix that also benefits Task 1's TurtleBot3 setup,
since it was silently relying on that assumption being true.

**Parameters retuned for this robot** (`task2_egrobots_rover_params.yaml`):
larger `safe_distance`/`clear_distance` (this chassis is significantly larger
than TurtleBot3, so more clearance is needed relative to the sensor's position
on the mast) and a wider `cone_angle_deg` (to cover the wider chassis track).

---

## 5. Design Decisions

**Custom robot sized from a reference photo, not exact measurements.** No
precise dimensions were available, so chassis, wheel, and mast sizes were
estimated by comparing the robot's proportions to nearby people in a reference
photo — explicitly a rough-but-reasonable approach appropriate for simulation,
not a claim of dimensional accuracy.

**Sensor mount height was iteratively tuned, not fixed at the mast top.**
Initially placed at the top of the mast (~0.85 m), the LiDAR's horizontal scan
plane passed entirely over typical Gazebo test obstacles. It was lowered to
roughly the mast's midpoint so it reliably intersects obstacles at a realistic
height — a legitimate design decision about where a 2D LiDAR should sit,
distinct from a bug.

**True 4-wheel-drive required `gazebo_ros2_control`, not the simpler
`gazebo_ros_diff_drive` plugin.** The latter only supports one joint per side
and rejected multiple `<left_joint>`/`<right_joint>` tags outright ("inconsistent
number of joints specified"). `ros2_controllers`' `DiffDriveController` genuinely
supports wheel groups via `left_wheel_names`/`right_wheel_names` lists, so the
drivetrain was migrated to the full `ros2_control` stack instead.

**Wheel friction values were left untouched per direction from the team**, even
when they were the most likely lever for smoothing out a turning issue. Instead,
the physics solver's iteration count was increased in a custom world file — this
meaningfully improved turning consistency without touching friction at all.

**Some turning inconsistency remains and is understood, not hidden.** A rigid
4-wheel vehicle with no suspension is a classically over-constrained contact
problem for a physics engine — any three contact points define a stable plane,
and a rigid fourth point has to be resolved approximately. This was confirmed
directly by watching `/joint_states` during a pure turn command: even with
uniform friction across all wheels, one diagonal pair carried most of the
rotation while the other lagged. Raising solver iterations reduced this
significantly; a proper fix would add wheel suspension, which was out of scope
for this task's timeframe.

**The avoidance node's cone-detection was fixed to be angle-aware.** Porting the
Task 1 code to this robot's differently-configured LiDAR (`angle_min = -π`
instead of TurtleBot3's `0`) revealed that the original cone-detection logic
silently assumed index 0 was always "straight ahead." It now computes the
straight-ahead index from the scan message's own angle fields, making the same
node correctly portable to any sensor configuration — a real bug caught by
testing on new hardware, not a Task 2-specific hack.

---

## 6. Known Limitations / Possible Extensions

- No wheel suspension — the residual turning inconsistency described above is
  a direct, understood consequence of this; adding compliant/spring wheel
  joints would be the proper fix.
- The rover's dimensions are estimated, not measured from the real robot.
- Sensor mount height was tuned against generic Gazebo test shapes, not a
  specific known obstacle size the real deployment will face.
- As in Task 1: no full path planning around obstacles toward the goal, no
  recovery from being fully boxed in, and testing was limited to single static
  obstacles in an otherwise empty world.
