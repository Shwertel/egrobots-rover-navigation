# egrobots-rover-navigation — Custom Robot URDF, Gazebo Bridging & TF2 Localization (ROS 2 Humble)

A ROS 2 package that models Egrobots' own rover from scratch in URDF, bridges it
into Gazebo with a working 4-wheel drivetrain and LiDAR, and uses **TF2** to
locate the robot and to place its sensor readings in a fixed world frame.

Built for the Egrobots ROS 2 Week 2 Task (Robot Localization and Coordinate
Frames), extending Week 1's Autonomous Obstacle Avoidance package.

---

## 1. Overview

This package builds a custom mobile robot — not TurtleBot3 — matching Egrobots'
real rover: a wide chassis, four driven wheels, and a raised sensor mast. The
robot is:

- **Modeled in URDF/xacro** — chassis, four wheels, mast, and sensor mount, each
  a proper link/joint with visual, collision, and inertial properties.
- **Bridged into Gazebo** — a ray-sensor LiDAR publishes `/scan` from the sensor
  mount, and a genuine **4-wheel-drive** drivetrain (`gazebo_ros2_control` +
  `ros2_controllers`' `DiffDriveController`) drives all four wheels together.
- **Located via TF2** — the robot's pose comes from the `odom → base_link`
  transform. Nothing is hardcoded.
- **Reasoning in frames, not just reading them** — LiDAR returns are converted
  from the sensor's own frame into the fixed reference frame, so a detected
  obstacle gets a world position independent of where the robot was standing.
- **Driven by its own position** — a geofence behaviour keeps the rover within a
  configurable radius of the reference frame's origin.

### How the task requirements are met

| ID | Requirement | Where |
|---|---|---|
| R1 | Work with existing coordinate frames and sensor data | `report_obstacle_in_reference_frame()` — transforms LiDAR returns from `sensor_mount` into `odom` |
| R2 | Obtain and use the robot's position/orientation | `update_pose_from_tf()` — looks up `odom → base_link` at 10 Hz |
| R3 | Publish or expose the current position | `/robot_pose` (`PoseStamped`), plus `/detected_obstacle` and a broadcast `detected_obstacle` frame |
| R4 | Use the position to implement a simple behaviour | Geofence — distance from the `odom` origin decides whether the rover keeps cruising |
| R5 | Visualize and verify the coordinate frames | Bundled RViz2 config (auto-launched) + `ros2 run tf2_tools view_frames` |

---

## 2. Package Contents

| File | Purpose |
|---|---|
| `urdf/egrobots_rover.urdf.xacro` | The custom robot: chassis, 4 wheels, mast, sensor mount, LiDAR, and the `ros2_control` hardware interface |
| `worlds/egrobots_world.world` | Gazebo world with raised physics solver iterations (see Section 6) |
| `config/ros2_controllers.yaml` | `controller_manager` config: `joint_state_broadcaster` + a 4-wheel `diff_drive_controller`, with velocity/acceleration limits and skid-steer odometry calibration |
| `config/geofence_params.yaml` | Avoidance + geofence parameters (`behavior: geofence`) |
| `config/goal_navigation_params.yaml` | Avoidance + goal-seeking parameters (`behavior: goal`) |
| `launch/simulation.launch.py` | Shared bring-up: Gazebo, robot spawn, controllers, RViz2 — no behaviour node |
| `launch/geofence.launch.py` | Simulation + the geofence behaviour |
| `launch/goal_navigation.launch.py` | Simulation + goal navigation to `(goal_x, goal_y)` |
| `rviz/egrobots_rover.rviz` | RViz2 config showing the frame tree, robot model, laser scan, geofence, and detections |
| `obstacle_avoider/obstacle_avoider_node.py` | The single node: TF2 localization, obstacle avoidance, and the geofence behaviour |
| `obstacle_avoider/manual_controller.py` | Keyboard teleop, for manually driving the robot |

---

## 3. Build & Run

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-controller-manager
```

### Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place this package here as obstacle_avoider/

cd ~/ros2_ws
colcon build --packages-select obstacle_avoider
source install/setup.bash
```

### Run

Two behaviours, each with its own launch file. Both start Gazebo, spawn the
rover, activate the drive controllers, start `obstacle_avoider_node`, and open
RViz2 with the frame visualisation. Add `rviz:=false` to skip RViz.

The geofence behaviour (R4 — cruise, avoid obstacles, turn back at the boundary):
```bash
ros2 launch obstacle_avoider geofence.launch.py
```

Goal navigation (Week 1's behaviour on this rover — drive to a coordinate):
```bash
ros2 launch obstacle_avoider goal_navigation.launch.py
```
The target can be overridden without editing YAML:
```bash
ros2 launch obstacle_avoider goal_navigation.launch.py goal_x:=5.0 goal_y:=2.0
```

Both include `simulation.launch.py`, which can also be run alone to bring up the
robot with no behaviour attached — useful for manual driving or teleop testing.

The node starts **disabled** by design. To begin:
```bash
ros2 service call /start_avoidance std_srvs/srv/Trigger
```
```bash
ros2 service call /stop_avoidance std_srvs/srv/Trigger
```

The bundled world contains no obstacles — add them from the Gazebo GUI, within
the geofence radius so the rover meets them while cruising.

### Verifying the coordinate frames (R5)
```bash
ros2 run tf2_tools view_frames
```
generates a PDF of the full frame tree. To inspect a single transform live:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
RViz2 (launched automatically, Fixed Frame `odom`) shows the frame axes, the
robot model, the laser scan, the geofence ring, and the transformed obstacle
detection.

### Manual driving
The drive controller listens on a different topic than the node's default, so
remap explicitly:
```bash
ros2 run obstacle_avoider manual_controller --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---

## 4. Coordinate Frames

```
odom                     fixed reference frame, published by diff_drive_controller
 └── base_link           the robot body
      ├── wheel_front_left / wheel_front_right
      ├── wheel_rear_left / wheel_rear_right
      └── mast
           └── sensor_mount     the LiDAR reports its ranges in this frame
odom
 └── detected_obstacle   published by this node, where the nearest obstacle is
```

`odom → base_link` is published by the drive controller from wheel odometry.
The static chain `base_link → mast → sensor_mount` and the wheel joint
transforms come from `robot_state_publisher` and `joint_state_broadcaster`.
`odom → detected_obstacle` is broadcast by `obstacle_avoider_node`.

---

## 5. ROS 2 Interface

### Node: `obstacle_avoider_node`

**Subscribes:** `/scan` (`sensor_msgs/msg/LaserScan`), and `/tf` + `/tf_static`
via a `TransformListener`.

**Publishes:**

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` (remapped to `/diff_drive_controller/cmd_vel_unstamped`) | `geometry_msgs/msg/Twist` | Velocity commands |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | R3: the robot's current pose in `odom` |
| `/detected_obstacle` | `geometry_msgs/msg/PointStamped` | R1/R3: nearest obstacle, expressed in `odom` |
| `/geofence_marker` | `visualization_msgs/msg/Marker` | The boundary ring, drawn in `odom` |
| `/tf` | `tf2_msgs/msg/TFMessage` | The `detected_obstacle` frame |

**Services:** `/start_avoidance` and `/stop_avoidance` (`std_srvs/srv/Trigger`).

**Parameters** (`config/geofence_params.yaml`, `config/goal_navigation_params.yaml`):

| Parameter | Value | Meaning |
|---|---|---|
| `behavior` | `geofence` / `goal` | Which position-driven behaviour runs when the path is clear |
| `safe_distance` | `1.2` m | Distance below which an obstacle triggers avoidance |
| `clear_distance` | `1.5` m | Distance the path must exceed before the turn ends |
| `linear_speed` | `1.0` m/s | Cruise speed |
| `angular_speed` | `0.8` rad/s | Turn rate while avoiding; cap while steering |
| `cone_angle_deg` | `30.0` deg | Half-angle of the forward detection cone |
| `clearing_distance` | `2.0` m | Distance driven past an obstacle before steering resumes |
| `direction_scan_deg` | `90.0` deg | Half-angle of the window used to choose a turn side |
| `heading_kp` | `1.5` | Proportional gain when steering toward a bearing |
| `reference_frame` | `odom` | The fixed frame the robot localises against |
| `robot_frame` | `base_link` | The robot's body frame |

Geofence only:

| Parameter | Value | Meaning |
|---|---|---|
| `max_distance_from_origin` | `5.0` m | R4: the geofence radius |
| `boundary_return_ratio` | `0.9` | Re-entry declared at this fraction of the radius |

Goal navigation only:

| Parameter | Value | Meaning |
|---|---|---|
| `goal_x` / `goal_y` | `10.0` / `0.0` | Target position in the reference frame |
| `goal_tolerance` | `0.15` m | Distance within which the goal counts as reached |
| `heading_tolerance_deg` | `30.0` deg | Pivot in place until within this of the target bearing, then drive straight |
| `goal_approach_distance` | `1.0` m | Begin decelerating this far out |

### Behaviour

Three states, evaluated every scan:

- **CRUISING** — drive forward.
- **TURNING** — an obstacle is within `safe_distance`; pivot toward the side with
  more open space until the path is clear past `clear_distance`.
- **CLEARING** — drive `clearing_distance` forward before any steering resumes,
  so the rover actually gets past the obstacle instead of turning back into it.

Once the path is clear, the `behavior` parameter decides what happens next —
hold the geofence, or steer to the goal. Both steer from the pose obtained via
TF2, and both share the same avoidance states above.

The geofence and the goal-arrival check are evaluated on every scan, *before*
the avoidance branches can return early, so neither is starved while the robot
is avoiding something. Obstacle avoidance outranks both: a collision reflex must
not wait on a position-based rule, and it runs purely off the LiDAR so it never
depends on odometry.

In goal mode the forward speed is capped by proximity to the goal in *every*
state, not just while goal-seeking. `CLEARING` drives at full speed, so a goal
falling inside its clearing run would otherwise be crossed at 1.0 m/s and
overshot by the robot's 0.25 m stopping distance.

---

## 6. Design Decisions

**Custom robot sized from a reference photo, not exact measurements.** No precise
dimensions were available, so chassis, wheel, and mast sizes were estimated from
the robot's proportions relative to people in a reference photo — a
rough-but-reasonable approach for simulation, not a claim of dimensional
accuracy.

**Sensor mount height was iteratively tuned.** Initially at the top of the mast
(~0.85 m), the LiDAR's scan plane passed over typical Gazebo test obstacles. It
was lowered to roughly the mast's midpoint so it reliably intersects obstacles at
a realistic height.

**True 4-wheel-drive required `gazebo_ros2_control`.** The simpler
`gazebo_ros_diff_drive` plugin supports only one joint per side and rejects
multiple `<left_joint>`/`<right_joint>` tags outright. `DiffDriveController`
supports wheel groups via `left_wheel_names`/`right_wheel_names`, so the
drivetrain was migrated to the full `ros2_control` stack.

**Velocity and acceleration limits had to be explicitly enabled.**
`DiffDriveController` ignores `max_velocity`/`max_acceleration` unless
`has_velocity_limits`/`has_acceleration_limits` are separately set to `true` —
both default to `false`. With the flags missing, the limiter silently did
nothing and every avoidance turn commanded an instantaneous torque reversal,
which broke wheel traction. Enabling them ramps the command over a few control
cycles instead.

**Skid-steer odometry was calibrated against ground truth.** A rigid 4-wheel
rover must scrub its wheels sideways to turn, but `DiffDriveController`
integrates wheel encoders assuming no slip. Measured against Gazebo's true model
pose, a commanded 0.6 rad/s pivot reported **85° of odometry yaw for 63° of real
rotation** — a 1.35× over-report. `wheel_separation_multiplier: 1.18` brings the
measured ratio to roughly 1.0, reflecting an effective track wider than the
geometric one.

**Avoidance commits to clearing an obstacle, rather than reacting frame by
frame.** An earlier version ended avoidance as soon as the narrow forward cone
read clear. Because a pivot can rotate an obstacle out of a 30° cone without the
robot moving at all, the rover would immediately steer back toward its original
heading, re-trigger avoidance, and oscillate in place indefinitely — logged
traces showed it holding the same position for over 40 seconds. The `CLEARING`
state fixes this by requiring real forward displacement past the obstacle.

**Turn direction is chosen over a window wider than the detection cone.** A
dead-centred obstacle produces equal minimum ranges on both sides of a 30° cone,
which always resolved to the same side. Direction is now chosen over a 90°
window, with the mean range breaking a tie on the minimum.

**Obstacle detections are transformed into the reference frame, not left in
sensor coordinates.** A LiDAR return is inherently relative — "1.2 m at 15°" is
meaningless without knowing where the sensor was. Running it through
`sensor_mount → odom` yields a stable world position. This is the clearest
demonstration in the package that frames are being composed rather than merely
read: the published point carries `z ≈ 0.475`, the sensor's height above
`base_link`, which only appears because the transform was genuinely applied.

**The geofence, not goal navigation, is what this task submits for R4.** Week 1's
node drove to a goal coordinate. That behaviour depends on absolute position
accuracy — precisely what wheel odometry on a skid-steer platform cannot provide
(see Section 7). The geofence is driven by position relative to a fixed frame,
satisfies the requirement just as directly, and degrades gracefully rather than
catastrophically as odometry drifts. Goal navigation is still available via
`goal_navigation.launch.py`, kept as a second `behavior` mode on the same node
rather than a forked copy, so both share one avoidance implementation.

**The robot turns to face its target, then drives straight.** Beyond
`heading_tolerance_deg` it pivots in place instead of arcing toward the bearing.
A pivot is actually the *worst* case for wheel scrub — 58% of each wheel's motion
is lateral sliding, against 20% for an arc at 1.0 m/s — but it minimises *total*
rotation, and rotation is where nearly all the odometry error comes from.
Measured on this robot: a straight 10 m run finished 0.01 m from truth, while a
5 m run containing a single avoidance turn finished 2.3 m out. Continuous
proportional steering was correcting the whole way and accumulating error with
every correction.

---

## 7. Known Limitations

**Wheel odometry drifts, and cannot be fully fixed by calibration.** This is the
most significant limitation and it is inherent, not a tuning oversight:

- Lateral slip is invisible to wheel encoders. A skid-steer robot slides
  sideways while turning, and the odometry has no way to observe it.
- Dead reckoning has no absolute reference, so error accumulates without bound.
- The slip itself is not repeatable — an identical 0.6 rad/s command produced
  between 60° and 75° of real rotation across runs, because a rigid 4-wheel
  chassis with no suspension is an over-constrained contact problem.

In one logged run the node reported `Goal reached at (10.07, -0.05)` while
Gazebo placed the robot at `(2.18, 8.04)` — an 11 m error. In ROS terms, `odom`
is a continuous-but-drifting frame; a drift-free `map` frame is what a
localization source would provide. This robot has none, so absolute positions
degrade over time. Obstacle avoidance is unaffected, since it reads only the
LiDAR.

The proper fixes, both out of scope for this task's timeframe, are an IMU fused
with wheel odometry (e.g. `robot_localization`'s EKF), or a ground-truth
odometry plugin for simulation-only testing.

**Other limitations:**

- No wheel suspension; some turning inconsistency remains, as described above.
- The rover's dimensions are estimated, not measured from the real robot.
- The bundled world contains no obstacles; they are added manually in Gazebo.
- No path planning around obstacles, and no recovery from being fully boxed in.
- Tested with single static obstacles in an otherwise empty world, not in
  cluttered or dynamic environments.
