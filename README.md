# obstacle_avoider — Autonomous Obstacle Avoidance & Goal Navigation (ROS 2 Humble)

A ROS 2 package that drives a simulated TurtleBot3 from its current position to a
goal point, while detecting and avoiding obstacles along the way using LiDAR.

Built as part of the Egrobots ROS 2 Week Task (Autonomous Obstacle Avoidance).

---

## 1. Overview

The robot continuously reads its LiDAR scan (`/scan`) and its odometry (`/odom`).
On each scan cycle it applies two behaviors in priority order:

1. **Obstacle avoidance (highest priority).** If an obstacle enters a forward-facing
   detection cone within `safe_distance`, the robot commits to turning in place
   toward the side with more open space, and keeps turning until the path is
   clearly clear (past `clear_distance`) — not just barely clear — before resuming.
2. **Goal navigation.** When no obstacle is blocking the path, the robot steers
   toward a target position (`goal_x`, `goal_y`) using proportional heading
   control, slowing its forward speed while turning sharply and speeding up once
   roughly facing the goal. It stops once within `goal_tolerance` of the goal.

The behavior can be started and stopped on demand via ROS 2 services, and every
tunable threshold is exposed as a ROS 2 parameter rather than hardcoded.

---

## 2. Package Contents

| File | Purpose |
|---|---|
| `obstacle_avoider/obstacle_avoider_node.py` | Main node: avoidance + goal navigation, parameters, start/stop services |
| `obstacle_avoider/manual_controller.py` | Keyboard teleop node for manually driving the robot (testing/demo aid) |
| `obstacle_avoider/scan_subscriber.py` | Early learning-step node: LiDAR subscriber only (not part of the final behavior) |
| `obstacle_avoider/velocity_publisher.py` | Early learning-step node: constant-forward publisher only (not part of the final behavior) |
| `config/obstacle_avoider_params.yaml` | All tunable parameters for `obstacle_avoider_node`, in one place |
| `launch/obstacle_avoidance.launch.py` | Launch file that starts Gazebo (TurtleBot3) + the obstacle avoider node, loading parameters from the YAML file |

`scan_subscriber` and `velocity_publisher` were built first, separately, to
understand ROS 2 pub/sub before combining both into `obstacle_avoider_node`. They
are kept in the package as a record of that process but aren't required to run
the final application.

---

## 3. Build & Run — From a Clean Workspace

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-gazebo-ros-pkgs
```

Set the TurtleBot3 model (once, persists via `.bashrc`):
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
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
ros2 launch obstacle_avoider obstacle_avoidance.launch.py
```

This launches Gazebo (empty world, TurtleBot3 spawned) and starts
`obstacle_avoider_node` with all parameters loaded from
`config/obstacle_avoider_params.yaml`. The node starts **disabled** by design,
so behavior must be explicitly started:

```bash
ros2 service call /start_avoidance std_srvs/srv/Trigger
```

To stop at any time (robot halts immediately):
```bash
ros2 service call /stop_avoidance std_srvs/srv/Trigger
```

### Setting a goal / tuning parameters
Edit `config/obstacle_avoider_params.yaml` before launching, or change values
live while the node is running:
```bash
ros2 param set /obstacle_avoider_node goal_x 1.5
ros2 param set /obstacle_avoider_node goal_y -1.0
ros2 param set /obstacle_avoider_node linear_speed 0.3
```
(Note: changing `goal_x`/`goal_y` after the goal was already reached requires a
`/stop_avoidance` then `/start_avoidance` to reset the reached-goal flag.)

### Manual driving (optional, for testing)
```bash
ros2 run obstacle_avoider manual_controller
```
Controls: `w`/`s` forward/backward, `a`/`d` turn left/right, `x` stop, `Ctrl+C` quit.
Do not run this at the same time as `obstacle_avoider_node` while it's enabled —
both publish to `/cmd_vel` and will conflict.

---

## 4. ROS 2 Components

### Node: `obstacle_avoider_node`

**Subscribes to:**
| Topic | Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR distance readings, used to detect obstacles ahead |
| `/odom` | `nav_msgs/msg/Odometry` | Robot's current position and orientation, used for goal navigation |

**Publishes to:**
| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands (linear/angular) sent to the robot |

**Provides services:**
| Service | Type | Purpose |
|---|---|---|
| `/start_avoidance` | `std_srvs/srv/Trigger` | Enables avoidance + navigation behavior |
| `/stop_avoidance` | `std_srvs/srv/Trigger` | Disables the behavior and immediately publishes a zero `Twist` |

**Parameters** (all in `config/obstacle_avoider_params.yaml`):

| Parameter | Default | Meaning |
|---|---|---|
| `safe_distance` | `0.5` (m) | Distance below which an obstacle triggers avoidance |
| `clear_distance` | `0.7` (m) | Distance the path must exceed before avoidance ends and navigation resumes (kept higher than `safe_distance` to prevent rapid switching) |
| `linear_speed` | `0.2` (m/s) | Base forward speed while navigating toward the goal |
| `angular_speed` | `0.5` (rad/s) | Turning speed used while avoiding an obstacle, and the max turn rate while navigating |
| `cone_angle_deg` | `30.0` (deg) | Half-angle of the forward-facing obstacle detection cone |
| `goal_x` | `2.0` (m) | Target X position, in the `odom` frame |
| `goal_y` | `0.0` (m) | Target Y position, in the `odom` frame |
| `goal_tolerance` | `0.15` (m) | Distance within which the goal is considered reached |
| `heading_kp` | `1.5` | Proportional gain for turning to face the goal |

Parameters can be read/changed live while the node is running:
```bash
ros2 param list /obstacle_avoider_node
ros2 param get /obstacle_avoider_node linear_speed
ros2 param set /obstacle_avoider_node safe_distance 1.0
```

### Node: `manual_controller`
Publishes `geometry_msgs/msg/Twist` to `/cmd_vel` based on keyboard input, for
manual testing/demo purposes.

---

## 5. Design Decisions

**Avoidance always overrides navigation.** Every scan cycle checks for a nearby
obstacle first; goal-seeking logic only runs once the robot is not currently
avoiding something. This guarantees the robot never drives through an obstacle
just because the goal happens to be on the other side of it.

**Committed turn direction, chosen once per avoidance episode.** When avoidance
starts, the robot compares the closest distance on its left vs. right within the
detection cone and locks in a turn direction toward the side with more open
space — it does not re-evaluate direction every cycle. An earlier version picked
direction fresh every scan, which caused visible left-right jitter as the
closest-obstacle angle flickered slightly frame to frame.

**Hysteresis between `safe_distance` and `clear_distance`.** Avoidance mode does
not end the instant the obstacle edge leaves the detection cone — it requires
distance to exceed a separate, larger `clear_distance` threshold. Without this,
the robot would flip back to forward motion prematurely (mid-turn, obstacle
still effectively in the way) and immediately re-trigger avoidance, causing
repeated near-collisions before the two thresholds were separated.

**Forward-facing cone, not the full 360° scan.** Restricting obstacle checks to
a `cone_angle_deg` cone in front of the robot (rather than the closest point in
the entire scan) avoids reacting to objects beside or behind the robot that are
not actually in its path.

**Proportional heading control for goal navigation.** Turn rate scales with how
far off the robot's heading is from the goal direction (`heading_kp *
heading_error`), rather than a fixed turn speed — this gives a smoother approach
than snapping between fixed left/right turns. Forward speed is additionally
scaled by `cos(heading_error)`, so the robot turns in place first when badly
misaligned, then accelerates once roughly facing the goal, instead of driving a
wide, wasteful arc.

**Start/stop as a service, not a topic.** `/start_avoidance` and
`/stop_avoidance` are one-shot, on/off commands with no ongoing data to stream —
`std_srvs/srv/Trigger` fits that better than a topic (meant for continuous data)
and needs no input arguments, matching the "just do this" nature of the command.

**Parameters over hardcoded constants, centralized in a YAML file.** All nine
tunables live in `config/obstacle_avoider_params.yaml` rather than scattered
through launch-file Python or hardcoded in the node. This keeps configuration as
data, separate from launch logic, and scales cleanly as more parameters get
added — editing a YAML line is simpler and less error-prone than editing a
Python dict inside a launch file.

---

## 6. Known Limitations / Possible Extensions

- Avoidance always turns toward whichever side currently has more clearance at
  the moment it triggers — it doesn't plan ahead or re-check mid-turn.
- No recovery for being fully boxed in (e.g., a dead end); would spin in place
  indefinitely.
- Goal navigation assumes a static goal in the `odom` frame; the robot doesn't
  replan a path around an obstacle toward the goal — it only turns away and
  resumes heading toward the same goal once clear, which can occasionally take
  a longer route than a full path planner would.
- Tested in an empty Gazebo world with single, static obstacles; not evaluated
  in cluttered or dynamic (moving-obstacle) environments.
- Manual keyboard control processes one key at a time, so simultaneous combined
  movement (e.g., forward + turn) isn't possible without adding explicit combo
  keys — a limitation of single-key polling, not of the robot's `Twist` command
  itself (which does support simultaneous linear + angular motion).
