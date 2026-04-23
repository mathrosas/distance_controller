# Checkpoint 17 — Distance Controller

ROS 2 C++ **PID position controller** for the **Husarion ROSBot XL** (4-wheel mecanum / holonomic). The node drives the robot through a chain of relative `(dx, dy)` waypoints **expressed in the body frame** by running two independent PID loops on the body-frame position errors `ex_b`, `ey_b`. It subscribes to the EKF-fused odometry, publishes `geometry_msgs/Twist` on `/cmd_vel`, and works against both the Gazebo simulation and the real CyberWorld ROSBot XL — the waypoint list **and** the PID gain set are selected from a **scene number** passed as a CLI argument.

<p align="center">
  <img src="media/waypoints-sim.png" alt="ROSBot XL distance-controller waypoint trace in simulation" width="650"/>
</p>

## How It Works

<p align="center">
  <img src="media/gazebo-world.png" alt="ROSBot XL spawned in the Gazebo empty world (top view)" width="600"/>
</p>

### Control Loop

1. A single-node executable `distance_controller` subscribes to `/odometry/filtered` (`nav_msgs/Odometry`) and publishes `geometry_msgs/Twist` on `/cmd_vel`. Odometry and timer callbacks share a **Reentrant callback group** so they can run concurrently on a `MultiThreadedExecutor`.
2. On construction, `setup_scene(scene_number)` selects the waypoint list **and** the PID gains — `1` for Gazebo, `2` for CyberWorld
3. The first odom message latches the initial pose and spawns a **40 Hz wall-timer** (`dt = 25 ms`) that runs the control loop
4. When a new segment starts, the relative body-frame displacement `(wp.dx, wp.dy)` is rotated by the **current yaw** to build an absolute world-frame target `(segment_target_x, segment_target_y)`:
   - `dx_world =  dx·cos(φ) − dy·sin(φ)`
   - `dy_world =  dx·sin(φ) + dy·cos(φ)`
5. Per tick:
   - World-frame error `(ex, ey) = (target − current)`
   - Rotated back into the **body frame**: `ex_b =  cos(φ)·ex + sin(φ)·ey`, `ey_b = −sin(φ)·ex + cos(φ)·ey`
   - Velocity error uses the odometry twist directly: `dex = 0 − v_x`, `dey = 0 − v_y`
   - Two independent PIDs compute `(linear.x, linear.y)` in body frame; `angular.z = 0`
   - The `(linear.x, linear.y)` vector is renormalised so `‖v‖ ≤ max_speed_`
6. A waypoint is considered reached when **both** `‖(ex_b, ey_b)‖ < 0.01 m` **and** `‖(v_x, v_y)‖ < 0.01 m/s`. The node then zeroes the twist, pauses `1.0 s`, resets integral accumulators and moves to the next waypoint
7. `/cmd_vel` is held at zero while no subscribers are connected, and integral accumulators are reset so the controller doesn't wind up while waiting for the driver

### PID Configuration

Gains are per-axis (independent on `x` and `y`) and differ between sim and CyberWorld:

| Parameter | Scene 1 (Simulation) | Scene 2 (CyberWorld) |
|---|---|---|
| `Kp_x`, `Kp_y` | `2.5` | `2.0` |
| `Ki_x`, `Ki_y` | `0.005` | `0.001` |
| `Kd_x`, `Kd_y` | `0.3` | `0.3` |
| Max linear speed `max_speed_` | `0.8 m/s` | `0.45 m/s` |
| Position tolerance | `0.01 m` | `0.01 m` |
| Velocity tolerance | `0.01 m/s` | `0.01 m/s` |
| Control rate | `40 Hz` (`dt = 25 ms`) | `40 Hz` |
| Inter-waypoint pause | `1.0 s` | `1.0 s` |

The derivative term uses the **measured body-frame velocity** from the odometry twist rather than the discrete error delta — this gives a cleaner D-term and lets the position P and velocity D loops share a single PID structure.

## Waypoint Scenes

All waypoints are relative `(dx, dy, dyaw)` triplets **in the body frame** — the third element is unused by this node and always `0.0`.

### Scene 1 — Simulation (`scene_number = 1`, 10 waypoints)

```
(0, 1), (0,-1), (0,-1), (0, 1),
(1, 1), (-1,-1), (1,-1), (-1, 1),
(1, 0), (-1, 0)
```

### Scene 2 — CyberWorld (`scene_number = 2`, 4 waypoints)

```
(1.0, 0.0), (0.0, -0.55), (0.0, 0.55), (-1.0, 0.0)
```

Net displacement in both scenes is `(0, 0)` — the robot returns to its starting pose, which makes a closed-loop odometry-drift check trivial.

## Real Robot Deployment (CyberWorld)

<p align="center">
  <img src="media/waypoints-real.png" alt="Real ROSBot XL distance-controller waypoint trace recorded in CyberWorld" width="650"/>
</p>

The same executable runs **unmodified** on the real Husarion ROSBot XL in The Construct's **CyberWorld** lab — only the scene number changes. Scene `2` tightens the gains, caps the speed, and swaps the waypoint list for a `1.0 m × 0.55 m` rectangle that fits the physical arena:

1. The ROSBot XL real-robot stack (`rosbot_xl_ros` + its EKF + wheel controllers) is already running on the physical robot; `/odometry/filtered` is served over the CyberWorld connection
2. The `distance_controller` node is launched locally with `scene_number = 2`:

   ```bash
   ros2 run distance_controller distance_controller 2
   ```
3. Closed-loop tracking uses the **same EKF-fused odometry topic** (`/odometry/filtered`) that the sim subscribes to — the controller is feedback-source agnostic
4. Scene-specific tuning for the real robot:
   - Lower `Kp` (`2.0` vs `2.5`) and lower `Ki` (`0.001` vs `0.005`) damp mecanum-wheel slip
   - `max_speed_ = 0.45 m/s` keeps the robot well under the `0.8 m/s` platform limit and avoids overshoot on the `1.0 m` first segment
   - Same `0.01 m` position tolerance and `0.01 m/s` velocity tolerance as sim — the combined gate prevents declaring a waypoint reached while the robot is still coasting

### Sim ↔ real parity

| Concern | Simulation (scene 1) | Real CyberWorld (scene 2) |
|---|---|---|
| Feedback topic | `/odometry/filtered` | `/odometry/filtered` |
| Waypoint count | 10 | 4 |
| Max segment length | `√2 m` | `1.0 m` |
| PID gains | `Kp=2.5, Ki=0.005, Kd=0.3` | `Kp=2.0, Ki=0.001, Kd=0.3` |
| Max linear speed | `0.8 m/s` | `0.45 m/s` |
| Position tolerance | `0.01 m` | `0.01 m` |
| Velocity tolerance | `0.01 m/s` | `0.01 m/s` |
| Clock | sim time | wall clock |

## ROS 2 Interface

| Name | Type | Description |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` (sub) | EKF-fused odometry — pose (`x`, `y`, yaw) and twist (`v_x`, `v_y`) |
| `/cmd_vel` | `geometry_msgs/Twist` (pub) | Body-frame command (`linear.x`, `linear.y`, `angular.z = 0`) |

## Project Structure

```
distance_controller/
├── src/
│   └── distance_controller.cpp
├── include/
├── media/
├── CMakeLists.txt
└── package.xml
```

## How to Use

### Prerequisites

- ROS 2 Humble
- Gazebo (bundled with the `rosbot_xl_gazebo` simulation)
- `tf2`, `nav_msgs`, `geometry_msgs`
- `rosbot_xl_ros` stack in the same workspace (description + controllers + EKF)

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select distance_controller --symlink-install
source install/setup.bash
```

### Simulation

```bash
# Terminal 1 — ROSBot XL in Gazebo
ros2 launch rosbot_xl_gazebo simulation.launch.py

# Terminal 2 — PID distance controller (scene 1 = simulation waypoint set)
ros2 run distance_controller distance_controller 1
```

The scene argument is optional — omitting it defaults to scene `1` (simulation).

### Real robot (CyberWorld)

```bash
ros2 run distance_controller distance_controller 2
```

### Sanity checks

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odometry/filtered
```

## Key Concepts Covered

- **Per-axis PID in the body frame**: two independent loops on `ex_b`, `ey_b` — simpler than the scalar-distance + projection formulation, and maps naturally to holonomic command channels
- **Body-frame waypoints**: displacements are evaluated in the robot's body frame at segment start, then converted to absolute world-frame targets via the current yaw
- **Velocity-feedback D-term**: derivative uses the measured twist (`v_x`, `v_y`) rather than a discrete error difference — cleaner and noise-tolerant
- **Position + velocity arrival gate**: both `‖error‖` and `‖velocity‖` must drop below their tolerances before advancing, avoiding premature declaration at turnaround points
- **Multi-threaded executor**: Reentrant callback group lets odom callbacks and the timer interleave without blocking
- **Scene-specific tuning**: gain set, speed cap and waypoint list all switch on a single CLI argument

## Technologies

- ROS 2 Humble
- C++ 17 (`rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2`)
- Husarion ROSBot XL (4-wheel mecanum) in Gazebo Sim + CyberWorld
