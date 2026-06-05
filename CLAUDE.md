# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

- **Platform**: NVIDIA Jetson Orin (Ubuntu 22.04, Tegra kernel), **ROS 2 Humble**
- **Build system**: `colcon` with `ament_cmake` and `ament_python`
- The codebase was Frankensteined from the [sam_bot nav2 tutorial](https://github.com/arsalan-anwari/sam_bot) which was written for Foxy — some patterns (diff drive plugin names, Gazebo API) may be Foxy-era and need Humble equivalents

## Build & Run

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build all packages (from workspace root)
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash

# Build a single package
colcon build --symlink-install --packages-select <package_name>
```

> **`--symlink-install` gotcha:** it creates a per-file symlink in `install/` for each file that exists **at build time**. So *editing* an already-installed file (a launch/config/yaml) takes effect live with no rebuild — but **adding a new file** under `config/`, `launch/`, or `maps/` produces no symlink until you rebuild that package (`colcon build --symlink-install --packages-select sam_bot_bringup`). Symptom of forgetting: the launch can't find the new RViz/param/map file even though it's in `src/`.

### Launch modes

**Simulation (Gazebo + SLAM + Nav2)**
```bash
ros2 launch sam_bot_bringup bringup.launch.py
```
Uses `use_sim_time:=true`, delayed sequence: Gazebo (0s) → laserscan merger (10s) → SLAM (15s) → Nav2 (20s).

**Real hardware — SLAM (build a new map)**
```bash
ros2 launch sam_bot_bringup real_lidar_slam.launch.py [use_uart_odom:=true]
```
Save the map when done: `ros2 run nav2_map_server map_saver_cli -f <map_name>`

**Real hardware — AMCL (navigate on existing map)**
```bash
ros2 launch sam_bot_bringup real_lidar_amcl.launch.py [use_rviz:=true] [use_uart_odom:=true]
```
Loads `src/sam_bot_bringup/maps/web_save.yaml` by default. Startup order: lidars + uart_odom (0s) → laserscan merger (3s) → map_server + AMCL (5s).

**Real hardware — full Nav2 navigation (localization + planner + controller + behaviors + BT)**
```bash
ros2 launch sam_bot_bringup real_lidar_navigate.launch.py [use_rviz:=true] [use_uart_odom:=true]
```
This is the primary autonomous-navigation entry point — see the dedicated section below.

**LiDARs only (debug)**
```bash
ros2 launch ldlidar_node ldlidar_dual_with_mgr.launch.py
```

## Architecture

```
Embedded MCU (UART /dev/ttyTHS1)
        │ EmbeddedOdometryMessage (packed struct: magic, x, y, theta)
        ▼
   uart_odom_node  ──────────────────────────────────────────────────►  /odom  (odom frame)
        │                                                                  │
        │  also publishes /embedded_odom (raw embedded frame)              │
        │  + static TF: embedded_odom → odom (-90° z rotation)            ▼
        │                                                           odom → base_link TF
        │
LD19 left (/dev/lidar_left)  ──► /lidar_left/ldlidar_node/scan ─┐
LD19 right (/dev/lidar_right) ─► /lidar_right/ldlidar_node/scan ─┤
                                                              ▼
                                           multi-laserscan-toolbox-ros2
                                                              │
                                                              ▼
                                                           /scan  (base_link frame, 360°)
                                                              │
                                               ┌─────────────┴──────────────┐
                                               ▼                            ▼
                                          SLAM Toolbox               AMCL + map_server
                                        (map → odom TF)            (localization only)
```

### Key packages

| Package | Role |
|---|---|
| `uart_odom` | Polls embedded MCU at 100 Hz via UART; publishes `/odom` + `odom→base_link` TF |
| `dual-ldlidar` | Third-party driver for LD19 LiDARs; runs as lifecycle-managed component nodes |
| `multi-laserscan-toolbox-ros2` | Merges two `LaserScan` topics into one `/scan` in `base_link` frame |
| `sam_bot_bringup` | All launch files, nav2/slam/amcl/laserscan configs, and the saved map |
| `sam_bot_description` | Robot URDF (originally tutorial-grade, Gazebo sim only) |

## Full Nav2 navigation stack (`real_lidar_navigate.launch.py`)

This is the main launch file for autonomous navigation on the real robot and the one most actively worked on. It reuses the AMCL launch's localization tier and adds the Nav2 navigation tier on top.

**Files that define it:**
- Launch: [`launch/real_lidar_navigate.launch.py`](src/sam_bot_bringup/launch/real_lidar_navigate.launch.py)
- Params: [`config/nav2_params_real_lidar.yaml`](src/sam_bot_bringup/config/nav2_params_real_lidar.yaml) — **one file holds both localization and navigation sections.** The AMCL-only launch reads the same file and ignores the nav sections.
- RViz: [`config/real_lidar_navigate.rviz`](src/sam_bot_bringup/config/real_lidar_navigate.rviz) — navigate-specific view (costmaps, footprint, global/local plans). The AMCL launch uses its own `real_lidar_amcl.rviz`; **keep these two RViz configs separate.**
- Velocity bridge lives in [`uart_odom_node.py`](src/uart_odom/uart_odom/uart_odom_node.py) (not a separate node — see below).

**Lifecycle / startup order (TimerActions):** lidars + uart_odom (0s) → laserscan merger (3s) → `map_server`+`amcl`+`lifecycle_manager_localization` (5s) → `controller_server`+`planner_server`+`behavior_server`+`bt_navigator`+`waypoint_follower`+`lifecycle_manager_navigation` (8s). Two **separate** lifecycle managers: `_localization` and `_navigation`.

**This robot is holonomic (omnidrive) and its yaw is owned by the embedded MCU, not Nav2.** Consequences baked into the config:
- DWB is **translation-only**: `max_vel_theta: 0`, `vtheta_samples: 1`, and the heading critics (`RotateToGoal`, `GoalAlign`, `PathAlign`) are removed. Critics kept: `BaseObstacle, PathDist, GoalDist, Oscillation`.
- The goal checker ignores final heading: `SimpleGoalChecker` with `yaw_goal_tolerance: 3.15` (> π). Goals complete at any yaw — **do not "fix" this.**
- AMCL uses `nav2_amcl::OmniMotionModel` (not differential) so localization survives lateral/strafing moves.
- `robot_radius: 0.303` (circular footprint) on both costmaps; live `/scan` obstacle layers give reactive obstacle avoidance.

**Velocity → MCU bridge (in `uart_odom_node`, same process):** `controller_server` publishes `geometry_msgs/Twist` on `/cmd_vel`; `uart_odom_node` subscribes and forwards it over the **same** serial port it reads odometry from (a second node can't open `/dev/ttyTHS1`). Conversion to embedded frame (x=right, y=forward): `emb_vx = -linear.y`, `emb_vy = linear.x`; `angular.z` is **discarded**. Message is a packed `VelocityToEmbedded` struct (`magic='a'`, `type='v'`, `float32 vx, vy`). A watchdog sends zero velocity if `/cmd_vel` goes stale (>0.3 s). The robot will not move until the MCU firmware parses the `'v'` struct.

**Gotchas learned the hard way (don't reintroduce):**
- **Pass resolved string paths to localization nodes, not `LaunchConfiguration` substitutions inside parameter dicts.** `{"yaml_filename": LaunchConfiguration(...)}` left `map_server` with an empty filename → `on_configure` failed → whole localization tier stuck `unconfigured`, map never showed.
- **Do not remove `spin` from `behavior_server`.** The stock Humble `bt_navigator` default behavior trees reference the `/spin` action server; without it `bt_navigator` fails to activate and the navigation lifecycle manager stalls. `spin` is kept but is an inert no-op here (it commands ω, which the bridge discards).
- **Humble `behavior_server` param names** are `costmap_topic`/`footprint_topic` (singular) and `global_frame: odom` — not the newer `local_costmap_topic`/`local_frame` style.
- The old [`nav2_params.yaml`](src/sam_bot_bringup/config/nav2_params.yaml) is the **Foxy sim** config (`recoveries_server`, `use_sim_time: True`) — not used by real hardware; don't copy patterns from it.

See [`README.md`](README.md) "Real-hardware Nav2" for the full tuning table (DWB limits, AMCL alphas, dynamic-obstacle upgrade path via `nav2_collision_monitor`/MPPI).

## Coordinate Frame Convention

The embedded system uses a **different axis convention** than ROS:
- Embedded: `x = right`, `y = forward`, `z = up`
- ROS odom: `x = forward`, `y = left`, `z = up`

`uart_odom_node` applies this mapping in `publish_transformed_odom()`:
```python
odom_x = embedded_y       # forward → x
odom_y = -embedded_x      # right → -y
odom_theta = embedded_theta - π/2
```
A static TF `embedded_odom → odom` (−90° yaw) is also broadcast for any node that needs it.

## Serial Protocol (MCU ↔ uart_odom)

Both structs are `__attribute__((packed))`:
- **Query** (host → MCU): `{ magic: 'a', messageType: 'q' }` — 2 bytes
- **Response** (MCU → host): `{ magic, x: float32, y: float32, theta: float32 }` — 13 bytes

Serial port: `/dev/ttyTHS1`, 115200 baud. Timeout per read: 20 ms.

## LiDAR USB Device Naming

Both LD19 units report the same USB serial number (`0001`), so udev cannot distinguish them by identity. A udev rule in `src/sam_bot_bringup/udev/99-ldlidar.rules` matches by **physical USB port path** instead, creating stable symlinks:

| Symlink | Jetson USB port | LiDAR |
|---|---|---|
| `/dev/lidar_left` | `1-4.1` (currently `ttyUSB0`) | Left LD19 |
| `/dev/lidar_right` | `1-4.2` (currently `ttyUSB1`) | Right LD19 |

The rule must be installed on the OS (not just present in the repo):
```bash
sudo cp src/sam_bot_bringup/udev/99-ldlidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

If a LiDAR is moved to a different physical port, find the new port path with `udevadm info --name=/dev/ttyUSBx --attribute-walk | grep KERNELS` and update the `KERNELS==` value in the rules file. The driver YAMLs (`ldlidar_left.yaml`, `ldlidar_right.yaml`) reference the symlinks and never need to change. See `README.md` for the full procedure.

## Known Gaps / Open Design Questions

1. ~~No navigation stack on real hardware yet.~~ **Done** — `real_lidar_navigate.launch.py` + the navigation sections of `nav2_params_real_lidar.yaml` wire up planner, controller (DWB), behaviors, and BT navigator. See the "Full Nav2 navigation stack" section above. Remaining work is tuning (DWB limits, AMCL alphas) and firmware (MCU must parse the `'v'` velocity struct).

2. ~~Chassis is holonomic but AMCL uses `DifferentialMotionModel`.~~ **Done** — switched to `nav2_amcl::OmniMotionModel`. `alpha1..alpha5` were carried over from the differential tune and may still need re-tuning for the omni model.

3. **Command interface is velocity-based (decided).** Nav2 sends `geometry_msgs/Twist` on `/cmd_vel`; `uart_odom_node` converts to the embedded frame and sends a `VelocityToEmbedded` (`vx`, `vy`) packed struct over UART. Yaw is **not** commanded by Nav2 (owned by the MCU). The alternative "send goal coordinates over UART" approach was not taken.

4. **URDF (`sam_bot_description.urdf`) is Gazebo-oriented** (uses `libgazebo_ros_diff_drive`). A real-hardware URDF without Gazebo plugins, and with correct `lidar_left_link` / `lidar_right_link` frames, is still needed. Note navigation does **not** currently require it: costmap footprint comes from `robot_radius`, and the merged `/scan` is already in `base_link`, so no `robot_state_publisher`/`robot_description` runs on real hardware (the RViz `RobotModel` display will stay empty until one is added).

5. **LiDAR frame IDs in URDF vs. driver:** The driver uses `lidar_left_link` and `lidar_right_link` (set in `ldlidar_left.yaml`/`ldlidar_right.yaml`). The URDF must declare matching TF frames for the laserscan merger and AMCL to work correctly on real hardware.
