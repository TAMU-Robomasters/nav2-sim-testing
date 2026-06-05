# nav2-sim-testing

ROS 2 Humble navigation stack for a swerve-drive robot with dual LD19 LiDARs on a Jetson Orin.
See [CLAUDE.md](CLAUDE.md) for architecture details, coordinate frame conventions, and launch instructions.

---

## LiDAR USB Device Naming

Both LD19 units report the same USB serial number (`0001`), so the OS cannot distinguish them by identity alone — plug order determines which one becomes `ttyUSB0` vs `ttyUSB1`, and that can change on replug.

A udev rule at [`src/sam_bot_bringup/udev/99-ldlidar.rules`](src/sam_bot_bringup/udev/99-ldlidar.rules) solves this by matching each LiDAR to its **physical USB port** on the Jetson and creating stable symlinks:

| Symlink | Physical port | LiDAR |
|---|---|---|
| `/dev/lidar_left` | `1-4.1` | Left LD19 |
| `/dev/lidar_right` | `1-4.2` | Right LD19 |

### Installing the rule

```bash
sudo cp src/sam_bot_bringup/udev/99-ldlidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify it worked:
```bash
ls -la /dev/lidar_*
# /dev/lidar_left  -> ttyUSBx
# /dev/lidar_right -> ttyUSBx
```

### Changing which USB port a LiDAR lives on

If you move a LiDAR to a different physical USB port, the port path will change and the symlink will stop appearing. Fix it in three steps:

**1. Find the new port path**

Plug the LiDAR into its new port and run:
```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep KERNELS
```
(Use `ttyUSB1` if that's where it landed.) Look for a value like `1-4.3` — it's the third line down, right after the interface entry (`1-4.3:1.0`).

**2. Update the udev rule**

Edit [`src/sam_bot_bringup/udev/99-ldlidar.rules`](src/sam_bot_bringup/udev/99-ldlidar.rules) and replace the `KERNELS==` value for whichever LiDAR moved:

```
SUBSYSTEM=="tty", KERNELS=="1-4.X", ...  # change 1-4.X to the new path
```

Reinstall and reload:
```bash
sudo cp src/sam_bot_bringup/udev/99-ldlidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**3. No YAML changes needed**

The LiDAR driver configs ([`ldlidar_left.yaml`](src/dual-ldlidar/ldlidar_node/params/ldlidar_left.yaml), [`ldlidar_right.yaml`](src/dual-ldlidar/ldlidar_node/params/ldlidar_right.yaml)) reference `/dev/lidar_left` and `/dev/lidar_right` — the stable symlinks — so they don't need to change when you move ports.

---

## Real-hardware Nav2 — full navigation stack

Bring up the entire Nav2 stack (localization + planner + controller + behaviors + BT navigator) on the real robot:

```bash
ros2 launch sam_bot_bringup real_lidar_navigate.launch.py \
  [use_rviz:=true] [use_uart_odom:=true] [map:=/abs/path/to/map.yaml]
```

Startup order: lidars + UART node (0s) → laserscan merger (3s) → map_server + AMCL (5s) → controller + planner + behaviors + bt_navigator (8s). Send a goal with the **2D Nav Goal** tool in RViz or the `NavigateToPose` action.

All navigation parameters live in [`nav2_params_real_lidar.yaml`](src/sam_bot_bringup/config/nav2_params_real_lidar.yaml) (the same file used by the AMCL-only launch — the navigation sections are simply ignored there).

### How motion reaches the robot

`controller_server` (DWB) publishes `geometry_msgs/Twist` on `/cmd_vel`. The **`uart_odom_node`** subscribes to `/cmd_vel` and forwards it to the MCU over the *same* serial port it reads odometry from (`/dev/ttyTHS1`) — a separate node can't be used because the port is opened exclusively. base_link velocities are converted to the embedded frame (`x=right, y=forward`):

```
emb_vx (right)   = -cmd_vel.linear.y
emb_vy (forward) =  cmd_vel.linear.x
cmd_vel.angular.z is DISCARDED   # yaw is owned by the embedded controller
```

The velocity message is a packed struct the **MCU firmware must parse**:

| field | type | value |
|---|---|---|
| `magic` | `uint8` | `'a'` |
| `messageType` | `uint8` | `'v'` |
| `vx` | `float32` | embedded +x (right), m/s |
| `vy` | `float32` | embedded +y (forward), m/s |

> **Safety:** the node runs a watchdog — if `/cmd_vel` goes stale for >`cmd_vel_timeout` (default 0.3 s) it sends a zero `'v'`. The firmware should likewise treat a zero `'v'` **and** any comms loss as "stop."

### Tuning knobs & concerns

| Setting (in `nav2_params_real_lidar.yaml`) | Current | Notes |
|---|---|---|
| `robot_radius` (both costmaps) | `0.303` | = 0.605 m measured diameter ÷ 2. Re-measure if bumpers/lidar overhang change the real footprint — drives inflation and how close it plans to walls. |
| DWB `max_vel_x/y`, `max_speed_xy` | `0.5` | Placeholder = max translational speed. Set to the robot's real limit. |
| DWB `acc_lim_x/y`, `decel_lim_x/y` | `1.0 / -1.0` | Placeholders. Too low → sluggish; too high → jerky/overshoot. |
| AMCL `alpha1..alpha5` | `0.2` | **Were tuned for the differential model.** AMCL now uses `OmniMotionModel` (correct for this holonomic robot) — these likely need re-tuning. |
| Goal checker `xy_goal_tolerance` | `0.25` m | How close to the goal *position* counts as arrived. |
| Goal checker `yaw_goal_tolerance` | `3.15` rad | **> π on purpose** → final heading is ignored. The robot's yaw is controlled by the embedded algorithm, so the goal completes at *any* heading. Don't "fix" this. |
| Planner `tolerance` | `0.5` m | NavFn *fallback* radius used only when the exact goal cell is blocked — **not** the arrival distance (that's `xy_goal_tolerance`). |

**Yaw is decoupled by design.** DWB is configured translation-only (`max_vel_theta: 0`, `vtheta_samples: 1`) and the heading critics (`RotateToGoal`, `GoalAlign`, `PathAlign`) are removed, so Nav2 never tries to rotate the robot during path following.

**The `spin` recovery is a no-op, not removed.** It commands ω, which the UART bridge discards, so the robot won't physically rotate during a spin recovery. It must still be present in `behavior_server`, though: the stock `bt_navigator` default behavior trees reference the `/spin` action server, and if it doesn't exist, `bt_navigator` fails to load those trees and **refuses to activate** (lifecycle stays `inactive`). To genuinely drop spin you'd have to point `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` at custom spin-free behavior-tree XMLs.

### Dynamic obstacle avoidance

What works **now** (reactive): both costmaps have an `obstacle_layer` fed by the live merged `/scan` with marking *and* raytrace clearing, DWB re-evaluates trajectories against the local costmap at 20 Hz, and the BT replans the global path — so anything the lidars see (including something moving into the path) is avoided.

What this does **not** do: predict where moving obstacles are heading. To go further later:
- raise local costmap `update_frequency` and tune inflation,
- add [`nav2_collision_monitor`](https://docs.nav2.org/configuration/packages/collision_monitor/index.html) as a hard, controller-independent safety stop,
- or swap DWB for the **MPPI** controller (ships in Humble, handles dynamics far better than DWB).

### Gotchas

- **`use_sim_time` must be `false`** for every node on real hardware (it is, throughout `nav2_params_real_lidar.yaml`).
- The default BT XML is left unset so `bt_navigator` uses the Humble built-in defaults — do **not** point it at the old Foxy `navigate_w_replanning_and_recovery.xml`.
- The merged `/scan` is already in `base_link`, so no URDF/lidar-link TFs are needed for navigation (the costmap footprint comes from `robot_radius`, not the URDF).
