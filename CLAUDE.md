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
ros2 launch sam_bot_bringup real_lidar_navigate.launch.py [use_rviz:=true] [use_uart_odom:=true] [enable_mcu_bridge:=true] [map:=/abs/path.yaml] [use_keepout:=true]
```
This is the primary autonomous-navigation entry point — see the dedicated section below.
Pass `enable_mcu_bridge:=true` to also serve the AutoX auto-aim app over the same
UART (transform query + firing solution) — see "MCU bridge for AutoX" below.
Pass `map:=` to load a different map (default `maps/3v3_field.yaml`), and
`use_keepout:=false` to run on a map that has **no** matching keepout mask — see
"Configurable map / keepout toggle" below.

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
| `uart_odom` | Polls embedded MCU at 100 Hz via UART; publishes `/odom` + `odom→base_link` TF. With `enable_mcu_bridge:=true` it also owns the AutoX MCU bridge (transform service + firing-solution sink) on the same port |
| `mcu_msgs` | Interface package for the AutoX bridge: `srv/QueryTransform`, `msg/FiringSolution` |
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

#### Configurable map / keepout toggle

`real_lidar_navigate.launch.py` takes two args for swapping maps:
- **`map:=/abs/path.yaml`** — map served by `map_server` (default `maps/3v3_field.yaml`).
- **`use_keepout:=false`** — run **without** the keepout-zone filter, for a map that has no matching keepout mask. Default `true` (today's behavior unchanged).

Because these need *resolved* values (see the empty-`yaml_filename` gotcha above, and because starting/omitting the filter servers is a structural choice), the launch body is wrapped in an **`OpaqueFunction`** so `map` is a real string and `use_keepout` a real bool. What `use_keepout:=false` changes:
- The two filter servers (`filter_mask_server`, `costmap_filter_info_server`) are **not** started.
- `lifecycle_manager_localization`'s `node_names` is overridden to `['map_server', 'amcl']` (leaf-param dict layered after the YAML — same mechanism as `yaml_filename`). **This is the load-bearing part:** leaving the two filter servers in `node_names` while they aren't running stalls the whole localization bringup waiting to configure absent nodes.
- [`config/keepout_off.yaml`](src/sam_bot_bringup/config/keepout_off.yaml) is layered onto `planner_server`, setting `keepout_filter: enabled: false`. The filter plugin is nested under `global_costmap.global_costmap.ros__parameters`, so it can't be reached with a per-node dict — hence the tiny second params file. It disables the plugin rather than blanking `filters: []` (an empty list trips rclcpp's array type inference).

**Gotchas (don't reintroduce):**
- **`keepout_off.yaml` is a *new* file under `config/`**, so `colcon build --symlink-install --packages-select sam_bot_bringup` is required once before `use_keepout:=false` works (the `--symlink-install` new-file caveat at the top of this file). Editing the existing launch/YAMLs takes effect live; the new file does not.
- Keep `use_keepout` default `true` — the committed `3v3_field` map ships with a matching `keepout_mask`, so the default path must stay the keepout-on flow.
- A new map's keepout mask (if any) must share the map's `origin` and `resolution` or the no-go zones land in the wrong place. A mask smaller than the map is fine — uncovered area is treated as free.

### Waypoint patrol script (`square_patrol.py`)

A standalone client that drives the robot in a repeating loop of waypoints (a square by default) using the `nav2_simple_commander` `BasicNavigator` API. It does **not** launch anything — start the nav stack first, then run the script in a second terminal:

```bash
# terminal 1: bring up the full stack
ros2 launch sam_bot_bringup real_lidar_navigate.launch.py use_rviz:=true
# terminal 2: run the patrol (blocks at waitUntilNav2Active until the stack is ACTIVE)
ros2 run sam_bot_bringup square_patrol.py
```

- File: [`scripts/square_patrol.py`](src/sam_bot_bringup/scripts/square_patrol.py). Installed via `install(PROGRAMS … DESTINATION lib/${PROJECT_NAME})` in [`CMakeLists.txt`](src/sam_bot_bringup/CMakeLists.txt) — that `lib/` dir (not `share/`) is what `ros2 run` searches. Edit `SQUARE_CORNERS` at the top to set real map coordinates.
- **It calls `setInitialPose((0,0,0))` to match `amcl`'s `set_initial_pose: true` in the YAML.** `waitUntilNav2Active(localizer='amcl')` internally republishes the navigator's initial pose to `/initialpose` and blocks until it sees `/amcl_pose`; setting it to the same (0,0,0) the YAML uses keeps the two from fighting (and avoids an empty-`frame_id` pose that AMCL would reject, hanging the wait forever).
- Waypoint **orientation is irrelevant** (holonomic robot, `yaw_goal_tolerance > π`) — poses use identity quaternions.

**Gotchas (don't reintroduce):**
- **New scripts need `chmod +x` on the source file.** With `--symlink-install`, the installed entry is a symlink back to `src/…/scripts/`, so `ros2 run` honors the *source* file's mode. A non-executable script gives `No executable found` even though it built fine. The mode is tracked by git, so it's a one-time fix. (`ros2 pkg executables sam_bot_bringup` confirms it's discoverable.)
- **Adding the script is a *new* file**, so `colcon build --packages-select sam_bot_bringup` is required once before the first `ros2 run` (the `--symlink-install` rebuild caveat at the top of this file).
- For Python-only tooling that grows beyond one script, prefer a separate `ament_python` package over piling `install(PROGRAMS …)` lines into this `ament_cmake` bringup package.

### One-shot autoboot scripts (`autoboot.sh` / `autoboot_disable.sh`)

Two repo-root bash scripts wrap "bring up the whole stack, wait until it's ACTIVE, then start patrolling" into a single fire-and-forget command, plus a matching teardown. They are for unattended startup (rc.local / systemd / "just run it") — they do **not** replace the manual two-terminal flow above, which is still better for interactive tuning.

- [`autoboot.sh`](autoboot.sh): sources ROS + the overlay, launches `real_lidar_navigate.launch.py` **detached in its own process group** (`setsid`, so it survives the terminal), polls `ros2 lifecycle get` on `map_server, amcl, controller_server, planner_server, behavior_server, bt_navigator` until all report `active` (120 s timeout, aborts if the launch dies early), then starts `square_patrol.py` — also detached — and exits. PGIDs and logs live in `/tmp/sentry_autoboot/`. The two ROS commands live in an **`EDIT ME` block of `bash` arrays at the top** (`LAUNCH_CMD`, `PATROL_CMD`) — AutoX-`start`-style: to add a launch arg (`map:=…`, `use_keepout:=false`) just add/uncomment a line (one arg per line, no trailing backslashes). Both commands run through one `spawn_detached` helper that passes the argv array through verbatim, so args with `${VAR:-default}` substitutions survive intact. Set **`RUN_PATROL=false`** (or edit the flag in the block) to bring the stack up to ACTIVE but skip `square_patrol.py` — verifies the launch **without driving the robot**. Verified end-to-end on real hardware (ACTIVE in ~21 s, `/cmd_vel` at 10 Hz, clean teardown).
- [`autoboot_disable.sh`](autoboot_disable.sh): reads the PGID files and tears down patrol first, then the launch, escalating `SIGINT → SIGTERM → SIGKILL` per **process group**; finishes with a `pkill -f` fallback sweep (`square_patrol.py`, `real_lidar_navigate.launch.py`, `nav2_`, `ldlidar`, `uart_odom`, `laserscan_toolbox`) for anything orphaned when the pidfiles are gone.

```bash
./autoboot.sh                       # headless (RViz off), MCU bridge off, then patrol
USE_RVIZ=true ./autoboot.sh         # env-overridable: USE_RVIZ, USE_UART_ODOM,
ENABLE_MCU_BRIDGE=true ./autoboot.sh #   ENABLE_MCU_BRIDGE, ACTIVE_TIMEOUT, RUN_PATROL
RUN_PATROL=false ./autoboot.sh      # bring stack to ACTIVE but DON'T drive the robot
./autoboot_disable.sh               # stop patrol + stack
```

**Design points / gotchas (don't reintroduce):**
- **Defaults differ from the launch on one axis:** `use_rviz` defaults to `false` here (autoboot is headless) whereas the launch file defaults it to `true`. `use_uart_odom` (true) and `enable_mcu_bridge` (false) match the launch. All are env-overridable at the top of `autoboot.sh`.
- **Everything runs under `setsid`** so `kill -<sig> -<PGID>` (negative PID = whole group) cleanly stops `ros2 launch` *and* all the child nodes it spawned. Killing just the recorded launch PID would orphan the nodes.
- **The scripts hard-code `WS_ROOT=/home/orin/repos/nav2-sim-testing`** and `source /opt/ros/humble/setup.bash` + `install/setup.bash`. If the workspace moves or isn't built, `autoboot.sh` refuses to start (missing overlay check).
- **`autoboot.sh` refuses to double-start** if a live launch PGID is already recorded — run `autoboot_disable.sh` first.
- **The ROS `setup.bash` sourcing is wrapped in `set +u` / `set -u`.** `autoboot.sh` runs under `set -euo pipefail`, but ament's `setup.bash` references unbound vars (`AMENT_TRACE_SETUP_FILES`, …) and dies under `nounset` with `unbound variable`. Don't drop the `set +u` guard around the two `source` lines. (The inner `spawn_detached` `bash -c` re-sources ROS but has no `set -u`, so it needs no guard.)
- **`autoboot_disable.sh`'s fallback `pkill -f` sweep is broad** — patterns like `nav2_` match *any* process whose full command line merely contains the string, including an unrelated shell running a `grep` that mentions it. It's a last-resort sweep for orphans after the per-PGID kill, so this is usually harmless, but don't rely on it as the primary teardown and be wary running it next to commands that echo those names.
- Autoboot inherits the launch's default map (currently `maps/airbnb.yaml`, set in `real_lidar_navigate.launch.py`); to boot a different map, add a `map:=…` line to `LAUNCH_CMD` in the `EDIT ME` block. (The old `3v3_fields.yaml` plural-vs-singular mismatch is gone.)

#### systemd service — start on boot + `systemctl status` (`install_service.sh` / `uninstall_service.sh`)

For "is the stack up for the match?" checkability and start-on-boot/restart-on-crash, there's a real systemd unit `sentry_nav`, modeled on AutoX's `utils/setup_boot_script.sh` / `kill_onboot_cv`. It runs the **same** `autoboot.sh` (single source of truth for the editable command) with **`SERVICE_MODE=true`**.

```bash
./install_service.sh                    # install + enable + start (prompts for sudo)
RUN_PATROL=false ./install_service.sh   # same, but bring up stack WITHOUT driving (safe verify)
systemctl status sentry_nav             # <-- the "is the match running?" check
systemctl is-active sentry_nav          # active / inactive, scriptable
journalctl -u sentry_nav -f             # live logs (replaces /tmp/sentry_autoboot/*.log in service mode)
sudo systemctl restart sentry_nav       # after editing args in autoboot.sh's EDIT ME block
./uninstall_service.sh                  # stop + disable + remove
```

**How `SERVICE_MODE` differs from the plain `./autoboot.sh` run** (both in the one file):
- **Foreground, not detached.** The launch + patrol run as `setsid` children but `autoboot.sh` **blocks** (`wait` on the launch) instead of exiting, so systemd can track the process (`Type=simple`), capture output in the journal, and `Restart=always` on crash. Plain mode still detaches and exits (fire-and-forget) as before.
- **Teardown is a trap** (`cleanup_service` on EXIT/TERM/INT): on `systemctl stop`, systemd SIGTERMs the wrapper → the trap kills the child process groups **and** runs the same broad `pkill` sweep as `autoboot_disable.sh` (because `ros2 launch` reparents some nodes out of our group). systemd's own `KillMode=control-group` cgroup kill is the real backstop; the trap just keeps a *manual* `SERVICE_MODE=true` + Ctrl-C run from orphaning nodes. No pidfiles are written in service mode.

**Install/verify gotchas (learned the hard way):**
- **The unit bakes the options in as `Environment=` at install time** (`USE_RVIZ`, `USE_UART_ODOM`, `ENABLE_MCU_BRIDGE`, `RUN_PATROL`, `SERVICE_MODE=true`). To change the *map* or add a launch arg, edit `LAUNCH_CMD` in `autoboot.sh` and `systemctl restart`; to change one of the baked env knobs, re-run `install_service.sh` (optionally `RUN_PATROL=false ./install_service.sh`).
- **`ros2 node list` lies during teardown checks** — it's served from the ros2 daemon's discovery cache and shows *dead* nodes for a while after they exit. To verify a teardown really happened, check **processes** (`ps`/`pgrep`) or `ros2 daemon stop && ros2 daemon start` before listing. This burned a debugging cycle (looked like the trap failed when the stack was already gone).
- **`systemctl start` with the default `RUN_PATROL=true` drives the robot** (patrol starts as soon as the stack is ACTIVE). Verify the systemd wiring with `RUN_PATROL=false ./install_service.sh` first (stack comes up, no motion), then re-install with patrol on for the match.
- Do **not** add `After=graphical.target` to the unit (same trap AutoX documents): it's ordered after `multi-user.target` (our `WantedBy`), creating an ordering cycle that makes systemd silently drop the boot start job.
- Installing needs sudo (writes `/etc/systemd/system/sentry_nav.service`), so `install_service.sh` can't be run non-interactively without a passwordless sudo.

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

Odometry structs are `__attribute__((packed))`:
- **Query** (host → MCU): `{ magic: 'a', messageType: 'q' }` — 2 bytes
- **Response** (MCU → host): `{ magic, x: float32, y: float32, theta: float32 }` — 13 bytes

Serial port: `/dev/ttyTHS1`, **460800 baud** (raised from 115200 so the 69-byte
transform reply clocks in at ~1.5 ms; the MCU firmware must match). Timeout per
read: 20 ms.

## MCU bridge for AutoX (`enable_mcu_bridge`)

Only **one** process may own `/dev/ttyTHS1`, but on the sentry both nav (odometry +
`/cmd_vel`) and the AutoX auto-aim app need the MCU at the same time. So when
`enable_mcu_bridge:=true`, `uart_odom_node` is the **sole serial owner** and also
serves AutoX's MCU traffic over ROS:

- **`/mcu/query_transform`** (`mcu_msgs/srv/QueryTransform`) — AutoX requests the
  camera→ballistic transform (`frame_delay_ms` → `success, yaw, pitch, matrix[16]`).
  The service callback does the serial round-trip **synchronously** (writes a 3-byte
  `TransformQuery` `messageType 't'`, reads the 69-byte `EmbeddedTransformationMessage`).
- **`/mcu/firing_solution`** (`mcu_msgs/msg/FiringSolution`) — fire-and-forget sink;
  the callback writes a 12-byte `JetsonMessage` (`messageType 'd'`) to the MCU.

All callbacks run in the node's default `MutuallyExclusiveCallbackGroup` on the
single-threaded executor, so the synchronous transform service can never overlap the
100 Hz odom poll or `/cmd_vel` writes on the one serial line — no lock needed. The
transform query (`'t'`) and odom query (`'q'`) coexist on the wire; the MCU dispatches
by `messageType`. The flag is **off by default**, so odom-only mode is unchanged.

The matching `MCU=ROS` consumer lives in AutoX (`src/drivers/mcu.py` `RosBackend`).
Run order and the full picture are in the AutoX repo's `README.md`.

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
