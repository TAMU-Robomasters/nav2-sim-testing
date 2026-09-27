"""
Real LiDAR Navigation Launch (full Nav2 stack)
AMCL localization + planner + controller + behaviors + BT navigator on real
hardware using dual LD19 LiDARs. No Gazebo. Holonomic (omnidrive) robot whose
yaw is controlled by the embedded MCU, so Nav2 only commands translation.

Builds on real_lidar_amcl.launch.py and adds the navigation stack.

Launch sequence:
  0s  - LDLidar dual nodes + their lifecycle manager
  0s  - UART node (odom -> base_link TF + /odom; also /cmd_vel -> UART velocity)
  3s  - Multi-laserscan toolbox (merges lidar scans -> /scan in base_link)
  5s  - map_server + amcl + localization lifecycle manager
        (+ filter servers only when use_keepout:=true)
  8s  - controller + planner + behavior + bt_navigator + waypoint_follower
        + navigation lifecycle manager

Configurable at launch:
  map:=/abs/path/to/map.yaml   pick a different map (default: maps/airbnb.yaml)
  use_keepout:=false           run without the keepout-zone filter (default: true)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Build the launch actions with resolved config values.

    Wrapped in an OpaqueFunction so `map` and `use_keepout` are available as a
    real string / real bool here. That matters for two reasons documented in
    CLAUDE.md: (1) map_server needs a *resolved string* yaml_filename — a raw
    LaunchConfiguration substitution in the param dict comes through empty and
    stalls the localization tier; (2) whether to start the keepout filter
    servers (and how to set the localization lifecycle manager's node_names) is
    a structural decision that's far cleaner with a plain Python bool than with
    If/Unless-gated duplicate nodes.
    """
    bringup_share = get_package_share_directory("sam_bot_bringup")

    # ---------- Launch arguments (substitutions) ----------
    use_rviz = LaunchConfiguration("use_rviz")
    use_uart_odom = LaunchConfiguration("use_uart_odom")
    enable_mcu_bridge = LaunchConfiguration("enable_mcu_bridge")
    rviz_config = LaunchConfiguration("rviz_config")

    # ---------- Resolved (concrete) values ----------
    map_yaml_path = LaunchConfiguration("map").perform(context)
    use_keepout = (
        LaunchConfiguration("use_keepout").perform(context).lower()
        in ("true", "1", "yes")
    )

    # ---------- Config / map paths ----------
    # Resolved to plain strings (mirrors real_lidar_amcl.launch.py). Passing
    # these as LaunchConfiguration substitutions inside parameter dicts can leave
    # map_server with an empty yaml_filename, which makes on_configure fail and
    # the whole localization chain stall 'unconfigured'.
    nav2_params_path = str(
        Path(bringup_share) / "config" / "nav2_params_real_lidar.yaml"
    )
    keepout_mask_path = str(Path(bringup_share) / "maps" / "keepout_mask.yaml")
    keepout_off_path = str(Path(bringup_share) / "config" / "keepout_off.yaml")
    laserscan_params_path = str(
        Path(bringup_share) / "config" / "laserscan_toolbox_real_lidar_params.yaml"
    )

    # ---------- 0. LDLidar dual nodes + lifecycle manager (immediate) ----------
    ldlidar_dual_with_mgr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ldlidar_node"))
                / "launch"
                / "ldlidar_dual_with_mgr.launch.py"
            )
        )
    )

    # ---------- 1. UART node (optional, immediate) ----------
    # Reads pose from MCU (-> /odom + odom->base_link TF) AND forwards /cmd_vel
    # to the MCU as velocity commands over the same serial port.
    uart_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("uart_odom"))
                / "launch"
                / "uart_odom.launch.py"
            )
        ),
        launch_arguments={"enable_mcu_bridge": enable_mcu_bridge}.items(),
        condition=IfCondition(use_uart_odom),
    )

    # ---------- 2. Multi-laserscan toolbox (3s delay) ----------
    laserscan_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("multi-laserscan-toolbox-ros2"))
                / "launch"
                / "laserscan_toolbox.launch.py"
            )
        ),
        launch_arguments={
            "params_file": laserscan_params_path,
            "use_sim_time": "false",
        }.items(),
    )
    delayed_laserscan = TimerAction(period=3.0, actions=[laserscan_toolbox_launch])

    # ---------- 3. AMCL localization stack (5s delay) ----------
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params_path, {"yaml_filename": map_yaml_path}],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_path],
    )

    # Keepout-zone filter servers (only when use_keepout). filter_mask_server
    # serves keepout_mask.yaml as an OccupancyGrid; costmap_filter_info_server
    # publishes the filter metadata the global costmap's KeepoutFilter consumes.
    # Both are lifecycle nodes managed by lifecycle_manager_localization. The
    # mask path is passed as a resolved-string Node override (same gotcha as
    # map_server's yaml_filename — a LaunchConfiguration in the param dict comes
    # through empty).
    filter_mask_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[nav2_params_path, {"yaml_filename": keepout_mask_path}],
    )

    costmap_filter_info_server_node = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    # The localization lifecycle manager must only manage nodes that are
    # actually launched. When keepout is off, the two filter servers are not
    # started, so drop them from node_names (leaf-param dict override, layered
    # after the yaml — same proven mechanism as the yaml_filename override
    # above). Leaving them listed would stall the whole localization bringup
    # waiting to configure nodes that never appear.
    localization_params = [nav2_params_path]
    localization_actions = [map_server_node, amcl_node]
    if use_keepout:
        localization_actions += [
            filter_mask_server_node,
            costmap_filter_info_server_node,
        ]
    else:
        localization_params = [
            nav2_params_path,
            {"node_names": ["map_server", "amcl"]},
        ]

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=localization_params,
    )
    localization_actions.append(lifecycle_manager_localization)

    delayed_localization = TimerAction(period=5.0, actions=localization_actions)

    # ---------- 4. Navigation stack (8s delay) ----------
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    # planner_server hosts the global costmap, which is where the KeepoutFilter
    # plugin lives. When keepout is off, layer keepout_off.yaml on top to disable
    # that plugin (it can't be reached with a per-node dict — it's nested under
    # global_costmap.global_costmap.ros__parameters).
    planner_params = [nav2_params_path]
    if not use_keepout:
        planner_params = [nav2_params_path, keepout_off_path]

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=planner_params,
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_path],
    )

    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_path],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav2_params_path],
    )

    delayed_navigation = TimerAction(
        period=8.0,
        actions=[
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            lifecycle_manager_navigation,
        ],
    )

    # ---------- 5. RViz2 ----------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return [
        # 0. LDLidar dual + lifecycle manager (immediate)
        ldlidar_dual_with_mgr_launch,
        # 1. UART odom + cmd_vel bridge (immediate, if enabled)
        uart_odom_launch,
        # 2. Laserscan merger after 3s
        delayed_laserscan,
        # 3. map_server + AMCL (+ filter servers if use_keepout) after 5s
        delayed_localization,
        # 4. Navigation stack after 8s
        delayed_navigation,
        # 5. RViz
        rviz_node,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("sam_bot_bringup")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 if true",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(Path(bringup_share) / "config" / "real_lidar_navigate.rviz"),
        description="Absolute path to RViz config file (navigate view: costmaps, "
        "footprint, global/local plans)",
    )
    declare_use_uart_odom = DeclareLaunchArgument(
        "use_uart_odom",
        default_value="true",
        description="Launch UART node (odometry + /cmd_vel bridge) if true",
    )
    declare_enable_mcu_bridge = DeclareLaunchArgument(
        "enable_mcu_bridge",
        default_value="false",
        description="Have the UART node also serve AutoX MCU traffic "
        "(transform query + firing solution) over the same serial port",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=str(Path(bringup_share) / "maps" / "3v3_field.yaml"),
        description="Absolute path to the map YAML for map_server "
        "(default: maps/airbnb.yaml)",
    )
    declare_use_keepout = DeclareLaunchArgument(
        "use_keepout",
        default_value="true",
        description="Load the keepout-zone filter (filter_mask_server + "
        "costmap_filter_info_server + global-costmap KeepoutFilter). Set false "
        "to navigate on a map that has no matching keepout mask",
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_rviz_config,
            declare_use_uart_odom,
            declare_enable_mcu_bridge,
            declare_map,
            declare_use_keepout,
            OpaqueFunction(function=launch_setup),
        ]
    )
