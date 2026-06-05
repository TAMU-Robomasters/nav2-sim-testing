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
  8s  - controller + planner + behavior + bt_navigator + waypoint_follower
        + navigation lifecycle manager
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("sam_bot_bringup")

    # ---------- Launch arguments ----------
    use_rviz = LaunchConfiguration("use_rviz")
    use_uart_odom = LaunchConfiguration("use_uart_odom")
    rviz_config = LaunchConfiguration("rviz_config")

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

    # ---------- Config / map paths ----------
    # Resolved to plain strings (mirrors real_lidar_amcl.launch.py). Passing
    # these as LaunchConfiguration substitutions inside parameter dicts can leave
    # map_server with an empty yaml_filename, which makes on_configure fail and
    # the whole localization chain stall 'unconfigured'.
    nav2_params_path = str(
        Path(bringup_share) / "config" / "nav2_params_real_lidar.yaml"
    )
    map_yaml_path = str(Path(bringup_share) / "maps" / "web_good_save.yaml")
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

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[nav2_params_path],
    )

    delayed_localization = TimerAction(
        period=5.0,
        actions=[map_server_node, amcl_node, lifecycle_manager_localization],
    )

    # ---------- 4. Navigation stack (8s delay) ----------
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_path],
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

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_rviz_config,
            declare_use_uart_odom,
            # 0. LDLidar dual + lifecycle manager (immediate)
            ldlidar_dual_with_mgr_launch,
            # 1. UART odom + cmd_vel bridge (immediate, if enabled)
            uart_odom_launch,
            # 2. Laserscan merger after 3s
            delayed_laserscan,
            # 3. map_server + AMCL after 5s
            delayed_localization,
            # 4. Navigation stack after 8s
            delayed_navigation,
            # 5. RViz
            rviz_node,
        ]
    )
