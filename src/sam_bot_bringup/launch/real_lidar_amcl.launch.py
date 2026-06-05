"""
Real LiDAR AMCL Launch
AMCL localization on a pre-built map using dual LD19 LiDARs.
No Gazebo simulation - real hardware with lifecycle management.

Launch sequence:
  0s  - LDLidar dual nodes + their lifecycle manager
  0s  - UART odometry node (odom -> base_link TF + /odom topic)
  3s  - Multi-laserscan toolbox (merges /lidar_left/.../scan + /lidar_right/.../scan -> /scan)
  5s  - map_server + amcl + localization lifecycle manager
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
        default_value=str(Path(bringup_share) / "config" / "real_lidar_amcl.rviz"),
        description="Absolute path to RViz config file",
    )
    declare_use_uart_odom = DeclareLaunchArgument(
        "use_uart_odom",
        default_value="true",
        description="Launch UART odometry node if true",
    )

    # ---------- Config / map paths ----------
    nav2_params_path = str(Path(bringup_share) / "config" / "nav2_params_real_lidar.yaml")
    laserscan_params_path = str(
        Path(bringup_share) / "config" / "laserscan_toolbox_real_lidar_params.yaml"
    )
    map_yaml_path = str(Path(bringup_share) / "maps" / "web_good_save.yaml")

    # ---------- 0. LDLidar dual nodes + lifecycle manager (immediate) ----------
    # Launches two ldlidar_component managed nodes (lidar_left, lidar_right)
    # plus the nav2_lifecycle_manager that transitions them to Active.
    ldlidar_dual_with_mgr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ldlidar_node"))
                / "launch"
                / "ldlidar_dual_with_mgr.launch.py"
            )
        )
    )

    # ---------- 1. UART odometry (optional, immediate) ----------
    # Reads pose from microcontroller, converts coordinate frame, publishes
    # /odom and the odom -> base_link TF.
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
    # Merges /lidar_left/ldlidar_node/scan + /lidar_right/ldlidar_node/scan
    # into a single /scan topic in the base_link frame.
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
    # map_server loads the pre-built map; amcl localizes against it using /scan.
    # A dedicated lifecycle manager transitions both nodes to Active.
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

    # ---------- 4. RViz2 ----------
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
            # 1. UART odometry - odom->base_link TF (immediate, if enabled)
            uart_odom_launch,
            # 2. Laserscan merger after 3s (wait for lidar nodes to activate)
            delayed_laserscan,
            # 3. map_server + AMCL after 5s (wait for /scan to be available)
            delayed_localization,
            # 4. RViz
            rviz_node,
        ]
    )
