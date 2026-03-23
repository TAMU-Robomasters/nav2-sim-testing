from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    bringup_share = get_package_share_directory("sam_bot_bringup")
    sam_bot_share = get_package_share_directory("sam_bot_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # ---------- Config / map file paths ----------
    nav2_params_path = str(Path(bringup_share) / "config" / "nav2_params.yaml")
    laserscan_params_path = str(
        Path(bringup_share) / "config" / "laserscan_toolbox_params.yaml"
    )
    map_yaml_path = str(Path(bringup_share) / "maps" / "my_map.yaml")

    # ---------- 0. Sam bot (Gazebo, rviz, robot, EKF) - launches first ----------
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(sam_bot_share) / "launch" / "display.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ---------- 1. Multi-laserscan toolbox (10s delay - wait for Gazebo) ----------
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
            "use_sim_time": "true",
        }.items(),
    )
    delayed_laserscan = TimerAction(period=10.0, actions=[laserscan_toolbox_launch])

    # ---------- 2. Nav2 bringup (localization + navigation, 15s delay) ----------
    # Uses nav2_bringup's bringup_launch.py with slam:=False so that
    # map_server + AMCL handle localisation instead of SLAM Toolbox.
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(nav2_bringup_share) / "launch" / "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": map_yaml_path,
            "use_sim_time": "true",
            "params_file": nav2_params_path,
        }.items(),
    )
    delayed_nav2_bringup = TimerAction(period=15.0, actions=[nav2_bringup_launch])

    return LaunchDescription(
        [
            declare_use_sim_time,
            # 0. Gazebo + robot + rviz (immediate)
            display_launch,
            # 1. Laserscan toolbox after 10s (Gazebo needs time to spawn)
            delayed_laserscan,
            # 2. Nav2 bringup (AMCL + navigation) after 15s
            delayed_nav2_bringup,
        ]
    )
