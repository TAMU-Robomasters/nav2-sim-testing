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

    # ---------- Launch arguments ----------
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # ---------- Config file paths ----------
    nav2_params_path = str(Path(bringup_share) / "config" / "nav2_params.yaml")
    laserscan_params_path = str(
        Path(bringup_share) / "config" / "laserscan_toolbox_params.yaml"
    )

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

    # ---------- 2. SLAM Toolbox (15s delay) ----------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("slam_toolbox"))
                / "launch"
                / "online_async_launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    delayed_slam = TimerAction(period=15.0, actions=[slam_launch])

    # ---------- 3. Nav2 Navigation (20s delay) ----------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("nav2_bringup"))
                / "launch"
                / "navigation_launch.py"
            )
        ),
        launch_arguments={"params_file": nav2_params_path}.items(),
    )
    delayed_nav2 = TimerAction(period=20.0, actions=[nav2_launch])

    return LaunchDescription(
        [
            declare_use_sim_time,
            # 0. Gazebo + robot + rviz (immediate)
            display_launch,
            # 1. Laserscan toolbox after 10s (Gazebo needs time to spawn)
            delayed_laserscan,
            # 2. SLAM after 15s
            delayed_slam,
            # 3. Nav2 after 20s
            delayed_nav2,
        ]
    )
