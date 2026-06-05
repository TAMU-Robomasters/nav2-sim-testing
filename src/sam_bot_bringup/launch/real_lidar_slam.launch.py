from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("sam_bot_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_uart_odom = LaunchConfiguration("use_uart_odom")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz if true",
    )

    default_rviz_config = str(
        Path(get_package_share_directory("ldlidar_node"))
        / "config"
        / "ldlidar_dual.rviz"
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Absolute path to RViz config file",
    )

    declare_use_uart_odom = DeclareLaunchArgument(
        "use_uart_odom",
        default_value="true",
        description="Launch UART odometry node if true",
    )

    laserscan_params_path = str(
        Path(bringup_share) / "config" / "laserscan_toolbox_real_lidar_params.yaml"
    )
    slam_params_path = str(
        Path(bringup_share) / "config" / "slam_toolbox_real_lidar_params.yaml"
    )

    ldlidar_dual_with_mgr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("ldlidar_node"))
                / "launch"
                / "ldlidar_dual_with_mgr.launch.py"
            )
        )
    )

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
            "use_sim_time": use_sim_time,
        }.items(),
    )
    delayed_laserscan = TimerAction(period=3.0, actions=[laserscan_toolbox_launch])

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("slam_toolbox"))
                / "launch"
                / "online_async_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_path,
        }.items(),
    )
    delayed_slam = TimerAction(period=5.0, actions=[slam_launch])

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
            declare_use_sim_time,
            declare_use_rviz,
            declare_use_uart_odom,
            declare_rviz_config,
            ldlidar_dual_with_mgr_launch,
            uart_odom_launch,
            delayed_laserscan,
            delayed_slam,
            rviz_node,
        ]
    )
