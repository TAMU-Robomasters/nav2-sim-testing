
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyTHS1',
        description='Serial port for UART odometry'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )

    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )

    declare_update_rate = DeclareLaunchArgument(
        'update_rate',
        default_value='100.0',
        description='Node update rate in Hz'
    )

    uart_odom_node = ExecuteProcess(
        cmd=[
            'uart_odom_node',
            '--ros-args',
            '-p', ['port:=', LaunchConfiguration('port')],
            '-p', ['baudrate:=', LaunchConfiguration('baudrate')],
            '-p', ['odom_frame:=', LaunchConfiguration('odom_frame')],
            '-p', ['base_frame:=', LaunchConfiguration('base_frame')],
            '-p', ['publish_tf:=', LaunchConfiguration('publish_tf')],
            '-p', ['update_rate:=', LaunchConfiguration('update_rate')],
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_port,
        declare_baudrate,
        declare_odom_frame,
        declare_base_frame,
        declare_publish_tf,
        declare_update_rate,
        uart_odom_node,
    ])
