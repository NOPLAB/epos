from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='USB0',
        description='EPOS USB port name'
    )

    auto_home_arg = DeclareLaunchArgument(
        'auto_home',
        default_value='true',
        description='Automatically start homing on node startup'
    )

    exit_after_homing_arg = DeclareLaunchArgument(
        'exit_after_homing',
        default_value='true',
        description='Exit node after homing completes'
    )

    # Pan-Tilt Homing Node
    pan_tilt_homing_node = Node(
        package='shooter_control',
        executable='pan_tilt_homing_node',
        name='pan_tilt_homing_node',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'pan_node_id': 3,
            'tilt_node_id': 4,
            'homing_acceleration': 5000,
            'speed_switch': 500,
            'speed_index': 100,
            'homing_timeout': 30000,
            'auto_home': LaunchConfiguration('auto_home'),
            'exit_after_homing': LaunchConfiguration('exit_after_homing'),
        }]
    )

    return LaunchDescription([
        port_name_arg,
        auto_home_arg,
        exit_after_homing_arg,
        pan_tilt_homing_node,
    ])
