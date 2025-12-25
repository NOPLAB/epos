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

    # Pan homing parameters
    pan_homing_acceleration_arg = DeclareLaunchArgument(
        'pan_homing_acceleration',
        default_value='7500',
        description='Pan homing acceleration [RPM/s]'
    )

    pan_speed_switch_arg = DeclareLaunchArgument(
        'pan_speed_switch',
        default_value='4500',
        description='Pan speed switch [RPM]'
    )

    pan_speed_index_arg = DeclareLaunchArgument(
        'pan_speed_index',
        default_value='750',
        description='Pan speed index [RPM]'
    )

    # Tilt homing parameters
    tilt_homing_acceleration_arg = DeclareLaunchArgument(
        'tilt_homing_acceleration',
        default_value='1250',
        description='Tilt homing acceleration [RPM/s]'
    )

    tilt_speed_switch_arg = DeclareLaunchArgument(
        'tilt_speed_switch',
        default_value='750',
        description='Tilt speed switch [RPM]'
    )

    tilt_speed_index_arg = DeclareLaunchArgument(
        'tilt_speed_index',
        default_value='125',
        description='Tilt speed index [RPM]'
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
            'pan.homing_acceleration': LaunchConfiguration('pan_homing_acceleration'),
            'pan.speed_switch': LaunchConfiguration('pan_speed_switch'),
            'pan.speed_index': LaunchConfiguration('pan_speed_index'),
            'tilt.homing_acceleration': LaunchConfiguration('tilt_homing_acceleration'),
            'tilt.speed_switch': LaunchConfiguration('tilt_speed_switch'),
            'tilt.speed_index': LaunchConfiguration('tilt_speed_index'),
            'homing_timeout': 60000,
            'auto_home': LaunchConfiguration('auto_home'),
            'exit_after_homing': LaunchConfiguration('exit_after_homing'),
        }]
    )

    return LaunchDescription([
        port_name_arg,
        auto_home_arg,
        exit_after_homing_arg,
        pan_homing_acceleration_arg,
        pan_speed_switch_arg,
        pan_speed_index_arg,
        tilt_homing_acceleration_arg,
        tilt_speed_switch_arg,
        tilt_speed_index_arg,
        pan_tilt_homing_node,
    ])
