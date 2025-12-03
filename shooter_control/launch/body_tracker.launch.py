from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='40.0',
        description='Proportional gain'
    )

    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='2.0',
        description='Integral gain'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='1.5',
        description='Maximum angular velocity (rad/s)'
    )

    camera_device_id_arg = DeclareLaunchArgument(
        'camera_device_id',
        default_value='0',
        description='Camera device ID (e.g., 0 for /dev/video0)'
    )

    use_camera_device_arg = DeclareLaunchArgument(
        'use_camera_device',
        default_value='true',
        description='Use direct camera device (true) or ROS image topic (false)'
    )

    # Body tracker node
    body_tracker_node = Node(
        package='shooter_control',
        executable='body_tracker_node',
        name='body_tracker',
        output='screen',
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'camera_device_id': LaunchConfiguration('camera_device_id'),
            'use_camera_device': LaunchConfiguration('use_camera_device'),
        }]
    )

    return LaunchDescription([
        kp_arg,
        ki_arg,
        max_angular_velocity_arg,
        camera_device_id_arg,
        use_camera_device_arg,
        body_tracker_node,
    ])
