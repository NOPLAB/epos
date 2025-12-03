import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('shooter_description')
    bringup_pkg = get_package_share_directory('shooter_bringup')

    # Paths
    xacro_file = os.path.join(description_pkg, 'urdf', 'shooter.urdf.xacro')
    controllers_file = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    # Get USB port from environment variable (default: USB0)
    usb_port = os.environ.get('EPOS_USB_DEVICE', 'USB0')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false usb_port:=', usb_port]),
        value_type=str
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    pan_tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_tilt_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay diff_drive_controller after joint_state_broadcaster
    delay_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # Delay pan_tilt_controller after diff_drive_controller
    delay_pan_tilt_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[pan_tilt_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller,
        delay_pan_tilt_controller,
    ])
