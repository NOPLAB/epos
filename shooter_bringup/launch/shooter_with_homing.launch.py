import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Step 1: Pan-Tilt Homing Node (runs first, exits after completion)
    pan_tilt_homing_node = Node(
        package='shooter_control',
        executable='pan_tilt_homing_node',
        name='pan_tilt_homing_node',
        output='screen',
        parameters=[{
            'port_name': usb_port,
            'pan_node_id': 3,
            'tilt_node_id': 4,
            'homing_acceleration': 5000,
            'speed_switch': 500,
            'speed_index': 100,
            'homing_timeout': 30000,
            'auto_home': True,
            'exit_after_homing': True,
        }]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Step 2: Controller Manager (starts after homing completes)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Start controller_manager after homing node exits
    delay_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_homing_node,
            on_exit=[controller_manager_node],
        )
    )

    # Step 3: Spawners
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

    # Delay joint_state_broadcaster after controller_manager
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_homing_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
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
        pan_tilt_homing_node,
        delay_controller_manager,
        delay_joint_state_broadcaster,
        delay_diff_drive_controller,
        delay_pan_tilt_controller,
    ])
