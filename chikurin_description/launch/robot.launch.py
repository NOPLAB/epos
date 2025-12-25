import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('chikurin_description')

    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'chikurin.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Get USB port from environment variable (default: USB0)
    usb_port = os.environ.get('EPOS_USB_DEVICE', 'USB0')

    # Launch arguments
    enable_homing_arg = DeclareLaunchArgument(
        'enable_homing',
        default_value='false',
        description='Enable pan-tilt homing during hardware activation'
    )
    enable_homing = LaunchConfiguration('enable_homing')

    # Robot description with enable_homing passed to xacro
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=false',
            ' usb_port:=', usb_port,
            ' enable_homing:=', enable_homing
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
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

    shooter_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['shooter_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Sequential spawning using event handlers
    delay_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    delay_pan_tilt_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[pan_tilt_controller_spawner],
        )
    )

    delay_shooter_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_controller_spawner,
            on_exit=[shooter_controller_spawner],
        )
    )

    return LaunchDescription([
        enable_homing_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller,
        delay_pan_tilt_controller,
        delay_shooter_controller,
    ])
