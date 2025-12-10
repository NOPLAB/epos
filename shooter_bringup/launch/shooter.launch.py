import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
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

    # Pan-Tilt homing parameters from environment
    pan_homing_accel = int(os.environ.get('PAN_HOMING_ACCEL', '7500'))
    pan_speed_switch = int(os.environ.get('PAN_SPEED_SWITCH', '4500'))
    pan_speed_index = int(os.environ.get('PAN_SPEED_INDEX', '750'))
    pan_center_offset = int(os.environ.get('PAN_CENTER_OFFSET', '1650000'))
    pan_center_velocity = int(os.environ.get('PAN_CENTER_VELOCITY', '13500'))
    tilt_homing_accel = int(os.environ.get('TILT_HOMING_ACCEL', '1250'))
    tilt_speed_switch = int(os.environ.get('TILT_SPEED_SWITCH', '750'))
    tilt_speed_index = int(os.environ.get('TILT_SPEED_INDEX', '125'))
    tilt_center_offset = int(os.environ.get('TILT_CENTER_OFFSET', '110000'))
    tilt_center_velocity = int(os.environ.get('TILT_CENTER_VELOCITY', '2250'))
    homing_timeout = int(os.environ.get('HOMING_TIMEOUT', '60000'))

    # Launch arguments
    enable_homing_arg = DeclareLaunchArgument(
        'enable_homing',
        default_value='false',
        description='Enable pan-tilt homing before starting controllers'
    )
    enable_homing = LaunchConfiguration('enable_homing')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false usb_port:=', usb_port]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Pan-Tilt Homing Node (only when homing is enabled)
    pan_tilt_homing_node = Node(
        package='shooter_control',
        executable='pan_tilt_homing_node',
        name='pan_tilt_homing_node',
        output='screen',
        condition=IfCondition(enable_homing),
        parameters=[{
            'port_name': usb_port,
            'pan_node_id': 3,
            'tilt_node_id': 4,
            'pan.homing_acceleration': pan_homing_accel,
            'pan.speed_switch': pan_speed_switch,
            'pan.speed_index': pan_speed_index,
            'pan.center_offset': pan_center_offset,
            'pan.center_velocity': pan_center_velocity,
            'tilt.homing_acceleration': tilt_homing_accel,
            'tilt.speed_switch': tilt_speed_switch,
            'tilt.speed_index': tilt_speed_index,
            'tilt.center_offset': tilt_center_offset,
            'tilt.center_velocity': tilt_center_velocity,
            'homing_timeout': homing_timeout,
            'auto_home': True,
            'exit_after_homing': True,
        }]
    )

    # Controller Manager (without homing)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen',
        condition=UnlessCondition(enable_homing)
    )

    # Controller Manager (with homing - starts after homing completes)
    controller_manager_node_after_homing = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Start controller_manager after homing node exits
    delay_controller_manager_after_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_homing_node,
            on_exit=[controller_manager_node_after_homing],
        ),
        condition=IfCondition(enable_homing)
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=UnlessCondition(enable_homing)
    )

    joint_state_broadcaster_spawner_after_homing = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30'
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

    # Event handlers for homing mode
    # Start joint_state_broadcaster spawner after homing completes
    # The spawner will wait for controller_manager via --controller-manager-timeout
    delay_joint_state_broadcaster_after_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_homing_node,
            on_exit=[joint_state_broadcaster_spawner_after_homing],
        ),
        condition=IfCondition(enable_homing)
    )

    delay_diff_drive_controller_after_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_after_homing,
            on_exit=[diff_drive_controller_spawner],
        ),
        condition=IfCondition(enable_homing)
    )

    # Event handlers for non-homing mode
    delay_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        ),
        condition=UnlessCondition(enable_homing)
    )

    # Delay pan_tilt_controller after diff_drive_controller (common)
    delay_pan_tilt_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[pan_tilt_controller_spawner],
        )
    )

    # Delay shooter_controller after pan_tilt_controller (common)
    delay_shooter_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_controller_spawner,
            on_exit=[shooter_controller_spawner],
        )
    )

    return LaunchDescription([
        enable_homing_arg,
        robot_state_publisher_node,
        # Without homing
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller,
        # With homing
        pan_tilt_homing_node,
        delay_controller_manager_after_homing,
        delay_joint_state_broadcaster_after_homing,
        delay_diff_drive_controller_after_homing,
        # Common
        delay_pan_tilt_controller,
        delay_shooter_controller,
    ])
