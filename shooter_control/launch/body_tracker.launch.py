from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # PI control parameters
    pan_kp_arg = DeclareLaunchArgument(
        'pan_kp',
        default_value='0.002',
        description='Pan proportional gain'
    )

    pan_ki_arg = DeclareLaunchArgument(
        'pan_ki',
        default_value='0.0001',
        description='Pan integral gain'
    )

    tilt_kp_arg = DeclareLaunchArgument(
        'tilt_kp',
        default_value='0.002',
        description='Tilt proportional gain'
    )

    tilt_ki_arg = DeclareLaunchArgument(
        'tilt_ki',
        default_value='0.0001',
        description='Tilt integral gain'
    )

    # Camera parameters
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

    # Detection parameters
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO detection confidence threshold'
    )

    # Tracking parameters
    tracking_button_index_arg = DeclareLaunchArgument(
        'tracking_button_index',
        default_value='0',
        description='Joystick button index for tracking toggle (0=A button)'
    )

    lost_threshold_arg = DeclareLaunchArgument(
        'lost_threshold',
        default_value='150',
        description='Frames before declaring target lost (150 = ~5sec at 30fps)'
    )

    yolo_verification_interval_arg = DeclareLaunchArgument(
        'yolo_verification_interval',
        default_value='30',
        description='Frames between YOLO verification during tracking'
    )

    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.3',
        description='IoU threshold for YOLO verification matching'
    )

    # Body tracker node
    body_tracker_node = Node(
        package='shooter_control',
        executable='body_tracker_node',
        name='body_tracker',
        output='screen',
        parameters=[{
            'pan_kp': LaunchConfiguration('pan_kp'),
            'pan_ki': LaunchConfiguration('pan_ki'),
            'tilt_kp': LaunchConfiguration('tilt_kp'),
            'tilt_ki': LaunchConfiguration('tilt_ki'),
            'camera_device_id': LaunchConfiguration('camera_device_id'),
            'use_camera_device': LaunchConfiguration('use_camera_device'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'tracking_button_index': LaunchConfiguration('tracking_button_index'),
            'lost_threshold': LaunchConfiguration('lost_threshold'),
            'yolo_verification_interval': LaunchConfiguration('yolo_verification_interval'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
        }]
    )

    return LaunchDescription([
        pan_kp_arg,
        pan_ki_arg,
        tilt_kp_arg,
        tilt_ki_arg,
        camera_device_id_arg,
        use_camera_device_arg,
        confidence_threshold_arg,
        tracking_button_index_arg,
        lost_threshold_arg,
        yolo_verification_interval_arg,
        iou_threshold_arg,
        body_tracker_node,
    ])
