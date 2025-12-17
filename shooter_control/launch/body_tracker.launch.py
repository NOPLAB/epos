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

    base_angular_gain_arg = DeclareLaunchArgument(
        'base_angular_gain',
        default_value='0.5',
        description='Base angular velocity gain [rad/s per rad]'
    )

    velocity_feedforward_gain_arg = DeclareLaunchArgument(
        'velocity_feedforward_gain',
        default_value='0.005',
        description='Velocity feedforward gain for tracking moving targets'
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

    verification_interval_arg = DeclareLaunchArgument(
        'verification_interval',
        default_value='30',
        description='Frames between detection verification during tracking'
    )

    detection_miss_threshold_arg = DeclareLaunchArgument(
        'detection_miss_threshold',
        default_value='3',
        description='Consecutive detection misses before returning to detection mode'
    )

    detection_skip_frames_arg = DeclareLaunchArgument(
        'detection_skip_frames',
        default_value='3',
        description='Frames to skip between detections in DETECTING mode'
    )

    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.3',
        description='IoU threshold for YOLO verification matching'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show GUI window (set false for headless mode)'
    )

    # Scan parameters (DETECTING mode) - encoder count based
    scan_enabled_arg = DeclareLaunchArgument(
        'scan_enabled',
        default_value='true',
        description='Enable scanning in DETECTING mode'
    )

    scan_pan_velocity_arg = DeclareLaunchArgument(
        'scan_pan_velocity',
        default_value='0.5',
        description='Pan scan velocity (rad/s)'
    )

    scan_tilt_velocity_arg = DeclareLaunchArgument(
        'scan_tilt_velocity',
        default_value='0.3',
        description='Tilt scan velocity (rad/s)'
    )

    pan_counts_per_revolution_arg = DeclareLaunchArgument(
        'pan_counts_per_revolution',
        default_value='6600000.0',
        description='Pan encoder counts per revolution'
    )

    tilt_counts_per_revolution_arg = DeclareLaunchArgument(
        'tilt_counts_per_revolution',
        default_value='6600000.0',
        description='Tilt encoder counts per revolution'
    )

    scan_pan_min_counts_arg = DeclareLaunchArgument(
        'scan_pan_min_counts',
        default_value='-1650000',
        description='Pan scan minimum (encoder counts)'
    )

    scan_pan_max_counts_arg = DeclareLaunchArgument(
        'scan_pan_max_counts',
        default_value='1650000',
        description='Pan scan maximum (encoder counts)'
    )

    scan_tilt_min_counts_arg = DeclareLaunchArgument(
        'scan_tilt_min_counts',
        default_value='-500000',
        description='Tilt scan minimum (encoder counts)'
    )

    scan_tilt_max_counts_arg = DeclareLaunchArgument(
        'scan_tilt_max_counts',
        default_value='300000',
        description='Tilt scan maximum (encoder counts)'
    )

    scan_tilt_step_counts_arg = DeclareLaunchArgument(
        'scan_tilt_step_counts',
        default_value='200000',
        description='Tilt step between scan passes (encoder counts)'
    )

    # Auto-fire parameters
    auto_fire_enabled_arg = DeclareLaunchArgument(
        'auto_fire_enabled',
        default_value='false',
        description='Enable automatic firing when target is centered'
    )

    auto_fire_threshold_arg = DeclareLaunchArgument(
        'auto_fire_threshold',
        default_value='0.7',
        description='Detection confidence threshold for auto-fire'
    )

    auto_fire_cooldown_ms_arg = DeclareLaunchArgument(
        'auto_fire_cooldown_ms',
        default_value='2000',
        description='Cooldown between shots (ms)'
    )

    auto_fire_center_tolerance_arg = DeclareLaunchArgument(
        'auto_fire_center_tolerance',
        default_value='0.15',
        description='Center tolerance for auto-fire (normalized)'
    )

    shooter_speed_rpm_arg = DeclareLaunchArgument(
        'shooter_speed_rpm',
        default_value='10000.0',
        description='Shooter motor speed (RPM)'
    )

    shooter_spinup_ms_arg = DeclareLaunchArgument(
        'shooter_spinup_ms',
        default_value='500',
        description='Shooter spinup time (ms)'
    )

    shooter_fire_duration_ms_arg = DeclareLaunchArgument(
        'shooter_fire_duration_ms',
        default_value='300',
        description='Firing duration (ms)'
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
            'base_angular_gain': LaunchConfiguration('base_angular_gain'),
            'velocity_feedforward_gain': LaunchConfiguration('velocity_feedforward_gain'),
            'camera_device_id': LaunchConfiguration('camera_device_id'),
            'use_camera_device': LaunchConfiguration('use_camera_device'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'tracking_button_index': LaunchConfiguration('tracking_button_index'),
            'lost_threshold': LaunchConfiguration('lost_threshold'),
            'verification_interval': LaunchConfiguration('verification_interval'),
            'detection_miss_threshold': LaunchConfiguration('detection_miss_threshold'),
            'detection_skip_frames': LaunchConfiguration('detection_skip_frames'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'show_window': LaunchConfiguration('show_window'),
            'scan_enabled': LaunchConfiguration('scan_enabled'),
            'scan_pan_velocity': LaunchConfiguration('scan_pan_velocity'),
            'scan_tilt_velocity': LaunchConfiguration('scan_tilt_velocity'),
            'pan_counts_per_revolution': LaunchConfiguration('pan_counts_per_revolution'),
            'tilt_counts_per_revolution': LaunchConfiguration('tilt_counts_per_revolution'),
            'scan_pan_min_counts': LaunchConfiguration('scan_pan_min_counts'),
            'scan_pan_max_counts': LaunchConfiguration('scan_pan_max_counts'),
            'scan_tilt_min_counts': LaunchConfiguration('scan_tilt_min_counts'),
            'scan_tilt_max_counts': LaunchConfiguration('scan_tilt_max_counts'),
            'scan_tilt_step_counts': LaunchConfiguration('scan_tilt_step_counts'),
            'auto_fire_enabled': LaunchConfiguration('auto_fire_enabled'),
            'auto_fire_threshold': LaunchConfiguration('auto_fire_threshold'),
            'auto_fire_cooldown_ms': LaunchConfiguration('auto_fire_cooldown_ms'),
            'auto_fire_center_tolerance': LaunchConfiguration('auto_fire_center_tolerance'),
            'shooter_speed_rpm': LaunchConfiguration('shooter_speed_rpm'),
            'shooter_spinup_ms': LaunchConfiguration('shooter_spinup_ms'),
            'shooter_fire_duration_ms': LaunchConfiguration('shooter_fire_duration_ms'),
        }]
    )

    return LaunchDescription([
        pan_kp_arg,
        pan_ki_arg,
        tilt_kp_arg,
        tilt_ki_arg,
        base_angular_gain_arg,
        velocity_feedforward_gain_arg,
        camera_device_id_arg,
        use_camera_device_arg,
        confidence_threshold_arg,
        tracking_button_index_arg,
        lost_threshold_arg,
        verification_interval_arg,
        detection_miss_threshold_arg,
        detection_skip_frames_arg,
        iou_threshold_arg,
        show_window_arg,
        scan_enabled_arg,
        scan_pan_velocity_arg,
        scan_tilt_velocity_arg,
        pan_counts_per_revolution_arg,
        tilt_counts_per_revolution_arg,
        scan_pan_min_counts_arg,
        scan_pan_max_counts_arg,
        scan_tilt_min_counts_arg,
        scan_tilt_max_counts_arg,
        scan_tilt_step_counts_arg,
        auto_fire_enabled_arg,
        auto_fire_threshold_arg,
        auto_fire_cooldown_ms_arg,
        auto_fire_center_tolerance_arg,
        shooter_speed_rpm_arg,
        shooter_spinup_ms_arg,
        shooter_fire_duration_ms_arg,
        body_tracker_node,
    ])
