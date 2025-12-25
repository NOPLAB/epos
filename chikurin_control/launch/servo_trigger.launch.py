from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    pwm_chip_arg = DeclareLaunchArgument(
        'pwm_chip',
        default_value='2',
        description='PWM chip number (RPi5 typically uses 2)'
    )

    pwm_channel_arg = DeclareLaunchArgument(
        'pwm_channel',
        default_value='0',
        description='PWM channel (0 for PWM0)'
    )

    trigger_button_arg = DeclareLaunchArgument(
        'trigger_button',
        default_value='5',
        description='Joystick button index for trigger (5 = RB)'
    )

    idle_angle_arg = DeclareLaunchArgument(
        'idle_angle_deg',
        default_value='0.0',
        description='Idle position angle in degrees'
    )

    trigger_angle_arg = DeclareLaunchArgument(
        'trigger_angle_deg',
        default_value='180.0',
        description='Trigger position angle in degrees'
    )

    return_delay_arg = DeclareLaunchArgument(
        'return_delay_ms',
        default_value='300',
        description='Delay in ms before returning to idle angle'
    )

    # Servo Trigger Node
    servo_trigger_node = Node(
        package='shooter_control',
        executable='servo_trigger_node',
        name='servo_trigger_node',
        output='screen',
        parameters=[{
            'pwm_chip': LaunchConfiguration('pwm_chip'),
            'pwm_channel': LaunchConfiguration('pwm_channel'),
            'trigger_button': LaunchConfiguration('trigger_button'),
            'idle_angle_deg': LaunchConfiguration('idle_angle_deg'),
            'trigger_angle_deg': LaunchConfiguration('trigger_angle_deg'),
            'return_delay_ms': LaunchConfiguration('return_delay_ms'),
        }]
    )

    return LaunchDescription([
        pwm_chip_arg,
        pwm_channel_arg,
        trigger_button_arg,
        idle_angle_arg,
        trigger_angle_arg,
        return_delay_arg,
        servo_trigger_node,
    ])
