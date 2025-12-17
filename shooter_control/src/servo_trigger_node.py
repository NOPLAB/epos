#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from rpi_hardware_pwm import HardwarePWM


class ServoTriggerNode(Node):
    def __init__(self):
        super().__init__('servo_trigger_node')

        # Declare parameters
        self.declare_parameter('pwm_channel', 0)
        self.declare_parameter('pwm_chip', 0)
        self.declare_parameter('trigger_button', 5)  # RB button
        self.declare_parameter('idle_angle_deg', 0.0)
        self.declare_parameter('trigger_angle_deg', 180.0)
        self.declare_parameter('return_delay_ms', 300)

        # Get parameters
        self.pwm_channel = self.get_parameter('pwm_channel').value
        self.pwm_chip = self.get_parameter('pwm_chip').value
        self.trigger_button = self.get_parameter('trigger_button').value
        self.idle_angle_deg = self.get_parameter('idle_angle_deg').value
        self.trigger_angle_deg = self.get_parameter('trigger_angle_deg').value
        self.return_delay_ms = self.get_parameter('return_delay_ms').value

        # Initialize PWM (50Hz for servo)
        self.pwm = HardwarePWM(pwm_channel=self.pwm_channel, hz=50, chip=self.pwm_chip)
        self.pwm.start(self.angle_to_duty(self.idle_angle_deg))

        # State
        self.prev_button_state = False
        self.return_timer = None

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # Service for external trigger
        self.trigger_srv = self.create_service(
            Trigger, '~/trigger', self.trigger_callback)

        self.get_logger().info('Servo Trigger Node initialized')
        self.get_logger().info(f'  PWM: chip={self.pwm_chip}, channel={self.pwm_channel}')
        self.get_logger().info(f'  Trigger button: {self.trigger_button}')
        self.get_logger().info(f'  Idle angle: {self.idle_angle_deg} deg, Trigger angle: {self.trigger_angle_deg} deg')
        self.get_logger().info(f'  Return delay: {self.return_delay_ms} ms')

    def angle_to_duty(self, angle_deg: float) -> float:
        """Convert angle (0-180) to duty cycle (2.5-12.5%)"""
        angle_deg = max(0.0, min(180.0, angle_deg))
        # 0 deg = 2.5%, 180 deg = 12.5%
        return 2.5 + (angle_deg / 180.0) * 10.0

    def joy_callback(self, msg: Joy):
        if self.trigger_button >= len(msg.buttons):
            return

        button_pressed = msg.buttons[self.trigger_button] == 1

        # Detect rising edge
        if button_pressed and not self.prev_button_state:
            self.trigger_servo()

        self.prev_button_state = button_pressed

    def trigger_callback(self, request, response):
        """Service callback for external trigger"""
        self.trigger_servo()
        response.success = True
        response.message = 'Servo triggered'
        return response

    def trigger_servo(self):
        # Cancel pending timer
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.return_timer = None

        # Move to trigger angle
        self.pwm.change_duty_cycle(self.angle_to_duty(self.trigger_angle_deg))
        self.get_logger().info(f'Servo triggered -> {self.trigger_angle_deg} degrees')

        # Schedule return to idle
        self.return_timer = self.create_timer(
            self.return_delay_ms / 1000.0,
            self.return_to_idle)

    def return_to_idle(self):
        self.pwm.change_duty_cycle(self.angle_to_duty(self.idle_angle_deg))
        self.get_logger().info(f'Servo returned -> {self.idle_angle_deg} degrees')
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.return_timer = None

    def destroy_node(self):
        self.pwm.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
