#include <algorithm>
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class ServoTriggerNode : public rclcpp::Node
{
public:
  ServoTriggerNode() : Node("servo_trigger_node")
  {
    // Declare parameters
    declare_parameter("pwm_chip", 2);            // pwmchip number (RPi5 typically uses 2)
    declare_parameter("pwm_channel", 0);         // PWM channel (0 for PWM0)
    declare_parameter("trigger_button", 5);      // RB button index
    declare_parameter("servo_period_ns", 20000000);  // 20ms = 50Hz
    declare_parameter("duty_min_ns", 500000);    // 0.5ms for 0 degrees
    declare_parameter("duty_max_ns", 2400000);   // 2.4ms for 180 degrees
    declare_parameter("idle_angle_deg", 0.0);    // Idle position angle
    declare_parameter("trigger_angle_deg", 180.0); // Trigger position angle
    declare_parameter("return_delay_ms", 300);   // Delay before returning to idle

    // Get parameters
    pwm_chip_ = get_parameter("pwm_chip").as_int();
    pwm_channel_ = get_parameter("pwm_channel").as_int();
    trigger_button_ = get_parameter("trigger_button").as_int();
    servo_period_ns_ = get_parameter("servo_period_ns").as_int();
    duty_min_ns_ = get_parameter("duty_min_ns").as_int();
    duty_max_ns_ = get_parameter("duty_max_ns").as_int();
    idle_angle_deg_ = get_parameter("idle_angle_deg").as_double();
    trigger_angle_deg_ = get_parameter("trigger_angle_deg").as_double();
    return_delay_ms_ = get_parameter("return_delay_ms").as_int();

    // Calculate duty cycles from angles
    duty_idle_ns_ = angleToDuty(idle_angle_deg_);
    duty_trigger_ns_ = angleToDuty(trigger_angle_deg_);

    // Build sysfs paths
    pwm_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) +
                "/pwm" + std::to_string(pwm_channel_);

    // Initialize PWM
    if (!initializePwm()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize PWM");
      return;
    }

    // Set initial position to idle angle
    setDutyCycle(duty_idle_ns_);
    enablePwm(true);

    // Create subscriber
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ServoTriggerNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Servo Trigger Node initialized");
    RCLCPP_INFO(get_logger(), "  PWM: pwmchip%d/pwm%d", pwm_chip_, pwm_channel_);
    RCLCPP_INFO(get_logger(), "  Trigger button: %d (RB)", trigger_button_);
    RCLCPP_INFO(get_logger(), "  Idle angle: %.1f deg, Trigger angle: %.1f deg",
                idle_angle_deg_, trigger_angle_deg_);
    RCLCPP_INFO(get_logger(), "  Return delay: %d ms", return_delay_ms_);
  }

  ~ServoTriggerNode()
  {
    // Disable PWM and unexport on shutdown
    enablePwm(false);
    unexportPwm();
  }

private:
  bool initializePwm()
  {
    // Export PWM channel
    std::string export_path = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) + "/export";
    std::ofstream export_file(export_path);
    if (export_file.is_open()) {
      export_file << pwm_channel_;
      export_file.close();
      // Wait for sysfs to create the pwm directory
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Check if PWM path exists
    std::ifstream check_file(pwm_path_ + "/period");
    if (!check_file.good()) {
      RCLCPP_ERROR(get_logger(), "PWM path not available: %s", pwm_path_.c_str());
      return false;
    }

    // Set period
    if (!setPeriod(servo_period_ns_)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "PWM initialized at %s", pwm_path_.c_str());
    return true;
  }

  void unexportPwm()
  {
    std::string unexport_path = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) + "/unexport";
    std::ofstream unexport_file(unexport_path);
    if (unexport_file.is_open()) {
      unexport_file << pwm_channel_;
      unexport_file.close();
    }
  }

  bool setPeriod(int period_ns)
  {
    std::ofstream period_file(pwm_path_ + "/period");
    if (!period_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open period file");
      return false;
    }
    period_file << period_ns;
    return true;
  }

  bool setDutyCycle(int duty_ns)
  {
    std::ofstream duty_file(pwm_path_ + "/duty_cycle");
    if (!duty_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open duty_cycle file");
      return false;
    }
    duty_file << duty_ns;
    return true;
  }

  bool enablePwm(bool enable)
  {
    std::ofstream enable_file(pwm_path_ + "/enable");
    if (!enable_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open enable file");
      return false;
    }
    enable_file << (enable ? "1" : "0");
    return true;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check if button exists
    if (static_cast<size_t>(trigger_button_) >= msg->buttons.size()) {
      return;
    }

    bool button_pressed = msg->buttons[trigger_button_] == 1;

    // Detect rising edge (button just pressed)
    if (button_pressed && !prev_button_state_) {
      triggerServo();
    }

    prev_button_state_ = button_pressed;
  }

  void triggerServo()
  {
    // Cancel any pending return timer
    if (return_timer_) {
      return_timer_->cancel();
      return_timer_.reset();
    }

    // Move to trigger angle
    setDutyCycle(duty_trigger_ns_);
    RCLCPP_INFO(get_logger(), "Servo triggered -> %.1f degrees", trigger_angle_deg_);

    // Schedule return to idle angle
    return_timer_ = create_wall_timer(
      std::chrono::milliseconds(return_delay_ms_),
      [this]() {
        setDutyCycle(duty_idle_ns_);
        RCLCPP_INFO(get_logger(), "Servo returned -> %.1f degrees", idle_angle_deg_);
        return_timer_->cancel();
        return_timer_.reset();
      });
  }

  int angleToDuty(double angle_deg)
  {
    // Clamp angle to 0-180 range
    angle_deg = std::max(0.0, std::min(180.0, angle_deg));
    // Linear interpolation between duty_min_ns (0 deg) and duty_max_ns (180 deg)
    return duty_min_ns_ + static_cast<int>((duty_max_ns_ - duty_min_ns_) * angle_deg / 180.0);
  }

  // Parameters
  int pwm_chip_;
  int pwm_channel_;
  int trigger_button_;
  int servo_period_ns_;
  int duty_min_ns_;
  int duty_max_ns_;
  double idle_angle_deg_;
  double trigger_angle_deg_;
  int return_delay_ms_;

  // Calculated duty cycles
  int duty_idle_ns_;
  int duty_trigger_ns_;

  // State
  std::string pwm_path_;
  bool prev_button_state_ = false;
  rclcpp::TimerBase::SharedPtr return_timer_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
