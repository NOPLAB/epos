#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PanTiltJoyNode : public rclcpp::Node
{
public:
  PanTiltJoyNode() : Node("pan_tilt_joy_node")
  {
    // Declare parameters
    declare_parameter("pan_axis", 3);           // Right stick horizontal (default for many controllers)
    declare_parameter("tilt_axis", 4);          // Right stick vertical
    declare_parameter("pan_velocity", 1.0);     // Max velocity in rad/s at full deflection
    declare_parameter("tilt_velocity", 1.0);    // Max velocity in rad/s at full deflection
    declare_parameter("deadzone", 0.1);         // Joystick deadzone
    declare_parameter("shooter_trigger_axis", 5);  // RT trigger axis
    declare_parameter("shooter_speed_rpm", 5000.0); // Shooter speed in RPM

    // Get parameters
    pan_axis_ = get_parameter("pan_axis").as_int();
    tilt_axis_ = get_parameter("tilt_axis").as_int();
    pan_velocity_ = get_parameter("pan_velocity").as_double();
    tilt_velocity_ = get_parameter("tilt_velocity").as_double();
    deadzone_ = get_parameter("deadzone").as_double();
    shooter_trigger_axis_ = get_parameter("shooter_trigger_axis").as_int();
    shooter_speed_rpm_ = get_parameter("shooter_speed_rpm").as_double();
    // Convert RPM to rad/s for ros2_control
    shooter_speed_rad_s_ = shooter_speed_rpm_ * 2.0 * M_PI / 60.0;

    // Create publishers
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pan_tilt_controller/commands", 10);
    shooter_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/shooter_controller/commands", 10);

    // Create subscriber
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&PanTiltJoyNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Pan-Tilt Joy Node initialized (velocity control)");
    RCLCPP_INFO(get_logger(), "  Pan axis: %d, Tilt axis: %d", pan_axis_, tilt_axis_);
    RCLCPP_INFO(get_logger(), "  Pan velocity: %.3f rad/s, Tilt velocity: %.3f rad/s", pan_velocity_, tilt_velocity_);
    RCLCPP_INFO(get_logger(), "  Shooter trigger axis: %d, speed: %.0f RPM", shooter_trigger_axis_, shooter_speed_rpm_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Check if axes exist
    if (static_cast<size_t>(pan_axis_) >= msg->axes.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pan axis %d not available (only %zu axes)", pan_axis_, msg->axes.size());
      return;
    }

    // Get joystick values
    double pan_input = msg->axes[pan_axis_];
    double tilt_input = 0.0;

    if (static_cast<size_t>(tilt_axis_) < msg->axes.size()) {
      tilt_input = msg->axes[tilt_axis_];
    }

    // Apply deadzone
    if (std::abs(pan_input) < deadzone_) {
      pan_input = 0.0;
    }
    if (std::abs(tilt_input) < deadzone_) {
      tilt_input = 0.0;
    }

    // Calculate velocity commands
    double pan_vel = pan_input * pan_velocity_;
    double tilt_vel = tilt_input * tilt_velocity_;

    // Publish pan-tilt velocity command
    auto cmd = std_msgs::msg::Float64MultiArray();
    cmd.data = {pan_vel, tilt_vel};
    command_pub_->publish(cmd);

    // Handle shooter trigger (RT button)
    // RT axis: 1.0 = not pressed, -1.0 = fully pressed
    if (static_cast<size_t>(shooter_trigger_axis_) < msg->axes.size()) {
      double trigger_value = msg->axes[shooter_trigger_axis_];
      double shooter_velocity = 0.0;
      // Trigger is pressed when value < 0.5 (axis goes from 1.0 to -1.0)
      if (trigger_value < 0.5) {
        shooter_velocity = shooter_speed_rad_s_;
      }
      auto shooter_cmd = std_msgs::msg::Float64MultiArray();
      shooter_cmd.data = {shooter_velocity};
      shooter_pub_->publish(shooter_cmd);
    }
  }

  // Parameters
  int pan_axis_;
  int tilt_axis_;
  double pan_velocity_;
  double tilt_velocity_;
  double deadzone_;
  int shooter_trigger_axis_;
  double shooter_speed_rpm_;
  double shooter_speed_rad_s_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr shooter_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltJoyNode>());
  rclcpp::shutdown();
  return 0;
}
