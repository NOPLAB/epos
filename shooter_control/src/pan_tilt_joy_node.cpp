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
    declare_parameter("pan_scale", 0.1);        // Radians per update at full deflection
    declare_parameter("tilt_scale", 0.1);       // Radians per update at full deflection
    declare_parameter("pan_min", -1.57);        // Min pan angle (radians)
    declare_parameter("pan_max", 1.57);         // Max pan angle (radians)
    declare_parameter("tilt_min", -0.5);        // Min tilt angle (radians)
    declare_parameter("tilt_max", 0.5);         // Max tilt angle (radians)
    declare_parameter("deadzone", 0.1);         // Joystick deadzone
    declare_parameter("enable_tilt", false);    // Enable tilt control (disabled by default)

    // Get parameters
    pan_axis_ = get_parameter("pan_axis").as_int();
    tilt_axis_ = get_parameter("tilt_axis").as_int();
    pan_scale_ = get_parameter("pan_scale").as_double();
    tilt_scale_ = get_parameter("tilt_scale").as_double();
    pan_min_ = get_parameter("pan_min").as_double();
    pan_max_ = get_parameter("pan_max").as_double();
    tilt_min_ = get_parameter("tilt_min").as_double();
    tilt_max_ = get_parameter("tilt_max").as_double();
    deadzone_ = get_parameter("deadzone").as_double();
    enable_tilt_ = get_parameter("enable_tilt").as_bool();

    // Initialize positions to center
    pan_position_ = 0.0;
    tilt_position_ = 0.0;

    // Create publisher
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pan_tilt_controller/commands", 10);

    // Create subscriber
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&PanTiltJoyNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Pan-Tilt Joy Node initialized");
    RCLCPP_INFO(get_logger(), "  Pan axis: %d, Tilt axis: %d", pan_axis_, tilt_axis_);
    RCLCPP_INFO(get_logger(), "  Pan scale: %.3f, Tilt scale: %.3f", pan_scale_, tilt_scale_);
    RCLCPP_INFO(get_logger(), "  Tilt enabled: %s", enable_tilt_ ? "true" : "false");
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

    if (enable_tilt_ && static_cast<size_t>(tilt_axis_) < msg->axes.size()) {
      tilt_input = msg->axes[tilt_axis_];
    }

    // Apply deadzone
    if (std::abs(pan_input) < deadzone_) {
      pan_input = 0.0;
    }
    if (std::abs(tilt_input) < deadzone_) {
      tilt_input = 0.0;
    }

    // Update positions (incremental control)
    pan_position_ += pan_input * pan_scale_;
    tilt_position_ += tilt_input * tilt_scale_;

    // Clamp to limits
    pan_position_ = std::clamp(pan_position_, pan_min_, pan_max_);
    tilt_position_ = std::clamp(tilt_position_, tilt_min_, tilt_max_);

    // Publish command
    auto cmd = std_msgs::msg::Float64MultiArray();
    if (enable_tilt_) {
      cmd.data = {pan_position_, tilt_position_};
    } else {
      cmd.data = {pan_position_};
    }
    command_pub_->publish(cmd);
  }

  // Parameters
  int pan_axis_;
  int tilt_axis_;
  double pan_scale_;
  double tilt_scale_;
  double pan_min_;
  double pan_max_;
  double tilt_min_;
  double tilt_max_;
  double deadzone_;
  bool enable_tilt_;

  // State
  double pan_position_;
  double tilt_position_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltJoyNode>());
  rclcpp::shutdown();
  return 0;
}
