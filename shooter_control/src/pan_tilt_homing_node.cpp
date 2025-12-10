#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <future>
#include "epos/EPOSController.h"
#include "epos/Definitions.h"

class PanTiltHomingNode : public rclcpp::Node
{
public:
  PanTiltHomingNode() : Node("pan_tilt_homing_node")
  {
    // Declare parameters
    declare_parameter("device_name", "EPOS4");
    declare_parameter("protocol_stack_name", "MAXON SERIAL V2");
    declare_parameter("interface_name", "USB");
    declare_parameter("port_name", "USB0");
    declare_parameter("baudrate", 1000000);

    declare_parameter("pan_node_id", 3);
    declare_parameter("tilt_node_id", 4);

    // Pan homing parameters
    declare_parameter("pan.homing_acceleration", 7500);   // RPM/s (1.5x base)
    declare_parameter("pan.speed_switch", 4500);          // RPM (1.5x base)
    declare_parameter("pan.speed_index", 750);            // RPM (1.5x base)
    declare_parameter("pan.center_offset", 1650000);
    declare_parameter("pan.center_velocity", 13500);      // RPM (velocity when returning to center)

    // Tilt homing parameters
    declare_parameter("tilt.homing_acceleration", 1250);  // RPM/s (1/4 base)
    declare_parameter("tilt.speed_switch", 750);          // RPM (1/4 base)
    declare_parameter("tilt.speed_index", 125);           // RPM (1/4 base)
    declare_parameter("tilt.center_offset", 110000);
    declare_parameter("tilt.center_velocity", 2250);      // RPM (velocity when returning to center)

    declare_parameter("homing_timeout", 60000);           // ms

    // Get parameters
    device_name_ = get_parameter("device_name").as_string();
    protocol_stack_name_ = get_parameter("protocol_stack_name").as_string();
    interface_name_ = get_parameter("interface_name").as_string();
    port_name_ = get_parameter("port_name").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    pan_node_id_ = get_parameter("pan_node_id").as_int();
    tilt_node_id_ = get_parameter("tilt_node_id").as_int();

    pan_homing_acceleration_ = get_parameter("pan.homing_acceleration").as_int();
    pan_speed_switch_ = get_parameter("pan.speed_switch").as_int();
    pan_speed_index_ = get_parameter("pan.speed_index").as_int();
    pan_center_offset_ = get_parameter("pan.center_offset").as_int();
    pan_center_velocity_ = get_parameter("pan.center_velocity").as_int();

    tilt_homing_acceleration_ = get_parameter("tilt.homing_acceleration").as_int();
    tilt_speed_switch_ = get_parameter("tilt.speed_switch").as_int();
    tilt_speed_index_ = get_parameter("tilt.speed_index").as_int();
    tilt_center_offset_ = get_parameter("tilt.center_offset").as_int();
    tilt_center_velocity_ = get_parameter("tilt.center_velocity").as_int();

    homing_timeout_ = get_parameter("homing_timeout").as_int();

    // Create homing service
    homing_service_ = create_service<std_srvs::srv::Trigger>(
      "~/home",
      std::bind(&PanTiltHomingNode::homingCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Pan-Tilt Homing Node initialized");
    RCLCPP_INFO(get_logger(), "  Pan node_id: %d, Tilt node_id: %d", pan_node_id_, tilt_node_id_);
    RCLCPP_INFO(get_logger(), "  Call service '~/home' to start homing sequence");

    // Auto-home on startup and exit after completion
    declare_parameter("auto_home", true);
    declare_parameter("exit_after_homing", true);

    if (get_parameter("auto_home").as_bool()) {
      // Use a timer to start homing after node is fully initialized
      startup_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          startup_timer_->cancel();
          bool success = performHoming();

          if (get_parameter("exit_after_homing").as_bool()) {
            if (success) {
              RCLCPP_INFO(get_logger(), "Homing completed, shutting down node...");
              rclcpp::shutdown();
            } else {
              RCLCPP_ERROR(get_logger(), "Homing failed, shutting down node with error...");
              rclcpp::shutdown();
            }
          }
        });
    }
  }

  bool isHomingSuccessful() const { return homing_successful_; }

private:
  void homingCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool success = performHoming();
    response->success = success;
    response->message = success ? "Homing completed successfully" : "Homing failed";
  }

  bool performHoming()
  {
    RCLCPP_INFO(get_logger(), "Starting pan-tilt homing sequence...");

    // Create controllers for pan and tilt
    auto pan_controller = std::make_unique<EPOSController>(
      device_name_, protocol_stack_name_, interface_name_, port_name_,
      baudrate_, static_cast<unsigned short>(pan_node_id_));

    auto tilt_controller = std::make_unique<EPOSController>(
      device_name_, protocol_stack_name_, interface_name_, port_name_,
      baudrate_, static_cast<unsigned short>(tilt_node_id_));

    // Initialize pan
    if (!pan_controller->initialize()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize pan controller");
      return false;
    }

    // Initialize tilt
    if (!tilt_controller->initialize()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize tilt controller");
      pan_controller->close();
      return false;
    }

    // Home pan and tilt axes simultaneously
    auto pan_future = std::async(std::launch::async, [this, &pan_controller]() {
      return homeAxisToCenterOfLimits(pan_controller.get(), "pan");
    });
    auto tilt_future = std::async(std::launch::async, [this, &tilt_controller]() {
      return homeAxisToCenterOfLimits(tilt_controller.get(), "tilt");
    });

    bool pan_success = pan_future.get();
    bool tilt_success = tilt_future.get();

    // Close controllers
    pan_controller->close();
    tilt_controller->close();

    if (pan_success && tilt_success) {
      RCLCPP_INFO(get_logger(), "Homing completed successfully");
      homing_successful_ = true;
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Homing failed");
      homing_successful_ = false;
      return false;
    }
  }

  bool homeAxisToCenterOfLimits(EPOSController* controller, const std::string& axis_name)
  {
    RCLCPP_INFO(get_logger(), "[%s] Starting homing to center of limits...", axis_name.c_str());

    // Get axis-specific parameters
    const bool is_pan = (axis_name == "pan");
    const int accel = is_pan ? pan_homing_acceleration_ : tilt_homing_acceleration_;
    const int spd_switch = is_pan ? pan_speed_switch_ : tilt_speed_switch_;
    const int spd_index = is_pan ? pan_speed_index_ : tilt_speed_index_;
    const int center_offset = is_pan ? pan_center_offset_ : tilt_center_offset_;
    const int center_velocity = is_pan ? pan_center_velocity_ : tilt_center_velocity_;

    if (!controller->setHomingParameter(
          accel, spd_switch, spd_index,
          center_offset,  // home_offset: offset from negative limit to center
          0,              // current_threshold (not used for limit switch homing)
          0))             // home_position
    {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to set homing parameters", axis_name.c_str());
      return false;
    }

    // Home to negative limit switch
    RCLCPP_INFO(get_logger(), "[%s] Moving to negative limit switch...", axis_name.c_str());
    if (!controller->activateHomingMode()) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to activate homing mode", axis_name.c_str());
      return false;
    }

    if (!controller->findHome(HM_NEGATIVE_LIMIT_SWITCH)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to start homing to negative limit", axis_name.c_str());
      return false;
    }

    if (!controller->waitForHomingAttained(homing_timeout_)) {
      RCLCPP_ERROR(get_logger(), "[%s] Timeout waiting for negative limit switch", axis_name.c_str());
      controller->stopHoming();
      return false;
    }

    RCLCPP_INFO(get_logger(), "[%s] Homing attained. Moving to center (position 0)...", axis_name.c_str());

    // Move to center position (0)
    if (!controller->activatePositionMode()) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to activate position mode", axis_name.c_str());
      return false;
    }

    if (!controller->setPositionProfile(center_velocity, accel, accel)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to set position profile", axis_name.c_str());
      return false;
    }

    if (!controller->moveToPosition(0, true, true)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to move to center position", axis_name.c_str());
      return false;
    }

    // Wait for target reached
    bool target_reached = false;
    auto start_time = std::chrono::steady_clock::now();
    while (!target_reached) {
      if (!controller->isTargetReached(target_reached)) {
        RCLCPP_ERROR(get_logger(), "[%s] Failed to get movement state", axis_name.c_str());
        return false;
      }

      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > homing_timeout_) {
        RCLCPP_ERROR(get_logger(), "[%s] Timeout waiting for center position", axis_name.c_str());
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(get_logger(), "[%s] Homing completed. Now at center position.", axis_name.c_str());
    return true;
  }

  // Parameters
  std::string device_name_;
  std::string protocol_stack_name_;
  std::string interface_name_;
  std::string port_name_;
  int baudrate_;
  int pan_node_id_;
  int tilt_node_id_;
  int pan_homing_acceleration_;
  int pan_speed_switch_;
  int pan_speed_index_;
  int pan_center_offset_;
  int pan_center_velocity_;
  int tilt_homing_acceleration_;
  int tilt_speed_switch_;
  int tilt_speed_index_;
  int tilt_center_offset_;
  int tilt_center_velocity_;
  int homing_timeout_;

  // State
  bool homing_successful_{false};

  // ROS
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltHomingNode>());
  rclcpp::shutdown();
  return 0;
}
