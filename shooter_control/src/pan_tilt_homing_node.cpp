#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
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

    // Homing parameters
    declare_parameter("homing_acceleration", 5000);  // RPM/s
    declare_parameter("speed_switch", 500);          // RPM (speed to search for switch)
    declare_parameter("speed_index", 100);           // RPM (speed to search for index)
    declare_parameter("homing_timeout", 30000);      // ms

    // Get parameters
    device_name_ = get_parameter("device_name").as_string();
    protocol_stack_name_ = get_parameter("protocol_stack_name").as_string();
    interface_name_ = get_parameter("interface_name").as_string();
    port_name_ = get_parameter("port_name").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    pan_node_id_ = get_parameter("pan_node_id").as_int();
    tilt_node_id_ = get_parameter("tilt_node_id").as_int();

    homing_acceleration_ = get_parameter("homing_acceleration").as_int();
    speed_switch_ = get_parameter("speed_switch").as_int();
    speed_index_ = get_parameter("speed_index").as_int();
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

    // auto tilt_controller = std::make_unique<EPOSController>(
    //   device_name_, protocol_stack_name_, interface_name_, port_name_,
    //   baudrate_, static_cast<unsigned short>(tilt_node_id_));

    // Initialize pan
    if (!pan_controller->initialize()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize pan controller");
      return false;
    }

    // // Initialize tilt
    // if (!tilt_controller->initialize()) {
    //   RCLCPP_ERROR(get_logger(), "Failed to initialize tilt controller");
    //   pan_controller->close();
    //   return false;
    // }

    // Home pan axis
    bool pan_success = homeAxisToCenterOfLimits(pan_controller.get(), "pan");

    // // Home tilt axis
    // bool tilt_success = homeAxisToCenterOfLimits(tilt_controller.get(), "tilt");

    // Close controllers
    pan_controller->close();
    // tilt_controller->close();

    // if (pan_success && tilt_success) {
    if (pan_success) {
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

    // Set homing parameters
    // Parameters: acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position
    if (!controller->setHomingParameter(
          homing_acceleration_, speed_switch_, speed_index_,
          0,     // home_offset
          0,     // current_threshold (not used for limit switch homing)
          0))    // home_position
    {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to set homing parameters", axis_name.c_str());
      return false;
    }

    // Step 1: Home to negative limit switch
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

    // Get position at negative limit
    int negative_limit_pos = 0;
    if (!controller->getPosition(negative_limit_pos)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to get position at negative limit", axis_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "[%s] Negative limit position: %d", axis_name.c_str(), negative_limit_pos);

    // Step 2: Home to positive limit switch
    RCLCPP_INFO(get_logger(), "[%s] Moving to positive limit switch...", axis_name.c_str());
    if (!controller->findHome(HM_POSITIVE_LIMIT_SWITCH)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to start homing to positive limit", axis_name.c_str());
      return false;
    }

    if (!controller->waitForHomingAttained(homing_timeout_)) {
      RCLCPP_ERROR(get_logger(), "[%s] Timeout waiting for positive limit switch", axis_name.c_str());
      controller->stopHoming();
      return false;
    }

    // Get position at positive limit
    int positive_limit_pos = 0;
    if (!controller->getPosition(positive_limit_pos)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to get position at positive limit", axis_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "[%s] Positive limit position: %d", axis_name.c_str(), positive_limit_pos);

    // Step 3: Calculate center position
    int center_pos = (negative_limit_pos + positive_limit_pos) / 2;
    RCLCPP_INFO(get_logger(), "[%s] Center position: %d", axis_name.c_str(), center_pos);

    // Step 4: Move to center position
    RCLCPP_INFO(get_logger(), "[%s] Moving to center position...", axis_name.c_str());
    if (!controller->activatePositionMode()) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to activate position mode", axis_name.c_str());
      return false;
    }

    if (!controller->setPositionProfile(speed_switch_, homing_acceleration_, homing_acceleration_)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to set position profile", axis_name.c_str());
      return false;
    }

    if (!controller->moveToPosition(center_pos, true, true)) {
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

    // Step 5: Define current position as zero (origin)
    RCLCPP_INFO(get_logger(), "[%s] Defining current position as origin...", axis_name.c_str());
    if (!controller->definePosition(0)) {
      RCLCPP_ERROR(get_logger(), "[%s] Failed to define position as origin", axis_name.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "[%s] Homing completed. Origin set at center of limits.", axis_name.c_str());
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
  int homing_acceleration_;
  int speed_switch_;
  int speed_index_;
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
