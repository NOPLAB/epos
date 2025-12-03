#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "epos/srv/homing.hpp"
#include "epos/EPOSController.h"

class HomingServiceNode : public rclcpp::Node
{
public:
  HomingServiceNode() : Node("homing_service")
  {
    // Declare parameters
    this->declare_parameter<std::string>("device_name", "EPOS4");
    this->declare_parameter<std::string>("protocol_stack_name", "MAXON SERIAL V2");
    this->declare_parameter<std::string>("interface_name", "USB");
    this->declare_parameter<std::string>("port_name", "USB0");
    this->declare_parameter<int>("baudrate", 1000000);
    this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());
    this->declare_parameter<std::vector<int64_t>>("node_ids", std::vector<int64_t>());

    // Get parameters
    device_name_ = this->get_parameter("device_name").as_string();
    protocol_stack_name_ = this->get_parameter("protocol_stack_name").as_string();
    interface_name_ = this->get_parameter("interface_name").as_string();
    port_name_ = this->get_parameter("port_name").as_string();
    baudrate_ = static_cast<unsigned int>(this->get_parameter("baudrate").as_int());

    auto joint_names = this->get_parameter("joint_names").as_string_array();
    auto node_ids = this->get_parameter("node_ids").as_integer_array();

    if (joint_names.size() != node_ids.size()) {
      RCLCPP_ERROR(this->get_logger(), "joint_names and node_ids must have the same size");
      return;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      joint_to_node_id_[joint_names[i]] = static_cast<unsigned short>(node_ids[i]);
      RCLCPP_INFO(this->get_logger(), "Registered joint '%s' with node_id=%d",
                  joint_names[i].c_str(), static_cast<int>(node_ids[i]));
    }

    // Create service
    service_ = this->create_service<epos::srv::Homing>(
      "~/homing",
      std::bind(&HomingServiceNode::handleHoming, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Homing service ready");
  }

private:
  void handleHoming(
    const std::shared_ptr<epos::srv::Homing::Request> request,
    std::shared_ptr<epos::srv::Homing::Response> response)
  {
    std::vector<std::pair<std::string, unsigned short>> joints_to_home;

    if (request->joint_name.empty()) {
      // Home all registered joints
      for (const auto& pair : joint_to_node_id_) {
        joints_to_home.push_back(pair);
      }
    } else {
      // Home specific joint
      auto it = joint_to_node_id_.find(request->joint_name);
      if (it == joint_to_node_id_.end()) {
        response->success = false;
        response->message = "Joint '" + request->joint_name + "' not found";
        return;
      }
      joints_to_home.push_back(*it);
    }

    if (joints_to_home.empty()) {
      response->success = false;
      response->message = "No joints configured for homing";
      return;
    }

    // Perform homing for each joint
    for (const auto& joint : joints_to_home) {
      RCLCPP_INFO(this->get_logger(), "Homing joint '%s' (node_id=%d)",
                  joint.first.c_str(), joint.second);

      auto controller = std::make_unique<EPOSController>(
        device_name_, protocol_stack_name_, interface_name_, port_name_,
        baudrate_, joint.second);

      if (!controller->initialize()) {
        response->success = false;
        response->message = "Failed to initialize controller for joint '" + joint.first + "'";
        return;
      }

      if (!controller->activateHomingMode()) {
        response->success = false;
        response->message = "Failed to activate homing mode for joint '" + joint.first + "'";
        controller->close();
        return;
      }

      if (!controller->setHomingParameter(
            request->homing_acceleration,
            request->speed_switch,
            request->speed_index,
            request->home_offset,
            request->current_threshold,
            request->home_position)) {
        response->success = false;
        response->message = "Failed to set homing parameters for joint '" + joint.first + "'";
        controller->close();
        return;
      }

      if (!controller->findHome(request->homing_method)) {
        response->success = false;
        response->message = "Failed to start homing for joint '" + joint.first + "'";
        controller->close();
        return;
      }

      if (!controller->waitForHomingAttained(request->timeout_ms)) {
        bool attained = false;
        bool error = false;
        controller->getHomingState(attained, error);

        if (error) {
          response->success = false;
          response->message = "Homing error for joint '" + joint.first + "'";
        } else {
          response->success = false;
          response->message = "Homing timeout for joint '" + joint.first + "'";
        }
        controller->stopHoming();
        controller->close();
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Homing completed for joint '%s'", joint.first.c_str());
      controller->disable();
      controller->close();
    }

    response->success = true;
    response->message = "Homing completed successfully";
  }

  std::string device_name_;
  std::string protocol_stack_name_;
  std::string interface_name_;
  std::string port_name_;
  unsigned int baudrate_;
  std::map<std::string, unsigned short> joint_to_node_id_;
  rclcpp::Service<epos::srv::Homing>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingServiceNode>());
  rclcpp::shutdown();
  return 0;
}
