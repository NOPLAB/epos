#include "epos/EPOSHardwareInterface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace epos
{

hardware_interface::CallbackReturn EPOSHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters
  if (info_.hardware_parameters.count("device_name")) {
    device_name_ = info_.hardware_parameters.at("device_name");
  }
  if (info_.hardware_parameters.count("protocol_stack_name")) {
    protocol_stack_name_ = info_.hardware_parameters.at("protocol_stack_name");
  }
  if (info_.hardware_parameters.count("interface_name")) {
    interface_name_ = info_.hardware_parameters.at("interface_name");
  }
  if (info_.hardware_parameters.count("port_name")) {
    port_name_ = info_.hardware_parameters.at("port_name");
  }
  if (info_.hardware_parameters.count("baudrate")) {
    baudrate_ = std::stoul(info_.hardware_parameters.at("baudrate"));
  }
  if (info_.hardware_parameters.count("counts_per_revolution")) {
    counts_per_revolution_ = std::stod(info_.hardware_parameters.at("counts_per_revolution"));
  }

  // Initialize joints from URDF
  joints_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
    joints_[i].name = info_.joints[i].name;

    // Get node_id from joint parameters
    if (info_.joints[i].parameters.count("node_id")) {
      joints_[i].node_id = static_cast<unsigned short>(
        std::stoul(info_.joints[i].parameters.at("node_id")));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' missing required parameter 'node_id'", joints_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get per-joint counts_per_revolution (optional, falls back to hardware default)
    if (info_.joints[i].parameters.count("counts_per_revolution")) {
      joints_[i].counts_per_revolution = std::stod(
        info_.joints[i].parameters.at("counts_per_revolution"));
    } else {
      joints_[i].counts_per_revolution = counts_per_revolution_;
    }

    // Get velocity profile parameters (optional)
    if (info_.joints[i].parameters.count("velocity_acceleration")) {
      joints_[i].velocity_acceleration = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("velocity_acceleration")));
    }
    if (info_.joints[i].parameters.count("velocity_deceleration")) {
      joints_[i].velocity_deceleration = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("velocity_deceleration")));
    }

    // Get position profile parameters (optional)
    if (info_.joints[i].parameters.count("position_velocity")) {
      joints_[i].position_velocity = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("position_velocity")));
    }
    if (info_.joints[i].parameters.count("position_acceleration")) {
      joints_[i].position_acceleration = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("position_acceleration")));
    }
    if (info_.joints[i].parameters.count("position_deceleration")) {
      joints_[i].position_deceleration = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("position_deceleration")));
    }

    // Get homing parameters (optional)
    if (info_.joints[i].parameters.count("homing_enabled")) {
      joints_[i].homing.enabled =
        (info_.joints[i].parameters.at("homing_enabled") == "true" ||
         info_.joints[i].parameters.at("homing_enabled") == "1");
    }
    if (info_.joints[i].parameters.count("homing_method")) {
      joints_[i].homing.homing_method = static_cast<signed char>(
        std::stoi(info_.joints[i].parameters.at("homing_method")));
    }
    if (info_.joints[i].parameters.count("homing_acceleration")) {
      joints_[i].homing.homing_acceleration = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("homing_acceleration")));
    }
    if (info_.joints[i].parameters.count("homing_speed_switch")) {
      joints_[i].homing.speed_switch = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("homing_speed_switch")));
    }
    if (info_.joints[i].parameters.count("homing_speed_index")) {
      joints_[i].homing.speed_index = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("homing_speed_index")));
    }
    if (info_.joints[i].parameters.count("homing_center_offset")) {
      joints_[i].homing.center_offset =
        std::stoi(info_.joints[i].parameters.at("homing_center_offset"));
    }
    if (info_.joints[i].parameters.count("homing_center_velocity")) {
      joints_[i].homing.center_velocity = static_cast<unsigned int>(
        std::stoul(info_.joints[i].parameters.at("homing_center_velocity")));
    }
    if (info_.joints[i].parameters.count("homing_timeout")) {
      joints_[i].homing.timeout_ms =
        std::stoi(info_.joints[i].parameters.at("homing_timeout"));
    }

    // Get soft limit parameters (optional)
    if (info_.joints[i].parameters.count("soft_limit_enabled")) {
      joints_[i].soft_limit.enabled =
        (info_.joints[i].parameters.at("soft_limit_enabled") == "true" ||
         info_.joints[i].parameters.at("soft_limit_enabled") == "1");
    }
    if (info_.joints[i].parameters.count("soft_limit_min")) {
      joints_[i].soft_limit.min_position =
        std::stoi(info_.joints[i].parameters.at("soft_limit_min"));
    }
    if (info_.joints[i].parameters.count("soft_limit_max")) {
      joints_[i].soft_limit.max_position =
        std::stoi(info_.joints[i].parameters.at("soft_limit_max"));
    }

    // Verify command interfaces (velocity and/or position)
    bool has_velocity_cmd = false;
    bool has_position_cmd = false;
    for (const auto & cmd_if : info_.joints[i].command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_cmd = true;
      } else if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        has_position_cmd = true;
      }
    }
    if (!has_velocity_cmd && !has_position_cmd) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' must have at least one command interface (velocity or position)",
        joints_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify state interfaces
    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto & state_if : info_.joints[i].state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      } else if (state_if.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      }
    }
    if (!has_position_state || !has_velocity_state) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' must have position and velocity state interfaces",
        joints_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "Joint '%s' configured with node_id=%d, counts_per_rev=%.0f, vel_accel=%u, vel_decel=%u",
      joints_[i].name.c_str(), joints_[i].node_id, joints_[i].counts_per_revolution,
      joints_[i].velocity_acceleration, joints_[i].velocity_deceleration);

    if (joints_[i].homing.enabled) {
      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' homing enabled: method=%d, accel=%u, speed_switch=%u, speed_index=%u, "
        "center_offset=%d, center_velocity=%u, timeout=%d",
        joints_[i].name.c_str(), joints_[i].homing.homing_method,
        joints_[i].homing.homing_acceleration, joints_[i].homing.speed_switch,
        joints_[i].homing.speed_index, joints_[i].homing.center_offset,
        joints_[i].homing.center_velocity, joints_[i].homing.timeout_ms);
    }

    if (joints_[i].soft_limit.enabled) {
      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' soft limits enabled: min=%d, max=%d counts",
        joints_[i].name.c_str(), joints_[i].soft_limit.min_position,
        joints_[i].soft_limit.max_position);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EPOSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EPOSHardwareInterface"), "Configuring...");

  // Create EPOSController instances
  for (auto & joint : joints_) {
    joint.controller = std::make_unique<EPOSController>(
      device_name_, protocol_stack_name_, interface_name_, port_name_,
      baudrate_, joint.node_id);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EPOSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EPOSHardwareInterface"), "Cleaning up...");

  for (auto & joint : joints_) {
    joint.controller.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EPOSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EPOSHardwareInterface"), "Activating...");

  // Initialize all joints first
  for (auto & joint : joints_) {
    if (!joint.controller->initialize()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Failed to initialize joint '%s' (node_id=%d)", joint.name.c_str(), joint.node_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Perform homing for joints that require it
  for (auto & joint : joints_) {
    if (joint.homing.enabled) {
      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Starting homing for joint '%s'...", joint.name.c_str());

      if (!performHoming(joint)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Homing failed for joint '%s'", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Homing completed for joint '%s'", joint.name.c_str());
    }
  }

  // Initialize state variables
  for (auto & joint : joints_) {
    // Read current position from hardware
    int position_counts = 0;
    int velocity_rpm = 0;
    joint.controller->getPosition(position_counts);
    joint.controller->getVelocity(velocity_rpm);

    joint.position = (static_cast<double>(position_counts) / joint.counts_per_revolution) * 2.0 * M_PI;
    joint.velocity = (static_cast<double>(velocity_rpm) / 60.0) * 2.0 * M_PI;
    joint.velocity_command = 0.0;
    joint.position_command = joint.position;
    joint.control_mode = ControlMode::NONE;
    joint.target_mode = ControlMode::NONE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EPOSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EPOSHardwareInterface"), "Deactivating...");

  for (auto & joint : joints_) {
    if (joint.control_mode == ControlMode::VELOCITY) {
      joint.controller->haltVelocityMovement();
    } else if (joint.control_mode == ControlMode::POSITION) {
      joint.controller->haltPositionMovement();
    }
    joint.controller->disable();
    joint.controller->close();
    joint.control_mode = ControlMode::NONE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EPOSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : joints_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EPOSHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : joints_) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command));
  }

  return command_interfaces;
}

hardware_interface::return_type EPOSHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Determine target mode for each joint based on requested interfaces
  for (auto & joint : joints_) {
    // Keep current mode by default (don't reset to NONE)
    joint.target_mode = joint.control_mode;

    // Check if this joint should be stopped
    for (const auto & interface : stop_interfaces) {
      if (interface.find(joint.name + "/") != std::string::npos) {
        joint.target_mode = ControlMode::NONE;
        break;
      }
    }

    // Check if this joint should start a new mode
    for (const auto & interface : start_interfaces) {
      // Interface format: "joint_name/interface_type"
      if (interface.find(joint.name + "/" + hardware_interface::HW_IF_POSITION) != std::string::npos) {
        joint.target_mode = ControlMode::POSITION;
        break;
      } else if (interface.find(joint.name + "/" + hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
        joint.target_mode = ControlMode::VELOCITY;
        break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EPOSHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (auto & joint : joints_) {
    if (joint.target_mode == joint.control_mode) {
      continue;
    }

    // Stop current mode
    if (joint.control_mode == ControlMode::VELOCITY) {
      joint.controller->haltVelocityMovement();
    } else if (joint.control_mode == ControlMode::POSITION) {
      joint.controller->haltPositionMovement();
    }

    // Switch to new mode
    if (joint.target_mode == ControlMode::VELOCITY) {
      if (!joint.controller->activateVelocityMode()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to activate velocity mode for joint '%s'", joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
      if (!joint.controller->setVelocityProfile(
            joint.velocity_acceleration, joint.velocity_deceleration)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to set velocity profile for joint '%s'", joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' switched to velocity mode (accel=%u, decel=%u)",
        joint.name.c_str(), joint.velocity_acceleration, joint.velocity_deceleration);
    } else if (joint.target_mode == ControlMode::POSITION) {
      if (!joint.controller->activatePositionMode()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to activate position mode for joint '%s'", joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
      if (!joint.controller->setPositionProfile(
            joint.position_velocity, joint.position_acceleration, joint.position_deceleration)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to set position profile for joint '%s'", joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
      // Initialize position command to current position
      int current_pos = 0;
      joint.controller->getPosition(current_pos);
      joint.position_command = (static_cast<double>(current_pos) / joint.counts_per_revolution) * 2.0 * M_PI;
      RCLCPP_INFO(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "Joint '%s' switched to position mode (vel=%u, accel=%u, decel=%u)",
        joint.name.c_str(), joint.position_velocity, joint.position_acceleration, joint.position_deceleration);
    }

    joint.control_mode = joint.target_mode;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EPOSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : joints_) {
    int position_counts = 0;
    int velocity_rpm = 0;

    if (joint.controller->getPosition(position_counts)) {
      // Store raw encoder counts for soft limit checking
      joint.position_counts = position_counts;
      // Convert encoder counts to radians (using per-joint counts_per_revolution)
      joint.position = (static_cast<double>(position_counts) / joint.counts_per_revolution) * 2.0 * M_PI;
    }

    if (joint.controller->getVelocity(velocity_rpm)) {
      // Convert RPM to rad/s
      joint.velocity = (static_cast<double>(velocity_rpm) / 60.0) * 2.0 * M_PI;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EPOSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : joints_) {
    if (joint.control_mode == ControlMode::VELOCITY) {
      // Convert rad/s to RPM
      long velocity_rpm = static_cast<long>((joint.velocity_command * 60.0) / (2.0 * M_PI));

      // Apply soft limits if enabled
      if (joint.soft_limit.enabled && velocity_rpm != 0) {
        const int pos = joint.position_counts;
        const int min_pos = joint.soft_limit.min_position;
        const int max_pos = joint.soft_limit.max_position;

        // Block movement toward limits
        if (pos <= min_pos && velocity_rpm < 0) {
          // At or beyond min limit, block negative velocity
          static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EPOSHardwareInterface"), steady_clock, 1000,
            "[%s] Soft limit: blocked negative velocity at pos=%d (min=%d)",
            joint.name.c_str(), pos, min_pos);
          velocity_rpm = 0;
        } else if (pos >= max_pos && velocity_rpm > 0) {
          // At or beyond max limit, block positive velocity
          static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EPOSHardwareInterface"), steady_clock, 1000,
            "[%s] Soft limit: blocked positive velocity at pos=%d (max=%d)",
            joint.name.c_str(), pos, max_pos);
          velocity_rpm = 0;
        }
      }

      // Debug log for pan/tilt joints
      if ((joint.name == "pan_joint" || joint.name == "tilt_joint") && velocity_rpm != 0) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_INFO_THROTTLE(
          rclcpp::get_logger("EPOSHardwareInterface"), steady_clock, 500,
          "Writing velocity to '%s': cmd=%.3f rad/s -> %ld RPM",
          joint.name.c_str(), joint.velocity_command, velocity_rpm);
      }

      if (!joint.controller->moveWithVelocity(velocity_rpm)) {
        RCLCPP_WARN(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to set velocity for joint '%s'", joint.name.c_str());
      }
    } else if (joint.control_mode == ControlMode::POSITION) {
      // Convert radians to encoder counts (using per-joint counts_per_revolution)
      long position_counts = static_cast<long>(
        (joint.position_command / (2.0 * M_PI)) * joint.counts_per_revolution);

      if (!joint.controller->moveToPosition(position_counts, true, true)) {
        RCLCPP_WARN(
          rclcpp::get_logger("EPOSHardwareInterface"),
          "Failed to set position for joint '%s'", joint.name.c_str());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool EPOSHardwareInterface::performHoming(JointState & joint)
{
  return homeToLimitAndCenter(joint);
}

bool EPOSHardwareInterface::homeToLimitAndCenter(JointState & joint)
{
  const auto & homing = joint.homing;

  // Set homing parameters
  if (!joint.controller->setHomingParameter(
        homing.homing_acceleration,
        homing.speed_switch,
        homing.speed_index,
        homing.center_offset,  // home_offset: offset from limit to center
        0,                     // current_threshold (not used for limit switch homing)
        0))                    // home_position
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to set homing parameters", joint.name.c_str());
    return false;
  }

  // Activate homing mode
  if (!joint.controller->activateHomingMode()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to activate homing mode", joint.name.c_str());
    return false;
  }

  // Start homing to limit switch
  RCLCPP_INFO(
    rclcpp::get_logger("EPOSHardwareInterface"),
    "[%s] Moving to limit switch (method=%d)...", joint.name.c_str(), homing.homing_method);

  if (!joint.controller->findHome(homing.homing_method)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to start homing", joint.name.c_str());
    return false;
  }

  // Wait for homing to complete
  if (!joint.controller->waitForHomingAttained(homing.timeout_ms)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Timeout waiting for homing to complete", joint.name.c_str());
    joint.controller->stopHoming();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EPOSHardwareInterface"),
    "[%s] Homing attained. Moving to center position (0)...", joint.name.c_str());

  // Move to center position (position 0)
  if (!joint.controller->activatePositionMode()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to activate position mode", joint.name.c_str());
    return false;
  }

  if (!joint.controller->setPositionProfile(
        homing.center_velocity,
        homing.homing_acceleration,
        homing.homing_acceleration))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to set position profile", joint.name.c_str());
    return false;
  }

  if (!joint.controller->moveToPosition(0, true, true)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EPOSHardwareInterface"),
      "[%s] Failed to move to center position", joint.name.c_str());
    return false;
  }

  // Wait for target reached
  auto start_time = std::chrono::steady_clock::now();
  bool target_reached = false;

  while (!target_reached) {
    if (!joint.controller->isTargetReached(target_reached)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "[%s] Failed to get movement state", joint.name.c_str());
      return false;
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > homing.timeout_ms) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EPOSHardwareInterface"),
        "[%s] Timeout waiting for center position", joint.name.c_str());
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EPOSHardwareInterface"),
    "[%s] Now at center position", joint.name.c_str());

  return true;
}

}  // namespace epos

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(epos::EPOSHardwareInterface, hardware_interface::SystemInterface)
