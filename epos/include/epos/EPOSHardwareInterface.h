#ifndef EPOS_HARDWARE_INTERFACE_H
#define EPOS_HARDWARE_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "epos/EPOSController.h"

namespace epos
{

class EPOSHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EPOSHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class ControlMode
  {
    NONE,
    VELOCITY,
    POSITION
  };

  struct JointState
  {
    std::string name;
    unsigned short node_id;
    double position{0.0};
    double velocity{0.0};
    double velocity_command{0.0};
    double position_command{0.0};
    ControlMode control_mode{ControlMode::NONE};
    ControlMode target_mode{ControlMode::NONE};
    std::unique_ptr<EPOSController> controller;
    double counts_per_revolution{4096.0};  // Per-joint counts per revolution
    // Velocity profile parameters (RPM/s)
    unsigned int velocity_acceleration{10000};
    unsigned int velocity_deceleration{10000};
    // Position profile parameters
    unsigned int position_velocity{5000};        // RPM
    unsigned int position_acceleration{10000};   // RPM/s
    unsigned int position_deceleration{10000};   // RPM/s
  };

  std::vector<JointState> joints_;

  // Hardware parameters
  std::string device_name_{"EPOS4"};
  std::string protocol_stack_name_{"MAXON SERIAL V2"};
  std::string interface_name_{"USB"};
  std::string port_name_{"USB0"};
  unsigned int baudrate_{1000000};

  // Conversion factors
  double counts_per_revolution_{4096.0};  // Encoder counts per revolution
};

}  // namespace epos

#endif  // EPOS_HARDWARE_INTERFACE_H
