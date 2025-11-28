# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 package providing a `ros2_control` hardware interface and C++ wrapper library for Maxon EPOS4 motor controllers. The project enables integration of EPOS4 devices with the ROS 2 control framework.

**Communication:** CANopen protocol over USB to EPOS4 hardware devices.

## Build Commands

```bash
# Build (from workspace root containing this repo)
colcon build --packages-select epos shooter

# Build with compile_commands.json for IDE support
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Run the shooter robot
ros2 launch shooter shooter.launch.py
```

## Architecture

### Package Structure
```
epos/                  # Main ROS 2 package
├── epos/              # Hardware interface library
│   ├── include/epos/
│   │   ├── EPOSController.h        # Low-level motor control wrapper
│   │   ├── EPOSHardwareInterface.h # ros2_control SystemInterface
│   │   └── Definitions.h           # Vendor C API bindings
│   └── src/
│       ├── EPOSController.cpp
│       └── EPOSHardwareInterface.cpp
└── shooter/           # Example differential drive robot
    ├── config/controllers.yaml     # ros2_control configuration
    ├── launch/shooter.launch.py
    └── urdf/shooter.urdf.xacro
```

### Layered Design
```
ros2_control framework (diff_drive_controller)
    ↓
EPOSHardwareInterface (ros2_control SystemInterface plugin)
    ↓
EPOSController (C++ wrapper)
    ↓
libEposCmd (Maxon vendor library)
    ↓
USB → EPOS4 Hardware
```

### Key Components

**EPOSHardwareInterface** (`epos/include/epos/EPOSHardwareInterface.h`)
- Implements `hardware_interface::SystemInterface` for ros2_control
- Manages multiple joints, each with its own EPOSController instance
- Exports velocity command and position/velocity state interfaces
- Configured via URDF `<ros2_control>` tags (see `shooter/urdf/shooter.urdf.xacro`)

**EPOSController** (`epos/include/epos/EPOSController.h`)
- Low-level facade for Maxon EPOS4 API
- Handles device initialization, mode activation, motion commands
- Constructor defaults: EPOS4 device, USB0 port, 1 Mbps baudrate, Node ID 1
- All public methods return `bool`; errors via `getLastErrorCode()`

**Definitions.h**
- Vendor-supplied C API header declaring all libEposCmd functions
- ~100+ functions for CANopen/USB communication, motion control, configuration

## URDF Configuration

The hardware interface is configured in URDF with the `<ros2_control>` tag:

```xml
<ros2_control name="epos_system" type="system">
  <hardware>
    <plugin>epos/EPOSHardwareInterface</plugin>
    <param name="device_name">EPOS4</param>
    <param name="port_name">USB0</param>
    <param name="baudrate">1000000</param>
    <param name="counts_per_revolution">4096</param>
  </hardware>
  <joint name="left_wheel_joint">
    <param name="node_id">1</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Critical Dependencies

- **ROS 2** with `ros2_control`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`
- **libEposCmd** - Maxon vendor library (linked via `-lEposCmd`)
- **Supported architectures:** x86, x86_64, ARM (soft-float, hard-float, aarch64)

## EPOSController API Reference

**Operation Modes:**
- Position mode: `activatePositionMode()`, `moveToPosition()`, `setPositionProfile()`
- Velocity mode: `activateVelocityMode()`, `moveWithVelocity()`, `setVelocityProfile()`

**State Management:**
- `initialize()` - Opens device, clears faults, enables motor
- `clearFault()`, `enable()`, `disable()`
- `getFaultState()`, `getEnableState()`

**Motion Feedback:**
- `getPosition()` - Returns quadcounts (4× encoder resolution)
- `getVelocity()` - Returns RPM
- `isTargetReached()`

**Units:**
- Position: quadcounts
- Velocity: RPM
- Acceleration: RPM/s

## Development Environment

- **Standard**: C++14
- **Build System**: ament_cmake (ROS 2)
- **Plugin registration**: `epos_hardware.xml` exports `epos/EPOSHardwareInterface`
