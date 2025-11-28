# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 package providing a `ros2_control` hardware interface and C++ wrapper library for Maxon EPOS4 motor controllers. The project enables integration of EPOS4 devices with the ROS 2 control framework.

**Communication:** CANopen protocol over USB to EPOS4 hardware devices.

## Build & Run Commands

### Docker Development (Recommended)

```bash
# Build images
docker compose build

# Development shell (with source mounts for live editing)
docker compose run shooter

# Run with keyboard control
docker compose --profile keyboard up

# Run with face tracking
docker compose --profile face up

# Run Gazebo simulation
docker compose --profile sim up
```

### Native Build (from ROS 2 workspace root)

```bash
# Build packages
colcon build --packages-select epos shooter

# Build with compile_commands.json for IDE support
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Run the shooter robot
ros2 launch shooter shooter.launch.py

# Run face tracker (separate terminal)
ros2 launch shooter face_tracker.launch.py

# Keyboard teleop (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Architecture

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

**FaceTrackerNode** (`shooter/src/face_tracker_node.cpp`)
- OpenCV-based face detection with Haar cascades
- PI controller for tracking (publishes to `/diff_drive_controller/cmd_vel_unstamped`)
- Can use direct camera device or ROS image topic

**Definitions.h** - Vendor-supplied C API header (~100+ functions for CANopen/USB communication)

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

## Dependencies

- **ROS 2 Humble** with `ros2_control`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`
- **libEposCmd** - Maxon vendor library (linked via `-lEposCmd`, included in `EPOS_Linux_Library/`)
- **OpenCV** - Face detection for face_tracker_node
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

**Units:** Position in quadcounts, velocity in RPM, acceleration in RPM/s

## Development Notes

- **C++ Standard**: C++14
- **Build System**: ament_cmake (ROS 2)
- **Plugin registration**: `epos_hardware.xml` exports `epos/EPOSHardwareInterface`
- **Vendor example**: `EPOS_Linux_Library/examples/HelloEposCmd/` demonstrates raw libEposCmd usage
