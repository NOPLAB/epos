# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a C++ wrapper library for controlling Maxon EPOS4 brushed/brushless DC motor controllers. The project provides an object-oriented interface (`EPOSController` class) that simplifies interaction with the vendor's C library (`libEposCmd`).

**Communication:** CANopen protocol over USB to EPOS4 hardware devices.

## Build Commands

```bash
# Build project
mkdir -p build && cd build
cmake ..
make

# Run demo application
./EPOS4
```

The executable is generated at `build/EPOS4`.

## Architecture

### Layered Design
```
User Application (main.cpp)
    ↓
EPOSController (C++ wrapper - src/EPOSController.cpp)
    ↓
Definitions.h (C bindings to vendor library)
    ↓
libEposCmd (Maxon vendor shared library - must be installed)
    ↓
USB Hardware → EPOS4 Motor Controller
```

### Key Components

**EPOSController Class** (`include/EPOSController.h`, `src/EPOSController.cpp`)
- Main facade providing high-level motor control API
- Handles device initialization, mode activation, motion commands, and state management
- Constructor sets defaults: EPOS4 device, USB0 port, 1 Mbps baudrate, Node ID 1
- All public methods return `bool` for success/failure
- Errors logged via `logError()` to stderr; retrieve codes with `getLastErrorCode()`

**Definitions.h** (`include/Definitions.h`)
- Vendor-supplied C API header (534 lines)
- Declares all low-level functions from libEposCmd
- Organized by: Communication, Configuration, Operation Modes, State Machine, Motion Info, Low-Level CAN
- 8+ operation modes: Profile Position, Profile Velocity, Homing, Interpolated Position, Position, Velocity, Current, Master Encoder, Step/Direction

### Common Usage Pattern

```cpp
EPOSController motor;
if (!motor.initialize()) {
    // handle error - check motor.getLastErrorCode()
}
motor.activatePositionMode();
motor.setPositionProfile(velocity, accel, decel);
motor.moveToPosition(targetPos, true, true);  // absolute, immediate
// ... check isTargetReached(), getPosition(), etc.
motor.close();
```

## Critical Dependencies

**libEposCmd** - Maxon vendor library
- Must be installed system-wide (linked via `-lEposCmd` in CMakeLists.txt)
- Provides ~100+ C functions wrapping CANopen and USB communication
- Obtain from Maxon Motor AG software distribution for Linux

**Supported architectures:** x86, x86_64, ARM (soft-float, hard-float, aarch64)

## Operation Modes

The EPOS4 controller supports multiple operation modes (activate before use):

- **Profile Position Mode**: Move to positions with velocity/acceleration profiles - `activatePositionMode()`, `moveToPosition()`
- **Profile Velocity Mode**: Run at specified velocities - `activateVelocityMode()`, `moveWithVelocity()`
- **Homing Mode**: Find reference/home position
- **Interpolated Position Mode**: Follow trajectories
- **Position/Velocity/Current Modes**: Direct continuous setpoints
- **Master Encoder Mode**: Synchronized multi-axis following
- **Step/Direction Mode**: Stepper motor compatibility

Each mode has corresponding activate, configure, and command functions in EPOSController.

## State Management

EPOS devices follow a state machine:
1. **Fault State**: Clear with `clearFault()` before any operation
2. **Disabled State**: Default after clearing faults
3. **Enabled State**: Required for motion - call `enable()`
4. **QuickStop State**: Emergency stop state

The `initialize()` method handles fault clearing and enabling automatically. Check states with `getFaultState()` and `getEnableState()`.

## Motion Control Details

**Profile Configuration**: Set velocity/acceleration profiles BEFORE issuing motion commands
- Position profile: `setPositionProfile(velocity, accel, decel)` - units: rpm, rpm/s
- Velocity profile: `setVelocityProfile(accel, decel)` - units: rpm/s

**Position units**: quadcounts (4× encoder counts for quadrature encoders)
**Velocity units**: rpm (revolutions per minute)

**Movement types**:
- Absolute vs Relative: `moveToPosition(targetPos, absolute=true, ...)`
- Immediate vs Buffered: `moveToPosition(..., immediately=true)` - immediate interrupts current movement

**Monitoring**: Call `getPosition()`, `getVelocity()`, `isTargetReached()` for real-time feedback

## Error Handling

- All EPOSController methods return `bool` (true=success, false=failure)
- On failure, retrieve error code: `unsigned int err = motor.getLastErrorCode()`
- Convert to string: `std::string errMsg = motor.getErrorString(err)`
- Vendor library errors retrieved via `VCS_GetErrorInfo()` from Definitions.h
- Comprehensive logging to stderr includes function name and error details

## Development Environment

- **Standard**: C++14
- **Build System**: CMake 3.5+
- **Compiler**: GCC/G++
- **IDE Support**: VSCode with C/C++ extension (configured via `.vscode/c_cpp_properties.json`)
- **compile_commands.json**: Auto-generated in build/ for clangd/IntelliSense
