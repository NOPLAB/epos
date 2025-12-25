cmake_minimum_required(VERSION 3.8)
project(epos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build (defines NDEBUG, disables debug logs)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Build EPOSController as independent library
add_library(epos_controller SHARED
  src/EPOSController.cpp
)

target_include_directories(epos_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(epos_controller EposCmd)

# Build ros2_control hardware interface
add_library(epos_hardware SHARED
  src/EPOSHardwareInterface.cpp
)

target_include_directories(epos_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(epos_hardware epos_controller)

ament_target_dependencies(epos_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface epos_hardware.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS epos_controller epos_hardware
  EXPORT export_epos
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets
ament_export_targets(export_epos HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(epos_controller epos_hardware)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
