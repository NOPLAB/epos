cmake_minimum_required(VERSION 3.8)
project(epos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Homing.srv"
)

# Build library
add_library(epos_hardware SHARED
  src/EPOSController.cpp
  src/EPOSHardwareInterface.cpp
)

target_include_directories(epos_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(epos_hardware EposCmd)

ament_target_dependencies(epos_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface epos_hardware.xml)

# Build homing service node
add_executable(homing_service_node src/homing_service_node.cpp src/EPOSController.cpp)
target_include_directories(homing_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(homing_service_node EposCmd)
ament_target_dependencies(homing_service_node rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(homing_service_node "${cpp_typesupport_target}")

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS epos_hardware
  EXPORT export_epos_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS homing_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_epos_hardware HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(epos_hardware)
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
