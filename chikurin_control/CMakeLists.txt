cmake_minimum_required(VERSION 3.8)
project(chikurin_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(epos REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Homing.srv"
)

# Body tracker library
add_library(body_tracker_lib
  src/body_tracker/person_detector.cpp
  src/body_tracker/target_tracker.cpp
  src/body_tracker/pan_tilt_controller.cpp
  src/body_tracker/scan_controller.cpp
  src/body_tracker/firing_controller.cpp
  src/body_tracker/tracker_ui.cpp
)
target_include_directories(body_tracker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(body_tracker_lib ${OpenCV_LIBS})
ament_target_dependencies(body_tracker_lib
  rclcpp
  std_srvs
)

# Body tracker node
add_executable(body_tracker_node src/body_tracker_node.cpp)
target_link_libraries(body_tracker_node body_tracker_lib)
ament_target_dependencies(body_tracker_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
)
target_include_directories(body_tracker_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(body_tracker_node ${OpenCV_LIBS})

add_executable(pan_tilt_homing_node src/pan_tilt_homing_node.cpp)
ament_target_dependencies(pan_tilt_homing_node
  rclcpp
  std_srvs
  epos
)

add_executable(pan_tilt_joy_node src/pan_tilt_joy_node.cpp)
ament_target_dependencies(pan_tilt_joy_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Build homing service node
add_executable(homing_service_node src/homing_service_node.cpp)
ament_target_dependencies(homing_service_node
  rclcpp
  epos
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(homing_service_node "${cpp_typesupport_target}")

install(TARGETS body_tracker_node pan_tilt_homing_node pan_tilt_joy_node homing_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS body_tracker_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS src/servo_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_trigger_node
)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/download_models.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(body_tracker_lib)

ament_package()
