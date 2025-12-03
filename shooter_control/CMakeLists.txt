cmake_minimum_required(VERSION 3.8)
project(shooter_control)

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

add_executable(body_tracker_node src/body_tracker_node.cpp)
ament_target_dependencies(body_tracker_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  std_msgs
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

install(TARGETS body_tracker_node pan_tilt_homing_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
