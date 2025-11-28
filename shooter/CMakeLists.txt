cmake_minimum_required(VERSION 3.8)
project(shooter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Body tracker node
add_executable(body_tracker_node src/body_tracker_node.cpp)
ament_target_dependencies(body_tracker_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)
target_include_directories(body_tracker_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(body_tracker_node ${OpenCV_LIBS})

install(TARGETS
  body_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  urdf
  config
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
