cmake_minimum_required(VERSION 3.8)
project(shooter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Face tracker node
add_executable(face_tracker_node src/face_tracker_node.cpp)
ament_target_dependencies(face_tracker_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)
target_include_directories(face_tracker_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(face_tracker_node ${OpenCV_LIBS})

install(TARGETS
  face_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  urdf
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
