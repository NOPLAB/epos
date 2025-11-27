cmake_minimum_required(VERSION 3.5)
project (EPOS4)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/EPOSController.cpp
    src/DifferentialDrive.cpp
)

target_link_libraries(${PROJECT_NAME} -lEposCmd)
