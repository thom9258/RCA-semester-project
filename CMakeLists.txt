cmake_minimum_required(VERSION 3.0)
project(robot_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_PROTO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(robot_control main.cpp)
target_link_libraries(robot_control ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES})
