cmake_minimum_required(VERSION 3.8)
project(y2_isaac_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joint_command_bridge src/joint_command_bridge.cpp)
ament_target_dependencies(joint_command_bridge
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  joint_command_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()