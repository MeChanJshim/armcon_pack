cmake_minimum_required(VERSION 3.8)
project(Y2RobMotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Y2Matrix REQUIRED)
find_package(Y2Kinematics REQUIRED)
find_package(Y2Trajectory REQUIRED)
find_package(gforce_control REQUIRED)
find_package(osqp REQUIRED)

set(AMENT_DEP
rclcpp sensor_msgs geometry_msgs
Y2Matrix Y2Kinematics Y2Trajectory gforce_control
)

include_directories(
  include
)

# Main controller sources
set(MAIN_SOURCES
src/robot_motion.cpp
src/hand_guiding.cpp
src/gforce_motion_control.cpp
src/state_monitoring.cpp)

# === Create libraries === 
add_library(robot_runtime_config src/runtime_config.cpp)
target_include_directories(robot_runtime_config PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_runtime_config rclcpp Y2Matrix)
target_link_libraries(robot_runtime_config yaml-cpp)

# robot_motion library
add_library(robot_motion ${MAIN_SOURCES})
target_include_directories(robot_motion PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_motion ${AMENT_DEP})
target_link_libraries(robot_motion robot_runtime_config)

# robot_command library
add_library(robot_command src/robot_command.cpp)
target_include_directories(robot_command PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_command ${AMENT_DEP})
target_link_libraries(robot_command robot_runtime_config)

# === Create executions ===
# singleArm_motion executable
add_executable(singleArm_motion src/singleArm_motion.cpp)
target_link_libraries(singleArm_motion robot_motion)

# singleArm_cmd executable
add_executable(singleArm_cmd src/singleArm_cmd.cpp)
target_link_libraries(singleArm_cmd robot_command)

# robot_measure executable
add_executable(robot_measure src/robot_measure.cpp)
ament_target_dependencies(robot_measure rclcpp std_msgs ${AMENT_DEP})
target_link_libraries(robot_measure robot_runtime_config)

# === Create installs ===
# install libraries
install(TARGETS robot_runtime_config robot_motion robot_command
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install executables
install(TARGETS singleArm_motion singleArm_cmd robot_measure
  DESTINATION lib/${PROJECT_NAME}
)

# install includes
install(DIRECTORY include/
DESTINATION include/)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AMENT_DEP} yaml-cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
