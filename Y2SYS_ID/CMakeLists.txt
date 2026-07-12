cmake_minimum_required(VERSION 3.8)
project(Y2SYS_ID)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cartesian_excitation_node src/cartesian_excitation_node.cpp)
target_compile_features(cartesian_excitation_node PUBLIC cxx_std_17)
ament_target_dependencies(cartesian_excitation_node
  rclcpp
  std_msgs
)

add_executable(response_recorder_node src/response_recorder_node.cpp)
target_compile_features(response_recorder_node PUBLIC cxx_std_17)
ament_target_dependencies(response_recorder_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(online_metrics_node src/online_metrics_node.cpp)
target_compile_features(online_metrics_node PUBLIC cxx_std_17)
ament_target_dependencies(online_metrics_node
  rclcpp
  std_msgs
)

add_executable(system_id_node src/system_id_node.cpp)
target_compile_features(system_id_node PUBLIC cxx_std_17)
ament_target_dependencies(system_id_node
  rclcpp
  std_msgs
)

install(TARGETS
  cartesian_excitation_node
  response_recorder_node
  online_metrics_node
  system_id_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyze_cartesian_id.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  results
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
