cmake_minimum_required(VERSION 3.8)
project(Y2Kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Y2Matrix REQUIRED)

# include directories
include_directories(
    include
)

# === Create libraries ===

# Kinematics library
add_library(Kinematics src/Kinematics.cpp)
target_include_directories(Kinematics PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(Kinematics PUBLIC rclcpp Y2Matrix)

# KinematicsKUKAiiwa library
add_library(KinematicsKUKAiiwa src/KinematicsKUKAiiwa.cpp)
target_include_directories(KinematicsKUKAiiwa PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(KinematicsKUKAiiwa PUBLIC Kinematics)
ament_target_dependencies(KinematicsKUKAiiwa PUBLIC rclcpp Y2Matrix)

# KinematicsUR10 library
add_library(KinematicsUR10 src/KinematicsUR10.cpp)
target_include_directories(KinematicsUR10 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(KinematicsUR10 PUBLIC Kinematics)
ament_target_dependencies(KinematicsUR10 PUBLIC rclcpp Y2Matrix)

# KinematicsUR10e library
add_library(KinematicsUR10e src/KinematicsUR10e.cpp)
target_include_directories(KinematicsUR10e PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(KinematicsUR10e PUBLIC Kinematics)
ament_target_dependencies(KinematicsUR10e PUBLIC rclcpp Y2Matrix)

# === Create executables ===

# Executable for testing KUKA iiwa kinematics
add_executable(ExampleOfKUKAiiwa src/Examples/ExampleOfKUKAiiwa.cpp)
target_link_libraries(ExampleOfKUKAiiwa KinematicsKUKAiiwa)
ament_target_dependencies(ExampleOfKUKAiiwa rclcpp Y2Matrix)

# Executable for testing UR10 kinematics
add_executable(ExampleOfUR10 src/Examples/ExampleOfUR10.cpp)
target_link_libraries(ExampleOfUR10 KinematicsUR10)
ament_target_dependencies(ExampleOfUR10 rclcpp Y2Matrix)

# === Create install ===
# install libraries
install(TARGETS Kinematics KinematicsKUKAiiwa KinematicsUR10 KinematicsUR10e
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# install executables
install(TARGETS ExampleOfKUKAiiwa ExampleOfUR10
  DESTINATION lib/${PROJECT_NAME}
)

# install header
install(DIRECTORY include/
  DESTINATION include/
)

# export ament dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Y2Matrix)

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
