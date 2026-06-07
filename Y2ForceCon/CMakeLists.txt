cmake_minimum_required(VERSION 3.8)
project(Y2ForceCon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Torch
if(NOT Torch_DIR AND DEFINED ENV{LIBTORCH})
  set(Torch_DIR "$ENV{LIBTORCH}/share/cmake/Torch")
endif()
find_package(Torch REQUIRED)

# 기본 핸들 (임포트 타깃 우선)
if(TARGET Torch::Torch)
  set(TORCH_LINK Torch::Torch)
  set(TORCH_INC "")
else()
  set(TORCH_LINK ${TORCH_LIBRARIES})
  set(TORCH_INC  ${TORCH_INCLUDE_DIRS})
endif()

# 디버그 출력 (한 번 확인해보자)
message(STATUS "TORCH_LIBRARIES='${TORCH_LIBRARIES}'")
message(STATUS "TORCH_INCLUDE_DIRS='${TORCH_INCLUDE_DIRS}'")

# LIBTORCH 루트 추정
if(NOT DEFINED _LIBTORCH)
  if(DEFINED ENV{LIBTORCH})
    set(_LIBTORCH "$ENV{LIBTORCH}")
  elseif(DEFINED Torch_DIR)
    get_filename_component(_LIBTORCH "${Torch_DIR}/../../.." ABSOLUTE)
  endif()
endif()

# ★ 가짜 토큰이 하나라도 등장하면 절대경로로 강제 교체
if("${TORCH_LIBRARIES}" MATCHES "(^|;)torch_library(;|$)" OR
   "${TORCH_LIBRARIES}" MATCHES "(^|;)torch(;|$)")
  if(NOT DEFINED _LIBTORCH)
    message(FATAL_ERROR "Cannot infer LIBTORCH; set ENV{LIBTORCH} to your libtorch root.")
  endif()

  set(TORCH_LINK
    "${_LIBTORCH}/lib/libtorch.so"
    "${_LIBTORCH}/lib/libtorch_cpu.so"
    "${_LIBTORCH}/lib/libc10.so"
  )
  if(EXISTS "${_LIBTORCH}/lib/libkineto.a")
    list(APPEND TORCH_LINK "${_LIBTORCH}/lib/libkineto.a")
  endif()

  list(APPEND TORCH_INC "${_LIBTORCH}/include")
endif()

# === Create libraries ===
add_library(Yadmittance_control src/admittance_control.cpp)
target_include_directories(Yadmittance_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(Yadmittance_control rclcpp)

add_library(nrs_3step_faac src/nrs_3step_faac.cpp)
target_include_directories(nrs_3step_faac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nrs_3step_faac rclcpp)

add_library(rl_mass_variation src/rl_mass_variation.cpp)
target_include_directories(rl_mass_variation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(rl_mass_variation rclcpp)
target_link_libraries(rl_mass_variation ${TORCH_LINK})

add_library(rl_md_variation src/rl_md_variation.cpp)
target_include_directories(rl_md_variation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(rl_md_variation rclcpp)
target_link_libraries(rl_md_variation ${TORCH_LINK})

add_library(context_naf src/context_naf.cpp)
target_include_directories(context_naf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(context_naf rclcpp)
target_link_libraries(context_naf ${TORCH_LINK})

add_library(contextNaf_mdGradi src/contextNaf_mdGradi.cpp)
target_include_directories(contextNaf_mdGradi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(contextNaf_mdGradi rclcpp)
target_link_libraries(contextNaf_mdGradi ${TORCH_LINK})

add_library(Naf_mdGradi src/Naf_mdGradi.cpp)
target_include_directories(Naf_mdGradi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(Naf_mdGradi rclcpp)
target_link_libraries(Naf_mdGradi ${TORCH_LINK})

add_library(Naf_massVari src/Naf_massVari.cpp)
target_include_directories(Naf_massVari PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INC})
ament_target_dependencies(Naf_massVari rclcpp)
target_link_libraries(Naf_massVari ${TORCH_LINK})

# === Create executables ===
add_executable(ACexample src/examples/ACexample.cpp)
target_link_libraries(ACexample Yadmittance_control)
ament_target_dependencies(ACexample rclcpp)


# === Create install ===
# install library
install(TARGETS Yadmittance_control nrs_3step_faac rl_mass_variation rl_md_variation context_naf contextNaf_mdGradi Naf_mdGradi Naf_massVari
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
# install executable
install(TARGETS ACexample
  DESTINATION lib/${PROJECT_NAME}
)
# install header
install(DIRECTORY include/
  DESTINATION include/
)

# install checkpoints
install(
  DIRECTORY src/checkpoints/
  DESTINATION share/${PROJECT_NAME}/checkpoints
)

# === Export library target & dependencies ===
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


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
