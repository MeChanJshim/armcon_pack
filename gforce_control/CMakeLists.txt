cmake_minimum_required(VERSION 3.8)
project(gforce_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

if(DEFINED ENV{LIBTORCH})
  if(EXISTS "$ENV{LIBTORCH}/share/cmake/Torch")
    set(_LIBTORCH "$ENV{LIBTORCH}")
  elseif(EXISTS "$ENV{LIBTORCH}/libtorch/share/cmake/Torch")
    set(_LIBTORCH "$ENV{LIBTORCH}/libtorch")
  endif()
endif()

if(NOT Torch_DIR AND DEFINED _LIBTORCH)
  set(Torch_DIR "${_LIBTORCH}/share/cmake/Torch")
endif()
find_package(Torch REQUIRED)

if(TARGET Torch::Torch)
  set(TORCH_LINK Torch::Torch)
  set(TORCH_INC "")
else()
  set(TORCH_LINK ${TORCH_LIBRARIES})
  set(TORCH_INC ${TORCH_INCLUDE_DIRS})
endif()

if(NOT DEFINED _LIBTORCH)
  if(DEFINED Torch_DIR)
    get_filename_component(_LIBTORCH "${Torch_DIR}/../../.." ABSOLUTE)
  endif()
endif()

if("${TORCH_LIBRARIES}" MATCHES "(^|;)torch_library(;|$)" OR
   "${TORCH_LIBRARIES}" MATCHES "(^|;)torch(;|$)")
  if(NOT DEFINED _LIBTORCH)
    message(FATAL_ERROR "Cannot infer LIBTORCH; set LIBTORCH to your libtorch root.")
  endif()

  set(TORCH_LINK
    "${_LIBTORCH}/lib/libtorch.so"
    "${_LIBTORCH}/lib/libtorch_cpu.so"
    "${_LIBTORCH}/lib/libc10.so")
  if(EXISTS "${_LIBTORCH}/lib/libkineto.a")
    list(APPEND TORCH_LINK "${_LIBTORCH}/lib/libkineto.a")
  endif()
  list(APPEND TORCH_INC "${_LIBTORCH}/include")
endif()

add_library(gforce_control
  src/admittance_control.cpp
  src/gforce_control.cpp
  src/six_dof_force_controller.cpp)

target_include_directories(gforce_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(gforce_control SYSTEM PUBLIC
  ${TORCH_INC})

target_link_libraries(gforce_control ${TORCH_LINK} yaml-cpp)
ament_target_dependencies(gforce_control rclcpp)
target_compile_features(gforce_control PUBLIC cxx_std_17)

install(TARGETS gforce_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config checkpoints
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp yaml-cpp)

ament_package()
