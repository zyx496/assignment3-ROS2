cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- 海康机器人 MVS SDK ----
# 默认 /opt/MVS，可用 -DMVS_ROOT=/path/to/MVS 或环境变量 MVS_ROOT 覆盖
if(NOT DEFINED MVS_ROOT)
  if(DEFINED ENV{MVS_ROOT})
    set(MVS_ROOT $ENV{MVS_ROOT})
  else()
    set(MVS_ROOT "/opt/MVS")
  endif()
endif()

find_path(MVS_INCLUDE_DIR MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH)
find_library(MVS_LIBRARY MvCameraControl
  PATHS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib/aarch64"
  NO_DEFAULT_PATH)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "未找到海康 MVS SDK（MVS_ROOT=${MVS_ROOT}）。"
    "请先安装 MVS SDK，或用 -DMVS_ROOT= 指定安装路径。")
endif()
message(STATUS "MVS SDK: include=${MVS_INCLUDE_DIR}, lib=${MVS_LIBRARY}")

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include ${MVS_INCLUDE_DIR})
ament_target_dependencies(camera_node rclcpp sensor_msgs)
target_link_libraries(camera_node ${MVS_LIBRARY})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
