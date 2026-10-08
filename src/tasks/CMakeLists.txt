cmake_minimum_required(VERSION 3.8)
project(tasks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# yaml-cpp：yaml_cpp_vendor 只提供 CMake 定位信息，库本身仍需 find_package(yaml-cpp)；
# 本机 vendor 的 ExternalProject 未构建，实际链接到系统 libyaml-cpp-dev (0.8.0)。
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# 任务层核心库：
#   context/  —— 与 arm_controller 的 ROS 接口封装（参数服务 / 话题 / TF）
#   executor/ —— 配置驱动的任务执行器（步骤解析、校验、执行）
# 任务定义在 config/tasks.yaml，新增任务不需要改本文件。
add_library(tasks_core STATIC
  context/task_context.cpp
  executor/step_types.cpp
  executor/task_executor.cpp
)

target_include_directories(tasks_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/context>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/executor>
)

ament_target_dependencies(tasks_core
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  robot_msgs
  std_msgs
  ament_index_cpp
  yaml_cpp_vendor
)

# 注意：ament_target_dependencies 内部使用 plain 签名调用 target_link_libraries，
# 因此这里必须同样使用 plain 签名，不能写 PUBLIC/PRIVATE。
target_link_libraries(tasks_core yaml-cpp::yaml-cpp)

# 任务层入口
add_executable(task_runner src/task_runner.cpp)
target_link_libraries(task_runner tasks_core yaml-cpp::yaml-cpp)
ament_target_dependencies(task_runner
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  robot_msgs
  std_msgs
  ament_index_cpp
  yaml_cpp_vendor
)

# 任务配置：运行时由 task_runner 的 tasks_config 参数指向该文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS tasks_core
  ARCHIVE DESTINATION lib
)

install(TARGETS
  task_runner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
