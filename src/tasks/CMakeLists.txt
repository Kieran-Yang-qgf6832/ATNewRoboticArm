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

# 任务层状态机框架 + 各任务（新增任务时把对应 .cpp 加进来即可）
add_library(tasks_core STATIC
  context/task_context.cpp
  task/idel_task.cpp
  task/reset_task.cpp
  task/task_space_segment_task.cpp
  task/cart_traj_task.cpp
  task/admittance_task.cpp
  task/joint_traj_task.cpp
  task/teach_pendant_task.cpp
  task/measure_task.cpp
)

target_include_directories(tasks_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/fsm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/context>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/task>
)

ament_target_dependencies(tasks_core
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  robot_msgs
)

# 任务链 runner
add_executable(task_runner src/task_runner.cpp)
target_link_libraries(task_runner tasks_core)
ament_target_dependencies(task_runner
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  robot_msgs
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
