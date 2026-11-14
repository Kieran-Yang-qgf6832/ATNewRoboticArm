cmake_minimum_required(VERSION 3.8)
project(launch_pack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

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

install(
  DIRECTORY launch config rviz 
  DESTINATION share/${PROJECT_NAME}
)

# rosbag 出图工具：读 .ros/ros2bags 下的 bag，把关节命令/状态画成逐关节 PNG。
# RENAME 去掉 .py，ros2 run 在 Linux 下要求文件名与命令名一致（同 tasks/waypoint_check）。
install(PROGRAMS tools/plot_rosbag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_rosbag
)

ament_package()
