cmake_minimum_required(VERSION 3.8)
project(arm_base_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)

# MuJoCo 基座驱动插件：在 pre_step 里按解析轨迹写 mocap 基座的位姿，
# 在 update 里以控制频率发布 BaseMotionState。
add_library(arm_base_motion_plugin SHARED
  src/base_motion_plugin.cpp
)
target_include_directories(arm_base_motion_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_base_motion_plugin
  rclcpp
  pluginlib
  realtime_tools
  geometry_msgs
  robot_msgs
  mujoco_ros2_control_plugins
)
# 注意：不能把 mujoco_vendor 交给 ament_target_dependencies——它的导出信息里
# INTERFACE_LINK_LIBRARIES 是一个相对路径（opt/mujoco_vendor/lib/libmujoco.so），
# 会被链接器当成 -lopt/mujoco_vendor/lib/libmujoco.so。上游插件用的是 mujoco::mujoco
# 这个 imported target，这里保持一致；头文件也由它带进来。
target_link_libraries(arm_base_motion_plugin mujoco::mujoco)
# 插件描述必须挂在 mujoco_ros2_control_plugins 这个包名下：加载方用
# pluginlib::ClassLoader<MuJoCoROS2ControlPluginBase>("mujoco_ros2_control_plugins", ...)
# 扫的是该包的 ament 索引，挂到本包名下会找不到。
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins arm_base_motion_plugin.xml)
install(TARGETS arm_base_motion_plugin LIBRARY DESTINATION lib)

# world→base_link 动态 TF：订阅基座状态话题，补上 robot_state_publisher 不再发布的基座变换。
add_executable(base_motion_tf_node src/base_motion_tf_node.cpp)
ament_target_dependencies(base_motion_tf_node
  rclcpp
  geometry_msgs
  tf2_ros
  robot_msgs
)
install(TARGETS base_motion_tf_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_float_mjcf.py
  scripts/evaluate_inertial_hold.py
  DESTINATION lib/${PROJECT_NAME}
)

# 不启用 ament_lint_auto：本机 colcon 环境的 python 缺少 catkin_pkg，
# 打开 lint 会让 ament_package_xml 在配置阶段直接失败（与本包代码无关）。

ament_package()
