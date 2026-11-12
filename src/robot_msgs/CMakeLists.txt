cmake_minimum_required(VERSION 3.5)
project(robot_msgs VERSION 3.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/AdmittanceCmd.msg"
  "msg/ServoCtrlCmd.msg"
  "msg/CartTrajCmd.msg"
  "msg/JointTrajCmd.msg"
  "msg/Vector.msg"
  "msg/TargetFace.msg"
  "msg/WaypointTrajCmd.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
