cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pinocchio REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(arm_model REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

set(LIBCMAES_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(LIBCMAES_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(LIBCMAES_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LIBCMAES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LIBCMAES_USE_OPENMP OFF CACHE BOOL "" FORCE)
set(LIBCMAES_ENABLE_SURROG OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/libcmaes
  ${CMAKE_CURRENT_BINARY_DIR}/libcmaes
  EXCLUDE_FROM_ALL
)

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

pluginlib_export_plugin_description_file(controller_interface arm_ctrl_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface sim_pid_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface arm_real_plugin.xml)

add_library(arm_controller SHARED
  controller/src/controller.cpp
  controller/src/sim_pid_controller.cpp
  controller/src/arm_real_interfaces.cpp
  arm/arm_fsm.cpp
  arm/default6dof_task.cpp
  arm/model_from_urdf.cpp
  lib/calculate/arm.cpp
  lib/calculate/kinamic.cpp
  lib/calculate/trajectory.cpp
  lib/cdc_trans/cdc_trans.cpp
  lib/paramter_identify/excitation_trajectory.cpp
  lib/paramter_identify/fourier_trajectory.cpp
  lib/paramter_identify/paramter_identify.cpp
)

target_include_directories(arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/inc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/arm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/calculate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/cdc_trans>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/fsm>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/paramter_identify>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  arm_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  hardware_interface
  robot_msgs
  std_msgs
  ament_index_cpp
  kdl_parser
  orocos_kdl
)

target_link_libraries(arm_controller
  Eigen3::Eigen
  pinocchio::pinocchio
  PkgConfig::LIBUSB
  libcmaes::cmaes
)

install(TARGETS arm_controller
LIBRARY DESTINATION lib)

install(
  FILES
    arm_ctrl_plugin.xml
    sim_pid_plugin.xml
    arm_real_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY controller/inc/
  DESTINATION include)

install(DIRECTORY arm/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/calculate/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/fsm/
  DESTINATION include/arm_controller
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/paramter_identify/
  DESTINATION include/arm_controller/paramter_identify
  FILES_MATCHING PATTERN "*.hpp")

ament_package()
