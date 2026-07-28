cmake_minimum_required(VERSION 3.8)
project(capstone_moveit_support)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(planning_only_controller_manager SHARED
  src/planning_only_controller_manager.cpp
)
target_compile_features(planning_only_controller_manager PUBLIC cxx_std_17)
ament_target_dependencies(
  planning_only_controller_manager
  moveit_core
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  moveit_core
  planning_only_controller_manager_plugin.xml
)

install(
  TARGETS planning_only_controller_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
