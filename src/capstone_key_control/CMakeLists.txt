cmake_minimum_required(VERSION 3.8)
project(capstone_key_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(ur3_key_mode_node src/ur3_key_mode_node.cpp)
target_compile_features(ur3_key_mode_node PUBLIC cxx_std_17)
ament_target_dependencies(
  ur3_key_mode_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS ur3_key_mode_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ur3_key_session DESTINATION lib/${PROJECT_NAME})

ament_package()
