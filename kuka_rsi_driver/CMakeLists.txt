cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Shared library for robot_manager_base
add_library(robot_manager_base STATIC src/robot_manager_base.cpp)
ament_target_dependencies(robot_manager_base std_msgs kuka_drivers_core controller_manager_msgs)
target_link_libraries(robot_manager_base kuka_drivers_core::communication_helpers)

add_executable(robot_manager_node_extended src/robot_manager_node_extended.cpp)
target_link_libraries(robot_manager_node_extended PRIVATE robot_manager_base)

add_executable(robot_manager_node_rsi_only src/robot_manager_node_rsi_only.cpp)
target_link_libraries(robot_manager_node_rsi_only PRIVATE robot_manager_base)

install(
  TARGETS robot_manager_base robot_manager_node_extended robot_manager_node_rsi_only
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_multi_robot_startup.py)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME})

ament_package()
