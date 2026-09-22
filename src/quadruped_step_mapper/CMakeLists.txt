cmake_minimum_required(VERSION 3.8)
project(quadruped_step_mapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cmd_vel_to_step_length_node
  src/cmd_vel_to_step_length_main.cpp
  src/cmd_vel_to_step_length_node.cpp
  src/step_length_mapper.cpp
)

add_executable(cmd_vel_to_serial_node
  src/cmd_vel_to_serial_main.cpp
  src/cmd_vel_to_serial_node.cpp
  src/step_length_mapper.cpp
)

target_include_directories(cmd_vel_to_step_length_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(cmd_vel_to_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmd_vel_to_step_length_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)
ament_target_dependencies(cmd_vel_to_serial_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS cmd_vel_to_step_length_node cmd_vel_to_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/serial_start_listener.py
    scripts/serial_step_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
