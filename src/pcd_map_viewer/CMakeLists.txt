cmake_minimum_required(VERSION 3.8)
project(pcd_map_viewer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)

target_include_directories(pcd_map_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd_map_publisher PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd_map_publisher ${PCL_LIBRARIES})
ament_target_dependencies(pcd_map_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS pcd_map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
