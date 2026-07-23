cmake_minimum_required(VERSION 3.8)
project(timesync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(time_sync_node src/time_sync_node.cpp)
target_compile_features(time_sync_node PUBLIC cxx_std_17)
ament_target_dependencies(time_sync_node rclcpp sensor_msgs std_msgs mavros_msgs)

set_target_properties(time_sync_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS time_sync_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
