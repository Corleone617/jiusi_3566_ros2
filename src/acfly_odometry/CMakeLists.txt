cmake_minimum_required(VERSION 3.8)
project(acfly_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(acfly_odometry_node src/acfly_odometry_node.cpp)
target_compile_features(acfly_odometry_node PUBLIC cxx_std_17)
target_link_libraries(acfly_odometry_node Eigen3::Eigen)
ament_target_dependencies(acfly_odometry_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros)

install(TARGETS acfly_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
