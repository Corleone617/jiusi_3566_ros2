cmake_minimum_required(VERSION 3.8)
project(bag_logd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_library(YAML_CPP_LIB yaml-cpp)
if(NOT YAML_CPP_LIB)
  find_package(yaml-cpp REQUIRED)
endif()

add_executable(bag_logd_node src/bag_logd_node.cpp)
target_compile_features(bag_logd_node PUBLIC cxx_std_17)
ament_target_dependencies(bag_logd_node
  rclcpp rosbag2_cpp rosbag2_compression rosbag2_storage std_msgs mavros_msgs ament_index_cpp)
target_link_libraries(bag_logd_node ${YAML_CPP_LIB})
target_include_directories(bag_logd_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

install(TARGETS bag_logd_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
