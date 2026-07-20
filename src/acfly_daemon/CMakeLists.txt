cmake_minimum_required(VERSION 3.8)
project(acfly_daemon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(YAML_CPP_LIB yaml-cpp)
if(NOT YAML_CPP_LIB)
  find_package(yaml-cpp REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProcessStatus.msg"
  "srv/ManageProcess.srv"
  "srv/GetProcessStatus.srv"
  DEPENDENCIES std_msgs
)

add_executable(acfly_daemon_node src/acfly_daemon_node.cpp)
target_compile_features(acfly_daemon_node PUBLIC cxx_std_17)
ament_target_dependencies(acfly_daemon_node rclcpp std_msgs std_srvs nav_msgs)
rosidl_target_interfaces(acfly_daemon_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(acfly_daemon_node ${YAML_CPP_LIB})

install(TARGETS acfly_daemon_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
