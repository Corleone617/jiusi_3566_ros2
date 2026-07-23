cmake_minimum_required(VERSION 3.8)
project(acfly_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息 CovDiag
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CovDiag.msg"
  DEPENDENCIES std_msgs
)

add_executable(acfly_odometry_node src/acfly_odometry_node.cpp)
target_compile_features(acfly_odometry_node PUBLIC cxx_std_17)
target_link_libraries(acfly_odometry_node Eigen3::Eigen)
ament_target_dependencies(acfly_odometry_node
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2_ros)

# 节点使用本包生成的 CovDiag 消息（Humble 标准做法）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(acfly_odometry_node "${cpp_typesupport_target}")

install(TARGETS acfly_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
