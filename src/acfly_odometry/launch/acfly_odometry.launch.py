# acfly_odometry 启动脚本
# 里程计融合：将 Odin1 SLAM 里程计（yaw+位置）与 FCU IMU 姿态（roll/pitch）融合
# 输出重力对齐的组合里程计 /mavros/odometry/out，注入飞控 EKF 估计器
# 同时监控 SLAM 协方差，退化时自动触发 Odin1 复位

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_yaml = os.path.join(
        get_package_share_directory("acfly_odometry"),
        "config", "acfly_odometry.yaml")

    acfly_odometry_node = Node(
        package="acfly_odometry",
        executable="acfly_odometry_node",
        name="acfly_odometry",
        output="screen",
        parameters=[config_yaml],
    )

    return LaunchDescription([acfly_odometry_node])
