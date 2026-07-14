# AcFly MAVROS 启动脚本
# MAVLink ↔ ROS 2 协议转换桥接
# 使用 AcFly 自定义 MAVLink 方言 + 40 插件黑名单裁切为 RK3566 优化

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 mavros_node，加载 AcFly 配置 + APM 插件列表。"""
    mavros_dir = get_package_share_directory("mavros")
    config_yaml = os.path.join(mavros_dir, "launch", "acfly_config.yaml")
    pluginlists_yaml = os.path.join(mavros_dir, "launch", "apm_pluginlists.yaml")

    # MAVROS 节点：串口 /dev/ttyS3 921600 波特率连接飞控
    # namespace=mavros，所有话题挂 /mavros/ 前缀
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[config_yaml, pluginlists_yaml],
    )

    return LaunchDescription([mavros_node])
