# bag_logd 启动脚本
# 飞行数据记录器（黑匣子），武装触发录制
# 定时切片（每 N 秒一个独立 bag） + 磁盘不足时从最旧切片开始逐目录删除

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 通过 ament 索引自动定位 bag_logd 包的安装路径
    # 无需硬编码路径，部署到不同机器自动适配
    bag_logd_dir = get_package_share_directory("bag_logd")
    config_yaml = os.path.join(bag_logd_dir, "config", "bag_logd.yaml")

    # 黑匣子节点：只订阅不发布，不影响其他节点
    bag_logd_node = Node(
        package="bag_logd",
        executable="bag_logd_node",
        name="bag_logd",
        output="screen",  # 打印到终端，方便实时查看状态
        arguments=["--config", config_yaml],
    )

    return LaunchDescription([bag_logd_node])
