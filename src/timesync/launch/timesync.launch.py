# timesync 启动脚本
# 系统时钟 GPS 同步：将香橙派（RK3566）无 RTC 电池的系统时钟同步到飞控 GPS 时间
# 采用 NTP 风格 RTT 补偿 + adjtimex 微调，分 bootstrap（硬跳）+ slew（微调）两阶段

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 自动定位 timesync 包的安装路径
    timesync_dir = get_package_share_directory("timesync")
    config_yaml = os.path.join(timesync_dir, "config", "timesync.yaml")

    # 时钟同步节点：需要 CAP_SYS_TIME Linux 能力
    # 发布 ~/offset_ms（当前偏差）+ ~/synced（是否已同步）+ ~/status（状态描述）
    time_sync_node = Node(
        package="timesync",
        executable="time_sync_node",
        name="mavros_time_sync",
        output="screen",
        parameters=[config_yaml],
    )

    return LaunchDescription([time_sync_node])
