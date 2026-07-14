# acfly_daemon 启动脚本
# 进程看门狗：管理所有子进程的启动、崩溃重启、有序关闭
# 按依赖顺序启动（mavros → odin1/timesync → odometry → bag_logd）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 自动定位 acfly_daemon 包的安装路径（ament 索引）
    supervisor_dir = get_package_share_directory("acfly_daemon")
    config_yaml = os.path.join(supervisor_dir, "config", "processes.yaml")

    # 主管节点：fork+exec 管理所有子进程
    # 发布 ~/status 话题 + 提供 ~/start ~/stop ~/restart ~/shutdown 服务
    supervisor_node = Node(
        package="acfly_daemon",
        executable="acfly_daemon_node",
        name="acfly_daemon",
        output="screen",
        parameters=[{"config_path": config_yaml}],
    )

    return LaunchDescription([supervisor_node])
