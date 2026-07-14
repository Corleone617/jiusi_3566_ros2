# acfly_odometry 启动脚本
# 里程计融合：将 Odin1 SLAM 里程计（yaw+位置）与 FCU IMU 姿态（roll/pitch）融合
# 输出重力对齐的组合里程计 /mavros/odometry/out，注入飞控 EKF 估计器
# 同时监控 SLAM 协方差，退化时自动触发 Odin1 复位

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    acfly_odometry_node = Node(
        package="acfly_odometry",
        executable="acfly_odometry_node",
        name="acfly_odometry",
        output="screen",
        parameters=[{
            "mount_pitch_deg": 45.0,  # Odin1 传感器前倾 45° 安装角补偿
        }],
    )

    return LaunchDescription([acfly_odometry_node])
