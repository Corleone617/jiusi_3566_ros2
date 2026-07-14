# acfly_odometry — 里程计融合

## 简介

`acfly_odometry` 将 Odin1 SLAM 里程计（yaw+位置）与飞控 IMU 姿态（重力对齐的 roll/pitch）融合，
输出重力对齐的组合里程计，注入飞控 EKF 估计器。同时监控 SLAM 协方差，发散时自动触发复位。

## 核心逻辑

### 融合算法

```
Odin1 SLAM odom（高频）     FCU IMU（重力对齐 roll/pitch）
        │                            │
        ▼                            ▼
   提取 yaw + position         提取 roll + pitch
        │                            │
        └──────────┬─────────────────┘
                   ▼
         q_fused = Rz(yaw) * Ry(pitch) * Rx(roll)
                   │
                   ▼
         增加安装补偿（默认 -45° pitch）
          q_body = q_mount⁻¹ * q_fused
                   │
                   ▼
         速率转换到机体坐标系
                   │
                   ▼
         发布 /mavros/odometry/out
```

### 安装角补偿

传感器安装角（`q_mount`）将传感器帧的速度转换到机体帧：

```
q_mount = Rz(mount_yaw) * Ry(mount_pitch) * Rx(mount_roll)
v_body  = q_mount⁻¹ * (v_slam_in_slam_frame)
```

默认参数：`mount_pitch = -45°`（传感器前倾 45° 安装）。

### 协方差退化检测

```
SLAM odom incoming (20Hz)
  │
  ├─ 检查 pos_cov[0] > cov_pos_threshold (0.25m²)
  ├─ 检查 att_cov[0] > cov_att_threshold_deg2 (1.0deg²)
  │
  ├─ 累计超限计数
  │     │
  │     ├─ 计数 < cov_degrade_bound(30次) → 继续
  │     │
  │     └─ 计数 >= 30次 (≈1.5s) → 触发复位
  │           │
  │           ├─ 检查复位冷却时间 (≥5s 上次复位)
  │           └─ echo "set algo_reset 1" > /tmp/odin_command.txt
  │
  └─ 正常 → 计数归零
```

## 状态机

```
INIT (对齐窗口 5s)
  │
  ├─ 收到 FCU IMU roll/pitch（重力对齐） + SLAM odom yaw
  │     → 建立融合四元数
  │
  └─ 对齐窗口结束 → RUNNING
        │
        ├─ 收到新 SLAM odom → 替换 yaw + position，保持 roll/pitch
        ├─ 协方差超过阈值 → 计数累积
        │     └─ 超限触发 → RESET (写命令到文件，不影响发布)
        └─ 正常 → 发布 /mavros/odometry/out
```

## 话题

| 话题 | 类型 | 方向 | QoS | 说明 |
|---|---|---|---|---|
| `/odin1/odometry_highfreq` | `Odometry` | 订阅 | best_effort | Odin1 SLAM 高频里程计 (400Hz) |
| `/mavros/imu/data` | `Imu` | 订阅 | best_effort | FCU IMU 数据 |
| `/mavros/odometry/out` | `Odometry` | 发布 | reliable(10) | 融合后里程计（注入飞控 EKF） |
| `/tf` (map→base_link) | `TransformStamped` | 发布 | — | 动态 TF |

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `align_duration_s` | 5.0 | 对齐窗口时长 |
| `mount_roll` | 0.0 | 传感器安装 roll |
| `mount_pitch` | -45.0 | 传感器安装 pitch |
| `mount_yaw` | 0.0 | 传感器安装 yaw |
| `cov_pos_threshold` | 0.25 | 位置协方差触发阈值 (m²) |
| `cov_att_threshold_deg2` | 1.0 | 姿态协方差触发阈值 (deg²) |
| `cov_degrade_bound` | 30 | 持续超限计数阈值 |
| `reset_cooldown_sec` | 5.0 | 复位冷却时间 |

## 依赖

- `nav_msgs`, `sensor_msgs`, `geometry_msgs` — ROS 2 标准消息
- `tf2_ros` — 坐标变换广播
- `Eigen3` — 四元数/矩阵运算
