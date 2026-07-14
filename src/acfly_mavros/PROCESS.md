# acfly_mavros — 飞控通信桥接

## 简介

`acfly_mavros` 是 MAVLink ↔ ROS 2 的协议转换桥，负责香橙派与 ArduPilot 飞控之间双向通信。
采用 MAVROS 2.x 代码库 + AcFly 自定义 MAVLink 方言 + CPU 优化裁切，专为 RK3566 嵌入式部署。

## 核心逻辑

### 三节点架构

ROS `MultiThreadedExecutor` 中运行三个节点：

```
[物理传输(UART/USB/UDP)] → [libmavconn]
                              │
                              ↓
                        [mavros_router]
                              │
                        ┌─────┴─────┐
                        ↓           ↓
                  [mavros_node]  [mavros_uas]
                 (参数容器)     (插件系统 + 状态机)
```

| 节点 | 功能 |
|---|---|
| `mavros_router` | 消息路由中枢，MAVLink ↔ DDS 双向转发，含消息 ID 黑名单过滤 |
| `mavros_uas` | 飞控通信核心：22 个标准插件 + 37 个扩展插件 |
| `mavros_node` | 参数声明节点 |

### CPU 优化（2026-06-24）

| 优化项 | 原始值 | 优化后 |
|---|---|---|
| executor 线程数 | `clamp(cores, 4, 16)` | `clamp(cores, 1, 2)` |
| spin 超时 | 1000ms | 100ms |
| diagnostic updater 频率 | 1.0 Hz | 0.2 Hz |
| 禁用插件数 | 12 个 | 40 个 |
| Router MAVLink 消息过滤 | 无 | 黑名单 msgid 跳过序列化 |

### 插件黑名单（40 个禁用）

禁用的插件类别：
- **视觉/外部跟踪**: vision_pose, vision_speed, mocap_pose, odom, wheel_odometry
- **传感器**: distance_sensor, gps_rtk, altimeter, wind_estimation
- **执行器**: actuator_control, esc_status, esc_telemetry
- **其他**: hil, log_transfer, tunnel, debug_value, traj_bezier, vfr_hud 等

### 启用的核心插件

| 插件 | 功能 | 发布话题 |
|---|---|---|
| `sys_status` | 飞控状态（连接、模式、武装） | `/mavros/state` |
| `sys_time` | 系统时间同步 | `/mavros/time_reference` |
| `imu_pub` | IMU 数据 | `/mavros/imu/data` |
| `global_position` | GPS 全局位置 | `/mavros/global_position/global` |
| `local_position` | 本地位置/速度 | `/mavros/local_position/pose`, `/mavros/local_position/velocity_local` |
| `gps_rtk_raw` | GPS RTK 原始数据 | `/mavros/gpsstatus/gps{1,2}/raw` |
| `battery` | 电池状态 | `/mavros/battery` |
| `rc_io` | RC 遥控器输入 | `/mavros/rc/in` |
| `param` | 参数读写 | 服务: `~/param/get`, `~/param/set` |
| `command` | 命令发送 | 服务: `~/cmd/arming`, `~/cmd/takeoff`, `~/cmd/land`, `~/cmd/set_mode` |
| `setpoint_raw` | 位置 / 速度 / 姿态控制 | `/mavros/setpoint_raw/local`, `/mavros/setpoint_raw/attitude` |
| `setpoint_position` | 简化位置控制 | `/mavros/setpoint_position/local` |

### 自定义 MAVLink 方言

`acfly.xml` 扩展定义：
- `MAV_AUTOPILOT_REFLEX` (值 20) — AcFly 飞控标识
- `HL_FAILURE_FLAG` — 高级故障标志位（GPS/气压计/加速度计/陀螺仪/磁力计/地形/电池/RC/offboard/发动机/地理围栏/估计器/任务）
- `MAV_MODE` — 飞控模式（ARM/DISARM 变体）
- `MAV_GOTO` — 导航目标动作

## 发布话题（核心）

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/mavros/state` | `State` | reliable(1) | 飞控状态（armed/connected/mode） |
| `/mavros/time_reference` | `TimeReference` | best_effort | GPS 时间基准（timesync 使用） |
| `/mavros/imu/data` | `Imu` | best_effort | FCU IMU（加速度/角速度/姿态） |
| `/mavros/local_position/pose` | `PoseStamped` | best_effort | 本地位置 |
| `/mavros/local_position/velocity_local` | `TwistStamped` | best_effort | 本地速度 |
| `/mavros/global_position/global` | `NavSatFix` | best_effort | GPS 经纬度 |
| `/mavros/battery` | `BatteryState` | reliable(1) | 电池状态 |
| `/mavros/rc/in` | `RCOut` | reliable(10) | RC 遥控器输入 |
| `/mavros/gpsstatus/gps1/raw` | `GPSRAW` | best_effort | GPS1 原始数据 |
| `/mavros/home_position/home` | `HomePosition` | reliable(1) | Home 位置 |

## 订阅话题（核心）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/mavros/setpoint_position/local` | `PoseStamped` | 位置控制 |
| `/mavros/setpoint_raw/local` | `PositionTarget` | 精细位置/速度/加速度控制 |
| `/mavros/setpoint_velocity/cmd_vel` | `TwistStamped` | 速度控制 |

## 服务（核心）

| 服务 | 功能 |
|---|---|
| `/mavros/cmd/arming` | 解锁/上锁 |
| `/mavros/cmd/takeoff` | 起飞（相对高度） |
| `/mavros/cmd/land` | 降落 |
| `/mavros/set_mode` | 设置飞行模式 |
| `/mavros/param/get` | 读取飞控参数 |
| `/mavros/param/set` | 设置飞控参数 |

## 传输方式

| 连接方式 | 参数 | 场景 |
|---|---|---|
| 串口 | `tcp-l://:5760` bridged | RK3566 UART → Pixhawk TELEM2 |
| UDP | `udp://:14550@192.168.x.x` | 网络连接 |

## 依赖

- `libmavconn` — MAVLink 传输层库
- `mavros_msgs` — ROS 2 消息定义
- `GeographicLib` — WGS-84 坐标转换
- `diagnostic_updater` — 健康状态
- `Eigen3` — 矩阵运算（IMU 姿态转换）
