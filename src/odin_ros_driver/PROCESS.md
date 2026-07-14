# odin_ros_driver — Odin1 激光雷达 ROS 驱动

## 简介

`odin_ros_driver` 是 Manifold Tech Odin1 传感器（LiDAR + RGB 相机 + IMU）的 ROS 2 驱动。
通过 USB 连接传感器，接收 SLAM 里程计、点云、图像、IMU 等数据并发布为 ROS 话题。
SDK 以静态链接库 `liblydHostApi_arm.a` 形式提供，驱动调用其 API 获取数据。

## 核心逻辑

### 主循环 (10Hz)

```
main()
  │
  ├─ 加载 control_command.yaml
  ├─ 初始化 lidar SDK (lidar_init)
  ├─ 等待 USB 设备 (vendor 0x2207, product 0x0019)
  ├─ 创建 MultiSensorPublisher（所有 ROS 发布者/服务）
  │
  └─ 10Hz 主循环:
       ├─ rclcpp::spin_some()        # 处理 ROS 回调
       ├─ try_process_pair()          # 点云着色渲染（disabled）
       ├─ process_command_file()      # 监控 /tmp/odin_command.txt
       └─ 定时器回调:
            ├─ publishImu()           # 400Hz IMU 数据
            └─ publishPtpOffset()     # PTP 时间同步
```

### 设备回调 — `lidar_device_callback()`

USB 热插拔检测，设备连接时自动初始化：

```
设备插入
  │
  ├─ 停止当前设备数据流
  ├─ 创建 handle：lidar_create_device_handle()
  ├─ 设置日志目录：/data/odin1_device_log/YYYYMMDDHHMM/
  ├─ 配置传感器模式 (SLAM/RAW)
  ├─ 设置数据流类型（IMU、摄像头、点云）
  ├─ 应用自定义参数 (control_command.yaml)
  ├─ 传输图像 mask (config/mask.png)
  └─ 启动数据流：lidar_start()
```

### 数据回调 — `lidar_data_callback()`

根据数据类型分发处理：

```
lidar_data_t.utype:
  │
  ├─ LIDAR_DT_RAW_RGB         → publishRgb()
  │     └─ 直接发 JPEG (CompressedImage，省 CPU)
  │       可选: cv::imdecode → 原始 BGR8 Image (CPU 昂贵，关闭)
  │
  ├─ LIDAR_DT_RAW_IMU         → IMU 线程 (独立线程，400Hz)
  │     └─ 坐标轴转换 (传感器 → ROS) → publishImu()
  │
  ├─ LIDAR_DT_RAW_DTOF        → publishIntensityCloud()
  │     └─ 原始 DTOF 点云 (x/y/z/intensity/confidence)
  │
  ├─ LIDAR_DT_SLAM_CLOUD      → publishPC2XYZRGBA()
  │     └─ SLAM 点云 (带 RGB)
  │
  ├─ LIDAR_DT_SLAM_ODOMETRY   → publishOdometry()
  │     └─ 标准 SLAM 里程计 + map→base_link TF
  │
  ├─ LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ → publishOdometry() (高频)
  │     └─ 400Hz 里程计
  │
  ├─ LIDAR_DT_SLAM_ODOMETRY_TF → publishOdometry(TRANSFORM)
  │     └─ 重定位成功时发布 map→odom TF
  │
  ├─ LIDAR_DT_SLAM_WIWC       → publishWiwc()
  │     └─ 外参: T_CL (camera→lidar), T_IL (imu→lidar)
  │
  ├─ LIDAR_DT_NTP             → PTP 时间同步
  │     └─ 300 样本窗口: 平滑 delay + offset
  │
  └─ LIDAR_DT_DEV_STATUS      → CSV 日志
        └─ SOC 温度、CPU/RAM 占用、传感器 ODR (disabled)
```

### PTP 时间同步

```
传感器 NTP 协议消息
  │
  ├─ delay_ns = (T4 - T1) - (T3 - T2)
  ├─ offset_ns = ((T2 - T1) + (T3 - T4)) / 2
  │
  ├─ 300 样本滑动窗口 (三倍标准差过滤异常值)
  └─ 平滑后: 所有话题时间戳 = 传感器原始时间戳 - smoothed_offset
```

### 命令文件系统 (10Hz 轮询)

监控 `/tmp/odin_command.txt`，接受 `set <param> <value>` 格式：

```
支持的命令:
  set algo_reset 1              # SLAM 重定位
  set save_map 1                # 保存地图（后台线程）
  set camera_ae_mode 1          # 自动曝光
  set camera_awb_mode 1         # 自动白平衡
  set output_odometry 1         # 开/关里程计输出
  set output_cloud_raw 1        # 开/关原始点云
  set output_image_compressed 1 # 开/关压缩图像
  ...
```

`save_map 1` 在后台线程中调用 `lidar_save_map()` API（阻塞，预计耗时 30s）。

## 话题

### 发布 (odin1 namespace)

| 话题 | 类型 | 频率 | 配置开关 |
|---|---|---|---|
| `/odin1/odometry` | `Odometry` | ~20Hz | `output_odometry` |
| `/odin1/odometry_highfreq` | `Odometry` | 400Hz | `output_odometry` |
| `/odin1/imu` | `Imu` | 400Hz | `output_imu` |
| `/odin1/cloud_raw` | `PointCloud2` | 10-15Hz | `output_cloud_raw` |
| `/odin1/cloud_slam` | `PointCloud2` | ~10Hz | `output_cloud_slam` |
| `/odin1/cloud_render` | `PointCloud2` | ~10Hz | `output_cloud_render` (disabled) |
| `/odin1/image/compressed` | `CompressedImage` | 29Hz | `output_image_compressed` |
| `/odin1/image` | `Image` (bgr8) | 29Hz | `output_image` (disabled) |
| `/odin1/path` | `MarkerArray` | ~20Hz | `output_path` |
| `/odin1/wiwc` | `Odometry` | 按需 | — |
| `/odin1/image_overlay/compressed` | `CompressedImage` | 29Hz | `send_overlay` (disabled) |
| `/odin1/cloud_reprojection` | `PointCloud2` | ~10Hz | `send_reprojection` (disabled) |
| `/odin1/depth` | `Image` (32FC1) | ~10Hz | `send_depth` (disabled) |

### 服务

| 服务 | 功能 |
|---|---|
| `/odin1/get_ae` | 查询自动曝光状态 |
| `/odin1/set_ae` | 设置自动曝光参数 |
| `/odin1/get_awb` | 查询自动白平衡状态 |
| `/odin1/set_awb` | 设置自动白平衡参数 |

## CPU 优化配置（RK3566 生产环境）

```yaml
# 仅发布 JPEG 压缩图像，不发布原始 BGR
sendrgb: 0
sendrgbcompressed: 1

# 禁用 CPU 密集型后处理
sendcloudrender: 0       # PCL 点云着色
senddepth: 0             # 深度图转换
sendreprojection: 0     # 点云重投影
sendoverlay: 0           # 图像叠加

# 禁用非关键功能
devstatuslog: 0          # CSV 设备日志
recorddata: 0            # MindCloud 二进制录制
```

## 依赖

- `liblydHostApi_arm.a` — Odin1 SDK 静态库（ARM64）
- `libopencv` — 图像解码（仅 JPEG→BGR 转换时使用）
- `libtbb` — OpenCV 多线程后端
- `libusb-1.0` — USB 设备检测
- `PCL` — 点云处理（仅 cloud_render 模式）
- `Eigen3` — 矩阵运算
