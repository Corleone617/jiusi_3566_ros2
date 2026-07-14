# bag_logd — 飞行数据记录器

## 简介

`bag_logd` 是飞行数据"黑匣子"，监听飞控状态，解锁时自动录制所有传感器数据为 rosbag2 格式。
RK3566 只有 ~16GB 存储空间，一次 30 分钟飞行产生 ~30GB 数据，通过**定时切片 + 磁盘不足删最旧切片**实现安全滚动录制。

## 核心逻辑

### 状态机

```
节点启动
  │
  ├─ 创建订阅（所有录制话题 + /mavros/state）
  ├─ 创建 disk_check_timer（5s 间隔）
  │
  └─ 等待 mavros 连接
       │
       ├─ Armed 消息 → arm_state_cb()
       │     │
       │     ├─ 1. 检查磁盘空间 (≥ min_start_free_space_mb)
       │     │      └─ 不足 → 跳过，不录制
       │     ├─ 2. close_writer()（如果有残留）
       │     ├─ 3. cleanup_old_bags() 删除所有历史 bag 目录
       │     ├─ 4. open_new_bag() 创建第一个切片目录
       │     ├─ 5. start_rotation_timer() 启动切片定时器
       │     └─ recording_ = true
       │
       ├─ 录制中: rotation_timer 每 rotation_interval_sec 秒触发
       │     │
       │     └─ rotate_bag():
       │         ├─ close_writer()  → 关闭当前切片（独立可读）
       │         └─ open_new_bag()  → 创建下一个切片
       │
       ├─ 录制中: disk_check_cb() 每 5s 触发
       │     │
       │     ├─ 磁盘充足 (≥ min_recording_free_space_mb) → 无操作
       │     │
       │     └─ 磁盘不足 → 清理最旧切片:
       │         ├─ 扫描 /data/bags/ 下所有目录
       │         ├─ 按时间戳名字排序（最旧在前）
       │         ├─ 从最旧开始删除，直到 free_mb ≥ 阈值
       │         └─ 跳过 current_bag_path_（活跃切片不删）
       │
       └─ Disarmed → arm_state_cb()
             ├─ stop_rotation_timer()
             ├─ close_writer()
             ├─ recording_ = false
             └─ 等待下次解锁
```

### Bag 切片结构

`rotation_interval_sec: 300`（5 分钟），每次切片是**独立完整的 rosbag2 目录**：

```
/data/bags/
├── 20260714120000/        # 12:00~12:05 切片（独立 bag 目录）
│   ├── metadata.yaml
│   └── 20260714120000_0.db3
├── 20260714120500/        # 12:05~12:10 切片
│   ├── metadata.yaml
│   └── 20260714120500_0.db3
├── 20260714121000/        # 12:10~12:15 切片
│   ├── metadata.yaml
│   └── 20260714121000_0.db3
└── 20260714121500/        # ← 当前活跃切片（写入中，不删）
    ├── metadata.yaml
    └── 20260714121500_0.db3  (growing)
```

**关键特性**：每个切片目录独立完整，可单独用 `ros2 bag play` 回放，不依赖其他切片。

### 定时切片 — rotate_bag()

```
rotation_timer 触发 (每 300s):
  │
  ├─ 检查 armed_ && recording_（非录制中跳过）
  ├─ close_writer()           → 关闭当前切片（flush + finalize metadata.yaml）
  ├─ 检查 armed_ 仍为 true    → 防止旋转期间 disarm
  └─ open_new_bag()           → 新建目录（新时间戳）
```

断电最多丢失当前切片（≤5 分钟数据）。

### 磁盘不足清理 — disk_check_cb()

```
disk_check_cb 触发 (每 5s):
  │
  ├─ free_mb >= threshold → 无操作
  │
  └─ free_mb < threshold:
       │
       ├─ opendir() 扫描 /data/bags/
       ├─ 收集所有子目录名（时间戳格式 YYYYMMDDHHmmSS）
       ├─ 跳过 current_bag_path_（活跃切片）
       ├─ std::sort() 按时间戳字典序（最旧在前）
       │
       └─ for 每个旧目录 (从最旧到最新):
            ├─ 检查 free_mb >= threshold → break（够了）
            ├─ 跳过 current_bag_path_
            └─ remove_dir() 递归删除整个目录
```

**与旧版本区别**：

| | 旧版 | 新版 |
|---|---|---|
| 切片方式 | rosbag2 内部分片 `.db3`（共享 metadata） | 独立 bag 目录，每个可单独回放 |
| 磁盘不足 | 删整个 bag（丢失全部已录数据） | 按时间戳从最旧切片开始逐目录删 |
| 活跃数据保护 | 无（删除即全丢） | `current_bag_path_` 检查，活跃切片不删 |
| 旋转间隔 | 无自动旋转 | `rotation_timer` 定时触发 |
| 最大丢失 | 触发时全部数据 | 最多旋转间隔时长（5 分钟） |

## 16GB 空间下滚动记录实际效果

30 分钟飞行 ≈ 30GB bag（~1GB/min），16GB 存储，5 分钟切片：

```
t=0    ARM → 创建 12:00/（活跃）
t=5    旋转 → 关闭 12:00/ (5GB) → 创建 12:05/（活跃）
t=10   旋转 → 关闭 12:05/ (5GB) → 创建 12:10/（活跃）   合计 10GB, free ~6GB
t=15   旋转 → 关闭 12:10/ (5GB) → 创建 12:15/（活跃）   合计 15GB, free ~1GB
       ↓ disk_check_cb 触发（free < 2GB）
       → 扫描目录: [12:00, 12:05, 12:10, 12:15(活跃)]
       → 删 12:00/ → free ~6GB → 够了，停止
t=20   旋转 → 关闭 12:15/ → 创建 12:20/                 合计 ~16GB, free ~0GB
       → 删 12:05/ → free ~5GB
t=25   旋转 → 关闭 12:20/ → 创建 12:25/                 合计 ~16GB
       → 删 12:10/ → free ~5GB
t=30   DISARM → close_writer
       → 保留: 12:15/, 12:20/, 12:25/ (最后 15 分钟)
```

**结果**：始终保留最近 ~15 分钟的飞行数据（3 个切片），旧切片择机清理，不会丢失中间段数据。

## 话题

### 订阅（录制）

| 话题 | 类型 | QoS | Depth |
|---|---|---|---|
| `/odin1/odometry_highfreq` | `Odometry` | reliable | 4000 |
| `/odin1/imu` | `Imu` | reliable | 4000 |
| `/odin1/cloud_raw` | `PointCloud2` | reliable | 100 |
| `/odin1/cloud_slam` | `PointCloud2` | reliable | 100 |
| `/odin1/image/compressed` | `CompressedImage` | reliable | 100 |
| `/mavros/imu/data` | `Imu` | sensor_data | 4000 |
| `/mavros/gpsstatus/gps1/raw` | `GPSRAW` | sensor_data | 100 |
| `/mavros/local_position/pose` | `PoseStamped` | sensor_data | 100 |

### 订阅（控制）

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/mavros/state` | `State` | reliable(1) | 飞控状态（解锁/上锁） |

## 参数（from config/bag_logd.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `storage_path` | `/data/bags` | Bag 存储根目录 |
| `rotation_interval_sec` | 300 | 切片间隔 (s)，0=禁用切片 |
| `cleanup_mode` | `auto` | `auto`=ARM 时清理旧 bag，`retain`=保留 |
| `min_start_free_space_mb` | 2048 | ARM 时最低空闲磁盘 (MB)，不足拒绝录制 |
| `min_recording_free_space_mb` | 2048 | 录制时最低空闲 (MB)，不足触发删旧切片 |
| `disk_check_interval_sec` | 5 | 磁盘检查周期 (s) |
| `arm_record.enabled` | true | 武装触发录制开关 |
| `arm_record.mavros_state_topic` | `/mavros/state` | 飞控状态话题 |

## 依赖

- `rosbag2_cpp` / `rosbag2_storage` — bag 读写
- `mavros_msgs` — `State` 消息类型
- `ament_index_cpp` — 自动定位配置文件
- `yaml-cpp` — YAML 配置解析

## 关键实现细节

- `GenericSubscription` — 无需编译时类型，运行时按字符串匹配
- `writer_mutex_` — 保护 writer_ 指针的多线程安全
- `ensure_storage_dir()` — 递归创建父目录
- 异常安全 — `open_new_bag()` 失败时 catch + reset，不崩溃
- 活跃切片保护 — `disk_check_cb` 和 `cleanup_old_bags` 都通过 `current_bag_path_` 检查跳过活跃目录
- 旋转期间 disarm 检查 — `rotate_bag` 关闭后再开前检查 `armed_`，防止 disarmed 后误开新切片
- 切片文件命名 — `format_timestamp()` 输出 `YYYYMMDDHHmmSS`，保证字典序即时间序
