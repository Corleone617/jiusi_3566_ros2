---
name: acfly-dev
description: AcFly 飞行控制系统开发 — 激光雷达(odin1) + 飞控(acfly mavlink) + ROS2 时间同步 + bag 录制。适用于 RK3566/3588 ARM64 平台。
---

# AcFly 飞行控制系统开发规范

## 项目架构

```
odin1(激光雷达) ──USB──→ odin_ros_driver ──→ /odin1/odometry_highfreq
                                                         /odin1/imu
                                                         /odin1/cloud_slam
                                                         /odin1/cloud_raw
                                                         /odin1/image/compressed

FCU(飞控acfly) ──串口──→ mavros ──→ /mavros/imu/data
                         MAVLink   /mavros/local_position/pose
                                   /mavros/time_reference
                                   /mavros/timesync_status
                                   /mavros/state
                                   /mavros/gpsstatus/gps1/raw
                                   /mavros/global_position/global
                                   /mavros/odometry/out

acfly_odometry ──→ 里程计融合 ──→ TF: map → base_link

timesync ──→ 系统时钟同步(CLOCK_REALTIME)
bag_logd ──→ rosbag2 录制到 /data/bags
acfly_daemon ──→ 守护进程统一管理
```

## 编译规则

### 必须
```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j4" colcon build --executor sequential --allow-overriding libmavconn mavlink mavros mavros_msgs
```

- **绝不使用 `--symlink-install`**。使用该参数会在 install/ 中创建指向 build/ 的软链接，部署到生产板时会全部变成死链。
- RK3566 仅 3.8GB 内存，使用 `-j4` 可能 OOM。如遇 OOM 先用 `sudo fallocate -l 4G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile` 增大 swap。
- `--executor sequential` 防止 cmake configure 阶段多包并行内存叠加。
- `mavros_extras` 已用系统 deb 包（源码中有 `COLCON_IGNORE`），不参与本地编译。

### 增量编译
```bash
MAKEFLAGS="-j4" colcon build --executor sequential --allow-overriding libmavconn mavlink mavros mavros_msgs
```

## 部署规则

生产板是 RK3566 自研板（acfly 用户），无网络、无 RTC 电池。

### 编译后部署
```bash
cd offline
./post_build_deploy.sh
```

此脚本会自动：
1. 配置 `/etc/ld.so.conf.d/ros2-humble.conf` + `ldconfig`（统一走 ld.so.cache）
2. CAP_SYS_TIME 通过 `acfly_daemon.service` 的 `AmbientCapabilities=CAP_SYS_NICE CAP_SYS_TIME` 授予，**不再用 setcap**（避免 colcon build 覆盖二进制后 capability 丢失）
3. 创建 `/data/bags` `/tmp/acfly_logs` 并设权限
4. 安装 GeographicLib 数据集（egm96-5）

### 生产板首次部署
```bash
cd install/offline
./deploy_3566.sh
```

包含完整离线依赖安装、系统 service、USB 修复等。

## 关键配置文件

| 文件 | 作用 | 修改后 |
|------|------|--------|
| `install/mavros/share/mavros/launch/apm_pluginlists.yaml` | mavros 插件黑名单 | 需重启 daemon，不编译 |
| `install/mavros/share/mavros/launch/acfly_config.yaml` | mavros 串口/插件参数 | 需重启 daemon，不编译 |
| `src/timesync/config/timesync.yaml` | timesync 参数 (window_size, 阈值, RTT) | 需 colcon build + 重启 daemon。**勿把 window_size 调大(>40)**，见"时间同步"节 |
| `src/acfly_odometry/config/acfly_odometry.yaml` | 里程计参数 (安装角, 协方差阈值, 输出率) | 需 colcon build + 重启 daemon |
| `install/bag_logd/share/bag_logd/config/bag_logd.yaml` | bag 录制话题/磁盘策略 | 需重启 daemon，不编译 |
| `src/acfly_daemon/config/processes.yaml` | 子进程启动命令/依赖 | 需 colcon build acfly_daemon |
| `src/bag_logd/src/bag_logd_node.cpp` | bag 录制逻辑 | 需 colcon build |
| `src/timesync/src/time_sync_node.cpp` | 系统时钟同步逻辑 | 需 colcon build（cap 由 systemd AmbientCapabilities 授予，免 setcap） |
| `src/acfly_odometry/src/acfly_odometry_node.cpp` | 里程计融合逻辑 | 需 colcon build |
| `src/acfly_daemon/src/acfly_daemon_node.cpp` | 守护进程管理 | 需 colcon build |

**注意：** 以上 install 路径的文件修改后必须同步到同名 src 路径，保证 git 提交的一致性。

**YAML 加载机制（重要）：** 各节点配置加载方式不同，改完 yaml 不生效时先查这个：
- `bag_logd`：节点 C++ 代码自加载（`get_package_share_directory` + `yaml-cpp`），不依赖启动方式。
- `mavros`/`timesync`/`acfly_odometry`：走 `ros2 launch`，由 launch 文件 `parameters=[config_yaml]` 注入。`processes.yaml` 里这三个都用 `args: ["launch", ...]`。
- **切勿**把这三个改回裸 `ros2 run`——那样 yaml 不加载，参数全走代码默认值。

## mavros 插件管理

mavros 当前只启用 8 个必要插件：
```
command, global_position, gps_status, imu, local_position, odometry, sys_status, sys_time
```

所有其他的（setpoint系列、manual_control、rc_io、vision系列等）都在 denylist 中。

### MAVLink msg 331 = ODOMETRY
- 插件 `odometry` 负责 `/mavros/odometry/out` → MAVLink ODOMETRY → FCU
- 飞控 EKF2 融合依赖此消息，**绝对不能禁 odometry 插件**
- 禁用 odometry 会导致飞控收不到里程计数据，定位失败

### MAVLink msg 410
- 飞控返回 `Command 410 -- ack timeout` 是正常现象（ACFLY 不完全支持该 MAVLink 命令）
- 不影响功能，无需处理

## 时间同步

三层结构：
```
MAVLink TIMESYNC(FCU↔mavros) → mavros sys_time → timesync → Linux CLOCK_REALTIME
```

- timesync 的 CAP_SYS_TIME 由 `acfly_daemon.service` 的 `AmbientCapabilities` 授予（**不用 setcap**），colcon build 后无需重新授权
- ldconfig 注册 ROS2 库路径仍是标准做法（plugin loader 等场景需要）
- 无 GPS 时使用飞控 RTC 时间（通过地面站同步过），年份 > 2020 的阈值需要确保通过。FCU RTC 是自由运行的硬件时钟（VBAT 维持），GPS 只在首次定位 `gps_setRtcOnce` 一次性 step，不连续驯服。
- 两次 bootstrap 之间的冷却期为 3 秒
- **必须禁用系统 NTP 服务（chronyd / systemd-timesyncd）**：它们会和 timesync 抢 CLOCK_REALTIME，形成反馈环——实测 chrony 活跃时 offset ~5 分钟必发散到 -500ms+，触发 re-bootstrap 死循环。部署脚本 `deploy_3566.sh` 已自动 `disable --now + mask`，手动部署需执行 `sudo systemctl disable --now chronyd systemd-timesyncd && sudo systemctl mask chronyd systemd-timesyncd`
- **slew 控制环参数（关键）**：`apply_slew` 每秒把 offset 的 **1/20** 喂给 `adjtimex(ADJ_OFFSET_SINGLESHOT)`，配合 `window_size=30`。增益×滞后 < 1 才能稳定收敛。**切勿**把增益调回完整 offset（恒定速率 slew = 积分作用 + 测量滞后 = 必发散，实测 window=300 时 ~40min、window=30 时更快撞 500ms 死循环）。`window_size` 也勿超过 ~40（滞后太大同样不稳）。这两个参数在 `timesync.yaml`，由 `timesync.launch.py` 经 `ros2 launch` 加载（daemon 的 `processes.yaml` 已配 launch 启动）。

## 守护进程

5 个子进程按依赖顺序启动：
```
Phase 0: mavros + odin1_driver（无依赖，并行）
Phase 1: timesync + acfly_odometry（依赖 phase 0 + 等 topic 出现）
Phase 2: bag_logd（依赖 phase 1）
```

- 启动子进程前自动 `pkill -9 -f` 杀同名僵尸进程（防止上次崩溃残留占 USB/串口）
- 崩溃重启指数退避：1s → 2s → 4s → ... → 60s 上限
- 关闭时逆序 SIGINT：bag_logd → acfly_odometry → timesync → odin1_driver → mavros

## 代码规范

### 中文注释
- 所有源码注释和使用说明必须使用中文
- 变量名/函数名使用英文（C++/Python 语法要求）
- 日志消息能中文就中文，方便运维排查

### 修改原则
- **不确定的一定先问**——不要猜测硬件行为或飞控协议细节。涉及 mavlink 消息 ID、飞控参数、硬件引脚等，先确认再改。
- **没有叫优化的地方不要优化**——保持代码朴素，避免引入未请求的抽象层或设计模式。
- **新增日志比静默失败好**——磁盘不足、USB 断开、编译 OOM 等异常必须有可追踪的日志。

### 可读性
- 一个函数做一件事，超过 80 行考虑拆分
- 配置项集中管理（YAML 文件），不要散落在代码里
- 关键状态转换加注释说明触发条件
- 守护进程的定时器驱动状态机顺序固定且加注释标记

## odin1 激光雷达

- USB 设备 ID: `2207:0019`
- 运行时依赖 USB 独占访问，不能两个进程同时打开
- SDK 退出时会触发 `malloc_consolidate(): invalid chunk size` 崩溃（闭源 SDK bug，不影响运行时）
- 需要在 Odometry 模式下运行（`custom_map_mode: 0`）
- `use_host_ros_time: 2` 将 odin1 时间轴对齐到主机时间

## 常见问题排查

| 现象 | 原因 | 处理 |
|------|------|------|
| timesync 反复 re-bootstrap | clock_settime 无 CAP_SYS_TIME | 检查 `acfly_daemon.service` 的 `AmbientCapabilities` 是否包含 `CAP_SYS_TIME`（部署脚本已配置）；用 `getpcaps <pid>` 验证运行时是否真的拿到了 cap |
| timesync offset 发散到 ±数十 ms / 死循环 | (a) chronyd 未 mask → 抢 CLOCK_REALTIME；(b) slew 增益过高（offset/20 被改回完整 offset）或 window_size 过大(>40) → 积分作用+滞后发散 | (a) `sudo systemctl mask chronyd systemd-timesyncd`；(b) 确认 `timesync.yaml` 的 `window_size≤30` 且 `apply_slew` 喂 offset/20（见 time_sync_node.cpp 注释） |
| bag_logd 起不来 | /data/bags 无权限 | `sudo mkdir -p /data/bags && sudo chown` |
| odin1 USB busy | 僵尸进程占 USB | 守护进程新加的僵尸杀灭自动处理 |
| mavros 连接超时 | 串口号错或飞控未上电 | 检查 `/dev/ttyS3` |
| 飞控收不到里程计 | odometry 插件被误禁 | 检查 denylist 不包含 odometry |
