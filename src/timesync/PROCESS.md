# timesync — 系统时钟 GPS 同步

## 简介

`timesync` 将香橙派（RK3566）系统时钟同步到飞控 GPS 时间。
没有 RTC 电池的嵌入式板启动后时钟回到 1970 年，必须依赖此进程纠正。
rosbag2 依赖精确时间戳，时钟不准会污染所有录制数据。

## 核心逻辑

### 时间补偿算法

```
GPS 消息 (/mavros/time_reference)
  │
  ├─ offset_ns = gps_time_ns - host_time_ns
  │
  ├─ RTT 补偿（来自 /mavros/timesync_status）
  │     │
  │     ├─ 0 < RTT < rtt_max_ms(50ms) → offset_ns += RTT / 2
  │     └─ 否则 → 丢弃该次测量
  │
  ├─ 300 样本滑动窗口
  │     │
  │     ├─ ≥10 样本 → 10% trimmed mean（去头尾 10%）
  │     └─ <10 样本 → 简单算术平均
  │
  └─ 平滑后的 offset_ns
```

### 两种时钟校正方式

| 方式 | 函数 | 条件 | 效果 |
|---|---|---|---|
| **Bootstrap**（硬跳） | `clock_settime()` | offset > 500ms 或 INIT→TRACKING | 直接设到正确时间 |
| **Slew**（微调） | `adjtimex(STA_PLL)` | offset ≤ 500ms，在 TRACKING 中 | 频率微调 ~500ppm，每次最多 ±100ms |

### 状态机

```
INIT
  │
  ├─ 收到 /mavros/time_reference
  │     └─ 检查 gps_year >= 2020（拒绝未就绪 GPS）
  │           ├─ offset > bootstrap_threshold(500ms)
  │           │     └─ clock_settime() → 3s 冷却 → TRACKING
  │           └─ offset ≤ 500ms
  │                 └─ adjtimex() → TRACKING
  │
TRACKING
  │
  ├─ 收到 GPS 时间 → 计算 offset → adjtimex() 微调
  │
  └─ GPS 超时 (≥5s 无新数据) → HOLD
         │
         ├─ 维持当前 offset，不更新
         │
         └─ GPS 恢复 → TRACKING
```

### 冷却 / 保护机制

| 机制 | 值 | 说明 |
|---|---|---|
| Bootstrap 冷却 | 3s | `clock_settime` 后 3s 内不再次触发，防止震荡 |
| RTT 过滤 | 0 < RTT < 50ms | 丢弃异常 RTT |
| 最小 GPS 年份 | 2020 | 过滤 GPS 模块初始化时的 1980 年默认值 |
| 滑动窗口 | 300 样本 | 平滑突变 |
| Slew 限制 | ±100ms / 次 | `adjtimex` 单次最大调整量 |

## 话题

| 话题 | 类型 | 方向 | QoS | 说明 |
|---|---|---|---|---|
| `/mavros/time_reference` | `TimeReference` | 订阅 | best_effort | GPS 时间基准 |
| `/mavros/timesync_status` | `TimesyncStatus` | 订阅 | best_effort | MAVLink 时间同步 RTT |
| `~/offset_ms` | `Float32` | 发布 | transient_local | 当前平滑 offset (ms) |
| `~/synced` | `Bool` | 发布 | transient_local | 是否已同步 |
| `~/status` | `String` | 发布 | transient_local | 状态描述 |

## 参数（from config/timesync.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `window_size` | 300 | 滑动窗口样本数 |
| `bootstrap_threshold_ms` | 500.0 | 硬跳阈值 |
| `gps_timeout_sec` | 5.0 | GPS 超时进入 HOLD |
| `min_gps_year` | 2020 | 最小有效 GPS 年份 |
| `post_bootstrap_grace_sec` | 3.0 | Bootstrap 后冷却 |
| `rtt_max_ms` | 50.0 | RTT 丢弃阈值 |

## 依赖

- `mavros_msgs` — TimeReference / TimesyncStatus 消息定义
- `CAP_SYS_TIME` — Linux capability（允许非 root 调用 clock_settime / adjtimex）
- `sensor_msgs` — TimeReference 标准消息
