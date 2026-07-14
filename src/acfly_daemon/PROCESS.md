# acfly_daemon — 进程守护

## 简介

`acfly_daemon` 是系统的进程看门狗，负责管理所有子进程的启动、崩溃重启、有序关闭。
RK3566 只有 4GB RAM，通过严格控制启动顺序避免内存峰值叠加。

## 核心逻辑

### YAML 驱动进程管理

从 `config/processes.yaml` 读取进程清单，包括依赖关系、启动命令、等待话题：

| 字段 | 说明 |
|---|---|
| `name` | 进程名 |
| `command` | 启动命令（argv 列表） |
| `dependencies` | 必须处于 RUNNING 状态的依赖进程 |
| `wait_for_topics` | 等待出现在 ROS graph 的话题列表 |
| `min_uptime_before_restart` | 最短运行时间，低于此崩溃视为不稳定 |
| `restart_on_healthy_prefix` | 健康话题匹配时主动重启（定时器循环计数递减） |

### 启动流程

```
daemon 启动 → 加载 YAML → 所有进程置为 PENDING
                 │
    ┌─────────── 2Hz 定时器 ───────────┐
    │                                    │
    handle_pending()                     │
      → 检查依赖是否全部 RUNNING          │
      → 检查 wait_for_topics 是否已发布   │
      → 满足: PENDING→STARTING          │
      → 启动子进程（子进程自己 ros2 run） │
    │                                    │
    check_started()                      │
      → 轮询子进程心跳 /status 话题       │
      → 超时: 视为启动失败，回退 PENDING  │
      → 成功: STARTING→RUNNING          │
    │                                    │
    handle_restarts()                    │
      → 检测 STOPPED 进程               │
      → 指数退避: 1s→2s→4s→8s→...→60s  │
      → 退避到期: STOPPED→PENDING      │
    │                                    │
    check_stop_timeouts()                │
      → 发送 SIGINT 后计时               │
      → 10s 超时 → SIGKILL              │
    │                                    │
    reap_children()                      │
      → waitpid(WNOHANG) 回收僵尸进程    │
    │                                    │
    publish_status()                     │
      → 发布 /acfly_daemon/status        │
```

### 进程状态机

```
PENDING ──→ STARTING ──→ RUNNING
  ↑           │              │
  │           │(超时)        │(崩溃)
  │           ↓              ↓
  └──────── RESTARTING ←────┘
                        │(重启计数用尽)
                        ↓
                     STOPPED
```

### 关机流程

1. 逆依赖顺序（bag_logd 最后启动的最先关闭）
2. 每个进程 SIGINT → 等待 10s → SIGKILL
3. 全部关闭后 daemon 退出

## 话题 / 服务

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/acfly_daemon/status` | `ProcessStatus` | 发布 | 所有进程状态数组 |
| `/acfly_daemon/start` | `ManageProcess` | 服务 | 启动指定进程 |
| `/acfly_daemon/stop` | `ManageProcess` | 服务 | 停止指定进程 |
| `/acfly_daemon/restart` | `ManageProcess` | 服务 | 重启指定进程 |
| `/acfly_daemon/shutdown` | `Trigger` | 服务 | 有序关闭所有进程 |
| `/acfly_daemon/get_status` | `GetProcessStatus` | 服务 | 查询进程状态 |

## 依赖关系（RK3566 4GB 场景）

```
mavros ──→ odin1_driver ──→ timesync (依赖 mavros)
    │            │
    └──→ acfly_odometry (依赖 mavros + odin1)
              │
              └──→ bag_logd (依赖 timesync + acfly_odometry)
```

顺序启动确保：
- `mavros` 先连接飞控，获取 GPS 时间
- `timesync` 在 mavros 运行后同步系统时钟
- `odin1_driver` 独立于 mavros，可并行启动
- `acfly_odometry` 需要 mavros IMU + odin1 odom 同时就绪
- `bag_logd` 最后启动，此时所有录制话题已就绪
