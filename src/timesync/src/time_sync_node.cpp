// timesync — 系统时钟 GPS 同步节点
// 将香橙派（RK3566，无 RTC 电池）的系统时钟同步到飞控 GPS 时间
// 采用 NTP 风格 RTT 补偿 + adjtimex 微调，分两阶段：
//   1. bootstrap（硬跳）：offset > 500ms 时用 clock_settime 直接设置
//   2. slew（微调）：offset ≤ 500ms 时用 adjtimex(STA_PLL) 频率微调，每次最多 ±100ms
//
// 状态机：
//   INIT → 首次收到有效 GPS 时间 → do_bootstrap + 进入 TRACKING
//   TRACKING → 每次 GPS 时间回调 → apply_slew 微调
//   TRACKING → GPS 超时 ≥5s → HOLD（保持最后频率，不更新）
//   HOLD → GPS 恢复 → 重新 apply_slew

#include <algorithm>
#include <sys/timex.h>
#include <sys/time.h>
#include <ctime>
#include <deque>
#include <mutex>
#include <string>
#include <cmath>
#include <chrono>
#include <vector>
#include <cstdarg>
#include <cstring>
#include <optional>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class TimeSyncNode : public rclcpp::Node
{
public:
  TimeSyncNode()
    : Node("mavros_time_sync"),
    state_(State::INIT),
    synced_(false),
    gps_lost_count_(0)
  {
    // 启动时清零内核时钟纪律状态：旧版 apply_slew 误用 STA_PLL 会在内核累积频率误差，
    // 且该状态是系统级全局的、跨进程重启不消失。必须显式清零，否则重启 timesync 仍带病运行。
    reset_kernel_clock_discipline();

    // ---- 参数声明 ----
    declare_parameter("window_size", 300);           // 滑动窗口样本数
    declare_parameter("bootstrap_threshold_ms", 500.0); // bootstrap 触发阈值 (ms)
    declare_parameter("gps_timeout_sec", 5.0);       // GPS 超时进入 HOLD (s)
    declare_parameter("min_gps_year", 2020);          // 最小有效 GPS 年份
    declare_parameter("post_bootstrap_grace_sec", 3.0);// bootstrap 后冷却时间 (s)
    declare_parameter("print_interval_sec", 5.0);     // 状态打印间隔 (s)
    declare_parameter("rtt_max_ms", 50.0);            // RTT 丢弃阈值 (ms)

    window_size_ = get_parameter("window_size").as_int();
    bootstrap_threshold_ms_ = get_parameter("bootstrap_threshold_ms").as_double();
    gps_timeout_sec_ = get_parameter("gps_timeout_sec").as_double();
    min_gps_year_ = get_parameter("min_gps_year").as_int();
    post_bootstrap_grace_sec_ = get_parameter("post_bootstrap_grace_sec").as_double();
    print_interval_sec_ = get_parameter("print_interval_sec").as_double();
    rtt_max_ns_ = static_cast<int64_t>(get_parameter("rtt_max_ms").as_double() * 1e6);

    // ---- 发布者 ----
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();         // 新订阅者能收到最新值

    offset_pub_ = create_publisher<std_msgs::msg::Float64>("~/offset_ms", 10);
    synced_pub_ = create_publisher<std_msgs::msg::Bool>("~/synced", latched_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", latched_qos);

    // ---- 订阅者 ----
    auto default_qos = rclcpp::QoS(10).best_effort(); // best_effort：高频率消息允许丢包

    // 订阅 GPS 时间基准（来自 mavros sys_time 插件）
    time_ref_sub_ = create_subscription<sensor_msgs::msg::TimeReference>(
      "/mavros/time_reference", default_qos,
      std::bind(&TimeSyncNode::time_ref_cb, this, std::placeholders::_1));

    // 订阅 MAVLink 时间同步状态（获取 RTT）
    tsync_sub_ = create_subscription<mavros_msgs::msg::TimesyncStatus>(
      "/mavros/timesync_status", default_qos,
      std::bind(&TimeSyncNode::tsync_cb, this, std::placeholders::_1));

    // 监控定时器：10Hz 检查 GPS 超时 + 定期打印同步精度
    monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TimeSyncNode::monitor_cb, this));

    last_print_time_ = get_clock()->now();
    publish_status("INIT: waiting for GPS time reference");

    RCLCPP_INFO(get_logger(),
      "TimeSync node started. window=%d, bootstrap_thr=%.1fms, gps_timeout=%.1fs, grace=%.1fs, rtt_max=%.0fms",
      window_size_, bootstrap_threshold_ms_, gps_timeout_sec_, post_bootstrap_grace_sec_,
      rtt_max_ns_ / 1e6);
  }

private:
  // 同步状态枚举
  enum class State { INIT, BOOTSTRAP, TRACKING, HOLD };

  // 时间同步状态回调：缓存最新 RTT（使用原子变量保证线程安全）
  void tsync_cb(const mavros_msgs::msg::TimesyncStatus::SharedPtr msg)
  {
    rtt_ns_.store(static_cast<int64_t>(msg->round_trip_time_ms * 1e6));
  }

  // GPS 时间基准回调：核心同步逻辑
  void time_ref_cb(const sensor_msgs::msg::TimeReference::SharedPtr msg)
  {
    // 过滤无效时间：GPS 模块未就绪时时间戳可能为 1980 年
    if (msg->time_ref.sec < min_gps_year_to_epoch()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GPS time not valid yet (year < %d), skipping", min_gps_year_);
      return;
    }

    last_gps_time_ = get_clock()->now();              // 记录最后收到 GPS 时间的时间
    gps_lost_count_ = 0;                              // 重置丢失计数

    // 计算偏移：GPS 时间 - 系统时间
    // offset > 0 → 系统时钟偏慢
    int64_t gps_ns = rclcpp_time_to_ns(msg->time_ref);
    int64_t host_ns = rclcpp_time_to_ns(msg->header.stamp);
    int64_t offset_ns = gps_ns - host_ns;

    // NTP 风格 RTT 补偿：当 0 < RTT < 50ms 时，offset += RTT / 2
    // 假设往返路径对称，取一半补偿传输延迟
    int64_t rtt = rtt_ns_.load();
    if (rtt > 0 && rtt < rtt_max_ns_)
      offset_ns += rtt / 2;

    // 存入滑动窗口
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      offset_window_.push_back(offset_ns);
      if (static_cast<int>(offset_window_.size()) > window_size_) {
        offset_window_.pop_front();                  // 窗口满则丢弃最旧值
      }
    }

    // 计算平滑后的偏移
    double smoothed_ms = get_smoothed_offset_ms();

    // 状态机分发
    switch (state_) {
      case State::INIT:
        handle_init(smoothed_ms);
        break;
      case State::TRACKING:
        handle_tracking(smoothed_ms);
        break;
      case State::HOLD:
        handle_hold(smoothed_ms);
        break;
      default:
        break;
    }

    publish_offset(smoothed_ms);
  }

  // INIT 状态处理：首次收到有效 GPS 时间 → 执行 bootstrap 并进入 TRACKING
  void handle_init(double smoothed_ms)
  {
    RCLCPP_INFO(get_logger(), "First valid GPS time received, offset: %.1fms", smoothed_ms);
    if (!do_bootstrap(smoothed_ms)) {                  // 硬跳失败则等待下次重试
      RCLCPP_ERROR(get_logger(),
        "BOOTSTRAP FAILED — check CAP_SYS_TIME capability, staying in INIT");
      return;
    }
    state_ = State::TRACKING;
    synced_ = true;
    publish_synced(true);
    bootstrap_time_ = get_clock()->now();             // 记录 bootstrap 时间用于冷却
    publish_status("TRACKING: clock synced, slew mode");
    RCLCPP_INFO(get_logger(), "BOOTSTRAP done, system clock set to GPS time");
  }

  // TRACKING 状态处理：每次 GPS 时间回调时微调时钟
  void handle_tracking(double smoothed_ms)
  {
    double abs_offset = std::abs(smoothed_ms);

    // bootstrap 冷却期：冷却期内即使 offset 大也不重新硬跳
    if (bootstrap_time_.has_value()) {
      double grace_elapsed = (get_clock()->now() - *bootstrap_time_).seconds();
      if (grace_elapsed < post_bootstrap_grace_sec_) {
        apply_slew(smoothed_ms);                     // 冷却期只微调
        return;
      }
      bootstrap_time_.reset();                       // 冷却结束
    }

    // 偏移超过阈值 → 重新 bootstrap
    if (abs_offset > bootstrap_threshold_ms_) {
      RCLCPP_WARN(get_logger(),
        "Offset %.1fms exceeds threshold %.1fms, re-bootstrapping",
        smoothed_ms, bootstrap_threshold_ms_);
      if (!do_bootstrap(smoothed_ms)) {
        RCLCPP_ERROR(get_logger(),
          "re-bootstrap FAILED — check CAP_SYS_TIME, continuing with slew");
        apply_slew(smoothed_ms);                     // 降级到微调
        return;
      }
      bootstrap_time_ = get_clock()->now();
      publish_status("TRACKING: re-bootstrapped (large offset)");
      return;
    }

    apply_slew(smoothed_ms);

    // 偏差小于 10ms 时不频繁打印
    if (abs_offset < 10.0) {
      publish_status("TRACKING: offset %.2fms", smoothed_ms);
    }
  }

  // HOLD 状态处理：GPS 恢复后重新进入 TRACKING
  void handle_hold(double smoothed_ms)
  {
    RCLCPP_INFO(get_logger(), "GPS restored from HOLD, offset: %.1fms", smoothed_ms);

    if (std::abs(smoothed_ms) > bootstrap_threshold_ms_) {
      if (!do_bootstrap(smoothed_ms)) {
        RCLCPP_ERROR(get_logger(),
          "HOLD restore bootstrap FAILED — check CAP_SYS_TIME, using slew");
        apply_slew(smoothed_ms);
      } else {
        bootstrap_time_ = get_clock()->now();
      }
    } else {
      apply_slew(smoothed_ms);
    }

    state_ = State::TRACKING;
    synced_ = true;
    publish_synced(true);
    publish_status("TRACKING: GPS restored");
  }

  // 硬跳：用 clock_settime(CLOCK_REALTIME) 一步到位
  // 需要 CAP_SYS_TIME Linux capability
  bool do_bootstrap(double smoothed_ms)
  {
    int64_t offset_ns = get_median_offset_ns();      // 用中位数而不是平滑值
    if (offset_ns == 0) return false;

    int64_t host_ns = get_host_time_ns();
    int64_t target_ns = host_ns + offset_ns;

    struct timespec ts;
    ts.tv_sec = target_ns / 1000000000LL;
    ts.tv_nsec = target_ns % 1000000000LL;

    if (clock_settime(CLOCK_REALTIME, &ts) != 0) {
      RCLCPP_ERROR(get_logger(),
        "clock_settime failed: %s (need CAP_SYS_TIME?)", strerror(errno));
      return false;
    }

    // 硬跳后清零内核频率纪律状态：clock_settime 只重置相位，
    // 内核累积的频率误差（若有）会立即把时钟再次拉偏。必须一并清零。
    reset_kernel_clock_discipline();

    // 硬跳后清除窗口旧数据（旧 offset 已无意义）
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      offset_window_.clear();
    }

    RCLCPP_INFO(get_logger(),
      "clock_settime: offset was %.2fms (median %.2fms)",
      smoothed_ms, static_cast<double>(offset_ns) / 1e6);
    return true;
  }

  // 微调：用 ADJ_OFFSET_SINGLESHOT 渐进式 slew（单次渐进调整）
  // 重要：绝不使用 STA_PLL/ADJ_OFFSET 的内核 PLL 频率纪律——
  //   旧版用 STA_PLL 每秒喂 offset，内核 PLL 会持续积分频率估计，
  //   ~5 分钟后频率 windup 导致控制环发散（offset 从 0 冲到 -500ms+，触发 re-bootstrap 死循环）。
  //   ADJ_OFFSET_SINGLESHOT 是单次 slew，不激活 PLL、不积分频率，根除 windup。
  //   内核默认以 ~500ppm 速率渐进消化 offset，配合每秒测量+重喂可稳定跟踪。
  //   平台无关的标准 Linux API，RK3566/3588 ARM64 同样适用。
  void apply_slew(double offset_ms)
  {
    struct timex tx = {};
    tx.modes = ADJ_OFFSET_SINGLESHOT;                 // 单次渐进 slew（无 PLL 频率纪律）

    long offset_us = static_cast<long>(offset_ms * 1000.0);
    long max_step_us = 100000;                        // 每次最多 ±100ms
    if (offset_us > max_step_us) offset_us = max_step_us;
    if (offset_us < -max_step_us) offset_us = -max_step_us;

    tx.offset = offset_us;

    if (adjtimex(&tx) < 0)
      RCLCPP_ERROR(get_logger(), "adjtimex failed: %s", strerror(errno));
  }

  // 清零内核时钟纪律状态：清除 STA_PLL 等纪律位 + 频率校正归零
  // 用于启动时清理旧代码遗留的内核 windup，以及每次 bootstrap 后防止残余频率误差
  // 把时钟再次拉偏（clock_settime 只重置相位，不清频率）
  void reset_kernel_clock_discipline()
  {
    struct timex tx = {};
    tx.modes = ADJ_STATUS | ADJ_FREQUENCY;
    tx.status = 0;                                    // 清除所有纪律位（STA_PLL/STA_FLL/STA_UNSYNC 等）
    tx.freq = 0;                                      // 频率校正归零
    if (adjtimex(&tx) < 0) {
      RCLCPP_WARN(get_logger(), "reset_kernel_clock_discipline failed: %s", strerror(errno));
    }
  }

  // 监控回调：10Hz，检查 GPS 超时 + 定期打印同步精度
  void monitor_cb()
  {
    if (state_ == State::INIT || state_ == State::HOLD) {
      return;
    }

    if (!last_gps_time_) {
      return;
    }

    // GPS 超时检查：超过 gps_timeout_sec 秒无新数据进入 HOLD
    double elapsed = (get_clock()->now() - *last_gps_time_).seconds();
    if (elapsed > gps_timeout_sec_) {
      gps_lost_count_++;
      if (gps_lost_count_ >= 3 && state_ == State::TRACKING) {
        // 连续 3 次检查超时才进入 HOLD，防止误判
        RCLCPP_WARN(get_logger(), "GPS timeout (%.1fs), entering HOLD", elapsed);
        state_ = State::HOLD;
        synced_ = false;
        publish_synced(false);
        publish_status("HOLD: GPS lost, holding last freq");
      }
    }

    // 定期打印同步精度
    auto now = get_clock()->now();
    if ((now - last_print_time_).seconds() >= print_interval_sec_) {
      print_sync_accuracy();
      last_print_time_ = now;
    }
  }

  // 打印当前同步精度信息
  void print_sync_accuracy()
  {
    double offset_ms = get_smoothed_offset_ms();
    int samples;
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      samples = static_cast<int>(offset_window_.size());
    }

    const char * state_label;
    if (bootstrap_time_.has_value() &&
        (get_clock()->now() - *bootstrap_time_).seconds() < post_bootstrap_grace_sec_) {
      state_label = "GRACE";                          // bootstrap 冷却期
    } else {
      state_label = "TRACK";
    }

    RCLCPP_INFO(get_logger(), "sync | offset=%+.2fms | samples=%d | state=%s | rtt=%.1fms",
      offset_ms, samples, state_label, rtt_ns_.load() / 1e6);
  }

  // 计算平滑偏移：样本 < 10 用简单平均，≥10 用 10% trimmed mean
  double get_smoothed_offset_ms() const
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (offset_window_.empty()) return 0.0;

    if (offset_window_.size() < 10) {
      int64_t sum = 0;
      for (auto v : offset_window_) sum += v;
      return static_cast<double>(sum) / static_cast<double>(offset_window_.size()) / 1e6;
    }

    // Trimmed mean：去掉前 10% 和后 10% 的极值，再取平均
    std::vector<int64_t> sorted(offset_window_.begin(), offset_window_.end());
    std::sort(sorted.begin(), sorted.end());

    size_t trim = sorted.size() / 10;
    int64_t sum = 0;
    for (size_t i = trim; i < sorted.size() - trim; ++i) {
      sum += sorted[i];
    }
    return static_cast<double>(sum) / static_cast<double>(sorted.size() - 2 * trim) / 1e6;
  }

  // 取窗口中位数（用于 bootstrap 硬跳）
  int64_t get_median_offset_ns() const
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (offset_window_.empty()) return 0;

    std::vector<int64_t> sorted(offset_window_.begin(), offset_window_.end());
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
  }

  void publish_offset(double offset_ms)
  {
    std_msgs::msg::Float64 msg;
    msg.data = offset_ms;
    offset_pub_->publish(msg);
  }

  void publish_synced(bool val)
  {
    std_msgs::msg::Bool msg;
    msg.data = val;
    synced_pub_->publish(msg);
  }

  void publish_status(const char * fmt, ...)
  {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    std_msgs::msg::String msg;
    msg.data = std::string(buf);
    status_pub_->publish(msg);
  }

  // ROS 时间 → 纳秒转换
  int64_t rclcpp_time_to_ns(const builtin_interfaces::msg::Time & t) const
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
  }

  // 获取当前系统时钟 (CLOCK_REALTIME) 纳秒值
  int64_t get_host_time_ns() const
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + static_cast<int64_t>(ts.tv_nsec);
  }

  // 将 min_gps_year 转换为 Unix 时间戳（用于过滤未初始化 GPS）
  int64_t min_gps_year_to_epoch() const
  {
    struct tm tm = {};
    tm.tm_year = min_gps_year_ - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 0;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    time_t t = timegm(&tm);
    return static_cast<int64_t>(t);
  }

  // ---- 成员变量 ----
  State state_;
  bool synced_;
  int gps_lost_count_;
  std::optional<rclcpp::Time> last_gps_time_;

  int window_size_;
  double bootstrap_threshold_ms_;
  double gps_timeout_sec_;
  int min_gps_year_;
  double post_bootstrap_grace_sec_;
  double print_interval_sec_;
  int64_t rtt_max_ns_;

  std::deque<int64_t> offset_window_;                // 滑动窗口，存储最近 N 次 offset 测量值
  mutable std::mutex window_mutex_;                   // 保护 offset_window_

  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_sub_;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr tsync_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr synced_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::atomic<int64_t> rtt_ns_{0};                   // 最新 RTT（原子变量，多线程安全）
  std::optional<rclcpp::Time> bootstrap_time_;       // 上次 bootstrap 时间
  rclcpp::Time last_print_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
