// 飞行数据记录器（黑匣子）
// 监听 /mavros/state，解锁时自动录制，上锁时关闭
// 定时切片：每 rotation_interval_sec 秒关闭当前 bag 目录并创建新目录
// 磁盘保护：磁盘不足时按时间戳从最旧切片开始逐目录删除，跳过当前活跃切片
// 每个切片是独立的 rosbag2 目录，可单独用 ros2 bag play 回放
//
// 16GB 存储 + 30 分钟飞行 ≈ 30GB 数据的滚动录制：
//   t=0~5  创建切片1，闭包时变成独立目录
//   t=5~10 创建切片2
//   t=10~15 创建切片3（累计 ~15GB，剩余 ~1GB）
//   disk_check_cb 触发 → 删最旧切片1 → 剩 ~10GB
//   最终保留最近 10~15 分钟（2~3 个切片）

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mavros_msgs/msg/state.hpp>

namespace {

// 单个话题的录制配置
struct TopicConfig {
  std::string name;   // 话题名，如 /odin1/imu
  std::string type;   // 消息类型，如 sensor_msgs/msg/Imu
  std::string qos;    // QoS: "sensor_data"=best_effort, "reliable"=reliable
  int depth = 10;     // 队列深度，高频话题需要更大值防丢包
};

// 全局配置项，从 YAML 文件加载
struct Config {
  std::string storage_path = "/data/bags";   // bag 存储根目录
  int rotation_interval_sec = 300;            // 切片间隔 (s)，0=禁用切片
  int max_bagfile_duration_sec = 0;           // [已废弃] rosbag2 内部分片
  int max_bagfile_size_mb = 0;                // [已废弃] rosbag2 内部分片
  std::string cleanup_mode = "auto";           // 清理模式：auto=解锁时删旧，retain=保留
  bool arm_record_enabled = true;             // 是否武装触发录制
  std::string mavros_state_topic = "/mavros/state";  // 飞控状态话题
  std::vector<TopicConfig> topics;            // 录制话题清单

  int min_recording_free_space_mb = 2048;     // 录制中最低空闲 (MB)，不足触发删旧切片
  int disk_check_interval_sec = 30;           // 磁盘检查周期 (s)
};

// 格式化时间戳为 YYYYMMDDHHmmSS，作为 bag 目录名前缀
// 支持按字典序排序即时间序
std::string format_timestamp(const std::time_t t)
{
  std::tm tm{};
  if (!localtime_r(&t, &tm)) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(t));
    return std::string(buf);
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d%02d%02d%02d",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return std::string(buf);
}

// 查询磁盘剩余空间 (MB)，返回 -1 表示查询失败
int64_t disk_free_mb(const std::string & path)
{
  struct statvfs vfs;
  if (statvfs(path.c_str(), &vfs) != 0) return -1;
  return static_cast<int64_t>(vfs.f_bavail) * vfs.f_frsize / 1048576;
}

// 递归删除目录（含子目录和所有文件）
// 注意：使用 lstat 而非 stat，遇到符号链接只删除链接本身，绝不跟随递归
//       （防止 evil symlink 指向 / 等系统目录被递归清空）
void remove_dir(const std::string & path)
{
  DIR * d = opendir(path.c_str());
  if (!d) {
    fprintf(stderr, "[bag_logd] remove_dir: opendir(%s) failed: %s\n",
            path.c_str(), strerror(errno));
    return;
  }
  struct dirent * entry;
  while ((entry = readdir(d)) != nullptr) {
    if (entry->d_name[0] == '.') continue;
    std::string full = path + "/" + entry->d_name;
    struct stat st;
    if (lstat(full.c_str(), &st) != 0) continue;       // lstat 不跟随符号链接
    if (S_ISLNK(st.st_mode)) {
      // 符号链接：只删除链接本身，绝不递归进入其目标
      if (unlink(full.c_str()) != 0) {
        fprintf(stderr, "[bag_logd] remove_dir: unlink symlink(%s) failed: %s\n",
                full.c_str(), strerror(errno));
      }
    } else if (S_ISDIR(st.st_mode)) {
      remove_dir(full);
    } else {
      if (std::remove(full.c_str()) != 0) {
        fprintf(stderr, "[bag_logd] remove_dir: remove(%s) failed: %s\n",
                full.c_str(), strerror(errno));
      }
    }
  }
  closedir(d);
  if (rmdir(path.c_str()) != 0) {
    fprintf(stderr, "[bag_logd] remove_dir: rmdir(%s) failed: %s\n",
            path.c_str(), strerror(errno));
  }
}

// 校验 storage_path 安全性：防止递归删除误伤系统目录或被路径穿越攻击
// 规则：绝对路径、不含 ".." 分量、不在系统目录黑名单、至少 2 级路径分量
// 成功时把 path 规范化（去多余/尾部斜杠），失败时 reason 填充原因
bool validate_storage_path(std::string & path, std::string & reason)
{
  if (path.empty()) { reason = "storage_path is empty"; return false; }
  if (path[0] != '/') { reason = "storage_path must be absolute: " + path; return false; }

  // 拆分并规范化路径分量，拒绝 ".."
  std::vector<std::string> parts;
  std::string seg;
  for (size_t i = 0; i <= path.size(); ++i) {
    if (i == path.size() || path[i] == '/') {
      if (!seg.empty() && seg != ".") {
        if (seg == "..") { reason = "storage_path must not contain '..': " + path; return false; }
        parts.push_back(seg);
      }
      seg.clear();
    } else {
      seg += path[i];
    }
  }

  if (parts.empty()) { reason = "storage_path is root '/'"; return false; }
  if (parts.size() < 2) { reason = "storage_path too shallow (need at least /x/y): " + path; return false; }

  // 重建规范化路径
  std::string normalized;
  for (const auto & p : parts) normalized += "/" + p;

  // 系统目录黑名单（一级系统目录禁止作为存储根）
  static const std::vector<std::string> denylist = {
    "/", "/data", "/home", "/etc", "/usr", "/var", "/tmp", "/boot",
    "/root", "/opt", "/bin", "/sbin", "/lib", "/lib64", "/proc",
    "/sys", "/dev", "/run", "/media", "/mnt", "/srv"
  };
  for (const auto & bad : denylist) {
    if (normalized == bad) {
      reason = "storage_path refuses system directory: " + normalized;
      return false;
    }
  }

  path = normalized;
  return true;
}

// 从 YAML 文件加载配置
Config load_config(const std::string & config_file)
{
  Config cfg;
  try {
    YAML::Node root = YAML::LoadFile(config_file);
    if (root["storage_path"])    cfg.storage_path = root["storage_path"].as<std::string>();
    if (root["rotation_interval_sec"]) cfg.rotation_interval_sec = root["rotation_interval_sec"].as<int>();
    if (root["max_bagfile_duration_sec"]) cfg.max_bagfile_duration_sec = root["max_bagfile_duration_sec"].as<int>();
    if (root["max_bagfile_size_mb"])      cfg.max_bagfile_size_mb = root["max_bagfile_size_mb"].as<int>();
    if (root["cleanup_mode"])             cfg.cleanup_mode = root["cleanup_mode"].as<std::string>();

    if (root["min_recording_free_space_mb"])
      cfg.min_recording_free_space_mb = root["min_recording_free_space_mb"].as<int>();
    if (root["disk_check_interval_sec"])
      cfg.disk_check_interval_sec = root["disk_check_interval_sec"].as<int>();

    // 合法性校验
    {
      std::string reason;
      if (!validate_storage_path(cfg.storage_path, reason)) {
        fprintf(stderr, "[bag_logd] FATAL: %s\n", reason.c_str());
        throw std::runtime_error(reason);
      }
    }
    if (cfg.min_recording_free_space_mb < 0) {
      fprintf(stderr, "[bag_logd] WARN: min_recording_free_space_mb is negative, using 0\n");
      cfg.min_recording_free_space_mb = 0;
    }
    if (cfg.disk_check_interval_sec < 1) {
      fprintf(stderr, "[bag_logd] WARN: disk_check_interval_sec too small, using 5\n");
      cfg.disk_check_interval_sec = 5;
    }
    if (cfg.rotation_interval_sec < 0) {
      fprintf(stderr, "[bag_logd] WARN: rotation_interval_sec negative, disabling rotation\n");
      cfg.rotation_interval_sec = 0;
    }

    if (auto ar = root["arm_record"]) {
      if (ar["enabled"])            cfg.arm_record_enabled = ar["enabled"].as<bool>();
      if (ar["mavros_state_topic"]) cfg.mavros_state_topic = ar["mavros_state_topic"].as<std::string>();
    }
    if (root["topics"] && root["topics"].IsSequence()) {
      for (const auto & t : root["topics"]) {
        cfg.topics.push_back({
          t["name"].as<std::string>(),
          t["type"].as<std::string>(),
          t["qos"] ? t["qos"].as<std::string>() : "sensor_data",
          t["depth"] ? t["depth"].as<int>() : 10
        });
      }
    }
  } catch (const YAML::Exception & e) {
    fprintf(stderr, "[bag_logd] FATAL: YAML error: %s\n", e.what());
    throw;
  }
  return cfg;
}

}  // namespace

// BagRecorderNode: 飞行数据记录器核心节点
// 不发布任何话题，仅订阅录制
// 状态机：arm_state_cb (解锁/上锁) + rotation_timer (定时切片) + disk_check_cb (磁盘清理)
class BagRecorderNode : public rclcpp::Node
{
public:
  // 构造函数：初始化配置、订阅、定时器
  BagRecorderNode(const Config & config)
  : Node("bag_logd"),
    cfg_(config)
  {
    RCLCPP_INFO(get_logger(), "=== FLIGHT DATA RECORDER ===");
    RCLCPP_INFO(get_logger(), "  storage: %s", cfg_.storage_path.c_str());
    if (cfg_.rotation_interval_sec > 0) {
      RCLCPP_INFO(get_logger(), "  rotation: %ds (independent dirs, disk-low deletes oldest)",
        cfg_.rotation_interval_sec);
    } else {
      RCLCPP_INFO(get_logger(), "  rotation: disabled (single bag, no slice)");
    }
    RCLCPP_INFO(get_logger(), "  disk: min_recording=%dMB  check_interval=%ds",
      cfg_.min_recording_free_space_mb, cfg_.disk_check_interval_sec);
    RCLCPP_INFO(get_logger(), "  cleanup: mode=%s", cfg_.cleanup_mode.c_str());
    RCLCPP_INFO(get_logger(), "  arm_record: %s  topic: %s",
      cfg_.arm_record_enabled ? "ON" : "OFF", cfg_.mavros_state_topic.c_str());
    for (const auto & t : cfg_.topics) {
      RCLCPP_INFO(get_logger(), "  record: %-45s %s [%s]",
        t.name.c_str(), t.type.c_str(), t.qos.c_str());
    }

    create_subscriptions();

    if (cfg_.arm_record_enabled) {
      // 武装触发模式下：创建飞控状态订阅，等解锁后自动开始录制
      arm_state_sub_ = create_subscription<mavros_msgs::msg::State>(
        cfg_.mavros_state_topic, rclcpp::QoS(1).reliable(),
        std::bind(&BagRecorderNode::arm_state_cb, this, std::placeholders::_1));
    } else {
      // 非武装模式：立即开始录制
      ensure_storage_dir();
      cleanup_old_bags();
      try {
        {
          std::lock_guard<std::mutex> lock(writer_mutex_);
          open_new_bag();
          recording_ = true;
        }
        start_rotation_timer();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to open bag at startup: %s", e.what());
        {
          std::lock_guard<std::mutex> lock(writer_mutex_);
          writer_.reset();
          current_bag_path_.clear();
        }
        recording_ = false;
        schedule_retry();                             // 5s 周期重试恢复录制
      }
    }

    ensure_storage_dir();

    // 磁盘检查定时器：录制中每 N 秒检查磁盘剩余空间
    disk_check_timer_ = create_wall_timer(
      std::chrono::seconds(cfg_.disk_check_interval_sec),
      std::bind(&BagRecorderNode::disk_check_cb, this));
  }

  // 析构：取消定时器，关闭 writer
  ~BagRecorderNode() override
  {
    disk_check_timer_->cancel();
    if (rotation_timer_) rotation_timer_->cancel();
    if (retry_timer_) retry_timer_->cancel();
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (writer_) {
      try { writer_->close(); } catch (...) {}
      writer_.reset();
    }
  }

private:
  // 确保存储目录存在（递归创建父目录）
  void ensure_storage_dir()
  {
    struct stat st;
    if (stat(cfg_.storage_path.c_str(), &st) == 0) return;

    std::string path = cfg_.storage_path;
    std::string accum;
    for (size_t i = 0; i < path.size(); ++i) {
      accum += path[i];
      if (path[i] == '/' || i + 1 == path.size()) {
        if (accum.size() > 1 && accum.back() == '/') accum.pop_back();
        if (!accum.empty()) {
          if (mkdir(accum.c_str(), 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "[bag_logd] mkdir(%s) failed: %s\n",
                    accum.c_str(), strerror(errno));
          }
        }
      }
    }
  }

  // 创建录制订阅：使用 GenericSubscription，无需编译时确定类型
  void create_subscriptions()
  {
    subscriptions_.clear();
    for (const auto & t : cfg_.topics) {
      rclcpp::QoS qos(rclcpp::KeepLast(t.depth));
      if (t.qos == "sensor_data") qos.best_effort();   // 传感器数据：允许丢包，省带宽
      else qos.reliable();                               // 关键数据：可靠传输

      try {
        subscriptions_.push_back(create_generic_subscription(
          t.name, t.type, qos,
          [this, name = t.name, type = t.type]
          (std::shared_ptr<rclcpp::SerializedMessage> msg) {
            std::scoped_lock<std::mutex> lock(writer_mutex_);
            if (!writer_) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
                "writer not ready, dropping messages (%s)", name.c_str());
              return;
            }
            try {
              writer_->write(msg, name, type, this->now());
            } catch (const std::exception & e) {
              RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "write failed on %s: %s", name.c_str(), e.what());
            }
          }));
      } catch (const std::exception & e) {
        // 消息类型未安装时跳过，不阻塞其他话题录制
        RCLCPP_WARN(get_logger(), "Skipping topic %s (%s): %s",
          t.name.c_str(), t.type.c_str(), e.what());
      }
    }
  }

  // 创建新的 bag 目录并开始录制
  // 目录名格式: /data/bags/YYYYMMDDHHmmSS（时间戳）
  void open_new_bag()
  {
    current_bag_start_ = std::time(nullptr);

    // 时钟回拨检测：如果系统时间看起来比存储目录的修改时间还早
    // 说明 NTP 尚未同步，用目录 mtime 作为后备时间戳
    struct stat st;
    if (stat(cfg_.storage_path.c_str(), &st) == 0 && st.st_mtime > current_bag_start_) {
      RCLCPP_WARN(get_logger(),
        "System clock appears stale (%lld < %lld), using storage dir mtime as fallback. "
        "Check NTP sync.",
        static_cast<long long>(current_bag_start_), static_cast<long long>(st.st_mtime));
      current_bag_start_ = st.st_mtime;
    }

    // 处理重名冲突：同名目录已存在时追加 _1, _2 ...
    const std::string base =
      cfg_.storage_path + "/" + format_timestamp(current_bag_start_);
    current_bag_path_ = base;
    for (int i = 1; stat(current_bag_path_.c_str(), &st) == 0; ++i) {
      current_bag_path_ = base + "_" + std::to_string(i);
    }

    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions opts;
    opts.uri = current_bag_path_;
    opts.storage_id = "sqlite3";
    opts.max_cache_size = 64 * 1024 * 1024;  // 64MB 写缓存，吸收 IMU@400Hz + 点云的 I/O 突发
    // 注意：不再传入 max_bagfile_duration/max_bagfile_size
    // 内部分片由 rotation_timer 替代，每个切片是独立目录
    writer_->open(opts);

    RCLCPP_INFO(get_logger(), "Bag opened: %s", current_bag_path_.c_str());
  }

  // 关闭当前 bag（flush 数据 + 关闭 writer）
  void close_writer()
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (!writer_) return;
    try {
      writer_->close();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "close_writer failed: %s", e.what());
    }
    writer_.reset();
    current_bag_path_.clear();
    RCLCPP_INFO(get_logger(), "Bag closed");
  }

  // 启动切片旋转定时器（仅在解锁 + 配置 rotation_interval_sec > 0 时）
  void start_rotation_timer()
  {
    if (cfg_.rotation_interval_sec <= 0) return;  // 切片功能禁用
    if (rotation_timer_) return;                    // 已启动，避免重复
    rotation_timer_ = create_wall_timer(
      std::chrono::seconds(cfg_.rotation_interval_sec),
      std::bind(&BagRecorderNode::rotate_bag, this));
    RCLCPP_INFO(get_logger(), "Rotation timer started: interval=%ds",
      cfg_.rotation_interval_sec);
  }

  // 停止切片旋转定时器（上锁时）
  void stop_rotation_timer()
  {
    if (!rotation_timer_) return;
    rotation_timer_->cancel();
    rotation_timer_.reset();
    RCLCPP_INFO(get_logger(), "Rotation timer stopped");
  }

  // 切片旋转：关闭当前 bag 目录 → 创建新目录
  // 每 rotation_interval_sec 秒触发一次
  void rotate_bag()
  {
    if (!recording_) return;                        // 未录制中，跳过
    if (!writer_ || current_bag_path_.empty()) return;

    RCLCPP_INFO(get_logger(), "Rotating bag — closing current slice");
    close_writer();

    try {
      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (cfg_.arm_record_enabled && !armed_) {        // 旋转期间上锁，终止
        RCLCPP_WARN(get_logger(), "Disarmed during rotation, recording aborted");
        return;
      }
      open_new_bag();
      recording_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to open bag after rotation — data gap for this slice: %s", e.what());
      writer_.reset();
      current_bag_path_.clear();
      recording_ = false;
      schedule_retry();                               // 5s 周期重试恢复录制
    }
  }

  // 安排录制重试：open_new_bag 失败后周期性重试（5s 间隔）
  // 由 retry_open_bag 判断是否真正重开（磁盘不足/未武装时自动顺延）
  void schedule_retry()
  {
    if (retry_timer_) return;                          // 已在重试中
    retry_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&BagRecorderNode::retry_open_bag, this));
    RCLCPP_WARN(get_logger(), "Recording retry scheduled (5s interval)");
  }

  // 重试打开新 bag：成功则恢复录制，失败则继续等待下次重试
  void retry_open_bag()
  {
    if (recording_) {                                  // 已恢复录制
      if (retry_timer_) { retry_timer_->cancel(); retry_timer_.reset(); }
      return;
    }
    // 武装触发模式下未解锁，不应录制（等下一次 ARM 触发）
    if (cfg_.arm_record_enabled && !armed_) {
      if (retry_timer_) { retry_timer_->cancel(); retry_timer_.reset(); }
      return;
    }
    // 磁盘空间不足时暂不重试（H5 停录后等空间恢复）
    int64_t free_mb = disk_free_mb(cfg_.storage_path);
    if (free_mb >= 0 && free_mb < cfg_.min_recording_free_space_mb) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "Retry deferred: disk still low (%ld MB < %d MB)",
        free_mb, cfg_.min_recording_free_space_mb);
      return;
    }
    try {
      std::lock_guard<std::mutex> lock(writer_mutex_);
      open_new_bag();
      recording_ = true;
      RCLCPP_INFO(get_logger(), "Recording recovered after retry: %s",
        current_bag_path_.c_str());
      start_rotation_timer();
      if (retry_timer_) { retry_timer_->cancel(); retry_timer_.reset(); }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Retry open_new_bag failed: %s", e.what());
      std::lock_guard<std::mutex> lock(writer_mutex_);
      writer_.reset();
      current_bag_path_.clear();
      // 保留 retry_timer_ 继续重试
    }
  }

  // 飞控状态回调：处理解锁/上锁事件
  // 使用 compare_exchange_strong 防止双触发竞态
  void arm_state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    if (msg->armed) {
      // ---- 解锁 ----
      // compare_exchange：原子检查并置位，防止两条武装消息并发生成两个录制
      bool expected = false;
      if (!armed_.compare_exchange_strong(expected, true)) return;

      RCLCPP_INFO(get_logger(), "ARMED — starting recording");
      ensure_storage_dir();

      close_writer();                                // 关闭可能残留的旧 writer
      cleanup_old_bags();                            // 删除所有历史 bag 为本次飞行腾空间
      try {
        std::lock_guard<std::mutex> lock(writer_mutex_);
        if (!armed_) {                               // 清理期间上锁，终止
          RCLCPP_WARN(get_logger(), "Disarmed during arm processing, recording aborted");
          return;
        }
        open_new_bag();
        recording_ = true;
        start_rotation_timer();                      // 启动切片旋转
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to open bag on ARM: %s", e.what());
        writer_.reset();
        current_bag_path_.clear();
        recording_ = false;
        armed_ = false;                              // 失败回退
        schedule_retry();                           // 武装模式下由下一条 state 消息触发恢复
      }
    } else {
      // ---- 上锁 ----
      bool expected = true;
      if (!armed_.compare_exchange_strong(expected, false)) return;

      RCLCPP_INFO(get_logger(), "DISARMED — closing bag");
      recording_ = false;
      if (cfg_.arm_record_enabled) {
        stop_rotation_timer();
      }
      close_writer();                                // flush + 关闭当前切片
    }
  }

  // 磁盘检查回调：录制中每 disk_check_interval_sec 秒执行
  // 磁盘不足时从最旧切片开始删除，直到剩余空间 ≥ 阈值
  // 永远不删除当前活跃切片
  void disk_check_cb()
  {
    if (!recording_) return;                         // 未录制中，跳过

    int64_t free_mb = disk_free_mb(cfg_.storage_path);
    if (free_mb < 0) {
      RCLCPP_ERROR(get_logger(), "  Cannot query disk space, unable to monitor");
      return;
    }

    if (free_mb >= cfg_.min_recording_free_space_mb) return;  // 磁盘充足

    RCLCPP_WARN(get_logger(),
      "  DISK LOW (%ld MB < %d MB) — cleaning oldest bag dirs",
      free_mb, cfg_.min_recording_free_space_mb);

    // 扫描存储目录，收集所有子目录名
    std::vector<std::string> dirs;
    DIR * d = opendir(cfg_.storage_path.c_str());
    if (d) {
      struct dirent * entry;
      while ((entry = readdir(d)) != nullptr) {
        if (entry->d_name[0] == '.') continue;       // 跳过 . 和 ..
        if (entry->d_type != DT_DIR && entry->d_type != DT_UNKNOWN) continue;
        std::string full = cfg_.storage_path + "/" + entry->d_name;
        if (full == current_bag_path_) continue;      // 保护活跃切片
        struct stat st;
        if (lstat(full.c_str(), &st) != 0) continue;  // lstat 不跟随符号链接
        if (S_ISLNK(st.st_mode)) {
          // 存储目录里不应有符号链接，删除链接本身并告警（不跟随，不加入待删列表）
          RCLCPP_WARN(get_logger(), "  Removing stray symlink in storage: %s", entry->d_name);
          unlink(full.c_str());
          continue;
        }
        if (!S_ISDIR(st.st_mode)) continue;
        dirs.push_back(entry->d_name);
      }
      closedir(d);
    } else {
      RCLCPP_ERROR(get_logger(),
        "  disk_check: opendir(%s) failed: %s",
        cfg_.storage_path.c_str(), strerror(errno));
    }

    // 按时间戳字典序排序（最旧的在前）
    std::sort(dirs.begin(), dirs.end());

    // 从最旧开始删除，直到剩余空间 ≥ 阈值
    for (const auto & name : dirs) {
      free_mb = disk_free_mb(cfg_.storage_path);
      if (free_mb < 0 || free_mb >= cfg_.min_recording_free_space_mb) break;

      std::string full = cfg_.storage_path + "/" + name;
      if (full == current_bag_path_) continue;        // 双重保护
      RCLCPP_INFO(get_logger(), "  Remove old slice: %s", name.c_str());
      remove_dir(full);
    }

    free_mb = disk_free_mb(cfg_.storage_path);
    if (free_mb >= 0) {
      RCLCPP_INFO(get_logger(), "  After cleanup: free=%ld MB", free_mb);
    }

    // 清理后仍不达标：紧急停止录制，避免写满磁盘拖垮系统其它组件
    // 空间恢复后由 retry_timer（retry_open_bag）自动恢复录制
    if (free_mb >= 0 && free_mb < cfg_.min_recording_free_space_mb) {
      RCLCPP_ERROR(get_logger(),
        "DISK FULL after cleanup (%ld MB < %d MB) — stopping recording to protect system",
        free_mb, cfg_.min_recording_free_space_mb);
      recording_ = false;
      stop_rotation_timer();
      close_writer();
      schedule_retry();
    }
  }

  // 解锁时清理所有旧的 bag 目录（为本次飞行腾出空间）
  // 跳过当前正在录制的目录
  void cleanup_old_bags()
  {
    if (cfg_.cleanup_mode != "auto") return;          // 清理模式被禁用
    ensure_storage_dir();

    DIR * d = opendir(cfg_.storage_path.c_str());
    if (!d) {
      RCLCPP_ERROR(get_logger(), "cleanup_old_bags: opendir(%s) failed: %s",
        cfg_.storage_path.c_str(), strerror(errno));
      return;
    }
    struct dirent * entry;
    while ((entry = readdir(d)) != nullptr) {
      if (entry->d_name[0] == '.') continue;
      std::string full = cfg_.storage_path + "/" + entry->d_name;
      if (full == current_bag_path_) continue;        // 保护活跃切片
      struct stat st;
      if (lstat(full.c_str(), &st) != 0) continue;    // lstat 不跟随符号链接
      if (S_ISLNK(st.st_mode)) {
        // 存储目录里不应有符号链接，删除链接本身并告警（绝不跟随）
        RCLCPP_WARN(get_logger(), "  Removing stray symlink in storage: %s", entry->d_name);
        unlink(full.c_str());
        continue;
      }
      if (!S_ISDIR(st.st_mode)) continue;             // 非目录（普通文件等）跳过
      RCLCPP_INFO(get_logger(), "  Remove old bag: %s", entry->d_name);
      remove_dir(full);
    }
    closedir(d);
  }

  // ---- 成员变量 ----
  Config cfg_;
  std::time_t current_bag_start_ = 0;        // 当前 bag 创建时间戳
  std::string current_bag_path_;             // 当前活跃切片路径，用于保护不删除
  std::atomic<bool> armed_{false};           // 飞控解锁状态
  std::atomic<bool> recording_{false};       // 是否正在录制（writer_ 已打开）

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::mutex writer_mutex_;                  // 保护 writer_ 指针的读写安全

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr arm_state_sub_;
  rclcpp::TimerBase::SharedPtr disk_check_timer_;   // 磁盘检查定时器
  rclcpp::TimerBase::SharedPtr rotation_timer_;     // 切片旋转定时器
  rclcpp::TimerBase::SharedPtr retry_timer_;        // 录制失败重试定时器
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 自动定位配置文件路径
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("bag_logd")
                  + "/config/bag_logd.yaml";
  } catch (...) {
    config_path = "config/bag_logd.yaml";
  }
  std::string storage_override;

  // 命令行参数解析
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--storage" && i + 1 < argc) {
      storage_override = argv[++i];
    }
  }

  // 加载配置
  Config cfg;
  try {
    cfg = load_config(config_path);
  } catch (const YAML::Exception & e) {
    fprintf(stderr, "[bag_logd] FATAL: Cannot load config: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (cfg.topics.empty()) {
    fprintf(stderr, "[bag_logd] FATAL: no topics configured in %s\n",
            config_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (!storage_override.empty()) {
    cfg.storage_path = storage_override;
    std::string reason;
    if (!validate_storage_path(cfg.storage_path, reason)) {
      fprintf(stderr, "[bag_logd] FATAL: %s\n", reason.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("bag_logd"), "Config loaded from: %s",
    config_path.c_str());

  auto node = std::make_shared<BagRecorderNode>(cfg);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
