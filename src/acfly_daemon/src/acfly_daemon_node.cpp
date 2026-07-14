// acfly_daemon — 进程看门狗/主管节点
// 按 YAML 配置文件管理所有子进程的启动、崩溃重启、有序关闭
// 按依赖顺序启动：mavros → odin1/timesync → acfly_odometry → bag_logd
// 崩溃重启采用指数退避：1s → 2s → 4s → ... → 60s 上限
// 关闭时逆依赖序 SIGINT → 10s 超时 → SIGKILL
//
// 发布 /acfly_daemon/status 话题（transient_local QoS）
// 提供服务：~/start ~/stop ~/restart ~/shutdown ~/get_status

#include <algorithm>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

#include "acfly_daemon/msg/process_status.hpp"
#include "acfly_daemon/srv/get_process_status.hpp"
#include "acfly_daemon/srv/manage_process.hpp"

namespace ad = acfly_daemon;

// 全局信号标记（volatile sig_atomic_t 保证信号安全）
static volatile sig_atomic_t g_signal_received = 0;

// 信号处理器：只设置标记，不在处理函数内做复杂操作
static void signal_handler(int) { g_signal_received = 1; }

// 进程状态枚举
enum class ProcessState {
  PENDING,     // 等待依赖就绪
  STARTING,    // 已 fork，等待变为 RUNNING
  RUNNING,     // 正常运行中
  RESTARTING,  // 崩溃后等待退避倒计时
  STOPPING,    // 已发 SIGINT，等待进程退出
  STOPPED      // 已退出 / 已停止
};

// 状态值转可读字符串
std::string state_to_str(ProcessState s) {
  switch (s) {
    case ProcessState::PENDING:    return "PENDING";
    case ProcessState::STARTING:   return "STARTING";
    case ProcessState::RUNNING:    return "RUNNING";
    case ProcessState::RESTARTING: return "RESTARTING";
    case ProcessState::STOPPING:   return "STOPPING";
    case ProcessState::STOPPED:    return "STOPPED";
  }
  return "UNKNOWN";
}

// 被管理进程的核心数据结构
struct ManagedProcess {
  std::string name;                          // 进程名（唯一标识）
  std::string command;                       // 可执行文件路径
  std::vector<std::string> args;             // 命令行参数
  bool auto_restart = true;                  // 是否自动重启
  double shutdown_timeout_sec = 10.0;        // SIGINT 后等多久才 SIGKILL
  std::vector<std::string> dependencies;     // 依赖进程名列表（全部 RUNNING 才启动）
  std::vector<std::string> wait_for_topics;  // 等待出现在 ROS graph 的话题

  pid_t pid = -1;                            // 子进程 PID，-1 表示未运行
  ProcessState state = ProcessState::PENDING;
  int restart_count = 0;                     // 累计重启次数
  rclcpp::Time start_time;                   // 最近一次启动时间
  rclcpp::Time crash_time;                   // 最近一次崩溃时间
  int exit_code = 0;                         // 退出码
  std::string exit_reason;                   // 退出原因描述
  double current_backoff_sec = 1.0;          // 当前退避秒数（指数增长）
  rclcpp::Time stop_request_time;            // 收到停止请求的时间
  bool stopping = false;                      // 是否正在停止中
};

// AcflyDaemon: 核心看门狗节点
class AcflyDaemon : public rclcpp::Node {
public:
  explicit AcflyDaemon()
  : Node("acfly_daemon")
  {
    // 从 ROS 参数服务器获取配置文件路径，默认 /tmp/acfly_processes.yaml
    std::string config_path = declare_parameter("config_path",
      "/tmp/acfly_processes.yaml");

    // 加载 YAML 配置
    load_config(config_path);

    // ---- 发布者 + 服务 ----
    // 状态发布：transient_local QoS 保证新订阅者收到最新值
    status_pub_ = create_publisher<ad::msg::ProcessStatus>(
      "~/status", rclcpp::QoS(10).transient_local());

    start_srv_ = create_service<ad::srv::ManageProcess>(
      "~/start",
      [this](const std::shared_ptr<ad::srv::ManageProcess::Request> req,
             std::shared_ptr<ad::srv::ManageProcess::Response> resp) {
        handle_start(req, resp);
      });

    stop_srv_ = create_service<ad::srv::ManageProcess>(
      "~/stop",
      [this](const std::shared_ptr<ad::srv::ManageProcess::Request> req,
             std::shared_ptr<ad::srv::ManageProcess::Response> resp) {
        handle_stop(req, resp);
      });

    restart_srv_ = create_service<ad::srv::ManageProcess>(
      "~/restart",
      [this](const std::shared_ptr<ad::srv::ManageProcess::Request> req,
             std::shared_ptr<ad::srv::ManageProcess::Response> resp) {
        handle_restart(req, resp);
      });

    // shutdown 服务：触发全局有序关闭
    shutdown_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/shutdown",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        handle_shutdown(req, resp);
      });

    get_status_srv_ = create_service<ad::srv::GetProcessStatus>(
      "~/get_status",
      [this](const std::shared_ptr<ad::srv::GetProcessStatus::Request> req,
             std::shared_ptr<ad::srv::GetProcessStatus::Response> resp) {
        handle_get_status(req, resp);
      });

    health_check_hz_ = declare_parameter("health_check_hz", 2.0);

    start_time_ = now();

    // 主健康检查定时器：2Hz 频率驱动状态机
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / health_check_hz_)),
      [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(), "Acfly daemon initialized with %zu processes",
      processes_.size());

    for (auto & mp : processes_) {
      RCLCPP_INFO(get_logger(), "  Registered: %s [%s] deps=[%s] topics=[%s]",
        mp.name.c_str(),
        mp.auto_restart ? "auto-restart" : "manual",
        join(mp.dependencies).c_str(),
        join(mp.wait_for_topics).c_str());
    }
  }

  // 析构：执行全局有序关闭
  ~AcflyDaemon()
  {
    shutdown_all();
  }

private:
  // 从 YAML 文件加载进程配置
  void load_config(const std::string & path)
  {
    YAML::Node config = YAML::LoadFile(path);

    // 全局 supervisor 配置
    auto sup = config["supervisor"];
    if (sup) {
      log_dir_ = sup["log_dir"].as<std::string>("/tmp/acfly_logs");
      graceful_shutdown_timeout_ = sup["graceful_shutdown_timeout_sec"].as<double>(10.0);
      initial_backoff_sec_ = sup["initial_backoff_sec"].as<double>(1.0);
      max_backoff_sec_ = sup["max_backoff_sec"].as<double>(60.0);
    }

    // 确保日志目录存在
    mkdir(log_dir_.c_str(), 0755);

    auto procs = config["processes"];
    if (!procs || !procs.IsSequence()) {
      throw std::runtime_error("No 'processes' sequence found in config");
    }

    // 逐进程解析
    for (const auto & node : procs) {
      ManagedProcess mp;
      mp.name = node["name"].as<std::string>();
      mp.command = node["command"].as<std::string>();
      mp.auto_restart = node["auto_restart"].as<bool>(true);
      mp.shutdown_timeout_sec = node["shutdown_timeout_sec"].as<double>(10.0);
      mp.current_backoff_sec = initial_backoff_sec_;

      auto a = node["args"];
      if (a && a.IsSequence()) {
        for (const auto & arg : a) {
          mp.args.push_back(arg.as<std::string>());
        }
      }

      auto d = node["dependencies"];
      if (d && d.IsSequence()) {
        for (const auto & dep : d) {
          mp.dependencies.push_back(dep.as<std::string>());
        }
      }

      auto t = node["wait_for_topics"];
      if (t && t.IsSequence()) {
        for (const auto & topic : t) {
          mp.wait_for_topics.push_back(topic.as<std::string>());
        }
      }

      processes_.push_back(std::move(mp));
    }
  }

  // 按名称查找被管进程
  ManagedProcess * find_process(const std::string & name)
  {
    for (auto & mp : processes_) {
      if (mp.name == name) return &mp;
    }
    return nullptr;
  }

  // 检查所有依赖进程是否都已 RUNNING
  bool dependencies_running(const ManagedProcess & mp)
  {
    for (const auto & dep : mp.dependencies) {
      auto * dp = find_process(dep);
      if (!dp || dp->state != ProcessState::RUNNING) {
        return false;
      }
    }
    return true;
  }

  // 检查等待的话题是否已在 ROS graph 中出现
  bool topics_ready(const ManagedProcess & mp)
  {
    if (mp.wait_for_topics.empty()) return true;
    auto topic_map = get_topic_names_and_types();
    for (const auto & required : mp.wait_for_topics) {
      if (topic_map.find(required) == topic_map.end()) {
        return false;
      }
    }
    return true;
  }

  // fork + exec 启动子进程
  void start_process(ManagedProcess & mp)
  {
    pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(get_logger(), "fork failed for %s: %s",
        mp.name.c_str(), strerror(errno));
      mp.state = ProcessState::RESTARTING;           // fork 失败视为崩溃，等待退避
      mp.crash_time = now();
      mp.exit_reason = "fork_failed:" + std::string(strerror(errno));
      return;
    }

    if (pid == 0) {
      // ---- 子进程 ----
      // 重定向 stdout/stderr 到日志文件
      std::string log_path = log_dir_ + "/" + mp.name + ".log";
      int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
      if (fd >= 0) {
        dup2(fd, STDOUT_FILENO);                     // stdout → 日志文件
        dup2(fd, STDERR_FILENO);                     // stderr → 日志文件
        close(fd);
      }

      // 构建 argv 数组（execvp 要求以 nullptr 结尾）
      std::vector<char *> argv;
      std::string cmd = mp.command;
      argv.push_back(const_cast<char *>(cmd.c_str()));
      for (auto & arg : mp.args) {
        argv.push_back(const_cast<char *>(arg.c_str()));
      }
      argv.push_back(nullptr);

      execvp(mp.command.c_str(), argv.data());
      // 如果 execvp 返回，说明执行失败
      _exit(127);
    }

    // ---- 父进程 ----
    mp.pid = pid;
    mp.state = ProcessState::STARTING;
    mp.start_time = now();
    mp.exit_code = 0;
    mp.exit_reason.clear();

    RCLCPP_INFO(get_logger(), "Started %s (pid=%d)", mp.name.c_str(), pid);
  }

  // 请求停止进程：发送 SIGINT，设置超时计时
  void request_stop(ManagedProcess & mp)
  {
    if (mp.pid <= 0) return;

    RCLCPP_INFO(get_logger(), "Requesting stop of %s (pid=%d, SIGINT)",
      mp.name.c_str(), mp.pid);

    mp.stopping = true;
    mp.stop_request_time = now();
    mp.state = ProcessState::STOPPING;
    kill(mp.pid, SIGINT);                           // 优雅退出信号
  }

  // 强制终止：发送 SIGKILL（在 SIGINT 超时后调用）
  void force_kill(ManagedProcess & mp)
  {
    if (mp.pid <= 0) return;

    RCLCPP_WARN(get_logger(), "Force killing %s (pid=%d, SIGKILL)",
      mp.name.c_str(), mp.pid);

    kill(mp.pid, SIGKILL);
    mp.stopping = false;
    mp.pid = -1;
    mp.state = ProcessState::STOPPED;
  }

  // 回收子进程（waitpid WNOHANG，非阻塞）
  void reap_children()
  {
    int status;
    pid_t w;
    while ((w = waitpid(-1, &status, WNOHANG)) > 0) {
      for (auto & mp : processes_) {
        if (mp.pid == w) {
          mp.pid = -1;

          if (WIFEXITED(status)) {
            mp.exit_code = WEXITSTATUS(status);
            mp.exit_reason = "exit:" + std::to_string(mp.exit_code);
          } else if (WIFSIGNALED(status)) {
            int sig = WTERMSIG(status);
            mp.exit_code = -sig;
            mp.exit_reason = "signal:" + std::to_string(sig);
          }

          if (mp.stopping) {
            // 有计划的停止 → STOPPED
            mp.stopping = false;
            mp.state = ProcessState::STOPPED;
            RCLCPP_INFO(get_logger(), "%s stopped (%s)",
              mp.name.c_str(), mp.exit_reason.c_str());
          } else {
            // 非预期崩溃 → RESTARTING（等退避倒计时）
            mp.state = ProcessState::RESTARTING;
            mp.crash_time = now();
            RCLCPP_WARN(get_logger(), "%s exited unexpectedly (%s), restart_count=%d",
              mp.name.c_str(), mp.exit_reason.c_str(), mp.restart_count);
          }
          break;
        }
      }
    }
  }

  // 检查停止超时：SIGINT 后超时未退出则 SIGKILL
  void check_stop_timeouts()
  {
    auto now_t = now();
    for (auto & mp : processes_) {
      if (mp.stopping && mp.pid > 0) {
        double elapsed = (now_t - mp.stop_request_time).seconds();
        if (elapsed > mp.shutdown_timeout_sec) {
          force_kill(mp);
        }
      }
    }
  }

  // 处理崩溃重启：退避时间到则重新启动
  void handle_restarts()
  {
    auto now_t = now();
    for (auto & mp : processes_) {
      if (mp.state != ProcessState::RESTARTING) continue;
      if (!mp.auto_restart) continue;

      double elapsed = (now_t - mp.crash_time).seconds();
      if (elapsed >= mp.current_backoff_sec) {      // 退避时间到
        mp.restart_count++;
        RCLCPP_INFO(get_logger(), "Restarting %s (attempt %d, backoff %.1fs)",
          mp.name.c_str(), mp.restart_count, mp.current_backoff_sec);

        start_process(mp);

        // 指数退避：1s → 2s → 4s → 8s → ... → max_backoff_sec_
        mp.current_backoff_sec = std::min(
          mp.current_backoff_sec * 2.0,
          max_backoff_sec_);
      }
    }
  }

  // 检查 STARTING 进程：如果子进程仍在运行，标记为 RUNNING
  void check_started()
  {
    for (auto & mp : processes_) {
      if (mp.state != ProcessState::STARTING) continue;
      if (mp.pid <= 0) continue;

      // kill(pid, 0) 不发送信号，只检查进程是否存在
      if (kill(mp.pid, 0) == 0) {
        mp.state = ProcessState::RUNNING;
        RCLCPP_INFO(get_logger(), "%s is now RUNNING (pid=%d)",
          mp.name.c_str(), mp.pid);
      } else {
        RCLCPP_WARN(get_logger(), "%s pid=%d already gone, will reap",
          mp.name.c_str(), mp.pid);
      }
    }
  }

  // 启动就绪的 PENDING 进程：依赖满足 + 话题就绪
  void handle_pending()
  {
    for (auto & mp : processes_) {
      if (mp.state != ProcessState::PENDING) continue;
      if (!dependencies_running(mp)) continue;       // 依赖未就绪
      if (!topics_ready(mp)) continue;               // 话题未出现

      RCLCPP_INFO(get_logger(), "Starting %s (dependencies met)", mp.name.c_str());
      start_process(mp);
    }
  }

  // 主定时器回调：2Hz 频率，按固定顺序执行各步骤
  void on_timer()
  {
    if (g_signal_received) {
      RCLCPP_INFO(get_logger(), "Signal received, initiating shutdown");
      shutdown_all();
      rclcpp::shutdown();
      return;
    }

    // 执行顺序：
    // 1. 回收僵尸进程
    // 2. 检查停止超时
    // 3. 确认已启动进程
    // 4. 处理崩溃重启
    // 5. 启动就绪进程
    // 6. 发布状态
    reap_children();
    check_stop_timeouts();
    check_started();
    handle_restarts();
    handle_pending();
    publish_status();
  }

  // 发布所有进程状态（包含 supervisor 自身）
  void publish_status()
  {
    // Supervisor 自身状态
    ad::msg::ProcessStatus msg;
    msg.name = "__supervisor__";
    msg.state = supervisor_state_str();
    msg.pid = getpid();
    msg.restart_count = 0;
    msg.uptime_sec = (now() - start_time_).seconds();
    msg.exit_code = 0;
    status_pub_->publish(msg);

    // 各子进程状态
    for (auto & mp : processes_) {
      ad::msg::ProcessStatus pmsg;
      pmsg.name = mp.name;
      pmsg.state = state_to_str(mp.state);
      pmsg.pid = mp.pid;
      pmsg.restart_count = mp.restart_count;
      if (mp.state == ProcessState::RUNNING) {
        pmsg.uptime_sec = (now() - mp.start_time).seconds();
      } else {
        pmsg.uptime_sec = 0.0;
      }
      pmsg.exit_code = mp.exit_code;
      pmsg.exit_reason = mp.exit_reason;
      status_pub_->publish(pmsg);
    }
  }

  // supervisor 全局状态：根据子进程汇总
  std::string supervisor_state_str()
  {
    bool all_running = true;
    bool any_pending = false;
    bool any_stopping = false;
    for (auto & mp : processes_) {
      if (mp.state == ProcessState::PENDING || mp.state == ProcessState::STARTING) any_pending = true;
      if (mp.state == ProcessState::STOPPING) any_stopping = true;
      if (mp.state != ProcessState::RUNNING && mp.state != ProcessState::STOPPED) all_running = false;
    }
    if (shutting_down_ || any_stopping) return "STOPPING";
    if (any_pending && !all_running) return "STARTING";
    if (all_running) return "RUNNING";
    return "DEGRADED";
  }

  // 全局有序关闭：逆依赖序 SIGINT → 等待 → SIGKILL
  void shutdown_all()
  {
    if (shutting_down_) return;                      // 防止重复调用
    shutting_down_ = true;

    RCLCPP_INFO(get_logger(), "Shutting down all processes");

    // 逆序发送 SIGINT（bag_logd 最后启动的最先关闭）
    for (auto it = processes_.rbegin(); it != processes_.rend(); ++it) {
      auto & mp = *it;
      if (mp.state == ProcessState::STARTING ||
          mp.state == ProcessState::RUNNING ||
          mp.state == ProcessState::RESTARTING) {
        request_stop(mp);
      }
    }

    // 等待所有进程退出（最长 graceful_shutdown_timeout_ 秒）
    auto deadline = now() + rclcpp::Duration::from_seconds(graceful_shutdown_timeout_);
    while (now() < deadline) {
      reap_children();
      check_stop_timeouts();
      bool all_stopped = true;
      for (auto & mp : processes_) {
        if (mp.state != ProcessState::STOPPED && mp.state != ProcessState::PENDING) {
          all_stopped = false;
          break;
        }
      }
      if (all_stopped) break;
      usleep(100000);                                // 100ms 粒度轮询
    }

    // 超时残留的进程强制 SIGKILL
    for (auto & mp : processes_) {
      if (mp.state != ProcessState::STOPPED && mp.state != ProcessState::PENDING) {
        force_kill(mp);
      }
    }

    RCLCPP_INFO(get_logger(), "All processes shut down");
  }

  // handle_start 服务回调：手动启动指定进程
  void handle_start(const std::shared_ptr<ad::srv::ManageProcess::Request> req,
    std::shared_ptr<ad::srv::ManageProcess::Response> resp)
  {
    if (req->name.empty()) {
      resp->success = false;
      resp->message = "Process name required";
      return;
    }
    auto * mp = find_process(req->name);
    if (!mp) {
      resp->success = false;
      resp->message = "Unknown process: " + req->name;
      return;
    }
    if (mp->state == ProcessState::RUNNING || mp->state == ProcessState::STARTING) {
      resp->success = false;
      resp->message = req->name + " is already running";
      return;
    }
    mp->state = ProcessState::PENDING;
    mp->current_backoff_sec = initial_backoff_sec_;  // 重置退避
    start_process(*mp);                               // 直接启动（不等定时器）
    resp->success = mp->state == ProcessState::STARTING;
    resp->message = resp->success ? req->name + " starting" : "Failed to start " + req->name;
  }

  // handle_stop 服务回调：手动停止指定进程（name 为空则停止全部）
  void handle_stop(const std::shared_ptr<ad::srv::ManageProcess::Request> req,
    std::shared_ptr<ad::srv::ManageProcess::Response> resp)
  {
    if (req->name.empty()) {
      shutdown_all();
      resp->success = true;
      resp->message = "All processes shutting down";
      return;
    }
    auto * mp = find_process(req->name);
    if (!mp) {
      resp->success = false;
      resp->message = "Unknown process: " + req->name;
      return;
    }
    if (mp->state != ProcessState::RUNNING && mp->state != ProcessState::STARTING) {
      resp->success = false;
      resp->message = req->name + " is not running";
      return;
    }
    request_stop(*mp);
    resp->success = true;
    resp->message = req->name + " stopping";
  }

  // handle_restart 服务回调：手动重启指定进程
  void handle_restart(const std::shared_ptr<ad::srv::ManageProcess::Request> req,
    std::shared_ptr<ad::srv::ManageProcess::Response> resp)
  {
    if (req->name.empty()) {
      shutdown_all();
      for (auto & mp : processes_) {
        mp.state = ProcessState::PENDING;
        mp.current_backoff_sec = initial_backoff_sec_;
      }
      resp->success = true;
      resp->message = "All processes restarting";
      return;
    }
    auto * mp = find_process(req->name);
    if (!mp) {
      resp->success = false;
      resp->message = "Unknown process: " + req->name;
      return;
    }
    if (mp->state == ProcessState::RUNNING || mp->state == ProcessState::STARTING) {
      request_stop(*mp);                             // 先发 SIGINT
    }
    // 等待进程停止（阻塞当前线程，最多等到 force_kill 生效）
    while (mp->state == ProcessState::STOPPING) {
      usleep(50000);
      reap_children();
      check_stop_timeouts();
    }
    mp->state = ProcessState::PENDING;
    mp->current_backoff_sec = initial_backoff_sec_;
    start_process(*mp);
    resp->success = mp->state == ProcessState::STARTING;
    resp->message = resp->success ? req->name + " restarting" : "Failed to restart " + req->name;
  }

  // handle_shutdown 服务回调：触发全局关闭
  void handle_shutdown(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    resp->success = true;
    resp->message = "Shutting down supervisor and all processes";
    g_signal_received = 1;
  }

  // handle_get_status 服务回调：查询进程状态
  void handle_get_status(const std::shared_ptr<ad::srv::GetProcessStatus::Request> req,
    std::shared_ptr<ad::srv::GetProcessStatus::Response> resp)
  {
    if (req->name.empty()) {
      // 查询所有进程
      for (auto & mp : processes_) {
        ad::msg::ProcessStatus pmsg;
        pmsg.name = mp.name;
        pmsg.state = state_to_str(mp.state);
        pmsg.pid = mp.pid;
        pmsg.restart_count = mp.restart_count;
        pmsg.uptime_sec = (mp.state == ProcessState::RUNNING)
          ? (now() - mp.start_time).seconds() : 0.0;
        pmsg.exit_code = mp.exit_code;
        pmsg.exit_reason = mp.exit_reason;
        resp->statuses.push_back(pmsg);
      }
      resp->success = true;
      resp->message = "Returning all process statuses";
      return;
    }
    auto * mp = find_process(req->name);
    if (!mp) {
      resp->success = false;
      resp->message = "Unknown process: " + req->name;
      return;
    }
    ad::msg::ProcessStatus pmsg;
    pmsg.name = mp->name;
    pmsg.state = state_to_str(mp->state);
    pmsg.pid = mp->pid;
    pmsg.restart_count = mp->restart_count;
    pmsg.uptime_sec = (mp->state == ProcessState::RUNNING)
      ? (now() - mp->start_time).seconds() : 0.0;
    pmsg.exit_code = mp->exit_code;
    pmsg.exit_reason = mp->exit_reason;
    resp->statuses.push_back(pmsg);
    resp->success = true;
    resp->message = "";
  }

  rclcpp::Time now() { return get_clock()->now(); }

  static std::string join(const std::vector<std::string> & v)
  {
    std::string r;
    for (size_t i = 0; i < v.size(); ++i) {
      if (i > 0) r += ", ";
      r += v[i];
    }
    return r;
  }

  // ---- 成员变量 ----
  std::vector<ManagedProcess> processes_;

  // 全局 supervisor 配置
  std::string log_dir_;
  double graceful_shutdown_timeout_ = 10.0;          // 全局关闭超时 (s)
  double initial_backoff_sec_ = 1.0;                 // 初始退避时间
  double max_backoff_sec_ = 60.0;                    // 最大退避时间
  double health_check_hz_ = 2.0;                     // 健康检查频率
  bool shutting_down_ = false;                        // 是否正在关闭中
  rclcpp::Time start_time_;

  // ROS 2 接口
  rclcpp::Publisher<ad::msg::ProcessStatus>::SharedPtr status_pub_;
  rclcpp::Service<ad::srv::ManageProcess>::SharedPtr start_srv_;
  rclcpp::Service<ad::srv::ManageProcess>::SharedPtr stop_srv_;
  rclcpp::Service<ad::srv::ManageProcess>::SharedPtr restart_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;
  rclcpp::Service<ad::srv::GetProcessStatus>::SharedPtr get_status_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 安装信号处理器（SIGINT/SIGTERM → 标记 -> on_timer 中优雅关闭）
  struct sigaction sa {};
  sa.sa_handler = signal_handler;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  auto node = std::make_shared<AcflyDaemon>();

  RCLCPP_INFO(node->get_logger(), "Acfly daemon running");
  RCLCPP_INFO(node->get_logger(), "Topics: ~/status");
  RCLCPP_INFO(node->get_logger(), "Services: ~/start, ~/stop, ~/restart, ~/shutdown, ~/get_status");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
