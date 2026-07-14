// acfly_odometry — 里程计融合节点
// 将 Odin1 SLAM 里程计（精确 yaw + 位置）与 FCU IMU 姿态（重力对齐的 roll/pitch）融合
// 输出重力对齐的组合里程计 /mavros/odometry/out，注入飞控 EKF 估计器
// 同时监控 SLAM 协方差，发散时自动触发 Odin1 SLAM 复位
//
// 融合算法：
//   1. 从 Odin1 SLAM 提取 yaw + position（准确，不受加速度影响）
//   2. 从 FCU IMU 提取 roll + pitch（重力对齐，可靠）
//   3. q_fused = Rz(yaw_slam) * Ry(pitch_fcu) * Rx(roll_fcu)
//   4. 加上安装角补偿：q_body = q_mount^-1 * q_fused
//   5. 速度转换到机体坐标系：v_body = q_mount^-1 * v_slam
//
// 协方差退化检测：
//   统计连续协方差超限的帧数，超过 cov_degrade_bound 次（≈1.5s @20Hz）则写复位命令到文件

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

class OdomGravityAlignNode : public rclcpp::Node
{
public:
  OdomGravityAlignNode()
  : Node("acfly_odometry"),
    last_published_stamp_(0, 0, RCL_ROS_TIME)
  {
    // ---- 基础参数 ----
    declare_parameter("output_rate", 20.0);             // 输出频率 (Hz)
    declare_parameter("odom_frame", "map");             // 里程计父坐标系
    declare_parameter("child_frame", "base_link");      // 里程计子坐标系（机体）
    declare_parameter("attitude_alignment_sec", 5.0);   // 姿态对齐窗口时长 (s)
    declare_parameter("mount_roll_deg", 0.0);           // 传感器安装 roll 补偿 (°)
    declare_parameter("mount_pitch_deg", 45.0);         // 传感器安装 pitch 补偿 (°)
    declare_parameter("mount_yaw_deg", 0.0);            // 传感器安装 yaw 补偿 (°)

    // ---- 协方差退化检测参数 ----
    declare_parameter("cov_check_enabled", true);        // 启用协方差检测
    declare_parameter("cov_pos_threshold", 0.25);        // 位置协方差阈值 (m²)
    declare_parameter("cov_att_threshold_deg2", 1.0);    // 姿态协方差阈值 (deg²)
    declare_parameter("cov_degrade_bound", 20);          // 连续超限计数阈值
    declare_parameter("reset_cooldown_sec", 2.0);        // 复位冷却时间 (s)

    output_rate_ = get_parameter("output_rate").as_double();
    odom_frame_ = get_parameter("odom_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    attitude_alignment_sec_ = get_parameter("attitude_alignment_sec").as_double();
    node_start_time_ = get_clock()->now();

    // 安装角四元数：q_mount = Rz * Ry * Rx
    double roll_r  = get_parameter("mount_roll_deg").as_double()  * M_PI / 180.0;
    double pitch_r = get_parameter("mount_pitch_deg").as_double() * M_PI / 180.0;
    double yaw_r   = get_parameter("mount_yaw_deg").as_double()   * M_PI / 180.0;

    q_mount_ = Eigen::AngleAxisd(yaw_r,   Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pitch_r, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(roll_r,  Eigen::Vector3d::UnitX());
    q_mount_inv_ = q_mount_.conjugate();  // 逆四元数用于速度转换

    // 协方差检测参数
    cov_check_enabled_ = get_parameter("cov_check_enabled").as_bool();
    cov_pos_threshold_ = get_parameter("cov_pos_threshold").as_double();
    cov_att_threshold_deg2_ = get_parameter("cov_att_threshold_deg2").as_double();
    cov_degrade_bound_ = get_parameter("cov_degrade_bound").as_int();
    reset_cooldown_sec_ = get_parameter("reset_cooldown_sec").as_double();

    if (cov_check_enabled_) {
      RCLCPP_INFO(get_logger(),
        "Covariance check enabled: pos_thr=%.2f m²  att_thr=%.1f deg²  bound=%d  cooldown=%.1fs",
        cov_pos_threshold_, cov_att_threshold_deg2_, cov_degrade_bound_, reset_cooldown_sec_);
    }

    RCLCPP_INFO(get_logger(),
      "mount RPY=%.1f/%.1f/%.1f deg  output_rate=%.0f Hz  frame=%s->%s",
      get_parameter("mount_roll_deg").as_double(),
      get_parameter("mount_pitch_deg").as_double(),
      get_parameter("mount_yaw_deg").as_double(),
      output_rate_, odom_frame_.c_str(), child_frame_.c_str());

    // 订阅 Odin1 SLAM 高频里程计（400Hz，best_effort 允许丢包）
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odin1/odometry_highfreq",
      rclcpp::SensorDataQoS(),
      std::bind(&OdomGravityAlignNode::odom_cb, this, std::placeholders::_1));

    // 订阅 FCU IMU 数据（用于提取 roll/pitch 重力对齐）
    att_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(&OdomGravityAlignNode::att_cb, this, std::placeholders::_1));

    // 发布融合后里程计到 /mavros/odometry/out（飞控 EKF 输入）
    aligned_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mavros/odometry/out", rclcpp::QoS(10).reliable());

    // 定时器以 output_rate_ 频率触发融合和发布
    auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / output_rate_));
    publish_timer_ = create_wall_timer(period,
      std::bind(&OdomGravityAlignNode::timer_cb, this));

    // 发布静态 TF：map -> odom, map -> odin1_odom
    publish_static_tfs();
  }

private:
  // 检查里程计协方差是否退化
  // 当 pos_cov > pos_thresh 或 att_cov > att_thresh_deg2 累计超过 degrade_bound 次返回 true
  bool checkCovarianceDeagation(
      const std::array<double, 36>& covariance,
      double pos_thresh, double att_thresh_deg2, int degrade_bound)
  {
    // rad² → deg² 换算系数
    const double rad2_to_deg2 = (180.0 / M_PI) * (180.0 / M_PI);

    // 检查协方差矩阵对角线元素：索引 0/7/14 为位置方差，21/28/35 为姿态方差 (rad²)
    bool degraded = (covariance[0]  > pos_thresh ||
                     covariance[7]  > pos_thresh ||
                     covariance[14] > pos_thresh ||
                     covariance[21] * rad2_to_deg2 > att_thresh_deg2 ||
                     covariance[28] * rad2_to_deg2 > att_thresh_deg2 ||
                     covariance[35] * rad2_to_deg2 > att_thresh_deg2);

    if (degraded) {
      cov_degrade_cnt_++;                            // 超限累计
    } else {
      if (cov_degrade_cnt_ > 0) cov_degrade_cnt_--;  // 正常则缓慢恢复
    }
    if (cov_degrade_cnt_ > degrade_bound) {
      cov_degrade_cnt_ = 0;
      return true;
    }
    return false;
  }

  // 发布静态坐标变换：map → odom, map → odin1_odom（均为 identity）
  void publish_static_tfs()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = "odom";
    tf_broadcaster_.sendTransform(tf);

    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = "odin1_odom";
    tf_broadcaster_.sendTransform(tf);

    RCLCPP_INFO(get_logger(),
      "static TFs published: %s -> odom, %s -> odin1_odom",
      odom_frame_.c_str(), odom_frame_.c_str());
  }

  // SLAM 里程计回调：缓存最新数据
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
  }

  // FCU IMU 回调：缓存最新数据（用于提取重力对齐的 roll/pitch）
  void att_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(att_mutex_);
    latest_att_ = msg;
  }

  // 定时器回调：以 output_rate_ 频率执行融合算法并发布
  void timer_cb()
  {
    nav_msgs::msg::Odometry::SharedPtr odom;
    sensor_msgs::msg::Imu::SharedPtr att;

    // 获取最新数据快照
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!latest_odom_) return;                    // 尚无里程计数据
      odom = latest_odom_;
    }

    {
      std::lock_guard<std::mutex> lock(att_mutex_);
      att = latest_att_;
    }

    // 跳过已经发布过的时间戳（防止重复发布）
    if (rclcpp::Time(odom->header.stamp) <= last_published_stamp_) {
      return;
    }

    // ---- 协方差退化检测 → 触发 Odin1 SLAM 复位 ----
    if (cov_check_enabled_) {
      auto now = this->now();
      if ((now - last_reset_time_).seconds() > reset_cooldown_sec_) {
        // 冷却期内不重复触发
        if (checkCovarianceDeagation(
              odom->pose.covariance,
              cov_pos_threshold_, cov_att_threshold_deg2_, cov_degrade_bound_))
        {
          RCLCPP_WARN(get_logger(),
            "Covariance degraded! Triggering Odin SLAM reset");
          // 写复位命令到文件，Odin1 驱动 10Hz 轮询此文件
          std::ofstream cmd("/tmp/odin_command.txt", std::ios::trunc);
          if (cmd.is_open()) {
            cmd << "set algo_reset 1\n";
            cmd.close();
            last_reset_time_ = now;
          } else {
            RCLCPP_ERROR(get_logger(),
              "Failed to open /tmp/odin_command.txt for reset command");
          }
        }
      }
    }

    // 复位后 1 秒内抑制输出（等待 SLAM 恢复）
    if ((this->now() - last_reset_time_).seconds() <= 1.0) {
      return;
    }

    // ---- 姿态对齐 ----
    bool att_valid = false;
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond::Identity();
    double elapsed = (this->now() - node_start_time_).seconds();

    // 对齐窗口：前 attitude_alignment_sec_ 秒内使用 FCU IMU 姿态
    // FCU IMU 的 roll/pitch 是基于重力向量的，比 SLAM 的 roll/pitch 更可靠
    if (att && elapsed < attitude_alignment_sec_) {
      q_fcu = Eigen::Quaterniond(att->orientation.w,
                                 att->orientation.x,
                                 att->orientation.y,
                                 att->orientation.z);
      q_fcu.normalize();
      att_valid = true;
    }

    // ---- 构建输出里程计 ----
    auto aligned = std::make_unique<nav_msgs::msg::Odometry>();
    aligned->header.stamp = odom->header.stamp;
    aligned->header.frame_id = odom_frame_;
    aligned->child_frame_id = child_frame_;

    aligned->pose = odom->pose;                     // 位置直接从 SLAM 复制

    // 速度转换：传感器坐标系 → 机体坐标系
    {
      Eigen::Vector3d v_s(odom->twist.twist.linear.x,
                          odom->twist.twist.linear.y,
                          odom->twist.twist.linear.z);
      Eigen::Vector3d w_s(odom->twist.twist.angular.x,
                          odom->twist.twist.angular.y,
                          odom->twist.twist.angular.z);
      Eigen::Vector3d v_b = q_mount_inv_ * v_s;     // 线速度：传感器→机体
      Eigen::Vector3d w_b = q_mount_inv_ * w_s;     // 角速度：传感器→机体
      aligned->twist.twist.linear.x = v_b.x();
      aligned->twist.twist.linear.y = v_b.y();
      aligned->twist.twist.linear.z = v_b.z();
      aligned->twist.twist.angular.x = w_b.x();
      aligned->twist.twist.angular.y = w_b.y();
      aligned->twist.twist.angular.z = w_b.z();
    }

    // SLAM 四元数（传感器坐标系）
    Eigen::Quaterniond q_slam(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);
    q_slam.normalize();

    // 补偿安装角：q_slam_drone = q_slam * q_mount^-1
    Eigen::Quaterniond q_slam_drone = q_slam * q_mount_inv_;
    q_slam_drone.normalize();

    if (att_valid) {
      // ---- 融合模式 ----
      // 从 SLAM 提取 yaw，从 FCU IMU 提取 roll/pitch
      Eigen::Vector3d rpy_slam =
        q_slam_drone.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX 欧拉角
      double yaw_slam = rpy_slam[0];                          // 取 yaw

      Eigen::Vector3d rpy_fcu =
        q_fcu.toRotationMatrix().eulerAngles(2, 1, 0);        // FCU 姿态欧拉角
      double roll_fcu  = rpy_fcu[2];                          // 取 roll（重力对齐）
      double pitch_fcu = rpy_fcu[1];                          // 取 pitch

      // 组合四元数：yaw 来自 SLAM，roll/pitch 来自 FCU
      Eigen::Quaterniond q_aligned_drone =
        Eigen::AngleAxisd(yaw_slam,  Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch_fcu, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll_fcu,  Eigen::Vector3d::UnitX());
      q_aligned_drone.normalize();

      aligned->pose.pose.orientation.w = q_aligned_drone.w();
      aligned->pose.pose.orientation.x = q_aligned_drone.x();
      aligned->pose.pose.orientation.y = q_aligned_drone.y();
      aligned->pose.pose.orientation.z = q_aligned_drone.z();
    } else {
      // ---- 直通模式（超出对齐窗口） ----
      // 只做安装角补偿，直接使用 SLAM 姿态
      aligned->pose.pose.orientation.w = q_slam_drone.w();
      aligned->pose.pose.orientation.x = q_slam_drone.x();
      aligned->pose.pose.orientation.y = q_slam_drone.y();
      aligned->pose.pose.orientation.z = q_slam_drone.z();
    }

    // 发布动态 TF（map → base_link）
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = aligned->header.stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = aligned->pose.pose.position.x;
      tf.transform.translation.y = aligned->pose.pose.position.y;
      tf.transform.translation.z = aligned->pose.pose.position.z;
      tf.transform.rotation = aligned->pose.pose.orientation;
      tf_pub_.sendTransform(tf);
    }

    last_published_stamp_ = rclcpp::Time(odom->header.stamp);
    aligned_pub_->publish(std::move(aligned));
  }

  // ---- 成员变量 ----
  double output_rate_;
  std::string odom_frame_;
  std::string child_frame_;
  double attitude_alignment_sec_;
  Eigen::Quaterniond q_mount_;                // 传感器安装角四元数
  Eigen::Quaterniond q_mount_inv_;            // 安装角逆四元数

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr att_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_{this};
  tf2_ros::TransformBroadcaster tf_pub_{this};

  std::mutex odom_mutex_;
  std::mutex att_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::Imu::SharedPtr latest_att_;

  rclcpp::Time last_published_stamp_;         // 上次发布的时间戳，防重复
  rclcpp::Time node_start_time_;
  rclcpp::Time last_reset_time_{0, 0, RCL_ROS_TIME};  // 上次 SLAM 复位时间

  // 协方差检测参数
  bool cov_check_enabled_;
  double cov_pos_threshold_;
  double cov_att_threshold_deg2_;
  int cov_degrade_bound_;
  double reset_cooldown_sec_;
  int cov_degrade_cnt_ = 0;                   // 当前连续超限计数
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomGravityAlignNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
