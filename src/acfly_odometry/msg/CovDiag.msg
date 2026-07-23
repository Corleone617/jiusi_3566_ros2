# SLAM 协方差退化诊断
# 由 acfly_odometry 节点每帧（与 /mavros/odometry/out 同频）发布，
# 供 bag 录制后离线分析 SLAM 退化趋势与复位触发时机。

std_msgs/Header header       # stamp = 对应 SLAM odom 的时间戳
float64 max_pos_cov          # max(pose.covariance[0,7,14])，位置不确定度 (m²)
float64 max_att_cov_deg2     # max(pose.covariance[21,28,35]) 换算后，姿态不确定度 (deg²)
int32 degrade_cnt            # 当前连续超限计数（超限++，正常--，触发复位后清零）
int32 degrade_bound          # 触发复位的计数阈值（画"跳闸线"用）
bool reset_triggered         # 本帧是否触发了 Odin1 SLAM 复位
