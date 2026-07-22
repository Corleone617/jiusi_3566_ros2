#!/bin/bash
#
# AcFly 部署脚本 — RK3588 专用（精简版）
# 功能：① 检查系统依赖 + 工程二进制 ② 设置权限（setcap + 数据目录）
#       ③ 禁用系统时间服务（避免与 timesync 抢时钟）④ 设置开机自启
# 工程路径按脚本所在位置自动推导：offline/ 的父目录即工程根（含 install/）
# 用法:  cd <工程根> && sudo ./offline/deploy_3588.sh
#

set -e

# ---- root 权限检查 ----
# 本脚本要写 /etc/ld.so.conf.d、setcap、systemctl、/etc/systemd/system 等，必须 root
if [ "$(id -u)" -ne 0 ]; then
    echo "============================================"
    echo "错误：本脚本需要 root 权限运行"
    echo "请用:  sudo $0"
    echo "============================================"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"    # <工程根>/offline
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"           # 工程根（含 src/ install/）
INSTALL_DIR="$PROJECT_DIR/install"
REAL_USER="${SUDO_USER:-$USER}"

echo "============================================"
echo "  AcFly 部署 (RK3588)"
echo "  工程根:   $PROJECT_DIR"
echo "  install:  $INSTALL_DIR"
echo "  运行用户: $REAL_USER"
echo "============================================"

if [ ! -d "$INSTALL_DIR" ]; then
    echo "! 未找到 install/ 目录：$INSTALL_DIR"
    echo "  请先在本机执行 colcon build（source /opt/ros/humble/setup.bash && colcon build）"
    exit 1
fi

# ============================================================
# [1/5] 检查系统依赖
# ============================================================
echo ""
echo "[1/5] 检查系统依赖..."

REQUIRED_PKGS=(
    "ros-humble-rosbag2"
    "ros-humble-rosbag2-storage-mcap"
    "ros-humble-cyclonedds"
    "ros-humble-rmw-cyclonedds-cpp"
    "ros-humble-cv-bridge"
    "ros-humble-pcl-conversions"
    "ros-humble-tf2-ros"
    "ros-humble-mavros"
    "ros-humble-mavros-extras"
    "ros-humble-image-transport"
    "ros-humble-message-filters"
    "ros-humble-class-loader"
    "ros-humble-nav-msgs"
    "ros-humble-sensor-msgs"
    "ros-humble-diagnostic-msgs"
    "ros-humble-geographic-msgs"
    "ros-humble-std-srvs"
    "libusb-1.0-0"
    "libgeographic19"
    "libyaml-cpp0.7"
    "libsqlite3-0"
    "libopencv-dev"
)

MISSING=""
for pkg in "${REQUIRED_PKGS[@]}"; do
    dpkg -s "$pkg" &>/dev/null || MISSING="$MISSING $pkg"
done

if [ -n "$MISSING" ]; then
    echo "  安装缺失依赖:$MISSING"
    if sudo apt update -qq 2>/dev/null; then
        sudo apt install -y $MISSING 2>/dev/null || echo "  ! 部分包安装失败，请检查网络或手动安装"
    else
        echo "  ! 无法连接 apt 源，跳过在线依赖安装"
        echo "  缺失包:$MISSING"
    fi
else
    echo "  系统依赖齐全 ✓"
fi

# 关键二进制存在性检查
for bin in \
    "acfly_daemon/lib/acfly_daemon/acfly_daemon_node" \
    "timesync/lib/timesync/time_sync_node" \
    "bag_logd/lib/bag_logd/bag_logd_node" \
    "acfly_odometry/lib/acfly_odometry/acfly_odometry_node"; do
    if [ ! -f "$INSTALL_DIR/$bin" ]; then
        echo "  ! 缺失二进制: $INSTALL_DIR/$bin"
        echo "    请先 colcon build"
        exit 1
    fi
done
echo "  工程关键二进制齐全 ✓"

# ldconfig: 把 ROS2 库路径写进 ld.so.conf（setcap 安全模式下 dlopen 不走 LD_LIBRARY_PATH）
ROS_LDCONF="/etc/ld.so.conf.d/ros2-humble.conf"
if [ ! -f "$ROS_LDCONF" ] || ! grep -q "/opt/ros/humble/lib" "$ROS_LDCONF" 2>/dev/null; then
    echo "/opt/ros/humble/lib" | tee "$ROS_LDCONF" > /dev/null
fi
if [ -d "/opt/ros/humble/lib/aarch64-linux-gnu" ] && \
   ! grep -q "/opt/ros/humble/lib/aarch64-linux-gnu" "$ROS_LDCONF" 2>/dev/null; then
    echo "/opt/ros/humble/lib/aarch64-linux-gnu" | tee -a "$ROS_LDCONF" > /dev/null
fi
ldconfig 2>/dev/null || true
echo "  ld.so.conf 注册 ROS2 库路径 ✓"

# ============================================================
# [2/5] 设置权限（setcap + 数据目录）
# ============================================================
echo ""
echo "[2/5] 设置权限..."

# timesync 需要 cap_sys_time 才能 clock_settime/adjtimex
# 注意：每次 colcon build 后 capability 丢失，需重新执行本步
TIMESYNC_BIN="$INSTALL_DIR/timesync/lib/timesync/time_sync_node"
setcap cap_sys_time+ep "$TIMESYNC_BIN"
echo "  setcap cap_sys_time → timesync ✓"
if ldd "$TIMESYNC_BIN" 2>&1 | grep -q "not found"; then
    echo "  ! 警告: timesync 依赖库缺失（setcap 安全模式下走 ld.so.cache）："
    ldd "$TIMESYNC_BIN" 2>&1 | grep "not found"
fi

mkdir -p /data/bags /tmp/acfly_logs
chown -R "$REAL_USER:$REAL_USER" /data/bags /tmp/acfly_logs
echo "  存储目录 /data/bags, /tmp/acfly_logs ✓"

# ============================================================
# [3/5] 禁用系统时间服务（避免与 timesync 抢 CLOCK_REALTIME）
# ============================================================
# timesync 节点用 GPS 时间驯服 CLOCK_REALTIME；chronyd / systemd-timesyncd / ntpd
# 若同时运行，会用互联网 NTP 或本地频率估计反向纠正时钟，与 timesync 形成反馈环，
# 导致 offset 反复发散、re-bootstrap 死循环（已实测）。3588 板若无 RTC 电池、
# 且由 GPS 授时，本就不需要系统 NTP。此处幂等，未安装也不报错。
echo ""
echo "[3/5] 禁用系统时间服务（避免与 timesync 冲突）..."
systemctl disable --now chronyd 2>/dev/null && echo "  chronyd 已停用并禁用 ✓" || echo "  chronyd 未安装或已禁用"
systemctl disable --now systemd-timesyncd 2>/dev/null && echo "  systemd-timesyncd 已停用并禁用 ✓" || echo "  systemd-timesyncd 未安装或已禁用"
systemctl disable --now ntpd 2>/dev/null && echo "  ntpd 已停用并禁用 ✓" || echo "  ntpd 未安装或已禁用"
systemctl mask chronyd 2>/dev/null || true      # mask 防止被其它服务间接拉起
systemctl mask systemd-timesyncd 2>/dev/null || true
systemctl mask ntpd 2>/dev/null || true

# ============================================================
# [4/5] 生成启动/停止脚本
# ============================================================
echo ""
echo "[4/5] 生成启动/停止脚本..."

tee /usr/local/bin/acfly-start > /dev/null << SCRIPT_EOF
#!/bin/bash
# 手动启动 AcFly 全套系统
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$INSTALL_DIR/setup.bash"
exec ros2 launch acfly_daemon acfly_daemon.launch.py
SCRIPT_EOF
chmod +x /usr/local/bin/acfly-start

tee /usr/local/bin/acfly-stop > /dev/null << 'SCRIPT_EOF'
#!/bin/bash
# 优雅停止 AcFly 系统（通过 daemon 的 ~/shutdown 服务逆序关闭所有子进程）
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 service call /acfly_daemon/shutdown std_srvs/srv/Trigger 2>/dev/null || \
    echo "Daemon not running or already stopped"
SCRIPT_EOF
chmod +x /usr/local/bin/acfly-stop

echo "  /usr/local/bin/acfly-start ✓"
echo "  /usr/local/bin/acfly-stop ✓"

# ============================================================
# [5/5] 安装 systemd 开机自启
# ============================================================
echo ""
echo "[5/5] 安装 systemd 开机自启..."

SERVICE_FILE="/tmp/acfly_daemon.service"

cat > "$SERVICE_FILE" << SERVICE_EOF
[Unit]
Description=AcFly Flight Control Daemon (RK3588 autostart)
StartLimitIntervalSec=60
StartLimitBurst=20

[Service]
Type=simple
User=$REAL_USER
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="COLCON_CURRENT_PREFIX=$INSTALL_DIR"
# /tmp 是 tmpfs，重启即清空；ExecStartPre 重建该目录供节点内部日志(*.log)使用
# 切勿改回 StandardOutput=file:/tmp/...：ExecStartPre 会继承该设置，在目录创建前就要打开日志文件，触发 209/STDOUT 死锁
ExecStartPre=/bin/mkdir -p /tmp/acfly_logs
# 开机时 PX4 飞控 boot 需 ~15-30s，慢于 mavros 的 conn_timeout(10s)，mavros 启动过早会连不上飞控
# 延迟 20s 等飞控就绪（若飞控 boot 更慢可调大），避免开机后必须手动 restart
ExecStartPre=/bin/sleep 20
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source $INSTALL_DIR/setup.bash && exec ros2 launch acfly_daemon acfly_daemon.launch.py"
Restart=on-failure
RestartSec=1s
TimeoutStartSec=50
TimeoutStopSec=30
KillMode=mixed
KillSignal=SIGTERM
# 赋予实时调度能力:odin SDK 的 IMU 线程需 SCHED_FIFO/SCHED_RR，非 root 下会 EPERM 回退默认调度
# 切勿加 CapabilityBoundingSet 限定，否则会砍掉 timesync 二进制的 cap_sys_time 文件能力
AmbientCapabilities=CAP_SYS_NICE
StandardOutput=journal
StandardError=journal
SyslogIdentifier=acfly_daemon

[Install]
WantedBy=multi-user.target
SERVICE_EOF

cp "$SERVICE_FILE" /etc/systemd/system/acfly_daemon.service
rm -f "$SERVICE_FILE"
systemctl daemon-reload
systemctl enable acfly_daemon
echo "  acfly_daemon.service 已安装并启用（开机自启）✓"

# ============================================================
# 完成
# ============================================================
echo ""
echo "============================================"
echo "  部署完成 ✓"
echo "--------------------------------------------"
echo "  立即启动: sudo systemctl start acfly_daemon"
echo "  停止:     sudo systemctl stop acfly_daemon"
echo "  状态:     sudo systemctl status acfly_daemon"
echo "  服务日志: journalctl -u acfly_daemon -f"
echo "  daemon/子进程日志: /tmp/acfly_logs/"
echo "  手动启动: acfly-start   手动停止: acfly-stop"
echo "  取消自启: sudo systemctl disable acfly_daemon"
echo "--------------------------------------------"
echo "  提示: 每次 colcon build 后需重新 setcap："
echo "    sudo setcap cap_sys_time+ep $TIMESYNC_BIN"
echo "============================================"
