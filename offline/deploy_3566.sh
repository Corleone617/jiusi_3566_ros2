#!/bin/bash
#
# AcFly 部署脚本 — 离线优先，在线兜底
# 用法:  cd install/offline && ./deploy.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"    # install/offline/
INSTALL_DIR="$(dirname "$SCRIPT_DIR")"           # install/
REAL_USER="${SUDO_USER:-$USER}"

# --- 0. 修复 install/ 中所有硬编码路径（适配不同设备/用户名/路径） ---
echo "[0/8] 检查 install/ 硬编码路径..."
# 常见的旧前缀列表，部署到新板子时自动替换
OLD_PREFIXES=(
    "/home/orangepi/jiusi_625/install"
    "/home/acfly/jiusi_625/install"
)
for OLD_PREFIX in "${OLD_PREFIXES[@]}"; do
    FOUND=$(grep -rlF "$OLD_PREFIX" "$INSTALL_DIR" --include="*.sh" --include="*.bash" --include="*.cmake" 2>/dev/null | wc -l)
    if [ "$FOUND" -gt 0 ]; then
        grep -rlF "$OLD_PREFIX" "$INSTALL_DIR" --include="*.sh" --include="*.bash" --include="*.cmake" 2>/dev/null | \
            xargs -r sed -i "s|$OLD_PREFIX|$INSTALL_DIR|g"
        echo "  已修正 $FOUND 个含 $OLD_PREFIX 的文件 ✓"
    fi
done
echo "  路径检查完成 ✓"

echo "============================================"
echo "  AcFly 系统部署脚本"
echo "============================================"
echo "  安装目录: $INSTALL_DIR"
echo "  用户:      $REAL_USER"
echo ""

# --- 1. 离线依赖包 ---
echo "[1/8] 安装离线依赖包..."

TO_INSTALL=()
for DEB_FILE in "$SCRIPT_DIR"/*.deb; do
    [ -f "$DEB_FILE" ] || continue
    PKG_NAME=$(dpkg-deb -f "$DEB_FILE" Package 2>/dev/null)
    if [ -z "$PKG_NAME" ]; then
        echo "  ! 无法解析 $DEB_FILE，跳过"
        continue
    fi
    if dpkg -s "$PKG_NAME" &>/dev/null; then
        echo "  ${PKG_NAME} 已安装 ✓"
    else
        echo "  ${PKG_NAME} 待安装"
        TO_INSTALL+=("$DEB_FILE")
    fi
done

if [ ${#TO_INSTALL[@]} -gt 0 ]; then
    echo "  批量安装 ${#TO_INSTALL[@]} 个离线依赖..."
    sudo dpkg -i "${TO_INSTALL[@]}" || {
        echo "  ! 部分依赖安装失败（通常是缺少传递依赖），尝试修复..."
        sudo apt-get install -f -y 2>/dev/null || true
    }
    echo "  离线依赖安装完成 ✓"
else
    echo "  所有离线依赖已安装 ✓"
fi

# --- 2. 在线系统依赖 + ldconfig ---
echo ""
echo "[2/8] 检查在线系统依赖..."

REQUIRED_PKGS=(
    "ros-humble-rosbag2"
    "ros-humble-rosbag2-storage-mcap"
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
    echo "  安装缺失依赖: $MISSING"
    if sudo apt update -qq 2>/dev/null; then
        sudo apt install -y $MISSING 2>/dev/null || echo "  ! 部分包安装失败，请检查网络或手动安装"
    else
        echo "  ! 无法连接 apt 源，跳过在线依赖安装"
        echo "  缺失包: $MISSING"
    fi
else
    echo "  全部依赖已安装 ✓"
fi

# ldconfig: 确保 /opt/ros/humble/lib 在链接器搜索路径中（dlopen 不走 LD_LIBRARY_PATH）
ROS_LDCONF="/etc/ld.so.conf.d/ros2-humble.conf"
if [ ! -f "$ROS_LDCONF" ] || ! grep -q "/opt/ros/humble/lib" "$ROS_LDCONF" 2>/dev/null; then
    echo "/opt/ros/humble/lib" | sudo tee "$ROS_LDCONF" > /dev/null
fi
if [ -d "/opt/ros/humble/lib/aarch64-linux-gnu" ] && \
   ! grep -q "/opt/ros/humble/lib/aarch64-linux-gnu" "$ROS_LDCONF" 2>/dev/null; then
    echo "/opt/ros/humble/lib/aarch64-linux-gnu" | sudo tee -a "$ROS_LDCONF" > /dev/null
fi
sudo ldconfig 2>/dev/null || true

# --- 3. RK3566 USB 稳定性修复 ---
echo ""
echo "[3/8] RK3566 USB 稳定性修复..."

APPLIED=false
USB_SYSTEMD="/etc/systemd/system/usbcore-old-scheme.service"

# 方式1: 内核 cmdline (Orange Pi / Armbian)
if [ -f /boot/orangepiEnv.txt ]; then
    if grep -q "usbcore.old_scheme_first=1" /boot/orangepiEnv.txt 2>/dev/null; then
        echo "  usbcore.old_scheme_first=1 已配置 (orangepiEnv) ✓"
    else
        sudo sed -i '/^extraargs=/ s/$/ usbcore.old_scheme_first=1/' /boot/orangepiEnv.txt
        echo "  /boot/orangepiEnv.txt 已追加 ✓"
        APPLIED=true
    fi
fi

# 方式2: 内核 cmdline (extlinux)
if [ -f /boot/extlinux/extlinux.conf ]; then
    if grep -q "usbcore.old_scheme_first=1" /boot/extlinux/extlinux.conf 2>/dev/null; then
        echo "  usbcore.old_scheme_first=1 已配置 (extlinux) ✓"
    else
        sudo sed -i '/^[[:space:]]*APPEND/ s/$/ usbcore.old_scheme_first=1/' /boot/extlinux/extlinux.conf
        echo "  /boot/extlinux/extlinux.conf 已追加 ✓"
        APPLIED=true
    fi
fi

# 方式3: systemd oneshot (usbcore 内置且 cmdline 不可改的场景, 如 RK3566 BSP)
if [ "$APPLIED" = false ]; then
    if [ -f "$USB_SYSTEMD" ]; then
        echo "  $USB_SYSTEMD 已存在 ✓"
    else
        sudo tee "$USB_SYSTEMD" > /dev/null << 'UNIT_EOF'
[Unit]
Description=USB stability fix for RK3566 (old_scheme_first + no autosuspend)
Before=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c "echo 1 > /sys/module/usbcore/parameters/old_scheme_first && echo -1 > /sys/module/usbcore/parameters/autosuspend"
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT_EOF
        echo "  systemd oneshot 已创建 ✓"
    fi
    sudo systemctl daemon-reload
    sudo systemctl enable usbcore-old-scheme 2>/dev/null || true
    sudo systemctl start usbcore-old-scheme 2>/dev/null || true
    echo "  usbcore-old-scheme.service 已启用 ✓"
fi

# 运行时立即生效
echo 1 | sudo tee /sys/module/usbcore/parameters/old_scheme_first > /dev/null 2>&1 || true
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend > /dev/null 2>&1 || true

# --- 4. 权限设置 ---
echo ""
echo "[4/8] 设置权限..."

TIMESYNC_BIN="$INSTALL_DIR/timesync/lib/timesync/time_sync_node"
if [ -f "$TIMESYNC_BIN" ]; then
    sudo setcap cap_sys_time+ep "$TIMESYNC_BIN"
    echo "  setcap timesync ✓"
    if ldd "$TIMESYNC_BIN" 2>&1 | grep -q "not found"; then
        echo "  ! 警告: timesync 依赖库缺失，请检查 ldconfig:"
        ldd "$TIMESYNC_BIN" 2>&1 | grep "not found"
    else
        echo "  timesync 依赖库完整 ✓"
    fi
else
    echo "  ! timesync 二进制未找到: $TIMESYNC_BIN"
fi

# --- 禁用系统 NTP 服务，避免和 timesync 抢时钟 ---
# timesync 节点用 GPS 时间驯服 CLOCK_REALTIME；chronyd / systemd-timesyncd 若同时运行
# 会用互联网 NTP 或本地频率估计反向纠正时钟，与 timesync 形成反馈环，
# 导致 offset 反复发散、re-bootstrap 死循环（已实测：chrony 活跃时 ~5 分钟必发散）。
# 生产板无网络，本就不需要系统 NTP。此处幂等，未安装也不会报错。
echo "  禁用系统时间服务（避免与 timesync 冲突）..."
sudo systemctl disable --now chronyd 2>/dev/null && echo "  chronyd 已停用并禁用 ✓" || echo "  chronyd 未安装或已禁用"
sudo systemctl disable --now systemd-timesyncd 2>/dev/null && echo "  systemd-timesyncd 已停用并禁用 ✓" || echo "  systemd-timesyncd 未安装或已禁用"
sudo systemctl disable --now ntpd 2>/dev/null && echo "  ntpd 已停用并禁用 ✓" || echo "  ntpd 未安装或已禁用"
sudo systemctl mask chronyd 2>/dev/null || true      # mask 防止被其它服务间接拉起
sudo systemctl mask systemd-timesyncd 2>/dev/null || true
sudo systemctl mask ntpd 2>/dev/null || true

sudo mkdir -p /data/bags /tmp/acfly_logs
sudo chown -R "$REAL_USER:$REAL_USER" /data/bags /tmp/acfly_logs
echo "  存储目录 /data/bags ✓"
echo "  日志目录 /tmp/acfly_logs ✓"

# --- 5. GeographicLib 数据集 ---
echo ""
echo "[5/8] GeographicLib 数据集..."

GEOID_FILE="/usr/share/GeographicLib/geoids/egm96-5.pgm"
if [ -f "$GEOID_FILE" ]; then
    echo "  egm96-5 数据集已存在 ✓"
elif [ -f "$SCRIPT_DIR/egm96-5.tar.bz2" ]; then
    echo "  安装 egm96-5 数据集..."
    sudo mkdir -p /usr/share/GeographicLib/geoids
    sudo tar -xjf "$SCRIPT_DIR/egm96-5.tar.bz2" -C /usr/share/GeographicLib/
    [ -f "$GEOID_FILE" ] && echo "  egm96-5 数据集安装完成 ✓" || echo "  ! 解压后未找到 $GEOID_FILE"
else
    echo "  ! egm96-5.tar.bz2 未找到，将尝试从 mavros 安装..."
    GEO_SCRIPT="$INSTALL_DIR/mavros/lib/mavros/install_geographiclib_datasets.sh"
    [ -f "$GEO_SCRIPT" ] && sudo bash "$GEO_SCRIPT" || echo "  ! GeographicLib 安装脚本也未找到"
fi

# --- 6. 生成启动/停止脚本 ---
echo ""
echo "[6/8] 生成启动/停止脚本..."

sudo tee /usr/local/bin/acfly-start > /dev/null << SCRIPT_EOF
#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$INSTALL_DIR/setup.bash"
exec ros2 launch acfly_daemon acfly_daemon.launch.py
SCRIPT_EOF
sudo chmod +x /usr/local/bin/acfly-start

sudo tee /usr/local/bin/acfly-stop > /dev/null << 'SCRIPT_EOF'
#!/bin/bash
ros2 service call /acfly_daemon/shutdown std_srvs/srv/Trigger 2>/dev/null || \
    echo "Daemon not running or already stopped"
SCRIPT_EOF
sudo chmod +x /usr/local/bin/acfly-stop

echo "  /usr/local/bin/acfly-start ✓"
echo "  /usr/local/bin/acfly-stop ✓"

# --- 7. 安装 systemd 开机自启 ---
echo ""
echo "[7/8] 安装 systemd 开机自启..."

SERVICE_FILE="/tmp/acfly_daemon.service"

cat > "$SERVICE_FILE" << SERVICE_EOF
[Unit]
Description=AcFly Flight Control Daemon
After=usbcore-old-scheme.service
Wants=usbcore-old-scheme.service

[Service]
Type=simple
User=$REAL_USER
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="COLCON_CURRENT_PREFIX=$INSTALL_DIR"
ExecStart=/bin/bash -c "\
  source /opt/ros/humble/setup.bash && \
  source $INSTALL_DIR/setup.bash && \
  exec ros2 launch acfly_daemon acfly_daemon.launch.py"

Restart=on-failure
RestartSec=1s
StartLimitBurst=20
StartLimitIntervalSec=60
TimeoutStartSec=50
TimeoutStopSec=30
KillMode=mixed
KillSignal=SIGTERM
StandardOutput=file:/tmp/acfly_logs/acfly_daemon.log
StandardError=file:/tmp/acfly_logs/acfly_daemon.log
SyslogIdentifier=acfly_daemon

[Install]
WantedBy=multi-user.target
SERVICE_EOF

sudo cp "$SERVICE_FILE" /etc/systemd/system/acfly_daemon.service
rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload
sudo systemctl enable acfly_daemon
echo "  systemd 服务已安装并启用 ✓"

# --- 8. 完成 ---
echo ""
echo "[8/8] 部署完成"
echo ""
echo "============================================"
echo "  启动: sudo systemctl start acfly_daemon"
echo "  停止: sudo systemctl stop acfly_daemon"
echo "  状态: sudo systemctl status acfly_daemon"
echo "  日志: journalctl -u acfly_daemon -f"
echo "  数据: /data/bags/"
echo "  进程日志: /tmp/acfly_logs/"
echo "============================================"
