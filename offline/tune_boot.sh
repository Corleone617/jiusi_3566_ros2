#!/bin/bash
#
# tune_boot.sh — 解决 rk3566 开机卡顿
#
# 消除以下 systemd 90s 超时阻塞：
#   1. /by-partlabel/oem      — 分区标签不存在，mount 单元卡住
#   2. /by-partlabel/userdata — 同上
#   3. Rockchip Wifi/BT       — 重启循环卡住
#   4. ifup@wlan0             — 网络接口未就绪
#   5. bluetooth.service      — 蓝牙硬件未就绪
#
# 策略：不碰全局 DefaultTimeoutStopSec（可能影响 journald/文件系统等关键服务），
#       只给已知慢服务做 drop-in 超时覆盖 + mask 不需要的服务。
#
# 用法：sudo bash tune_boot.sh
#

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "错误：本脚本需要 root 权限，请用 sudo 运行："
    echo "  sudo bash tune_boot.sh"
    exit 1
fi

echo "============================================"
echo "  rk3566 Boot Time Tuning"
echo "============================================"
echo ""

# ── 1. partition mount 超时覆盖 ──────────────────────────────────
echo "[1/5] 分区挂载超时处理..."

# systemd-fstab-generator 将 fstab 路径转义为单元名：
#   /by-partlabel/oem      → by\x2dpartlabel-oem.mount
#   /by-partlabel/userdata → by\x2dpartlabel-userdata.mount
#
# 这些 mount 单元可能是 fstab 动态生成的，用 systemctl mask 最可靠

PART_MOUNTS=(
    "by\\x2dpartlabel-oem.mount"
    "by\\x2dpartlabel-userdata.mount"
)

PART_LABELS=(
    "oem"
    "userdata"
)

for i in "${!PART_MOUNTS[@]}"; do
    UNIT="${PART_MOUNTS[$i]}"
    LABEL="${PART_LABELS[$i]}"

    if [ -e "/dev/disk/by-partlabel/${LABEL}" ]; then
        # 分区存在，降低挂载超时
        mkdir -p "/etc/systemd/system/${UNIT}.d"
        cat > "/etc/systemd/system/${UNIT}.d/timeout.conf" << EOF
[Unit]
JobTimeoutSec=15s

[Mount]
TimeoutSec=10s
EOF
        echo "  ${UNIT} TimeoutSec=10s ✓"
    else
        # 分区不存在，mask 掉整个 mount 单元
        systemctl mask "$UNIT" 2>/dev/null || ln -sf /dev/null "/etc/systemd/system/${UNIT}" 2>/dev/null || true
        echo "  ${UNIT} masked ✓ (分区不存在)"
    fi
done

# ── 2. Rockchip Wifi/BT 重启循环 ─────────────────────────────────
echo ""
echo "[2/5] Rockchip Wifi/BT 服务处理..."

# 尝试找到 Rockchip WiFi/BT 服务（常见命名）
WIFI_BT_SVC=""
for candidate in \
    "rk_wifi_bt.service" \
    "wifibt.service" \
    "brcm_patchram_plus.service" \
    "rtk_btusb.service" \
    "rk_wifi_init.service"; do
    if systemctl list-unit-files "$candidate" &>/dev/null; then
        WIFI_BT_SVC="$candidate"
        break
    fi
done

# 如果上述都不匹配，模糊搜索
if [ -z "$WIFI_BT_SVC" ]; then
    WIFI_BT_SVC=$(systemctl list-unit-files --type=service 2>/dev/null \
        | grep -iE 'wifi.*bt|bt.*wifi|rk_wifi|wifibt|rockchip.*(wifi|bt)|chip.*wifi' \
        | awk '{print $1}' | head -1)
fi

if [ -n "$WIFI_BT_SVC" ]; then
    mkdir -p "/etc/systemd/system/${WIFI_BT_SVC}.d"
    cat > "/etc/systemd/system/${WIFI_BT_SVC}.d/stop-restart-loop.conf" << EOF
[Service]
TimeoutStartSec=15s
TimeoutStopSec=5s
RestartSec=30s
StartLimitBurst=3
StartLimitIntervalSec=60s
EOF
    echo "  ${WIFI_BT_SVC} 超时已限制 ✓"
else
    echo "  未找到 Rockchip Wifi/BT 服务，跳过"
fi

# ── 3. ifupdown 超时覆盖 ─────────────────────────────────────────
echo ""
echo "[3/5] ifupdown 服务超时覆盖..."

mkdir -p /etc/systemd/system/networking.service.d
cat > /etc/systemd/system/networking.service.d/timeout.conf << 'EOF'
[Service]
TimeoutStartSec=15s
TimeoutStopSec=5s
EOF
echo "  networking.service TimeoutStopSec=5s ✓"

mkdir -p /etc/systemd/system/ifup@.service.d
cat > /etc/systemd/system/ifup@.service.d/timeout.conf << 'EOF'
[Service]
TimeoutStopSec=5s
EOF
echo "  ifup@.service TimeoutStopSec=5s ✓"

# mask ifupdown-wait-online（NetworkManager 管理网络时不需要）
if systemctl is-active NetworkManager &>/dev/null; then
    systemctl mask ifupdown-wait-online.service 2>/dev/null || true
    echo "  ifupdown-wait-online masked ✓"
fi

# ── 4. ModemManager ──────────────────────────────────────────────
echo ""
echo "[4/5] ModemManager..."

if ! ls /dev/ttyUSB* &>/dev/null 2>&1; then
    systemctl mask ModemManager.service 2>/dev/null || true
    echo "  ModemManager masked (无 ttyUSB 设备) ✓"
else
    echo "  检测到 ttyUSB，保留 ModemManager"
fi

# ── 5. 蓝牙服务 ──────────────────────────────────────────────────
echo ""
echo "[5/5] 蓝牙服务处理..."

if hciconfig 2>/dev/null | grep -q "hci0"; then
    mkdir -p /etc/systemd/system/bluetooth.service.d
    cat > /etc/systemd/system/bluetooth.service.d/timeout.conf << 'EOF'
[Service]
TimeoutStopSec=5s
EOF
    echo "  bluetooth.service TimeoutStopSec=5s ✓"
else
    systemctl mask bluetooth.service 2>/dev/null || true
    echo "  bluetooth.service masked (无蓝牙硬件) ✓"
fi

systemctl daemon-reload

echo ""
echo "============================================"
echo "  tune_boot.sh 完成 ✓"
echo "============================================"
echo ""
echo "  下次重启生效。验证命令:"
echo "    systemd-analyze blame | head -15"
echo ""
