#!/bin/bash
# MaskEditor Linux打包脚本
# 使用方法: 将此脚本和mask_editor.py放在同一目录, 然后运行:
#   chmod +x build_linux.sh && ./build_linux.sh
# 支持 Ubuntu 20.04 / 22.04

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== MaskEditor Linux 打包脚本 ==="
echo "目标: 生成单文件可执行程序"
echo ""

# 检查 mask_editor.py 是否存在
if [ ! -f "mask_editor.py" ]; then
    echo "[ERROR] mask_editor.py 不在当前目录!"
    exit 1
fi

# 安装系统依赖 (PyQt6需要的库)
echo "[1/4] 安装系统依赖..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3 python3-pip python3-venv \
    libgl1-mesa-glx libglib2.0-0 libfontconfig1 \
    libxcb-xinerama0 libxcb-cursor0 libxkbcommon0 \
    libdbus-1-3 libegl1 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 \
    libxcb-shape0 2>/dev/null || true

# 创建虚拟环境
echo "[2/4] 创建Python虚拟环境..."
VENV_DIR="$SCRIPT_DIR/.venv_build"
if [ -d "$VENV_DIR" ]; then
    rm -rf "$VENV_DIR"
fi
python3 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

# 安装Python依赖
echo "[3/4] 安装Python依赖..."
pip install --upgrade pip -q
pip install PyQt5 numpy pyinstaller -q

# 打包
echo "[4/4] 使用PyInstaller打包..."
pyinstaller --onefile --windowed \
    --name MaskEditor \
    --distpath "$SCRIPT_DIR/dist_linux" \
    --workpath "$SCRIPT_DIR/build_linux" \
    --specpath "$SCRIPT_DIR" \
    mask_editor.py

# 清理
deactivate
rm -rf "$VENV_DIR" "$SCRIPT_DIR/build_linux" "$SCRIPT_DIR/MaskEditor.spec"

echo ""
echo "=== 打包完成! ==="
echo "可执行文件: $SCRIPT_DIR/dist_linux/MaskEditor"
echo "大小: $(du -h "$SCRIPT_DIR/dist_linux/MaskEditor" | cut -f1)"
echo ""
echo "使用方法: ./dist_linux/MaskEditor"
