#!/bin/bash
# Sensor Data Viewer launcher — sources ROS2 Humble then starts the GUI
#
# Usage:  ./run_viewer.sh [bag_path]

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ -z "$DISPLAY" ]; then
    echo "[WARN] DISPLAY is not set. GUI may not open."
    echo "       If running over SSH, use: ssh -X user@host"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] /opt/ros/humble/setup.bash not found"
    exit 1
fi

exec python3 "$SCRIPT_DIR/sensor_viewer.py" "$@"
