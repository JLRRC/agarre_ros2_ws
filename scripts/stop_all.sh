#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/stop_all.sh
# Summary: Stops the ROS/Gazebo stack started by the panel.
set -euo pipefail

WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

echo "[STOP] stop_all.sh — stopping stack..."
"$WS_DIR/scripts/kill_all.sh"
