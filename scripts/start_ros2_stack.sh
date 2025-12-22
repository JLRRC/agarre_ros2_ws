#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/start_ros2_stack.sh
# Summary: Sources ROS 2 and starts ros_gz_bridge for clock and cameras.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
BRIDGE_SCRIPT="$WS_DIR/scripts/run_gz_ros_bridge.sh"

if [[ ! -x "$BRIDGE_SCRIPT" ]]; then
  echo "[ERROR] No existe o no es ejecutable: $BRIDGE_SCRIPT" >&2
  exit 1
fi

exec "$BRIDGE_SCRIPT"
