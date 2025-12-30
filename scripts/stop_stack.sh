# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts/stop_stack.sh
# Nombre: stop_stack.sh
# Qué hace: Detiene todos los procesos del stack ROS2/Gazebo/panel.

#!/usr/bin/env bash
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
SCRIPTS_DIR="$WS_DIR/scripts"

echo "[STACK] Deteniendo stack completo..."
"$SCRIPTS_DIR/kill_all.sh" || true
