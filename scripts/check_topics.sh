# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts/check_topics.sh
# Nombre: check_topics.sh
# Qué hace: Verifica la presencia de tópicos y controladores claves en el stack.

#!/usr/bin/env bash
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

echo "[CHECK] Listando tópicos relevantes..."
ros2 topic list | egrep 'camera|/clock|grasp|desired_grasp|joint|controller' || true

echo "[CHECK] Controladores ros2_control"
ros2 control list_controllers || true

if command -v gz >/dev/null 2>&1; then
  echo "[CHECK] Tópicos de Gazebo"
  gz topic -l || true
else
  echo "[CHECK] gz no disponible en PATH"
fi
