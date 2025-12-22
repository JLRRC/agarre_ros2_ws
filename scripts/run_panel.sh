#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_panel.sh
# Summary: Cold boot launcher for the main Qt panel.
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
LOG_DIR="$WS_DIR/log"
mkdir -p "$LOG_DIR"

echo "[INFO] COLD BOOT previo (matando Gazebo/Bridge/Rosbag si estuvieran vivos)..."
pkill -f "ros2 bag record"     >/dev/null 2>&1 || true
pkill -f "ros_gz_bridge"       >/dev/null 2>&1 || true
pkill -f "parameter_bridge"    >/dev/null 2>&1 || true
pkill -f "gz sim"              >/dev/null 2>&1 || true
pkill -f "gz gui"              >/dev/null 2>&1 || true
pkill -f "gzserver"            >/dev/null 2>&1 || true
pkill -f "gzclient"            >/dev/null 2>&1 || true
sleep 0.3

echo "[INFO] Cargando entorno ROS 2 Jazzy + overlay del workspace..."
# Evita el clásico fallo con set -u y AMENT_TRACE_SETUP_FILES
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

# Evitar warnings GLX/Qt (forzamos software también con DISPLAY)
export QT_OPENGL=software
export QT_XCB_GL_INTEGRATION=none
export LIBGL_ALWAYS_SOFTWARE=1
if [[ -z "${DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi

PANEL_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/main_panel.py"
if [[ ! -f "$PANEL_PY" ]]; then
  PANEL_PY="$WS_DIR/ur5_qt_panel/main_panel.py"
fi

if [[ ! -f "$PANEL_PY" ]]; then
  echo "[ERROR] No encuentro main_panel.py"
  echo "        Probé:"
  echo "        - $WS_DIR/src/ur5_qt_panel/ur5_qt_panel/main_panel.py"
  echo "        - $WS_DIR/ur5_qt_panel/main_panel.py"
  exit 1
fi

echo "[INFO] Lanzando Panel SUPER PRO: $PANEL_PY"
export WS_DIR
exec python3 -u "$PANEL_PY"
