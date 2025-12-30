#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_panel_ROS2_PRO.sh
# Summary: ROS2/Gazebo-only launcher for delivery (sin pestañas ML/TFM).

set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
SCRIPTS_DIR="$WS_DIR/scripts"
LOG_DIR="$WS_DIR/log"

echo "[INFO] Panel ROS2 PRO — COLD BOOT"
echo "[INFO] WS_DIR=$WS_DIR"

cd "$WS_DIR"
mkdir -p "$LOG_DIR"

# ----------------------------
# 1) COLD BOOT: limpiar procesos ROS2/Gazebo
# ----------------------------
if [[ -x "$SCRIPTS_DIR/kill_all.sh" ]]; then
  echo "[KILL] Ejecutando scripts/kill_all.sh ..."
  "$SCRIPTS_DIR/kill_all.sh" || true
else
  echo "[KILL] kill_all.sh no existe o no es ejecutable, usando pkill fallback..."
fi

pkill -f "ros2 bag record"    >/dev/null 2>&1 || true
pkill -f "ros_gz_bridge"      >/dev/null 2>&1 || true
pkill -f "parameter_bridge"   >/dev/null 2>&1 || true
pkill -f "gz sim"             >/dev/null 2>&1 || true
pkill -f "gzserver"           >/dev/null 2>&1 || true
pkill -f "gzclient"           >/dev/null 2>&1 || true
pkill -f "rqt_image_view"     >/dev/null 2>&1 || true
pkill -f "robot_state_publisher" >/dev/null 2>&1 || true
pkill -f "ros2_control_node"  >/dev/null 2>&1 || true
pkill -f "controller_manager" >/dev/null 2>&1 || true
pkill -f "spawner"            >/dev/null 2>&1 || true
pkill -f "main_panel.py"      >/dev/null 2>&1 || true
pkill -f "ur5_qt_panel"       >/dev/null 2>&1 || true

for i in {1..40}; do
  if pgrep -f "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gzserver|gzclient|ros2_control_node|controller_manager|spawner|robot_state_publisher|main_panel\.py" >/dev/null 2>&1; then
    sleep 0.15
  else
    break
  fi
done
echo "[KILL] OK. Sistema limpio."

# ----------------------------
# 2) Source ROS (evitar AMENT_TRACE_SETUP_FILES con set -u)
# ----------------------------
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPTS_DIR/fastdds_no_shm.xml"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

# ----------------------------
# 3) Env Gazebo (evitar multicast / rutas recursos)
# ----------------------------
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_TRANSPORT_IP="${GZ_TRANSPORT_IP:-127.0.0.1}"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Panel en modo solo ROS2/Gazebo
export PANEL_COLD_BOOT="${PANEL_COLD_BOOT:-1}"
export PANEL_ROS2_ONLY="${PANEL_ROS2_ONLY:-1}"

# Evitar warnings GLX/Qt (forzamos software también con DISPLAY)
export QT_OPENGL=software
export QT_XCB_GL_INTEGRATION=none
export LIBGL_ALWAYS_SOFTWARE=1
if [[ -z "${DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi

# ----------------------------
# 4) Ejecutar panel
# ----------------------------
PANEL_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/main_panel.py"
if [[ ! -f "$PANEL_PY" ]]; then
  echo "[ERROR] No existe el panel en: $PANEL_PY"
  echo "[HINT] Revisa que el paquete sea ur5_qt_panel y que main_panel.py esté ahí."
  exit 1
fi

echo "[OK] Lanzando panel: $PANEL_PY"
exec python3 -u "$PANEL_PY"
