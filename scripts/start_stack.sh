# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts/start_stack.sh
# Nombre: start_stack.sh
# Qué hace: Inicia el stack completo (Gazebo headless + bridge + panel) en background.

#!/usr/bin/env bash
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
SCRIPTS_DIR="$WS_DIR/scripts"
LOG_DIR="$WS_DIR/log"
export WS_DIR
export SCRIPTS_DIR

mkdir -p "$LOG_DIR"

echo "[STACK] Limpiando procesos previos..."
"$SCRIPTS_DIR/kill_all.sh" >/dev/null 2>&1 || true

echo "[STACK] Lanzando Gazebo headless (Xvfb)..."
WS_DIR="$WS_DIR" "$SCRIPTS_DIR/start_gazebo_headless_xvfb.sh" >/tmp/start_gazebo.log 2>&1 &
GAZEBO_PID=$!
sleep 3

echo "[STACK] Lanzando ros_gz_bridge..."
WS_DIR="$WS_DIR" "$SCRIPTS_DIR/run_gz_ros_bridge.sh" >/tmp/ros_gz_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2

echo "[STACK] Lanzando ros2_control (TF + controllers)..."
ros2 launch ur5_bringup ur5_ros2_control.launch.py use_sim_time:=true >/tmp/ur5_ros2_control.log 2>&1 &
RSP_PID=$!
sleep 1

echo "[STACK] Lanzando panel V2..."
INSTALLED_BIN="$WS_DIR/install/ur5_qt_panel/lib/ur5_qt_panel/panel_v2"
SRC_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py"
if [[ -x "$INSTALLED_BIN" ]]; then
  PANEL_COLD_BOOT=0 exec "$INSTALLED_BIN"
fi
if [[ -f "$SRC_PY" ]]; then
  export PYTHONPATH="$WS_DIR/src/ur5_qt_panel:${PYTHONPATH:-}"
  PANEL_COLD_BOOT=0 exec /usr/bin/python3 "$SRC_PY"
fi
echo "[ERROR] No encuentro panel_v2 (instalado o src)." >&2
exit 1
