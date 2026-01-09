#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_panel.sh
# Summary: Cold boot launcher for the main Qt panel.
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
LOG_DIR="$WS_DIR/log"
mkdir -p "$LOG_DIR"

open_log_terminal() {
  local title="$1"
  local cmd="$2"
  local started=0
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$cmd" >/dev/null 2>&1 && started=1 || true
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "$title" -e bash -lc "$cmd" >/dev/null 2>&1 && started=1 || true
  elif command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle="$title" -e bash -lc "$cmd" >/dev/null 2>&1 && started=1 || true
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="$title" --command "bash -lc \"$cmd\"" >/dev/null 2>&1 && started=1 || true
  elif command -v mate-terminal >/dev/null 2>&1; then
    mate-terminal --title="$title" -- bash -lc "$cmd" >/dev/null 2>&1 && started=1 || true
  elif command -v terminator >/dev/null 2>&1; then
    terminator --title="$title" -e "bash -lc \"$cmd\"" >/dev/null 2>&1 && started=1 || true
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd" >/dev/null 2>&1 && started=1 || true
  else
    echo "[WARN] No encuentro emulador de terminal. Ejecuta a mano: $cmd"
    return 1
  fi
  if [[ "$started" == "1" ]]; then
    echo "[INFO] Logs: terminal abierto -> $title"
    return 0
  fi
  echo "[WARN] Logs: no pude abrir terminal -> $title"
  return 1
}

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
if [[ -n "${DISPLAY:-}" && -z "${PANEL_OPEN_LOGS:-}" ]]; then
  PANEL_OPEN_LOGS=1
fi
if [[ "${PANEL_OPEN_LOGS:-0}" == "1" ]]; then
  echo "[INFO] Logs: abriendo terminales de Gazebo y ROS2"
  open_log_terminal "Gazebo logs" "while true; do if ls '$LOG_DIR'/gz_*.log >/dev/null 2>&1; then tail -n 200 -F '$LOG_DIR'/gz_*.log; else echo '[WAIT] No hay log de Gazebo aun'; sleep 1; fi; done"
  open_log_terminal "ROS2 logs" "while true; do if ls '$LOG_DIR'/ros2_control.log '$LOG_DIR'/ros_gz_bridge.log >/dev/null 2>&1; then tail -n 200 -F '$LOG_DIR'/ros2_control.log '$LOG_DIR'/ros_gz_bridge.log; else echo '[WAIT] No hay logs ROS2 aun'; sleep 1; fi; done"
fi
exec python3 -u "$PANEL_PY"
