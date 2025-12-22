#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/start_gazebo_headless_xvfb.sh
# Summary: Runs Gazebo headless using Xvfb on a virtual display.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
WORLD_FILE="${WORLD_FILE:-$WS_DIR/worlds/ur5_mesa_objetos.sdf}"

if [[ ! -f "$WORLD_FILE" ]]; then
  echo "[ERROR] No se encuentra el world: $WORLD_FILE" >&2
  exit 1
fi

cleanup() {
  if [[ -n "${GAZEBO_PID:-}" ]]; then
    kill "$GAZEBO_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "${XVFB_PID:-}" ]]; then
    kill "$XVFB_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "[INFO] Iniciando Xvfb en :99..."
Xvfb :99 -screen 0 1280x720x24 &> /tmp/xvfb_gazebo.log &
XVFB_PID=$!

sleep 3

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

echo "[INFO] Lanzando Gazebo con $WORLD_FILE en modo headless (Xvfb)..."
cd "$WS_DIR"

# -r: correr simulacion sin pausa
# -s: modo servidor (sin GUI)
gz sim -s -r "$WORLD_FILE" &> /tmp/gazebo_headless.log &
GAZEBO_PID=$!

echo "[INFO] Gazebo PID $GAZEBO_PID. Logs en /tmp/gazebo_headless.log"
echo "[INFO] Xvfb PID $XVFB_PID. Logs en /tmp/xvfb_gazebo.log"

echo "[INFO] Pulsa Ctrl+C para parar Gazebo y Xvfb."
wait "$GAZEBO_PID"
