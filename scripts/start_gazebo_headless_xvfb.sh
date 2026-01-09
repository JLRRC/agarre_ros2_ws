#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/start_gazebo_headless_xvfb.sh
# Summary: Runs Gazebo headless using EGL (no Xvfb).
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
}
trap cleanup EXIT

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

echo "[INFO] Lanzando Gazebo con $WORLD_FILE en modo headless (EGL)..."
cd "$WS_DIR"

# -r: correr simulacion sin pausa
# -s: modo servidor (sin GUI)
env -u DISPLAY \
  __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  GZ_RENDER_ENGINE=ogre2 \
  gz sim -s -r --headless-rendering "$WORLD_FILE" &> /tmp/gazebo_headless.log &
GAZEBO_PID=$!

echo "[INFO] Gazebo PID $GAZEBO_PID. Logs en /tmp/gazebo_headless.log"

echo "[INFO] Pulsa Ctrl+C para parar Gazebo y Xvfb."
wait "$GAZEBO_PID"
