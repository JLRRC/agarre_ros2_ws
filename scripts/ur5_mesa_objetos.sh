#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_mesa_objetos.sh
# Summary: Runs Gazebo with the UR5 table world (GUI).
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
WORLD_FILE="${WORLD_FILE:-$WS_DIR/worlds/ur5_mesa_objetos.sdf}"

if [[ ! -f "$WORLD_FILE" ]]; then
  echo "[ERROR] No se encuentra el world: $WORLD_FILE" >&2
  exit 1
fi

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  echo "[INFO] Cargando overlay del workspace: $WS_DIR"
  source "$WS_DIR/install/setup.bash"
else
  echo "[WARN] No existe $WS_DIR/install/setup.bash"
  echo "[WARN] Compila primero con: colcon build --symlink-install"
fi
set -u

echo "[INFO] Lanzando Gazebo con el mundo UR5 + mesa + objetos (GUI)"
cd "$WS_DIR"

export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

gz sim "$WORLD_FILE" --verbose
