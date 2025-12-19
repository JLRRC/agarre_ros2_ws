#!/usr/bin/env bash
set -e

# Workspace del entorno ROS 2 + Gazebo del TFM
WS_DIR="$HOME/TFM/agarre_ros2_ws"

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[INFO] Cargando overlay del workspace: $WS_DIR"
  source "$WS_DIR/install/setup.bash"
else
  echo "[WARN] No existe $WS_DIR/install/setup.bash"
  echo "[WARN] Compila primero con: colcon build --symlink-install"
fi

echo "[INFO] Configurando rutas de recursos para Gazebo (GZ_SIM_RESOURCE_PATH)..."
export GZ_SIM_RESOURCE_PATH="$WS_DIR/worlds:$WS_DIR/install:$GZ_SIM_RESOURCE_PATH"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

cd "$WS_DIR"

WORLD_FILE="worlds/ur5_mesa_objetos.sdf"

if [ ! -f "$WORLD_FILE" ]; then
  echo "[ERROR] No se encuentra el world $WORLD_FILE"
  exit 1
fi

echo "[INFO] Lanzando Gazebo en modo headless con el mundo: $WORLD_FILE"
echo "[INFO] (UR5 + mesa + objetos del TFM)"

# IMPORTANTE:
# - DISPLAY=       → borra el DISPLAY para que NO use el X del Mac (ssh -Y)
# - --headless-rendering → usa EGL / renderizado sin GUI (solo sensores)
# DISPLAY= gz sim -s --headless-rendering "$WORLD_FILE" --verbose
#
#
gz sim  "$WORLD_FILE" --verbose
