#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_ur5_world.sh
# Summary: Starts Gazebo headless with the UR5 table world.
set -e

# Workspace del entorno ROS 2 + Gazebo del TFM
WS_DIR="$HOME/TFM/agarre_ros2_ws"
export WS_DIR

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
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

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
#
env -u DISPLAY \
  __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  GZ_RENDER_ENGINE=ogre2 \
  gz sim -s -r --headless-rendering "$WORLD_FILE" --verbose
