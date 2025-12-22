#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/run_grasp_node.sh
# Summary: Sources ROS 2 and runs grasp_pose_publisher.
set -e

WS_DIR="$HOME/TFM/agarre_ros2_ws"
VISION_DIR="$HOME/TFM/agarre_inteligente"

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
  echo "[INFO] Cargando overlay del workspace: $WS_DIR"
  source "$WS_DIR/install/setup.bash"
else
  echo "[WARN] No existe $WS_DIR/install/setup.bash"
  echo "[WARN] Compila primero con: colcon build --symlink-install"
fi

# Opcional pero útil: reutilizar código del proyecto de visión
if [ -d "$VISION_DIR/src" ]; then
  export PYTHONPATH="$VISION_DIR/src:$PYTHONPATH"
  echo "[INFO] Añadido al PYTHONPATH: $VISION_DIR/src"
fi

echo "[INFO] Lanzando nodo de publicación de poses de agarre (grasp_pose_publisher)..."

# Probamos primero un nombre de ejecutable, y si falla, otro
if ! ros2 run grasp_pose_publisher grasp_pose_publisher_node; then
  echo "[WARN] Ejecutable 'grasp_pose_publisher_node' no encontrado, probando 'grasp_pose_publisher'..."
  ros2 run grasp_pose_publisher grasp_pose_publisher
fi

