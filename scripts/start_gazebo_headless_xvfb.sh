#!/usr/bin/env bash
set -e

echo "[INFO] Iniciando Xvfb en :99..."
# Lanzar un display virtual en :99 (sin GUI física)
Xvfb :99 -screen 0 1280x720x24 &> /tmp/xvfb_gazebo.log &
XVFB_PID=$!

# Darle un poco de margen para que levante
sleep 3

# Variables para que Gazebo use este display y render por software
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Si ya usas overlay del workspace:
if [ -f "/home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source /home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash
fi

echo "[INFO] Lanzando Gazebo con el mundo ur5_mesa_objetos.sdf en modo headless (Xvfb)..."
cd /home/laboratorio/TFM/agarre_ros2_ws

# -r: correr simulación, sin pausar al inicio
gz sim -r worlds/ur5_mesa_objetos.sdf &> /tmp/gazebo_headless.log & 
GAZEBO_PID=$!

echo "[INFO] Gazebo lanzado con PID $GAZEBO_PID. Logs en /tmp/gazebo_headless.log"
echo "[INFO] Xvfb PID = $XVFB_PID. Logs en /tmp/xvfb_gazebo.log"
echo "[INFO] Pulsa Ctrl+C en este terminal para parar Gazebo y Xvfb."

# Esperar a que termine Gazebo
wait $GAZEBO_PID

echo "[INFO] Parando Xvfb..."
kill $XVFB_PID || true

