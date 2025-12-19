#!/usr/bin/env bash
set -e

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[INFO] Cargando overlay del workspace..."
source ~/TFM/agarre_ros2_ws/install/setup.bash

echo "[INFO] Lanzando Gazebo con el mundo UR5 + mesa + objetos..."
cd ~/TFM/agarre_ros2_ws
gz sim worlds/ur5_mesa_objetos.sdf --verbose
