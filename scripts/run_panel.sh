#!/usr/bin/env bash
set -e

# Ruta al workspace
WS_DIR="$HOME/TFM/agarre_ros2_ws"

echo "[INFO] Cerrando procesos previos del panel / Gazebo / bridge (si existen)..."

# Matar panel Qt antiguo
pkill -f "ur5_qt_panel/main_panel.py" 2>/dev/null || true

# Matar Gazebo lanzado por nuestros scripts (gz sim ...)
pkill -f "gz sim" 2>/dev/null || true

# Matar bridge ros_gz_bridge
pkill -f "ros2 run ros_gz_bridge" 2>/dev/null || true

# (Opcional) matar scripts nuestros colgados
pkill -f "run_ur5_world.sh" 2>/dev/null || true
pkill -f "run_experiment_rgb.sh" 2>/dev/null || true
pkill -f "run_experiment_rgbd.sh" 2>/dev/null || true
pkill -f "eval_last_experiment.sh" 2>/dev/null || true

sleep 1

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[INFO] Cargando overlay del workspace..."
cd "$WS_DIR"
source install/setup.bash 2>/dev/null || true

echo "[INFO] Lanzando panel Qt (ur5_qt_panel/main_panel.py)..."
cd "$WS_DIR"
python3 src/ur5_qt_panel/ur5_qt_panel/main_panel.py
