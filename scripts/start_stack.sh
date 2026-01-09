# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts/start_stack.sh
# Nombre: start_stack.sh
# Qué hace: Inicia el stack completo (Gazebo headless + bridge + panel) en background.

#!/usr/bin/env bash
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
SCRIPTS_DIR="$WS_DIR/scripts"
LOG_DIR="$WS_DIR/log"
export WS_DIR
export SCRIPTS_DIR

mkdir -p "$LOG_DIR"

echo "[STACK] Limpiando procesos previos..."
"$SCRIPTS_DIR/kill_all.sh" >/dev/null 2>&1 || true

echo "[STACK] Lanzando Gazebo headless (Xvfb)..."
WS_DIR="$WS_DIR" "$SCRIPTS_DIR/start_gazebo_headless_xvfb.sh" >/tmp/start_gazebo.log 2>&1 &
GAZEBO_PID=$!
sleep 3

echo "[STACK] Lanzando ros_gz_bridge..."
WS_DIR="$WS_DIR" "$SCRIPTS_DIR/run_gz_ros_bridge.sh" >/tmp/ros_gz_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 2

echo "[STACK] Lanzando robot_state_publisher (TF)..."
ros2 launch ur5_bringup ur5_rsp.launch.py >/tmp/ur5_rsp.log 2>&1 &
RSP_PID=$!
sleep 1

echo "[STACK] Lanzando panel SUPER PRO..."
PANEL_COLD_BOOT=0 exec "$SCRIPTS_DIR/run_panel.sh"
