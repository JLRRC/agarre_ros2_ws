#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/kill_all.sh
# Summary: Stops ROS/Gazebo related processes for a clean start.
# NOTE: uso manual de emergencia; no debe ser llamado automaticamente por el panel.
set -euo pipefail

echo "[KILL] kill_all.sh (SUPER PRO) — stopping everything..."

# Patrones típicos (incluye variantes gz-sim)
PATTERNS=(
  "ros2 bag record"
  "ros_gz_bridge"
  "parameter_bridge"
  "gz sim"
  "gz-sim"
  "gz-sim-server"
  "gz-sim-gui"
  "gzserver"
  "gzclient"
  "Xvfb"
  "ros2 launch ur5_bringup"
  "ur5_ros2_control.launch.py"
  "ros2_control_node"
  "robot_state_publisher"
  "world_tf_publisher"
  "controller_manager spawner"
  "spawner_joint"
  "spawner_gripper"
)

term_all() {
  for p in "${PATTERNS[@]}"; do
    pkill -TERM -f "$p" >/dev/null 2>&1 || true
  done
}

kill_all() {
  for p in "${PATTERNS[@]}"; do
    pkill -KILL -f "$p" >/dev/null 2>&1 || true
  done
}

any_running() {
  pgrep -af "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|Xvfb|ros2 launch ur5_bringup|ur5_ros2_control.launch.py|ros2_control_node|robot_state_publisher|world_tf_publisher|controller_manager spawner|spawner_joint|spawner_gripper" >/dev/null 2>&1
}

# 1) TERM
term_all

# 2) Espera corta
for _ in {1..30}; do
  if any_running; then
    sleep 0.15
  else
    break
  fi
done

# 3) KILL si queda algo
if any_running; then
  echo "[KILL] Algunos procesos siguen vivos. Forzando SIGKILL..."
  kill_all
fi

# 4) Última espera
for _ in {1..30}; do
  if any_running; then
    sleep 0.15
  else
    break
  fi
done

if any_running; then
  echo "[KILL][WARN] Aún quedan procesos. Lista:"
  pgrep -af "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|Xvfb" || true
else
  echo "[KILL] OK. Sistema limpio."
fi

