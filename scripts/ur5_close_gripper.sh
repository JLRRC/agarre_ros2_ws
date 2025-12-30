#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_close_gripper.sh
# Summary: Publishes a close command to the RG2 gripper controller.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
PART_FILE="${GZ_PARTITION_FILE:-$WS_DIR/log/gz_partition.txt}"
if [ -z "${GZ_PARTITION:-}" ] && [ -f "$PART_FILE" ]; then
  GZ_PARTITION="$(cat "$PART_FILE")"
  export GZ_PARTITION
fi

TOPIC_J1="${TOPIC_J1:-/rg2_finger_joint1_cmd}"
TOPIC_J2="${TOPIC_J2:-/rg2_finger_joint2_cmd}"
TOPIC_GRIPPER="${TOPIC_GRIPPER:-/gripper_controller/commands}"

# Si Gazebo está activo, prioriza los topics puenteados (ROS->GZ).
gazebo_running() {
  pgrep -f "gz sim|gzserver" >/dev/null 2>&1
}

# Publish directly to Gazebo (bypass ROS bridge) if possible.
gz_pub_double() {
  local topic="$1"
  local value="$2"
  if ! command -v gz >/dev/null 2>&1; then
    return 1
  fi
  gz topic -t "$topic" -m gz.msgs.Double -p "data: ${value}" >/dev/null 2>&1
}

# Cerrar gripper (valor grande)
if [[ "${FORCE_ROS2_CONTROL:-0}" != "1" ]] && gazebo_running; then
  if gz_pub_double "/rg2_finger_joint1_cmd" "0.8" && gz_pub_double "/rg2_finger_joint2_cmd" "0.8"; then
    exit 0
  fi
  PUB_OPTS="--once --wait-matching-subscriptions 0"
  ros2 topic pub $PUB_OPTS "$TOPIC_J1" std_msgs/msg/Float64 "data: 0.8"
  ros2 topic pub $PUB_OPTS "$TOPIC_J2" std_msgs/msg/Float64 "data: 0.8"
  exit 0
fi

CTRL_OUT="$(ros2 control list_controllers 2>/dev/null || true)"
if echo "$CTRL_OUT" | grep -qE "^gripper_controller[[:space:]].*\\bactive\\b"; then
  PUB_OPTS="--once --wait-matching-subscriptions 0"
  ros2 topic pub $PUB_OPTS "$TOPIC_GRIPPER" std_msgs/msg/Float64MultiArray "{data: [0.8, 0.8]}"
else
  PUB_OPTS="--once --wait-matching-subscriptions 0"
  ros2 topic pub $PUB_OPTS "$TOPIC_J1" std_msgs/msg/Float64 "data: 0.8"
  ros2 topic pub $PUB_OPTS "$TOPIC_J2" std_msgs/msg/Float64 "data: 0.8"
fi
