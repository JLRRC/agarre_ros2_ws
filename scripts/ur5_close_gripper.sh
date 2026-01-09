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
# Cierre: 0.0 deja los dedos cerrados en Gazebo; invertir respecto a open.
GRIPPER_VALUE="${GRIPPER_VALUE:-0.0}"
VERIFY_TIMEOUT_S="${VERIFY_TIMEOUT_S:-2}"
ACTION_TIMEOUT_S="${ACTION_TIMEOUT_S:-3}"

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

log() {
  echo "[GRIPPER] $*"
}

ros2_topic_list() {
  if ! command -v ros2 >/dev/null 2>&1; then
    return 1
  fi
  ros2 topic list 2>/dev/null || true
}

ros2_action_list() {
  if ! command -v ros2 >/dev/null 2>&1; then
    return 1
  fi
  ros2 action list 2>/dev/null || true
}

has_ros_topic() {
  local topic="$1"
  if [[ -z "${ROS_TOPICS:-}" ]]; then
    return 1
  fi
  grep -Fxq "$topic" <<<"$ROS_TOPICS" || return 1
}

has_ros_action() {
  local action="$1"
  if [[ -z "${ROS_ACTIONS:-}" ]]; then
    return 1
  fi
  grep -Fxq "$action" <<<"$ROS_ACTIONS" || return 1
}

gz_topic_list() {
  if ! command -v gz >/dev/null 2>&1; then
    return 1
  fi
  if ! gazebo_running; then
    return 1
  fi
  timeout 2 gz topic -l 2>/dev/null || true
}

has_gz_topic() {
  local topic="$1"
  if [[ -z "${GZ_TOPICS:-}" ]]; then
    return 1
  fi
  grep -Fxq "$topic" <<<"$GZ_TOPICS" || return 1
}

ensure_gripper_controller() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "ros2 CLI no disponible para gripper_controller"
    return 1
  fi
  local ctrl_out
  ctrl_out="$(ros2 control list_controllers 2>/dev/null || true)"
  if echo "$ctrl_out" | grep -qE "^gripper_controller[[:space:]].*\\bactive\\b"; then
    return 0
  fi
  if echo "$ctrl_out" | grep -qE "^gripper_controller[[:space:]]"; then
    log "gripper_controller presente pero inactivo -> activando"
  else
    log "gripper_controller no listado -> intentando cargar/activar"
  fi
  ros2 control load_controller --set-state active gripper_controller >/dev/null 2>&1 || true
  ctrl_out="$(ros2 control list_controllers 2>/dev/null || true)"
  echo "$ctrl_out" | grep -qE "^gripper_controller[[:space:]].*\\bactive\\b"
}

verify_joint_states() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "verify: ros2 CLI no disponible"
    return 1
  fi
  local tmp_out
  tmp_out="$(mktemp)"
  if ! timeout "$VERIFY_TIMEOUT_S" ros2 topic echo --once /joint_states >"$tmp_out" 2>/dev/null; then
    rm -f "$tmp_out"
    log "verify: sin /joint_states (timeout ${VERIFY_TIMEOUT_S}s)"
    return 1
  fi
  python3 - "$tmp_out" <<'PY'
import re
import sys

in_path = sys.argv[1]
names = []
positions = []
section = None

for raw in open(in_path, "r", encoding="utf-8", errors="ignore"):
    line = raw.strip()
    if line == "name:":
        section = "name"
        continue
    if line == "position:":
        section = "position"
        continue
    if re.match(r"^[A-Za-z_]+:", line):
        section = None
        continue
    if line.startswith("- "):
        val = line[2:].strip()
        if section == "name":
            names.append(val)
        elif section == "position":
            positions.append(val)

mapping = dict(zip(names, positions))
j1 = mapping.get("rg2_finger_joint1")
j2 = mapping.get("rg2_finger_joint2")
if j1 is None or j2 is None:
    print("[GRIPPER] verify: joints RG2 no encontrados en /joint_states")
else:
    print(f"[GRIPPER] verify: rg2_finger_joint1={j1} rg2_finger_joint2={j2}")
PY
  rm -f "$tmp_out"
}

ROS_TOPICS="$(ros2_topic_list || true)"
ROS_ACTIONS="$(ros2_action_list || true)"
GZ_TOPICS="$(gz_topic_list || true)"

use_mode=""
if [[ "${FORCE_ROS2_CONTROL:-0}" == "1" ]]; then
  use_mode="ros2_control"
elif has_ros_topic "$TOPIC_J1" && has_ros_topic "$TOPIC_J2"; then
  use_mode="ros_topics"
elif has_ros_topic "$TOPIC_GRIPPER"; then
  use_mode="ros2_control"
elif has_ros_action "/gripper_controller/gripper_cmd"; then
  use_mode="ros2_action"
elif has_gz_topic "/rg2_finger_joint1_cmd" && has_gz_topic "/rg2_finger_joint2_cmd"; then
  use_mode="gz"
else
  use_mode="ros_topics"
fi

PUB_OPTS="--once --wait-matching-subscriptions 0"

case "$use_mode" in
  ros_topics)
    log "target: ROS topics $TOPIC_J1 $TOPIC_J2"
    log "cmd: ros2 topic pub std_msgs/msg/Float64 (x2)"
    ros2 topic pub $PUB_OPTS "$TOPIC_J1" std_msgs/msg/Float64 "data: ${GRIPPER_VALUE}" || true
    ros2 topic pub $PUB_OPTS "$TOPIC_J2" std_msgs/msg/Float64 "data: ${GRIPPER_VALUE}" || true
    ;;
  ros2_control)
    log "target: ROS2 control $TOPIC_GRIPPER"
    log "cmd: ros2 topic pub std_msgs/msg/Float64MultiArray"
    ensure_gripper_controller || log "gripper_controller no activo"
    ros2 topic pub $PUB_OPTS "$TOPIC_GRIPPER" std_msgs/msg/Float64MultiArray "{data: [${GRIPPER_VALUE}, ${GRIPPER_VALUE}]}" || true
    ;;
  ros2_action)
    log "target: ROS2 action /gripper_controller/gripper_cmd"
    log "cmd: ros2 action send_goal control_msgs/action/GripperCommand"
    timeout "$ACTION_TIMEOUT_S" ros2 action send_goal /gripper_controller/gripper_cmd \
      control_msgs/action/GripperCommand "{command: {position: ${GRIPPER_VALUE}, max_effort: 0.0}}" || true
    ;;
  gz)
    log "target: GZ topics /rg2_finger_joint1_cmd /rg2_finger_joint2_cmd"
    log "cmd: gz topic -m gz.msgs.Double (x2)"
    gz_pub_double "/rg2_finger_joint1_cmd" "${GRIPPER_VALUE}" || true
    gz_pub_double "/rg2_finger_joint2_cmd" "${GRIPPER_VALUE}" || true
    ;;
esac

sleep 0.2
verify_joint_states || true
