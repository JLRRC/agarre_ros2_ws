#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_home.sh
# Summary: Moves UR5 to HOME pose with controller checks.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
ARM_TRAJ_TOPIC="${ARM_TRAJ_TOPIC:-/ur5_arm_joint_trajectory}"
HOME_ENV="${HOME_ENV:-$WS_DIR/scripts/ur5_home_pose.env}"

# HOME (ajusta si quieres otra postura)
DEFAULT_HOME_POS_0="0.054"
DEFAULT_HOME_POS_1="0.028"
DEFAULT_HOME_POS_2="0.016"
DEFAULT_HOME_POS_3="0.016"
DEFAULT_HOME_POS_4="0.028"
DEFAULT_HOME_POS_5="0.016"
HOME_POS_0="${HOME_POS_0:-$DEFAULT_HOME_POS_0}"
HOME_POS_1="${HOME_POS_1:-$DEFAULT_HOME_POS_1}"
HOME_POS_2="${HOME_POS_2:-$DEFAULT_HOME_POS_2}"
HOME_POS_3="${HOME_POS_3:-$DEFAULT_HOME_POS_3}"
HOME_POS_4="${HOME_POS_4:-$DEFAULT_HOME_POS_4}"
HOME_POS_5="${HOME_POS_5:-$DEFAULT_HOME_POS_5}"
if [[ -f "$HOME_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$HOME_ENV"
fi

# Si el HOME guardado es todo ceros, usa los defaults.
all_zero=1
for v in "$HOME_POS_0" "$HOME_POS_1" "$HOME_POS_2" "$HOME_POS_3" "$HOME_POS_4" "$HOME_POS_5"; do
  if awk "BEGIN {exit !(($v < -0.001) || ($v > 0.001))}"; then
    all_zero=0
  fi
done
if [[ "$all_zero" == "1" ]]; then
  echo "[ROBOT] WARN: HOME guardado es todo 0. Uso defaults."
  HOME_POS_0="$DEFAULT_HOME_POS_0"
  HOME_POS_1="$DEFAULT_HOME_POS_1"
  HOME_POS_2="$DEFAULT_HOME_POS_2"
  HOME_POS_3="$DEFAULT_HOME_POS_3"
  HOME_POS_4="$DEFAULT_HOME_POS_4"
  HOME_POS_5="$DEFAULT_HOME_POS_5"
fi
TSEC="${TSEC:-3}"
HOME_QUIET="${HOME_QUIET:-1}"
HOME_TIMEOUT="${HOME_TIMEOUT:-10}"

# Source entorno (robusto)
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
set -u

# Si Gazebo está activo, prioriza el topic puenteado (ROS->GZ).
gazebo_running() {
  pgrep -f "gz sim|gzserver" >/dev/null 2>&1
}

# Si ros2_control está activo, usa el topic del JointTrajectoryController.
detect_arm_topic() {
  local out
  if [[ "${FORCE_ROS2_CONTROL:-0}" != "1" ]] && gazebo_running; then
    echo "$ARM_TRAJ_TOPIC"
    return
  fi
  out="$(ros2 control list_controllers 2>/dev/null || true)"
  if echo "$out" | grep -qE "^joint_trajectory_controller[[:space:]]"; then
    if echo "$out" | grep -qE "^joint_trajectory_controller[[:space:]].*\\bactive\\b"; then
      echo "/joint_trajectory_controller/joint_trajectory"
      return
    fi
  fi
  echo "$ARM_TRAJ_TOPIC"
}
ARM_TRAJ_TOPIC="$(detect_arm_topic)"

# 1) Publica trayectoria al controlador de Gazebo (ROS->GZ bridge)
echo "[ROBOT] Enviando HOME -> ${ARM_TRAJ_TOPIC} (t=${TSEC}s)"
tmp_out="$(mktemp)"
if ! timeout "$HOME_TIMEOUT" ros2 topic pub --once "$ARM_TRAJ_TOPIC" trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
  points: [
    { positions: [$HOME_POS_0,$HOME_POS_1,$HOME_POS_2,$HOME_POS_3,$HOME_POS_4,$HOME_POS_5],
      time_from_start: {sec: $TSEC, nanosec: 0}
    }
  ]
}" >"$tmp_out" 2>&1; then
  rm -f "$tmp_out"
  echo "[ROBOT] WARN: HOME no se publicó en ${HOME_TIMEOUT}s."
  exit 0
fi
if [[ "$HOME_QUIET" != "1" ]]; then
  cat "$tmp_out" || true
fi
rm -f "$tmp_out"
