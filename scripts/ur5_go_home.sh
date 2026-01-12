#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_home.sh
# Summary: Moves UR5 to HOME pose with controller checks.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
ARM_TRAJ_TOPIC="${ARM_TRAJ_TOPIC:-/ur5_arm_joint_trajectory}"
HOME_ENV="${HOME_ENV:-$WS_DIR/scripts/ur5_home_pose.env}"

# HOME (ajusta si quieres otra postura)
HOME_POS_0="${HOME_POS_0:-0.0}"
HOME_POS_1="${HOME_POS_1:--1.57}"
HOME_POS_2="${HOME_POS_2:-1.57}"
HOME_POS_3="${HOME_POS_3:--1.57}"
HOME_POS_4="${HOME_POS_4:--1.57}"
HOME_POS_5="${HOME_POS_5:-0.0}"
if [[ -f "$HOME_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$HOME_ENV"
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
