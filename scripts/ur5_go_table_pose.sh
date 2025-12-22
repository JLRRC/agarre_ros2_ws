#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_table_pose.sh
# Summary: Moves UR5 over the table pose (tunable via env vars).
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
ARM_TRAJ_TOPIC="${ARM_TRAJ_TOPIC:-/ur5_arm_joint_trajectory}"

# Pose sobre la mesa (ajusta si quieres otra postura)
TABLE_POS_0="${TABLE_POS_0:-0.0}"
TABLE_POS_1="${TABLE_POS_1:--1.2}"
TABLE_POS_2="${TABLE_POS_2:-1.3}"
TABLE_POS_3="${TABLE_POS_3:--0.5}"
TABLE_POS_4="${TABLE_POS_4:-0.0}"
TABLE_POS_5="${TABLE_POS_5:-0.0}"
TSEC="${TSEC:-3}"
TABLE_TIMEOUT="${TABLE_TIMEOUT:-10}"

# Source entorno (robusto)
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
set -u

echo "[ROBOT] Enviando MESA -> ${ARM_TRAJ_TOPIC} (t=${TSEC}s)"
if ! timeout "$TABLE_TIMEOUT" ros2 topic pub --once "$ARM_TRAJ_TOPIC" trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
  points: [
    { positions: [$TABLE_POS_0,$TABLE_POS_1,$TABLE_POS_2,$TABLE_POS_3,$TABLE_POS_4,$TABLE_POS_5],
      time_from_start: {sec: $TSEC, nanosec: 0}
    }
  ]
}" >/dev/null 2>&1; then
  echo "[ROBOT] WARN: MESA no se publicó en ${TABLE_TIMEOUT}s."
  exit 0
fi
