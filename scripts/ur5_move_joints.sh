#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_move_joints.sh
# Summary: Mueve UR5 a posiciones articulares específicas (6 valores).
# Uso: ur5_move_joints.sh <j0> <j1> <j2> <j3> <j4> <j5>
# Nota: dejamos nounset desactivado para no romper los setup.* de ROS (usa AMENT_* internamente).
set -Eeo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
ARM_TRAJ_TOPIC="${ARM_TRAJ_TOPIC:-/joint_trajectory_controller/joint_trajectory}"

if [[ $# -ne 6 ]]; then
  echo "ERROR: Se requieren 6 posiciones articulares" >&2
  echo "Uso: $0 <j0> <j1> <j2> <j3> <j4> <j5>" >&2
  exit 1
fi

J0="$1"
J1="$2"
J2="$3"
J3="$4"
J4="$5"
J5="$6"

# Source ROS 2 (sin nounset para evitar warnings de AMENT_PYTHON_EXECUTABLE)
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

# Publicar trayectoria
ros2 topic pub --once "$ARM_TRAJ_TOPIC" trajectory_msgs/msg/JointTrajectory "{
  joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
  points: [
    {
      positions: [$J0, $J1, $J2, $J3, $J4, $J5],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}" >/dev/null 2>&1
