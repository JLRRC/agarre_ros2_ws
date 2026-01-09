#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_test_pose.sh
# Summary: Publishes a test joint trajectory for UR5.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

ARM_TRAJ_TOPIC="${ARM_TRAJ_TOPIC:-/joint_trajectory_controller/joint_trajectory}"
TSEC="${TSEC:-3}"

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

# Posición de prueba (brazo algo extendido sobre la mesa)
ros2 topic pub --once "$ARM_TRAJ_TOPIC" trajectory_msgs/msg/JointTrajectory "
joint_names:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
points:
- positions: [0.0, -1.2, 1.3, -0.5, 0.0, 0.0]
  time_from_start:
    sec: $TSEC
    nanosec: 0
"
