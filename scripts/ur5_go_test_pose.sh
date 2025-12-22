#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_test_pose.sh
# Summary: Publishes a test joint trajectory for UR5.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

# Posición de prueba (por ejemplo brazo algo extendido sobre la mesa)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
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
    sec: 3
    nanosec: 0
"
