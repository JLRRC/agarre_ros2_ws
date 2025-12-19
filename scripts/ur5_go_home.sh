#!/usr/bin/env bash
set -e

# Cargar entorno ROS 2 + workspace
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

# Trayectoria conjunta a HOME (ajusta posiciones si quieres otro home)
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory ... "
joint_names:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
points:
- positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
  time_from_start:
    sec: 3
    nanosec: 0
"
