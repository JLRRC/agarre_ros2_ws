#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

# Abrir gripper (valor pequeño)
ros2 topic pub --once /gripper_controller/command std_msgs/msg/Float64MultiArray "
data: [0.0]
"
