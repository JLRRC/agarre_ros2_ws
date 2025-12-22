#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_close_gripper.sh
# Summary: Publishes a close command to the RG2 gripper controller.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source "$HOME/TFM/agarre_ros2_ws/install/setup.bash"
fi

TOPIC_J1="${TOPIC_J1:-/rg2_finger_joint1_cmd}"
TOPIC_J2="${TOPIC_J2:-/rg2_finger_joint2_cmd}"

# Cerrar gripper (valor grande)
ros2 topic pub --once "$TOPIC_J1" std_msgs/msg/Float64 "data: 0.8"
ros2 topic pub --once "$TOPIC_J2" std_msgs/msg/Float64 "data: 0.8"
