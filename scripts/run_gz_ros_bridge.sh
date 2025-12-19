#!/usr/bin/env bash
set -e

WS_DIR="$HOME/TFM/agarre_ros2_ws"

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[INFO] Cargando overlay del workspace: $WS_DIR"
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

echo "[INFO] Lanzando puente ros_gz_bridge para cámaras y clock..."

# Sintaxis: /TOPIC@ROS_TYPE[gz.MSGS_TYPE
# ROS: sensor_msgs/msg/Image
# GZ : gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
  "/camera_overhead/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/camera_north/image@sensor_msgs/msg/Image[gz.msgs.Image"    \
  "/camera_south/image@sensor_msgs/msg/Image[gz.msgs.Image"    \
  "/camera_east/image@sensor_msgs/msg/Image[gz.msgs.Image"     \
  "/camera_west/image@sensor_msgs/msg/Image[gz.msgs.Image"     \
  "/world/ur5_mesa_objetos/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"

