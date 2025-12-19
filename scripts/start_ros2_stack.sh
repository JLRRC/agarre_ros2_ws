#!/usr/bin/env bash
set -e

echo "[INFO] Cargando entorno ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

if [ -f "/home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash" ]; then
  source /home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash
fi

cd /home/laboratorio/TFM/agarre_ros2_ws

echo "[INFO] Lanzando ros_gz_bridge para las cámaras y clock..."
# Aquí pon el comando real que uses para el bridge
# Ejemplo:
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /camera_overhead/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_north/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_south/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_east/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_west/image@sensor_msgs/msg/Image[gz.msgs.Image
