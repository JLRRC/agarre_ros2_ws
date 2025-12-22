<!-- URL: /home/laboratorio/TFM/agarre_ros2_ws/README.md -->
<!-- Summary: ROS 2 Jazzy workspace overview and build/run notes. -->
# agarre_ros2_ws

Workspace ROS 2 (Jazzy) para el trabajo de agarre inteligente.

## Build
```bash
cd ~/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Panel (modo PRO)
```bash
./scripts/run_panel_superpro.sh
```

## Gazebo (headless)
```bash
./scripts/run_ur5_world.sh
```

## Bridge ROS <-> Gazebo (YAML)
```bash
./scripts/run_gz_ros_bridge.sh
```
