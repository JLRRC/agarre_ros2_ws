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

## Dependencias ROS 2 (TF2 Python)

Para evitar errores `TypeException` en transformaciones TF2 (PoseStamped/PointStamped), instala el paquete:

```bash
sudo apt install ros-jazzy-tf2-geometry-msgs
```

## Bringup oficial (único)
```bash
ros2 launch ur5_bringup ur5_stack.launch.py
```

Variables útiles:
- `PANEL_COLD_BOOT=1` (limpia procesos previos antes de arrancar si usas el wrapper)
- `PANEL_GZ_GUI=1` (lanza Gazebo con GUI desde el wrapper)
- `PANEL_AUTO_BRIDGE=1` (permite que el panel lance el bridge)
- `PANEL_START_STACK=0` (modo panel-only desde el wrapper)

Argumentos útiles del launch:
- `headless:=true|false`
- `launch_panel:=true|false`
- `launch_gazebo:=true|false`
- `launch_rsp:=true|false`
- `launch_bridge:=true|false`
- `launch_ros2_control:=true|false`
- `launch_moveit:=true|false`

## Debug manual (solo si hace falta)
```bash
./scripts/run_ur5_world.sh
./scripts/run_gz_ros_bridge.sh
```

## MoveIt (bringup independiente)
```bash
ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py start_ros2_control:=false launch_rviz:=false
```
