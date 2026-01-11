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

## Validación obligatoria post-bringup (profesional)
```bash
bash scripts/validate_panel_flow.sh
```

Variables útiles:
- `PANEL_COLD_BOOT=1` (limpia procesos previos antes de arrancar si usas el wrapper)
- `PANEL_GZ_GUI=1` (lanza Gazebo con GUI desde el wrapper)
- `PANEL_MANAGED=1` (panel guiado por `/system_state`, no lanza procesos críticos)
- `PANEL_MOVEIT_REQUIRED=1` (bloquea READY si MoveIt no está listo)
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
- El panel ya no genera YAML runtime del bridge (usa `scripts/bridge_cameras.yaml`).
- `/clock` y `/world/<world>/pose/info` los aporta el launch oficial.
- `/system_state` lo publica `ur5_tools/system_state_manager`.

## MoveIt (bringup independiente)
```bash
ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py start_ros2_control:=false launch_rviz:=false
```
