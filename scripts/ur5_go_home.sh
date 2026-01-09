#!/usr/bin/env bash
# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_go_home.sh
# Nombre: ur5_go_home.sh
# Qué hace: Envía una trayectoria "HOME" al joint_trajectory_controller del UR5.
#           Valida controller_manager y controladores; activa controladores si hace falta.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
CM="/controller_manager"
CTRL_JSB="joint_state_broadcaster"
CTRL_TRAJ="joint_trajectory_controller"

# HOME (ajusta si quieres otra postura)
HOME_POS_0="${HOME_POS_0:-0.0}"
HOME_POS_1="${HOME_POS_1:--1.57}"
HOME_POS_2="${HOME_POS_2:-1.57}"
HOME_POS_3="${HOME_POS_3:--1.57}"
HOME_POS_4="${HOME_POS_4:--1.57}"
HOME_POS_5="${HOME_POS_5:-0.0}"
TSEC="${TSEC:-3}"

# Source entorno (robusto)
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
set -u

# 1) controller_manager vivo (servicios)
if ! ros2 service list 2>/dev/null | grep -q "^${CM}/list_controllers$"; then
  echo "[ROBOT] ERROR: No está disponible ${CM}/list_controllers (controller_manager no accesible)."
  exit 1
fi

# 2) Asegura controladores (si no están activos, intenta activarlos)
LC="$(ros2 control list_controllers 2>/dev/null || true)"

if ! echo "$LC" | grep -qE "^${CTRL_JSB}[[:space:]]+joint_state_broadcaster/JointStateBroadcaster"; then
  echo "[ROBOT] INFO: Cargando ${CTRL_JSB}..."
  ros2 run controller_manager spawner "$CTRL_JSB" -c "$CM" --controller-manager-timeout 30 --switch-timeout 30 || true
fi

if ! echo "$LC" | grep -qE "^${CTRL_TRAJ}[[:space:]]+joint_trajectory_controller/JointTrajectoryController"; then
  echo "[ROBOT] INFO: Cargando ${CTRL_TRAJ}..."
  ros2 run controller_manager spawner "$CTRL_TRAJ" -c "$CM" --controller-manager-timeout 30 --switch-timeout 30 || true
fi

# Relee estado
LC2="$(ros2 control list_controllers 2>/dev/null || true)"
echo "$LC2" | sed -n '1,80p'

if ! echo "$LC2" | grep -qE "^${CTRL_JSB}.*\bactive\b"; then
  echo "[ROBOT] WARN: ${CTRL_JSB} no está ACTIVE. Intento activar con spawner..."
  ros2 run controller_manager spawner "$CTRL_JSB" -c "$CM" --activate --controller-manager-timeout 30 --switch-timeout 30 || true
fi

if ! echo "$LC2" | grep -qE "^${CTRL_TRAJ}.*\bactive\b"; then
  echo "[ROBOT] WARN: ${CTRL_TRAJ} no está ACTIVE. Intento activar con spawner..."
  ros2 run controller_manager spawner "$CTRL_TRAJ" -c "$CM" --activate --controller-manager-timeout 30 --switch-timeout 30 || true
fi

# 3) Espera a action server
ACTION="/${CTRL_TRAJ}/follow_joint_trajectory"
echo "[ROBOT] Esperando action server: $ACTION"
for i in {1..50}; do
  if ros2 action list 2>/dev/null | grep -q "^${ACTION}$"; then
    break
  fi
  sleep 0.1
done

if ! ros2 action list 2>/dev/null | grep -q "^${ACTION}$"; then
  echo "[ROBOT] ERROR: No aparece el action server $ACTION"
  exit 1
fi

# 4) Enviar objetivo HOME
echo "[ROBOT] Enviando HOME -> ${CTRL_TRAJ} (t=${TSEC}s)"
ros2 action send_goal "$ACTION" control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    points: [
      { positions: [$HOME_POS_0,$HOME_POS_1,$HOME_POS_2,$HOME_POS_3,$HOME_POS_4,$HOME_POS_5],
        time_from_start: {sec: $TSEC, nanosec: 0}
      }
    ]
  }
}" --feedback
