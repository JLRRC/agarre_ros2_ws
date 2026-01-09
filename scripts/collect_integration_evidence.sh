#!/usr/bin/env bash
set -eEo pipefail

# -------------------------
# Config por defecto
# -------------------------
WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
OBJ_X="${1:-0.50}"
OBJ_Y="${2:-0.00}"
OBJ_Z="${3:-0.20}"

RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

LAUNCH_PKG="ur5_bringup"
LAUNCH_FILE="view_ur5.launch.py"

TIMEOUT_SEC=20

# -------------------------
# Helpers
# -------------------------
log()  { echo -e "[SELFTEST] $*"; }
ok()   { echo -e "[OK] $*"; }
fail() { echo -e "[NO OK] $*"; exit 1; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Falta comando '$1' en PATH"
}

wait_until() {
  local desc="$1"; shift
  local end=$((SECONDS + TIMEOUT_SEC))
  while (( SECONDS < end )); do
    if eval "$*"; then
      ok "$desc"
      return 0
    fi
    sleep 0.5
  done
  fail "$desc (timeout ${TIMEOUT_SEC}s)"
}

cleanup() {
  set +e
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    log "Cerrando launch (pid=${LAUNCH_PID})..."
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -9 "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  pkill -9 -f "ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE}" >/dev/null 2>&1 || true
  pkill -9 -f "robot_state_publisher" >/dev/null 2>&1 || true
  pkill -9 -f "joint_state_publisher" >/dev/null 2>&1 || true
  pkill -9 -f "tf_probe" >/dev/null 2>&1 || true
  pkill -9 -f "tf2_echo" >/dev/null 2>&1 || true
  pkill -9 -f "ros2 topic hz /tf" >/dev/null 2>&1 || true
  set -e
}
trap cleanup EXIT

# -------------------------
# Pre-chequeos
# -------------------------
need_cmd ros2
need_cmd timeout
need_cmd grep
need_cmd awk

log "Matando procesos ROS viejos (evitar 'participant index')..."
pkill -9 -f "ros2" >/dev/null 2>&1 || true
pkill -9 -f "robot_state_publisher|joint_state_publisher|tf_probe|tf2_echo|gz sim|gzserver|ign gazebo|ros_gz_bridge|parameter_bridge" >/dev/null 2>&1 || true

log "Limpieza best-effort /dev/shm..."
rm -f /dev/shm/cdds_* /dev/shm/cyclonedds_* /dev/shm/ros_* /dev/shm/*fastrtps* 2>/dev/null || true

log "Sourcing ROS + workspace..."
# Evita crash por nounset con setup.bash
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u 2>/dev/null || true  # si quieres reactivar nounset, pero ya no rompe el source

export RMW_IMPLEMENTATION ROS_DOMAIN_ID

log "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "WS_DIR=${WS_DIR}"
log "OBJ=(${OBJ_X}, ${OBJ_Y}, ${OBJ_Z})"

# -------------------------
# Lanzar bringup
# -------------------------
log "Lanzando: ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE}"
ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" > /tmp/tf_selftest_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 1

# -------------------------
# Tests (OK / NO OK)
# -------------------------

wait_until "Nodo robot_state_publisher visible" \
  "ros2 node list | grep -q '^/robot_state_publisher$'"

wait_until "Topic /tf_static existe" \
  "ros2 topic info /tf_static >/dev/null 2>&1"

wait_until "Topic /tf existe" \
  "ros2 topic info /tf >/dev/null 2>&1"

# /tf_static debe tener publisher
wait_until "/tf_static tiene publisher" \
  "ros2 topic info /tf_static | grep -q 'Publisher count: [1-9]'"

# joint_state_publisher: o hay /joint_states o fallamos (sin joint_states no hay TF dinámico)
wait_until "Topic /joint_states existe" \
  "ros2 topic info /joint_states >/dev/null 2>&1"

wait_until "/joint_states publica al menos 1 msg" \
  "timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1"

# /tf dinámico: comprobar que sale algo
wait_until "/tf publica (al menos 1 msg)" \
  "timeout 3 ros2 topic echo /tf --once >/dev/null 2>&1"

# Ejecutar tf_probe y validar que:
# - tf_static > 0
# - tf > 0
# - ee != n/a
# - dist != n/a
log "Ejecutando tf_probe (6s)..."
PROBE_OUT="$(timeout 6 ros2 run ur5_tools tf_probe --ros-args -p obj_x:=${OBJ_X} -p obj_y:=${OBJ_Y} -p obj_z:=${OBJ_Z} 2>&1 || true)"
echo "$PROBE_OUT" | tail -n 30 > /tmp/tf_selftest_probe_tail.log

# Buscar última línea [PROBE]
LAST_LINE="$(echo "$PROBE_OUT" | grep '\[PROBE\]' | tail -n 1 || true)"
if [[ -z "$LAST_LINE" ]]; then
  echo "$PROBE_OUT" > /tmp/tf_selftest_probe_full.log
  fail "tf_probe no imprimió líneas [PROBE]. Mira /tmp/tf_selftest_probe_full.log"
fi

echo "[SELFTEST] tf_probe last: $LAST_LINE"

EE_VAL="$(echo "$LAST_LINE" | sed -n 's/.* ee=\([^ ]*\) .*/\1/p')"
TF_VAL="$(echo "$LAST_LINE" | sed -n 's/.* tf=\([0-9]\+\) .*/\1/p')"
TFS_VAL="$(echo "$LAST_LINE" | sed -n 's/.* tf_static=\([0-9]\+\).*/\1/p')"
DIST_VAL="$(echo "$LAST_LINE" | sed -n 's/.* dist=\([^ ]*\) .*/\1/p')"

# Normalizar vacíos
EE_VAL="${EE_VAL:-n/a}"
TF_VAL="${TF_VAL:-0}"
TFS_VAL="${TFS_VAL:-0}"
DIST_VAL="${DIST_VAL:-n/a}"

if [[ "$TFS_VAL" == "0" ]]; then
  fail "TF estático = 0 → robot_state_publisher no está publicando /tf_static (o QoS mal)."
fi

if [[ "$TF_VAL" == "0" ]]; then
  fail "TF dinámico = 0 → NO hay /joint_states efectivo o robot_state_publisher no publica dinámico."
fi

if [[ "$EE_VAL" == "n/a" ]]; then
  fail "EE = n/a → TF tree no contiene tool0/flange/etc o no transformable desde base."
fi

if [[ "$DIST_VAL" == "n/a" ]]; then
  fail "dist = n/a → no se pudo calcular TCP vs OBJ (faltan transforms)."
fi

ok "tf_probe OK: tf_static>0, tf>0, ee!=n/a, dist numérica"

log "Resumen logs:"
log "  launch: /tmp/tf_selftest_launch.log"
log "  probe_tail: /tmp/tf_selftest_probe_tail.log"

ok "SELFTEST COMPLETADO ✅"
exit 0