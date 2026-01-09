#!/usr/bin/env bash
set -euo pipefail

OBJ_X="${1:-0.50}"
OBJ_Y="${2:-0.00}"
OBJ_Z="${3:-0.20}"

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
LOG_FILE="/tmp/tf_selftest_launch.log"
DIAG_FILE="/tmp/tf_selftest_diag.txt"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_HOME="${ROS_HOME:-${WS_DIR}/log/ros}"
mkdir -p "${ROS_HOME}"
export ROS_HOME
ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
mkdir -p "${ROS_LOG_DIR}"

log() { echo "[SELFTEST] $*"; }

cleanup() {
  log "Cerrando bringup (best-effort)..."
  pkill -INT -f "ros2 launch ur5_bringup view_ur5.launch.py" >/dev/null 2>&1 || true
  sleep 1
  pkill -TERM -f "ros2 launch ur5_bringup view_ur5.launch.py" >/dev/null 2>&1 || true
  pkill -f robot_state_publisher >/dev/null 2>&1 || true
  pkill -f joint_state_publisher >/dev/null 2>&1 || true
  pkill -f robot_description_pub >/dev/null 2>&1 || true
  pkill -f tf_probe >/dev/null 2>&1 || true
}
trap cleanup EXIT

wait_for() {
  local timeout_sec="$1"; shift
  local cmd="$*"
  local start; start="$(date +%s)"
  while true; do
    if bash -lc "$cmd" >/dev/null 2>&1; then return 0; fi
    local now; now="$(date +%s)"
    if (( now - start >= timeout_sec )); then return 1; fi
    sleep 0.5
  done
}

run_timeout() {
  local sec="$1"; shift
  rm -f /tmp/_st_out.txt /tmp/_st_err.txt
  timeout "$sec" bash -lc "$*" >/tmp/_st_out.txt 2>/tmp/_st_err.txt
}

diag_dump() {
  {
    echo "================= TF SELFTEST DIAG ================="
    echo "date: $(date -Is)"
    echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "LOG_FILE=$LOG_FILE"
    echo

    echo "---- ros2 node list ----"
    ros2 node list || true
    echo

    echo "---- ros2 topic list (filtered) ----"
    ros2 topic list | grep -E '^/robot_description$|^/joint_states$|^/tf$|^/tf_static$' || true
    echo

    echo "---- ros2 topic info -v /robot_description ----"
    ros2 topic info -v /robot_description || true
    echo

    echo "---- ros2 topic info -v /joint_states ----"
    ros2 topic info -v /joint_states || true
    echo

    echo "---- ros2 node info /joint_state_publisher ----"
    ros2 node info /joint_state_publisher || true
    echo

    echo "---- ros2 node info /robot_state_publisher ----"
    ros2 node info /robot_state_publisher || true
    echo

    echo "---- joint_state_publisher param dump (subset) ----"
    ros2 param list /joint_state_publisher 2>/dev/null | egrep 'robot_description|use_robot|source_list|rate' || true
    echo
    ros2 param get /joint_state_publisher use_robot_description_topic 2>/dev/null || true
    echo

    echo "---- /robot_description sample (head) ----"
    timeout 3 ros2 topic echo /robot_description --once 2>/dev/null | head -n 8 || true
    echo

    echo "---- /joint_states sample (once) ----"
    timeout 3 ros2 topic echo /joint_states --once 2>/dev/null || true
    echo

    echo "---- /tf sample (once) ----"
    timeout 3 ros2 topic echo /tf --once 2>/dev/null | head -n 30 || true
    echo

    echo "---- tail launch log ($LOG_FILE) ----"
    tail -n 60 "$LOG_FILE" || true
    echo "===================================================="
  } | tee "$DIAG_FILE" >/dev/null

  echo "[INFO] Diagnóstico guardado en: $DIAG_FILE"
}

log "WS_DIR=$WS_DIR"
log "OBJ=(${OBJ_X}, ${OBJ_Y}, ${OBJ_Z})"
log "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
log "Log: $LOG_FILE"
log "Diag: $DIAG_FILE"

log "Matando procesos ROS viejos (best-effort)..."
cleanup || true

log "Sourcing ROS + workspace (sin nounset durante source)..."
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u

log "Lanzando bringup en background..."
rm -f "$LOG_FILE" "$DIAG_FILE"
( ros2 launch ur5_bringup view_ur5.launch.py >"$LOG_FILE" 2>&1 ) &
BRINGUP_PID="$!"
log "bringup pid=$BRINGUP_PID"

log "Esperando /robot_state_publisher (20s)..."
if wait_for 20 "ros2 node list | grep -qx '/robot_state_publisher'"; then
  echo "[OK] robot_state_publisher visible"
else
  echo "[NO OK] robot_state_publisher no aparece. Mira $LOG_FILE"
  diag_dump
  exit 1
fi

log "Chequeando robot_description por PARAM (10s)..."
if run_timeout 10 "ros2 param get /robot_state_publisher robot_description | head -n 5"; then
  if grep -q "<robot" /tmp/_st_out.txt; then
    echo "[OK] robot_description param contiene <robot ...>"
  else
    echo "[NO OK] robot_description param no parece URDF. Salida:"
    cat /tmp/_st_out.txt
    diag_dump
    exit 1
  fi
else
  echo "[NO OK] No pude leer robot_description param. Mira $LOG_FILE"
  diag_dump
  exit 1
fi

log "Chequeando /tf_static (TRANSIENT_LOCAL) (8s)..."
if run_timeout 8 "ros2 topic echo /tf_static --once --qos-durability transient_local --qos-reliability reliable"; then
  if grep -q "transforms:" /tmp/_st_out.txt; then
    echo "[OK] /tf_static entrega TFs"
  else
    echo "[NO OK] /tf_static recibido pero vacío/inesperado:"
    head -n 40 /tmp/_st_out.txt
    diag_dump
    exit 1
  fi
else
  echo "[NO OK] /tf_static no se recibió en 8s. Mira $LOG_FILE"
  diag_dump
  exit 1
fi

log "Chequeando /joint_states (10s)..."
if run_timeout 10 "ros2 topic echo /joint_states --once"; then
  echo "[OK] /joint_states emite"
else
  echo "[NO OK] /joint_states NO emite en 10s -> NO habrá /tf dinámico (TCP n/a)."
  diag_dump
  exit 1
fi

log "Chequeando /tf dinámico (10s)..."
if run_timeout 10 "ros2 topic echo /tf --once"; then
  if grep -q "transforms:" /tmp/_st_out.txt; then
    echo "[OK] /tf emite TF dinámico"
  else
    echo "[NO OK] /tf recibido pero sin transforms:"
    head -n 40 /tmp/_st_out.txt
    diag_dump
    exit 1
  fi
else
  echo "[NO OK] /tf NO emite en 10s. Mira $LOG_FILE"
  diag_dump
  exit 1
fi

log "Ejecutando tf_probe (12s)..."
set +e
timeout 12 ros2 run ur5_tools tf_probe --ros-args \
  -p obj_x:="$OBJ_X" -p obj_y:="$OBJ_Y" -p obj_z:="$OBJ_Z" \
  >/tmp/_tf_probe_run.txt 2>&1
set -e

if grep -q "\[PROBE\]" /tmp/_tf_probe_run.txt; then
  if grep -q "ee=n/a" /tmp/_tf_probe_run.txt; then
    echo "[NO OK] tf_probe sigue con ee=n/a (no hay TCP). Última línea:"
    grep "\[PROBE\]" /tmp/_tf_probe_run.txt | tail -n 1
    diag_dump
    exit 1
  fi
  if grep -E "\[PROBE\].*dist=[0-9]+\.[0-9]+" /tmp/_tf_probe_run.txt >/dev/null 2>&1; then
    echo "[OK] tf_probe: EE detectado y dist numérica"
    echo "----- última línea PROBE -----"
    grep "\[PROBE\]" /tmp/_tf_probe_run.txt | tail -n 1
    echo "------------------------------"
  else
    echo "[NO OK] tf_probe: EE detectado pero dist no es numérica. Última línea:"
    grep "\[PROBE\]" /tmp/_tf_probe_run.txt | tail -n 1
    diag_dump
    exit 1
  fi
else
  echo "[NO OK] tf_probe no imprimió nada. Mira /tmp/_tf_probe_run.txt"
  diag_dump
  exit 1
fi

echo "[OK] SELFTEST COMPLETO: robot_description + tf_static + joint_states + tf + tf_probe OK"
exit 0
