#!/usr/bin/env bash
set -euo pipefail

log() { echo "[VALIDATE] $*"; }
warn() { echo "[VALIDATE][WARN] $*" >&2; }

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ -n "${WS_DIR:-}" && -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
elif [[ -f "$(pwd)/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$(pwd)/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  warn "ros2 no está disponible en PATH."
  exit 1
fi

if command -v rg >/dev/null 2>&1; then
  filter() { rg -n "$1"; }
else
  filter() { grep -nE "$1"; }
fi

log "Comprobando /clock..."
if ros2 topic list | filter "^/clock$" >/dev/null 2>&1; then
  if timeout 2.0 ros2 topic echo --once /clock >/dev/null 2>&1; then
    log "OK /clock publica."
  else
    warn "Existe /clock pero no publica (timeout)."
  fi
else
  warn "/clock no existe."
fi

log "Comprobando /world/*/pose/info..."
pose_topics="$(ros2 topic list | filter "^/world/.*/pose/info$" || true)"
if [[ -n "${pose_topics}" ]]; then
  log "OK pose/info detectado:"
  echo "${pose_topics}" | sed "s/^/[VALIDATE]   /"
else
  warn "No se detecta /world/*/pose/info."
fi

log "Comprobando nodos clave..."
nodes="$(ros2 node list 2>/dev/null || true)"
echo "${nodes}" | filter "parameter_bridge|ros_gz_bridge" >/dev/null 2>&1 \
  && log "OK bridge node detectado." \
  || warn "No se detecta node de bridge."

echo "${nodes}" | filter "world_tf_publisher" >/dev/null 2>&1 \
  && log "OK world_tf_publisher detectado." \
  || warn "No se detecta world_tf_publisher."

echo "${nodes}" | filter "system_state_manager" >/dev/null 2>&1 \
  && log "OK system_state_manager detectado." \
  || warn "No se detecta system_state_manager."

log "Comprobando TF world->base_link..."
if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  if timeout 2.0 ros2 run tf2_ros tf2_echo world base_link >/dev/null 2>&1; then
    log "OK TF world->base_link."
  else
    warn "TF world->base_link no disponible (timeout)."
  fi
else
  warn "No se pudo validar TF world->base_link (falta timeout o ros2)."
fi

log "Comprobando TF base_link->tool0..."
if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
  if timeout 2.0 ros2 run tf2_ros tf2_echo base_link tool0 >/dev/null 2>&1; then
    log "OK TF base_link->tool0."
  else
    warn "TF base_link->tool0 no disponible (timeout)."
  fi
else
  warn "No se pudo validar TF base_link->tool0 (falta timeout o ros2)."
fi

log "Comprobando controller_manager..."
if ros2 service list | filter "/controller_manager/list_controllers$" >/dev/null 2>&1; then
  if command -v ros2 >/dev/null 2>&1 && command -v ros2control >/dev/null 2>&1; then
    if ros2 control list_controllers >/dev/null 2>&1; then
      log "OK ros2 control list_controllers."
    else
      warn "ros2 control list_controllers fallo."
    fi
  else
    log "Servicio controller_manager detectado."
  fi
else
  warn "No se detecta /controller_manager/list_controllers."
fi

log "Comprobando /system_state..."
if ros2 topic list | filter "^/system_state$" >/dev/null 2>&1; then
  if timeout 2.0 ros2 topic echo --once /system_state >/dev/null 2>&1; then
    log "OK /system_state publica."
  else
    warn "/system_state existe pero no publica (timeout)."
  fi
else
  warn "/system_state no existe."
fi

log "Comprobando /system_diag..."
if ros2 topic list | filter "^/system_diag$" >/dev/null 2>&1; then
  if timeout 2.0 ros2 topic echo --once /system_diag >/dev/null 2>&1; then
    log "OK /system_diag publica."
  else
    warn "/system_diag existe pero no publica (timeout)."
  fi
else
  warn "/system_diag no existe."
fi

log "Comprobando TF con tf_probe..."
if command -v ros2 >/dev/null 2>&1; then
  if timeout 6.0 ros2 run ur5_tools tf_probe --ros-args -p use_sim_time:=true >/dev/null 2>&1; then
    log "OK tf_probe (world->base_link, base_link->tool0)."
  else
    warn "tf_probe reporta TF inestable o faltante."
  fi
else
  warn "No se pudo ejecutar tf_probe (ros2 no disponible)."
fi

log "Comprobando JointTrajectory con jt_smoke_test..."
if command -v ros2 >/dev/null 2>&1; then
  if timeout 8.0 ros2 run ur5_tools jt_smoke_test --ros-args -p use_sim_time:=true >/dev/null 2>&1; then
    log "OK jt_smoke_test (joint_states cambian)."
  else
    warn "jt_smoke_test falló (no hay movimiento real)."
  fi
else
  warn "No se pudo ejecutar jt_smoke_test (ros2 no disponible)."
fi

log "Validación básica completada."
