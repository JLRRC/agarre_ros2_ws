#!/usr/bin/env bash
set -euo pipefail

# =========================
# start_panel_v2.sh
# Workspace: ~/TFM/agarre_ros2_ws
# =========================

# --- helpers ---
log() { echo "[START_PANEL_V2] $*"; }
err() { echo "[START_PANEL_V2] ERROR: $*" >&2; }

# Detectar WS_DIR (raíz del repo)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export WS_DIR

# Config (puedes exportar estas vars antes de lanzar)
: "${ROS_DISTRO:=jazzy}"
: "${PANEL_COLD_BOOT:=1}"          # 1 = mata procesos antes de arrancar
: "${PANEL_V2_PREFER_INSTALLED:=1}"# 1 = usa install/.../panel_v2 si existe
: "${PANEL_MODE:=manual}"          # manual=panel lanza Gazebo/bridge/MoveIt | auto=stack completo
: "${PANEL_START_STACK:=0}"        # 1 = autoarranca RSP+Gazebo
: "${PANEL_MANAGED:=0}"            # 1 = panel guiado por /system_state
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION

log "WS_DIR=$WS_DIR"

# --- cold boot: limpieza de procesos ---
if [[ "$PANEL_COLD_BOOT" == "1" ]]; then
  log "cold boot: matando procesos previos..."
  # Limpieza FastDDS (opcional pero útil)
  if [[ -d /dev/shm ]]; then
    rm -rf /dev/shm/fastdds* /dev/shm/ros* 2>/dev/null || true
  fi

  # Procesos típicos del stack
  pkill -f "gz sim"            2>/dev/null || true
  pkill -f "gzserver"          2>/dev/null || true
  pkill -f "gzclient"          2>/dev/null || true
  pkill -f "ign gazebo"        2>/dev/null || true
  pkill -f "ros_gz_bridge"     2>/dev/null || true
  pkill -f "parameter_bridge"  2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "ros2_control_node" 2>/dev/null || true
  pkill -f "controller_manager" 2>/dev/null || true
  pkill -f "spawner" 2>/dev/null || true
  pkill -f "release_objects_service" 2>/dev/null || true
  pkill -f "system_state_manager" 2>/dev/null || true
  pkill -f "ros2 launch ur5_bringup" 2>/dev/null || true

  # Paneles previos (ajusta patrones si lo necesitas)
  pkill -f "ur5_qt_panel"      2>/dev/null || true
  pkill -f "panel_v2.py"       2>/dev/null || true
  pkill -f "main_panel.py"     2>/dev/null || true
  pkill -f "ros2 run ur5_qt_panel panel_v2" 2>/dev/null || true

  any_running() {
    pgrep -af "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|ign gazebo|ros2 launch ur5_bringup|ros2_control_node|robot_state_publisher|world_tf_publisher|controller_manager|spawner|move_group|ur5_qt_panel|panel_v2.py|main_panel.py" >/dev/null 2>&1
  }

  # Verificación: si quedan procesos, intentar cierre forzado.
  for _ in {1..20}; do
    if any_running; then
      sleep 0.2
    else
      break
    fi
  done
  if any_running; then
    log "cold boot: procesos aún activos, forzando cierre..."
    pkill -KILL -f "ros2 bag record" >/dev/null 2>&1 || true
    pkill -KILL -f "ros_gz_bridge" >/dev/null 2>&1 || true
    pkill -KILL -f "parameter_bridge" >/dev/null 2>&1 || true
    pkill -KILL -f "gz sim" >/dev/null 2>&1 || true
    pkill -KILL -f "gz-sim" >/dev/null 2>&1 || true
    pkill -KILL -f "gzserver" >/dev/null 2>&1 || true
    pkill -KILL -f "gzclient" >/dev/null 2>&1 || true
    pkill -KILL -f "ign gazebo" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2 launch ur5_bringup" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2_control_node" >/dev/null 2>&1 || true
    pkill -KILL -f "robot_state_publisher" >/dev/null 2>&1 || true
    pkill -KILL -f "world_tf_publisher" >/dev/null 2>&1 || true
    pkill -KILL -f "controller_manager" >/dev/null 2>&1 || true
    pkill -KILL -f "spawner" >/dev/null 2>&1 || true
    pkill -KILL -f "move_group" >/dev/null 2>&1 || true
    pkill -KILL -f "release_objects_service" >/dev/null 2>&1 || true
    pkill -KILL -f "system_state_manager" >/dev/null 2>&1 || true
    pkill -KILL -f "ur5_qt_panel" >/dev/null 2>&1 || true
    pkill -KILL -f "panel_v2.py" >/dev/null 2>&1 || true
    pkill -KILL -f "main_panel.py" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2 run ur5_qt_panel panel_v2" >/dev/null 2>&1 || true
  fi
fi

# --- cargar entorno ROS2 + overlay ---
# --- cargar entorno ROS2 + overlay ---
log "cargando entorno ROS 2 ${ROS_DISTRO} ($WS_DIR)"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  err "No existe $ROS_SETUP. ¿Seguro que estás en ROS 2 ${ROS_DISTRO}?"
  exit 1
fi

# 🔑 ROS 2 Jazzy NO soporta `set -u` durante source
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$WS_DIR/install/setup.bash"
else
  log "Aviso: no existe install/setup.bash (¿falta colcon build?). Continuo igualmente."
fi
set -u

# --- Recursos Gazebo y modo headless ---
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export PANEL_AUTO_BRIDGE_DELAY_MS="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
log "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"

if [[ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=0
fi
export QT_XCB_GL_INTEGRATION=none
export PANEL_SKIP_CLEANUP="${PANEL_SKIP_CLEANUP:-0}"
export PANEL_KILL_STALE="${PANEL_KILL_STALE:-1}"
if [[ -z "${DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi
export PANEL_STALE_GRACE_SEC="${PANEL_STALE_GRACE_SEC:-20}"
export PANEL_KEEP_CAMERAS="${PANEL_KEEP_CAMERAS:-${PANEL_CAMERA_REQUIRED:-0}}"

# Preparar modelo runtime con params reales para gz_ros2_control (panel lanza gz sim directamente).
runtime_models_root="$WS_DIR/log/gz_models"
runtime_ur5_model="$runtime_models_root/ur5_rg2"
rm -rf "$runtime_ur5_model" 2>/dev/null || true
mkdir -p "$runtime_ur5_model"
cp -a "$WS_DIR/models/ur5_rg2/." "$runtime_ur5_model/" 2>/dev/null || true
controllers_yaml="$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
import os
print(os.path.join(get_package_share_directory("ur5_description"), "config", "ur5_controllers.yaml"))
PY
)"
if [[ -n "$controllers_yaml" && -f "$runtime_ur5_model/model.sdf" ]]; then
  python3 - <<PY
import re
path = r"$runtime_ur5_model/model.sdf"
params = r"$controllers_yaml"
with open(path, "r", encoding="utf-8") as f:
    text = f.read()
pat = re.compile(r'(<plugin filename="gz_ros2_control-system"[^>]*>)(.*?)(</plugin>)', re.DOTALL)
m = pat.search(text)
if m:
    header, body, footer = m.groups()
    if "<parameters>" in body:
        body = re.sub(r"<parameters>.*?</parameters>", f"<parameters>{params}</parameters>", body, flags=re.DOTALL)
    else:
        body = body + f"\n            <parameters>{params}</parameters>\n"
    text = text[:m.start()] + header + body + footer + text[m.end():]
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
PY
  export GZ_SIM_RESOURCE_PATH="$runtime_models_root:$GZ_SIM_RESOURCE_PATH"
fi

log "Nota: start_panel_v2.sh es wrapper; usa ros2 launch ur5_bringup ur5_stack.launch.py"

PANEL_START_ROS2_CONTROL="${PANEL_START_ROS2_CONTROL:-0}"
PANEL_LAUNCH_BRIDGE="${PANEL_LAUNCH_BRIDGE:-$PANEL_START_STACK}"
PANEL_LAUNCH_WORLD_TF="${PANEL_LAUNCH_WORLD_TF:-1}"
PANEL_LAUNCH_SYSTEM_STATE="${PANEL_LAUNCH_SYSTEM_STATE:-1}"
export PANEL_ALLOW_UNSETTLED_ON_TIMEOUT="${PANEL_ALLOW_UNSETTLED_ON_TIMEOUT:-1}"
export DEBUG_LOGS_TO_STDOUT="${DEBUG_LOGS_TO_STDOUT:-0}"

if [[ "${PANEL_MODE}" == "manual" ]]; then
  PANEL_START_STACK="0"
  PANEL_START_ROS2_CONTROL="0"
  PANEL_LAUNCH_BRIDGE="0"
  PANEL_LAUNCH_WORLD_TF="0"
  PANEL_LAUNCH_SYSTEM_STATE="0"
  PANEL_MANAGED="0"
fi
HEADLESS="true"
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  HEADLESS="false"
fi
if [[ -n "${DISPLAY:-}" && -z "${PANEL_GZ_GUI:-}" ]]; then
  HEADLESS="false"
fi
if [[ -z "${PANEL_CAMERA_REQUIRED:-}" ]]; then
  if [[ "$HEADLESS" == "true" ]]; then
    export PANEL_CAMERA_REQUIRED=0
  else
    export PANEL_CAMERA_REQUIRED=1
  fi
elif [[ -n "${DISPLAY:-}" && "${PANEL_CAMERA_REQUIRED}" == "0" ]]; then
  export PANEL_CAMERA_REQUIRED=1
fi
if [[ "${PANEL_CAMERA_REQUIRED}" == "1" || "${PANEL_CAMERA_REQUIRED}" == "true" ]]; then
  HEADLESS="false"
fi

if [[ -z "${GZ_RENDER_ENGINE:-}" ]]; then
  if [[ "$HEADLESS" == "true" ]]; then
    export GZ_RENDER_ENGINE="ogre2"
  else
    export GZ_RENDER_ENGINE="ogre"
  fi
fi

LAUNCH_GZ="false"
LAUNCH_RSP="false"
if [[ "${PANEL_START_STACK}" == "1" ]]; then
  LAUNCH_GZ="true"
  LAUNCH_RSP="true"
  PANEL_MANAGED="1"
fi

LAUNCH_BRIDGE="false"
if [[ "${PANEL_LAUNCH_BRIDGE}" == "1" ]]; then
  LAUNCH_BRIDGE="true"
fi

LAUNCH_ROS2_CONTROL="false"
if [[ "${PANEL_START_ROS2_CONTROL}" == "1" ]]; then
  LAUNCH_ROS2_CONTROL="true"
fi

LAUNCH_WORLD_TF="false"
if [[ "${PANEL_LAUNCH_WORLD_TF}" == "1" ]]; then
  LAUNCH_WORLD_TF="true"
fi

LAUNCH_SYSTEM_STATE="false"
if [[ "${PANEL_LAUNCH_SYSTEM_STATE}" == "1" ]]; then
  LAUNCH_SYSTEM_STATE="true"
fi
if [[ "$LAUNCH_GZ" == "true" || "$LAUNCH_RSP" == "true" || "$LAUNCH_BRIDGE" == "true" || "$LAUNCH_ROS2_CONTROL" == "true" || "$LAUNCH_WORLD_TF" == "true" || "$LAUNCH_SYSTEM_STATE" == "true" ]]; then
  PANEL_MANAGED="1"
fi

LAUNCH_FILE_PKG="ur5_stack.launch.py"
LAUNCH_FILE_INSTALLED="$WS_DIR/install/ur5_bringup/share/ur5_bringup/$LAUNCH_FILE_PKG"
LAUNCH_FILE_SRC="$WS_DIR/src/ur5_bringup/launch/$LAUNCH_FILE_PKG"
LAUNCH_TARGET="ur5_bringup $LAUNCH_FILE_PKG"
if [[ -f "$LAUNCH_FILE_INSTALLED" ]]; then
  LAUNCH_TARGET="ur5_bringup $LAUNCH_FILE_PKG"
elif [[ -f "$LAUNCH_FILE_SRC" ]]; then
  LAUNCH_TARGET="$LAUNCH_FILE_SRC"
else
  err "no encuentro ur5_stack.launch.py en install o src"
  err " - $LAUNCH_FILE_INSTALLED"
  err " - $LAUNCH_FILE_SRC"
  err "Solución típica: colcon build --symlink-install"
  exit 1
fi

if [[ "${DEBUG_LOGS_TO_STDOUT}" == "1" ]]; then
  exec ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    panel_managed:="$PANEL_MANAGED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
fi

launch_log="$WS_DIR/log/ros2_launch.log"
mkdir -p "$WS_DIR/log"
PANEL_LOG_FILTER="${PANEL_LOG_FILTER:-0}"
if [[ "$PANEL_LOG_FILTER" == "1" ]]; then
  stdbuf -oL -eL ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    panel_managed:="$PANEL_MANAGED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    2>&1 \
    | tee "$launch_log" \
    | awk '
        /\[STARTUP\]/ || /\[BTN\]/ { print > "/dev/stderr"; fflush("/dev/stderr") }
        /\[ERROR\]/ || /Traceback/ || /Exception/ || /FATAL/ { print > "/dev/stderr"; fflush("/dev/stderr") }
      '
else
  stdbuf -oL -eL ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    panel_managed:="$PANEL_MANAGED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    2>&1 \
    | tee "$launch_log"
fi
