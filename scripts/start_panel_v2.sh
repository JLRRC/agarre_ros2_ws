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
: "${PANEL_START_STACK:=0}"        # 1 = autoarranca RSP+Gazebo, 0 = solo panel (default)
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

  # Paneles previos (ajusta patrones si lo necesitas)
  pkill -f "ur5_qt_panel"      2>/dev/null || true
  pkill -f "panel_v2.py"       2>/dev/null || true
  pkill -f "main_panel.py"     2>/dev/null || true
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

export LIBGL_ALWAYS_SOFTWARE=1
export QT_XCB_GL_INTEGRATION=none
export PANEL_SKIP_CLEANUP=1
if [[ -z "${DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi

log "Nota: start_panel_v2.sh es wrapper; usa ros2 launch ur5_bringup ur5_stack.launch.py"

PANEL_START_ROS2_CONTROL="${PANEL_START_ROS2_CONTROL:-0}"
PANEL_LAUNCH_BRIDGE="${PANEL_LAUNCH_BRIDGE:-$PANEL_START_STACK}"
HEADLESS="true"
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  HEADLESS="false"
fi

LAUNCH_GZ="false"
LAUNCH_RSP="false"
if [[ "${PANEL_START_STACK}" == "1" ]]; then
  LAUNCH_GZ="true"
  LAUNCH_RSP="true"
fi

LAUNCH_BRIDGE="false"
if [[ "${PANEL_LAUNCH_BRIDGE}" == "1" ]]; then
  LAUNCH_BRIDGE="true"
fi

LAUNCH_ROS2_CONTROL="false"
if [[ "${PANEL_START_ROS2_CONTROL}" == "1" ]]; then
  LAUNCH_ROS2_CONTROL="true"
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

exec ros2 launch $LAUNCH_TARGET \
  headless:="$HEADLESS" \
  launch_panel:=true \
  launch_gazebo:="$LAUNCH_GZ" \
  launch_rsp:="$LAUNCH_RSP" \
  launch_bridge:="$LAUNCH_BRIDGE" \
  launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
  panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
  panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
