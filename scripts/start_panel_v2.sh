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
if [[ "${PANEL_START_STACK}" == "1" ]]; then
  export PANEL_AUTO_BRIDGE="${PANEL_AUTO_BRIDGE:-1}"
else
  export PANEL_AUTO_BRIDGE="${PANEL_AUTO_BRIDGE:-0}"
fi
export PANEL_AUTO_BRIDGE_DELAY_MS="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
log "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"

# Alinear particion de GZ para que el bridge vea los topics correctos.
if [[ -z "${GZ_PARTITION:-}" ]]; then
  GZ_PARTITION="ur5pro_$(date +%s)"
  export GZ_PARTITION
fi
mkdir -p "$WS_DIR/log"
echo "$GZ_PARTITION" > "$WS_DIR/log/gz_partition.txt"
log "GZ_PARTITION set to: $GZ_PARTITION"

if [[ "${PANEL_START_STACK}" == "1" ]]; then
  # Lanzar robot_state_publisher antes de Gazebo para publicar /robot_description
  log "Lanzando robot_state_publisher (UR5 RSP)"
  ros2 launch ur5_bringup ur5_rsp.launch.py use_sim_time:=true >/tmp/ur5_rsp.log 2>&1 &

  # Lanzar Gazebo en modo headless por defecto
  if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
    log "Lanzando Gazebo en modo GUI"
    GZ_PARTITION="$GZ_PARTITION" gz sim -r "$WS_DIR/worlds/ur5_mesa_objetos.sdf" &
  else
    log "Lanzando Gazebo en modo headless"
    EGL_VENDOR_DEFAULT="/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    EGL_VENDOR_PATH="${EGL_VENDOR:-$EGL_VENDOR_DEFAULT}"
    EGL_ENV=()
    if [[ -f "$EGL_VENDOR_PATH" ]]; then
      EGL_ENV+=("__EGL_VENDOR_LIBRARY_FILENAMES=$EGL_VENDOR_PATH")
    fi
    env -u DISPLAY "${EGL_ENV[@]}" GZ_RENDER_ENGINE=ogre2 GZ_PARTITION="$GZ_PARTITION" \
      gz sim -s -r --headless-rendering "$WS_DIR/worlds/ur5_mesa_objetos.sdf" &
  fi
else
  log "PANEL_START_STACK=0 → no se autoarranca RSP/Gazebo (modo manual desde el panel)"
fi

# --- decidir cómo lanzar panel_v2 ---
INSTALLED_BIN="$WS_DIR/install/ur5_qt_panel/lib/ur5_qt_panel/panel_v2"
SRC_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py"

if [[ "$PANEL_V2_PREFER_INSTALLED" == "1" && -x "$INSTALLED_BIN" ]]; then
  log "Lanzando panel_v2 (instalado): $INSTALLED_BIN"
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_XCB_GL_INTEGRATION=none
  export PANEL_SKIP_CLEANUP=1
  if [[ -z "${DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM=offscreen
  fi
  exec "$INSTALLED_BIN"
fi

# Fallback: ejecutar desde src
if [[ -f "$SRC_PY" ]]; then
  log "Lanzando panel_v2 (src): $SRC_PY"
  # Asegurar que Python encuentre el paquete
  export PYTHONPATH="$WS_DIR/src/ur5_qt_panel:${PYTHONPATH:-}"
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_XCB_GL_INTEGRATION=none
  export PANEL_SKIP_CLEANUP=1
  if [[ -z "${DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM=offscreen
  fi
  exec /usr/bin/python3 "$SRC_PY"
fi

# Último fallback: módulo python (si estuviera instalado en site-packages)
log "Fallback: intentando python -m ur5_qt_panel.panel_v2"
if /usr/bin/python3 -c "import ur5_qt_panel.panel_v2" >/dev/null 2>&1; then
  exec /usr/bin/python3 -m ur5_qt_panel.panel_v2
fi

err "no encuentro panel_v2 en:"
err " - $INSTALLED_BIN (instalado ejecutable)"
err " - $SRC_PY (archivo fuente)"
err "Solución típica: recuperar src/ur5_qt_panel/ur5_qt_panel/panel_v2.py o reconstruir con colcon build."
exit 1
