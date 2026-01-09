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

# --- FIX 1 y 3: Recursos Gazebo y modo headless ---
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
log "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"

# Lanzar Gazebo en modo headless por defecto
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  log "Lanzando Gazebo en modo GUI"
  gz sim -r "$WS_DIR/worlds/ur5_mesa_objetos.sdf" &
else
  log "Lanzando Gazebo en modo headless"
  gz sim -s -r "$WS_DIR/worlds/ur5_mesa_objetos.sdf" &
fi

# Config (puedes exportar estas vars antes de lanzar)
: "${ROS_DISTRO:=jazzy}"
: "${PANEL_COLD_BOOT:=1}"          # 1 = mata procesos antes de arrancar
: "${PANEL_V2_PREFER_INSTALLED:=1}"# 1 = usa install/.../panel_v2 si existe

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

# --- decidir cómo lanzar panel_v2 ---
INSTALLED_BIN="$WS_DIR/install/ur5_qt_panel/lib/ur5_qt_panel/panel_v2"
SRC_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py"

if [[ "$PANEL_V2_PREFER_INSTALLED" == "1" && -x "$INSTALLED_BIN" ]]; then
  log "Lanzando panel_v2 (instalado): $INSTALLED_BIN"
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_XCB_GL_INTEGRATION=none
  exec "$INSTALLED_BIN"
fi

# Fallback: ejecutar desde src
if [[ -f "$SRC_PY" ]]; then
  log "Lanzando panel_v2 (src): $SRC_PY"
  # Asegurar que Python encuentre el paquete
  export PYTHONPATH="$WS_DIR/src/ur5_qt_panel:${PYTHONPATH:-}"
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_XCB_GL_INTEGRATION=none
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
