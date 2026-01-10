#!/usr/bin/env bash
# Ruta/URL: file:///home/laboratorio/TFM/agarre_ros2_ws/scripts
# Nombre: start_all.sh
# Qué hace: wrapper para START ALL desde el panel; lanza el bringup oficial start_panel_v2.sh.

set -euo pipefail

WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

# Si quieres forzar cold boot desde aquí, descomenta:
# export PANEL_COLD_BOOT=1

# START ALL debe arrancar el stack completo
export PANEL_START_STACK=1

exec "${WS_DIR}/scripts/start_panel_v2.sh"
