#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/patch_panel_checks_v2.sh
# Summary: Applies robustness patches to main_panel.py with backups.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
PANEL_PY="$WS_DIR/src/ur5_qt_panel/ur5_qt_panel/main_panel.py"

if [[ ! -f "$PANEL_PY" ]]; then
  echo "[ERROR] No existe: $PANEL_PY"
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BK_DIR="$WS_DIR/backups/panel_checks_v2_${STAMP}"
mkdir -p "$BK_DIR"
cp -a "$PANEL_PY" "$BK_DIR/main_panel.py.bak"

echo "[INFO] Backup: $BK_DIR/main_panel.py.bak"

# 1) Spawner robusto (timeouts + activate + controller_manager explícito)
perl -pi -e 's/ros2 run controller_manager spawner joint_state_broadcaster \|\| true/ros2 run controller_manager spawner joint_state_broadcaster -c \\/controller_manager --activate --controller-manager-timeout 30 --switch-timeout 30 || true/g' "$PANEL_PY"
perl -pi -e 's/ros2 run controller_manager spawner joint_trajectory_controller \|\| true/ros2 run controller_manager spawner joint_trajectory_controller -c \\/controller_manager --activate --controller-manager-timeout 30 --switch-timeout 30 || true/g' "$PANEL_PY"

# 2) No abortar por detección frágil del panel (deja que ur5_go_home.sh valide de verdad)
perl -0777 -pi -e 's/self\.log_fn\(\Q"[ROBOT] No hay controladores activos (controller_manager no está corriendo)."\E\)\s*\n\s*return/self.log_fn("[ROBOT] WARN: El panel no detecta controladores activos. Continúo; el script HOME valida controller_manager/controladores.")/g' "$PANEL_PY"
perl -0777 -pi -e 's/self\.log_fn\(\Q"[ROBOT] joint_trajectory_controller no activo."\E\)\s*\n\s*return/self.log_fn("[ROBOT] WARN: El panel no detecta joint_trajectory_controller ACTIVE. Continúo; el script HOME valida y\\/o activa si hace falta.")/g' "$PANEL_PY"

echo "[OK] Parche aplicado en: $PANEL_PY"
echo "[INFO] Revertir: cp $BK_DIR/main_panel.py.bak $PANEL_PY"
