#!/usr/bin/env bash
set -euo pipefail

TS="$(date +%Y%m%d_%H%M%S)"
OUT="$PWD/reports/integration_evidence/$TS"
mkdir -p "$OUT" "$OUT/logs"

log(){ echo "[$(date +%H:%M:%S)] $*"; }
run(){ timeout "${2:-4}s" bash -lc "$1" >>"$OUT/$3" 2>&1 || true; }

log "Guardando info sistema..."
{
  echo "timestamp=$TS"
  uname -a || true
  lsb_release -a 2>/dev/null || true
  echo "ROS_DISTRO=${ROS_DISTRO:-<none>}"
  echo "GZ_PARTITION=${GZ_PARTITION:-<none>}"
  echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-<none>}"
} > "$OUT/system_info.txt"

log "Guardando estado ROS2 (con timeouts)..."
run "ros2 topic list" 4 ros_state.txt
run "ros2 node list"  4 ros_state.txt
run "ros2 service list" 4 ros_state.txt
run "ros2 control list_controllers" 4 ros_state.txt

log "Guardando estado GZ (topics)..."
run "gz topic -l" 4 gz_state.txt
run "for t in \$(gz topic -l 2>/dev/null | grep -E '/world/.*/pose/info' || true); do echo --- \$t ---; gz topic -e -t \$t -n 1 --json-output || true; done" 4 gz_state.txt

log "Copiando logs locales..."
cp -av "$PWD/log" "$OUT/logs/" 2>/dev/null || true

# Opcional: enlazar summary_base.csv del repo de IA si existe
if [ -f "$HOME/TFM/agarre_inteligente/experiments/summary_base.csv" ]; then
  cp -av "$HOME/TFM/agarre_inteligente/experiments/summary_base.csv" "$OUT/" 2>/dev/null || true
fi

log "OK. Evidencias en: $OUT"
echo "$OUT" > "$OUT/_PATH.txt"
