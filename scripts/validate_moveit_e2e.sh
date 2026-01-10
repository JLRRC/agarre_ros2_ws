#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
LOG_DIR="$WS_DIR/log"
PANEL_LOG="$LOG_DIR/panel_v2.log"
BRIDGE_LOG="$LOG_DIR/ros_gz_bridge.log"
MOVEIT_LOG="$LOG_DIR/moveit_bringup.log"
MOVEIT_BRIDGE_LOG="$LOG_DIR/moveit_bridge.log"

mkdir -p "$LOG_DIR"

set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

log() { echo "[E2E] $*"; }

log "check moveit_commander"
python3 - <<'PY'
import importlib
ok = False
try:
    from moveit.planning import MoveItPy  # type: ignore
    ok = True
except Exception:
    ok = False
if not ok:
    ok = importlib.util.find_spec("moveit_commander") is not None
raise SystemExit(0 if ok else 1)
PY
if [ $? -ne 0 ]; then
  log "FAIL: MoveIt Python no disponible (instala ros-jazzy-moveit-py)"
  exit 1
fi

cleanup() {
  log "limpieza: kill_all.sh"
  "$WS_DIR/scripts/kill_all.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

log "clean session"
cleanup

log "start panel (offscreen)"
PANEL_COLD_BOOT=1 PANEL_AUTO_BRIDGE=0 PANEL_V2_PREFER_INSTALLED=1 \
  QT_QPA_PLATFORM=offscreen \
  "$WS_DIR/scripts/start_panel_v2.sh" >"$PANEL_LOG" 2>&1 &
PANEL_PID=$!

log "wait gazebo"
for _ in {1..30}; do
  if pgrep -f "gz sim" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! pgrep -f "gz sim" >/dev/null 2>&1; then
  log "FAIL: gz sim not running"
  exit 1
fi

if [ -f "$LOG_DIR/gz_partition.txt" ]; then
  export GZ_PARTITION
  GZ_PARTITION="$(cat "$LOG_DIR/gz_partition.txt" | tr -d ' \n')"
  log "GZ_PARTITION detectado: $GZ_PARTITION"
fi

log "start bridge"
WS_DIR="$WS_DIR" "$WS_DIR/scripts/run_gz_ros_bridge.sh" >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

log "wait /clock"
if timeout 6 ros2 topic list --no-daemon --spin-time 2 2>/dev/null | grep -qx "/clock"; then
  log "OK: /clock visible"
else
  if grep -q "/clock" "$BRIDGE_LOG" 2>/dev/null; then
    log "WARN: /clock no visible en ROS 2 (bridge creado). Continuo."
  else
    log "FAIL: /clock not available"
    exit 1
  fi
fi

log "start moveit bringup"
ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py \
  start_ros2_control:=false launch_rviz:=false >"$MOVEIT_LOG" 2>&1 &
MOVEIT_PID=$!

log "wait move_group"
for _ in {1..120}; do
  if pgrep -f "move_group" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! pgrep -f "move_group" >/dev/null 2>&1; then
  log "FAIL: move_group no inició correctamente"
  exit 1
fi

log "start moveit bridge"
ros2 run ur5_tools ur5_moveit_bridge >"$MOVEIT_BRIDGE_LOG" 2>&1 &
MOVEIT_BRIDGE_PID=$!

log "wait moveit bridge process"
for _ in {1..20}; do
  if pgrep -f "ur5_moveit_bridge" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! pgrep -f "ur5_moveit_bridge" >/dev/null 2>&1; then
  log "FAIL: moveit bridge no arrancó"
  exit 1
fi
sleep 5

log "wait /desired_grasp subscriber"
for _ in {1..30}; do
  sub_count="$(ros2 topic info /desired_grasp 2>/dev/null | awk '/Subscription count/ {print $3}')"
  if [ "${sub_count:-0}" -gt 0 ]; then
    break
  fi
  sleep 0.5
done
if [ "${sub_count:-0}" -le 0 ]; then
  log "FAIL: /desired_grasp sin suscriptores"
  exit 1
fi

log "publish desired_grasp"
python3 - <<'PY'
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node("e2e_desired_grasp_pub")
pub = node.create_publisher(PoseStamped, "/desired_grasp", 10)
msg = PoseStamped()
msg.header.frame_id = "base_link"
msg.pose.position.x = 0.40
msg.pose.position.y = 0.0
msg.pose.position.z = 0.30
msg.pose.orientation.w = 1.0

for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
PY

sleep 8

log "check logs"
if grep -q "Planificación MoveItPy OK" "$MOVEIT_BRIDGE_LOG" 2>/dev/null || \
   grep -q "Planificación con MoveIt OK" "$MOVEIT_BRIDGE_LOG" 2>/dev/null; then
  log "OK: moveit_bridge planned"
else
  log "FAIL: no planning confirmation in moveit_bridge.log"
  exit 1
fi

if grep -q "You can start planning now" "$MOVEIT_LOG" 2>/dev/null; then
  log "OK: move_group started"
else
  log "WARN: move_group log missing"
fi

log "E2E validation finished"
