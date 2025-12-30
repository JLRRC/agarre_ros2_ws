#!/usr/bin/env bash
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_quick_test.sh
# Summary: Runs a short UR5 test (HOME + optional gripper open/close) with minimal logs.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"

set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash"
set -u

LC="$(ros2 control list_controllers 2>/dev/null || true)"

export HOME_QUIET=1
export HOME_TIMEOUT="${HOME_TIMEOUT:-10}"
"$WS_DIR/scripts/ur5_go_home.sh"

if echo "$LC" | grep -qE "^gripper_controller[[:space:]].*\\bactive\\b"; then
  timeout "${GRIPPER_TIMEOUT:-6}" "$WS_DIR/scripts/ur5_open_gripper.sh" >/dev/null 2>&1 || true
  sleep 0.5
  timeout "${GRIPPER_TIMEOUT:-6}" "$WS_DIR/scripts/ur5_close_gripper.sh" >/dev/null 2>&1 || true
else
  # Si no hay ros2_control, intentamos igualmente control directo en GZ.
  timeout "${GRIPPER_TIMEOUT:-6}" "$WS_DIR/scripts/ur5_open_gripper.sh" >/dev/null 2>&1 || true
  sleep 0.5
  timeout "${GRIPPER_TIMEOUT:-6}" "$WS_DIR/scripts/ur5_close_gripper.sh" >/dev/null 2>&1 || true
fi

if [[ -f "$WS_DIR/scripts/ur5_report_dimensions.py" ]]; then
  python3 "$WS_DIR/scripts/ur5_report_dimensions.py" || true
fi
