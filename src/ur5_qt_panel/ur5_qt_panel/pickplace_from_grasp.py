#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pickplace_from_grasp.py
# Summary: CLI pick&place using last_grasp JSON + numeric IK (UR5).
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import subprocess
import time
from typing import Dict, List, Optional

try:
    import yaml
except Exception:
    yaml = None

from panel_config import (
    BASKET_DROP,
    GRIPPER_ATTACH_PREFIX,
    GZ_WORLD,
    SCRIPTS_DIR,
    UR5_JOINT_NAMES,
    WS_DIR,
)
from panel_utils import bash_preamble, build_gz_env, detect_arm_trajectory_topic, resolve_gz_partition
from ur5_kinematics import ik_ur5, rot_x, rot_z


def _read_joint_positions_from_gz(partition: str) -> Optional[List[float]]:
    gz_env = build_gz_env(resolve_gz_partition(partition))
    cmd_list = gz_env + "gz topic -l"
    res_list = subprocess.run(["bash", "-lc", cmd_list], text=True, capture_output=True)
    if res_list.returncode != 0:
        print("[ERROR] No puedo listar topics de Gazebo.")
        return None
    candidates = []
    for line in (res_list.stdout or "").splitlines():
        if "joint_state" in line and "ur5" in line:
            candidates.append(line.strip())
    if not candidates:
        print("[ERROR] No hay joint_state en Gazebo.")
        return None
    topic = candidates[0]
    cmd = gz_env + f"gz topic -e -n 1 -t '{topic}' --json-output"
    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
    if res.returncode != 0:
        print("[ERROR] joint_state Gazebo: fallo leyendo.")
        return None
    raw = (res.stdout or "").strip()
    if not raw:
        print("[ERROR] joint_state Gazebo vacío.")
        return None
    try:
        data = json.loads(raw[raw.find('{'):])
    except Exception as exc:
        print(f"[ERROR] parseando joint_state Gazebo: {exc}")
        return None
    if "msg" in data and isinstance(data["msg"], dict):
        data = data["msg"]
    mapping = {}
    names = data.get("name")
    positions = data.get("position")
    if isinstance(names, list) and isinstance(positions, list) and len(names) == len(positions):
        for name, pos in zip(names, positions):
            try:
                mapping[str(name)] = float(pos)
            except Exception:
                continue
    joints = data.get("joint")
    if isinstance(joints, list):
        for joint in joints:
            if not isinstance(joint, dict):
                continue
            name = joint.get("name")
            if not name:
                continue
            pos = joint.get("position")
            if isinstance(pos, list) and pos:
                pos = pos[0]
            if pos is None:
                axis1 = joint.get("axis1") or {}
                if isinstance(axis1, dict):
                    pos = axis1.get("position")
                    if pos is None:
                        pos = axis1.get("angle")
            try:
                mapping[str(name)] = float(pos)
            except Exception:
                continue
    if not mapping:
        print("[ERROR] joint_state Gazebo sin joints.")
        return None
    ordered = []
    for j in UR5_JOINT_NAMES:
        if j not in mapping:
            print(f"[ERROR] falta {j} en joint_state Gazebo.")
            return None
        ordered.append(float(mapping[j]))
    print("[INFO] joint_states desde Gazebo.")
    return ordered


def _read_joint_positions(partition: str) -> Optional[List[float]]:
    cmd_type = bash_preamble(WS_DIR) + "ros2 topic list -t | awk '/^\\/joint_states[[:space:]]/ {print $2}'"
    res_type = subprocess.run(["bash", "-lc", cmd_type], text=True, capture_output=True)
    msg_type = (res_type.stdout or "").strip().strip("[]")
    if not msg_type:
        return _read_joint_positions_from_gz(partition)
    cmd = bash_preamble(WS_DIR) + f"timeout 2 ros2 topic echo /joint_states {shlex.quote(msg_type)} --once"
    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
    if res.returncode != 0:
        print(f"[ERROR] joint_states ({msg_type}): {res.stderr.strip() or res.stdout.strip()}")
        return _read_joint_positions_from_gz(partition)
    if yaml is None:
        print("[ERROR] yaml no disponible.")
        return None
    data = yaml.safe_load(res.stdout)
    names = data.get("name") or []
    pos = data.get("position") or []
    if not names or not pos or len(names) != len(pos):
        print("[ERROR] joint_states incompleto.")
        return None
    mapping = dict(zip(names, pos))
    out = []
    for j in UR5_JOINT_NAMES:
        if j not in mapping:
            print(f"[ERROR] falta {j} en joint_states.")
            return None
        out.append(float(mapping[j]))
    return out


def _send_joint_trajectory(positions: List[float], duration: float) -> bool:
    sec = max(0.5, float(duration))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    msg = {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
        "joint_names": UR5_JOINT_NAMES,
        "points": [{
            "positions": [round(p, 5) for p in positions],
            "time_from_start": {"sec": sec_i, "nanosec": nsec_i},
        }],
    }
    topic = detect_arm_trajectory_topic()
    cmd = (
        bash_preamble(WS_DIR)
        + "timeout 8 ros2 topic pub --once "
        + f"{shlex.quote(topic)} trajectory_msgs/msg/JointTrajectory "
        + shlex.quote(json.dumps(msg))
    )
    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
    if res.returncode != 0:
        print(f"[ERROR] trajectory: {res.stderr.strip() or res.stdout.strip()}")
        return False
    return True


def _run_script(path: str) -> None:
    cmd = bash_preamble(WS_DIR) + f"timeout 8 '{path}' || true"
    subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", default="pieza_pick_mesa")
    parser.add_argument("--grasp", default=None, help="Ruta a last_grasp_<obj>.json")
    parser.add_argument("--partition", default="")
    args = parser.parse_args()

    obj = args.object
    grasp_path = args.grasp or os.path.expanduser(f"~/TFM/reports/ros2_pickplace/last_grasp_{obj}.json")
    if not os.path.isfile(grasp_path):
        print(f"[ERROR] No existe grasp: {grasp_path}")
        return 1
    with open(grasp_path, "r", encoding="utf-8") as f:
        grasp = json.load(f)
    world_xy = grasp.get("world_xy_est")
    if not world_xy:
        print("[ERROR] grasp sin world_xy_est.")
        return 1
    wx, wy = float(world_xy[0]), float(world_xy[1])
    wz = float(grasp.get("world_z", 0.80))
    yaw = float(grasp.get("angle_deg", 0.0)) * math.pi / 180.0
    rot = rot_z(yaw) @ rot_x(math.pi)

    q_current = _read_joint_positions(args.partition)
    if not q_current:
        return 1
    z_pre = wz + 0.12
    z_grasp = wz + 0.02
    z_lift = wz + 0.15
    basket_x, basket_y, basket_z = BASKET_DROP
    basket_pre = basket_z + 0.12
    basket_drop = basket_z + 0.03
    poses = [
        ("pregrasp", (wx, wy, z_pre)),
        ("grasp", (wx, wy, z_grasp)),
        ("lift", (wx, wy, z_lift)),
        ("basket_pre", (basket_x, basket_y, basket_pre)),
        ("basket_drop", (basket_x, basket_y, basket_drop)),
        ("basket_pre_2", (basket_x, basket_y, basket_pre)),
    ]
    q_plan = []
    q_seed = q_current
    for label, pos in poses:
        q_sol, err, ok = ik_ur5(pos, rot, q_seed)
        print(f"[IK] {label} err={err:.4f} ok={ok}")
        if not ok or err > 0.15:
            print(f"[ERROR] IK falla en {label}.")
            return 1
        q_plan.append(q_sol)
        q_seed = q_sol

    _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
    if not _send_joint_trajectory(q_plan[0].tolist(), 4.0):
        return 1
    if not _send_joint_trajectory(q_plan[1].tolist(), 3.0):
        return 1
    _run_script(os.path.join(SCRIPTS_DIR, "ur5_close_gripper.sh"))
    part = resolve_gz_partition(args.partition)
    gz_env = build_gz_env(part)
    attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/attach"
    attach_cmd = f"gz topic -t '{attach_topic}' -m gz.msgs.Empty -p 'unused: true'"
    subprocess.run(["bash", "-lc", gz_env + attach_cmd], text=True, capture_output=True)
    _send_joint_trajectory(q_plan[2].tolist(), 3.0)
    _send_joint_trajectory(q_plan[3].tolist(), 4.0)
    _send_joint_trajectory(q_plan[4].tolist(), 3.0)
    _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
    detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/detach"
    detach_cmd = f"gz topic -t '{detach_topic}' -m gz.msgs.Empty -p 'unused: true'"
    subprocess.run(["bash", "-lc", gz_env + detach_cmd], text=True, capture_output=True)
    _send_joint_trajectory(q_plan[5].tolist(), 3.0)
    print(f"[OK] Pick&place completado para {obj}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
