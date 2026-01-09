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
from typing import Dict, List, Optional, Tuple

try:
    import yaml
except Exception:
    yaml = None

from panel_config import (
    BASKET_DROP,
    GRIPPER_ATTACH_PREFIX,
    GZ_WORLD,
    PICK_MAX_TCP_OBJ_DIST,
    PICK_MAX_IK_ERR_PREGRASP,
    SCRIPTS_DIR,
    UR5_JOINT_NAMES,
    WS_DIR,
)
from panel_utils import (
    bash_preamble,
    build_gz_env,
    detect_arm_trajectory_topic,
    object_out_of_reach,
    resolve_gz_partition,
    world_to_base,
)
from ur5_kinematics import fk_ur5, ik_ur5, rot_x, rot_z


def _json_log(payload: Dict[str, object]) -> None:
    try:
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    except Exception:
        print(payload)


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _yaw_candidates(yaw: float) -> List[float]:
    cands = [
        yaw,
        0.0,
        yaw + (math.pi / 2.0),
        yaw - (math.pi / 2.0),
        yaw + (math.pi / 4.0),
        yaw - (math.pi / 4.0),
        math.pi,
        -math.pi,
    ]
    out = []
    for val in cands:
        v = _wrap_angle(float(val))
        if all(abs(v - prev) > 1e-4 for prev in out):
            out.append(v)
    return out


def _seed_candidates(q_seed: List[float]) -> List[List[float]]:
    seeds = [list(q_seed)]
    delta = 0.08
    for idx in range(len(q_seed)):
        for sign in (-1.0, 1.0):
            cand = list(q_seed)
            cand[idx] = float(cand[idx]) + (sign * delta)
            seeds.append(cand)
    return seeds


def _normalize_ik_result(result) -> Tuple[Optional[List[float]], Optional[float], bool]:
    if result is None:
        return None, None, False
    if isinstance(result, (list, tuple)):
        if len(result) >= 3:
            q, err, ok = result[0], result[1], result[2]
            return list(q) if q is not None else None, float(err) if err is not None else None, bool(ok)
        if len(result) == 2:
            q, err = result[0], result[1]
            return list(q) if q is not None else None, float(err) if err is not None else None, q is not None
        if len(result) == 1:
            q = result[0]
            return list(q) if q is not None else None, 0.0, q is not None
    try:
        return list(result), 0.0, True
    except Exception:
        return None, None, False


def _call_ik(pos, rot, seed, rot_weight: Optional[float] = None):
    try:
        if rot_weight is not None:
            res = ik_ur5(pos, rot, seed, rot_weight=rot_weight)
        else:
            res = ik_ur5(pos, rot, seed)
        return _normalize_ik_result(res), ""
    except TypeError:
        try:
            res = ik_ur5(pos, seed)
            return _normalize_ik_result(res), ""
        except Exception as exc:
            return (None, None, False), f"exception:{exc}"
    except Exception as exc:
        return (None, None, False), f"exception:{exc}"


def _within_limits(q: List[float]) -> Tuple[bool, str]:
    limits = None
    for mod in ("ur5_kinematics", "panel_config"):
        try:
            limits = getattr(__import__(mod, fromlist=["UR5_JOINT_LIMITS"]), "UR5_JOINT_LIMITS", None)
        except Exception:
            limits = None
        if limits:
            break
    if not limits:
        return True, "limits_skipped"
    if len(limits) != 6:
        return True, "limits_skipped"
    for idx, (val, lim) in enumerate(zip(q, limits)):
        try:
            lo, hi = float(lim[0]), float(lim[1])
        except Exception:
            return True, "limits_skipped"
        if val < lo or val > hi:
            return False, f"limit_j{idx}"
    return True, "limits_ok"


def _parse_pose_json(raw: str) -> List[dict]:
    if not raw:
        return []
    candidates = []
    buf = ""
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("{") and line.endswith("}"):
            candidates.append(line)
        elif line.startswith("{"):
            buf = line
        elif buf:
            buf += line
            if line.endswith("}"):
                candidates.append(buf)
                buf = ""
    if not candidates and "{" in raw and "}" in raw:
        start = raw.find("{")
        end = raw.rfind("}")
        if start >= 0 and end > start:
            candidates.append(raw[start : end + 1])
    for payload in reversed(candidates):
        try:
            data = json.loads(payload)
        except Exception:
            continue
        poses = []
        if isinstance(data, dict):
            poses = data.get("pose") or data.get("poses") or []
            if not poses and isinstance(data.get("msg"), dict):
                msg = data.get("msg")
                poses = msg.get("pose") or msg.get("poses") or []
        if isinstance(poses, list) and poses:
            return poses
    return []


def _get_object_pose_gz(obj_name: str, partition: str) -> Optional[List[float]]:
    gz_env = build_gz_env(resolve_gz_partition(partition))
    cmd = gz_env + f"gz topic -e -n 1 -t '/world/{GZ_WORLD}/pose/info' --json-output"
    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
    if res.returncode != 0:
        return None
    poses = _parse_pose_json(res.stdout or "")
    if not poses:
        return None
    target = None
    for pose in poses:
        if not isinstance(pose, dict):
            continue
        if pose.get("name") == obj_name:
            target = pose
            break
    if target is None:
        prefix = f"{obj_name}::"
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            nm = str(pose.get("name") or "")
            if nm.startswith(prefix):
                target = pose
                break
    if not target:
        return None
    pos = target.get("position") or {}
    try:
        return [float(pos.get("x")), float(pos.get("y")), float(pos.get("z"))]
    except (TypeError, ValueError):
        return None


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
    if "data" in data and isinstance(data["data"], dict):
        data = data["data"]
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
    data = None
    text = res.stdout or ""
    idx = text.find("\nname:")
    if idx == -1 and text.startswith("name:"):
        idx = 0
    if idx > 0:
        text = text[idx + 1:]
    for doc in yaml.safe_load_all(text):
        if isinstance(doc, dict) and doc.get("name") and doc.get("position"):
            data = doc
            break
    if data is None:
        print("[ERROR] joint_states invalido (YAML sin datos).")
        return None
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
    clean_positions = []
    for p in positions:
        val = p
        if isinstance(val, (list, tuple)) and len(val) == 1:
            val = val[0]
        try:
            fval = float(val)
        except (TypeError, ValueError):
            print(f"[ERROR] joint position no numerica ({val}).")
            return False
        if not math.isfinite(fval):
            print(f"[ERROR] joint position no finita ({fval}).")
            return False
        clean_positions.append(fval)
    sec = max(0.5, float(duration))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    msg = {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
        "joint_names": UR5_JOINT_NAMES,
        "points": [{
            "positions": [round(p, 5) for p in clean_positions],
            "time_from_start": {"sec": sec_i, "nanosec": nsec_i},
        }],
    }
    if yaml is not None:
        try:
            payload_text = yaml.safe_dump(msg, default_flow_style=True, sort_keys=False)
        except Exception:
            payload_text = json.dumps(msg)
    else:
        payload_text = json.dumps(msg)
    payload_text = " ".join(payload_text.splitlines())
    topic = detect_arm_trajectory_topic()
    cmd = (
        bash_preamble(WS_DIR)
        + "timeout 8 ros2 topic pub --once "
        + f"{shlex.quote(topic)} trajectory_msgs/msg/JointTrajectory "
        + shlex.quote(payload_text)
    )
    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
    if res.returncode == 0:
        return True
    print(f"[WARN] trajectory: {res.stderr.strip() or res.stdout.strip()}")
    print(f"[WARN] payload: {payload_text}")
    payload = {
        "topic": topic,
        "joint_names": UR5_JOINT_NAMES,
        "positions": clean_positions,
        "sec": sec_i,
        "nanosec": nsec_i,
    }
    fb = (
        "import json\n"
        "import rclpy\n"
        "from builtin_interfaces.msg import Duration\n"
        "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint\n"
        f"payload = json.loads({json.dumps(payload)!r})\n"
        "rclpy.init(args=None)\n"
        "node = rclpy.create_node('pickplace_joint_pub')\n"
        "pub = node.create_publisher(JointTrajectory, payload['topic'], 10)\n"
        "msg = JointTrajectory()\n"
        "msg.joint_names = payload['joint_names']\n"
        "pt = JointTrajectoryPoint()\n"
        "pt.positions = payload['positions']\n"
        "pt.time_from_start = Duration(sec=int(payload['sec']), nanosec=int(payload['nanosec']))\n"
        "msg.points = [pt]\n"
        "pub.publish(msg)\n"
        "rclpy.spin_once(node, timeout_sec=0.2)\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
    )
    fb_cmd = bash_preamble(WS_DIR) + f"python3 - <<'PY'\n{fb}PY"
    fb_res = subprocess.run(["bash", "-lc", fb_cmd], text=True, capture_output=True)
    if fb_res.returncode != 0:
        print(f"[ERROR] trajectory rclpy: {fb_res.stderr.strip() or fb_res.stdout.strip()}")
        return False
    print("[OK] trajectory publicada con rclpy.")
    return True


def _run_script(path: str) -> None:
    cmd = bash_preamble(WS_DIR) + f"timeout 8 '{path}' || true"
    subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", default="pick_demo")
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
    obj_pose = _get_object_pose_gz(obj, args.partition)
    if obj_pose:
        wx, wy, wz = float(obj_pose[0]), float(obj_pose[1]), float(obj_pose[2])
        try:
            dx = wx - float(world_xy[0])
            dy = wy - float(world_xy[1])
            print(f"[PICK] Delta grasp->obj: dx={dx:.3f} dy={dy:.3f}")
        except (TypeError, ValueError, IndexError):
            pass
    else:
        wx, wy = float(world_xy[0]), float(world_xy[1])
        wz = float(grasp.get("world_z", 0.80))
    if object_out_of_reach(wx, wy):
        print(f"[ERROR] {obj} fuera del alcance en ({wx:.3f},{wy:.3f}).")
        return 1
    bx, by, bz = world_to_base(wx, wy, wz)
    yaw = float(grasp.get("angle_deg", 0.0)) * math.pi / 180.0
    rot_grasp = rot_z(yaw) @ rot_x(math.pi)
    rot_basket = rot_z(0.0) @ rot_x(math.pi)

    q_current = _read_joint_positions(args.partition)
    if not q_current:
        return 1
    z_pre = bz + 0.12
    z_pre_fallback = bz + 0.18
    z_grasp = bz + 0.02
    z_lift = bz + 0.15
    z_lift_fallback = bz + 0.10
    basket_x, basket_y, basket_z = BASKET_DROP
    basket_bx, basket_by, basket_bz = world_to_base(basket_x, basket_y, basket_z)
    basket_pre = basket_bz + 0.12
    basket_drop = basket_bz + 0.03
    poses = [
        ("pregrasp", (bx, by, z_pre)),
        ("grasp", (bx, by, z_grasp)),
        ("lift", (bx, by, z_lift)),
        ("basket_pre", (basket_bx, basket_by, basket_pre)),
        ("basket_drop", (basket_bx, basket_by, basket_drop)),
        ("basket_pre_2", (basket_bx, basket_by, basket_pre)),
    ]
    q_plan = []
    q_seed = q_current
    use_basket_script = False
    attempt_id = 0
    for label, pos in poses:
        err_limit = PICK_MAX_IK_ERR_PREGRASP if label == "pregrasp" else 0.15
        base_yaw = yaw if not label.startswith("basket") else 0.0
        yaw_list = _yaw_candidates(base_yaw)
        z_offsets = [0.0]
        if label == "pregrasp":
            z_offsets += [0.02, 0.04, 0.06, 0.08, 0.10]
        seeds = _seed_candidates(q_seed)
        q_sol = None
        err = None
        ok = False
        solved = False
        best_err = None
        best_sol = None
        best_yaw = None
        best_z = None

        for z_off in z_offsets:
            z_try = float(pos[2]) + float(z_off)
            for yaw_try in yaw_list:
                rot = rot_z(yaw_try) @ rot_x(math.pi)
                for seed_idx, seed in enumerate(seeds):
                    attempt_id += 1
                    (q_try, err_try, ok_try), reason = _call_ik(pos[:2] + (z_try,), rot, seed)
                    payload = {
                        "phase": label,
                        "attempt": attempt_id,
                        "x": round(float(pos[0]), 4),
                        "y": round(float(pos[1]), 4),
                        "z": round(float(z_try), 4),
                        "yaw": round(float(yaw_try), 4),
                        "seed_idx": seed_idx,
                        "seed": [round(float(v), 3) for v in seed],
                        "ok": bool(ok_try),
                        "err": None if err_try is None else round(float(err_try), 5),
                        "z_offset": round(float(z_off), 3),
                        "reason": reason or ("ok" if ok_try else "no_solution"),
                    }
                    _json_log(payload)

                    if ok_try and err_try is not None and err_try <= err_limit:
                        q_sol = q_try
                        err = err_try
                        ok = True
                        solved = True
                        break
                    if ok_try and err_try is None:
                        q_sol = q_try
                        err = 0.0
                        ok = True
                        solved = True
                        break
                    if ok_try and err_try is not None:
                        if best_err is None or err_try < best_err:
                            best_err = err_try
                            best_sol = q_try
                            best_yaw = yaw_try
                            best_z = z_try
                if solved:
                    break
            if solved:
                break

        err_limit = PICK_MAX_IK_ERR_PREGRASP if label == "pregrasp" else 0.15
        if not ok or err is None or err > err_limit:
            if label.startswith("basket"):
                _json_log({
                    "phase": label,
                    "result": "fallback_basket",
                    "attempts": attempt_id,
                    "best_err": None if best_err is None else round(float(best_err), 5),
                    "best_yaw": None if best_yaw is None else round(float(best_yaw), 4),
                    "best_z": None if best_z is None else round(float(best_z), 4),
                })
                print(f"[WARN] IK falla en {label}. Uso pose CESTA fija.")
                use_basket_script = True
                break
            _json_log({
                "phase": label,
                "result": "fail",
                "attempts": attempt_id,
                "best_err": None if best_err is None else round(float(best_err), 5),
                "best_yaw": None if best_yaw is None else round(float(best_yaw), 4),
                "best_z": None if best_z is None else round(float(best_z), 4),
            })
            print(f"[ERROR] IK falla en {label}.")
            return 1
        q_plan.append(q_sol)
        q_seed = q_sol

    _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
    if not _send_joint_trajectory(q_plan[0].tolist(), 4.0):
        return 1
    grasp_duration = 3.0
    if not _send_joint_trajectory(q_plan[1].tolist(), grasp_duration):
        return 1
    time.sleep(grasp_duration + 0.2)
    q_actual = _read_joint_positions(args.partition)
    if not q_actual:
        print("[ERROR] sin joint_states para validar TCP.")
        return 1
    obj_check = _get_object_pose_gz(obj, args.partition) or [wx, wy, wz]
    tcp_pos, _tcp_rot = fk_ur5(q_actual)
    obj_bx, obj_by, obj_bz = world_to_base(obj_check[0], obj_check[1], obj_check[2])
    dist = math.sqrt(
        (tcp_pos[0] - obj_bx) ** 2
        + (tcp_pos[1] - obj_by) ** 2
        + (tcp_pos[2] - obj_bz) ** 2
    )
    if dist > PICK_MAX_TCP_OBJ_DIST:
        print(f"[ERROR] TCP lejos del objeto ({dist:.3f} m). Abortando pick.")
        _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
        return 1
    _run_script(os.path.join(SCRIPTS_DIR, "ur5_close_gripper.sh"))
    part = resolve_gz_partition(args.partition)
    gz_env = build_gz_env(part)
    attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/attach"
    attach_cmd = f"gz topic -t '{attach_topic}' -m gz.msgs.Empty -p 'unused: true'"
    subprocess.run(["bash", "-lc", gz_env + attach_cmd], text=True, capture_output=True)
    _send_joint_trajectory(q_plan[2].tolist(), 3.0)
    if use_basket_script:
        _run_script(os.path.join(SCRIPTS_DIR, "ur5_go_basket_pose.sh"))
        _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
        detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/detach"
        detach_cmd = f"gz topic -t '{detach_topic}' -m gz.msgs.Empty -p 'unused: true'"
        subprocess.run(["bash", "-lc", gz_env + detach_cmd], text=True, capture_output=True)
        print(f"[OK] Pick&place completado para {obj} (soltado en cesta)")
        return 0
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
