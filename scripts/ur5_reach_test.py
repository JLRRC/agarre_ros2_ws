#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_reach_test.py
# Summary: Joint sweep + reach/height test for UR5 with logs.
from __future__ import annotations

import math
import os
import sys
import time
import subprocess

WS_DIR = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
PKG_DIR = os.path.join(WS_DIR, "src", "ur5_qt_panel")
if PKG_DIR not in sys.path:
    sys.path.append(PKG_DIR)

from ur5_qt_panel.panel_config import (  # type: ignore
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_SURFACE_Z,
    UR5_BASE_X,
    UR5_BASE_Y,
    UR5_JOINT_NAMES,
    UR5_REACH_RADIUS,
)
from ur5_qt_panel.panel_utils import (  # type: ignore
    detect_arm_trajectory_topic,
    load_home_pose,
    world_to_base,
)
from ur5_qt_panel.ur5_kinematics import ik_ur5, rot_x, rot_z  # type: ignore

import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _gz_running() -> bool:
    res = subprocess.run(["bash", "-lc", "pgrep -f \"gz sim\" >/dev/null 2>&1"], check=False)
    return res.returncode == 0


def _publish_joint(pub, positions, sec: float):
    msg = JointTrajectory()
    msg.joint_names = list(UR5_JOINT_NAMES)
    pt = JointTrajectoryPoint()
    pt.positions = list(positions)
    sec = max(0.5, float(sec))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    pt.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)
    msg.points = [pt]
    pub.publish(msg)


def _joint_sweep(pub, home, delta_deg: float, move_sec: float, pause_sec: float):
    delta = math.radians(delta_deg)
    for idx, name in enumerate(UR5_JOINT_NAMES):
        for sign in (1.0, -1.0):
            q = list(home)
            q[idx] = q[idx] + sign * delta
            print(f"[TEST] Joint {name} -> {sign:+.0f}{delta_deg:.1f} deg", flush=True)
            _publish_joint(pub, q, move_sec)
            time.sleep(move_sec + pause_sec)


def _find_max_radius(z_world: float, err_limit: float, ang_deg: float = 0.0):
    rot = rot_z(0.0) @ rot_x(math.pi)
    lo = 0.0
    hi = UR5_REACH_RADIUS
    ok_r = 0.0
    q_ok = None
    q_seed = load_home_pose()
    ang = math.radians(ang_deg)
    for _ in range(12):
        r = (lo + hi) / 2.0
        wx = UR5_BASE_X + r * math.cos(ang)
        wy = UR5_BASE_Y + r * math.sin(ang)
        bx, by, bz = world_to_base(wx, wy, z_world)
        q, err, ok = ik_ur5((bx, by, bz), rot, q_seed)
        if ok and err <= err_limit:
            ok_r = r
            q_ok = q
            lo = r
            q_seed = q
        else:
            hi = r
    return ok_r, q_ok


def _find_max_height(err_limit: float):
    rot = rot_z(0.0) @ rot_x(math.pi)
    z_lo = TABLE_SURFACE_Z
    z_hi = float(os.environ.get("TEST_REACH_Z_MAX", f"{TABLE_SURFACE_Z + 0.7:.3f}"))
    ok_z = z_lo
    q_ok = None
    q_seed = load_home_pose()
    for _ in range(12):
        z_mid = (z_lo + z_hi) / 2.0
        bx, by, bz = world_to_base(TABLE_CENTER_X, TABLE_CENTER_Y, z_mid)
        q, err, ok = ik_ur5((bx, by, bz), rot, q_seed)
        if ok and err <= err_limit:
            ok_z = z_mid
            q_ok = q
            z_lo = z_mid
            q_seed = q
        else:
            z_hi = z_mid
    return ok_z, q_ok


def main() -> int:
    if not _gz_running():
        print("[TEST] WARN: Gazebo no parece estar activo.", flush=True)
    topic = detect_arm_trajectory_topic()
    delta_deg = float(os.environ.get("TEST_JOINT_DELTA_DEG", "30"))
    move_sec = float(os.environ.get("TEST_JOINT_MOVE_SEC", "2.0"))
    pause_sec = float(os.environ.get("TEST_JOINT_PAUSE_SEC", "0.4"))
    err_limit = float(os.environ.get("TEST_REACH_IK_ERR", "0.03"))
    z_world = float(os.environ.get("TEST_REACH_Z", f"{TABLE_SURFACE_Z + 0.02:.3f}"))

    rclpy.init(args=None)
    node = rclpy.create_node("ur5_reach_test")
    pub = node.create_publisher(JointTrajectory, topic, 10)

    home = load_home_pose()
    print(f"[TEST] Topic: {topic}", flush=True)
    print(f"[TEST] HOME: {[round(v,3) for v in home]}", flush=True)

    print("[TEST] Barrido de joints...", flush=True)
    _publish_joint(pub, home, move_sec)
    time.sleep(move_sec + pause_sec)
    _joint_sweep(pub, home, delta_deg, move_sec, pause_sec)

    print("[TEST] Buscando limite de distancia en mesa...", flush=True)
    r_max, q_far = _find_max_radius(z_world, err_limit, ang_deg=0.0)
    print(f"[TEST] Distancia max @z={z_world:.3f}: r_max={r_max:.3f}", flush=True)
    if q_far is not None:
        _publish_joint(pub, q_far, move_sec + 1.0)
        time.sleep(move_sec + 1.0 + pause_sec)

    print("[TEST] Buscando limite de altura en centro mesa...", flush=True)
    z_max, q_high = _find_max_height(err_limit)
    print(f"[TEST] Altura max centro: z={z_max:.3f} (h={z_max - TABLE_SURFACE_Z:.3f})", flush=True)
    if q_high is not None:
        _publish_joint(pub, q_high, move_sec + 1.0)
        time.sleep(move_sec + 1.0 + pause_sec)

    print("[TEST] Volviendo HOME.", flush=True)
    _publish_joint(pub, home, move_sec)
    time.sleep(move_sec + pause_sec)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
