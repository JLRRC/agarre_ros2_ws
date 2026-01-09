#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/scripts/ur5_joint_limit_test.py
# Summary: Sweep UR5 joints to measure effective limits from /joint_states.
from __future__ import annotations

import csv
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WS_DIR = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
PKG_DIR = os.path.join(WS_DIR, "src", "ur5_qt_panel")
if PKG_DIR not in sys.path:
    sys.path.append(PKG_DIR)

from ur5_qt_panel.panel_config import LOG_DIR, UR5_JOINT_NAMES  # type: ignore
from ur5_qt_panel.panel_utils import detect_arm_trajectory_topic  # type: ignore


def _now_tag() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _normalize_joint_name(name: str) -> str:
    text = str(name)
    if "::" in text:
        return text.split("::")[-1]
    return text


class JointStateCache:
    def __init__(self, node, topic: str):
        self.msg: Optional[JointState] = None
        self.stamp: float = 0.0
        self.topic = topic
        self.sub = node.create_subscription(
            JointState,
            topic,
            self._on_msg,
            qos_profile_sensor_data,
        )

    def _on_msg(self, msg: JointState) -> None:
        self.msg = msg
        self.stamp = time.time()

    def positions(self) -> Optional[Dict[str, float]]:
        if self.msg is None:
            return None
        names = list(self.msg.name)
        positions = list(self.msg.position)
        out: Dict[str, float] = {}
        for name, pos in zip(names, positions):
            out[_normalize_joint_name(name)] = float(pos)
        return out


def _publish_joint(pub, positions: List[float], move_sec: float) -> None:
    msg = JointTrajectory()
    msg.joint_names = list(UR5_JOINT_NAMES)
    pt = JointTrajectoryPoint()
    pt.positions = list(positions)
    sec = max(0.4, float(move_sec))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    pt.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)
    msg.points = [pt]
    pub.publish(msg)


def _wait_for_joint_state(node, cache: JointStateCache, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cache.msg is not None:
            return True
    return False


def _wait_for_pose(
    node,
    cache: JointStateCache,
    targets: Dict[str, float],
    timeout: float,
    tol: float,
) -> Tuple[bool, float]:
    deadline = time.time() + timeout
    last_err = float("nan")
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        pos_map = cache.positions()
        if not pos_map:
            continue
        errs = []
        for joint in UR5_JOINT_NAMES:
            if joint not in pos_map:
                continue
            errs.append(abs(pos_map[joint] - targets[joint]))
        if errs:
            last_err = max(errs)
            if last_err <= tol:
                return True, last_err
    return False, last_err


def _wait_for_joint(
    node,
    cache: JointStateCache,
    joint: str,
    target: float,
    timeout: float,
    tol: float,
    stall_eps: float,
    stall_sec: float,
) -> Tuple[bool, Optional[float], str]:
    start = time.time()
    last_pos: Optional[float] = None
    last_change = start
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        pos_map = cache.positions()
        if not pos_map or joint not in pos_map:
            continue
        pos = pos_map[joint]
        err = abs(pos - target)
        if err <= tol:
            return True, pos, "ok"
        if last_pos is None or abs(pos - last_pos) > stall_eps:
            last_pos = pos
            last_change = time.time()
        elif time.time() - last_change > stall_sec:
            return False, pos, "stall"
    return False, last_pos, "timeout"


def main() -> int:
    topic = detect_arm_trajectory_topic()
    joint_states_topic = os.environ.get("TEST_JOINT_STATES_TOPIC", "/joint_states")
    step_deg = float(os.environ.get("TEST_LIMIT_STEP_DEG", "5"))
    max_range_deg = float(os.environ.get("TEST_LIMIT_MAX_RANGE_DEG", "180"))
    move_sec = float(os.environ.get("TEST_LIMIT_MOVE_SEC", "1.2"))
    settle_sec = float(os.environ.get("TEST_LIMIT_SETTLE_SEC", "0.15"))
    tol = float(os.environ.get("TEST_LIMIT_TOL_RAD", "0.02"))
    timeout = float(os.environ.get("TEST_LIMIT_TIMEOUT_SEC", "2.5"))
    stall_eps = float(os.environ.get("TEST_LIMIT_STALL_EPS", "0.003"))
    stall_sec = float(os.environ.get("TEST_LIMIT_STALL_SEC", "0.5"))
    out_dir = os.environ.get("TEST_LIMIT_OUT_DIR", LOG_DIR)

    step_deg = max(0.5, step_deg)
    max_range_deg = max(step_deg, max_range_deg)
    step_rad = math.radians(step_deg)
    max_range_rad = math.radians(max_range_deg)
    max_steps = int(max_range_deg / step_deg)

    rclpy.init(args=None)
    node = rclpy.create_node("ur5_joint_limit_test")
    pub = node.create_publisher(JointTrajectory, topic, 10)
    cache = JointStateCache(node, joint_states_topic)

    print(f"[LIMIT] Topic cmd: {topic}", flush=True)
    print(f"[LIMIT] Topic joint_states: {joint_states_topic}", flush=True)
    print(f"[LIMIT] step={step_deg:.1f} deg, max_range={max_range_deg:.1f} deg", flush=True)

    if not _wait_for_joint_state(node, cache, 5.0):
        print("[LIMIT] ERROR: no llegan joint_states.", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pos_map = cache.positions() or {}
    missing = [j for j in UR5_JOINT_NAMES if j not in pos_map]
    if missing:
        print(f"[LIMIT] ERROR: faltan joints en /joint_states: {missing}", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    base_positions = [pos_map[j] for j in UR5_JOINT_NAMES]
    base_targets = {j: pos_map[j] for j in UR5_JOINT_NAMES}

    results: Dict[str, Dict[str, object]] = {}
    samples: Dict[str, Dict[str, List[Dict[str, float]]]] = {}

    try:
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            results[joint] = {}
            samples[joint] = {"pos": [], "neg": []}
            print(f"[LIMIT] Joint {joint}...", flush=True)

            for sign, label in ((1.0, "pos"), (-1.0, "neg")):
                last_ok = base_positions[idx]
                stop_reason = "range_end"
                stop_target = None
                for step in range(1, max_steps + 1):
                    target = base_positions[idx] + sign * step * step_rad
                    if abs(target - base_positions[idx]) > max_range_rad + 1e-6:
                        break
                    cmd_positions = list(base_positions)
                    cmd_positions[idx] = target
                    _publish_joint(pub, cmd_positions, move_sec)
                    ok, actual, reason = _wait_for_joint(
                        node,
                        cache,
                        joint,
                        target,
                        timeout,
                        tol,
                        stall_eps,
                        stall_sec,
                    )
                    actual_val = float(actual) if actual is not None else None
                    err_val = abs(actual_val - float(target)) if actual_val is not None else None
                    samples[joint][label].append(
                        {
                            "target": float(target),
                            "actual": actual_val,
                            "error": err_val,
                        }
                    )
                    if ok and actual_val is not None:
                        last_ok = actual_val
                        time.sleep(settle_sec)
                        continue
                    stop_reason = reason
                    stop_target = target
                    break

                if label == "pos":
                    results[joint]["max"] = float(last_ok)
                    results[joint]["max_reason"] = stop_reason
                    results[joint]["max_target"] = float(stop_target) if stop_target is not None else None
                else:
                    results[joint]["min"] = float(last_ok)
                    results[joint]["min_reason"] = stop_reason
                    results[joint]["min_target"] = float(stop_target) if stop_target is not None else None

                _publish_joint(pub, base_positions, move_sec)
                _wait_for_pose(node, cache, base_targets, move_sec + timeout, max(tol, 0.04))
                time.sleep(settle_sec)
    finally:
        _publish_joint(pub, base_positions, move_sec)
        _wait_for_pose(node, cache, base_targets, move_sec + timeout, max(tol, 0.04))

    stamp = _now_tag()
    _ensure_dir(out_dir)
    json_path = os.path.join(out_dir, f"ur5_joint_limits_{stamp}.json")
    csv_path = os.path.join(out_dir, f"ur5_joint_limits_{stamp}.csv")

    payload = {
        "timestamp": stamp,
        "topic_cmd": topic,
        "joint_states_topic": joint_states_topic,
        "step_deg": step_deg,
        "max_range_deg": max_range_deg,
        "move_sec": move_sec,
        "timeout_sec": timeout,
        "tolerance_rad": tol,
        "stall_eps": stall_eps,
        "stall_sec": stall_sec,
        "base_positions": {j: float(v) for j, v in zip(UR5_JOINT_NAMES, base_positions)},
        "results": results,
        "samples": samples,
    }

    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)

    with open(csv_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "joint",
                "min_rad",
                "max_rad",
                "min_deg",
                "max_deg",
                "min_reason",
                "max_reason",
            ]
        )
        for joint in UR5_JOINT_NAMES:
            info = results.get(joint, {})
            min_rad = float(info.get("min", float("nan")))
            max_rad = float(info.get("max", float("nan")))
            writer.writerow(
                [
                    joint,
                    f"{min_rad:.6f}",
                    f"{max_rad:.6f}",
                    f"{math.degrees(min_rad):.2f}",
                    f"{math.degrees(max_rad):.2f}",
                    info.get("min_reason", ""),
                    info.get("max_reason", ""),
                ]
            )

    print(f"[LIMIT] Guardado JSON: {json_path}", flush=True)
    print(f"[LIMIT] Guardado CSV: {csv_path}", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
