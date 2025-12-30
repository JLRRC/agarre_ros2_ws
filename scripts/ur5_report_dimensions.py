#!/usr/bin/env python3
"""Report UR5 kinematics parameters used as link dimensions."""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path


KIN_KEYS = ("d1", "a2", "a3", "d4", "d5", "d6")


def _find_kinematics_yaml() -> str:
    env_path = os.environ.get("UR5_KINEMATICS_YAML", "")
    if env_path and os.path.isfile(env_path):
        return env_path

    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "ur_description"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        prefix = ""
    if prefix:
        candidate = os.path.join(prefix, "share", "ur_description", "config", "ur5", "default_kinematics.yaml")
        if os.path.isfile(candidate):
            return candidate

    ros_distro = os.environ.get("ROS_DISTRO", "").strip()
    if ros_distro:
        candidate = os.path.join(
            "/opt/ros",
            ros_distro,
            "share",
            "ur_description",
            "config",
            "ur5",
            "default_kinematics.yaml",
        )
        if os.path.isfile(candidate):
            return candidate

    ws_dir = os.environ.get("WS_DIR") or os.path.expanduser("~/TFM/agarre_ros2_ws")
    for root in (Path(ws_dir) / "install", Path(ws_dir) / "src"):
        if not root.is_dir():
            continue
        for path in root.rglob("default_kinematics.yaml"):
            if "ur5" in path.parts:
                return str(path)
    return ""


def _parse_kinematics(path: str) -> dict:
    values = {}
    pattern = re.compile(r"^([A-Za-z0-9_]+)\s*:\s*([-+0-9.eE]+)\s*$")
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            lines = handle.readlines()
    except OSError:
        return {}

    for raw in lines:
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        match = pattern.match(line)
        if not match:
            continue
        key, num = match.groups()
        if key not in KIN_KEYS:
            continue
        try:
            values[key] = float(num)
        except ValueError:
            continue
    if values:
        return values

    sections = {}
    current = None
    section_re = re.compile(r"^\s{2}([A-Za-z0-9_]+):\s*$")
    value_re = re.compile(r"^\s{4}([xyz]):\s*([-+0-9.eE]+)\s*$")
    for raw in lines:
        line = raw.split("#", 1)[0].rstrip()
        if not line:
            continue
        match = section_re.match(line)
        if match:
            current = match.group(1)
            sections.setdefault(current, {})
            continue
        match = value_re.match(line)
        if match and current:
            axis, num = match.groups()
            try:
                sections[current][axis] = float(num)
            except ValueError:
                continue

    derived = {}
    if "shoulder" in sections and "z" in sections["shoulder"]:
        derived["d1"] = sections["shoulder"]["z"]
    if "forearm" in sections and "x" in sections["forearm"]:
        derived["a2"] = sections["forearm"]["x"]
    if "wrist_1" in sections:
        if "x" in sections["wrist_1"]:
            derived["a3"] = sections["wrist_1"]["x"]
        if "z" in sections["wrist_1"]:
            derived["d4"] = sections["wrist_1"]["z"]
    if "wrist_2" in sections and "y" in sections["wrist_2"]:
        derived["d5"] = sections["wrist_2"]["y"]
    if "wrist_3" in sections and "y" in sections["wrist_3"]:
        derived["d6"] = sections["wrist_3"]["y"]
    return derived


def main() -> int:
    path = _find_kinematics_yaml()
    if not path:
        print("[ROBOT] DIM: No pude localizar default_kinematics.yaml (ur_description).")
        return 0
    values = _parse_kinematics(path)
    if not values:
        print(f"[ROBOT] DIM: No pude leer parametros en {path}.")
        return 0
    print(f"[ROBOT] DIM: kinematics file: {path}")
    for key in KIN_KEYS:
        if key in values:
            print(f"[ROBOT] DIM: {key} = {values[key]:.6f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
