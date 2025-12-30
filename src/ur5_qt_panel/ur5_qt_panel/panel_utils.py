#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
# Summary: Helpers and runners shared by the SUPER PRO panel.
from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtGui import QImage

from panel_config import (
    ARM_TRAJ_TOPIC_DEFAULT,
    DEBUG_FRAME_LOG,
    FASTRTPS_PROFILES,
    GZ_PARTITION_FILE,
    LOG_DIR,
    SCRIPTS_DIR,
    OBJECT_POSITIONS,
    OBJECT_POS_PATH,
    TABLE_CALIB_PATH,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_IMAGE_FLIP_X,
    TABLE_IMAGE_FLIP_Y,
    TABLE_IMAGE_SWAP_XY,
    TABLE_PIXEL_AFFINE,
    TABLE_PIXEL_HOMOGRAPHY,
    TABLE_PIXEL_RECT,
    TABLE_CAM_INFO,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
    UR5_BASE_X,
    UR5_BASE_Y,
    UR5_CONTROLLERS_YAML,
    UR5_HOME_DEFAULT,
    UR5_HOME_ENV,
    UR5_REACH_RADIUS,
    WS_DIR,
    ROS_AVAILABLE,
    CvBridge,
    Image,
    Clock,
    JointState,
    MultiThreadedExecutor,
    qos_profile_sensor_data,
    rclpy,
)

ROS_TOPIC_RE = re.compile(r"^/([A-Za-z0-9_]+/)*[A-Za-z0-9_]+$")
ROS_CMD_TIMEOUT = float(os.environ.get("PANEL_ROS_TIMEOUT", "1.5"))
STDBUF_PREFIX = "stdbuf -oL -eL " if shutil.which("stdbuf") else ""
GZ_LOG_FILTERS = [
    r"libEGL warning: egl: failed to create dri2 screen",
    r"libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device\\.",
]


def read_world_name(world_path: str) -> str:
    """Extract the Gazebo world name from the SDF/WORLD file."""
    try:
        text = Path(world_path).read_text(encoding="utf-8", errors="ignore")
        match = re.search(r'<world\s+name="([^"]+)"', text)
        if match:
            return match.group(1)
    except Exception:
        pass
    return Path(world_path).stem


# ------------------------------------------------------------------
# filesystem helpers
# ------------------------------------------------------------------

def ensure_dir(path: str):
    """Create directory if absent."""
    os.makedirs(path, exist_ok=True)


def rotate_log(path: str):
    """Rotate a log file before starting fresh."""
    try:
        if os.path.isfile(path):
            backup = f"{path}.{time.strftime('%Y%m%d_%H%M%S')}.bak"
            shutil.copy2(path, backup)
        with open(path, "w", encoding="utf-8") as f:
            f.write("")
    except Exception:
        pass


def log_to_file(cmd: str, log_path: str, filter_cmd: Optional[str]) -> str:
    """Build the pipeline that filters and logs a command."""
    ensure_dir(os.path.dirname(log_path))
    redir = f">> '{log_path}' 2>&1"
    if filter_cmd:
        return f"{cmd} | {filter_cmd} {redir}"
    return f"{cmd} {redir}"


def build_log_filter_cmd(filters: List[str], unbuffered: bool = False) -> str:
    if not filters:
        return ""
    expr = "|".join(filters)
    quoted = shlex.quote(expr)
    cmd = f"grep -Ev {quoted}"
    if unbuffered and STDBUF_PREFIX:
        return f"{STDBUF_PREFIX}{cmd}"
    return cmd


def now_tag() -> str:
    """Timestamp used for filenames."""
    return time.strftime("%Y%m%d_%H%M%S")


def safe_topic_name(topic: str) -> str:
    """Sanitize ROS topic names for filenames."""
    return re.sub(r"[^a-zA-Z0-9_]+", "_", topic.strip("/"))


def set_led(label, state: str):
    """Update a QLabel-based LED indicator."""
    colors = {
        "off": "#6b7280",
        "on": "#22c55e",
        "warn": "#f59e0b",
        "error": "#ef4444",
    }
    color = colors.get(state, colors["off"])
    label.setFixedSize(14, 14)
    label.setStyleSheet(
        f"background:{color}; border-radius:7px; border:1px solid #374151;"
    )


# ------------------------------------------------------------------
# system stats
# ------------------------------------------------------------------

def read_cpu_times() -> Optional[Tuple[int, int]]:
    try:
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline().strip()
        parts = line.split()
        if not parts or parts[0] != "cpu":
            return None
        nums = [int(x) for x in parts[1:]]
        total = sum(nums)
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
        return total, idle
    except Exception:
        return None


def read_meminfo_kb() -> Optional[Tuple[int, int]]:
    try:
        total = None
        avail = None
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                key, rest = line.split(":", 1)
                value = int(rest.strip().split()[0])
                if key == "MemTotal":
                    total = value
                elif key == "MemAvailable":
                    avail = value
                if total is not None and avail is not None:
                    break
        if total is None or avail is None:
            return None
        return total, avail
    except Exception:
        return None


def read_loadavg() -> Optional[Tuple[float, float, float]]:
    try:
        with open("/proc/loadavg", "r", encoding="utf-8") as f:
            parts = f.read().split()
        return float(parts[0]), float(parts[1]), float(parts[2])
    except Exception:
        return None


def kb_to_gb(kb: int) -> float:
    return kb / 1024.0 / 1024.0


# ------------------------------------------------------------------
# table calibration
# ------------------------------------------------------------------

def _pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    return nx, ny


def _norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = (nx + 0.5) * w
    py = (ny + 0.5) * h
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    return _pixel_to_norm(px, py, w, h)


def norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    return _norm_to_pixel(nx, ny, w, h)


def _apply_homography(mat: List[List[float]], u: float, v: float) -> Optional[Tuple[float, float]]:
    denom = (mat[2][0] * u) + (mat[2][1] * v) + mat[2][2]
    if abs(denom) < 1e-8:
        return None
    x = ((mat[0][0] * u) + (mat[0][1] * v) + mat[0][2]) / denom
    y = ((mat[1][0] * u) + (mat[1][1] * v) + mat[1][2]) / denom
    return x, y


def _invert_3x3(mat: List[List[float]]) -> Optional[List[List[float]]]:
    a, b, c = mat[0]
    d, e, f = mat[1]
    g, h, i = mat[2]
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-10:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def _solve_linear_system(a: List[List[float]], b: List[float]) -> Optional[List[float]]:
    n = len(b)
    mat = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = col
        max_val = abs(mat[col][col])
        for r in range(col + 1, n):
            val = abs(mat[r][col])
            if val > max_val:
                max_val = val
                pivot = r
        if max_val < 1e-10:
            return None
        if pivot != col:
            mat[col], mat[pivot] = mat[pivot], mat[col]
        pivot_val = mat[col][col]
        for c in range(col, n + 1):
            mat[col][c] /= pivot_val
        for r in range(n):
            if r == col:
                continue
            factor = mat[r][col]
            if abs(factor) < 1e-12:
                continue
            for c in range(col, n + 1):
                mat[r][c] -= factor * mat[col][c]
    return [mat[i][n] for i in range(n)]


def compute_homography(
    pixels: List[Tuple[float, float]],
    worlds: List[Tuple[float, float]],
) -> Optional[List[List[float]]]:
    if len(pixels) < 4 or len(worlds) < 4:
        return None
    pts = list(zip(pixels, worlds))[:4]
    a: List[List[float]] = []
    b: List[float] = []
    for (u, v), (x, y) in pts:
        a.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        b.append(x)
        a.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        b.append(y)
    sol = _solve_linear_system(a, b)
    if not sol:
        return None
    return [
        [sol[0], sol[1], sol[2]],
        [sol[3], sol[4], sol[5]],
        [sol[6], sol[7], 1.0],
    ]


def _pixel_to_world_at_z(px: float, py: float, w: int, h: int, z_target: float) -> Optional[Tuple[float, float]]:
    cam = TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    xcam = 1.0
    ycam = -(px - cx) / fx
    zcam = -(py - cy) / fy
    dir_world = (
        rot[0][0] * xcam + rot[0][1] * ycam + rot[0][2] * zcam,
        rot[1][0] * xcam + rot[1][1] * ycam + rot[1][2] * zcam,
        rot[2][0] * xcam + rot[2][1] * ycam + rot[2][2] * zcam,
    )
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    dz = dir_world[2]
    if abs(dz) < 1e-8:
        return None
    t = (float(z_target) - oz) / dz
    if t <= 0.0:
        return None
    xw = ox + t * dir_world[0]
    yw = oy + t * dir_world[1]
    return xw, yw


def pixel_to_table_xy(px: int, py: int, w: int, h: int, z_target: Optional[float] = None) -> Tuple[float, float]:
    if w <= 0 or h <= 0:
        return 0.0, 0.0
    if z_target is not None:
        out = _pixel_to_world_at_z(float(px), float(py), w, h, float(z_target))
        if out:
            return out
    if TABLE_PIXEL_HOMOGRAPHY:
        nx, ny = _pixel_to_norm(float(px), float(py), w, h)
        out = _apply_homography(TABLE_PIXEL_HOMOGRAPHY, nx, ny)
        if out:
            return out
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        x = (a * px) + (b * py) + c
        y = (d * px) + (e * py) + f
        return x, y
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        nx, ny = _pixel_to_norm(px, py, w, h)
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (x2 - x1) / max(1e-6, (npx2 - npx1))
        sy = (y2 - y1) / max(1e-6, (npy2 - npy1))
        x = x1 + (nx - npx1) * sx
        y = y1 + (ny - npy1) * sy
        return x, y
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    x = TABLE_CENTER_X + nx * TABLE_SIZE_X
    y = TABLE_CENTER_Y + ny * TABLE_SIZE_Y
    return x, y


def world_xyz_to_pixel(x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    if w <= 0 or h <= 0:
        return None
    cam = TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    vx = x - ox
    vy = y - oy
    vz = z - oz
    xcam = rot[0][0] * vx + rot[1][0] * vy + rot[2][0] * vz
    ycam = rot[0][1] * vx + rot[1][1] * vy + rot[2][1] * vz
    zcam = rot[0][2] * vx + rot[1][2] * vy + rot[2][2] * vz
    if xcam <= 1e-6:
        return None
    u = cx - (ycam / xcam) * fx
    v = cy - (zcam / xcam) * fy
    return int(max(0, min(w - 1, u))), int(max(0, min(h - 1, v)))


def table_xy_to_pixel(x: float, y: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    if w <= 0 or h <= 0:
        return None
    if TABLE_PIXEL_HOMOGRAPHY:
        inv = _invert_3x3(TABLE_PIXEL_HOMOGRAPHY)
        if not inv:
            return None
        out = _apply_homography(inv, float(x), float(y))
        if out:
            return _norm_to_pixel(out[0], out[1], w, h)
        return None
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        det = (a * e) - (b * d)
        if abs(det) < 1e-8:
            return None
        inv = [[e / det, -b / det], [-d / det, a / det]]
        tx = x - c
        ty = y - f
        px = (inv[0][0] * tx) + (inv[0][1] * ty)
        py = (inv[1][0] * tx) + (inv[1][1] * ty)
        return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (npx2 - npx1) / max(1e-6, (x2 - x1))
        sy = (npy2 - npy1) / max(1e-6, (y2 - y1))
        nx = npx1 + (x - x1) * sx
        ny = npy1 + (y - y1) * sy
        return _norm_to_pixel(nx, ny, w, h)
    nx = (x - TABLE_CENTER_X) / max(1e-6, TABLE_SIZE_X)
    ny = (y - TABLE_CENTER_Y) / max(1e-6, TABLE_SIZE_Y)
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = int((nx + 0.5) * w)
    py = int((ny + 0.5) * h)
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def load_table_calib() -> Optional[object]:
    if not os.path.isfile(TABLE_CALIB_PATH):
        return None
    try:
        with open(TABLE_CALIB_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        global TABLE_CAM_INFO
        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        TABLE_CAM_INFO = None
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
        cam = data.get("camera") if isinstance(data, dict) else None
        if isinstance(cam, dict):
            pos = cam.get("position")
            rot = cam.get("rotation")
            fx = cam.get("fx")
            fy = cam.get("fy")
            cx = cam.get("cx")
            cy = cam.get("cy")
            width = cam.get("width")
            height = cam.get("height")
            if (
                isinstance(pos, list) and len(pos) == 3
                and isinstance(rot, list) and len(rot) == 3
                and all(isinstance(row, list) and len(row) == 3 for row in rot)
            ):
                try:
                    TABLE_CAM_INFO = {
                        "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                        "rotation": [[float(v) for v in row] for row in rot],
                        "fx": float(fx),
                        "fy": float(fy),
                        "cx": float(cx),
                        "cy": float(cy),
                        "width": int(width),
                        "height": int(height),
                    }
                except (TypeError, ValueError):
                    TABLE_CAM_INFO = None
        mode = data.get("mode")
        if mode == "rect":
            p1 = data.get("p1")
            p2 = data.get("p2")
            w1 = data.get("w1")
            w2 = data.get("w2")
            if all(isinstance(v, list) and len(v) == 2 for v in (p1, p2, w1, w2)):
                dx = float(p2[0]) - float(p1[0])
                dy = float(p2[1]) - float(p1[1])
                if abs(dx) < 50 or abs(dy) < 50:
                    return None
                sx = (float(w2[0]) - float(w1[0])) / dx
                sy = (float(w2[1]) - float(w1[1])) / dy
                if abs(sx) < 0.0003 or abs(sy) < 0.0003 or abs(sx) > 0.02 or abs(sy) > 0.02:
                    return None
                TABLE_PIXEL_RECT = {"p1": tuple(p1), "p2": tuple(p2), "w1": tuple(w1), "w2": tuple(w2)}
                return TABLE_PIXEL_RECT
        if mode == "homography":
            mat = data.get("h")
            if (
                isinstance(mat, list)
                and len(mat) == 3
                and all(isinstance(row, list) and len(row) == 3 for row in mat)
            ):
                mat = [[float(v) for v in row] for row in mat]
                if _invert_3x3(mat):
                    TABLE_PIXEL_HOMOGRAPHY = mat
                    return TABLE_PIXEL_HOMOGRAPHY
        affine = data.get("affine")
        if (
            isinstance(affine, list)
            and len(affine) == 2
            and all(isinstance(row, list) and len(row) == 3 for row in affine)
        ):
            mat = [[float(v) for v in row] for row in affine]
            det = (mat[0][0] * mat[1][1]) - (mat[0][1] * mat[1][0])
            if abs(det) < 1e-8:
                return None
            TABLE_PIXEL_AFFINE = mat
            return TABLE_PIXEL_AFFINE
    except Exception:
        return None
    return None


def nearest_table_object(x: float, y: float) -> str:
    best = None
    best_d = 1e9
    for name, (ox, oy, _oz) in OBJECT_POSITIONS.items():
        d = (ox - x) ** 2 + (oy - y) ** 2
        if d < best_d:
            best = name
            best_d = d
    return best or "pieza_pick_mesa"


def get_object_positions() -> Dict[str, Tuple[float, float, float]]:
    return dict(OBJECT_POSITIONS)


def load_object_positions() -> None:
    if not os.path.isfile(OBJECT_POS_PATH):
        return
    try:
        with open(OBJECT_POS_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            for name, vals in data.items():
                if (
                    isinstance(vals, list)
                    and len(vals) == 3
                    and all(isinstance(v, (int, float)) for v in vals)
                ):
                    OBJECT_POSITIONS[name] = (float(vals[0]), float(vals[1]), float(vals[2]))
    except Exception:
        pass


def save_object_positions() -> None:
    data = {k: [float(v[0]), float(v[1]), float(v[2])] for k, v in OBJECT_POSITIONS.items()}
    try:
        with open(OBJECT_POS_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass


def load_home_pose() -> List[float]:
    vals = list(UR5_HOME_DEFAULT)
    if os.path.isfile(UR5_HOME_ENV):
        try:
            with open(UR5_HOME_ENV, "r", encoding="utf-8") as f:
                for line in f:
                    if not line.startswith("HOME_POS_"):
                        continue
                    key, _, raw = line.partition("=")
                    raw = raw.strip()
                    if not raw:
                        continue
                    try:
                        idx = int(key.split("_")[-1])
                    except ValueError:
                        continue
                    if 0 <= idx < len(vals):
                        try:
                            vals[idx] = float(raw)
                        except ValueError:
                            continue
        except Exception:
            vals = list(UR5_HOME_DEFAULT)
    if all(abs(v) < 1e-3 for v in vals):
        return list(UR5_HOME_DEFAULT)
    return vals


def object_out_of_reach(x: float, y: float) -> bool:
    dx = x - UR5_BASE_X
    dy = y - UR5_BASE_Y
    return (dx * dx + dy * dy) > (UR5_REACH_RADIUS * UR5_REACH_RADIUS)


def visible_table_object(name: str, position: Tuple[float, float, float]) -> bool:
    """Return True if the object is a table-relevant piece."""
    x, y, _z = position
    margin = 0.15
    half_x = TABLE_SIZE_X / 2.0 + margin
    half_y = TABLE_SIZE_Y / 2.0 + margin
    if abs(x) > half_x or abs(y) > half_y:
        return False
    blacklisted = (
        "upper_arm_link",
        "forearm_link",
        "wrist_",
        "rg2_",
        "tool0",
        "link_",
        "p1v",
        "p2v",
        "p3v",
        "p4v",
    )
    if any(name.startswith(token) for token in blacklisted):
        return False
    return True


def kill_process_group(proc: subprocess.Popen, label: str, log_fn) -> None:
    """Ensure a child process group is terminated, logging the attempt."""
    if proc is None:
        return
    try:
        proc.terminate()
    except Exception:
        pass
    try:
        pgid = os.getpgid(proc.pid)
        if pgid:
            os.killpg(pgid, signal.SIGTERM)
    except Exception:
        pass
    finally:
        log_fn(f"[{label}] Proceso detenido (pid={getattr(proc, 'pid', 'n/a')}).")

def bash_preamble(ws_dir: str) -> str:
    return (
        "set +u; "
        "export AMENT_TRACE_SETUP_FILES=\"${AMENT_TRACE_SETUP_FILES:-}\"; "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE='{FASTRTPS_PROFILES}'; "
        "export RMW_FASTRTPS_USE_SHM=0; "
        "if [ -z \"${RMW_IMPLEMENTATION:-}\" ]; then "
        "  if [ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/x86_64-linux-gnu/librmw_cyclonedds_cpp.so ]; then "
        "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
        "  fi; "
        "fi; "
        "source /opt/ros/jazzy/setup.bash; "
        f"if [ -f '{ws_dir}/install/setup.bash' ]; then source '{ws_dir}/install/setup.bash'; fi; "
        "set -u; "
        f"cd '{ws_dir}'; "
    )


def read_gz_partition_file() -> str:
    try:
        with open(GZ_PARTITION_FILE, "r", encoding="utf-8") as f:
            return f.read().strip()
    except Exception:
        return ""


def resolve_gz_partition(preferred: str = "") -> str:
    if preferred:
        return preferred
    env_part = os.environ.get("GZ_PARTITION", "").strip()
    if env_part:
        return env_part
    return read_gz_partition_file()


def build_gz_env(partition: str = "") -> str:
    gz_ip = os.environ.get("GZ_IP", "").strip()
    gz_transport_ip = os.environ.get("GZ_TRANSPORT_IP", "").strip()
    env = ""
    if gz_ip:
        env += f"export GZ_IP='{gz_ip}' ; "
    if gz_transport_ip:
        env += f"export GZ_TRANSPORT_IP='{gz_transport_ip}' ; "
    if partition:
        env += f"export GZ_PARTITION='{partition}' ; "
    return env


def run_cmd_output(cmd: str, timeout_sec: float = ROS_CMD_TIMEOUT) -> Tuple[str, Optional[str]]:
    """Run a shell command and capture stdout/stderr with timeout."""
    try:
        res = subprocess.run(
            ["bash", "-lc", cmd],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
        )
        return res.stdout or "", None
    except subprocess.TimeoutExpired:
        return "", f"timeout {timeout_sec}s"
    except Exception as exc:
        return "", str(exc)


def with_line_buffer(cmd: str) -> str:
    if STDBUF_PREFIX:
        return f"{STDBUF_PREFIX}{cmd}"
    return cmd


def write_bridge_runtime_yaml(runtime_path: str, world_name: str, base_yaml_path: str):
    ensure_dir(os.path.dirname(runtime_path))
    lines = [
        "# AUTO-GENERATED by Panel SUPER PRO\n",
        f"# world_name = {world_name}\n",
        "\n",
        "- ros_topic_name: /clock\n",
        f"  gz_topic_name: /world/{world_name}/clock\n",
        "  ros_type_name: rosgraph_msgs/msg/Clock\n",
        "  gz_type_name: gz.msgs.Clock\n",
        "  direction: GZ_TO_ROS\n\n",
    ]
    try:
        with open(base_yaml_path, "r", encoding="utf-8") as f:
            base = f.read().strip()
        if base:
            lines.extend(["\n", base, "\n"])
    except Exception as exc:
        lines.append(f"\n# WARN: no pude leer base_yaml: {exc}\n")
    with open(runtime_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))


def gripper_controller_defined() -> bool:
    try:
        with open(UR5_CONTROLLERS_YAML, "r", encoding="utf-8", errors="ignore") as f:
            return "gripper_controller:" in f.read()
    except Exception:
        return False


def list_controllers_state() -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    cmd = bash_preamble(WS_DIR) + "ros2 control list_controllers || true"
    out, err = run_cmd_output(cmd)
    if err:
        return None, err
    states: Dict[str, str] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            name = parts[0]
            state = parts[-1]
            states[name] = state
    return states, None


def list_active_controllers() -> Tuple[Optional[Set[str]], Optional[str]]:
    states, err = list_controllers_state()
    if err or states is None:
        return None, err
    active = {name for name, st in states.items() if st == "active"}
    return active, None


def ros2_control_running() -> bool:
    cmd = bash_preamble(WS_DIR) + "ros2 service list || true"
    out, err = run_cmd_output(cmd)
    if err:
        return False
    if "/controller_manager/list_controllers" in out:
        return True
    nodes_cmd = bash_preamble(WS_DIR) + "ros2 node list || true"
    nodes, err = run_cmd_output(nodes_cmd)
    if err:
        return False
    return "/controller_manager" in nodes


def gz_sim_running() -> bool:
    try:
        res = subprocess.run(
            ["bash", "-lc", "pgrep -f \"gz sim\" >/dev/null 2>&1"],
            check=False,
        )
        return res.returncode == 0
    except Exception:
        return False


def ros_clock_available() -> bool:
    cmd = bash_preamble(WS_DIR) + "ros2 topic list || true"
    out, err = run_cmd_output(cmd)
    if err:
        return False
    return "/clock" in out


def robot_control_available() -> bool:
    return ros2_control_running() or (gz_sim_running() and ros_clock_available())


def detect_arm_trajectory_topic() -> str:
    force_ros2 = os.environ.get("FORCE_ROS2_CONTROL", "0") == "1"
    if not force_ros2 and gz_sim_running():
        return ARM_TRAJ_TOPIC_DEFAULT
    active, err = list_active_controllers()
    if not err and active and "joint_trajectory_controller" in active:
        return "/joint_trajectory_controller/joint_trajectory"
    return ARM_TRAJ_TOPIC_DEFAULT


def parse_ros_topics(raw: str) -> Tuple[List[str], List[str]]:
    items = [t.strip() for t in raw.split() if t.strip()]
    if not items:
        return [], []
    invalid = [t for t in items if not ROS_TOPIC_RE.match(t)]
    valid = [t for t in items if t not in invalid]
    return valid, invalid


# ------------------------------------------------------------------
# cold boot helpers
# ------------------------------------------------------------------

def cold_boot_kill(log_fn):
    log_fn("[COLD] cold_boot_kill -> scripts/kill_all.sh")
    script = os.path.join(SCRIPTS_DIR, "kill_all.sh")
    if not os.path.isfile(script):
        log_fn(f"[COLD] No existe {script}")
        return
    try:
        subprocess.run(["bash", "-lc", f"'{script}' || true"], check=False)
    except Exception as exc:
        log_fn(f"[COLD] ERROR al ejecutar kill_all.sh: {exc}")


# ------------------------------------------------------------------
# command helpers
# ------------------------------------------------------------------

class CmdRunner(QObject):
    line = pyqtSignal(str)
    started = pyqtSignal(str)
    finished = pyqtSignal(str, int)

    def run_stream(self, tag: str, cmd: str, proc_slot: Optional[dict] = None, key: Optional[str] = None):
        def emit(msg: str) -> bool:
            try:
                self.line.emit(msg)
                return True
            except RuntimeError:
                return False

        def worker():
            if not emit(f"[{tag}] $ {cmd}"):
                return
            try:
                self.started.emit(tag)
                process = subprocess.Popen(
                    ["bash", "-lc", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                if proc_slot is not None and key is not None:
                    proc_slot[key] = process
                if process.stdout:
                    for line in process.stdout:
                        if not emit(f"[{tag}] {line.rstrip()}"):
                            return
                rc = process.wait()
                emit(f"[{tag}] [EXIT] rc={rc}")
                self.finished.emit(tag, rc)
            except Exception as exc:
                emit(f"[{tag}] [ERROR] {exc}")
                self.finished.emit(tag, -1)

        threading.Thread(target=worker, daemon=True).start()


class RosWorker(QObject):
    log = pyqtSignal(str)
    image = pyqtSignal(str, QImage, int, int, float)
    joint_state = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._running = False
        self._node = None
        self._exec = None
        self._bridge = None
        self._subs: Dict[str, object] = {}
        self._fps: Dict[str, List[float]] = {}
        self._last_clock_wall = 0.0
        self._last_emit: Dict[str, float] = {}
        self._joint_sub = None
        self._joint_topic = ""
        self._last_joint_emit = 0.0
        self._joint_emit_interval = 0.1
        try:
            max_fps = float(os.environ.get("PANEL_MAX_FPS", "12"))
            if max_fps <= 0:
                max_fps = 12.0
        except ValueError:
            max_fps = 12.0
        self._min_emit_interval = 1.0 / max_fps

    def start(self):
        if not ROS_AVAILABLE:
            self.log.emit("[ROS] No disponible (faltan deps).")
            return
        with self._lock:
            if self._running:
                return
            self._running = True
        threading.Thread(target=self._thread_main, daemon=True).start()

    def stop(self):
        with self._lock:
            self._running = False
        try:
            if self._node and self._subs:
                for topic, sub in list(self._subs.items()):
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception:
                        pass
                self._subs.clear()
            if self._node and self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception:
                    pass
                self._joint_sub = None
                self._joint_topic = ""
            if self._exec and self._node:
                try:
                    self._exec.remove_node(self._node)
                except Exception:
                    pass
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass
        self._node = None
        self._exec = None
        self._bridge = None
        try:
            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def subscribe_image(self, topic: str):
        if not ROS_AVAILABLE:
            return
        topic = topic.strip()
        if not topic:
            return
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return
            if topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception:
                    pass
                self._subs.pop(topic, None)
            try:
                sub = self._node.create_subscription(Image, topic, lambda msg, t=topic: self._on_image(msg, t), qos_profile_sensor_data)
                self._subs[topic] = sub
                self._fps[topic] = [0.0, time.time(), 0.0]
                self.log.emit(f"[ROS] Suscrito a {topic}")
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo {topic}: {exc}")

    def unsubscribe_image(self, topic: str):
        if not ROS_AVAILABLE:
            return
        topic = topic.strip()
        with self._lock:
            if self._node and topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception:
                    pass
                self._subs.pop(topic, None)
                self.log.emit(f"[ROS] Unsubscribe {topic}")

    def subscribe_joint_states(self, topic: str = "/joint_states"):
        if not ROS_AVAILABLE:
            return
        topic = (topic or "").strip()
        if not topic:
            return
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavia.")
                return
            if self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception:
                    pass
                self._joint_sub = None
            try:
                self._joint_topic = topic
                self._joint_sub = self._node.create_subscription(
                    JointState,
                    topic,
                    lambda msg, t=topic: self._on_joint_state(msg, t),
                    qos_profile_sensor_data,
                )
                self.log.emit(f"[ROS] Suscrito a {topic} (joint_states)")
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo joint_states: {exc}")

    def unsubscribe_joint_states(self):
        if not ROS_AVAILABLE:
            return
        with self._lock:
            if self._node and self._joint_sub is not None:
                try:
                    self._node.destroy_subscription(self._joint_sub)
                except Exception:
                    pass
                self._joint_sub = None
                self._joint_topic = ""
                self.log.emit("[ROS] Unsubscribe joint_states")

    def _thread_main(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR rclpy.init: {exc}")
                return
        try:
            self._node = rclpy.create_node("panel_superpro")
            self._bridge = CvBridge()
            self._exec = MultiThreadedExecutor(num_threads=2)
            self._exec.add_node(self._node)
            self._node.create_subscription(Clock, "/clock", lambda _msg: self._update_clock(), qos_profile_sensor_data)
            self.log.emit("[ROS] OK: nodo listo.")
        except Exception as exc:
            self.log.emit(f"[ROS] ERROR creando nodo/executor: {exc}")
            return
        while True:
            with self._lock:
                if not self._running:
                    break
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception as exc:
                self.log.emit(f"[ROS] WARN spin_once: {exc}")
                time.sleep(0.1)

    def _update_clock(self):
        with self._lock:
            self._last_clock_wall = time.time()

    def clock_alive(self) -> Tuple[bool, float]:
        with self._lock:
            if self._last_clock_wall <= 0.0:
                return False, float("inf")
            age = time.time() - self._last_clock_wall
            return age < 2.0, age

    def _on_joint_state(self, msg: "JointState", topic: str):
        if not ROS_AVAILABLE:
            return
        now = time.time()
        if (now - self._last_joint_emit) < self._joint_emit_interval:
            return
        self._last_joint_emit = now
        stamp = None
        try:
            if msg.header and (msg.header.stamp.sec or msg.header.stamp.nanosec):
                stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        except Exception:
            stamp = None
        payload = {
            "topic": topic,
            "stamp": stamp or now,
            "source": "ros",
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        self.joint_state.emit(payload)

    def _decode_depth_to_bgr(self, cv_depth):
        import numpy as np
        import cv2
        arr = cv_depth.astype("float32")
        arr[~np.isfinite(arr)] = 0.0
        lo = float(np.percentile(arr, 1.0))
        hi = float(np.percentile(arr, 99.0))
        if hi <= lo:
            hi = lo + 1e-3
        arr = (arr - lo) / (hi - lo)
        arr = np.clip(arr, 0.0, 1.0)
        u8 = (arr * 255.0).astype("uint8")
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

    def _on_image(self, msg: "Image", topic: str):
        if not ROS_AVAILABLE or not self._bridge:
            return
        now = time.time()
        last = self._last_emit.get(topic, 0.0)
        if (now - last) < self._min_emit_interval:
            return
        self._last_emit[topic] = now
        try:
            import cv2
            import numpy as np
            enc = (msg.encoding or "").lower()
            if any(flag in enc for flag in ("32fc1", "16uc1", "mono16")) or "depth" in topic:
                try:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except Exception:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg)
                if cv_depth.ndim == 3:
                    cv_depth = cv_depth[:, :, 0]
                bgr = self._decode_depth_to_bgr(cv_depth)
            else:
                try:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except Exception:
                    w = int(msg.width)
                    h = int(msg.height)
                    step = int(msg.step)
                    data = msg.data
                    if step == w * 3 and len(data) >= h * step:
                        arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
                        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    else:
                        bgr = self._bridge.imgmsg_to_cv2(msg)
            h, w = bgr.shape[:2]
            rec = self._fps.get(topic, [0.0, time.time(), 0.0])
            rec[0] += 1.0
            dt = max(1e-6, time.time() - rec[1])
            if dt >= 1.0:
                rec[2] = rec[0] / dt
                rec[0] = 0.0
                rec[1] = time.time()
            self._fps[topic] = rec
            fps = float(rec[2])
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            qimg = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
            self.image.emit(topic, qimg, w, h, fps)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame {topic}: {exc}")
