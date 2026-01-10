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
import math
try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtGui import QImage
from .logging_utils import timestamped_line

print(timestamped_line(f"[PANEL] panel_utils loaded from: {os.path.abspath(__file__)}"))

try:
    # Import correcto cuando ur5_qt_panel es un paquete (colcon/ament)
    from .panel_config import (
        ARM_TRAJ_TOPIC_DEFAULT,
        ATTACHABLE_OBJECTS,
        DEBUG_FRAME_LOG,
        FASTRTPS_PROFILES,
        GZ_PARTITION_FILE,
        GZ_WORLD,
        GRIPPER_ATTACH_PREFIX,
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
        TABLE_OBJECT_XY_MARGIN,
        TABLE_OBJECT_Z_MIN,
        TABLE_OBJECT_Z_MAX,
        TABLE_OBJECT_WHITELIST,
        UR5_BASE_X,
        UR5_BASE_Y,
        UR5_BASE_Z,
        UR5_CONTROLLERS_YAML,
        UR5_HOME_DEFAULT,
        UR5_HOME_ENV,
        UR5_MODEL_NAME,
        UR5_REACH_RADIUS,
        WS_DIR,
        BASE_FRAME,
        WORLD_FRAME,
        ROS_AVAILABLE,
        CvBridge,
        Image,
        Clock,
        JointState,
        SingleThreadedExecutor,
        qos_profile_sensor_data,
        rclpy,
    )
except Exception:
    # Fallback si alguien ejecuta módulos fuera del contexto de paquete
        from .panel_config import (  # type: ignore
            ARM_TRAJ_TOPIC_DEFAULT,
            ATTACHABLE_OBJECTS,
            DEBUG_FRAME_LOG,
            FASTRTPS_PROFILES,
            GZ_PARTITION_FILE,
            GZ_WORLD,
            GRIPPER_ATTACH_PREFIX,
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
            TABLE_OBJECT_XY_MARGIN,
            TABLE_OBJECT_Z_MIN,
            TABLE_OBJECT_Z_MAX,
            TABLE_OBJECT_WHITELIST,
            UR5_BASE_X,
            UR5_BASE_Y,
            UR5_BASE_Z,
            UR5_CONTROLLERS_YAML,
            UR5_HOME_DEFAULT,
            UR5_HOME_ENV,
            UR5_MODEL_NAME,
            UR5_REACH_RADIUS,
            WS_DIR,
            BASE_FRAME,
            WORLD_FRAME,
            ROS_AVAILABLE,
            CvBridge,
            Image,
            Clock,
            JointState,
            SingleThreadedExecutor,
            qos_profile_sensor_data,
            rclpy,
        )

try:
    from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, TransformStamped
    from builtin_interfaces.msg import Time as BuiltinTime
    from controller_manager_msgs.srv import ListControllers
    from std_msgs.msg import Empty, String
    from tf2_ros import (
        Buffer,
        TransformListener,
        StaticTransformBroadcaster,
        LookupException,
        ConnectivityException,
        ExtrapolationException,
        TransformRegistration,
        TypeException,
    )
    from tf2_msgs.msg import TFMessage
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    try:
        import tf2_geometry_msgs  # noqa: F401
    except Exception:
        tf2_geometry_msgs = None  # type: ignore
except Exception:
    PointStamped = None  # type: ignore
    PoseStamped = None  # type: ignore
    Quaternion = None  # type: ignore
    TransformStamped = None  # type: ignore
    BuiltinTime = None  # type: ignore
    tf2_geometry_msgs = None  # type: ignore
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore
    StaticTransformBroadcaster = None  # type: ignore
    LookupException = Exception  # type: ignore
    ConnectivityException = Exception  # type: ignore
    ExtrapolationException = Exception  # type: ignore
    TransformRegistration = None  # type: ignore
    TypeException = Exception  # type: ignore
    Duration = None  # type: ignore
    Time = None  # type: ignore
    Time = None  # type: ignore
    TFMessage = None  # type: ignore
    Empty = None  # type: ignore
    String = None  # type: ignore
    ListControllers = None  # type: ignore
    QoSProfile = None  # type: ignore
    ReliabilityPolicy = None  # type: ignore
    DurabilityPolicy = None  # type: ignore
    HistoryPolicy = None  # type: ignore
    ExternalShutdownException = None  # type: ignore
    Parameter = None  # type: ignore


ROS_TOPIC_RE = re.compile(r"^/([A-Za-z0-9_]+/)*[A-Za-z0-9_]+$")
ROS_CMD_TIMEOUT = float(os.environ.get("PANEL_ROS_TIMEOUT", "1.5"))
STDBUF_PREFIX = "stdbuf -oL -eL " if shutil.which("stdbuf") else ""
GZ_LOG_FILTERS = [
    r"libEGL warning: egl: failed to create dri2 screen",
    r"libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device\\.",
]

TELEPORT_BLOCK_PATTERNS = ("set_pose", "set_entity_pose", "teleport")

# Lock to guard concurrent access to OBJECT_POSITIONS (used across threads/UI callbacks).
OBJECT_POS_LOCK = threading.Lock()

_TF_TRANSFORM_WARN_LAST: Dict[str, float] = {}
_TF_TRANSFORM_WARN_COUNT: Dict[str, int] = {}
_TF_TRANSFORM_WARN_PERIOD = 5.0


def _log_tf_transform_warning(context: str, exc: Exception) -> None:
    global _TF_TRANSFORM_WARN_LAST, _TF_TRANSFORM_WARN_COUNT
    now = time.monotonic()
    key = f"{context}:{exc.__class__.__name__}"
    _TF_TRANSFORM_WARN_COUNT[key] = _TF_TRANSFORM_WARN_COUNT.get(key, 0) + 1
    last = _TF_TRANSFORM_WARN_LAST.get(key, 0.0)
    if (now - last) < _TF_TRANSFORM_WARN_PERIOD:
        return
    _TF_TRANSFORM_WARN_LAST[key] = now
    count = _TF_TRANSFORM_WARN_COUNT.get(key, 0)
    _TF_TRANSFORM_WARN_COUNT[key] = 0
    msg = f"[TRACE][TF] {context} failed ({count}): {exc}"
    print(timestamped_line(msg), flush=True)


def run_cmd(cmd: str, timeout: Optional[float] = None, capture_output: bool = True, check: bool = False, env: Optional[dict] = None):
    """Ejecuta un comando de shell de forma centralizada.

    Devuelve el objeto `subprocess.CompletedProcess` para unificar manejo de errores y timeouts.
    Las llamadas deben pasar el comando ya preparado (incluyendo `bash_preamble` si es necesario).
    """
    try:
        if timeout is not None:
            res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=capture_output, timeout=timeout, env=env)
        else:
            res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=capture_output, env=env)
        if check and res.returncode != 0:
            raise subprocess.CalledProcessError(res.returncode, cmd, output=res.stdout, stderr=res.stderr)
        return res
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(args=cmd, returncode=124, stdout="", stderr=str(exc))
    except Exception as exc:
        return subprocess.CompletedProcess(args=cmd, returncode=1, stdout="", stderr=str(exc))


def _contains_teleport_keywords(cmd: str) -> bool:
    lowered = cmd.lower()
    return any(kw in lowered for kw in TELEPORT_BLOCK_PATTERNS)

def world_to_base(x, y, z):
    return (x - UR5_BASE_X, y - UR5_BASE_Y, z - UR5_BASE_Z)

def base_to_world(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (x + UR5_BASE_X, y + UR5_BASE_Y, z + UR5_BASE_Z)

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
        return f"{cmd} 2>&1 | {filter_cmd} {redir}"
    return f"{cmd} {redir}"


def build_log_filter_cmd(filters: List[str], unbuffered: bool = False, include_regex: Optional[str] = None) -> str:
    if not filters and not include_regex:
        return ""
    parts = []
    if include_regex:
        parts.append(f"grep -E {shlex.quote(include_regex)}")
    if filters:
        expr = "|".join(filters)
        quoted = shlex.quote(expr)
        parts.append(f"grep -Ev {quoted}")
    cmd = " | ".join(parts)
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
    with OBJECT_POS_LOCK:
        items = list(OBJECT_POSITIONS.items())
    for name, (ox, oy, oz) in items:
        if not visible_table_object(name, (ox, oy, oz)):
            continue
        d = (ox - x) ** 2 + (oy - y) ** 2
        if d < best_d:
            best = name
            best_d = d
    if best:
        return best
    with OBJECT_POS_LOCK:
        if "pick_demo" in OBJECT_POSITIONS:
            return "pick_demo"
        if "pieza_pick_mesa" in OBJECT_POSITIONS:
            return "pieza_pick_mesa"
    return ""


def get_object_positions() -> Dict[str, Tuple[float, float, float]]:
    with OBJECT_POS_LOCK:
        return dict(OBJECT_POSITIONS)


def get_object_position(name: str) -> Optional[Tuple[float, float, float]]:
    with OBJECT_POS_LOCK:
        return OBJECT_POSITIONS.get(name)


def set_object_position(name: str, position: Tuple[float, float, float]) -> None:
    x, y, z = float(position[0]), float(position[1]), float(position[2])
    with OBJECT_POS_LOCK:
        OBJECT_POSITIONS[name] = (x, y, z)


def bulk_update_object_positions(updates: Dict[str, Tuple[float, float, float]], allow_new: bool = False) -> int:
    """Update multiple object poses with locking. Skips unknown objects unless allow_new is True."""
    updated = 0
    with OBJECT_POS_LOCK:
        for name, pos in updates.items():
            if not allow_new and name not in OBJECT_POSITIONS:
                continue
            try:
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            except Exception:
                continue
            OBJECT_POSITIONS[name] = (x, y, z)
            updated += 1
    return updated


def remove_object_position(name: str) -> None:
    with OBJECT_POS_LOCK:
        OBJECT_POSITIONS.pop(name, None)


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


def get_object_pose_gz(obj_name: str, partition: str = "") -> Optional[Tuple[float, float, float]]:
    """Obtiene la pose (x,y,z) y orientación (quaternion) de un modelo en Gazebo usando /pose/info."""
    if not ROS_AVAILABLE or TFMessage is None:
        return None
    node = _create_graph_node("panel_pose_probe")
    if node is None:
        return None
    target: Dict[str, object] = {}

    def _on_pose(msg: "TFMessage"):
        nonlocal target
        for tf in getattr(msg, "transforms", []):
            name = getattr(tf, "child_frame_id", "") or ""
            if name == obj_name or name.startswith(f"{obj_name}::"):
                t = tf.transform.translation
                r = tf.transform.rotation
                target = {
                    "position": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
                    "orientation": {"x": float(r.x), "y": float(r.y), "z": float(r.z), "w": float(r.w)},
                }
                break

    topic = f"/world/{GZ_WORLD}/pose/info"
    try:
        sub = node.create_subscription(TFMessage, topic, _on_pose, qos_profile_sensor_data)
    except Exception:
        try:
            node.destroy_node()
        except Exception:
            pass
        return None
    end = time.time() + 0.6
    while time.time() < end and not target:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        node.destroy_subscription(sub)
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    return target or None


def load_object_positions() -> None:
    if not os.path.isfile(OBJECT_POS_PATH):
        return
    try:
        with open(OBJECT_POS_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            with OBJECT_POS_LOCK:
                for name, vals in data.items():
                    if (
                        isinstance(vals, list)
                        and len(vals) == 3
                        and all(isinstance(v, (int, float)) for v in vals)
                    ):
                        OBJECT_POSITIONS[name] = (float(vals[0]), float(vals[1]), float(vals[2]))
                if "pieza_pick_mesa" in data and "pick_demo" not in data:
                    OBJECT_POSITIONS["pick_demo"] = OBJECT_POSITIONS["pieza_pick_mesa"]
                if "pieza_pick_mesa" in OBJECT_POSITIONS and "pick_demo" in OBJECT_POSITIONS:
                    OBJECT_POSITIONS.pop("pieza_pick_mesa", None)
    except Exception:
        pass


def save_object_positions() -> None:
    with OBJECT_POS_LOCK:
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

def save_home_pose(joint_values: List[float]) -> None:
    """Guarda la pose HOME actual en el archivo ur5_home_pose.env."""
    from .panel_config import UR5_HOME_ENV
    try:
        with open(UR5_HOME_ENV, "w", encoding="utf-8") as f:
            f.write("# Pose HOME del UR5, generada desde el panel\n")
            for idx, val in enumerate(joint_values):
                f.write(f"HOME_POS_{idx}={val}\n")
    except Exception as e:
        print(timestamped_line(f"[ERROR] No se pudo guardar la pose HOME: {e}"), flush=True)


def object_out_of_reach(x: float, y: float) -> bool:
    dx = x - UR5_BASE_X
    dy = y - UR5_BASE_Y
    return (dx * dx + dy * dy) > (UR5_REACH_RADIUS * UR5_REACH_RADIUS)


def world_to_base(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (x - UR5_BASE_X, y - UR5_BASE_Y, z - UR5_BASE_Z)


def visible_table_object(name: str, position: Tuple[float, float, float]) -> bool:
    """Return True if the object is a table-relevant piece."""
    x, y, z = position
    if TABLE_OBJECT_WHITELIST and name not in TABLE_OBJECT_WHITELIST:
        if name == "pick_demo" and "pieza_pick_mesa" in TABLE_OBJECT_WHITELIST:
            pass
        elif name == "pieza_pick_mesa" and "pick_demo" in TABLE_OBJECT_WHITELIST:
            pass
        else:
            return False
    half_x = TABLE_SIZE_X / 2.0 + TABLE_OBJECT_XY_MARGIN
    half_y = TABLE_SIZE_Y / 2.0 + TABLE_OBJECT_XY_MARGIN
    dx = x - TABLE_CENTER_X
    dy = y - TABLE_CENTER_Y
    if abs(dx) > half_x or abs(dy) > half_y:
        return False
    if z < TABLE_OBJECT_Z_MIN or z > TABLE_OBJECT_Z_MAX:
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
    res = run_cmd(cmd, timeout=timeout_sec, capture_output=True)
    if res.returncode == 124:
        return "", f"timeout {timeout_sec}s"
    if res.stderr:
        return res.stdout or "", res.stderr
    return res.stdout or "", None


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
        f"- ros_topic_name: /world/{world_name}/pose/info\n",
        f"  gz_topic_name: /world/{world_name}/pose/info\n",
        "  ros_type_name: tf2_msgs/msg/TFMessage\n",
        "  gz_type_name: gz.msgs.Pose_V\n",
        "  direction: GZ_TO_ROS\n\n",
    ]
    try:
        with open(base_yaml_path, "r", encoding="utf-8") as f:
            base = f.read().strip()
        if base:
            lines.extend(["\n", base, "\n"])
    except Exception as exc:
        lines.append(f"\n# WARN: no pude leer base_yaml: {exc}\n")
    # Add attach/detach topics for objects that expose detachable joints.
    for name in sorted(ATTACHABLE_OBJECTS):
        lines.extend([
            f"- ros_topic_name: {GRIPPER_ATTACH_PREFIX}/{name}/attach\n",
            f"  gz_topic_name: {GRIPPER_ATTACH_PREFIX}/{name}/attach\n",
            "  ros_type_name: std_msgs/msg/Empty\n",
            "  gz_type_name: gz.msgs.Empty\n",
            "  direction: ROS_TO_GZ\n\n",
            f"- ros_topic_name: {GRIPPER_ATTACH_PREFIX}/{name}/detach\n",
            f"  gz_topic_name: {GRIPPER_ATTACH_PREFIX}/{name}/detach\n",
            "  ros_type_name: std_msgs/msg/Empty\n",
            "  gz_type_name: gz.msgs.Empty\n",
            "  direction: ROS_TO_GZ\n\n",
        ])
    with open(runtime_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))


def gripper_controller_defined() -> bool:
    try:
        with open(UR5_CONTROLLERS_YAML, "r", encoding="utf-8", errors="ignore") as f:
            return "gripper_controller:" in f.read()
    except Exception:
        return False


def list_controllers_state() -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    if not ROS_AVAILABLE or ListControllers is None:
        return None, "ROS no disponible"
    node = _create_graph_node("panel_ctrl_probe")
    if node is None:
        return None, "node unavailable"
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=0.5):
        node.destroy_node()
        return None, "service unavailable"
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.6)
    if not future.done():
        node.destroy_node()
        return None, "timeout"
    result = future.result()
    states: Dict[str, str] = {}
    if result and hasattr(result, "controller"):
        for ctrl in result.controller:
            states[str(ctrl.name)] = str(ctrl.state)
    node.destroy_node()
    return states, None


def list_active_controllers() -> Tuple[Optional[Set[str]], Optional[str]]:
    states, err = list_controllers_state()
    if err or states is None:
        return None, err
    active = {name for name, st in states.items() if st == "active"}
    return active, None


def ros2_control_running() -> bool:
    node = _create_graph_node("panel_ctrl_check")
    if node is None:
        return False
    try:
        services = node.get_service_names_and_types()
        return any(name == "/controller_manager/list_controllers" for name, _ in services)
    except Exception:
        return False
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def _run_cmd_rc(cmd: str, timeout_sec: float = 2.0) -> Tuple[int, str]:
    res = run_cmd(cmd, timeout=timeout_sec, capture_output=True)
    return res.returncode, res.stdout or res.stderr or ""


def gz_sim_status() -> Tuple[bool, str]:
    cmd_proc = (
        "pgrep -af 'gz sim' >/dev/null 2>&1 || "
        "pgrep -af 'gzserver' >/dev/null 2>&1 || "
        "pgrep -af 'ign gazebo' >/dev/null 2>&1"
    )
    rc_proc, _ = _run_cmd_rc(cmd_proc, timeout_sec=1.2)
    if rc_proc == 0:
        return True, "proc"
    return False, "none"


def gz_sim_running() -> bool:
    ok, _reason = gz_sim_status()
    return ok


def bridge_status() -> Tuple[bool, str]:
    node = _create_graph_node("panel_bridge_check")
    if node is None:
        return False, "node"
    try:
        names = node.get_node_names_and_namespaces()
        for name, _ns in names:
            if name in ("parameter_bridge", "ros_gz_bridge"):
                return True, "rosnode"
        return False, "none"
    except Exception:
        return False, "err"
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def clock_status() -> Tuple[bool, str]:
    node = _create_graph_node("panel_clock_check")
    if node is None:
        return False, "node"
    try:
        topics = node.get_topic_names_and_types()
        has_clock = any(name == "/clock" for name, _ in topics)
        if not has_clock:
            return False, "none"
        pubs = node.get_publishers_info_by_topic("/clock")
        if pubs:
            return True, "publisher"
        return True, "topic"
    except Exception:
        return False, "err"
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def _create_graph_node(name: str):
    if not ROS_AVAILABLE:
        return None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except Exception:
        return None
    try:
        return rclpy.create_node(name)
    except Exception:
        return None


def ros_clock_available() -> bool:
    ok, _reason = clock_status()
    return ok


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
    res = run_cmd(f"'{script}' || true", timeout=10, capture_output=True)
    if res.returncode != 0:
        log_fn(f"[COLD] ERROR al ejecutar kill_all.sh: {res.stderr or res.stdout}")


# ------------------------------------------------------------------
# command helpers
# ------------------------------------------------------------------

class CmdRunner(QObject):
    line = pyqtSignal(str)
    started = pyqtSignal(str)
    finished = pyqtSignal(str, int)

    def run_cmd(self, cmd: str, timeout: Optional[float] = None, env: Optional[dict] = None):
        """Run a blocking shell command; return (rc, stdout, stderr)."""
        if _contains_teleport_keywords(cmd):
            snippet = cmd.splitlines()[0][:120]
            msg = f"[SAFETY] Teleport detectado: bloqueado ({snippet})"
            self.line.emit(msg)
            return 1, "", msg
        res = run_cmd(cmd, timeout=timeout, capture_output=True, env=env)
        stdout = res.stdout if res.stdout is not None else ""
        stderr = res.stderr if res.stderr is not None else ""
        return res.returncode, stdout, stderr

    def run_stream(self, tag: str, cmd: str, proc_slot: Optional[dict] = None, key: Optional[str] = None):
        def emit(msg: str) -> bool:
            try:
                self.line.emit(msg)
                return True
            except RuntimeError:
                return False

        def worker():
            if _contains_teleport_keywords(cmd):
                snippet = cmd.splitlines()[0][:120]
                emit(f"[SAFETY] Teleport detectado: bloqueado ({snippet})")
                return
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


class _RosWorkerThread(QThread):
    def __init__(self, worker: "RosWorker"):
        super().__init__()
        self._worker = worker

    def run(self):
        self._worker._thread_main()


class RosWorker(QObject):
    log = pyqtSignal(str)
    image = pyqtSignal(str, QImage, int, int, float)
    joint_state = pyqtSignal(object)
    system_state = pyqtSignal(str, str)

    def __init__(self, force_realtime: bool = False):
        super().__init__()
        self._lock = threading.Lock()
        self._running = False
        self._node = None
        self._exec = None
        self._bridge = None
        self._thread: Optional[QThread] = None
        self._subs: Dict[str, object] = {}
        self._pose_sub = None
        self._pose_topic = ""
        self._pose_cache: Dict[str, Tuple[float, float, float]] = {}
        self._pose_last_wall = 0.0
        self._pose_msg_count = 0
        self._pose_last_entities = 0
        self._pose_info_empty_logged = False
        self._pubs: Dict[str, object] = {}
        self._fps: Dict[str, List[float]] = {}
        self._frame_count: Dict[str, int] = {}
        self._last_clock_wall = 0.0
        self._last_emit: Dict[str, float] = {}
        self._joint_sub = None
        self._joint_topic = ""
        self._last_joint_emit = 0.0
        self._joint_emit_interval = 0.1
        self._last_joint_payload: Optional[dict] = None
        self._last_joint_wall: float = 0.0
        self._cleanup_done = False
        self._system_diag_reason: str = ""
        self._system_state_last: str = ""
        try:
            max_fps = float(os.environ.get("PANEL_MAX_FPS", "60"))
            if max_fps <= 0:
                max_fps = 60.0
        except ValueError:
            max_fps = 60.0
        self._min_emit_interval = 1.0 / max_fps
        self._force_realtime = force_realtime

    def start(self):
        if not ROS_AVAILABLE:
            self.log.emit("[ROS] No disponible (faltan deps).")
            return
        with self._lock:
            if self._running:
                return
            self._running = True
            self._cleanup_done = False
        thread = _RosWorkerThread(self)
        self._thread = thread
        self.moveToThread(thread)
        thread.start()

    def stop(self):
        with self._lock:
            self._running = False
        self._cleanup_ros()
        thread = self._thread
        if thread is not None:
            if QThread.currentThread() != thread:
                thread.quit()
                thread.wait(1000)
        self._thread = None

    def stop_and_join(self):
        self.stop()

    def list_topic_names(self) -> List[str]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return [name for name, _ in node.get_topic_names_and_types()]
        except Exception:
            return []

    def topic_has_publishers(self, topic: str) -> bool:
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            node = self._node
        if node is None:
            return False
        try:
            return bool(node.get_publishers_info_by_topic(topic))
        except Exception:
            return False

    def topic_names_and_types(self) -> List[Tuple[str, List[str]]]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return node.get_topic_names_and_types()
        except Exception:
            return []

    def service_names_and_types(self) -> List[Tuple[str, List[str]]]:
        with self._lock:
            node = self._node
        if node is None:
            return []
        try:
            return node.get_service_names_and_types()
        except Exception:
            return []

    def has_service(self, name: str) -> bool:
        target = (name or "").strip()
        if not target:
            return False
        for svc_name, _types in self.service_names_and_types():
            if svc_name == target:
                return True
        return False

    def subscribe_pose_info(self, topic: str) -> bool:
        if not ROS_AVAILABLE or TFMessage is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if self._pose_sub is not None and self._pose_topic == topic:
                return True
            if self._pose_sub is not None:
                try:
                    self._node.destroy_subscription(self._pose_sub)
                except Exception:
                    pass
                self._pose_sub = None
            try:
                self._pose_topic = topic
                self._pose_sub = self._node.create_subscription(
                    TFMessage,
                    topic,
                    self._on_pose_info,
                    qos_profile_sensor_data,
                )
                self.log.emit(f"[ROS] Suscrito a {topic} (pose info)")
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo pose info: {exc}")
                return False

    def _on_pose_info(self, msg: "TFMessage") -> None:
        if not ROS_AVAILABLE:
            return
        try:
            transforms = getattr(msg, "transforms", [])
        except Exception:
            transforms = []
        data: Dict[str, Tuple[float, float, float]] = {}
        for tf in transforms:
            name = getattr(tf, "child_frame_id", "") or ""
            if not name:
                header = getattr(tf, "header", None)
                frame = getattr(header, "frame_id", "") if header else ""
                if frame and frame not in ("world", "/world"):
                    name = frame
            t = getattr(tf, "transform", None)
            if not t or not getattr(t, "translation", None):
                continue
            tr = t.translation
            if name:
                data[name] = (float(tr.x), float(tr.y), float(tr.z))
                if "::" in name:
                    base = name.split("::")[0]
                    data.setdefault(base, (float(tr.x), float(tr.y), float(tr.z)))
        if not transforms:
            return
        with self._lock:
            if data:
                self._pose_cache.update(data)
                self._pose_last_entities = len(data)
                self._pose_info_empty_logged = False
            else:
                # Accept pose/info heartbeat even if frame names are missing.
                self._pose_last_entities = len(transforms)
                if not self._pose_info_empty_logged and transforms:
                    sample = transforms[0]
                    header = getattr(sample, "header", None)
                    frame_id = getattr(header, "frame_id", "") if header else ""
                    child_id = getattr(sample, "child_frame_id", "") or ""
                    self.log.emit(
                        f"[PHYSICS][POSE_INFO][DIAG] TFMessage sin nombres: frame_id={frame_id or 'n/a'} child_frame_id={child_id or 'n/a'}"
                    )
                    self._pose_info_empty_logged = True
            self._pose_last_wall = time.time()
            self._pose_msg_count += 1

    def pose_snapshot(self) -> Tuple[Dict[str, Tuple[float, float, float]], float]:
        with self._lock:
            return dict(self._pose_cache), float(self._pose_last_wall)

    def pose_info_status(self) -> Tuple[int, float]:
        """Return pose/info message count and last age (seconds)."""
        with self._lock:
            last = float(self._pose_last_wall)
            count = int(self._pose_msg_count)
        age = time.time() - last if last else float("inf")
        return count, age

    def pose_info_details(self) -> Tuple[int, float, int, str]:
        """Return pose/info count, last age, entities count, and topic."""
        with self._lock:
            last = float(self._pose_last_wall)
            count = int(self._pose_msg_count)
            entities = int(self._pose_last_entities)
            topic = str(self._pose_topic)
        age = time.time() - last if last else float("inf")
        return count, age, entities, topic

    def publish_empty(self, topic: str) -> bool:
        if not ROS_AVAILABLE or Empty is None:
            return False
        topic = (topic or "").strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                return False
            pub = self._pubs.get(topic)
            if pub is None:
                try:
                    pub = self._node.create_publisher(Empty, topic, 10)
                    self._pubs[topic] = pub
                except Exception:
                    return False
        try:
            pub.publish(Empty())
            return True
        except Exception:
            return False
    def get_last_joint_state(self) -> Tuple[Optional[dict], float]:
        with self._lock:
            return self._last_joint_payload, self._last_joint_wall

    def subscribe_image(self, topic: str, msg_type: str = "image") -> bool:
        if not ROS_AVAILABLE:
            return False
        topic = topic.strip()
        if not topic:
            return False
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return False
            if topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception:
                    pass
                self._subs.pop(topic, None)
            try:
                normalized = (msg_type or "").strip().lower()
                callback = self._on_image
                msg_cls = Image
                if "compressed" in normalized:
                    try:
                        from sensor_msgs.msg import CompressedImage

                        msg_cls = CompressedImage
                        callback = self._on_compressed_image
                    except Exception:
                        msg_cls = Image
                        callback = self._on_image
                sub = self._node.create_subscription(msg_cls, topic, lambda msg, t=topic: callback(msg, t), qos_profile_sensor_data)
                self._subs[topic] = sub
                self._fps[topic] = [0.0, time.time(), 0.0]
                qos_summary = self._format_qos(getattr(sub, "qos_profile", None))
                self.log.emit(f"[ROS] Suscrito a {topic} ({qos_summary})")
                return True
            except Exception as exc:
                self.log.emit(f"[ROS] ERROR suscribiendo {topic}: {exc}")
                return False
        return False

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

    def _format_qos(self, profile: Optional["QoSProfile"]) -> str:
        """Describe a QoS profile for logging."""
        if profile is None:
            return "QoS=default"
        reliability = self._reliability_name(getattr(profile, "reliability", None))
        depth = getattr(profile, "depth", None)
        depth_txt = f"depth={depth}" if depth is not None else "depth=?"
        return f"{reliability}@{depth_txt}"

    def _reliability_name(self, value: Optional[int]) -> str:
        if value is None:
            return "reliability=?"
        if ReliabilityPolicy is not None:
            if value == ReliabilityPolicy.RELIABLE:
                return "RELIABLE"
            if value == ReliabilityPolicy.BEST_EFFORT:
                return "BEST_EFFORT"
        return str(value)

    def _thread_main(self):
        if self._force_realtime:
            use_sim_time = False
        else:
            use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception as exc:
                try:
                    self.log.emit(f"[ROS] ERROR rclpy.init: {exc}")
                except RuntimeError:
                    return
                return
        try:
            overrides = None
            if Parameter is not None:
                overrides = [
                    Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))
                ]
            node_kwargs = {}
            if overrides:
                node_kwargs["parameter_overrides"] = overrides
            self._node = rclpy.create_node("panel_superpro", **node_kwargs)
            self._bridge = CvBridge()
            self._exec = SingleThreadedExecutor()
            self._exec.add_node(self._node)
            self._node.create_subscription(Clock, "/clock", lambda _msg: self._update_clock(), qos_profile_sensor_data)
            if String is not None:
                try:
                    self._subs["/system_state"] = self._node.create_subscription(
                        String, "/system_state", self._on_system_state, 10
                    )
                    self._subs["/system_diag"] = self._node.create_subscription(
                        String, "/system_diag", self._on_system_diag, 10
                    )
                except Exception:
                    pass
            try:
                self.log.emit("[ROS] OK: nodo listo.")
            except RuntimeError:
                return
        except Exception as exc:
            try:
                self.log.emit(f"[ROS] ERROR creando nodo/executor: {exc}")
            except RuntimeError:
                return
            return
        while True:
            with self._lock:
                if not self._running:
                    break
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception as exc:
                if ExternalShutdownException is not None and isinstance(exc, ExternalShutdownException):
                    break
                try:
                    # Evitar emitir si el QObject ya se destruyó durante el apagado
                    if self._running:
                        self.log.emit(f"[ROS] WARN spin_once: {exc}")
                    else:
                        break
                except RuntimeError:
                    break
                time.sleep(0.1)
        # Limpieza final (evita logs si ya estamos apagando)
        self._cleanup_ros()

    def _cleanup_ros(self):
        with self._lock:
            if self._cleanup_done:
                return
            self._cleanup_done = True
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
            if self._node and self._pose_sub is not None:
                try:
                    self._node.destroy_subscription(self._pose_sub)
                except Exception:
                    pass
                self._pose_sub = None
                self._pose_topic = ""
                self._pose_cache.clear()
                self._pose_last_wall = 0.0
                self._pose_msg_count = 0
                self._pose_last_entities = 0
            if self._node and self._pubs:
                for _topic, pub in list(self._pubs.items()):
                    try:
                        self._node.destroy_publisher(pub)
                    except Exception:
                        pass
                self._pubs.clear()
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

    def _update_clock(self):
        with self._lock:
            self._last_clock_wall = time.time()

    def clock_alive(self) -> Tuple[bool, float]:
        with self._lock:
            if self._last_clock_wall <= 0.0:
                return False, float("inf")
            age = time.time() - self._last_clock_wall
            return age < 2.0, age

    def node_ready(self) -> bool:
        with self._lock:
            return self._node is not None

    def _on_joint_state(self, msg: "JointState", topic: str):
        if not ROS_AVAILABLE:
            return

        now = time.time()

        # construir payload
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

        # ✅ guardar SIEMPRE (para consumo interno del panel)
        with self._lock:
            self._last_joint_payload = payload
            self._last_joint_wall = now

    def _on_system_diag(self, msg: "String") -> None:
        try:
            raw = getattr(msg, "data", "") or ""
        except Exception:
            return
        reason = ""
        if raw:
            try:
                data = json.loads(raw)
                reason = str(data.get("reason") or "")
            except Exception:
                reason = ""
        with self._lock:
            self._system_diag_reason = reason

    def _on_system_state(self, msg: "String") -> None:
        try:
            state = getattr(msg, "data", "") or ""
        except Exception:
            return
        if not state:
            return
        with self._lock:
            reason = self._system_diag_reason
            if state == self._system_state_last and not reason:
                return
            self._system_state_last = state
        try:
            self.system_state.emit(state, reason or "")
        except RuntimeError:
            pass

        # (opcional) emitir solo cada X para UI/debug
        if (now - self._last_joint_emit) < self._joint_emit_interval:
            return
        self._last_joint_emit = now
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
            self._emit_frame(topic, bgr)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame {topic}: {exc}")

    def _emit_frame(self, topic: str, bgr):
        now = time.time()
        last = self._last_emit.get(topic, 0.0)
        if (now - last) < self._min_emit_interval:
            return
        self._last_emit[topic] = now
        self._frame_count[topic] = self._frame_count.get(topic, 0) + 1
        try:
            import cv2
            import numpy as np

            h, w = bgr.shape[:2]
            rec = self._fps.get(topic, [0.0, time.time(), 0.0])
            rec[0] += 1.0
            dt = max(1e-6, time.time() - rec[1])
            if dt >= 1.0:
                rec[2] = rec[0] / dt
                rec[0] = 0.0
                rec[1] = time.time()
                if DEBUG_FRAME_LOG:
                    frames_total = self._frame_count.get(topic, 0)
                    self.log.emit(f"[CAMERA] {topic} frames={frames_total} fps={rec[2]:.1f}")
            self._fps[topic] = rec
            fps = float(rec[2])
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            qimg = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
            self.image.emit(topic, qimg, w, h, fps)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame emit {topic}: {exc}")

    def _on_compressed_image(self, msg: "CompressedImage", topic: str):
        if not ROS_AVAILABLE:
            return
        try:
            import cv2
            import numpy as np

            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError("No se pudo decodificar CompressedImage")
            self._emit_frame(topic, bgr)
        except Exception as exc:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR compressed frame {topic}: {exc}")


class TfHelper:
    """Lightweight helper that keeps a TF2 buffer spinning for Panel transforms."""

    def __init__(self):
        self._lock = threading.Lock()
        self._node = None
        self._buffer = None
        self._listener = None
        self._executor = None
        self._thread = None
        self._running = False
        self._stop_event = threading.Event()
        self._tf_topic_sub = None
        self._tf_static_topic_sub = None
        self._tf_msg_count = 0
        self._tf_static_msg_count = 0
        self._frames_seen_tf: Set[str] = set()
        self._frames_seen_tf_static: Set[str] = set()
        self._tf_listener_logged = False
        if ROS_AVAILABLE and Buffer is not None:
            self._start()

    def _start(self):
        with self._lock:
            if self._running or Buffer is None:
                return
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
            except Exception:
                pass
            self._node = rclpy.create_node("panel_tf_helper")
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self._node)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            tf_qos = qos_profile_sensor_data
            tf_static_qos = qos_profile_sensor_data
            if (
                QoSProfile is not None
                and ReliabilityPolicy is not None
                and DurabilityPolicy is not None
                and HistoryPolicy is not None
            ):
                tf_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=200,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                tf_static_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            if TFMessage is not None:
                try:
                    self._tf_topic_sub = self._node.create_subscription(
                        TFMessage, "/tf", self._on_tf_msg, tf_qos
                    )
                    self._tf_static_topic_sub = self._node.create_subscription(
                        TFMessage, "/tf_static", self._on_tf_static, tf_static_qos
                    )
                except Exception:
                    pass
            self._running = True
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._stop_event.clear()
            self._thread.start()

    def _spin(self):
        while not self._stop_event.is_set():
            with self._lock:
                if not self._running:
                    return
            try:
                if self._executor:
                    self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                pass
            time.sleep(0.02)

    def shutdown(self) -> None:
        """Stop the helper cleanly."""
        with self._lock:
            if not self._running:
                self._stop_event.set()
                return
            self._running = False
            self._stop_event.set()
        if self._executor:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._node:
            if self._tf_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_topic_sub)
                except Exception:
                    pass
                self._tf_topic_sub = None
            if self._tf_static_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_static_topic_sub)
                except Exception:
                    pass
                self._tf_static_topic_sub = None
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._buffer = None
        self._listener = None
        self._executor = None
        self._thread = None

    def _on_tf_msg(self, msg: "TFMessage") -> None:
        if not msg:
            return
        should_log = False
        with self._lock:
            transforms = list(getattr(msg, "transforms", []))
            self._tf_msg_count += len(transforms)
            for tf in transforms:
                if tf.header and tf.header.frame_id:
                    self._frames_seen_tf.add(tf.header.frame_id)
                if tf.child_frame_id:
                    self._frames_seen_tf.add(tf.child_frame_id)
            if not self._tf_listener_logged:
                self._tf_listener_logged = True
                should_log = True
        if should_log:
            self._log_tf_listener_active()

    def _on_tf_static(self, msg: "TFMessage") -> None:
        if not msg:
            return
        should_log = False
        with self._lock:
            transforms = list(getattr(msg, "transforms", []))
            self._tf_static_msg_count += len(transforms)
            for tf in transforms:
                if tf.header and tf.header.frame_id:
                    self._frames_seen_tf_static.add(tf.header.frame_id)
                if tf.child_frame_id:
                    self._frames_seen_tf_static.add(tf.child_frame_id)
            if not self._tf_listener_logged:
                self._tf_listener_logged = True
                should_log = True
        if should_log:
            self._log_tf_listener_active()

    def tf_listener_stats(self) -> Tuple[int, int]:
        with self._lock:
            return self._tf_msg_count, self._tf_static_msg_count

    def tf_frames_seen(self) -> Tuple[Set[str], Set[str]]:
        with self._lock:
            return set(self._frames_seen_tf), set(self._frames_seen_tf_static)
    def _log_tf_listener_active(self) -> None:
        stats = self.tf_listener_stats()
        print(
            timestamped_line(
                f"[TRACE] TF listener active (tf_msgs={stats[0]} tf_static={stats[1]})"
            ),
            flush=True,
        )

    def shutdown(self):
        with self._lock:
            if not self._running and self._stop_event.is_set():
                return
            self._running = False
            self._stop_event.set()
        if self._executor:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._node:
            if self._tf_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_topic_sub)
                except Exception:
                    pass
                self._tf_topic_sub = None
            if self._tf_static_topic_sub:
                try:
                    self._node.destroy_subscription(self._tf_static_topic_sub)
                except Exception:
                    pass
                self._tf_static_topic_sub = None
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._buffer = None
        self._listener = None
        self._executor = None
        self._thread = None

    def transform_point(self, point: "PointStamped", target_frame: str, timeout_sec: float = 0.8) -> Optional["PointStamped"]:
        """Transform a PointStamped into *target_frame* (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer or point is None or PoseStamped is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = point.header.frame_id
        if point.header and point.header.stamp:
            pose.header.stamp = point.header.stamp
        else:
            if BuiltinTime is not None:
                pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
            else:
                pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z
        pose.pose.orientation.w = 1.0
        converted = self.transform_pose(pose, target_frame, timeout_sec)
        if not converted:
            return None
        out = PointStamped()
        out.header = converted.header
        out.point.x = converted.pose.position.x
        out.point.y = converted.pose.position.y
        out.point.z = converted.pose.position.z
        return out

    def list_frames(self) -> Set[str]:
        """Return the set of available TF frames."""
        if not ROS_AVAILABLE or not self._buffer:
            return set()
        try:
            yaml_text = self._buffer.all_frames_as_yaml()
        except Exception:
            return set()
        frames = {frame for frame in _extract_frames_from_yaml(yaml_text) if frame}
        if frames:
            _log_tf_frames_once(frames)
        if not frames:
            _log_tf_yaml_head_once(yaml_text)
        return frames

    def transform_pose(self, pose: "PoseStamped", target_frame: str, timeout_sec: float = 0.8) -> Optional["PoseStamped"]:
        """Transform a PoseStamped into *target_frame* (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer or pose is None:
            return None
        end = time.time() + timeout_sec
        timeout = _duration_from_seconds(timeout_sec)
        while time.time() < end:
            try:
                if timeout is not None:
                    return self._buffer.transform(pose, target_frame, timeout)
                return self._buffer.transform(pose, target_frame)
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.05)
            except TypeException as exc:
                _log_tf_transform_warning("transform_pose", exc)
                return None
            except Exception as exc:
                _log_tf_transform_warning("transform_pose", exc)
                return None
        return None

    def can_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.1) -> bool:
        """Check if TF is available between frames (blocking until timeout)."""
        if not ROS_AVAILABLE or not self._buffer:
            return False
        end = time.time() + timeout_sec
        timeout = _duration_from_seconds(timeout_sec)
        while time.time() < end:
            try:
                ts = Time() if Time is not None else rclpy.time.Time()
                if timeout is not None:
                    return self._buffer.can_transform(target_frame, source_frame, ts, timeout)
                return self._buffer.can_transform(target_frame, source_frame, ts)
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.02)
        return False

    def lookup_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 1.0):
        """Lookup raw transform *target_frame* <- *source_frame*."""
        if not ROS_AVAILABLE or not self._buffer:
            return None
        end = time.time() + timeout_sec
        while time.time() < end:
            try:
                return self._buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(0.05)
        return None


_TF_HELPER: Optional[TfHelper] = None


def get_tf_helper() -> Optional[TfHelper]:
    """Return singleton TfHelper (if ROS is available)."""
    global _TF_HELPER
    if not ROS_AVAILABLE or Buffer is None:
        return None
    if _TF_HELPER is None:
        _TF_HELPER = TfHelper()
    return _TF_HELPER


def shutdown_tf_helper() -> None:
    """Stop and destroy the TF helper if it exists."""
    global _TF_HELPER
    helper = _TF_HELPER
    if helper:
        helper.shutdown()
        _TF_HELPER = None


def yaw_from_quaternion(quat: "Quaternion") -> float:
    """Return yaw angle (Z) from quaternion."""
    if quat is None:
        return 0.0
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def debug_dump_tf(
    target_frame: str, source_frame: str = "world", timeout_sec: float = 1.0
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    """Lookup and describe TF from *source_frame* to *target_frame*."""
    helper = get_tf_helper()
    if not helper:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    t = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    return (
        {
            "translation": (t.x, t.y, t.z),
            "yaw": yaw,
            "frame_rel": source_frame,
        },
        None,
    )


_TF_FRAMES_LOGGED = False
_TF_FRAME_SUMMARY_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
_EE_UNAVAILABLE_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
EE_FRAME_CANDIDATE_BASES = (
    "tool0",
    "tcp",
    "ee_link",
    "gripper_link",
    "wrist_3_link",
    "flange",
    "ee",
    "tool_link",
    "end_effector_link",
    "rg2_hand",
    "ft_frame",
)

ROBOT_FRAME_KEYWORDS = ("wrist", "shoulder", "elbow", "tool", "tcp", "ee", "flange", "rg2", "hand")
EE_FRAME_SUBSTRING_KEYWORDS = ("tool", "tcp", "ee", "gripper", "flange", "wrist", "rg2", "hand", "ft")

_BASE_FRAME_CACHE: Optional[str] = None
_LAST_TRACE_FRAME_LOG: Optional[str] = None


def _duration_from_seconds(timeout_sec: float) -> Optional["Duration"]:
    if Duration is None:
        return None
    total_ns = int(timeout_sec * 1e9)
    sec = total_ns // 1_000_000_000
    nanosec = total_ns % 1_000_000_000
    return Duration(seconds=int(sec), nanoseconds=int(nanosec))


def _parse_tf_yaml_records(yaml_text: str) -> List[Dict[str, object]]:
    if not yaml_text:
        return []
    parsed = None
    if yaml is not None:
        try:
            parsed = yaml.safe_load(yaml_text)
        except Exception:
            parsed = None
    records: List[Dict[str, object]] = []
    if isinstance(parsed, dict):
        frames = parsed.get("frames")
        if isinstance(frames, list):
            for entry in frames:
                if isinstance(entry, dict):
                    records.append(entry)
    if not records:
        for match in _FRAME_ID_PATTERN.finditer(yaml_text):
            records.append({"frame_id": match.group(1)})
    return records


_FRAME_ID_PATTERN = re.compile(r'frame_id:\s*"([^"]+)"')
_CHILD_FRAME_ID_PATTERN = re.compile(r'child_frame_id:\s*"([^"]+)"')
_FRAME_LINE_PATTERN = re.compile(r'Frame\s+["\']?([A-Za-z0-9_/:\.\-]+)["\']?', re.IGNORECASE)
_FRAME_KEY_PATTERN = re.compile(r'^\s*([A-Za-z0-9_/:\.\-]+)\s*:\s*(?:\{|\[|$)', re.MULTILINE)
_FRAME_LIST_ITEM_PATTERN = re.compile(r'^\s*-\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.MULTILINE)
_FRAME_LIST_FRAME_PATTERN = re.compile(r'^\s*-\s*frame\s*:\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.IGNORECASE | re.MULTILINE)


def _extract_frames_from_yaml(yaml_text: str) -> Set[str]:
    """Return frame names extracted from the TF YAML dump."""
    if not yaml_text:
        return set()
    found: Set[str] = set()
    for pattern in (
        _FRAME_ID_PATTERN,
        _CHILD_FRAME_ID_PATTERN,
        _FRAME_LINE_PATTERN,
        _FRAME_KEY_PATTERN,
        _FRAME_LIST_ITEM_PATTERN,
        _FRAME_LIST_FRAME_PATTERN,
    ):
        for match in pattern.finditer(yaml_text):
            name = match.group(1)
            if name:
                found.add(name)
    cleaned: Set[str] = set()
    ignore = {"frames", "transforms", "child_frames", "header", "data", "frame"}
    for frame in found:
        if frame.lower() in ignore:
            continue
        cleaned.add(frame)
    return cleaned


def _load_tf_frame_records(buffer) -> List[Dict[str, object]]:
    if buffer is None:
        return []
    try:
        yaml_text = buffer.all_frames_as_yaml()
    except Exception:
        return []
    return _parse_tf_yaml_records(yaml_text)


def _build_frame_graph(records: List[Dict[str, object]]) -> Tuple[Set[str], Dict[str, List[str]], Dict[str, str]]:
    frames: Set[str] = set()
    children: Dict[str, List[str]] = {}
    parent_map: Dict[str, str] = {}
    for entry in records:
        fid = entry.get("frame_id")
        if not fid:
            continue
        frames.add(fid)
        parent = entry.get("parent_frame_id")
        if parent and parent != fid:
            parent_map[fid] = parent
            children.setdefault(parent, []).append(fid)
        for child in entry.get("child_frames") or []:
            if isinstance(child, dict):
                child_id = child.get("frame_id")
                if child_id and child_id != fid:
                    frames.add(child_id)
                    parent_map[child_id] = fid
                    children.setdefault(fid, []).append(child_id)
    return frames, children, parent_map


def _collect_leaf_frames(frames: Set[str], children: Dict[str, List[str]]) -> List[str]:
    if not frames:
        return []
    parents = set(children.keys())
    return [frame for frame in frames if frame not in parents]


def _frame_depth(
    frame: str,
    parent_map: Dict[str, str],
    cache: Dict[str, int],
    visiting: Optional[Set[str]] = None,
) -> int:
    if frame in cache:
        return cache[frame]
    if visiting is None:
        visiting = set()
    if frame in visiting:
        cache[frame] = 0
        return 0
    visiting.add(frame)
    parent = parent_map.get(frame)
    if not parent or parent == frame:
        depth = 0
    else:
        depth = 1 + _frame_depth(parent, parent_map, cache, visiting)
    visiting.remove(frame)
    cache[frame] = depth
    return depth


def _log_tf_frames_once(frames: Set[str]) -> None:
    global _TF_FRAMES_LOGGED
    if _TF_FRAMES_LOGGED or not frames:
        return
    sample = ", ".join(sorted(frames)[:80])
    print(
        timestamped_line(f"[TRACE] Available TF frames ({len(frames)}): {sample}"),
        flush=True,
    )
    _TF_FRAMES_LOGGED = True

def _log_tf_frame_summary(all_frames: Set[str], robot_keyword_frames: List[str]) -> None:
    global _TF_FRAME_SUMMARY_LOGGED
    if _TF_FRAME_SUMMARY_LOGGED or not all_frames:
        return
    sample = ", ".join(sorted(all_frames)[:80])
    print(
        timestamped_line(
            f"[TRACE] TF summary frames={len(all_frames)} robot_candidates={len(robot_keyword_frames)} sample={sample}"
        ),
        flush=True,
    )
    _TF_FRAME_SUMMARY_LOGGED = True


def _log_tf_yaml_head_once(yaml_text: str) -> None:
    global _TF_YAML_HEAD_LOGGED
    if _TF_YAML_HEAD_LOGGED or not yaml_text:
        return
    lines = yaml_text.strip().splitlines()
    head = "\n".join(lines[:20])
    print(
        timestamped_line(f"[TRACE][DIAG] TF YAML head:\n{head}"),
        flush=True,
    )
    _TF_YAML_HEAD_LOGGED = True


def _log_ee_unavailable_once() -> None:
    global _EE_UNAVAILABLE_LOGGED
    if _EE_UNAVAILABLE_LOGGED:
        return
    print(timestamped_line("[TRACE] EE unavailable (no valid EE frame)"), flush=True)
    _EE_UNAVAILABLE_LOGGED = True


def _can_transform_between(helper: TfHelper, frame_a: str, frame_b: str, timeout_sec: float) -> bool:
    if not frame_a or not frame_b:
        return False
    if helper.can_transform(frame_a, frame_b, timeout_sec=timeout_sec):
        return True
    if helper.can_transform(frame_b, frame_a, timeout_sec=timeout_sec):
        return True
    return False


def _preferred_base_frame(helper: Optional[TfHelper], world_frame: str, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first base candidate that transforms to the world frame."""
    if helper is None:
        return None
    for candidate in BASE_FRAME_CANDIDATES:
        if not candidate:
            continue
        if _can_transform_between(helper, candidate, world_frame, timeout_sec=timeout_sec):
            return candidate
    return None


def discover_robot_base_frame(world_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Detect the most likely robot base frame given the TF tree."""
    helper = get_tf_helper()
    if helper is None:
        return None
    base_frame, _ = discover_base_and_ee_frames(world_frame, timeout_sec)
    return base_frame


def discover_world_frame(helper: TfHelper, base_frame: str, selection_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first world frame candidate that transforms to *base_frame*."""
    if helper is None or not base_frame:
        return selection_frame or WORLD_FRAME or "world"
    frames = helper.list_frames()
    candidates: List[str] = []
    if selection_frame:
        candidates.append(selection_frame)
    for candidate in (WORLD_FRAME, *WORLD_FRAME_CANDIDATES, "gz_world", "map", "odom"):
        if not candidate:
            continue
        if candidate not in candidates:
            candidates.append(candidate)
    for candidate in candidates:
        if candidate not in frames:
            continue
        if _can_transform_between(helper, candidate, base_frame, timeout_sec):
            return candidate
    return selection_frame or WORLD_FRAME or "world"


def discover_base_and_ee_frames(world_frame: Optional[str] = None, timeout_sec: float = 0.1) -> Tuple[Optional[str], Optional[str]]:
    """Return effective base and EE frames based on live TF contents."""
    global _BASE_FRAME_CACHE, _LAST_TRACE_FRAME_LOG, _EE_UNAVAILABLE_LOGGED
    helper = get_tf_helper()
    if helper is None or helper._buffer is None:
        return None, None
    frames = helper.list_frames()
    if not frames:
        return None, None
    records = _load_tf_frame_records(helper._buffer)
    graph_frames, children, parent_map = _build_frame_graph(records)
    all_frames = frames.union(graph_frames)
    if not all_frames:
        return None, None
    target_world = world_frame or WORLD_FRAME or "world"
    sorted_frames = sorted(all_frames)
    robot_keyword_frames = [
        frame for frame in sorted_frames if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
    ]
    _log_tf_frame_summary(all_frames, robot_keyword_frames)
    base_frame = _select_base_frame(
        helper,
        all_frames,
        children,
        parent_map,
        target_world,
        robot_keyword_frames,
        timeout_sec,
    )
    fallback_base = base_frame or BASE_FRAME or "base"
    ee_frame = _select_ee_frame(helper, all_frames, children, parent_map, fallback_base, timeout_sec)
    effective_base = base_frame or fallback_base
    disallowed_ee = {effective_base}
    if target_world:
        disallowed_ee.add(target_world)
    if ee_frame and ee_frame in disallowed_ee:
        ee_frame = None
        _log_ee_unavailable_once()
    elif ee_frame:
        _EE_UNAVAILABLE_LOGGED = False
    log_msg = f"[TRACE] Using BASE_FRAME_EFFECTIVE={effective_base or 'n/a'} EE_FRAME_EFFECTIVE={ee_frame or 'n/a'}"
    if log_msg != _LAST_TRACE_FRAME_LOG:
        print(timestamped_line(log_msg), flush=True)
        _LAST_TRACE_FRAME_LOG = log_msg
    return effective_base, ee_frame


def _select_base_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    world_frame: str,
    robot_keyword_frames: List[str],
    timeout_sec: float,
) -> Optional[str]:
    global _BASE_FRAME_CACHE
    candidate_order: List[str] = []
    seen: Set[str] = set()

    def add_candidate(name: Optional[str]) -> None:
        if not name or name in seen:
            return
        seen.add(name)
        candidate_order.append(name)

    add_candidate(BASE_FRAME)
    for candidate in BASE_FRAME_CANDIDATES:
        add_candidate(candidate)
    add_candidate(_BASE_FRAME_CACHE)

    robot_frames = list(robot_keyword_frames)
    if not robot_frames:
        leaves = _collect_leaf_frames(frames, children)
        fallback_frames = leaves or sorted(frames)
        depth_cache: Dict[str, int] = {}
        robot_frames = sorted(
            fallback_frames,
            key=lambda f: _frame_depth(f, parent_map, depth_cache),
            reverse=True,
        )

    base_world_only: Optional[str] = None
    base_link_candidate: Optional[str] = None
    for candidate in candidate_order:
        if candidate not in frames:
            continue
        if not _can_transform_between(helper, candidate, world_frame, timeout_sec):
            continue
        if candidate == "base_link":
            base_link_candidate = candidate
        robot_connected = any(
            _can_transform_between(helper, candidate, robot_frame, timeout_sec)
            for robot_frame in robot_frames
        )
        if robot_connected:
            _BASE_FRAME_CACHE = candidate
            return candidate
        if candidate == "base":
            base_world_only = base_world_only or candidate

    if base_link_candidate:
        _BASE_FRAME_CACHE = base_link_candidate
        return base_link_candidate
    if base_world_only:
        _BASE_FRAME_CACHE = base_world_only
        return base_world_only
    return None


def _select_ee_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    base_frame: str,
    timeout_sec: float,
) -> Optional[str]:
    if not base_frame:
        return None
    seen: Set[str] = set()
    candidates: List[str] = []

    def add_candidate(name: str) -> None:
        if not name or name in seen or name not in frames:
            return
        seen.add(name)
        candidates.append(name)

    for preferred in EE_FRAME_PREFERENCE:
        add_candidate(preferred)

    # Align with tf_probe keepers priority.
    for keeper in ("tool0", "tcp", "ee_link", "flange", "wrist_3_link", "ft_frame", "rg2_hand"):
        add_candidate(keeper)

    prefixes = set()
    for frame in frames:
        if "/" in frame:
            prefixes.add(frame.split("/", 1)[0])
        if "_" in frame:
            prefixes.add(frame.split("_", 1)[0])
    prefixes.discard("")

    for prefix in sorted(prefixes):
        for name in EE_FRAME_CANDIDATE_BASES:
            add_candidate(f"{prefix}/{name}")
            add_candidate(f"{prefix}_{name}")

    for name in EE_FRAME_CANDIDATE_BASES:
        add_candidate(name)

    for frame in sorted(frames):
        if any(keyword in frame.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS):
            add_candidate(frame)

    for candidate in candidates:
        if _can_transform_between(helper, base_frame, candidate, timeout_sec):
            return candidate

    leaves = _collect_leaf_frames(frames, children) or list(frames)
    preferred = [
        leaf
        for leaf in leaves
        if any(keyword in leaf.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS)
    ]
    leaf_candidates = preferred or leaves
    depth_cache: Dict[str, int] = {}
    ordered_leaves = sorted(
        leaf_candidates,
        key=lambda f: _frame_depth(f, parent_map, depth_cache),
        reverse=True,
    )
    for leaf in ordered_leaves:
        if _can_transform_between(helper, base_frame, leaf, timeout_sec):
            return leaf
    return None


def get_pose(
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    pose = {
        "frame": target_frame,
        "position": (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
        "orientation": (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
    }
    return pose, None


def _pose_from_transform(transform: "TransformStamped", frame_id: str) -> Optional["PoseStamped"]:
    if not transform or PoseStamped is None:
        return None
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = transform.header.stamp
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def lookup_pose(
    frame_from: str,
    frame_to: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional["PoseStamped"], Optional[str]]:
    """Return PoseStamped of *frame_from* expressed in *frame_to*."""
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(frame_to, frame_from, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {frame_from}->{frame_to} timed out"
    pose = _pose_from_transform(transform, frame_to)
    if not pose:
        return None, "pose helper unavailable"
    return pose, None


def transform_pose(
    pose: Optional["PoseStamped"],
    target_frame: str,
    timeout_sec: float = 0.8,
) -> Optional["PoseStamped"]:
    helper = get_tf_helper()
    if helper is None or pose is None:
        return None
    return helper.transform_pose(pose, target_frame, timeout_sec)


BASE_FRAME_CANDIDATES = [
    "base_link",
    "base",
    "ur5_base_link",
    "ur5_base",
    "ur5_arm_base_link",
    "ur5_base_link_ee",
]
EE_FRAME_PREFERENCE = [
    "tool0",
    "tcp",
    "ee_link",
    "ee",
    "tool_link",
    "end_effector_link",
    "flange",
    "wrist_3_link",
    "rg2_hand",
    "ft_frame",
]
WORLD_FRAME_CANDIDATES = ["world", "map", "odom"]


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x,y,z,w) from euler angles."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def detect_base_frame(
    source_frame: str = "world",
    timeout_sec: float = 0.2,
    candidates: Optional[List[str]] = None,
) -> Tuple[Optional[str], Optional[object], Optional[str]]:
    helper = get_tf_helper()
    if not helper:
        return None, None, "TF helper unavailable"
    global _BASE_FRAME_CACHE
    scan = []
    if candidates:
        scan.extend(candidates)
    if BASE_FRAME and BASE_FRAME not in scan:
        scan.append(BASE_FRAME)
    for frame in BASE_FRAME_CANDIDATES:
        if frame not in scan:
            scan.append(frame)
    if _BASE_FRAME_CACHE and _BASE_FRAME_CACHE not in scan:
        scan.append(_BASE_FRAME_CACHE)
    for frame in scan:
        if not frame:
            continue
        transform = helper.lookup_transform(frame, source_frame, timeout_sec=timeout_sec)
        if transform:
            _BASE_FRAME_CACHE = frame
            return frame, transform, None
    return None, None, f"lookup {source_frame}->({','.join(scan)}) timed out"


def _rotation_matrix_from_quaternion(quat: "Quaternion") -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    """Return rotation matrix built from quaternion."""
    if quat is None:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    w = float(quat.w)
    x = float(quat.x)
    y = float(quat.y)
    z = float(quat.z)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _apply_rotation(matrix: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]], vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Multiply 3x3 matrix by vector."""
    x, y, z = vector
    row0, row1, row2 = matrix
    return (
        row0[0] * x + row0[1] * y + row0[2] * z,
        row1[0] * x + row1[1] * y + row1[2] * z,
        row2[0] * x + row2[1] * y + row2[2] * z,
    )


def _do_transform_point(point: "PointStamped", transform: "TransformStamped") -> "PointStamped":
    """Apply transform stamped to a PointStamped."""
    rotation = _rotation_matrix_from_quaternion(transform.transform.rotation)
    px = point.point.x
    py = point.point.y
    pz = point.point.z
    rx, ry, rz = _apply_rotation(rotation, (px, py, pz))
    translation = transform.transform.translation
    res = PointStamped()
    res.header.frame_id = transform.header.frame_id
    res.header.stamp = transform.header.stamp
    res.point.x = rx + translation.x
    res.point.y = ry + translation.y
    res.point.z = rz + translation.z
    return res


def _apply_transform_to_tuple(point: Tuple[float, float, float], transform: "TransformStamped") -> Tuple[float, float, float]:
    rotation = _rotation_matrix_from_quaternion(transform.transform.rotation)
    rx, ry, rz = _apply_rotation(rotation, point)
    translation = transform.transform.translation
    return (rx + translation.x, ry + translation.y, rz + translation.z)


def _register_point_stamp_tf():
    """Register PointStamped transform if not already available."""
    if PointStamped is None or TransformStamped is None or TransformRegistration is None:
        return
    try:
        TransformRegistration().get(PointStamped)
        return
    except TypeException:
        pass
    try:
        TransformRegistration().add(PointStamped, _do_transform_point)
    except Exception:
        pass


_register_point_stamp_tf()


def transform_point_to_frame(
    world_pos: Tuple[float, float, float],
    target_frame: str,
    source_frame: str = "world",
    timeout_sec: float = 0.6,
) -> Tuple[Optional[Tuple[float, float, float]], Optional[object]]:
    helper = get_tf_helper()
    if helper is None or world_pos is None:
        return None, None
    if PoseStamped is None:
        return None, None
    if tf2_geometry_msgs is None:
        try:
            transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        except Exception as exc:
            _log_tf_transform_warning("transform_point_to_frame", exc)
            return None, None
    try:
        pose = PoseStamped()
        pose.header.frame_id = source_frame or ""
        if BuiltinTime is not None:
            pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
        else:
            pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = world_pos
        pose.pose.orientation.w = 1.0
        transformed = helper.transform_pose(pose, target_frame, timeout_sec)
        if not transformed:
            transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        coords = (
            transformed.pose.position.x,
            transformed.pose.position.y,
            transformed.pose.position.z,
        )
        transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
        return coords, transform
    except Exception as exc:
        _log_tf_transform_warning("transform_point_to_frame", exc)
        return None, None


def world_to_base_coords(
    world_pos: Tuple[float, float, float], frame: str = "world", require_tf: bool = False
) -> Tuple[
    Optional[Tuple[float, float, float]], Optional[object], Optional[str]
]:
    """Transform *world_pos* from *frame* into BASE_FRAME using TF if available."""
    helper = get_tf_helper()
    target_frame = BASE_FRAME or "base"
    if helper:
        transform = helper.lookup_transform(target_frame, frame, timeout_sec=1.0)
        if transform:
            point = PointStamped()
            point.header.frame_id = frame
            point.header.stamp = rclpy.time.Time()
            point.point.x, point.point.y, point.point.z = world_pos
            converted = helper.transform_point(point, target_frame, timeout_sec=0.6)
            if converted:
                coords = (
                    converted.point.x,
                    converted.point.y,
                    converted.point.z,
                )
                return coords, transform, None
        if require_tf:
            return None, None, f"lookup {frame}->{target_frame} timed out"
    if require_tf:
        reason = "TF helper unavailable" if helper else "no TF helper"
        return None, None, reason
    bx, by, bz = world_to_base(*world_pos)
    return (bx, by, bz), None, None


def _list_tf_topics() -> Tuple[List[str], List[str]]:
    """Return the available tf and tf_static topics."""
    node = _create_graph_node("panel_tf_topics")
    if node is None:
        return [], []
    try:
        topics = node.get_topic_names_and_types()
        tf = [name for name, _ in topics if name == "/tf"]
        tf_static = [name for name, _ in topics if name == "/tf_static"]
        return tf, tf_static
    except Exception:
        return [], []
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def _parse_static_tf_env() -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    env = os.environ.get("PANEL_STATIC_TF", "").strip()
    if not env:
        return None
    parts = env.split()
    if len(parts) != 6:
        return None
    try:
        x, y, z, roll, pitch, yaw = [float(v) for v in parts]
    except ValueError:
        return None
    quat = _euler_to_quaternion(roll, pitch, yaw)
    return (x, y, z), quat


def _publish_static_tf(world_frame: str, base_frame: str) -> Tuple[bool, Optional[str]]:
    if not ROS_AVAILABLE or StaticTransformBroadcaster is None:
        return False, "ROS unavailable"
    translation = None
    rotation = None
    pose = get_object_pose_gz(UR5_MODEL_NAME)
    if pose:
        pos = pose.get("position") or {}
        orient = pose.get("orientation") or {}
        try:
            translation = (
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                float(pos.get("z", 0.0)),
            )
            rotation = (
                float(orient.get("x", 0.0)),
                float(orient.get("y", 0.0)),
                float(orient.get("z", 0.0)),
                float(orient.get("w", 1.0)),
            )
        except Exception:
            translation = None
            rotation = None
    if translation is None:
        manual = _parse_static_tf_env()
        if manual:
            translation, rotation = manual
    if translation is None or rotation is None:
        return False, "no robot pose available"
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except Exception:
        pass
    node = None
    broadcaster = None
    try:
        node = rclpy.create_node("panel_static_tf")
        broadcaster = StaticTransformBroadcaster(node)
        tfs = TransformStamped()
        tfs.header.stamp = node.get_clock().now().to_msg()
        tfs.header.frame_id = world_frame
        tfs.child_frame_id = base_frame
        tfs.transform.translation.x = translation[0]
        tfs.transform.translation.y = translation[1]
        tfs.transform.translation.z = translation[2]
        tfs.transform.rotation.x = rotation[0]
        tfs.transform.rotation.y = rotation[1]
        tfs.transform.rotation.z = rotation[2]
        tfs.transform.rotation.w = rotation[3]
        broadcaster.sendTransform(tfs)
        time.sleep(0.05)
    except Exception as exc:
        if node:
            node.destroy_node()
        return False, f"static tf error: {exc}"
    if node:
        node.destroy_node()
    return True, f"Using STATIC TF {world_frame}->{base_frame}"


def diagnose_tf_tree(
    world_pose: Optional[Tuple[float, float, float]],
    selection_frame: Optional[str] = None,
) -> Dict[str, object]:
    tf_topics, tf_static = _list_tf_topics()
    world_candidates = []
    if selection_frame:
        world_candidates.append(selection_frame)
    world_candidates.extend([WORLD_FRAME, *WORLD_FRAME_CANDIDATES])
    # remove duplicates preserving order
    seen = []
    filtered_worlds = []
    for cand in world_candidates:
        if cand and cand not in seen:
            seen.append(cand)
            filtered_worlds.append(cand)
    world_candidates = filtered_worlds

    topics_summary = []
    if tf_topics:
        topics_summary.append("/tf")
    if tf_static:
        topics_summary.append("/tf_static")
    result = {
        "tf_topics": topics_summary,
        "world_frame": None,
        "base_frame": None,
        "selected_base": None,
        "transform": None,
        "ok": False,
        "error": None,
        "fallback": "none",
    }

    if not world_pose:
        result["error"] = "no world pose"
        return result

    helper = get_tf_helper()
    if helper is None:
        result["error"] = "TF helper unavailable"
        return result

    world_frame = selection_frame or WORLD_FRAME or "world"
    base_frame = discover_robot_base_frame(world_frame)
    if base_frame:
        world_frame = discover_world_frame(helper, base_frame, selection_frame)
    result["world_frame"] = world_frame
    result["base_frame"] = base_frame or BASE_FRAME or "base"

    def attempt_transform(target_base: str, target_world: str) -> Tuple[Optional[Tuple[float, float, float]], Optional["TransformStamped"], Optional[str]]:
        coords, transform = transform_point_to_frame(world_pose, target_base, source_frame=target_world, timeout_sec=0.6)
        if not coords or not transform:
            return None, None, f"transform {target_world}->{target_base} timed out"
        return coords, transform, None

    coords, transform, error = (None, None, None)
    if base_frame and world_frame:
        coords, transform, error = attempt_transform(base_frame, world_frame)
    if coords and transform:
        result.update(
            {
                "selected_base": coords,
                "transform": transform,
                "ok": True,
            }
        )
        return result

    if error:
        result["error"] = error
    else:
        result["error"] = f"lookup {world_frame}->{result['base_frame']} timed out"

    if os.environ.get("ENABLE_STATIC_TF_FALLBACK", "0") == "1":
        fallback_world = world_frame or WORLD_FRAME or "world"
        fallback_base = base_frame or BASE_FRAME or "base"
        ok, msg = _publish_static_tf(fallback_world, fallback_base)
        result["fallback"] = msg
        if ok:
            time.sleep(0.05)
            helper = get_tf_helper()
            base_frame = fallback_base
            world_frame = fallback_world
            result["world_frame"] = world_frame
            result["base_frame"] = base_frame
            coords, transform, error = attempt_transform(base_frame, world_frame)
            if coords and transform:
                result.update(
                    {
                        "selected_base": coords,
                        "transform": transform,
                        "ok": True,
                        "error": None,
                    }
                )
                return result
        else:
            if not result["error"]:
                result["error"] = "static tf publish failed"

    return result
