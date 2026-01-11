#!/usr/bin/env python3
"""
Panel V2 minimal: barra de control con los botones mostrados, sin logs ni dependencias del panel existente.
Ejecutar con: python -m ur5_qt_panel.panel_v2
"""
from __future__ import annotations

import os
import re
import math
import shlex
import signal
import shutil
import subprocess
import sys
import threading
import time
import json
from enum import Enum
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set
try:
    import psutil  # type: ignore
except Exception:
    psutil = None
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractScrollArea,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSlider,
    QSizePolicy,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QHeaderView,
    QTableWidget,
    QTableWidgetItem,
)

from .panel_config import (
    BAGS_DIR,
    BASE_FRAME,
    BRIDGE_BASE_YAML,
    DEFAULT_JOINT_MOVE_SEC,
    DEFAULT_WORLD_CANDIDATES,
    EXTRA_OBJECTS,
    TABLE_OBJECTS,
    DYNAMIC_OBJECTS,
    EGL_VENDOR,
    GZ_PARTITION_FILE,
    GZ_WORLD,
    JOINT_SLIDER_DEG_MAX,
    JOINT_SLIDER_DEG_MIN,
    JOINT_SLIDER_SCALE,
    LOG_DIR,
    MODELS_DIR,
    SCRIPTS_DIR,
    TABLE_CENTER_X,
    TABLE_CENTER_Y,
    TABLE_SIZE_X,
    TABLE_SIZE_Y,
    SELECTION_SNAP_DIST,
    AUTO_START_BRIDGE,
    AUTO_START_BRIDGE_DELAY_MS,
    AUTO_START_BRIDGE_MAX_RETRIES,
    UR5_HOME_DEFAULT,
    UR5_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
    GRIPPER_ATTACH_PREFIX,
    UR5_BASE_X,
    UR5_BASE_Y,
    UR5_BASE_Z,
    UR5_CONTROLLERS_YAML,
    UR5_MODEL_NAME,
    BASKET_DROP,
    WORLD_FRAME,
    WORLDS_DIR,
    WS_DIR,
    ROS_AVAILABLE,
)
from .panel_utils import (
    bash_preamble,
    build_gz_env,
    CmdRunner,
    GZ_LOG_FILTERS,
    RosWorker,
    detect_base_frame,
    diagnose_tf_tree,
    ensure_dir,
    bulk_update_object_positions,
    get_object_positions,
    get_object_position,
    gz_sim_status,
    base_to_world,
    world_to_base,
    load_home_pose,
    save_object_positions,
    log_to_file,
    parse_ros_topics,
    read_world_name,
    nearest_table_object,
    object_out_of_reach,
    resolve_gz_partition,
    rotate_log,
    set_led,
    table_xy_to_pixel,
    pixel_to_table_xy,
    _parse_pose_json,
    transform_point_to_frame,
    get_pose,
    discover_base_and_ee_frames,
    get_tf_helper,
    _can_transform_between,
    _preferred_base_frame,
    _list_tf_topics,
    shutdown_tf_helper,
    _log_tf_yaml_head_once,
    world_xyz_to_pixel,
    with_line_buffer,
    build_log_filter_cmd,
    yaw_from_quaternion,
    debug_dump_tf,
    rclpy,
    ROBOT_FRAME_KEYWORDS,
    gripper_controller_defined,
)
from .logging_utils import timestamped_line

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
try:
    from rclpy.parameter import Parameter
except Exception:
    Parameter = None
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
try:
    from controller_manager_msgs.srv import ListControllers
except Exception:
    ListControllers = None
try:
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import MoveGroup
except Exception:
    ActionClient = None
    FollowJointTrajectory = None
    MoveGroup = None

CAMERA_TOPIC_PREFIX = "/camera"
CAMERA_DISPLAY_INTERVAL_MS = 80
SELECTION_TIMEOUT_SEC = float(os.environ.get("PANEL_SELECTION_TIMEOUT_SEC", "12.0"))
MOVEIT_POSE_TOPIC = "/desired_grasp"
TRAJ_ACTION_FALLBACK = os.environ.get("PANEL_TRAJ_ACTION_FALLBACK", "1") == "1"
TRAJ_ACTION_FALLBACK_DELAY_SEC = float(os.environ.get("PANEL_TRAJ_ACTION_FALLBACK_DELAY_SEC", "1.0"))
TRAJ_ACTION_FALLBACK_EPS_RAD = float(os.environ.get("PANEL_TRAJ_ACTION_FALLBACK_EPS_RAD", "0.002"))
TRAJ_ACTION_FALLBACK_TIMEOUT_SEC = float(os.environ.get("PANEL_TRAJ_ACTION_FALLBACK_TIMEOUT_SEC", "2.0"))
CONTROLLER_READY_TIMEOUT_SEC = float(os.environ.get("PANEL_CONTROLLER_READY_TIMEOUT_SEC", "3.0"))
MOVEIT_READY_TIMEOUT_SEC = float(os.environ.get("PANEL_MOVEIT_READY_TIMEOUT_SEC", "20.0"))
GZ_LAUNCH_TIMEOUT_SEC = float(os.environ.get("PANEL_GZ_LAUNCH_TIMEOUT_SEC", "20.0"))
BRIDGE_LAUNCH_TIMEOUT_SEC = float(os.environ.get("PANEL_BRIDGE_LAUNCH_TIMEOUT_SEC", "12.0"))
MOVEIT_LAUNCH_TIMEOUT_SEC = float(os.environ.get("PANEL_MOVEIT_LAUNCH_TIMEOUT_SEC", "25.0"))
CONTROLLER_DROP_GRACE_SEC = float(os.environ.get("PANEL_CTRL_DROP_GRACE_SEC", "3.0"))
TRACE_PRINT_PERIOD_SEC = float(os.environ.get("PANEL_TRACE_PRINT_PERIOD_SEC", "3.0"))
DEBUG_POSES_PERIOD_SEC = float(os.environ.get("PANEL_DEBUG_POSES_PERIOD_SEC", "3.0"))
PICK_LOG_MIN_INTERVAL_SEC = float(os.environ.get("PANEL_PICK_LOG_MIN_INTERVAL_SEC", "2.0"))
GRIPPER_CMD_TOPIC = os.environ.get("PANEL_GRIPPER_CMD_TOPIC", "/gripper_controller/commands")
GRIPPER_OPEN_RAD = float(os.environ.get("PANEL_GRIPPER_OPEN_RAD", "1.0"))
GRIPPER_CLOSED_RAD = float(os.environ.get("PANEL_GRIPPER_CLOSED_RAD", "0.0"))
GRIPPER_JOINT2_SIGN = float(os.environ.get("PANEL_GRIPPER_JOINT2_SIGN", "1.0"))
def _env_flag(name: str, default: str = "0") -> bool:
    value = os.environ.get(name, default)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


PANEL_MANAGED = _env_flag("PANEL_MANAGED", "0")
PANEL_MOVEIT_REQUIRED = _env_flag("PANEL_MOVEIT_REQUIRED", "1")


class SystemState(Enum):
    BOOT = "BOOT"
    WAITING_GAZEBO = "WAITING_GAZEBO"
    WAITING_CONTROLLERS = "WAITING_CONTROLLERS"
    READY_BASIC = "READY_BASIC"
    READY_VISION = "READY_VISION"
    READY_MOVEIT = "READY_MOVEIT"
    ERROR = "ERROR"


class MoveItState(Enum):
    OFF = "OFF"
    STARTING = "STARTING"
    WAITING_MOVEIT_READY = "WAITING_MOVEIT_READY"
    READY = "READY"
    ERROR = "ERROR"
DROP_OBJECT_NAMES = [
    "drop_obj_01_box_cube",
    "drop_obj_02_box_flat",
    "drop_obj_03_box_tall",
    "drop_obj_04_cyl_long",
    "drop_obj_05_cyl_mid",
    "drop_obj_06_cyl_puck",
    "drop_obj_07_sphere_big",
    "drop_obj_08_sphere_small",
    "drop_obj_09_box_bar",
    "drop_obj_10_cross",
]
OBJECT_SETTLE_Z_EPS = 0.002
OBJECT_FALL_Z_EPS = 0.01
OBJECT_SETTLE_WINDOW_SEC = 1.0
OBJECT_SETTLE_POLL_SEC = 0.6
OBJECT_SETTLE_TIMEOUT_SEC = 10.0
OBJECT_SETTLE_XY_EPS = 0.002
SETTLE_PATTERNS = ("drop_obj_",)
SETTLE_MANUAL = {"cubo_rojo", "cilindro_verde", "caja_azul", "pieza_pick_mesa"}
ALLOW_UNSETTLED_ON_TIMEOUT = bool(int(os.environ.get("PANEL_ALLOW_UNSETTLED_ON_TIMEOUT", "0")))
CAMERA_READY_FRAMES = int(os.environ.get("PANEL_CAMERA_READY_FRAMES", "3"))
CAMERA_INIT_GRACE_SEC = float(os.environ.get("PANEL_CAMERA_INIT_GRACE_SEC", "4.0"))
CAMERA_READY_MAX_AGE_SEC = float(os.environ.get("PANEL_CAMERA_READY_MAX_AGE_SEC", "2.5"))
_CAMERA_REQUIRED_ENV = os.environ.get("PANEL_CAMERA_REQUIRED")
CAMERA_REQUIRED = None if _CAMERA_REQUIRED_ENV is None else _CAMERA_REQUIRED_ENV.strip().lower() not in ("0", "false", "no", "off")
STALE_PROCESS_GRACE_SEC = float(os.environ.get("PANEL_STALE_GRACE_SEC", "20"))
TF_INIT_GRACE_SEC = float(os.environ.get("PANEL_TF_INIT_GRACE_SEC", "3.0"))
CONTROLLER_CHECK_INTERVAL_SEC = 3.0
CONTROLLER_START_GRACE_SEC = float(os.environ.get("PANEL_CTRL_START_GRACE_SEC", "12.0"))
POSE_INFO_MAX_AGE_SEC = float(os.environ.get("PANEL_POSE_INFO_MAX_AGE_SEC", "1.0"))
POSE_INFO_POLL_SEC = float(os.environ.get("PANEL_POSE_INFO_POLL_SEC", "0.5"))
POSE_INFO_LOG_PERIOD = float(os.environ.get("PANEL_POSE_INFO_LOG_PERIOD", "2.0"))
ATTACH_DIST_M = float(os.environ.get("PANEL_ATTACH_DIST_M", "0.02"))
ATTACH_REL_EPS = float(os.environ.get("PANEL_ATTACH_REL_EPS", "0.002"))
ATTACH_HAND_MOVE_EPS = float(os.environ.get("PANEL_ATTACH_HAND_MOVE_EPS", "0.005"))
ATTACH_WINDOW_SEC = float(os.environ.get("PANEL_ATTACH_WINDOW_SEC", "0.6"))
ATTACH_SNAP_EPS = float(os.environ.get("PANEL_ATTACH_SNAP_EPS", "0.003"))
HAND_LINK_CANDIDATES = ("rg2_hand", "gripper", "tool0", "ee_link")
PICK_TF_RETRY_SEC = float(os.environ.get("PANEL_PICK_TF_RETRY_SEC", "0.2"))
PICK_TF_TIMEOUT_SEC = float(os.environ.get("PANEL_PICK_TF_TIMEOUT_SEC", "3.0"))
FALL_TEST_DELAY_SEC = float(os.environ.get("PANEL_FALL_TEST_DELAY_SEC", "1.0"))


def _proto_time_to_seconds(value: Dict[str, object]) -> float:
    if not isinstance(value, dict):
        return 0.0
    sec = float(value.get("sec", 0.0)) if value.get("sec") is not None else 0.0
    nsec = float(value.get("nsec", 0.0)) if value.get("nsec") is not None else 0.0
    return sec + nsec * 1e-9


def _parse_sdf_gravity(world_path: str) -> Tuple[float, float, float]:
    if not world_path:
        return (0.0, 0.0, -9.81)
    try:
        text = Path(world_path).read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return (0.0, 0.0, -9.81)
    match = re.search(r"<gravity>([^<]+)</gravity>", text, re.IGNORECASE)
    if match:
        numbers = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", match.group(1))
        if len(numbers) >= 3:
            try:
                return tuple(float(n) for n in numbers[:3])
            except ValueError:
                pass
    return (0.0, 0.0, -9.81)


def _make_pose_data(
    position: Tuple[float, float, float],
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    frame: str = "base_link",
) -> Dict[str, object]:
    return {"position": position, "orientation": orientation, "frame": frame}


def _build_pose_stamped(data: Dict[str, object]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = data.get("frame", "base_link")
    position = data.get("position", (0.0, 0.0, 0.0))
    orientation = data.get("orientation", (0.0, 0.0, 0.0, 1.0))
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = orientation
    return pose


POSE_HOME_DATA = _make_pose_data((0.18, 0.0, 0.35))
POSE_TABLE_DATA = _make_pose_data((0.30, -0.35, 0.28))
POSE_BASKET_DATA = _make_pose_data((0.45, 0.28, 0.32))
JOINT_TABLE_POSE_RAD = [
    math.radians(-2.7),
    math.radians(-0.1),
    math.radians(90.0),
    math.radians(-5.5),
    math.radians(-90.7),
    math.radians(0.9),
]
JOINT_BASKET_POSE_RAD = [
    math.radians(167.6),
    math.radians(-0.1),
    math.radians(90.0),
    math.radians(-5.5),
    math.radians(-90.7),
    math.radians(0.9),
]
JOINT_HOME_POSE_RAD = [
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
]
PRE_GRASP_POSE_DATA = _make_pose_data((0.28, -0.10, 0.35))
GRASP_POSE_DATA = _make_pose_data((0.28, -0.10, 0.20))
TRANSPORT_POSE_DATA = _make_pose_data((0.48, 0.10, 0.40))
DROP_POSE_DATA = _make_pose_data((0.48, 0.20, 0.33))
PICK_SEQUENCE = [
    ("PRE_GRASP", PRE_GRASP_POSE_DATA, 0.6),
    ("GRASP", GRASP_POSE_DATA, 0.8),
    ("TRANSPORT", TRANSPORT_POSE_DATA, 0.6),
    ("DROP", DROP_POSE_DATA, 0.8),
]


def _is_camera_topic(topic: str) -> bool:
    """Return True for the candidate camera topics we care about."""
    if not topic:
        return False
    normalized = topic.strip()
    if not normalized.startswith(CAMERA_TOPIC_PREFIX):
        return False
    leaf = normalized.split("/")[-1].lower()
    return "image" in leaf or "depth" in leaf


from .cameras_tab import ObjectListPanel
from .calibration_service import CalibrationService, CalibrationMode
from .ur5_kinematics import fk_ur5


class CameraView(QLabel):
    """Large camera view that auto-scales to the widget size."""
    
    clicked = pyqtSignal(int, int)  # Emite (x, y) en píxeles de la imagen original

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._qimg = None
        self._img_width = 0
        self._img_height = 0
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(480, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setText(title)
        self.setStyleSheet("background:#0b0f14; color:#94a3b8; border:1px solid #1f2937;")

    def set_frame(self, qimg, width=0, height=0):
        self._qimg = qimg
        self._img_width = width
        self._img_height = height
        self._update_pixmap()

    def mousePressEvent(self, event):
        """Convertir click en widget a coordenadas de imagen."""
        if event.button() == Qt.LeftButton and self._qimg is not None:
            # Obtener coordenadas del click en el widget
            widget_x = event.x()
            widget_y = event.y()
            
            # Convertir a coordenadas de imagen
            if self._img_width > 0 and self._img_height > 0:
                pixmap = self.pixmap()
                if pixmap:
                    # Calcular offset del pixmap centrado
                    px_w = pixmap.width()
                    px_h = pixmap.height()
                    offset_x = (self.width() - px_w) // 2
                    offset_y = (self.height() - px_h) // 2
                    
                    # Coordenadas relativas al pixmap
                    rel_x = widget_x - offset_x
                    rel_y = widget_y - offset_y
                    
                    # Escalar a imagen original
                    if 0 <= rel_x < px_w and 0 <= rel_y < px_h:
                        img_x = int(rel_x * self._img_width / px_w)
                        img_y = int(rel_y * self._img_height / px_h)
                        img_x = max(0, min(self._img_width - 1, img_x))
                        img_y = max(0, min(self._img_height - 1, img_y))
                        self.clicked.emit(img_x, img_y)
        super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_pixmap(self):
        if self._qimg is None:
            return
        pix = QPixmap.fromImage(self._qimg)
        pix = pix.scaled(self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(pix)


class _PanelLogger:
    """Logger façade so ControlPanelV2 can call get_logger().info(...)."""

    def __init__(self, panel: "ControlPanelV2"):
        self._panel = panel

    def info(self, msg: str) -> None:
        self._panel._log(msg)


class ControlPanelV2(QMainWindow):
    retry_send_joints = pyqtSignal()
    status_updated = pyqtSignal(bool, bool, bool, bool, bool, bool, bool)
    signal_status = pyqtSignal(str, bool)
    signal_refresh_controls = pyqtSignal()
    signal_set_led = pyqtSignal(object, str)
    signal_update_objects = pyqtSignal()
    signal_start_objects_settle_watch = pyqtSignal()
    signal_handle_objects_settled = pyqtSignal()
    signal_schedule_camera_health_check = pyqtSignal(int)
    signal_update_camera_topics = pyqtSignal(object)
    signal_connect_camera = pyqtSignal()
    signal_request_auto_bridge_start = pyqtSignal()
    signal_bridge_ready = pyqtSignal()
    signal_calibration_check = pyqtSignal()
    signal_trace_ready = pyqtSignal()
    signal_schedule_home_offset = pyqtSignal(int, int)
    signal_close_panel = pyqtSignal()
    signal_tf_ready = pyqtSignal(bool)
    signal_calib_ready = pyqtSignal(bool)
    signal_controllers_ready = pyqtSignal(bool)
    signal_error = pyqtSignal(str)
    signal_moveit_state = pyqtSignal(str, str)
    def _ui_set_status(self, text: str, error: bool = False) -> None:
        self.signal_status.emit(text, error)

    def _camera_health_check(self):
        """Chequeo periódico: si no llegan imágenes, log throttled y alerta visual."""
        if not self._camera_required:
            return
        now = time.time()
        age = now - self._last_camera_frame_ts if self._last_camera_frame_ts else float("inf")
        if self._camera_subscribed and age > 2.5:
            if (now - getattr(self, '_last_camera_diag_log', 0.0)) > 2.5:
                self._emit_log(f"[CAMERA][DIAG] No llegan imágenes desde hace {age:.1f}s en {self.camera_topic or 'N/A'}")
                self._last_camera_diag_log = now
            self.camera_info.setText(f"Sin imágenes ({age:.1f}s)")
            self.camera_info.setStyleSheet("color: #f43f5e; font-weight: bold;")
        elif self._camera_subscribed:
            self.camera_info.setStyleSheet("")
        # Timer controlado por _camera_health_timer.

    def get_health_report(self) -> dict:
        """Emitir un resumen JSON del pipeline: topics, nodos, servicios, world_name, pose_info, TF, cámaras, controllers, MoveIt2, etc."""
        report = {}
        # Topics y tipos
        topics_types = []
        try:
            if self._ros_worker_started and self.ros_worker.node_ready():
                topics_types = self.ros_worker.topic_names_and_types()
        except Exception:
            pass
        report["topics_types"] = topics_types
        # Nodos y servicios
        nodes = []
        services = []
        actions = []
        try:
            if self._ros_worker_started and self.ros_worker.node_ready():
                nodes = self.ros_worker.list_node_names() if hasattr(self.ros_worker, "list_node_names") else []
                services = self.ros_worker.list_service_names() if hasattr(self.ros_worker, "list_service_names") else []
                actions = self.ros_worker.list_action_names() if hasattr(self.ros_worker, "list_action_names") else []
        except Exception:
            pass
        report["nodes"] = nodes
        report["services"] = services
        report["actions"] = actions
        # world_name detectado y pose_info_topic
        world_name = self._gz_world_name or self._detect_world_name() or "unknown"
        pose_info_topic = self._discover_pose_info_topic(world_name)
        report["world_name_detected"] = world_name
        return report

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel V2")
        self.setMinimumWidth(900)
        self.ws_dir = os.environ.get("WS_DIR", WS_DIR)
        self.gz_proc = None
        self.bridge_proc = None
        self.bag_proc = None
        self.moveit_proc = None
        self.moveit_bridge_proc = None
        self.release_service_proc = None
        self.world_tf_proc = None
        self.rsp_proc = None
        self.gz_partition = ""
        self._status_check_inflight = False
        self._gz_running = False
        self._gz_state = "GAZEBO_OFF"
        self._gz_state_pending = ""
        self._gz_state_change_ts = 0.0
        self._last_clock_ok_ts = 0.0
        self._gz_world_name = None
        self._bridge_running = False
        self._moveit_running = False
        self._moveit_bridge_running = False
        self._gz_launching = False
        self._gz_launch_start = 0.0
        self._bridge_launching = False
        self._bridge_launch_start = 0.0
        self._moveit_launching = False
        self._moveit_launch_start = 0.0
        self._controller_spawn_last_start = 0.0
        self._tf_ready_timer: Optional[QTimer] = None
        self._tf_ready_last_notice = 0.0
        self._tf_ready_state = False
        self._tf_not_ready_logged = False
        self._trace_ready = False
        self._bridge_ready = False
        self._trace_print_period = max(0.5, TRACE_PRINT_PERIOD_SEC)
        self._ee_warn_period = 5.0
        self._trace_debug_logged = False
        self._panel_start_ts = time.time()
        self._debug_logs_enabled = os.environ.get("DEBUG_LOGS_TO_STDOUT", "0") == "1"
        self._panel_logger = _PanelLogger(self)
        if CAMERA_REQUIRED is None:
            self._camera_required = os.environ.get("PANEL_GZ_GUI", "0").strip().lower() in ("1", "true", "yes", "on")
        else:
            self._camera_required = bool(CAMERA_REQUIRED)
        self._system_state = SystemState.BOOT
        self._system_state_reason = "boot"
        self._system_error_reason = ""
        self._managed_mode = PANEL_MANAGED
        self._moveit_required = PANEL_MOVEIT_REQUIRED
        self._tf_invalid = False
        self._moveit_state = MoveItState.OFF
        self._moveit_state_reason = "manual"
        self._moveit_block_reason: Optional[str] = None
        self.signal_status.connect(self._set_status_async)
        self.signal_refresh_controls.connect(self._refresh_controls)
        self.signal_set_led.connect(self._set_led_async)
        self.signal_update_objects.connect(self._update_objects)
        self.signal_start_objects_settle_watch.connect(self._start_objects_settle_watch)
        self.signal_handle_objects_settled.connect(self._handle_objects_settled)
        self.signal_schedule_camera_health_check.connect(self._schedule_camera_health_check)
        self.signal_update_camera_topics.connect(self._update_camera_topics_async)
        self.signal_connect_camera.connect(self._connect_camera)
        self.signal_request_auto_bridge_start.connect(self._request_auto_bridge_start)
        self.signal_bridge_ready.connect(self._on_bridge_ready)
        self.signal_calibration_check.connect(self._on_calibration_check)
        self.signal_trace_ready.connect(self._on_trace_ready)
        self.signal_schedule_home_offset.connect(self._schedule_home_offset_retry)
        self.signal_close_panel.connect(self.close)
        self._emit_log(
            f"[STARTUP] camera_required={self._camera_required} "
            f"PANEL_CAMERA_REQUIRED={_CAMERA_REQUIRED_ENV or 'unset'} "
            f"debug_logs_enabled={self._debug_logs_enabled}"
        )
        self.signal_tf_ready.connect(self._on_tf_ready_signal)
        self.signal_calib_ready.connect(self._on_calib_ready_signal)
        self.signal_controllers_ready.connect(self._on_controllers_ready_signal)
        self.signal_error.connect(self._on_error_signal)
        self.signal_moveit_state.connect(self._on_moveit_state_signal)
        self._moveit_node: Optional[Node] = None
        self._moveit_pose_pub = None
        self._gripper_pub = None
        self._gripper_topic = ""
        self._traj_pub = None
        self._traj_topic = ""
        self._traj_action_client = None
        self._traj_action_name = ""
        self._traj_action_inflight = False
        self._traj_fallback_last_ts = 0.0
        self._moveit_action_client = None
        self._controller_client = None
        self._controller_client_name = ""
        self._objects_settled = False
        self._objects_seen_fall = False
        self._settle_worker_active = False
        self._settle_thread: Optional[threading.Thread] = None
        self._objects_release_done = False
        self._pose_info_ok = False
        self._pose_info_msg_count = 0
        self._pose_info_last_age = float("inf")
        self._pose_info_last_log = 0.0
        self._pose_info_timer: Optional[QTimer] = None
        self._physics_runtime_check_scheduled = False
        self._pose_info_resub_ts = 0.0
        self._pose_info_diag_logged = False
        self._tf_no_msgs_logged = False
        self._pick_block_reason: Optional[str] = None
        self._fall_test_last_log = 0.0
        self._detach_feature_checked = False
        self._detach_feature_available = False
        self._detach_feature_logged = False
        self._started_gazebo = False
        self._started_bridge = False
        self._started_moveit = False
        self._started_moveit_bridge = False
        self._started_release_service = False
        self._started_world_tf = False
        self._started_rsp = False
        self._started_bag = False
        self._detach_inflight = False
        self._detach_attempted = False
        self._detach_auto_disabled = False
        self._detach_backoff_until = 0.0
        self._trace_transform_warn_last: Dict[str, float] = {}
        self._trace_transform_warn_count: Dict[str, int] = {}
        self._trace_transform_warn_period = 5.0
        self._pick_disable_warn_ts = 0.0
        self._pick_tf_inflight = False
        self._pick_log_last_sig = ""
        self._pick_log_last_ts = 0.0
        self._fall_test_active = False
        self._settle_log_once_done = False
        self._settle_log_snapshot_next = False
        self._settle_log_snapshot_active = False
        self._bag_running = False
        self._reset_trace_throttle("init")
        self._timers_started = False
        self._gripper_closed = False
        self._manual_inflight = False
        self._manual_pending = False
        self._script_motion_active = False
        self._closing = False
        self._shutdown_complete = False
        self._last_tcp_world = None
        self._last_tcp_rpy_deg = None
        self._debug_joints_to_stdout = os.environ.get("DEBUG_JOINTS_TO_STDOUT", "0") == "1"
        self.joint_topic = os.environ.get("PANEL_JOINT_STATES_TOPIC", "/joint_states")
        self._joint_subscribed = False
        self._last_joint_positions: Dict[str, float] = {}
        self._last_joint_time: float = 0.0
        self._last_joint_stamp: float = 0.0
        self._joint_current_topic = ""
        self._joint_active = False
        self._joint_names_warned = False
        self.dof_pos_labels: Dict[str, QLabel] = {}
        self.dof_vel_labels: Dict[str, QLabel] = {}
        self.gripper_labels: Dict[str, QLabel] = {}
        self.gripper_total_lbl: Optional[QLabel] = None
        self.tcp_xyz_lbl: Optional[QLabel] = None
        self.tcp_rpy_lbl: Optional[QLabel] = None
        self.vel_norm_lbl: Optional[QLabel] = None
        self.vel_max_lbl: Optional[QLabel] = None
        self.eff_max_lbl: Optional[QLabel] = None
        self.joint_sliders = []
        self.joint_value_labels = []
        self._last_slider_values = {}  # Track cambios en sliders para debug
        self._slider_update_blocked_until = 0.0  # Bloquea actualizaciones de sliders por gestos manuales
        self._updating_sliders_from_joint_state = False  # Flag para evitar loops
        self.camera_topic = os.environ.get("PANEL_CAMERA_TOPIC", "/camera_overhead/image")
        self._camera_subscribed = False
        self._camera_stream_ok = False
        self._camera_topic_hz = 0.0
        self._camera_topic_check_inflight = False
        self._camera_health_retry_scheduled = False
        self._camera_frame_count = 0
        self._camera_ready_frames = max(1, CAMERA_READY_FRAMES)
        self._calibrating = False
        self._calib_points = []  # Lista de (px, py, wx, wy)
        self._auto_joint2_move_done = False
        self._selected_object = None  # Objeto seleccionado para pick
        self._controller_check_inflight = False
        self._controllers_ok = False
        self._controllers_reason = "controladores no verificados"
        self._controllers_state = "STARTING"
        self._last_controller_check = 0.0
        self._controller_spawn_inflight = False
        self._controller_spawn_done = False
        self._selected_px = None  # Píxel seleccionado (px, py)
        self._selected_world = None  # Coordenadas del mundo (x, y)
        self._last_tf_status: Optional[Dict[str, object]] = None
        self._last_selection_frame: Optional[str] = WORLD_FRAME or "world"
        self._base_frame_effective: Optional[str] = BASE_FRAME or "base"
        self._selection_timestamp: float = 0.0
        self._ee_frame_effective: Optional[str] = None
        self._last_selected_world_pose: Optional[Tuple[float, float, float, str]] = None
        self._last_selected_base_pose: Optional[Tuple[float, float, float, str]] = None
        self._last_ee_warn_ts: float = 0.0
        self._last_ee_diag_ts: float = 0.0
        self._trace_timer: Optional[QTimer] = None
        self.trace_group: Optional[QGroupBox] = None
        self.trace_table: Optional[QTableWidget] = None
        self.chk_trace_freeze: Optional[QCheckBox] = None
        self._spawn_positions_snapshot = get_object_positions()
        self._external_state: Optional[str] = None
        self._external_state_reason: str = ""
        self._external_state_last: float = 0.0
        self._emit_log(
            f"[STARTUP] panel_v2 argv0={os.path.abspath(sys.argv[0])} file={os.path.abspath(__file__)}"
        )
        self.lbl_trace_frames: Optional[QLabel] = None
        self.lbl_trace_error_base: Optional[QLabel] = None
        self.lbl_trace_error_world: Optional[QLabel] = None
        self.lbl_trace_tf_translation: Optional[QLabel] = None
        self.lbl_trace_tf_yaw: Optional[QLabel] = None
        self.btn_copy_trace: Optional[QPushButton] = None
        self.lbl_moveit_status: Optional[QLabel] = None
        self.lbl_moveit_bridge_status: Optional[QLabel] = None
        self._trace_cached_text = ""
        self._pose_stream_proc = {}  # Slot para proceso de stream de poses
        self._pose_debug_timer: Optional[QTimer] = None
        self._auto_bridge_attempts = 0
        self._auto_bridge_timer_scheduled = False
        self._bridge_start_ts = 0.0
        self.runner = CmdRunner()
        self.runner.line.connect(lambda msg: self._log(msg))
        self._emit_log("[STARTUP] Limpieza de procesos fantasma")
        self._cleanup_stray_processes()
        self._emit_log("[STARTUP] Limpieza de cache Python")
        self._clean_cache_dirs()
        self._emit_log("[STARTUP] Creando CalibrationService")
        self.calib_service = CalibrationService(log_fn=self._log)
        self._emit_log("[STARTUP] Creando RosWorker")
        self.ros_worker = RosWorker(force_realtime=True)
        self.ros_worker.image.connect(self._on_image)
        self.ros_worker.joint_state.connect(self._on_joint_state)
        self.ros_worker.log.connect(self._log_ros_message)
        self.ros_worker.system_state.connect(self._on_system_state_update)
        self._ros_worker_started = False
        self._ensure_ros_worker_started()
        self._calibration_ready = False
        self._last_calib_block_log = 0.0
        self._emit_log("[STARTUP] Inicializando publisher MoveIt")
        self._init_moveit_publisher()
    
        # Conectar señal de retry para movimiento manual (thread-safe)
        self.retry_send_joints.connect(self._send_joints_retry)
    
        self._emit_log("[STARTUP] Construyendo UI")
        self._build_ui()
        self._emit_log("[STARTUP] UI lista")
    
        # Timer para actualizar objetos
        self.objects_timer = QTimer(self)
        self.objects_timer.timeout.connect(self._update_objects)
        self.objects_timer.start(1000)  # Actualizar cada segundo
    
        # Timer para auto-conectar cámara (activado tras lanzar el bridge)
        # (no se programa inmediatamente para evitar logs antes de que el usuario arranque Gazebo/bridge)
        # Suscribir joint_states con un temporizador de reintento corto
        self.joint_timer = QTimer(self)
        self.joint_timer.timeout.connect(self._auto_subscribe_joints)
        self.joint_timer.start(800)
    
        # Inicializar estado UI (arranque manual por botones del panel)
        # (self._update_ui_state() se llama al final de _build_ui)
        self._emit_log("[STARTUP] UI state inicial aplicado")
    
        # Forzar estado inicial en OFF para LEDs y permitir arranque manual
        self._emit_log("[STARTUP] LEDs forzados a OFF")

        # Chequeo de estado asíncrono tras 1s
        QTimer.singleShot(1000, self._refresh_status_async)
        if not self._managed_mode:
            self._request_auto_bridge_start()
    
        # Test inicial de logging
        self._emit_log("[PANEL-V2] Panel iniciado - logging activo")
        self._emit_log(f"[PANEL-V2] Debug logs enabled: {self._debug_logs_enabled}")

    def get_logger(self):
        return self._panel_logger

    def _init_moveit_publisher(self) -> None:
        if not ROS_AVAILABLE:
            self._log("[Panel] ROS no disponible; MoveIt publisher deshabilitado.")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception as exc:
            self._log(f"[Panel] Advertencia: rclpy.init falló ({exc}), se intentará de nuevo.")
        try:
            if self._moveit_node is None:
                use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
                overrides = None
                if Parameter is not None:
                    overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))]
                node_kwargs = {}
                if overrides:
                    node_kwargs["parameter_overrides"] = overrides
                self._moveit_node = rclpy.create_node("panel_v2_moveit_publisher", **node_kwargs)
            if self._moveit_pose_pub is None and self._moveit_node is not None:
                self._moveit_pose_pub = self._moveit_node.create_publisher(PoseStamped, MOVEIT_POSE_TOPIC, 10)
            if self._moveit_pose_pub:
                self._log(f"[Panel] MoveIt publisher listo en {MOVEIT_POSE_TOPIC}")
        except Exception as exc:
            self._log(f"[Panel] ERROR creando publisher MoveIt: {exc}")
            self._moveit_pose_pub = None

    @pyqtSlot()
    def _request_auto_bridge_start(self) -> None:
        if self._managed_mode:
            return
        if not AUTO_START_BRIDGE or self._closing or self._bridge_running:
            return
        if self._auto_bridge_timer_scheduled:
            return
        self._auto_bridge_timer_scheduled = True
        if self._auto_bridge_attempts == 0:
            self._log("[AUTO] Autostart bridge: esperando Gazebo")
        QTimer.singleShot(AUTO_START_BRIDGE_DELAY_MS, self._auto_bridge_tick)

    def _auto_bridge_tick(self) -> None:
        if self._managed_mode:
            return
        self._auto_bridge_timer_scheduled = False
        if not AUTO_START_BRIDGE or self._closing or self._bridge_running:
            return
        gz_state = self._gazebo_state()
        if gz_state == "GAZEBO_OFF":
            self._auto_bridge_attempts += 1
            if self._auto_bridge_attempts >= AUTO_START_BRIDGE_MAX_RETRIES:
                self._log("[AUTO] Autostart bridge: timeout (Gazebo no disponible)")
                return
            self._auto_bridge_timer_scheduled = True
            QTimer.singleShot(1000, self._auto_bridge_tick)
            return
        self._log("[AUTO] Autostart bridge: Gazebo activo")
        self._start_bridge()

    def _get_traj_publisher(self, topic: str):
        if self._moveit_node is None:
            return None
        if self._traj_pub is None or self._traj_topic != topic:
            try:
                self._traj_pub = self._moveit_node.create_publisher(JointTrajectory, topic, 10)
                self._traj_topic = topic
            except Exception as exc:
                self._log(f"[Panel] ERROR creando publisher JointTrajectory: {exc}")
                self._traj_pub = None
        return self._traj_pub

    def _get_gripper_publisher(self, topic: str):
        if self._moveit_node is None:
            return None
        if self._gripper_pub is None or self._gripper_topic != topic:
            try:
                self._gripper_pub = self._moveit_node.create_publisher(Float64MultiArray, topic, 10)
                self._gripper_topic = topic
            except Exception as exc:
                self._log(f"[Panel] ERROR creando publisher gripper: {exc}")
                self._gripper_pub = None
        return self._gripper_pub

    def _traj_action_target(self, traj_topic: str) -> str:
        if not traj_topic:
            return ""
        base = traj_topic.rsplit("/joint_trajectory", 1)[0]
        return f"{base}/follow_joint_trajectory"

    def _joint_motion_since(self, snapshot: Dict[str, float]) -> bool:
        if not snapshot:
            return False
        for name, prev in snapshot.items():
            curr = self._last_joint_positions.get(name)
            if curr is None:
                continue
            if abs(curr - prev) > TRAJ_ACTION_FALLBACK_EPS_RAD:
                return True
        return False

    def _send_joint_trajectory_action(self, positions: List[float], sec: float, traj_topic: str) -> Tuple[bool, str]:
        if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
            return False, "Action FollowJointTrajectory no disponible"
        if self._moveit_node is None:
            return False, "Nodo ROS no listo"
        action_name = self._traj_action_target(traj_topic)
        if not action_name:
            return False, "Action target vacío"
        if self._traj_action_client is None or self._traj_action_name != action_name:
            try:
                self._traj_action_client = ActionClient(self._moveit_node, FollowJointTrajectory, action_name)
                self._traj_action_name = action_name
            except Exception as exc:
                self._traj_action_client = None
                self._traj_action_name = ""
                return False, f"ActionClient error: {exc}"
        client = self._traj_action_client
        if client is None:
            return False, "ActionClient no disponible"
        if not client.wait_for_server(timeout_sec=1.5):
            return False, f"Action server no disponible: {action_name}"
        sec = max(0.0, float(sec))
        sec_i = int(sec)
        nsec_i = int((sec - sec_i) * 1e9)
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(UR5_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [round(p, 4) for p in positions]
        point.time_from_start.sec = sec_i
        point.time_from_start.nanosec = nsec_i
        traj.points = [point]
        goal.trajectory = traj
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=TRAJ_ACTION_FALLBACK_TIMEOUT_SEC)
        goal_handle = future.result() if future.done() else None
        if not goal_handle or not goal_handle.accepted:
            return False, "Goal rechazado"
        return True, action_name

    def _schedule_traj_action_fallback(self, positions: List[float], sec: float, traj_topic: str) -> None:
        if not TRAJ_ACTION_FALLBACK:
            return
        if self._traj_action_inflight:
            return
        now = time.time()
        if (now - self._traj_fallback_last_ts) < 1.0:
            return
        snapshot = dict(self._last_joint_positions)

        def worker():
            time.sleep(TRAJ_ACTION_FALLBACK_DELAY_SEC)
            if self._closing:
                return
            if self._joint_motion_since(snapshot):
                return
            self._traj_action_inflight = True
            try:
                ok, info = self._send_joint_trajectory_action(positions, sec, traj_topic)
                if ok:
                    self._traj_fallback_last_ts = time.time()
                    self._emit_log(f"[ROBOT] Fallback action en {info}")
                else:
                    self._emit_log(f"[ROBOT] Fallback action falló: {info}")
            finally:
                self._traj_action_inflight = False

        threading.Thread(target=worker, daemon=True).start()

    def _publish_joint_trajectory(self, positions: List[float], sec: float) -> Tuple[bool, str]:
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return False, "Nodo ROS no listo"
        ok, reason = self._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
        if not ok:
            return False, f"controladores no listos: {reason}"
        topic = self._select_traj_topic()
        if not topic:
            return False, "joint_trajectory_controller no disponible"
        pub = self._get_traj_publisher(topic)
        if not pub:
            return False, "Publisher JointTrajectory no disponible"
        sec = max(0.0, float(sec))
        sec_i = int(sec)
        nsec_i = int((sec - sec_i) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = list(UR5_JOINT_NAMES)
        try:
            traj.header.stamp = Time().to_msg()
        except Exception:
            pass
        point = JointTrajectoryPoint()
        point.positions = [round(p, 4) for p in positions]
        point.time_from_start.sec = sec_i
        point.time_from_start.nanosec = nsec_i
        traj.points = [point]
        self._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
        pub.publish(traj)
        return True, topic

    def _publish_moveit_pose(self, label: str, pose_data: Dict[str, object]) -> None:
        if not self._moveit_required:
            self._set_status(f"MoveIt deshabilitado; bloqueando {label}", error=True)
            self._emit_log(f"[MOVEIT] Bloqueado: {label} (MoveIt deshabilitado)")
            return
        if self._managed_mode and not self._state_ready_moveit():
            reason = self._system_state_reason or self._system_state.value
            self._set_status(f"Sistema no listo; esperando {label} ({reason})", error=False)
            self._emit_log(f"[MOVEIT] Bloqueado: {label} (state={self._system_state.value})")
            return
        if self._moveit_state != MoveItState.READY:
            reason = self._moveit_state_reason or "MoveIt no listo"
            if reason != self._moveit_block_reason:
                self._set_status(f"MoveIt no listo; esperando {label}", error=False)
                self._emit_log(f"[MOVEIT] Bloqueado: {label} ({reason})")
                self._moveit_block_reason = reason
            return
        self._moveit_block_reason = None
        if self._moveit_pose_pub is None or self._moveit_node is None:
            self._log(f"[Panel] LEGACY: MoveIt publisher no inicializado - no se envió pose {label}.")
            return
        pose = _build_pose_stamped(pose_data)
        try:
            pose.header.stamp = Time().to_msg()
        except Exception:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
        self._moveit_pose_pub.publish(pose)
        self.get_logger().info(
            f"[Panel] Sent {label} pose to MoveIt on {MOVEIT_POSE_TOPIC}; MoveIt is responsible for the motion."
        )

    @pyqtSlot()
    def _start_objects_settle_watch(self) -> None:
        if self._closing:
            return
        if self._settle_worker_active:
            return
        if not self._pose_info_ok:
            now = time.monotonic()
            if (now - getattr(self, '_last_settle_gating_log', 0.0)) > 1.5:
                self._emit_log("[PHYSICS][SETTLE] gating: pose_info_ready=false, settle NO iniciado")
                self._last_settle_gating_log = now
            self._update_pose_info_status()
            return
        self._ensure_pose_subscription()
        self._objects_settled = False
        self._objects_seen_fall = False
        spawn = dict(get_object_positions())
        for name, pos in EXTRA_OBJECTS.items():
            spawn[name] = pos
        for name, pos in TABLE_OBJECTS.items():
            spawn.setdefault(name, pos)
        self._spawn_positions_snapshot = spawn
        self._settle_log_snapshot_active = (
            self._settle_log_snapshot_next or not self._settle_log_once_done
        )
        self._settle_log_snapshot_next = False
        self._settle_worker_active = True
        if self._settle_thread and self._settle_thread.is_alive():
            return
        self._settle_thread = threading.Thread(target=self._objects_settle_worker, daemon=True)
        self._settle_thread.start()

    def _invalidate_settle(self, reason: str, *, restart: bool = True) -> None:
        if self._closing:
            return
        self._objects_settled = False
        self._objects_seen_fall = False
        self._calibration_ready = False
        self._spawn_positions_snapshot = get_object_positions()
        self._emit_log(f"[PHYSICS] Revalidar settle: {reason}")
        if restart and self._gz_running:
            self.signal_start_objects_settle_watch.emit()

    def _run_fall_test_async(self) -> None:
        if self._fall_test_active or not self._gz_running or self._closing:
            return
        if not self._pose_info_ok:
            now = time.monotonic()
            if (now - self._fall_test_last_log) >= POSE_INFO_LOG_PERIOD:
                self._emit_log("[PHYSICS][FALL_TEST] waiting pose/info data...")
                self._fall_test_last_log = now
            return
        self._fall_test_active = True
        world_path = self.world_combo.currentText().strip()
        world_name = self._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)

        def worker():
            try:
                poses_t0 = self._read_world_pose_info(world_name)
                if not poses_t0:
                    self._emit_log("[PHYSICS][FALL_TEST] no pose data (bridge?)")
                    return
                t0_map: Dict[str, float] = {}
                for pose in poses_t0:
                    name = pose.get("name")
                    if name in DROP_OBJECT_NAMES:
                        pos = pose.get("position") or {}
                        t0_map[name] = float(pos.get("z") or 0.0)
                time.sleep(max(0.1, FALL_TEST_DELAY_SEC))
                poses_t1 = self._read_world_pose_info(world_name)
                if not poses_t1:
                    self._emit_log("[PHYSICS][FALL_TEST] no pose data at t1")
                    return
                t1_map: Dict[str, float] = {}
                for pose in poses_t1:
                    name = pose.get("name")
                    if name in DROP_OBJECT_NAMES:
                        pos = pose.get("position") or {}
                        t1_map[name] = float(pos.get("z") or 0.0)
                for name in DROP_OBJECT_NAMES:
                    if name not in t0_map or name not in t1_map:
                        continue
                    dz = t0_map[name] - t1_map[name]
                    ok = dz >= OBJECT_FALL_Z_EPS
                    self._emit_log(
                        f"[PHYSICS][FALL_TEST] model={name} dz={dz:.3f} ok={str(ok).lower()}"
                    )
            finally:
                self._fall_test_active = False

        threading.Thread(target=worker, daemon=True).start()

    def _objects_settle_worker(self) -> None:
        settled = self.wait_for_objects_to_settle(
            timeout=OBJECT_SETTLE_TIMEOUT_SEC,
            log_snapshot=self._settle_log_snapshot_active,
        )
        if not settled:
            # Snapshot y log de diagnóstico automático en fallo
            self._log_settle_snapshot("fail/timeout")
            self._emit_log("[PHYSICS][SETTLE][DIAG] Snapshot automático por fallo de settle.")
        if not settled and ALLOW_UNSETTLED_ON_TIMEOUT:
            self._emit_log("[PHYSICS][SETTLE] Continuando sin estabilizar (override).")
            settled = True
        self._objects_settled = settled
        if settled:
            self._emit_log("[PHYSICS][SETTLE] OK")
            self.signal_handle_objects_settled.emit()
            self.signal_status.emit("Objetos estabilizados", False)
        else:
            self._emit_log("[PHYSICS][SETTLE] Timeout: objetos no estabilizados.")
            self.signal_status.emit("Timeout: objetos no estabilizados", True)
        self._settle_worker_active = False
        self.signal_refresh_controls.emit()

    @pyqtSlot()
    def _handle_objects_settled(self) -> None:
        self._refresh_controls()
        if self._bridge_running and not self._calibration_ready:
            self.signal_calibration_check.emit()

    def _log_calib_blocked(self, reason: str) -> None:
        now = time.time()
        if (now - self._last_calib_block_log) < 1.5:
            return
        self._last_calib_block_log = now
        self._emit_log(f"[CALIB] Bloqueada: {reason}")

    def _log_settle_snapshot(self, reason: str) -> None:
        world_path = self.world_combo.currentText().strip()
        world_name = self._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)
        targets = self._settle_targets()
        if not targets:
            return
        poses = self._read_world_pose_info(world_name)
        if not poses:
            return
        self._emit_log(f"[PHYSICS][SETTLE] snapshot({reason})")
        for pose in poses:
            name = pose.get("name")
            if name not in targets:
                continue
            position = pose.get("position") or {}
            try:
                x = float(position.get("x") or 0.0)
                y = float(position.get("y") or 0.0)
                z = float(position.get("z") or 0.0)
            except Exception:
                continue
            self._emit_log(f"[PHYSICS][SETTLE] model={name} z={z:.3f}")

    def _request_settle_snapshot(self, reason: str) -> None:
        if not self._gz_running:
            return
        self._settle_log_snapshot_next = True
        self._log_settle_snapshot(reason)

    def wait_for_objects_to_settle(
        self,
        timeout: float = OBJECT_SETTLE_TIMEOUT_SEC,
        *,
        log_snapshot: bool = False,
    ) -> bool:
        self._log_calib_blocked("esperando caída/estabilidad de objetos")
        world_path = self.world_combo.currentText().strip()
        world_name = self._gz_world_name or (read_world_name(world_path) if world_path else GZ_WORLD)
        targets = self._settle_targets()
        if not targets:
            self._log("[PHYSICS][SETTLE] No hay objetos dinámicos configurados para monitorizar.")
            return False
        start: Optional[float] = None
        pose_wait_start = time.time()
        stable_since: Dict[str, float] = {}
        stable_ok: Dict[str, bool] = {name: False for name in targets}
        has_fallen: Dict[str, bool] = {name: False for name in targets}
        attached_since: Dict[str, float] = {}
        attached_confirmed: Set[str] = set()
        attach_rel: Dict[str, Tuple[float, float, float]] = {}
        last_hand_pos: Optional[Tuple[float, float, float]] = None
        poses_seen = False
        spawn_positions = dict(self._spawn_positions_snapshot or {})
        last_positions: Dict[str, Tuple[float, float, float]] = {}
        last_block_log = 0.0
        last_pose_log = 0.0
        last_state_log: Dict[str, float] = {}
        state_log_period = 2.0
        if log_snapshot:
            self._log_settle_snapshot("inicio")
            self._settle_log_once_done = True
        while self._gz_running and not self._closing and ((time.time() - start) < timeout if start else True):
            poses = self._read_world_pose_info(world_name)
            if not poses:
                if (time.time() - pose_wait_start) >= timeout:
                    self._emit_log("[PHYSICS][SETTLE] Timeout: sin pose/info.")
                    return False
                if (time.time() - last_pose_log) >= 2.0:
                    self._emit_log("[PHYSICS][SETTLE] waiting pose/info data...")
                    last_pose_log = time.time()
                time.sleep(OBJECT_SETTLE_POLL_SEC)
                continue
            poses_seen = True
            if start is None:
                start = time.time()
            now = time.time()
            round_ok = True
            pose_map: Dict[str, Tuple[float, float, float]] = {}
            for pose in poses:
                name = pose.get("name")
                if not name:
                    continue
                position = pose.get("position") or {}
                pose_map[name] = (
                    float(position.get("x") or 0.0),
                    float(position.get("y") or 0.0),
                    float(position.get("z") or 0.0),
                )
            hand_pos = None
            for key, pos in pose_map.items():
                if any(key.endswith(f"::{cand}") or key == cand for cand in HAND_LINK_CANDIDATES):
                    hand_pos = pos
                    break
            hand_moved = False
            if hand_pos and last_hand_pos:
                dxh = hand_pos[0] - last_hand_pos[0]
                dyh = hand_pos[1] - last_hand_pos[1]
                dzh = hand_pos[2] - last_hand_pos[2]
                hand_moved = (dxh * dxh + dyh * dyh + dzh * dzh) ** 0.5 > ATTACH_HAND_MOVE_EPS
            if hand_pos:
                last_hand_pos = hand_pos
            for pose in poses:
                name = pose.get("name")
                if name not in targets:
                    continue
                position = pose.get("position") or {}
                x = float(position.get("x") or 0.0)
                y = float(position.get("y") or 0.0)
                z = float(position.get("z") or 0.0)
                prev = last_positions.get(name)
                dz = z - prev[2] if prev else 0.0
                dx = x - prev[0] if prev else 0.0
                dy = y - prev[1] if prev else 0.0
                spawn_z = spawn_positions.get(name, (x, y, z))[2]
                if (spawn_z - z) > OBJECT_FALL_Z_EPS:
                    has_fallen[name] = True
                require_fall = any(name.startswith(p) for p in SETTLE_PATTERNS)
                if not require_fall:
                    has_fallen[name] = True
                if hand_pos and name.startswith("drop_obj_"):
                    dxh = x - hand_pos[0]
                    dyh = y - hand_pos[1]
                    dzh = z - hand_pos[2]
                    dist = (dxh * dxh + dyh * dyh + dzh * dzh) ** 0.5
                    if dist <= ATTACH_DIST_M:
                        prev_rel = attach_rel.get(name)
                        rel = (dxh, dyh, dzh)
                        rel_ok = True
                        if prev_rel:
                            dr = (
                                (rel[0] - prev_rel[0]) ** 2
                                + (rel[1] - prev_rel[1]) ** 2
                                + (rel[2] - prev_rel[2]) ** 2
                            ) ** 0.5
                            rel_ok = dr <= ATTACH_REL_EPS
                        attach_rel[name] = rel
                        if rel_ok and (hand_moved or dist <= ATTACH_SNAP_EPS):
                            attached_since.setdefault(name, now)
                            if (now - attached_since[name]) >= ATTACH_WINDOW_SEC:
                                attached_confirmed.add(name)
                        else:
                            attached_since.pop(name, None)
                    else:
                        attached_since.pop(name, None)
                stable = bool(prev) and abs(dz) <= OBJECT_SETTLE_Z_EPS and abs(dx) <= OBJECT_SETTLE_XY_EPS and abs(dy) <= OBJECT_SETTLE_XY_EPS
                if name in attached_confirmed:
                    has_fallen[name] = False
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                elif not has_fallen.get(name, False):
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                elif stable:
                    if name not in stable_since:
                        stable_since[name] = now
                    elif (now - stable_since[name]) >= OBJECT_SETTLE_WINDOW_SEC:
                        stable_ok[name] = True
                else:
                    stable_ok[name] = False
                    stable_since.pop(name, None)
                last_positions[name] = (x, y, z)
                last_log = last_state_log.get(name, 0.0)
                if (now - last_log) >= state_log_period:
                    dz_from_spawn = spawn_z - z
                    stable_flag = bool(has_fallen.get(name, False) and stable_ok.get(name, False))
                    self._emit_log(
                        f"[PHYSICS][SETTLE] model={name} z={z:.3f} dz={dz_from_spawn:.3f} "
                        f"has_fallen={has_fallen.get(name, False)} stable={stable_flag}"
                    )
                    last_state_log[name] = now
                if not (has_fallen.get(name, False) and stable_ok.get(name, False)):
                    round_ok = False
            if round_ok and targets:
                self._objects_seen_fall = True
                return True
            if (now - last_block_log) >= 1.8:
                self._log_calib_blocked("esperando caída/estabilidad de objetos")
                last_block_log = now
            time.sleep(OBJECT_SETTLE_POLL_SEC)
        if poses_seen:
            for name in sorted(attached_confirmed):
                self._emit_log(f"[PHYSICS][ATTACH] model={name} attached=true")
            for name in sorted(targets):
                require_fall = any(name.startswith(p) for p in SETTLE_PATTERNS)
                if require_fall and not has_fallen.get(name, False):
                    self._emit_log(
                        f"[PHYSICS][SETTLE] model={name} has_fallen=false -> revisar SDF: static/kinematic/gravity/inertial/joint"
                    )
                else:
                    self._emit_log(f"[PHYSICS][SETTLE] model={name} has_fallen=true stable={stable_ok.get(name, False)}")
        return False

    def _build_ui(self) -> None:
        root = QWidget()
        main = QVBoxLayout()
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(6)

        # --- Fila superior de botones y estado ---
        top = QHBoxLayout()
        top.setSpacing(6)

        self.btn_kill_hard = QPushButton("KILL HARD")
        self.btn_kill_hard.setToolTip("Uso manual de emergencia (kill_all.sh)")
        self.btn_close_terminal = QPushButton("Cerrar Terminal")
        self.btn_debug_joints = QPushButton("Debug joints/poses → terminal")
        self.btn_debug_joints.setCheckable(True)
        self.btn_debug_logs = QPushButton("Debug logs → terminal")
        self.btn_debug_logs.setCheckable(True)
        self.btn_debug_logs.setChecked(self._debug_logs_enabled)
        self._apply_debug_button_style(self.btn_debug_joints, self._debug_joints_to_stdout)
        self._apply_debug_button_style(self.btn_debug_logs, self._debug_logs_enabled)

        self.btn_kill_hard.clicked.connect(lambda: self._debounced_btn_action(self.btn_kill_hard, lambda: self._run_script("kill_all.sh", "KILL HARD")))
        self.btn_close_terminal.clicked.connect(self._close_terminal)
        self.btn_debug_logs.clicked.connect(lambda: self._toggle_debug("DEBUG_LOGS_TO_STDOUT"))

        self.status_lbl = QLabel("Listo · Bridge ROS ↔ Gazebo activo (0.0s)")
        self.status_lbl.setAlignment(Qt.AlignCenter)
        self.status_lbl.setStyleSheet(
            "background:#0f172a; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
        )

        # --- Combo de mundos y modos ---
        self.world_combo = QComboBox()
        self.world_combo.setEditable(True)
        self._fill_worlds()
        self.btn_world_browse = QPushButton("Mundo…")
        self.btn_world_browse.clicked.connect(self._choose_world)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Auto", "Headless", "GUI"])
        self.btn_gz_start = QPushButton("Lanzar Gazebo")
        self.btn_gz_stop = QPushButton("Detener Gazebo")
        self.btn_gz_start.clicked.connect(self._start_gazebo)
        self.btn_gz_stop.clicked.connect(self._stop_gazebo)
        self.btn_debug_joints.clicked.connect(self._toggle_debug_poses)

        # --- Bridge YAML ---
        self.bridge_presets = QComboBox()
        self._fill_bridge_presets()
        self.bridge_presets.currentTextChanged.connect(self._apply_bridge_preset)
        self.bridge_edit = QLineEdit(BRIDGE_BASE_YAML)
        self.btn_bridge_browse = QPushButton("YAML…")
        self.btn_bridge_browse.clicked.connect(self._choose_yaml)
        self.btn_bridge_start = QPushButton("Lanzar bridge")
        self.btn_bridge_stop = QPushButton("Detener bridge")
        self.btn_bridge_start.clicked.connect(lambda: self._debounced_btn_action(self.btn_bridge_start, self._start_bridge))
        self.btn_bridge_stop.clicked.connect(lambda: self._debounced_btn_action(self.btn_bridge_stop, self._stop_bridge))

        # --- Bag ---
        self.bag_name = QLineEdit(f"demo_{int(time.time())}")
        self.bag_topics = QLineEdit(
            "/camera_overhead/image /camera_north/image /camera_lateral/image /camera_east/image /camera_west/image"
        )
        self.btn_bag_start = QPushButton("Start bag")
        self.btn_bag_stop = QPushButton("Stop bag")
        self.btn_bag_start.clicked.connect(lambda: self._debounced_btn_action(self.btn_bag_start, self._start_bag))
        self.btn_bag_stop.clicked.connect(lambda: self._debounced_btn_action(self.btn_bag_stop, self._stop_bag))

        # --- MoveIt ---
        self.btn_moveit_start = QPushButton("Lanzar MoveIt")
        self.btn_moveit_stop = QPushButton("Detener MoveIt")
        self.btn_moveit_bridge_start = QPushButton("Lanzar MoveIt bridge")
        self.btn_moveit_bridge_stop = QPushButton("Detener MoveIt bridge")
        self.btn_moveit_start.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_start, self._start_moveit)
        )
        self.btn_moveit_stop.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_stop, self._stop_moveit)
        )
        self.btn_moveit_bridge_start.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_bridge_start, self._start_moveit_bridge)
        )
        self.btn_moveit_bridge_stop.clicked.connect(
            lambda: self._debounced_btn_action(self.btn_moveit_bridge_stop, self._stop_moveit_bridge)
        )

        # --- LEDs de estado ---
        self.led_gz = QLabel()
        self.led_bridge = QLabel()
        self.led_clock = QLabel()
        self.led_bag = QLabel()
        self.led_ros2 = QLabel()
        self.led_ur5 = QLabel()
        self.led_moveit = QLabel()
        self.led_moveit_bridge = QLabel()
        for led in (
            self.led_gz,
            self.led_bridge,
            self.led_clock,
            self.led_bag,
            self.led_ros2,
            self.led_ur5,
            self.led_moveit,
            self.led_moveit_bridge,
        ):
            set_led(led, "off")

        # --- Sistema: labels para CPU/RAM/Load ---
        self.sys_cpu_lbl = QLabel("CPU  --")
        self.sys_ram_lbl = QLabel("RAM  --")
        self.sys_load_lbl = QLabel("Load  --")
        self.sys_health_lbl = QLabel("Todo OK")
        for lbl in (self.sys_cpu_lbl, self.sys_ram_lbl, self.sys_load_lbl, self.sys_health_lbl):
            lbl.setTextInteractionFlags(Qt.NoTextInteraction)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status_async)
        self.status_updated.connect(self._apply_status)

        for btn in (
            self.btn_kill_hard,
            self.btn_close_terminal,
            self.btn_debug_joints,
            self.btn_debug_logs,
        ):
            btn.setMinimumHeight(32)

        top.addWidget(self.btn_kill_hard)
        top.addWidget(self.btn_close_terminal)
        top.addWidget(self.btn_debug_joints)
        top.addWidget(self.btn_debug_logs)
        top.addWidget(self.status_lbl, 1)

        main.addLayout(top)

        controls_status_row = QHBoxLayout()
        controls_status_row.setSpacing(6)
        controls_col = QVBoxLayout()
        controls_col.setSpacing(4)

        gz_row = QHBoxLayout()
        gz_row.setSpacing(6)
        gz_row.addWidget(QLabel("Mundo:"))
        gz_row.addWidget(self.world_combo, 1)
        gz_row.addWidget(self.btn_world_browse)
        gz_row.addWidget(QLabel("Modo:"))
        gz_row.addWidget(self.mode_combo)
        gz_row.addWidget(self.btn_gz_start)
        gz_row.addWidget(self.btn_gz_stop)
        controls_col.addLayout(gz_row)

        br_row = QHBoxLayout()
        br_row.setSpacing(6)
        br_row.addWidget(QLabel("Bridge YAML:"))
        br_row.addWidget(self.bridge_presets)
        br_row.addWidget(self.bridge_edit, 1)
        br_row.addWidget(self.btn_bridge_browse)
        br_row.addWidget(self.btn_bridge_start)
        br_row.addWidget(self.btn_bridge_stop)
        controls_col.addLayout(br_row)

        bag_row = QHBoxLayout()
        bag_row.setSpacing(6)
        bag_row.addWidget(QLabel("Bag nombre:"))
        bag_row.addWidget(self.bag_name, 1)
        bag_row.addWidget(QLabel("Tópicos:"))
        bag_row.addWidget(self.bag_topics, 2)
        bag_row.addWidget(self.btn_bag_start)
        bag_row.addWidget(self.btn_bag_stop)
        controls_col.addLayout(bag_row)

        moveit_row = QHBoxLayout()
        moveit_row.setSpacing(6)
        moveit_row.addWidget(QLabel("MoveIt:"))
        moveit_row.addWidget(self.btn_moveit_start)
        moveit_row.addWidget(self.btn_moveit_stop)
        moveit_row.addSpacing(8)
        moveit_row.addWidget(QLabel("Bridge:"))
        moveit_row.addWidget(self.btn_moveit_bridge_start)
        moveit_row.addWidget(self.btn_moveit_bridge_stop)
        moveit_row.addStretch(1)
        controls_col.addLayout(moveit_row)

        controls_status_row.addLayout(controls_col, 3)

        status_group = QGroupBox("")
        status_group.setFlat(True)
        status_grid = QGridLayout()
        status_grid.setSpacing(4)
        status_grid.addWidget(QLabel("Gazebo"), 0, 0)
        status_grid.addWidget(self.led_gz, 0, 1)
        status_grid.addWidget(QLabel("Bridge"), 0, 2)
        status_grid.addWidget(self.led_bridge, 0, 3)
        status_grid.addWidget(QLabel("/clock"), 1, 0)
        status_grid.addWidget(self.led_clock, 1, 1)
        status_grid.addWidget(QLabel("Rosbag"), 1, 2)
        status_grid.addWidget(self.led_bag, 1, 3)
        status_grid.addWidget(QLabel("ros2_control"), 2, 0)
        status_grid.addWidget(self.led_ros2, 2, 1)
        status_grid.addWidget(QLabel("UR5 (sim)"), 2, 2)
        status_grid.addWidget(self.led_ur5, 2, 3)
        self.lbl_moveit_status = QLabel("MoveIt (OFF)")
        status_grid.addWidget(self.lbl_moveit_status, 3, 0)
        status_grid.addWidget(self.led_moveit, 3, 1)
        self.lbl_moveit_bridge_status = QLabel("MoveIt bridge")
        status_grid.addWidget(self.lbl_moveit_bridge_status, 3, 2)
        status_grid.addWidget(self.led_moveit_bridge, 3, 3)
        status_group.setLayout(status_grid)
        controls_status_row.addWidget(status_group, 1)

        sys_group = QGroupBox("")
        sys_group.setFlat(True)
        sys_group.setStyleSheet("")
        sys_layout = QVBoxLayout()
        sys_layout.setContentsMargins(6, 6, 6, 6)
        sys_layout.setSpacing(2)
        sys_layout.addWidget(self.sys_cpu_lbl)
        sys_layout.addWidget(self.sys_ram_lbl)
        sys_layout.addWidget(self.sys_load_lbl)
        sys_layout.addWidget(self.sys_health_lbl)
        sys_group.setLayout(sys_layout)
        controls_status_row.addWidget(sys_group, 0)

        main.addLayout(controls_status_row)

        cam_group = QGroupBox("")
        cam_group.setFlat(True)
        cam_layout = QVBoxLayout()
        cam_layout.setContentsMargins(6, 8, 6, 6)
        cam_layout.setSpacing(2)

        cam_top = QHBoxLayout()
        cam_top.setSpacing(4)
        self.camera_topic_combo = QComboBox()
        self.camera_topic_combo.setEditable(True)
        self.camera_topic_combo.addItem(self.camera_topic)
        self.btn_camera_refresh = QPushButton("↻")
        self.btn_camera_refresh.setToolTip("Detectar tópicos")
        self.btn_camera_refresh.setMaximumWidth(32)
        self.btn_camera_refresh.clicked.connect(self._refresh_camera_topics)
        self.btn_camera_connect = QPushButton("Conectar")
        self.btn_camera_connect.clicked.connect(self._connect_camera)
        cam_top.addWidget(QLabel("Tópico:"))
        cam_top.addWidget(self.camera_topic_combo, 1)
        cam_top.addWidget(self.btn_camera_refresh)
        cam_top.addWidget(self.btn_camera_connect)
        self.btn_calibrate = QPushButton("Calibrar")
        self.btn_calibrate.clicked.connect(self._start_calibration)
        cam_top.addWidget(self.btn_calibrate)
        cam_layout.addLayout(cam_top)

        self.camera_view = CameraView("Esperando imagen...")
        self.camera_view.clicked.connect(self._on_camera_click)
        self.camera_info = QLabel("Sin conexión")
        self.camera_info.setStyleSheet("color:#64748b;")
        cam_layout.addWidget(self.camera_view, 1)
        cam_layout.addWidget(self.camera_info)
        cam_group.setLayout(cam_layout)
        self._last_camera_frame_ts = 0.0
        self._camera_frame_lock = threading.Lock()
        self._camera_pending_frame = None
        self._camera_initializing = False
        self._camera_init_start = 0.0
        self._camera_display_timer = QTimer(self)
        self._camera_display_timer.setInterval(CAMERA_DISPLAY_INTERVAL_MS)
        self._camera_display_timer.timeout.connect(self._refresh_camera_display)
        self._camera_display_timer.start()
        self._camera_health_timer = QTimer(self)
        self._camera_health_timer.setInterval(1200)
        self._camera_health_timer.timeout.connect(self._camera_health_check)
        self._camera_msg_type = "image"
        self._camera_status_connected = False

        self.obj_panel = ObjectListPanel()
        self.obj_panel.selected.connect(self._on_object_clicked)
        self.obj_panel.setFixedWidth(220)

        g_manual = QGroupBox("")
        g_manual.setFlat(True)
        g_manual.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        g_manual.setMaximumHeight(380)
        manual_layout = QVBoxLayout()
        manual_layout.setContentsMargins(6, 6, 6, 6)
        manual_layout.setSpacing(3)

        manual_top = QHBoxLayout()
        manual_top.setSpacing(3)
        self.btn_send_joints = QPushButton("Mover articulaciones")
        self.btn_send_joints.setMinimumWidth(140)
        self.btn_send_joints.clicked.connect(self._send_joints)
        self.joint_time = QDoubleSpinBox()
        self.joint_time.setDecimals(2)
        self.joint_time.setRange(0.5, 8.0)
        self.joint_time.setSingleStep(0.25)
        self.joint_time.setValue(DEFAULT_JOINT_MOVE_SEC)
        self.joint_time.setSuffix(" s")
        self.joint_time.setMaximumWidth(70)
        self.chk_auto_joints = QCheckBox("Auto")
        self.chk_auto_joints.setChecked(True)
        self.btn_gripper = QPushButton("Cerrar gripper")
        self.btn_gripper.setCheckable(True)
        self.btn_gripper.setMinimumHeight(28)
        self.btn_gripper.clicked.connect(self._toggle_gripper_button)
        manual_top.addWidget(self.btn_send_joints)
        manual_top.addWidget(QLabel("t"))
        manual_top.addWidget(self.joint_time)
        manual_top.addWidget(self.chk_auto_joints)
        manual_top.addWidget(self.btn_gripper)
        manual_top.addStretch(1)
        manual_layout.addLayout(manual_top)

        slider_grid = QGridLayout()
        slider_grid.setHorizontalSpacing(3)
        slider_grid.setVerticalSpacing(2)
        slider_grid.setColumnStretch(2, 1)
        self.joint_sliders = []
        self.joint_value_labels = []
        slider_min = int(JOINT_SLIDER_DEG_MIN * JOINT_SLIDER_SCALE)
        slider_max = int(JOINT_SLIDER_DEG_MAX * JOINT_SLIDER_SCALE)
        home_pose = load_home_pose()
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            jlabel = QLabel(f"J{idx + 1}")
            jlabel.setToolTip(joint)
            btn_minus = QPushButton("-")
            btn_plus = QPushButton("+")
            btn_minus.setFixedWidth(20)
            btn_plus.setFixedWidth(20)
            btn_minus.clicked.connect(lambda _=False, i=idx: self._step_joint(i, -1))
            btn_plus.clicked.connect(lambda _=False, i=idx: self._step_joint(i, 1))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(slider_min, slider_max)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setFixedHeight(18)
            value_lbl = QLabel("0.0 deg / 0.00 rad")
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider.valueChanged.connect(lambda v, i=idx: self._on_slider_change(i, v))
            slider.sliderReleased.connect(self._maybe_send_auto)
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_lbl)
            if idx < len(home_pose):
                slider.setValue(int(round(math.degrees(home_pose[idx]) * JOINT_SLIDER_SCALE)))
            else:
                slider.setValue(0)
            slider_grid.addWidget(jlabel, idx, 0)
            slider_grid.addWidget(btn_minus, idx, 1)
            slider_grid.addWidget(slider, idx, 2)
            slider_grid.addWidget(btn_plus, idx, 3)
            slider_grid.addWidget(value_lbl, idx, 4)
        manual_layout.addLayout(slider_grid)

        info_grid = QGridLayout()
        info_grid.setHorizontalSpacing(6)
        info_grid.setVerticalSpacing(3)
        info_font_css = (
            "QGroupBox {"
            "font-size:11px; font-weight:700;"
            "background:#f8fafc;"
            "border:1px solid #cbd5f5;"
            "border-radius:8px;"
            "margin-top:6px;"
            "padding:8px;"
            "}"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; top: -4px; padding:0 4px; }"
            "QLabel { font-size:10px; }"
        )
        info_grid.setColumnStretch(0, 1)
        info_grid.setColumnStretch(1, 1)
        info_grid.setColumnStretch(2, 1)

        self.dof_pos_labels: Dict[str, QLabel] = {}
        self.dof_vel_labels: Dict[str, QLabel] = {}
        self.gripper_labels: Dict[str, QLabel] = {}
        self.gripper_total_lbl: Optional[QLabel] = None
        self.tcp_xyz_lbl: Optional[QLabel] = None
        self.tcp_rpy_lbl: Optional[QLabel] = None
        self.vel_norm_lbl: Optional[QLabel] = None
        self.vel_max_lbl: Optional[QLabel] = None
        self.eff_max_lbl: Optional[QLabel] = None

        dof_group = QGroupBox("DOF UR5 (pos/vel)")
        dof_group.setFlat(True)
        dof_group.setStyleSheet(info_font_css)
        dof_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        dof_layout = QGridLayout()
        dof_layout.setContentsMargins(2, 2, 2, 2)
        dof_layout.setSpacing(1)
        dof_headers = ["Joint", "Posición", "Velocidad"]
        dof_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for col, header in enumerate(dof_headers):
            dof_layout.addWidget(QLabel(f"<b>{header}</b>"), 0, col)
        for row, joint in enumerate(dof_joints, 1):
            name_lbl = QLabel(joint.replace("_", " "))
            pos_lbl = QLabel("--")
            vel_lbl = QLabel("--")
            self.dof_pos_labels[joint] = pos_lbl
            self.dof_vel_labels[joint] = vel_lbl
            dof_layout.addWidget(name_lbl, row, 0)
            dof_layout.addWidget(pos_lbl, row, 1)
            dof_layout.addWidget(vel_lbl, row, 2)
        dof_group.setLayout(dof_layout)
        info_grid.addWidget(dof_group, 0, 0, 2, 1)

        gripper_group = QGroupBox("Pinza RG2 (pos)")
        gripper_group.setFlat(True)
        gripper_group.setStyleSheet(info_font_css)
        gripper_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        gripper_layout = QGridLayout()
        gripper_layout.setContentsMargins(2, 2, 2, 2)
        gripper_layout.setSpacing(1)
        gripper_layout.addWidget(QLabel("<b>Joint</b>"), 0, 0)
        gripper_layout.addWidget(QLabel("<b>Apertura</b>"), 0, 1)
        for row, joint in enumerate(["rg2_finger_joint1", "rg2_finger_joint2"], start=1):
            name_lbl = QLabel(joint)
            pos_lbl = QLabel("--")
            self.gripper_labels[joint] = pos_lbl
            gripper_layout.addWidget(name_lbl, row, 0)
            gripper_layout.addWidget(pos_lbl, row, 1)
        self.gripper_total_lbl = QLabel("--")
        gripper_layout.addWidget(QLabel("Apertura total"), 3, 0)
        gripper_layout.addWidget(self.gripper_total_lbl, 3, 1)
        gripper_group.setLayout(gripper_layout)
        info_grid.addWidget(gripper_group, 0, 1, 2, 1)

        kin_group = QGroupBox("Cinemática (FK)")
        kin_group.setFlat(True)
        kin_group.setStyleSheet(info_font_css)
        kin_layout = QGridLayout()
        kin_layout.setContentsMargins(2, 2, 2, 2)
        kin_layout.setSpacing(1)
        self.tcp_xyz_lbl = QLabel("--")
        self.tcp_rpy_lbl = QLabel("--")
        kin_layout.addWidget(QLabel("<b>TCP xyz [m]</b>"), 0, 0)
        kin_layout.addWidget(self.tcp_xyz_lbl, 0, 1)
        kin_layout.addWidget(QLabel("<b>RPY [deg]</b>"), 1, 0)
        kin_layout.addWidget(self.tcp_rpy_lbl, 1, 1)
        kin_group.setLayout(kin_layout)
        info_grid.addWidget(kin_group, 0, 2)

        dyn_group = QGroupBox("Dinámica (vel/eff)")
        dyn_group.setFlat(True)
        dyn_group.setStyleSheet(info_font_css)
        dyn_layout = QGridLayout()
        dyn_layout.setContentsMargins(2, 2, 2, 2)
        dyn_layout.setSpacing(1)
        self.vel_max_lbl = QLabel("--")
        self.eff_max_lbl = QLabel("--")
        self.vel_norm_lbl = QLabel("--")
        dyn_layout.addWidget(QLabel("<b>||qdot||</b>"), 0, 0)
        dyn_layout.addWidget(self.vel_norm_lbl, 0, 1)
        dyn_layout.addWidget(QLabel("<b>max |qdot|</b>"), 1, 0)
        dyn_layout.addWidget(self.vel_max_lbl, 1, 1)
        dyn_layout.addWidget(QLabel("<b>max |eff|</b>"), 2, 0)
        dyn_layout.addWidget(self.eff_max_lbl, 2, 1)
        dyn_group.setLayout(dyn_layout)
        info_grid.addWidget(dyn_group, 1, 2)

        manual_layout.addLayout(info_grid)

        self.lbl_joint_states = QLabel("Joint states: esperando /joint_states ...")
        self.lbl_joint_states.setStyleSheet("color:#64748b; font-size:10px;")
        manual_layout.addWidget(self.lbl_joint_states)
        manual_layout.addStretch(1)
        g_manual.setLayout(manual_layout)
        self.trace_group = self._build_trace_group()

        g_robot = QGroupBox("Robot / DEMO (scripts existentes)")
        g_robot.setFlat(True)
        g_robot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        robot_layout = QVBoxLayout()
        robot_layout.setContentsMargins(6, 8, 6, 6)
        robot_layout.setSpacing(6)

        self.btn_home = QPushButton("UR5 → HOME")
        self.btn_table = QPushButton("UR5 → Mesa")
        self.btn_basket = QPushButton("UR5 → Cesta")
        for b in (self.btn_home, self.btn_table, self.btn_basket):
            b.setMinimumHeight(32)
        self.btn_home.clicked.connect(self._go_home)
        self.btn_table.clicked.connect(self._go_table)
        self.btn_basket.clicked.connect(self._go_basket)

        self.btn_pick_demo = QPushButton("PICK MESA → CESTA (DEMO)")
        self.btn_pick_demo.setMinimumHeight(32)
        self.btn_pick_demo.clicked.connect(self._run_pick_demo)

        robot_layout.addWidget(self.btn_home)
        robot_layout.addWidget(self.btn_table)
        robot_layout.addWidget(self.btn_basket)
        robot_layout.addWidget(self.btn_pick_demo)
        g_robot.setLayout(robot_layout)

        left_col = QVBoxLayout()
        left_col.setSpacing(6)
        left_col.addWidget(cam_group, 2)
        left_col.addWidget(g_robot, 1)
        left_col.addStretch(1)

        right_col = QVBoxLayout()
        right_col.setSpacing(6)
        right_col.addWidget(g_manual, 2)
        object_trace_layout = QHBoxLayout()
        object_trace_layout.setSpacing(6)
        object_trace_layout.addWidget(self.obj_panel, 1)
        if self.trace_group:
            object_trace_layout.addWidget(self.trace_group, 1)
        right_col.addLayout(object_trace_layout, 1)
        right_col.addStretch(1)

        top_row = QHBoxLayout()
        top_row.setSpacing(8)
        top_row.addLayout(left_col, 1)
        top_row.addLayout(right_col, 1)

        main.addLayout(top_row)

        # Al final del método, tras crear todos los widgets:
        root.setLayout(main)
        self.setCentralWidget(root)
        self._start_trace_timer()
        self._apply_status(False, False, False, False, False, False, False)

    def _debounced_btn_action(self, btn, action, delay_ms=1200):
        if not btn.isEnabled():
            return
        btn.setEnabled(False)
        try:
            action()
        finally:
            QTimer.singleShot(delay_ms, lambda: btn.setEnabled(True))

    def showEvent(self, event):
        super().showEvent(event)
        if not self._timers_started:
            self.status_timer.start(2500)
            self._timers_started = True

    def _set_status(self, text: str, error: bool = False):
        color = "#dc2626" if error else "#0f172a"
        self.status_lbl.setText(text)
        self.status_lbl.setStyleSheet(
            f"background:{color}; color:white; padding:8px 14px; border-radius:12px; font-weight:600;"
        )
        # SIEMPRE loguear errores; info solo si debug está activo
        if error:
            self._emit_log(f"[ERROR] {text}")
        elif self._debug_logs_enabled:
            self._emit_log(f"[INFO] {text}")

    @pyqtSlot(str, bool)
    def _set_status_async(self, text: str, error: bool = False) -> None:
        self._set_status(text, error=error)

    @pyqtSlot(object, str)
    def _set_led_async(self, led_obj: object, state: str) -> None:
        set_led(led_obj, state)

    @pyqtSlot(bool)
    def _on_tf_ready_signal(self, ready: bool) -> None:
        self._tf_ready_state = ready
        if not ready:
            self._trace_ready = False
        self._evaluate_system_state()
        self._refresh_controls()

    @pyqtSlot(bool)
    def _on_calib_ready_signal(self, ready: bool) -> None:
        self._calibration_ready = ready
        self._evaluate_system_state()
        self._refresh_controls()

    @pyqtSlot(bool)
    def _on_controllers_ready_signal(self, ready: bool) -> None:
        self._controllers_ok = ready
        self._evaluate_system_state()
        self._refresh_controls()

    @pyqtSlot(str)
    def _on_error_signal(self, msg: str) -> None:
        self._system_error_reason = msg
        self._set_system_state(SystemState.ERROR, msg)
        self._set_status(msg, error=True)
        self._refresh_controls()

    @pyqtSlot(str, str)
    def _on_moveit_state_signal(self, state: str, reason: str) -> None:
        try:
            state_enum = MoveItState[state]
        except KeyError:
            return
        self._moveit_state = state_enum
        self._moveit_state_reason = reason or state_enum.value
        if state_enum == MoveItState.OFF:
            set_led(self.led_moveit, "off")
        elif state_enum == MoveItState.STARTING:
            set_led(self.led_moveit, "warn")
        elif state_enum == MoveItState.WAITING_MOVEIT_READY:
            set_led(self.led_moveit, "warn")
        elif state_enum == MoveItState.READY:
            set_led(self.led_moveit, "on")
        else:
            set_led(self.led_moveit, "error")
            if reason:
                self._set_status(f"MoveIt error: {reason}", error=True)
        if reason and self._debug_logs_enabled:
            self._emit_log(f"[MOVEIT] {state_enum.value}: {reason}")
        self._update_moveit_status_label()
        self._refresh_controls()

    @pyqtSlot()
    def _on_trace_ready(self) -> None:
        if self._trace_ready:
            return
        self._trace_ready = True
        self._bridge_ready = True
        self._stop_tf_ready_timer()
        self._log("[TRACE] TF ready → TRACE enabled")
        self._refresh_trace_data()

    @pyqtSlot()
    def _on_calibration_check(self) -> None:
        if self._calibration_ready:
            return
        if not self._objects_settled and not ALLOW_UNSETTLED_ON_TIMEOUT:
            self._log_calib_blocked("esperando caída/estabilidad de objetos")
            return
        if not self._pose_info_ok:
            self._log_calib_blocked("pose/info no disponible")
            return
        if not self._tf_ready_state:
            self._log_calib_blocked("TF world->base_link no disponible")
            return
        if self._camera_required and not self._camera_stream_ok:
            self._log_calib_blocked("cámara no publica")
            return
        self._log("[CALIB] Inicializando calibración tras bridge")
        self._load_table_calibration()
        self._refresh_objects_from_gz_async()
        QTimer.singleShot(1500, self._refresh_objects_from_gz_async)
        QTimer.singleShot(4000, self._refresh_objects_from_gz_async)
        self._calibration_ready = True
        self.signal_calib_ready.emit(True)

    def _set_system_state(self, state: SystemState, reason: str) -> None:
        if self._system_state == state and self._system_state_reason == reason:
            return
        self._system_state = state
        self._system_state_reason = reason
        self._emit_log(f"[STATE] {state.value} ({reason})")

    def _evaluate_system_state(self) -> None:
        if self._system_error_reason:
            self._set_system_state(SystemState.ERROR, self._system_error_reason)
            return
        next_state = SystemState.READY_VISION
        next_reason = "Sistema listo (visión)"

        gz_state = self._gazebo_state()
        if gz_state != "GAZEBO_READY":
            next_state = SystemState.WAITING_GAZEBO
            next_reason = f"Gazebo {gz_state}"
        elif not self._controllers_ok:
            next_state = SystemState.WAITING_CONTROLLERS
            next_reason = self._controllers_reason or "controladores no listos"
        elif not self._tf_ready_state:
            next_state = SystemState.READY_BASIC
            next_reason = "TF world->base_link no disponible"
        elif not self._bridge_running or (self._camera_required and not self._camera_stream_ok) or not self._pose_info_ok:
            next_state = SystemState.READY_BASIC
            if not self._bridge_running:
                next_reason = "Bridge no listo"
            elif self._camera_required and not self._camera_stream_ok:
                next_reason = "Cámara no lista"
            else:
                next_reason = "pose/info no disponible"
        elif not self._calibration_ready or not self._objects_settled:
            next_state = SystemState.READY_BASIC
            if not self._objects_settled:
                next_reason = "Objetos no estabilizados"
            else:
                next_reason = "Calibración pendiente"
        elif self._moveit_required and self._moveit_state != MoveItState.READY:
            next_state = SystemState.READY_VISION
            next_reason = self._moveit_state_reason or "MoveIt no listo"
        elif self._moveit_required:
            next_state = SystemState.READY_MOVEIT
            next_reason = "Sistema listo (MoveIt)"

        prev_state = self._system_state
        if prev_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT) and next_state != prev_state:
            if next_state == SystemState.READY_BASIC:
                self._set_system_state(next_state, next_reason)
                return
            if next_state == SystemState.WAITING_CONTROLLERS and not self._controllers_ok and self._controller_drop_grace_active():
                self._emit_log(f"[WARN] drop a {next_state.value} suprimido: controladores en gracia ({next_reason})")
                self._set_system_state(next_state, next_reason)
                return
            self._system_error_reason = f"drop: {next_reason}"
            self._set_system_state(SystemState.ERROR, self._system_error_reason)
        else:
            self._set_system_state(next_state, next_reason)

    def _state_ready_basic(self) -> bool:
        return self._system_state in (SystemState.READY_BASIC, SystemState.READY_VISION, SystemState.READY_MOVEIT)

    def _state_ready_vision(self) -> bool:
        return self._system_state in (SystemState.READY_VISION, SystemState.READY_MOVEIT)

    def _state_ready_moveit(self) -> bool:
        return self._system_state == SystemState.READY_MOVEIT

    def _manual_control_ready(self) -> bool:
        return self._gazebo_state() == "GAZEBO_READY" and self._controllers_ok

    def _moveit_control_ready(self) -> bool:
        objects_ok = self._objects_settled or ALLOW_UNSETTLED_ON_TIMEOUT
        return (
            self._manual_control_ready()
            and self._moveit_ready()
            and (self._camera_stream_ok or not self._camera_required)
            and objects_ok
        )

    def _pick_ui_allowed(self) -> bool:
        return (
            self._manual_control_ready()
            and self._pose_info_ok
            and (self._camera_stream_ok or not self._camera_required)
            and self._tf_ready_state
        )

    @pyqtSlot(object)
    def _update_camera_topics_async(self, topics: object) -> None:
        self._update_camera_topics(list(topics) if topics else [])

    def _emit_log(self, msg: str, *, flush: bool = True):
        """Print a timestamped log line."""
        if not self._should_emit_log(msg):
            return
        print(timestamped_line(msg), flush=flush)

    def _should_emit_log(self, msg: str) -> bool:
        if getattr(self, "_debug_logs_enabled", False):
            return True
        if msg.startswith("[STARTUP]"):
            return True
        if msg.startswith("[BTN]"):
            return True
        if msg.startswith("[ERROR]"):
            return True
        return False

    def _set_motion_lock(self, active: bool):
        """Habilitar/deshabilitar control manual mientras corre un flujo predefinido."""
        self._script_motion_active = active
        QTimer.singleShot(0, self._refresh_controls)

    def _set_btn_state(self, btn: QPushButton, enabled: bool, tooltip: str = "") -> None:
        btn.setEnabled(enabled)
        if enabled:
            if tooltip:
                btn.setToolTip(tooltip)
            return
        if tooltip:
            btn.setToolTip(tooltip)

    def _set_launching_style(self, btn: QPushButton, active: bool) -> None:
        if active:
            btn.setStyleSheet("background:#f59e0b; color:#0f172a; font-weight:600;")
        else:
            btn.setStyleSheet("")

    def _clear_launching_if_timeout(self, label: str, start_ts: float, timeout_sec: float) -> bool:
        if not start_ts:
            return False
        if (time.time() - start_ts) < timeout_sec:
            return False
        self._emit_log(f"[WARN] {label} timeout; re-habilitando botón")
        return True

    def _controller_drop_grace_active(self) -> bool:
        if self._controller_spawn_inflight:
            return True
        if not self._controller_spawn_last_start:
            return False
        return (time.time() - self._controller_spawn_last_start) < CONTROLLER_DROP_GRACE_SEC

    def _require_ready_basic(self, action: str) -> bool:
        if self._state_ready_basic():
            return True
        reason = self._system_state_reason or self._system_state.value
        self._set_status(f"{action} en espera: {reason}", error=False)
        self._emit_log(f"[SAFETY] {action} bloqueado: {reason}")
        return False

    def _require_manual_ready(self, action: str) -> bool:
        if self._manual_control_ready():
            return True
        reason = "Gazebo no listo"
        if not self._controllers_ok:
            reason = self._controllers_reason or "controladores no listos"
        self._set_status(f"{action} en espera: {reason}", error=False)
        self._emit_log(f"[SAFETY] {action} bloqueado: {reason}")
        return False

    def _log(self, msg: str):
        if self._debug_logs_enabled:
            self._emit_log(msg)

    def _block_if_managed(self, action: str) -> bool:
        if not self._managed_mode:
            return False
        self._emit_log(f"[WARN] {action} bloqueado: PANEL_MANAGED=1")
        return True
    
    def _log_error(self, msg: str):
        """SIEMPRE loguear errores, incluso si debug no está activo."""
        self._emit_log(f"[ERROR] {msg}")
    
    def _log_warning(self, msg: str):
        """Loguear warnings cuando debug está activo."""
        if self._debug_logs_enabled:
            self._emit_log(f"[WARN] {msg}")

    def _log_ros_message(self, msg: str):
        """Mostrar siempre los mensajes provenientes del RosWorker, pero solo cuando el bridge esté activo."""
        if not self._bridge_running:
            return
        self._emit_log(msg)

    @pyqtSlot(str, str)
    def _on_system_state_update(self, state: str, reason: str) -> None:
        state = (state or "").strip().upper()
        if not state:
            return
        self._external_state = state
        self._external_state_reason = reason or ""
        self._external_state_last = time.time()

    def _external_state_active(self) -> bool:
        if not self._external_state:
            return False
        return (time.time() - self._external_state_last) < 2.0

    def _apply_external_system_state(self) -> None:
        state = (self._external_state or "").upper()
        reason = self._external_state_reason or ""
        mapping = {
            "BOOT": SystemState.BOOT,
            "BOOTING": SystemState.BOOT,
            "WAITING_GAZEBO": SystemState.WAITING_GAZEBO,
            "WAITING_CONTROLLERS": SystemState.WAITING_CONTROLLERS,
            "WAITING_TF": SystemState.WAITING_CONTROLLERS,
            "WAITING_BRIDGE": SystemState.READY_BASIC,
            "WAITING_CAMERA": SystemState.READY_BASIC,
            "WAITING_SETTLE": SystemState.READY_BASIC,
            "WAITING_MOVEIT": SystemState.READY_VISION,
            "READY_BASIC": SystemState.READY_BASIC,
            "READY_VISION": SystemState.READY_VISION,
            "READY_MOVEIT": SystemState.READY_MOVEIT,
            "READY": SystemState.READY_VISION,
            "ERROR": SystemState.ERROR,
        }
        target = mapping.get(state)
        if target is None:
            return
        if target == SystemState.READY_VISION and self._moveit_required:
            if self._moveit_state != MoveItState.READY:
                reason = self._moveit_state_reason or "MoveIt no listo"
                self._set_system_state(SystemState.READY_VISION, reason)
                return
            target = SystemState.READY_MOVEIT
        if target == SystemState.ERROR:
            self._system_error_reason = reason or self._system_error_reason
        else:
            self._system_error_reason = ""
        self._set_system_state(target, reason or f"Estado externo: {state}")
        if target in (SystemState.READY_BASIC, SystemState.READY_VISION, SystemState.READY_MOVEIT):
            self._ensure_pose_subscription()
            self._start_pose_info_watch()
            self._start_tf_ready_timer()

    def _log_camera_diagnostics(self, reason: str):
        """Emitir detalles adicionales para debugging cuando hay fallos de cámara."""
        if not self._camera_required:
            return
        if not self._debug_logs_enabled:
            return
        node_ready = self.ros_worker.node_ready()
        ctrl_ok = self._ros2_control_available()
        clock_ok, clock_age = self._clock_status()
        bridge_ok = self._bridge_running
        last_age = "n/a"
        if self._last_camera_frame_ts:
            last_age = f"{time.time() - self._last_camera_frame_ts:.1f}s"
        topics = []
        if node_ready:
            try:
                topics = self.ros_worker.list_topic_names()
            except Exception as exc:
                self._log(f"[CAMERA-DIAG] fallo listando topics: {exc}")
        camera_topics = [t for t in topics if t.startswith(CAMERA_TOPIC_PREFIX)]
        diag = (
            f"[CAMERA-DIAG] {reason} node_ready={node_ready} ros2_ctrl={ctrl_ok} "
            f"clock={clock_ok}:{clock_age} bridge={bridge_ok} "
            f"last_frame_age={last_age} camera_topics={len(camera_topics)}/{len(topics)}"
        )
        self._log(diag)
        if camera_topics:
            preview = ", ".join(camera_topics[:4])
            suffix = "..." if len(camera_topics) > 4 else ""
            self._log(f"[CAMERA-DIAG] camera topics sample: {preview}{suffix}")

    def _sync_moveit_from_system_state(self) -> None:
        if not self._moveit_required:
            self._moveit_state = MoveItState.OFF
            self._moveit_state_reason = "manual"
            return
        if not self._external_state_active():
            return
        if self._system_state == SystemState.READY_MOVEIT:
            if self._moveit_state != MoveItState.READY:
                self._moveit_state = MoveItState.READY
                self._moveit_state_reason = "move_group listo (externo)"
            return
        if self._system_state == SystemState.READY_VISION:
            self._moveit_state = MoveItState.WAITING_MOVEIT_READY
            self._moveit_state_reason = self._system_state_reason or "move_group no listo"

    def _clock_status(self) -> Tuple[bool, str]:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "node_off"
        ok, age = self.ros_worker.clock_alive()
        if ok:
            return True, f"age={age:.2f}s"
        # Fallback: detectar publishers en /clock aunque no haya recibido mensaje reciente.
        if self.ros_worker.topic_has_publishers("/clock"):
            return True, "publisher"
        return False, f"age={age:.2f}s"

    def _pose_info_topic(self) -> str:
        world_name = self._gz_world_name or self._detect_world_name() or GZ_WORLD
        return f"/world/{world_name}/pose/info"

    def _pose_info_active(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        return self.ros_worker.topic_has_publishers(self._pose_info_topic())

    def _pose_info_ready(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        if not self._pose_info_active():
            return False
        poses, ts = self.ros_worker.pose_snapshot()
        if not poses:
            return False
        if ts:
            age = time.time() - ts
            if age > POSE_INFO_MAX_AGE_SEC:
                return False
        return True

    def _gazebo_state(self) -> str:
        """GAZEBO_OFF / GAZEBO_STARTING / GAZEBO_READY (debounced)."""
        gz_proc_ok, _gz_reason = gz_sim_status()
        clock_ok, _clock_reason = self._clock_status()
        now = time.monotonic()
        if clock_ok:
            self._last_clock_ok_ts = now
        clock_grace = (now - self._last_clock_ok_ts) < 2.5
        if not gz_proc_ok:
            candidate = "GAZEBO_OFF"
        elif clock_ok or (self._gz_state == "GAZEBO_READY" and clock_grace):
            candidate = "GAZEBO_READY"
        else:
            candidate = "GAZEBO_STARTING"
        if candidate != self._gz_state:
            if candidate != self._gz_state_pending:
                self._gz_state_pending = candidate
                self._gz_state_change_ts = now
            elif (now - self._gz_state_change_ts) >= 1.0:
                self._gz_state = candidate
                self._gz_state_pending = ""
                reason = "process_alive"
                if self._gz_state == "GAZEBO_READY":
                    reason = "clock_ok" if clock_ok else "clock_grace"
                self._emit_log(
                    f"[STATE] Gazebo={self._gz_state} reason={reason} "
                    f"process={str(gz_proc_ok).lower()} clock={str(clock_ok).lower()}"
                )
        else:
            self._gz_state_pending = ""
        return self._gz_state

    def _ros2_control_available(self) -> bool:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        return bool(self._controller_manager_path())

    def _controller_manager_path(self) -> str:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return ""
        try:
            services = self.ros_worker.service_names_and_types()
        except Exception:
            services = []
        for name, _types in services:
            if name.endswith("/controller_manager/list_controllers"):
                return name.rsplit("/list_controllers", 1)[0]
        return "/controller_manager"

    def _select_traj_topic(self) -> str:
        topics = set(self.ros_worker.list_topic_names()) if self.ros_worker.node_ready() else set()
        if "/joint_trajectory_controller/joint_trajectory" in topics:
            return "/joint_trajectory_controller/joint_trajectory"
        candidates = sorted(t for t in topics if t.endswith("/joint_trajectory_controller/joint_trajectory"))
        if candidates:
            return candidates[0]
        return ""

    def _ensure_pose_subscription(self) -> None:
        if not self._bridge_running or self._closing:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return
        world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        topic = self._discover_pose_info_topic(world_name)
        self.ros_worker.subscribe_pose_info(topic)

    def _discover_pose_info_topic(self, world_name: str) -> str:
        """Return pose/info topic, preferring *world_name* when available."""
        expected = f"/world/{world_name}/pose/info"
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return expected
        try:
            topics = self.ros_worker.list_topic_names()
        except Exception:
            return expected
        candidates = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
        if not candidates:
            return expected
        if expected in candidates:
            return expected
        return sorted(candidates)[0]

    def _start_pose_info_watch(self) -> None:
        if self._pose_info_timer is None:
            self._pose_info_timer = QTimer(self)
            self._pose_info_timer.setInterval(int(POSE_INFO_POLL_SEC * 1000))
            self._pose_info_timer.timeout.connect(self._update_pose_info_status)
        if not self._pose_info_timer.isActive():
            self._pose_info_timer.start()
        self._update_pose_info_status()

    def _update_pose_info_status(self) -> None:
        if self._closing or not self._bridge_running:
            return
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return
        count, age, entities, topic = self.ros_worker.pose_info_details()
        self._pose_info_msg_count = count
        self._pose_info_last_age = age
        ready = count > 0 and age < POSE_INFO_MAX_AGE_SEC
        now = time.monotonic()
        if (now - self._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
            self._emit_log(
                f"[PHYSICS][POSE_INFO] ready={str(ready).lower()} count={count} age={age:.2f}s "
                f"entities={entities} topic={topic or 'n/a'}"
            )
            self._pose_info_last_log = now
        if ready and not self._pose_info_ok:
            self._pose_info_ok = True
            self._pose_info_diag_logged = False
            if self._gz_running:
                self.signal_start_objects_settle_watch.emit()
                QTimer.singleShot(0, self._schedule_physics_runtime_check)
        elif not ready:
            self._pose_info_ok = False
            if (now - self._pose_info_last_log) >= POSE_INFO_LOG_PERIOD:
                self._emit_log("[PHYSICS][SETTLE] waiting pose/info data...")
                self._pose_info_last_log = now
            if (now - self._pose_info_resub_ts) >= POSE_INFO_LOG_PERIOD:
                self._pose_info_resub_ts = now
                self._ensure_pose_subscription()
            if not self._pose_info_diag_logged and count == 0:
                self._pose_info_diag_logged = True
                try:
                    topics = self.ros_worker.list_topic_names()
                except Exception:
                    topics = []
                pose_topics = [t for t in topics if t.startswith("/world/") and t.endswith("/pose/info")]
                sample_topics = ", ".join(pose_topics[:5]) if pose_topics else "-"
                self._emit_log(f"[PHYSICS][POSE_INFO] available_pose_topics={sample_topics}")
                helper = get_tf_helper()
                frames = helper.list_frames() if helper else set()
                frames_sample = ", ".join(sorted(frames)[:10]) if frames else "-"
                self._emit_log(f"[TF] frames_sample={frames_sample}")

    def _log_button(self, label: str):
        self._emit_log(f"[BTN] {label}")
    
    def _cleanup_stray_processes(self):
        """Limpiar procesos fantasma de Gazebo, bridge y rosbag al startup."""
        if os.environ.get("PANEL_SKIP_CLEANUP", "0") == "1":
            self._emit_log("[STARTUP] Limpieza omitida (PANEL_SKIP_CLEANUP=1)")
            return
        # 1. Limpiar archivos de memoria compartida de FastDDS/FastRTPS
        try:
            self._emit_log("[STARTUP] Limpiando /dev/shm (FastDDS)")
            subprocess.run(
                ["sh", "-c", "rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null || true"],
                timeout=2,
            )
        except Exception:
            pass

        # 2. Limpiar procesos residuales del stack si procede.
        if os.environ.get("PANEL_KILL_STALE", "1") == "0":
            self._emit_log("[STARTUP] Limpieza de procesos deshabilitada (PANEL_KILL_STALE=0)")
            return
        if not psutil:
            self._emit_log("[STARTUP] psutil no disponible; no se limpian procesos residuales")
            return
        stale = self._list_stale_processes()
        if not stale:
            self._emit_log("[STARTUP] No hay procesos residuales detectados")
            return
        self._emit_log(f"[STARTUP] Procesos residuales detectados: {len(stale)}. Terminando...")
        procs = []
        for pid, cmd, _status in stale:
            try:
                proc = psutil.Process(pid)
                proc.terminate()
                procs.append(proc)
            except Exception:
                continue
        try:
            _, alive = psutil.wait_procs(procs, timeout=2.0)
            for proc in alive:
                try:
                    proc.kill()
                except Exception:
                    continue
            if alive:
                psutil.wait_procs(alive, timeout=1.0)
        except Exception:
            pass

    def _clean_cache_dirs(self):
        """Limpiar cachés de Python (__pycache__ y .pyc) en el workspace."""
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        ws_parent = os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
        
        removed_count = 0
        try:
            for root, dirs, _ in os.walk(ws_parent):
                if "__pycache__" in dirs:
                    cache_path = os.path.join(root, "__pycache__")
                    try:
                        shutil.rmtree(cache_path)
                        removed_count += 1
                    except Exception as e:
                        self._log_warning(f"No se pudo borrar {cache_path}: {e}")
            # También borrar archivos .pyc
            for root, _, files in os.walk(ws_parent):
                for f in files:
                    if f.endswith(".pyc"):
                        try:
                            os.remove(os.path.join(root, f))
                            removed_count += 1
                        except Exception as e:
                            self._log_warning(f"No se pudo borrar {f}: {e}")
            self._log(f"[STARTUP] ✅ Cache limpiado ({removed_count} items)")
        except Exception as e:
            self._log_error(f"Error limpiando cache: {e}")
    
    def _close_terminal(self):
        """Cerrar la aplicación del panel."""
        if self._closing:
            return
        self._closing = True
        try:
            self.btn_close_terminal.setEnabled(False)
        except Exception:
            pass
        self._log_button("Cerrar Terminal")
        self._log("[PANEL] Cerrando panel...")
        # Trigger the standard Qt close flow so closeEvent() runs cleanup.
        QTimer.singleShot(0, self.close)
    def _refresh_camera_topics(self):
        self._log_button("Refresh topics")
        self._set_status("Detectando tópicos de imagen…")

        def worker():
            if not self._ros_worker_started:
                self._ensure_ros_worker_started()
            if not self.ros_worker.node_ready():
                self.signal_status.emit("Nodo ROS no listo para tópicos", True)
                return
            topics = self.ros_worker.topic_names_and_types()
            candidates = [name for name, _types in topics if _is_camera_topic(name)]
            if candidates:
                self._log(f"[CAMERA] Candidate topics: {', '.join(candidates)}")
                self.signal_update_camera_topics.emit(candidates)
                self.signal_status.emit(f"Detectados {len(candidates)} tópicos de cámara", False)
            else:
                if self._camera_stream_ok or self._camera_frame_count > 0 or self._camera_subscribed:
                    self._log("[CAMERA] Tópicos aún no visibles en discovery (reintentando)")
                    self.signal_status.emit("Discovery cámara pendiente (reintentando)", False)
                    self.signal_schedule_camera_health_check.emit(2000)
                else:
                    self._log("[CAMERA] No se encontraron tópicos compatibles")
                    self.signal_status.emit("No se detectaron tópicos de cámara", False)

        threading.Thread(target=worker, daemon=True).start()

    def _controllers_ready(self) -> Tuple[bool, str]:
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False, "nodo ROS no listo"
        cm_path = self._controller_manager_path()
        if not cm_path:
            return False, "controller_manager no disponible"
        list_srv = f"{cm_path}/list_controllers"
        if not self.ros_worker.has_service(list_srv):
            return False, "list_controllers no disponible"
        resp, err = self._list_controllers(list_srv)
        if resp is None:
            return False, err or "no response"
        required = ["joint_state_broadcaster", "joint_trajectory_controller"]
        if gripper_controller_defined():
            required.append("gripper_controller")
        state_map = {c.name: c.state for c in resp.controller}
        missing = [name for name in required if name not in state_map]
        if missing:
            return False, "missing_controllers=[" + ", ".join(missing) + "]"
        inactive = []
        loaded = []
        unknown = []
        for name in required:
            state = state_map.get(name, "")
            kind = self._controller_state_kind(state)
            if kind == "ACTIVE":
                continue
            if kind == "INACTIVE":
                inactive.append(name)
            elif kind == "LOADED":
                loaded.append(name)
            else:
                unknown.append(name)
        if inactive or loaded or unknown:
            detail = []
            if inactive:
                detail.append(f"inactive: {', '.join(inactive)}")
            if loaded:
                detail.append(f"loaded: {', '.join(loaded)}")
            if unknown:
                detail.append(f"unknown: {', '.join(unknown)}")
            return False, "controllers not active (" + " | ".join(detail) + ")"
        return True, "controllers activos"

    def _controller_state_kind(self, state: str) -> str:
        state_lc = (state or "").strip().lower()
        if state_lc == "active":
            return "ACTIVE"
        if state_lc == "inactive":
            return "INACTIVE"
        if state_lc in ("unconfigured", "finalized"):
            return "LOADED"
        return "UNKNOWN"

    def _list_controllers(self, service_name: str) -> Tuple[Optional["ListControllers.Response"], Optional[str]]:
        if ListControllers is None:
            return None, "ListControllers no disponible"
        if self._moveit_node is None:
            return None, "Nodo ROS no listo"
        if not service_name:
            return None, "service vacío"
        if self._controller_client is None or self._controller_client_name != service_name:
            try:
                self._controller_client = self._moveit_node.create_client(ListControllers, service_name)
                self._controller_client_name = service_name
            except Exception as exc:
                self._controller_client = None
                self._controller_client_name = ""
                return None, f"client error: {exc}"
        client = self._controller_client
        if client is None:
            return None, "client no disponible"
        if not client.wait_for_service(timeout_sec=0.6):
            return None, "service timeout"
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return None, "service sin respuesta"
        return future.result(), None

    def _wait_for_controllers_ready(self, timeout_sec: float) -> Tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        last_reason = "controllers no listos"
        while time.monotonic() < deadline:
            ok, reason = self._controllers_ready()
            last_reason = reason
            if ok:
                return True, reason
            time.sleep(0.15)
        return False, last_reason

    @pyqtSlot(int)
    def _schedule_camera_health_check(self, delay_ms: int = 1800) -> None:
        if self._camera_health_retry_scheduled or not self._bridge_running:
            return
        self._camera_health_retry_scheduled = True
        def _run():
            self._camera_health_retry_scheduled = False
            self._check_camera_topic_health()
        QTimer.singleShot(delay_ms, _run)

    def _check_camera_topic_health(self):
        if not self._camera_required:
            return
        if self._camera_topic_check_inflight or not self._bridge_running:
            return
        topic = self.camera_topic_combo.currentText().strip() or self.camera_topic
        if not topic:
            return
        self._camera_topic_check_inflight = True

        def worker():
            try:
                now = time.time()
                last_age = now - self._last_camera_frame_ts if self._last_camera_frame_ts else float("inf")
                ready = self._camera_frame_count >= 1 and last_age < CAMERA_READY_MAX_AGE_SEC
                self._camera_stream_ok = bool(ready)
                self._camera_topic_hz = 0.0
                self._emit_log(
                    f"[BRIDGE] camera_topic={topic} frames={self._camera_frame_count} "
                    f"ready={self._camera_stream_ok} last_age={last_age:.2f}s"
                )
                if self._camera_stream_ok:
                    self._emit_log(f"[CAMERA] ready=True age={last_age:.2f}s")
                if not self._camera_stream_ok:
                    in_init_grace = False
                    if self._camera_initializing and self._camera_init_start:
                        in_init_grace = (now - self._camera_init_start) < CAMERA_INIT_GRACE_SEC
                    elif self._bridge_start_ts:
                        in_init_grace = (now - self._bridge_start_ts) < CAMERA_INIT_GRACE_SEC
                    if in_init_grace or (self._camera_frame_count == 0 and last_age < 2.0):
                        self.signal_status.emit("Cámara esperando frames…", False)
                    else:
                        self.signal_status.emit("Cámara no lista; esperando", False)
                    self.signal_schedule_camera_health_check.emit(2000)
                elif self._objects_settled and not self._calibration_ready:
                    self.signal_calibration_check.emit()
            finally:
                self._camera_topic_check_inflight = False
                self.signal_refresh_controls.emit()

        threading.Thread(target=worker, daemon=True).start()
    
    def _update_camera_topics(self, topics):
        if not topics:
            return
        current = self.camera_topic_combo.currentText()
        self.camera_topic_combo.clear()
        for topic in topics:
            self.camera_topic_combo.addItem(topic)
        # Restaurar selección previa si existe
        idx = self.camera_topic_combo.findText(current)
        if idx >= 0:
            self.camera_topic_combo.setCurrentIndex(idx)
    
    @pyqtSlot()
    def _connect_camera(self):
        self._log_button("Conectar cámara")
        if not self._bridge_running:
            self._log_warning("Cámara en espera: bridge no activo")
            self._set_status("Cámara en espera: bridge no activo", error=False)
            return
        topic = self.camera_topic_combo.currentText().strip()
        self._log(f"[CAMERA] Intentando conectar a: {topic}")
        if not topic:
            self._log_error("Tópico de cámara vacío")
            self._set_status("Tópico de cámara vacío", error=True)
            return
        if self._camera_subscribed:
            if topic == self.camera_topic:
                self._log(f"[CAMERA] Ya conectado a: {topic}")
                return
            self._log(f"[CAMERA] Desuscribiendo de: {self.camera_topic}")
            self._unsubscribe_camera()

        self.camera_topic = topic
        self.camera_info.setText("Conectando…")
        self.camera_view.setText("Conectando…")
        self._camera_status_connected = False
        self._subscribe_camera(topic)

    def _subscribe_camera(self, topic: str) -> bool:
        msg_type = self._resolve_camera_msg_type(topic)
        self._log(f"[CAMERA] Suscribiendo a {topic} (tipo={msg_type})")
        self._clear_camera_frame(reset_info=False)
        self._camera_initializing = True
        self._camera_init_start = time.time()
        self._camera_frame_count = 0
        self._camera_stream_ok = False
        try:
            subscribed = self.ros_worker.subscribe_image(topic, msg_type=msg_type)
        except Exception as exc:
            subscribed = False
            self._log_error(f"Error suscribiendo cámara: {exc}")
        if not subscribed:
            self._camera_subscribed = False
            self._log_camera_diagnostics("suscripción a cámara fallida")
            self._set_status("Cámara: nodo ROS no listo (reintentando)", error=False)
            return False
        self._camera_msg_type = msg_type
        self._camera_subscribed = True
        self._set_status(f"Suscrito a {topic}", error=False)
        self._log(f"[CAMERA] Suscripción OK a {topic}")
        self._start_camera_health_check(1200)
        return True

    @pyqtSlot(int)
    def _start_camera_health_check(self, delay_ms: int = 1200) -> None:
        self._camera_health_timer.setInterval(delay_ms)
        if not self._camera_health_timer.isActive():
            self._camera_health_timer.start()

    def _unsubscribe_camera(self) -> None:
        if not self._camera_subscribed or not self.camera_topic:
            return
        try:
            self.ros_worker.unsubscribe_image(self.camera_topic)
        except Exception as exc:
            self._log_warning(f"Error desuscribiendo: {exc}")
        self._camera_subscribed = False
        self._clear_camera_frame()

    def _clear_camera_frame(self, *, reset_info: bool = True) -> None:
        with self._camera_frame_lock:
            self._camera_pending_frame = None
        self.camera_view.setPixmap(QPixmap())
        if reset_info:
            self.camera_view.setText("Sin conexión")
            self.camera_info.setText("Sin conexión")
        self._camera_status_connected = False
        self._camera_msg_type = "image"
        self._last_camera_frame_ts = 0.0
        self._camera_frame_count = 0
        self._camera_stream_ok = False

    def _resolve_camera_msg_type(self, topic: str) -> str:
        normalized = topic.lower()
        for name, types in self.ros_worker.topic_names_and_types():
            if name != topic:
                continue
            for candidate in types:
                lower = candidate.lower()
                if "compressedimage" in lower:
                    return "compressed"
                if "image" in lower:
                    return "image"
        if "compressed" in normalized:
            return "compressed"
        return "image"
    
    def _auto_connect_camera(self):
        """Auto-conectar cámara al iniciar el panel."""
        if not self._bridge_running:
            return
        if self._camera_subscribed:
            return
        if not self.ros_worker.node_ready():
            self._log_camera_diagnostics("auto-connect esperando nodo ROS")
            self._log("[CAMERA] Nodo ROS aún no listo, reintentando conexión en 1s")
            QTimer.singleShot(1000, self._auto_connect_camera)
            return
        if self.camera_topic_combo.count() == 0:
            self._refresh_camera_topics()
            QTimer.singleShot(1500, self._auto_connect_camera)
            return
        self._log("[CAMERA] Auto-conectando cámara...")
        self.signal_connect_camera.emit()
        if not self._camera_subscribed:
            QTimer.singleShot(1500, self._auto_connect_camera)

    def _ensure_ros_worker_started(self):
        """Lazy start the RosWorker when the bridge flow begins."""
        if self._ros_worker_started:
            return
        self.ros_worker.start()
        self._ros_worker_started = True
        self._emit_log("[STARTUP] RosWorker iniciado")

    def _auto_subscribe_joints(self):
        if not ROS_AVAILABLE:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if not self.ros_worker.node_ready():
            return
        if self._joint_subscribed:
            return
        try:
            topic, found = self._discover_joint_states_topic(self.joint_topic)
            self.joint_topic = topic
            if topic and topic != self._joint_current_topic:
                self.ros_worker.subscribe_joint_states(topic)
                self._joint_current_topic = topic
            if found:
                self.lbl_joint_states.setText(f"Joint states: suscrito {topic}")
            if self._last_joint_stamp:
                self._joint_active = True
            if self._joint_active and hasattr(self, "joint_timer") and self.joint_timer.isActive():
                self._joint_subscribed = True
                self.joint_timer.stop()
        except Exception as exc:
            self._log_warning(f"No se pudo suscribir a joint_states: {exc}")

    def _discover_joint_states_topic(self, preferred: str) -> Tuple[str, bool]:
        topic = (preferred or "").strip() or "/joint_states"
        if not self.ros_worker.node_ready():
            return topic, False
        try:
            topics = self.ros_worker.list_topic_names()
        except Exception:
            return topic, False
        if topic in topics and self.ros_worker.topic_has_publishers(topic):
            return topic, True
        # fallback: cualquier tópico que termine en /joint_states
        for candidate in sorted(t for t in topics if t.endswith("/joint_states")):
            if self.ros_worker.topic_has_publishers(candidate):
                return candidate, True
        return topic, False

    @pyqtSlot()
    def _on_bridge_ready(self):
        self._ensure_pose_subscription()
        self._start_pose_info_watch()
        self._start_tf_ready_timer()
        self._auto_subscribe_joints()
        self._invalidate_settle("bridge activado", restart=True)
        QTimer.singleShot(300, self._refresh_camera_topics)
        QTimer.singleShot(600, self._auto_connect_camera)
        self._check_camera_topic_health()
        self.signal_calibration_check.emit()
        try:
            if self._gz_running:
                self._refresh_objects_from_gz_async()
                QTimer.singleShot(1500, self._refresh_objects_from_gz_async)
            QTimer.singleShot(1500, lambda: self._apply_home_joint2_offset(retries=3))
            self._detach_attempted = False
            self._detach_inflight = False
            self._detach_auto_disabled = False
            self._detach_backoff_until = 0.0
            if self._gz_running and not self._objects_release_done:
                if self._drop_detach_supported():
                    self._log("[PHYSICS] Desbloqueando objetos tras bridge.")
                    QTimer.singleShot(1500, self._release_objects)
                else:
                    self._objects_release_done = True
                    if not self._detach_feature_logged:
                        self._emit_log("[PHYSICS][DETACH] disabled: no attach/detach channel available")
                        self._detach_feature_logged = True
            if self._gz_running:
                QTimer.singleShot(900, self._run_fall_test_async)
        except Exception as exc:
            self._log_error(f"[BRIDGE] on_ready error: {exc}")
    
    def _on_image(self, topic: str, qimg, w: int, h: int, fps: float):
        if topic != self.camera_topic:
            return
        now = time.time()
        with self._camera_frame_lock:
            self._camera_pending_frame = (topic, qimg, w, h, fps, now)
        self._last_camera_frame_ts = now
        self._camera_frame_count += 1
        if self._camera_frame_count >= 1:
            self._camera_stream_ok = True
        self._reset_camera_retry_backoff()
        if self._camera_initializing:
            self._camera_initializing = False
            self._camera_init_start = 0.0

    def _reset_camera_retry_backoff(self):
        """No-op placeholder; watchdog removed."""
        return

    def _refresh_camera_display(self):
        frame = None
        with self._camera_frame_lock:
            frame = self._camera_pending_frame
            self._camera_pending_frame = None
        if not frame:
            return
        topic, qimg, w, h, fps, ts = frame
        display = qimg
        if w > 0 and h > 0:
            if self._calibrating:
                display = self._draw_calib_overlay(display, w, h)
            elif self._selected_px:
                display = self._draw_selection_overlay(display, w, h)
        self.camera_view.set_frame(display, w, h)
        self.camera_info.setText(f"Conectado · {w}x{h} · fps {fps:.1f}")
        if not self._camera_status_connected:
            self._set_status(f"Cámara: {topic} conectada", error=False)
            self._camera_status_connected = True
        self._last_camera_frame_ts = ts

    def _check_camera_stream(self):
        return
    def _on_joint_state(self, payload: Dict[str, object]):
        if not payload:
            return
        topic = payload.get("topic") or ""
        if topic:
            self._joint_current_topic = topic

        names = payload.get("name") or []
        pos_list = payload.get("position") or []
        vel_list = payload.get("velocity") or []
        eff_list = payload.get("effort") or []
        source = payload.get("source") or "ros"
        stamp = float(payload.get("stamp") or 0.0)
        if stamp:
            self._last_joint_stamp = stamp
        now = stamp or time.time()
        self._joint_active = True

        norm_names = [_normalize_joint_name(n) for n in names]
        pos_map: Dict[str, float] = {}
        vel_map: Dict[str, float] = {}
        eff_map: Dict[str, float] = {}
        for name, pos in zip(norm_names, pos_list):
            try:
                pos_map[name] = float(pos)
            except Exception:
                continue
        if isinstance(vel_list, list) and len(vel_list) == len(names):
            for name, vel in zip(norm_names, vel_list):
                try:
                    vel_map[name] = float(vel)
                except Exception:
                    continue
        if isinstance(eff_list, list) and len(eff_list) == len(names):
            for name, eff in zip(norm_names, eff_list):
                try:
                    eff_map[name] = float(eff)
                except Exception:
                    continue

        missing_vel = not vel_map or any(v is None for v in vel_map.values())
        if pos_map and missing_vel and self._last_joint_time and now > self._last_joint_time:
            dt = max(1e-6, now - self._last_joint_time)
            for name, pos in pos_map.items():
                prev = self._last_joint_positions.get(name)
                if prev is not None and vel_map.get(name) is None:
                    vel_map[name] = (pos - prev) / dt
        if pos_map:
            self._last_joint_positions.update(pos_map)
            self._last_joint_time = now
            if not self._joint_names_warned:
                if not any(j in pos_map for j in UR5_JOINT_NAMES):
                    sample = ", ".join(sorted(list(pos_map.keys()))[:8])
                    self._log_warning(
                        f"[JOINTS] Joint names no coinciden con UR5_JOINT_NAMES. sample={sample}"
                    )
                    self._joint_names_warned = True

        if pos_map and not any(slider.isSliderDown() for slider in self.joint_sliders):
            # Protección extra: no actualizar sliders si el usuario acaba de interactuar manualmente
            if time.time() > self._slider_update_blocked_until:
                self._updating_sliders_from_joint_state = True
                try:
                    for idx, joint in enumerate(UR5_JOINT_NAMES):
                        if joint not in pos_map:
                            continue
                        slider = self.joint_sliders[idx]
                        deg = math.degrees(pos_map[joint])
                        value = int(round(deg * JOINT_SLIDER_SCALE))
                        value = max(slider.minimum(), min(slider.maximum(), value))
                        if slider.value() != value:
                            slider.setValue(value)
                finally:
                    self._updating_sliders_from_joint_state = False

        for joint, pos_lbl in self.dof_pos_labels.items():
            pos = pos_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                pos_lbl.setText(f"{math.degrees(pos):.2f} deg / {pos:.3f} rad")
        for joint, vel_lbl in self.dof_vel_labels.items():
            vel = vel_map.get(joint)
            if vel is None:
                vel_lbl.setText("--")
            else:
                vel_lbl.setText(f"{vel:.3f} rad/s")

        grip_positions = []
        for joint, pos_lbl in self.gripper_labels.items():
            pos = pos_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                pos_mm = pos * 1000.0
                pos_lbl.setText(f"{pos_mm:.1f} mm / {pos:.4f} m")
                grip_positions.append(pos)
        if self.gripper_total_lbl is not None:
            if len(grip_positions) == len(self.gripper_labels):
                opening = sum(abs(v) for v in grip_positions) * 1000.0
                self.gripper_total_lbl.setText(f"{opening:.1f} mm")
            else:
                self.gripper_total_lbl.setText("--")

        q = []
        for joint in UR5_JOINT_NAMES:
            if joint not in pos_map:
                q = []
                break
            q.append(pos_map[joint])
        if len(q) == 6:
            pos, rot = fk_ur5(q)
            roll, pitch, yaw = _rot_to_rpy(rot)
            wx, wy, wz = base_to_world(float(pos[0]), float(pos[1]), float(pos[2]))
            self._last_tcp_world = (wx, wy, wz)
            if self.tcp_xyz_lbl is not None:
                self.tcp_xyz_lbl.setText(f"{wx:.3f}, {wy:.3f}, {wz:.3f}")
            if self.tcp_rpy_lbl is not None:
                r_deg = math.degrees(roll)
                p_deg = math.degrees(pitch)
                y_deg = math.degrees(yaw)
                self._last_tcp_rpy_deg = (r_deg, p_deg, y_deg)
                self.tcp_rpy_lbl.setText(f"{r_deg:.1f}, {p_deg:.1f}, {y_deg:.1f}")
        else:
            if self.tcp_xyz_lbl is not None:
                self.tcp_xyz_lbl.setText("--")
            if self.tcp_rpy_lbl is not None:
                self.tcp_rpy_lbl.setText("--")

        vel_values = [vel_map[j] for j in UR5_JOINT_NAMES if j in vel_map]
        if vel_values:
            norm = math.sqrt(sum(v * v for v in vel_values))
            vmax = max(abs(v) for v in vel_values)
            if self.vel_norm_lbl is not None:
                self.vel_norm_lbl.setText(f"{norm:.3f} rad/s")
            if self.vel_max_lbl is not None:
                self.vel_max_lbl.setText(f"{vmax:.3f} rad/s")
        else:
            if self.vel_norm_lbl is not None:
                self.vel_norm_lbl.setText("--")
            if self.vel_max_lbl is not None:
                self.vel_max_lbl.setText("--")

        eff_values = [eff_map[j] for j in UR5_JOINT_NAMES if j in eff_map]
        if eff_values:
            emax = max(abs(v) for v in eff_values)
            if self.eff_max_lbl is not None:
                self.eff_max_lbl.setText(f"{emax:.3f}")
        else:
            if self.eff_max_lbl is not None:
                self.eff_max_lbl.setText("--")

        if self.lbl_joint_states is not None:
            if self._last_joint_stamp:
                self.lbl_joint_states.setText(f"Joint states: ok ({len(names)} joints, {source})")
            else:
                self.lbl_joint_states.setText("Joint states: esperando /joint_states ...")

        if self._debug_joints_to_stdout and pos_map:
            joint_parts = []
            for joint in UR5_JOINT_NAMES:
                if joint in pos_map:
                    joint_parts.append(f"{joint}={pos_map[joint]:.3f}rad")
            for joint in self.gripper_labels:
                if joint in pos_map:
                    joint_parts.append(f"{joint}={pos_map[joint]:.4f}m")
            pose_txt = "tcp xyz=-- rpy=--"
            if len(q) == 6:
                pos, rot = fk_ur5(q)
                roll, pitch, yaw = _rot_to_rpy(rot)
                pose_txt = (
                    f"tcp xyz=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                    f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})"
                )
            msg = "[DEBUG] JOINTS " + " ".join(joint_parts) + f" | {pose_txt}"
            self._emit_log(msg)

    def _fill_worlds(self):
        self.world_combo.clear()
        for p in DEFAULT_WORLD_CANDIDATES:
            if os.path.isfile(p):
                self.world_combo.addItem(p)
        if os.path.isdir(WORLDS_DIR):
            for fn in sorted(os.listdir(WORLDS_DIR)):
                if fn.endswith((".sdf", ".world")):
                    full = os.path.join(WORLDS_DIR, fn)
                    if full not in DEFAULT_WORLD_CANDIDATES:
                        self.world_combo.addItem(full)
        if self.world_combo.count() == 0:
            self.world_combo.addItem(os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"))
        self.world_combo.setCurrentIndex(0)

    def _fill_bridge_presets(self):
        self.bridge_presets.clear()
        seen = set()
        if os.path.isfile(BRIDGE_BASE_YAML):
            self.bridge_presets.addItem(BRIDGE_BASE_YAML)
            seen.add(BRIDGE_BASE_YAML)
        if os.path.isdir(SCRIPTS_DIR):
            for fn in sorted(os.listdir(SCRIPTS_DIR)):
                if not fn.endswith(('.yaml', '.yml')):
                    continue
                full = os.path.join(SCRIPTS_DIR, fn)
                if full in seen:
                    continue
                self.bridge_presets.addItem(full)
                seen.add(full)
        if self.bridge_presets.count() == 0:
            self.bridge_presets.addItem(BRIDGE_BASE_YAML)
        self.bridge_presets.setCurrentIndex(0)

    def _apply_bridge_preset(self, text: str):
        if text:
            self.bridge_edit.setText(text)

    def _choose_world(self):
        self._log_button("Browse mundo")
        path, _ = QFileDialog.getOpenFileName(self, "Selecciona mundo", WORLDS_DIR, "SDF/WORLD (*.sdf *.world)")
        if path:
            self.world_combo.setCurrentText(path)

    def _choose_yaml(self):
        self._log_button("Browse bridge YAML")
        path, _ = QFileDialog.getOpenFileName(self, "Selecciona YAML del bridge", os.path.dirname(BRIDGE_BASE_YAML), "YAML (*.yaml *.yml)")
        if path:
            self.bridge_edit.setText(path)

    def _run_script(self, script_name: str, label: str):
        self._log_button(label)
        path = os.path.join(self.ws_dir, "scripts", script_name)
        if not os.path.isfile(path):
            self._set_status(f"No existe {script_name}", error=True)
            return

        def worker():
            self._ui_set_status(f"Ejecutando {label}…")
            try:
                res = subprocess.run(["bash", path], capture_output=True, text=True, timeout=180)
                if res.returncode == 0:
                    self._ui_set_status(f"OK {label}")
                else:
                    self._ui_set_status(f"Fallo {label} (rc={res.returncode})", error=True)
            except subprocess.TimeoutExpired:
                self._ui_set_status(f"Timeout {label}", error=True)
            except Exception as exc:
                self._ui_set_status(f"Error {label}: {exc}", error=True)

        threading.Thread(target=worker, daemon=True).start()

    def _toggle_debug(self, env_var: str):
        current = os.environ.get(env_var, "0")
        new_val = "0" if current == "1" else "1"
        os.environ[env_var] = new_val
        enabled = new_val == "1"
        if env_var == "DEBUG_LOGS_TO_STDOUT":
            self._debug_logs_enabled = enabled
            self.btn_debug_logs.setChecked(enabled)
            self._apply_debug_button_style(self.btn_debug_logs, enabled)
        if env_var == "DEBUG_JOINTS_TO_STDOUT":
            self.btn_debug_joints.setChecked(enabled)
            self._apply_debug_button_style(self.btn_debug_joints, enabled)
            if enabled:
                self._print_pose_snapshot()
        label = "ON" if enabled else "OFF"
        self._log_button(f"Toggle {env_var} -> {label}")
        self._set_status(f"{env_var} -> {label}")

    def _start_gazebo(self):
        if self._block_if_managed("Start Gazebo"):
            return
        if self._gz_launching:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        gz_state = self._gazebo_state()
        if gz_state != "GAZEBO_OFF":
            self._log_error(f"Gazebo ya activo ({gz_state})")
            self._set_status(f"Gazebo ya activo ({gz_state})", error=True)
            set_led(self.led_gz, "on" if gz_state == "GAZEBO_READY" else "warn")
            self.signal_refresh_controls.emit()
            return
        self._log_button("Start Gazebo")
        world = self.world_combo.currentText().strip()
        self._log(f"[GZ] Mundo: {world}")
        if not world:
            self._log_error("Mundo no seleccionado")
            self._set_status("Selecciona mundo para Gazebo", error=True)
            return
        if not os.path.isfile(world):
            self._log_error(f"Archivo mundo no existe: {world}")
            self._set_status("Mundo no existe", error=True)
            return
        self._set_status("Lanzando Gazebo…")
        self._gz_launching = True
        self._gz_launch_start = time.time()
        self._set_launching_style(self.btn_gz_start, True)
        self.btn_gz_start.setEnabled(False)
        self._start_robot_state_publisher()
        self.gz_partition = f"ur5pro_{int(time.time())}"
        os.environ["GZ_PARTITION"] = self.gz_partition
        try:
            with open(GZ_PARTITION_FILE, "w", encoding="utf-8") as f:
                f.write(self.gz_partition)
        except Exception:
            pass

        def worker():
            ensure_dir(LOG_DIR)
            gz_log = os.path.join(LOG_DIR, "gz_server.log")
            rotate_log(gz_log)
            runtime_models_root = os.path.join(LOG_DIR, "gz_models")
            runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
            controllers_yaml = UR5_CONTROLLERS_YAML
            if not os.path.isfile(controllers_yaml):
                self._ui_set_status("No se encontró ur5_controllers.yaml", error=True)
                self.signal_set_led.emit(self.led_gz, "error")
                self._gz_launching = False
                self._gz_launch_start = 0.0
                self.signal_refresh_controls.emit()
                return
            try:
                ensure_dir(runtime_ur5_model)
                shutil.copytree(os.path.join(MODELS_DIR, "ur5_rg2"), runtime_ur5_model, dirs_exist_ok=True)
                model_sdf = os.path.join(runtime_ur5_model, "model.sdf")
                if os.path.isfile(model_sdf):
                    with open(model_sdf, "r", encoding="utf-8") as f:
                        sdf_text = f.read()
                    sdf_text = sdf_text.replace("$(env UR5_CONTROLLERS_YAML)", controllers_yaml)
                    sdf_text = re.sub(
                        r"<parameters>\\s*--params-file\\s+[^<]+</parameters>",
                        f"<parameters>{controllers_yaml}</parameters>",
                        sdf_text,
                        flags=re.DOTALL,
                    )
                    with open(model_sdf, "w", encoding="utf-8") as f:
                        f.write(sdf_text)
            except Exception:
                pass
            render_engine = os.environ.get("GZ_RENDER_ENGINE", "").strip() or "ogre2"
            env = (
                build_gz_env(self.gz_partition)
                + f"export GZ_SIM_RESOURCE_PATH='{runtime_models_root}:{MODELS_DIR}:{WORLDS_DIR}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
                "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; export QT_LOGGING_RULES='qt.qml.*=false'; "
            )
            mode = self._effective_mode()
            runtime_world = world
            try:
                if os.path.isfile(world):
                    with open(world, "r", encoding="utf-8") as f:
                        world_text = f.read()
                    keep_cameras = os.environ.get("PANEL_KEEP_CAMERAS", "").strip() in ("1", "true", "True")
                    if not keep_cameras:
                        keep_cameras = os.environ.get("PANEL_CAMERA_REQUIRED", "").strip() in ("1", "true", "True")
                    if mode != "gui" and not keep_cameras:
                        world_text = re.sub(
                            r"<plugin\s+filename=['\"]gz-sim-sensors-system['\"][\s\S]*?</plugin>",
                            "",
                            world_text,
                            flags=re.DOTALL,
                        )
                        for cam_name in (
                            "camera_overhead",
                            "camera_north",
                            "camera_south",
                            "camera_east",
                            "camera_west",
                        ):
                            pattern = rf"<model\s+name=['\"]{cam_name}['\"]>.*?</model>"
                            world_text = re.sub(pattern, "", world_text, flags=re.DOTALL)
                    world_text = world_text.replace(
                        "<uri>model://ur5_rg2</uri>",
                        f"<uri>file://{runtime_ur5_model}</uri>",
                    )
                    runtime_world = os.path.join(LOG_DIR, "world_runtime.sdf")
                    with open(runtime_world, "w", encoding="utf-8") as f:
                        f.write(world_text)
            except Exception:
                runtime_world = world
            if mode == "gui":
                cmd_core = with_line_buffer(f"gz sim -r -v 1 {shlex.quote(runtime_world)}")
                cmd = bash_preamble(self.ws_dir) + env + log_to_file(cmd_core, gz_log, None)
            else:
                cmd_core = with_line_buffer(
                    "gz sim -s -r --headless-rendering --render-engine ogre2 -v 1 "
                    + shlex.quote(runtime_world)
                )
                cmd = (
                    bash_preamble(self.ws_dir)
                    + env
                    + f"env -u DISPLAY GZ_RENDER_ENGINE={render_engine} "
                    + log_to_file(cmd_core, gz_log, None)
                )
            try:
                self.gz_proc = subprocess.Popen(
                    ["bash", "-lc", cmd],
                    preexec_fn=os.setsid,
                )
                def monitor():
                    rc = self.gz_proc.wait()
                    self._emit_log(f"[GZ] gz sim exited rc={rc} (log: {gz_log})")
                    try:
                        tail = subprocess.run(
                            ["bash", "-lc", f"tail -n 80 {shlex.quote(gz_log)}"],
                            text=True,
                            capture_output=True,
                            timeout=2.0,
                        )
                        if tail.stdout:
                            for line in tail.stdout.splitlines():
                                self._emit_log(f"[GZ][LOG] {line}")
                    except Exception:
                        pass
                threading.Thread(target=monitor, daemon=True).start()
                self._started_gazebo = True
                self._gz_running = True
                self._gz_world_name = read_world_name(world) or GZ_WORLD
                self._log("[GZ] Gazebo lanzado")
                self._invalidate_settle("gazebo start", restart=True)
                self._pose_info_ok = False
                self._ensure_pose_subscription()
                self._ui_set_status("Gazebo lanzado")
                self.signal_set_led.emit(self.led_gz, "on")
                self.signal_request_auto_bridge_start.emit()
                self._gz_launching = False
                self._gz_launch_start = 0.0
                self.signal_refresh_controls.emit()
            except Exception as exc:
                self._ui_set_status(f"Error lanzando Gazebo: {exc}", error=True)
                self.signal_set_led.emit(self.led_gz, "error")
                self._gz_launching = False
                self._gz_launch_start = 0.0
                self.signal_refresh_controls.emit()

        threading.Thread(target=worker, daemon=True).start()
        # Programar ajuste automático de joint2 tras bridge (no en arranque de Gazebo)
        self._auto_joint2_move_done = False

    def _parse_first_json_object(self, text: str) -> Optional[Dict[str, object]]:
        if not text:
            return None
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        return None

    def _detect_world_name(self) -> Optional[str]:
        world_path = self.world_combo.currentText().strip()
        return read_world_name(world_path) if world_path else None

    def _read_world_stats(self, world_name: str) -> Dict[str, object]:
        return {}

    def _try_unpause_world(self, world_name: str) -> bool:
        return False

    def _probe_pose_motion(self, world_name: str, targets: Set[str]) -> Tuple[Optional[bool], float, Optional[str]]:
        if not targets:
            return None, 0.0, None
        poses_a = self._read_world_pose_info(world_name)
        if not poses_a:
            return None, 0.0, None
        dt = 0.4
        time.sleep(dt)
        poses_b = self._read_world_pose_info(world_name)
        if not poses_b:
            return None, dt, None
        def extract(poses: List[Dict[str, object]]) -> Dict[str, Tuple[float, float, float]]:
            data: Dict[str, Tuple[float, float, float]] = {}
            for pose in poses:
                name = pose.get("name")
                if name not in targets:
                    continue
                position = pose.get("position") or {}
                try:
                    data[name] = (
                        float(position.get("x") or 0.0),
                        float(position.get("y") or 0.0),
                        float(position.get("z") or 0.0),
                    )
                except Exception:
                    continue
            return data
        a = extract(poses_a)
        b = extract(poses_b)
        if not a or not b:
            return None, dt, None
        for name, pa in a.items():
            pb = b.get(name)
            if not pb:
                continue
            dx = pb[0] - pa[0]
            dy = pb[1] - pa[1]
            dz = pb[2] - pa[2]
            if abs(dx) > 1e-4 or abs(dy) > 1e-4 or abs(dz) > 1e-4:
                return True, dt, name
        return False, dt, None

    def check_physics_runtime(self) -> None:
        world_path = self.world_combo.currentText().strip()
        world_name = self._detect_world_name() or (read_world_name(world_path) if world_path else GZ_WORLD)
        self._gz_world_name = world_name
        self._ensure_pose_subscription()
        if not self._pose_info_ok:
            return
        stepping, dt, sample = self._probe_pose_motion(world_name, self._settle_targets())
        if stepping is None:
            for _ in range(3):
                time.sleep(0.25)
                stepping, dt, sample = self._probe_pose_motion(world_name, self._settle_targets())
                if stepping is not None:
                    break
        gravity = _parse_sdf_gravity(world_path)
        step_txt = "unknown" if stepping is None else str(stepping)
        inferred_by = "pose_delta" if stepping is not None else "none"
        dt_txt = f"{dt:.2f}s" if stepping is not None else "n/a"
        sample_txt = sample or "n/a"
        self._emit_log(
            f"[PHYSICS] world={world_name} gravity={gravity} stepping={step_txt} "
            f"inferred_by={inferred_by} dt={dt_txt} sample_model={sample_txt}"
        )

    def _schedule_physics_runtime_check(self) -> None:
        if self._physics_runtime_check_scheduled or self._closing:
            return
        self._physics_runtime_check_scheduled = True

        def _run():
            self._physics_runtime_check_scheduled = False
            if self._closing or not self._gz_running:
                return
            count, age = (0, float("inf"))
            if self._ros_worker_started and self.ros_worker.node_ready():
                count, age = self.ros_worker.pose_info_status()
            if count > 0 and age < POSE_INFO_MAX_AGE_SEC:
                threading.Thread(target=self.check_physics_runtime, daemon=True).start()
                return
            self._schedule_physics_runtime_check()

        QTimer.singleShot(int(POSE_INFO_POLL_SEC * 1000), _run)

    def _throw_objects(self):
        self._log_button("Lanzar objetos")
        self._set_status("Lanzando objetos en Gazebo…")

        # LEGACY: deshabilitado en modo física real (MoveIt-only / no teletransporte).
        self._log("[SAFETY] Teleport detectado: bloqueado (throw_objects)")
        self._set_status("Bloqueado: no se permite teletransporte", error=True)
        return

    def _toggle_debug_poses(self):
        """Toggle on/off streaming de poses y joints hacia el terminal del panel."""
        if self.btn_debug_joints.isChecked():
            self._start_debug_poses()
        else:
            self._stop_debug_poses()

    def _start_debug_poses(self):
        """Inicia streaming de poses parseadas (nombre + posición)."""
        if not self._gz_running:
            self._log_error("Gazebo no está activo")
            self.btn_debug_joints.setChecked(False)
            return
        self._stop_debug_poses()
        self._ensure_pose_subscription()
        self._print_pose_snapshot()
        self._pose_debug_timer = QTimer(self)
        self._pose_debug_timer.timeout.connect(self._print_pose_snapshot)
        self._pose_debug_timer.start(int(DEBUG_POSES_PERIOD_SEC * 1000))
        self._log("[DEBUG] Iniciado - snapshots de poses")
        self._apply_debug_button_style(self.btn_debug_joints, True)

    def _stop_debug_poses(self):
        """Detiene streaming de poses y joints."""
        if self._pose_debug_timer:
            self._pose_debug_timer.stop()
            self._pose_debug_timer.deleteLater()
            self._pose_debug_timer = None
        self._apply_debug_button_style(self.btn_debug_joints, False)
        self._log("[DEBUG] Detenido")

    def _apply_debug_button_style(self, button: QPushButton, enabled: bool) -> None:
        button.setStyleSheet(
            "background:#3b82f6; color:white; border-radius:6px;" if enabled else ""
        )

    def _print_pose_snapshot(self):
        """Imprime en una línea: TCP, cesta, mesa y objetos en frame world."""
        objs = {}
        if self._gz_running:
            world_name = self._gz_world_name or read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
            poses = self._read_world_pose_info(world_name)
            if poses:
                for pose in poses:
                    name = pose.get("name")
                    pos = pose.get("position") or {}
                    if name and name in get_object_positions():
                        objs[name] = (
                            float(pos.get("x") or 0.0),
                            float(pos.get("y") or 0.0),
                            float(pos.get("z") or 0.0),
                        )
            if not objs:
                objs = get_object_positions()
        obj_parts = []
        for name, (x, y, z) in sorted(objs.items()):
            obj_parts.append(f"{name}=({x:.3f},{y:.3f},{z:.3f})")
        obj_txt = "objs: " + (" ".join(obj_parts) if obj_parts else "-" )

        tcp_txt = "tcp=--"
        if self._last_tcp_world and self._last_tcp_rpy_deg:
            wx, wy, wz = self._last_tcp_world
            r_deg, p_deg, y_deg = self._last_tcp_rpy_deg
            tcp_txt = f"tcp=({wx:.3f},{wy:.3f},{wz:.3f}) rpy=({r_deg:.1f},{p_deg:.1f},{y_deg:.1f})"

        basket_x, basket_y, basket_z = BASKET_DROP
        basket_txt = f"basket=({basket_x:.3f},{basket_y:.3f},{basket_z:.3f})"
        table_txt = f"table=({TABLE_CENTER_X:.3f},{TABLE_CENTER_Y:.3f})"

        line = f"[DEBUG POSES] {tcp_txt} | {basket_txt} | {table_txt} | {obj_txt}"
        self._emit_log(line)

    def _drop_detach_supported(self) -> bool:
        if self._detach_feature_checked:
            return self._detach_feature_available
        self._detach_feature_checked = True
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if self.ros_worker and self.ros_worker.node_ready():
            if self.ros_worker.has_service("release_objects"):
                self._detach_feature_available = True
                return True
            try:
                topics = self.ros_worker.list_topic_names()
            except Exception:
                topics = []
            prefix = f"{GRIPPER_ATTACH_PREFIX}/"
            for topic in topics:
                if topic.startswith(prefix) and topic.endswith("/detach"):
                    self._detach_feature_available = True
                    return True
        self._detach_feature_available = False
        return False

    def _release_objects(self):
        """Publicar detach a todos los objetos en Gazebo (emergencia)."""
        if self._objects_release_done:
            return
        if self._detach_inflight:
            return
        if self._detach_auto_disabled and time.time() < self._detach_backoff_until:
            return
        if not self._drop_detach_supported():
            self._objects_release_done = True
            if not self._detach_feature_logged:
                self._emit_log("[PHYSICS][DETACH] disabled: no attach/detach channel available")
                self._detach_feature_logged = True
            return
        self._detach_inflight = True
        self._detach_attempted = True
        self._log_button("Soltar objetos")
        self._ui_set_status("Soltando objetos…")

        def worker():
            try:
                if not ROS_AVAILABLE or self._moveit_node is None:
                    self._log_error("ROS no disponible; no se pudo soltar objetos")
                    self._ui_set_status("Servicio objetos no accesible", error=True)
                    return
                client = self._moveit_node.create_client(Trigger, "release_objects")
                if not client.wait_for_service(timeout_sec=0.8):
                    self._log_error("Servicio release_objects no disponible")
                    self._ui_set_status("Servicio release_objects no disponible", error=True)
                    return
                attempts = 5
                backoff = 0.3
                success = False
                for attempt in range(1, attempts + 1):
                    t0 = time.time()
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(self._moveit_node, future, timeout_sec=0.8)
                    ok = bool(future.done() and future.result() and future.result().success)
                    dt = time.time() - t0
                    self._emit_log(f"[PHYSICS][DETACH] attempt={attempt} ok={ok} dt={dt:.3f}s")
                    if ok:
                        success = True
                        break
                    time.sleep(backoff)
                if not success:
                    self._emit_log(f"[PHYSICS][DETACH] FAILED after {attempts} attempts (non-blocking)")
                    self._ui_set_status("⚠ Detach no confirmado", error=True)
                    self._detach_backoff_until = time.time() + 30.0
                    self._detach_auto_disabled = True
                    return
                self._objects_release_done = True
                self._objects_settled = False
                self.signal_refresh_controls.emit()
                self._ui_set_status("✅ Objetos soltados")
                self._invalidate_settle("objetos liberados", restart=True)
            except Exception as e:
                self._log_error(f"Soltar objetos error: {e}")
                self._ui_set_status(f"Error soltando objetos: {e}", error=True)
            finally:
                self._detach_inflight = False

        threading.Thread(target=worker, daemon=True).start()

    def _start_release_service(self):
        if self.release_service_proc is not None and self.release_service_proc.poll() is None:
            return
        try:
            ensure_dir(LOG_DIR)
            svc_log = os.path.join(LOG_DIR, "release_objects_service.log")
            rotate_log(svc_log)
            env = build_gz_env(self.gz_partition) + f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
            cmd_core = with_line_buffer("ros2 run ur5_tools release_objects_service")
            cmd = bash_preamble(self.ws_dir) + env + f"{cmd_core} > '{svc_log}' 2>&1"
            self.release_service_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            self._started_release_service = True
        except Exception as exc:
            self._log_error(f"Error iniciando release_objects_service: {exc}")

    def _start_world_tf_publisher(self, world_name: str) -> None:
        if self.world_tf_proc is not None and self.world_tf_proc.poll() is None:
            return
        try:
            ensure_dir(LOG_DIR)
            tf_log = os.path.join(LOG_DIR, "world_tf_publisher.log")
            rotate_log(tf_log)
            use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
            sim_arg = " -p use_sim_time:=true" if use_sim_time else ""
            cmd_core = with_line_buffer(
                "ros2 run ur5_tools world_tf_publisher "
                f"--ros-args -p world_name:={shlex.quote(world_name)} "
                f"-p model_name:={shlex.quote(UR5_MODEL_NAME)} "
                f"-p base_frame:=base_link -p world_frame:=world{sim_arg}"
            )
            cmd = bash_preamble(self.ws_dir) + f"{cmd_core} > '{tf_log}' 2>&1"
            self.world_tf_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            self._started_world_tf = True
            self._emit_log("[TF] world_tf_publisher lanzado")
        except Exception as exc:
            self._log_error(f"[TF] Error iniciando world_tf_publisher: {exc}")

    def _stop_world_tf_publisher(self) -> None:
        self._kill_proc(self.world_tf_proc, "world_tf_publisher")
        self.world_tf_proc = None
        self._started_world_tf = False

    def _stop_gazebo(self):
        if self._block_if_managed("Stop Gazebo"):
            return
        self._log_button("Stop Gazebo")
        self._set_status("Deteniendo Gazebo…")
        self._stop_debug_poses()
        self._kill_proc(self.gz_proc, "gz sim")
        self.gz_proc = None
        self._kill_proc(self.rsp_proc, "robot_state_publisher")
        self.rsp_proc = None
        self._stop_world_tf_publisher()
        self._kill_proc(self.release_service_proc, "release_objects_service")
        self.release_service_proc = None
        self._objects_settled = False
        self._objects_seen_fall = False
        self._objects_release_done = False
        self._trace_ready = False
        self._tf_ready_state = False
        self._controllers_ok = False
        self._controllers_reason = "gazebo detenido"
        self._detach_inflight = False
        self._detach_attempted = False
        self._detach_auto_disabled = False
        self._detach_backoff_until = 0.0
        self._pose_info_ok = False
        self._gz_running = False
        set_led(self.led_gz, "off")

    def _start_robot_state_publisher(self):
        if self.rsp_proc is not None and self.rsp_proc.poll() is None:
            return
        try:
            ensure_dir(LOG_DIR)
            rsp_log = os.path.join(LOG_DIR, "ur5_rsp.log")
            rotate_log(rsp_log)
            env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
            cmd_core = with_line_buffer("ros2 launch ur5_bringup ur5_rsp.launch.py use_sim_time:=true")
            cmd = bash_preamble(self.ws_dir) + env + f"{cmd_core} > '{rsp_log}' 2>&1"
            self.rsp_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            self._started_rsp = True
            self._emit_log("[TF] robot_state_publisher lanzado")
        except Exception as exc:
            self._log_error(f"Error lanzando robot_state_publisher: {exc}")

    def _start_bridge(self):
        if self._block_if_managed("Start bridge"):
            return
        if self._bridge_launching:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        world_name = read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
        pose_topic = f"/world/{world_name}/pose/info"
        if self.ros_worker.node_ready() and self.ros_worker.topic_has_publishers(pose_topic):
            self._log_warning("Bridge ya activo (pose/info con publishers)")
            self._set_status("Bridge ya activo (pose/info detectado)", error=False)
            self._bridge_running = True
            set_led(self.led_bridge, "on")
            self.signal_refresh_controls.emit()
            return
        self._log_button("Start bridge")
        self._camera_stream_ok = False
        self._calibration_ready = False
        self._pose_info_ok = False
        self._pose_info_msg_count = 0
        self._pose_info_last_age = float("inf")
        self._pose_info_last_log = 0.0
        self._tf_no_msgs_logged = False
        # Permitir lanzar bridge en cuanto Gazebo esté realmente arriba (aunque el flag interno tarde en activarse)
        gz_state = self._gazebo_state()
        if gz_state == "GAZEBO_OFF":
            self._log_warning("Bridge en espera: Gazebo no activo")
            self._set_status("Bridge en espera: Gazebo no activo", error=False)
            set_led(self.led_bridge, "warn")
            return
        self._ensure_ros_worker_started()
        # Actualizar flag interno si lo detectamos corriendo
        if gz_state != "GAZEBO_OFF" and not self._gz_running:
            self._gz_running = True
            set_led(self.led_gz, "on")
            self._refresh_controls()
        base_yaml = self.bridge_edit.text().strip()
        self._log(f"[BRIDGE] YAML base: {base_yaml}")
        if not base_yaml or not os.path.isfile(base_yaml):
            self._log_error(f"YAML no encontrado: {base_yaml}")
            self._set_status("YAML no encontrado", error=True)
            set_led(self.led_bridge, "error")
            return
        self._start_robot_state_publisher()
        self._trace_ready = False
        self._reset_trace_throttle("bridge start")
        self._bridge_ready = False
        self._set_status("Lanzando bridge…")
        self._bridge_launching = True
        self._bridge_launch_start = time.time()
        self._set_launching_style(self.btn_bridge_start, True)
        self.btn_bridge_start.setEnabled(False)
        # Deshabilitar botón mientras arranca para que se vea en gris como Gazebo
        self._bridge_running = True
        self._started_bridge = True
        self._bridge_start_ts = time.time()
        self._refresh_controls()

        def worker():
            ensure_dir(LOG_DIR)
            runtime_yaml = base_yaml
            world_name = read_world_name(self.world_combo.currentText().strip()) or GZ_WORLD
            try:
                with open(base_yaml, "r", encoding="utf-8") as f:
                    yaml_text = f.read()
                yaml_text = yaml_text.replace(
                    "/world/ur5_mesa_objetos/",
                    f"/world/{world_name}/",
                )
                runtime_yaml = os.path.join(LOG_DIR, "bridge_runtime.yaml")
                with open(runtime_yaml, "w", encoding="utf-8") as f:
                    f.write(yaml_text)
            except Exception:
                runtime_yaml = base_yaml
            br_log = os.path.join(LOG_DIR, "ros_gz_bridge.log")
            rotate_log(br_log)
            env = (
                build_gz_env(resolve_gz_partition(self.gz_partition))
                + f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
            )
            cmd_core = with_line_buffer(
                "ros2 run ros_gz_bridge parameter_bridge --ros-args "
                + f"-p config_file:='{runtime_yaml}' -p lazy:=true"
            )
            cmd = bash_preamble(self.ws_dir) + env + f"{cmd_core} > '{br_log}' 2>&1"
            try:
                self.bridge_proc = subprocess.Popen(
                    ["bash", "-lc", cmd],
                    preexec_fn=os.setsid,
                )
                self._ui_set_status("Bridge lanzado")
                if not self._debug_logs_enabled:
                    self._emit_log("[INFO] Bridge lanzado")
                self.signal_set_led.emit(self.led_bridge, "on")
                self._bridge_running = True
                self._bridge_launching = False
                self._bridge_launch_start = 0.0
                self.signal_refresh_controls.emit()
                self._start_world_tf_publisher(world_name)
                self._spawn_controllers_async()
                self.signal_bridge_ready.emit()
            except Exception as exc:
                self._ui_set_status(f"Error lanzando bridge: {exc}", error=True)
                self.signal_set_led.emit(self.led_bridge, "error")
                self._bridge_running = False
                self._bridge_launching = False
                self._bridge_launch_start = 0.0
                self.signal_refresh_controls.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _spawn_controllers_async(self):
        def worker():
            if self._controller_spawn_done and self._controllers_ok:
                return
            if self._controller_spawn_inflight:
                return
            self._controller_spawn_last_start = time.time()
            self._controller_spawn_inflight = True
            for _ in range(20):
                if self._ros2_control_available():
                    break
                time.sleep(0.25)
            if not self._ros2_control_available():
                self._log_error("controller_manager no disponible; no se pueden spawnear controladores")
                self._controller_spawn_inflight = False
                return
            clock_ok = False
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                if self._ros_worker_started:
                    clock_ok, _age = self.ros_worker.clock_alive()
                if clock_ok:
                    break
                time.sleep(0.25)
            if not clock_ok:
                self._log_error("/clock no disponible; abortando spawners")
                self._controller_spawn_inflight = False
                return

            try:
                self._emit_log("[CTRL] Ajustando parámetros de gz_ros_control…")
                cmd_core = with_line_buffer(
                    "ros2 run ur5_tools gz_ros_control_guard "
                    "--ros-args -p use_sim_time:=true -p hold_joints:=false"
                )
                cmd = bash_preamble(self.ws_dir) + cmd_core
                if self._debug_logs_enabled:
                    subprocess.run(["bash", "-lc", cmd], timeout=12)
                else:
                    subprocess.run(
                        ["bash", "-lc", cmd],
                        timeout=12,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
            except Exception as exc:
                self._log_warning(f"[CTRL] gz_ros_control_guard falló: {exc}")
            cm_path = self._controller_manager_path() or "/controller_manager"
            list_srv = f"{cm_path}/list_controllers"
            resp, err = self._list_controllers(list_srv)
            if resp is None:
                self._log_error(f"list_controllers falló: {err or 'sin respuesta'}")
                self._controller_spawn_inflight = False
                return
            state_map = {c.name: c.state for c in resp.controller}
            controllers = ["joint_state_broadcaster", "joint_trajectory_controller"]
            if gripper_controller_defined():
                controllers.append("gripper_controller")
            else:
                self._emit_log("[CTRL] gripper_controller no definido; omitido.")

            to_spawn = []
            to_activate = []
            for name in controllers:
                state = state_map.get(name)
                if state is None:
                    to_spawn.append(name)
                    continue
                kind = self._controller_state_kind(state)
                if kind == "ACTIVE":
                    continue
                if kind == "INACTIVE":
                    to_activate.append(name)
                elif kind == "LOADED":
                    to_activate.append(name)
                else:
                    self._log_warning(f"[CTRL] Estado desconocido para {name}: {state}")

            if not to_spawn and not to_activate:
                self._emit_log("[CTRL] Controladores ya activos; no se requiere spawner.")
                self._controller_spawn_inflight = False
                self._controller_spawn_done = True
                return

            self._emit_log("[CTRL] Preparando controladores ros2_control…")
            env = os.environ.copy()
            for name in to_spawn:
                cmd = [
                    "ros2", "run", "controller_manager", "spawner",
                    name,
                    "-c", cm_path,
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "30",
                ]
                try:
                    if self._debug_logs_enabled:
                        res = subprocess.run(cmd, timeout=40, env=env)
                    else:
                        res = subprocess.run(
                            cmd,
                            timeout=40,
                            env=env,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                    if res.returncode != 0:
                        self._log_error(f"Spawner falló rc={res.returncode}: {' '.join(cmd)}")
                except Exception as exc:
                    self._log_error(f"Spawner error: {exc}")

            for name in to_activate:
                cmd = ["ros2", "control", "set_controller_state", name, "active"]
                try:
                    if self._debug_logs_enabled:
                        res = subprocess.run(cmd, timeout=20, env=env)
                    else:
                        res = subprocess.run(
                            cmd,
                            timeout=20,
                            env=env,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                    if res.returncode != 0:
                        self._log_error(f"Activación falló rc={res.returncode}: {' '.join(cmd)}")
                except Exception as exc:
                    self._log_error(f"Activación error: {exc}")

            self._emit_log("[CTRL] Preparación de controladores finalizada.")
            self._controller_spawn_inflight = False
            self._controller_spawn_done = True

        threading.Thread(target=worker, daemon=True).start()

    def _stop_bridge(self):
        if self._block_if_managed("Stop bridge"):
            return
        self._log_button("Stop bridge")
        self._camera_stream_ok = False
        self._camera_topic_hz = 0.0
        self._set_status("Deteniendo bridge…")
        self._kill_proc(self.bridge_proc, "parameter_bridge")
        self._kill_proc(self.rsp_proc, "robot_state_publisher")
        self._stop_world_tf_publisher()
        self.rsp_proc = None
        self._kill_proc(self.release_service_proc, "release_objects_service")
        self._trace_ready = False
        self._tf_ready_state = False
        self._reset_trace_throttle("bridge stop")
        if self._tf_ready_timer:
            self._tf_ready_timer.stop()
        if self._pose_info_timer:
            self._pose_info_timer.stop()
        self.bridge_proc = None
        self.release_service_proc = None
        self._bridge_running = False
        self._bridge_start_ts = 0.0
        set_led(self.led_bridge, "off")
        self._refresh_controls()
        self._bridge_ready = False
        self._controllers_ok = False
        self._controllers_reason = "bridge detenido"
        self._controller_spawn_inflight = False
        self._controller_spawn_done = False
        self._objects_release_done = False
        self._pose_info_ok = False

    def _start_moveit(self):
        if self._block_if_managed("Start MoveIt"):
            return
        if self._moveit_launching:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if self._move_group_ready():
            self.signal_moveit_state.emit("READY", "move_group ya activo")
            self._set_status("MoveIt ya activo")
            return
        if self.moveit_proc is not None and self.moveit_proc.poll() is None:
            self.signal_moveit_state.emit("WAITING_MOVEIT_READY", "move_group arrancado; esperando disponibilidad")
            threading.Thread(target=self._wait_for_moveit_ready, daemon=True).start()
            return
        self._log_button("Start MoveIt")
        self._set_status("Lanzando MoveIt…")
        self.signal_moveit_state.emit("STARTING", "launching move_group")
        self._moveit_launching = True
        self._moveit_launch_start = time.time()
        self._set_launching_style(self.btn_moveit_start, True)
        self.btn_moveit_start.setEnabled(False)
        try:
            ensure_dir(LOG_DIR)
            moveit_log = os.path.join(LOG_DIR, "moveit_bringup.log")
            rotate_log(moveit_log)
            env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
            use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
            sim_arg = " use_sim_time:=true" if use_sim_time else " use_sim_time:=false"
            cmd_core = with_line_buffer(
                "ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py "
                f"start_ros2_control:=false launch_rviz:=false{sim_arg}"
            )
            cmd = bash_preamble(self.ws_dir) + env + f"{cmd_core} > '{moveit_log}' 2>&1"
            self.moveit_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            self._started_moveit = True
            self._moveit_running = True
            self._set_status("MoveIt lanzado")
            self._refresh_controls()
            self.signal_moveit_state.emit("WAITING_MOVEIT_READY", "esperando move_group")
            threading.Thread(target=self._wait_for_moveit_ready, daemon=True).start()
        except Exception as exc:
            self._set_status(f"Error lanzando MoveIt: {exc}", error=True)
            self.signal_moveit_state.emit("ERROR", f"launch failed: {exc}")
            self._moveit_running = False
            self._moveit_launching = False
            self._moveit_launch_start = 0.0

    def _stop_moveit(self):
        if self._block_if_managed("Stop MoveIt"):
            return
        self._log_button("Stop MoveIt")
        self._set_status("Deteniendo MoveIt…")
        self._kill_proc(self.moveit_proc, "move_group")
        self.moveit_proc = None
        self._moveit_running = False
        self.signal_moveit_state.emit("OFF", "manual")
        self._refresh_controls()

    def _wait_for_moveit_ready(self):
        """Esperar a que move_group esté listo (action server + topics)."""
        deadline = time.monotonic() + max(2.0, MOVEIT_READY_TIMEOUT_SEC)
        while time.monotonic() < deadline:
            if self._closing:
                return
            if self.moveit_proc is None or self.moveit_proc.poll() is not None:
                try:
                    self.signal_moveit_state.emit("ERROR", "move_group terminó")
                except RuntimeError:
                    return
                self._moveit_launching = False
                self._moveit_launch_start = 0.0
                self.signal_refresh_controls.emit()
                return
            if not self._ros_worker_started:
                self._ensure_ros_worker_started()
            if self._move_group_ready():
                try:
                    self.signal_moveit_state.emit("READY", "move_group activo")
                except RuntimeError:
                    return
                self._moveit_launching = False
                self._moveit_launch_start = 0.0
                self.signal_refresh_controls.emit()
                return
            time.sleep(0.5)
        try:
            action_ready = self._moveit_action_ready()
            status_ready = self._moveit_status_ready()
            traj_ready = self._follow_joint_traj_ready()
            self._emit_log(
                f"[MOVEIT] READY checks: action={action_ready} status={status_ready} traj={traj_ready}"
            )
            self.signal_moveit_state.emit("WAITING_MOVEIT_READY", "timeout esperando move_group")
        except RuntimeError:
            return
        self.signal_refresh_controls.emit()

    def _start_moveit_bridge(self):
        if self._block_if_managed("Start MoveIt bridge"):
            return
        if self.moveit_bridge_proc is not None and self.moveit_bridge_proc.poll() is None:
            return
        if not self._ros_worker_started:
            self._ensure_ros_worker_started()
        if self._moveit_bridge_detected():
            self._set_status("MoveIt bridge ya activo", error=True)
            self._moveit_bridge_running = True
            set_led(self.led_moveit_bridge, "on")
            self._refresh_controls()
            return
        self._log_button("Start MoveIt bridge")
        if self._moveit_state != MoveItState.READY:
            self._log_warning("MoveIt no está activo; el bridge puede fallar")
        try:
            ensure_dir(LOG_DIR)
            bridge_log = os.path.join(LOG_DIR, "moveit_bridge.log")
            rotate_log(bridge_log)
            env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
            use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
            sim_arg = " --ros-args -p use_sim_time:=true" if use_sim_time else ""
            cmd_core = with_line_buffer(f"ros2 run ur5_tools ur5_moveit_bridge{sim_arg}")
            cmd = bash_preamble(self.ws_dir) + env + f"{cmd_core} > '{bridge_log}' 2>&1"
            self.moveit_bridge_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            self._started_moveit_bridge = True
            self._moveit_bridge_running = True
            set_led(self.led_moveit_bridge, "on")
            self._set_status("MoveIt bridge lanzado")
            self._refresh_controls()
        except Exception as exc:
            self._set_status(f"Error lanzando MoveIt bridge: {exc}", error=True)
            set_led(self.led_moveit_bridge, "error")
            self._moveit_bridge_running = False

    def _stop_moveit_bridge(self):
        if self._block_if_managed("Stop MoveIt bridge"):
            return
        self._log_button("Stop MoveIt bridge")
        self._set_status("Deteniendo MoveIt bridge…")
        self._kill_proc(self.moveit_bridge_proc, "ur5_moveit_bridge")
        self.moveit_bridge_proc = None
        self._moveit_bridge_running = False
        set_led(self.led_moveit_bridge, "off")
        self._refresh_controls()

    def _kill_proc(self, proc, label: str):
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            proc.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def _proc_alive(self, proc) -> bool:
        return proc is not None and proc.poll() is None


    def _save_home_from_sliders(self):
        """Lee los valores actuales de los sliders y guarda como nueva pose HOME."""
        from .panel_utils import save_home_pose
        joint_values = []
        for slider in self.joint_sliders:
            deg = slider.value() / JOINT_SLIDER_SCALE
            rad = math.radians(deg)
            joint_values.append(rad)
        save_home_pose(joint_values)
        self._set_status("Pose HOME guardada", error=False)

    def _rosbag_running(self) -> bool:
        return self._proc_alive(self.bag_proc)

    def _start_bag(self):
        self._log_button("Start bag")
        if self._proc_alive(self.bag_proc):
            self._set_status("Bag ya activo", error=True)
            return
        if not self._bridge_running:
            self._log_warning("Bag en espera: bridge no activo")
            self._set_status("Bag en espera: bridge no activo", error=False)
            set_led(self.led_bag, "warn")
            return
        name = self.bag_name.text().strip() or f"demo_{int(time.time())}"
        topics_raw = self.bag_topics.text().strip()
        self._log(f"[BAG] Nombre: {name}, Tópicos raw: {topics_raw}")
        topics, invalid = parse_ros_topics(topics_raw)
        if invalid:
            self._log_error(f"Tópicos inválidos: {invalid}")
            self._set_status("Tópicos inválidos", error=True)
            set_led(self.led_bag, "error")
            return
        if not topics:
            self._log_error("No hay tópicos para grabar")
            self._set_status("No hay tópicos", error=True)
            set_led(self.led_bag, "error")
            return
        ensure_dir(BAGS_DIR)
        outdir = os.path.join(BAGS_DIR, name)
        if os.path.exists(outdir):
            suffix = 1
            while os.path.exists(f"{outdir}_{suffix}"):
                suffix += 1
            outdir = f"{outdir}_{suffix}"
        self._log(f"[BAG] Directorio salida: {outdir}")
        self._set_status(f"Grabando bag en {outdir}…")
        set_led(self.led_bag, "warn")

        def worker():
            cmd = (
                bash_preamble(self.ws_dir)
                + "ros2 bag record -o "
                + shlex.quote(outdir)
                + " --topics "
                + " ".join(shlex.quote(t) for t in topics)
            )
            try:
                self.bag_proc = subprocess.Popen([
                    "bash",
                    "-lc",
                    cmd,
                ], preexec_fn=os.setsid)
                self._bag_running = True
                self._started_bag = True
                self._ui_set_status(f"Bag grabando → {outdir}")
                self.signal_set_led.emit(self.led_bag, "on")
                self.signal_refresh_controls.emit()
            except Exception as exc:
                self._ui_set_status(f"Error al grabar bag: {exc}", error=True)
                self.signal_set_led.emit(self.led_bag, "error")
                self.signal_refresh_controls.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _stop_bag(self):
        self._log_button("Stop bag")
        self._set_status("Deteniendo bag…")
        self._kill_proc(self.bag_proc, "ros2 bag record")
        self.bag_proc = None
        self._bag_running = False
        set_led(self.led_bag, "off")

    def _refresh_status_sync(self):
        """Chequeo síncrono de estado al startup."""
        clock_ok, _ = self._clock_status()
        gz_state = self._gazebo_state()
        gz_ok = gz_state == "GAZEBO_READY"
        br_ok = self._bridge_running or self._pose_info_active()
        bag_ok = self._rosbag_running()
        ctrl_ok = self._ros2_control_available()
        moveit_ok = self._moveit_ready()
        moveit_bridge_ok = self._moveit_bridge_detected()
        self._apply_status(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)

    def _refresh_status_async(self):
        if self._status_check_inflight:
            return
        self._status_check_inflight = True

        def worker():
            if self._managed_mode and self._external_state_active():
                state = self._system_state
                external_state = (self._external_state or "").upper()
                gz_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO")
                br_ok = external_state not in ("BOOT", "BOOTING", "WAITING_GAZEBO", "WAITING_BRIDGE")
                clock_ok = gz_ok
                ctrl_ok = state not in (
                    SystemState.BOOT,
                    SystemState.WAITING_GAZEBO,
                    SystemState.WAITING_CONTROLLERS,
                )
                moveit_ok = self._moveit_ready()
                moveit_bridge_ok = self._moveit_bridge_detected()
                bag_ok = self._bag_running
                self.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
                self._status_check_inflight = False
                return
            clock_ok, _ = self._clock_status()
            gz_state = self._gazebo_state()
            gz_ok = gz_state == "GAZEBO_READY"
            br_ok = self._bridge_running or self._pose_info_active()
            bag_ok = self._rosbag_running()
            ctrl_ok = self._ros2_control_available() if br_ok else False
            moveit_ok = self._moveit_ready()
            moveit_bridge_ok = self._moveit_bridge_detected()
            # [REMOVED REPETITIVE STATUS LOG] - solo loguea si hay cambios en el estado
            self.status_updated.emit(gz_ok, br_ok, clock_ok, bag_ok, ctrl_ok, moveit_ok, moveit_bridge_ok)
            self._status_check_inflight = False

        threading.Thread(target=worker, daemon=True).start()

    def _apply_status(
        self,
        gz_ok: bool,
        br_ok: bool,
        clock_ok: bool,
        bag_ok: bool,
        ctrl_ok: bool,
        moveit_ok: bool,
        moveit_bridge_ok: bool,
    ):
        gz_state = self._gz_state or "GAZEBO_OFF"
        if gz_state == "GAZEBO_READY":
            set_led(self.led_gz, "on")
        elif gz_state == "GAZEBO_STARTING":
            set_led(self.led_gz, "warn")
        else:
            set_led(self.led_gz, "off")
        set_led(self.led_bridge, "on" if br_ok else "off")
        set_led(self.led_clock, "on" if clock_ok else "off")
        set_led(self.led_bag, "on" if bag_ok else "off")
        set_led(self.led_ros2, "on" if ctrl_ok else "off")
        set_led(self.led_ur5, "on" if gz_ok else "off")
        if self._moveit_state == MoveItState.STARTING:
            set_led(self.led_moveit, "warn")
        elif self._moveit_state == MoveItState.WAITING_MOVEIT_READY:
            set_led(self.led_moveit, "warn")
        elif self._moveit_state == MoveItState.READY:
            set_led(self.led_moveit, "on")
        elif self._moveit_state == MoveItState.ERROR:
            set_led(self.led_moveit, "error")
        else:
            set_led(self.led_moveit, "off")
        set_led(self.led_moveit_bridge, "on" if moveit_bridge_ok else "off")
        self._gz_running = gz_ok
        self._bridge_running = br_ok
        self._bag_running = bag_ok
        self._moveit_running = moveit_ok
        self._moveit_bridge_running = moveit_bridge_ok
        if self._moveit_state == MoveItState.OFF and moveit_ok:
            self._moveit_state = MoveItState.READY
            if self._moveit_bridge_detected():
                self._moveit_state_reason = "moveit_bridge detectado"
            else:
                self._moveit_state_reason = "move_group detectado"
            set_led(self.led_moveit, "on")
        if self._moveit_state == MoveItState.READY and not moveit_ok:
            self._moveit_state = MoveItState.WAITING_MOVEIT_READY
            self._moveit_state_reason = "move_group no responde"
            set_led(self.led_moveit, "warn")
        if self._moveit_state in (MoveItState.STARTING, MoveItState.WAITING_MOVEIT_READY) and moveit_ok:
            self._moveit_state = MoveItState.READY
            self._moveit_state_reason = "move_group listo"
            set_led(self.led_moveit, "on")
        self._update_moveit_status_label()
        summary = []
        if gz_state == "GAZEBO_STARTING":
            summary.append("GZ:starting")
        else:
            summary.append(f"GZ:{'on' if gz_ok else 'off'}")
        summary.append(f"BR:{'on' if br_ok else 'off'}")
        summary.append(f"CLK:{'on' if clock_ok else 'off'}")
        summary.append(f"BAG:{'on' if bag_ok else 'off'}")
        summary.append(f"CTRL:{'on' if ctrl_ok else 'off'}")
        summary.append(f"MVT:{'on' if moveit_ok else 'off'}")
        summary.append(f"MBR:{'on' if moveit_bridge_ok else 'off'}")
        self.status_lbl.setText(" · ".join(summary))
        self._update_system_stats()
        self._refresh_controls()

    def _moveit_topics_ready(self) -> bool:
        if not self.ros_worker or not self.ros_worker.node_ready():
            return False
        return (
            self.ros_worker.topic_has_publishers("/move_action/status")
            or self.ros_worker.topic_has_publishers("/move_group/status")
            or self.ros_worker.topic_has_publishers("/planning_scene")
        )

    def _moveit_status_ready(self) -> bool:
        if not self.ros_worker or not self.ros_worker.node_ready():
            return False
        if self._moveit_topics_ready():
            return True
        return (
            self.ros_worker.has_service("/get_planning_scene")
            or self.ros_worker.has_service("/move_group/get_planning_scene")
        )

    def _moveit_action_ready(self) -> bool:
        if ActionClient is None or MoveGroup is None:
            return False
        if self._moveit_node is None:
            try:
                self._init_moveit_publisher()
            except Exception:
                return False
        if self._moveit_node is None:
            return False
        action_names = ["/move_action", "/move_group"]
        for name in action_names:
            if self._moveit_action_client is None or getattr(self._moveit_action_client, "action_name", "") != name:
                try:
                    self._moveit_action_client = ActionClient(self._moveit_node, MoveGroup, name)
                except Exception:
                    self._moveit_action_client = None
                    continue
            try:
                if self._moveit_action_client.wait_for_server(timeout_sec=0.2):
                    return True
            except Exception:
                continue
        return False

    def _follow_joint_traj_ready(self) -> bool:
        if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
            return False
        if self._moveit_node is None:
            return False
        traj_topic = self._select_traj_topic()
        action_name = self._traj_action_target(traj_topic) or "/joint_trajectory_controller/follow_joint_trajectory"
        if not action_name:
            return False
        if self._traj_action_client is None or self._traj_action_name != action_name:
            try:
                self._traj_action_client = ActionClient(self._moveit_node, FollowJointTrajectory, action_name)
                self._traj_action_name = action_name
            except Exception:
                self._traj_action_client = None
                self._traj_action_name = ""
                return False
        client = self._traj_action_client
        if client is None:
            return False
        try:
            return client.wait_for_server(timeout_sec=0.2)
        except Exception:
            return False

    def _moveit_bridge_detected(self) -> bool:
        if self._proc_alive(self.moveit_bridge_proc):
            return True
        if not self._ros_worker_started or not self.ros_worker.node_ready():
            return False
        return (
            self.ros_worker.topic_has_subscribers(MOVEIT_POSE_TOPIC)
            or self.ros_worker.topic_has_subscribers("/grasp_pose")
        )

    def _move_group_ready(self) -> bool:
        return self._moveit_action_ready() and self._moveit_status_ready() and self._follow_joint_traj_ready()

    def _moveit_ready(self) -> bool:
        if self._moveit_bridge_detected():
            return True
        return self._move_group_ready()

    def _update_moveit_status_label(self) -> None:
        if self.lbl_moveit_status is not None:
            state_label = self._moveit_state.value
            self.lbl_moveit_status.setText(f"MoveIt ({state_label})")
            reason = self._moveit_state_reason or state_label
            self.lbl_moveit_status.setToolTip(f"{state_label}: {reason}")
        if self.lbl_moveit_bridge_status is not None:
            bridge_label = "ON" if self._moveit_bridge_running else "OFF"
            self.lbl_moveit_bridge_status.setText(f"MoveIt bridge ({bridge_label})")

    def _update_system_stats(self):
        """Actualizar labels de CPU/RAM/Load, tolerando ausencia de psutil."""
        cpu_txt = "CPU  --"
        ram_txt = "RAM  --"
        load_txt = "Load  --"
        cpu_alert = False
        ram_alert = False
        load_alert = False
        stale_count = 0
        cores = max(1, os.cpu_count() or 1)
        try:
            if psutil:
                cpu = psutil.cpu_percent(interval=None)
                vm = psutil.virtual_memory()
                used_gb = vm.used / (1024 ** 3)
                total_gb = vm.total / (1024 ** 3)
                ram_txt = f"RAM  {used_gb:.1f}/{total_gb:.1f} GB ({vm.percent:.0f}%)"
                cpu_txt = f"CPU  {cpu:.0f}%"
                cpu_alert = cpu >= 85
                ram_alert = vm.percent >= 90
            else:
                # Fallback simple usando loadavg
                load1, load5, load15 = os.getloadavg()
                cores = max(1, os.cpu_count() or 1)
                cpu_txt = f"CPU  {load1 / cores * 100:.0f}%"
        except Exception:
            pass
        try:
            load1, load5, load15 = os.getloadavg()
            load_txt = f"Load  {load1:.2f} {load5:.2f} {load15:.2f}"
            if not psutil:
                cores = max(1, os.cpu_count() or 1)
            load_alert = load1 >= max(4.0, cores * 1.5)
        except Exception:
            pass
        try:
            stale_count, _stale_hint = self._detect_stale_processes()
        except Exception:
            stale_count = 0
        health_alert = stale_count > 0
        health_txt = "Proc.Zombis activos" if health_alert else "Todo OK"
        self._set_stat_label(self.sys_cpu_lbl, cpu_txt, cpu_alert)
        self._set_stat_label(self.sys_ram_lbl, ram_txt, ram_alert)
        self._set_stat_label(self.sys_load_lbl, load_txt, load_alert)
        self._set_stat_label(self.sys_health_lbl, health_txt, health_alert)

    def _set_stat_label(self, label: QLabel, text: str, alert: bool):
        color = "#dc2626" if alert else "#0f172a"
        label.setStyleSheet(f"font-size:11px; color:{color};")
        label.setText(text)

    def _known_process_pids(self) -> Set[int]:
        pids = {os.getpid()}
        if psutil:
            try:
                for parent in psutil.Process(os.getpid()).parents():
                    pids.add(parent.pid)
            except Exception:
                pass
        for proc in (
            self.gz_proc,
            self.bridge_proc,
            self.bag_proc,
            self.moveit_proc,
            self.moveit_bridge_proc,
            self.release_service_proc,
            self.rsp_proc,
        ):
            if proc is None:
                continue
            try:
                if proc.pid:
                    pids.add(proc.pid)
            except Exception:
                continue
        return pids

    def _list_stale_processes(self) -> List[Tuple[int, str, str]]:
        """Listar procesos del proyecto que no pertenecen al panel actual."""
        if not psutil:
            return []
        ws_dir = os.path.realpath(self.ws_dir)
        grace_sec = max(0.0, STALE_PROCESS_GRACE_SEC)
        cutoff = self._panel_start_ts - grace_sec
        ignore_pids = self._known_process_pids()
        patterns = (
            "ur5_qt_panel",
            "ur5_tools",
            "ur5_moveit_bridge",
            "release_objects_service",
            "ur5_moveit_config",
            "gz-transport-topic",
            "ros_gz_bridge",
            "parameter_bridge",
            "gz sim",
            "gzserver",
            "gzclient",
            "ign gazebo",
            "robot_state_publisher",
            "controller_manager",
            "spawner",
            "move_group",
        )
        stale = []
        for proc in psutil.process_iter(["pid", "cmdline", "name", "status"]):
            try:
                pid = proc.info["pid"]
                if pid in ignore_pids:
                    continue
                try:
                    if proc.create_time() >= cutoff:
                        continue
                except Exception:
                    pass
                cmdline = proc.info.get("cmdline") or []
                cmd = " ".join(cmdline) if cmdline else (proc.info.get("name") or "")
                cmd = cmd.strip()
                if not cmd:
                    continue
                cmd_lower = cmd.lower()
                if ws_dir in cmd:
                    stale.append((pid, cmd, proc.info.get("status") or ""))
                    continue
                if any(pat in cmd_lower for pat in patterns):
                    stale.append((pid, cmd, proc.info.get("status") or ""))
                    continue
                if proc.info.get("status") == psutil.STATUS_ZOMBIE and "ros2" in cmd_lower:
                    stale.append((pid, cmd, proc.info.get("status") or ""))
            except Exception:
                continue
        return stale

    def _detect_stale_processes(self) -> Tuple[int, str]:
        """Detecta procesos del proyecto que no pertenecen al panel actual."""
        stale = self._list_stale_processes()
        if not stale:
            return 0, ""
        sample_cmd = stale[0][1]
        hint = sample_cmd.split()[0] if sample_cmd else ""
        return len(stale), hint

    @pyqtSlot()
    def _refresh_controls(self):
        if self._closing:
            return
        if self._gz_launching and self._clear_launching_if_timeout("Gazebo", self._gz_launch_start, GZ_LAUNCH_TIMEOUT_SEC):
            self._gz_launching = False
            self._gz_launch_start = 0.0
            self._set_launching_style(self.btn_gz_start, False)
        if self._bridge_launching and self._clear_launching_if_timeout("Bridge", self._bridge_launch_start, BRIDGE_LAUNCH_TIMEOUT_SEC):
            self._bridge_launching = False
            self._bridge_launch_start = 0.0
            self._set_launching_style(self.btn_bridge_start, False)
        if self._moveit_launching and self._clear_launching_if_timeout("MoveIt", self._moveit_launch_start, MOVEIT_LAUNCH_TIMEOUT_SEC):
            self._moveit_launching = False
            self._moveit_launch_start = 0.0
            self._set_launching_style(self.btn_moveit_start, False)
        if self._managed_mode:
            if self._external_state_active():
                self._apply_external_system_state()
                self._sync_moveit_from_system_state()
            else:
                self._set_system_state(SystemState.WAITING_GAZEBO, "Esperando /system_state")
        else:
            self._evaluate_system_state()
        ready_basic = self._state_ready_basic()
        ready_vision = self._state_ready_vision()
        ready_moveit = self._state_ready_moveit()
        system_error = self._system_state == SystemState.ERROR
        gz_state = self._gazebo_state()
        gz_active = gz_state != "GAZEBO_OFF"
        bridge_active = self._bridge_running or self._pose_info_active()
        gz_proc_alive = self._proc_alive(self.gz_proc)
        bridge_proc_alive = self._proc_alive(self.bridge_proc)
        bag_proc_alive = self._proc_alive(self.bag_proc)
        moveit_proc_alive = self._proc_alive(self.moveit_proc)
        moveit_bridge_proc_alive = self._proc_alive(self.moveit_bridge_proc)
        if self._managed_mode:
            self.btn_gz_start.setEnabled(False)
            self.btn_gz_stop.setEnabled(False)
        else:
            self.btn_gz_start.setEnabled(not gz_active)
            self.btn_gz_stop.setEnabled(gz_proc_alive)
        if self._gz_launching:
            self._set_launching_style(self.btn_gz_start, True)
            self.btn_gz_start.setEnabled(False)
        else:
            self._set_launching_style(self.btn_gz_start, False)
        self.btn_debug_joints.setEnabled(gz_active)
        # Bridge controls siguen el estado de Gazebo para mantenerlos grises hasta que haya simulación
        bridge_enabled = gz_active
        self.bridge_presets.setEnabled(bridge_enabled and not bridge_active)
        self.bridge_edit.setEnabled(bridge_enabled and not bridge_active)
        if self._managed_mode:
            self.btn_bridge_browse.setEnabled(False)
            self.btn_bridge_start.setEnabled(False)
            self.btn_bridge_stop.setEnabled(False)
        else:
            self.btn_bridge_browse.setEnabled(bridge_enabled and not bridge_active)
            self.btn_bridge_start.setEnabled(bridge_enabled and not bridge_active)
            self.btn_bridge_stop.setEnabled(bridge_proc_alive)
        if self._bridge_launching:
            self._set_launching_style(self.btn_bridge_start, True)
            self.btn_bridge_start.setEnabled(False)
        else:
            self._set_launching_style(self.btn_bridge_start, False)
        # Bag depende de que Gazebo y bridge estén arriba
        bag_enabled = gz_active and bridge_active
        self.bag_name.setEnabled(bag_enabled and not bag_proc_alive)
        self.bag_topics.setEnabled(bag_enabled and not bag_proc_alive)
        self.btn_bag_start.setEnabled(bag_enabled and not bag_proc_alive)
        self.btn_bag_stop.setEnabled(bag_proc_alive)
        # MoveIt controls
        moveit_running = self._moveit_state != MoveItState.OFF
        if self._managed_mode:
            self.btn_moveit_start.setEnabled(False)
            self.btn_moveit_stop.setEnabled(False)
        else:
            self.btn_moveit_start.setEnabled(not self._moveit_ready())
            self.btn_moveit_stop.setEnabled(moveit_proc_alive)
        if self._moveit_launching:
            self._set_launching_style(self.btn_moveit_start, True)
            self.btn_moveit_start.setEnabled(False)
        else:
            self._set_launching_style(self.btn_moveit_start, False)
        moveit_bridge_enabled = ready_basic and self._tf_ready_state
        if self._managed_mode:
            self.btn_moveit_bridge_start.setEnabled(False)
            self.btn_moveit_bridge_stop.setEnabled(False)
        else:
            self.btn_moveit_bridge_start.setEnabled(moveit_bridge_enabled and not self._moveit_bridge_detected())
            self.btn_moveit_bridge_stop.setEnabled(moveit_bridge_proc_alive)
        
        # Habilitar/deshabilitar controles dependiendo del estado del bridge
        # Cámara: habilitada solo cuando el bridge está activo
        camera_enabled = bridge_active
        self.camera_topic_combo.setEnabled(camera_enabled)
        self.btn_camera_refresh.setEnabled(camera_enabled)
        self.btn_camera_connect.setEnabled(camera_enabled)
        self.btn_calibrate.setEnabled(
            camera_enabled
            and not system_error
            and self._objects_settled
            and self._camera_stream_ok
            and self._pose_info_ok
            and self._tf_ready_state
        )
        
        # Control manual: solo Gazebo READY + controladores activos
        manual_enabled = self._manual_control_ready() and not self._script_motion_active
        basic_reason = self._system_state_reason or self._system_state.value
        if not self._controllers_ok:
            basic_reason = self._controllers_reason or "controladores no listos"
        self._set_btn_state(
            self.btn_send_joints,
            manual_enabled,
            f"Bloqueado: {basic_reason}" if not manual_enabled else "Mover articulaciones",
        )
        self.joint_time.setEnabled(manual_enabled)
        self.chk_auto_joints.setEnabled(manual_enabled)
        for slider in self.joint_sliders:
            slider.setEnabled(manual_enabled)

        # Botones de movimiento: control directo (sin MoveIt)
        motion_enabled = self._manual_control_ready() and not self._script_motion_active
        motion_tip = "" if motion_enabled else f"Bloqueado: {basic_reason}"
        self._set_btn_state(self.btn_home, motion_enabled, motion_tip)
        self._set_btn_state(self.btn_table, motion_enabled, motion_tip)
        self._set_btn_state(self.btn_basket, motion_enabled, motion_tip)
        self._set_btn_state(self.btn_gripper, motion_enabled, motion_tip)
        self._schedule_controller_check()
        moveit_ready = self._moveit_ready()
        pick_enabled = self._moveit_control_ready() and bool(self._ee_frame_effective)
        pick_tip = "Requiere MoveIt, cámara y objetos estables"
        self._set_btn_state(self.btn_pick_demo, pick_enabled, pick_tip)
        # RELAXED GATING FOR DEMO PICK
        pick_ui_enabled = self._pose_info_ok
        if hasattr(self, "obj_panel"):
            self.obj_panel.setEnabled(pick_ui_enabled)
        if hasattr(self, "camera_view"):
            # RELAXED GATING FOR DEMO PICK
            self.camera_view.setEnabled(self._camera_stream_ok)
        if ready_basic and motion_enabled and self._controllers_ok and not pick_enabled:
            reason = "TF world->base_link no disponible"
            if not moveit_ready:
                reason = "MoveIt no listo"
            if not self._pose_info_ok:
                reason = "pose/info no disponible"
            if not self._ee_frame_effective:
                reason = "EE frame no disponible"
            if reason != self._pick_block_reason:
                self._set_status(f"PICK en espera: {reason}", error=False)
                self._emit_log(f"[PICK] Bloqueado: {reason}")
                self._pick_block_reason = reason
        else:
            self._pick_block_reason = None
        
        # World selector: deshabilitado cuando Gazebo está corriendo (no se puede cambiar)
        self.world_combo.setEnabled(not gz_active)
        self.mode_combo.setEnabled(not gz_active)
        self.btn_world_browse.setEnabled(not gz_active)

    def _schedule_controller_check(self) -> None:
        if not self._bridge_running:
            return
        now = time.time()
        if self._controller_check_inflight or (now - self._last_controller_check) < CONTROLLER_CHECK_INTERVAL_SEC:
            return
        self._controller_check_inflight = True

        def worker():
            ok, reason = self._controllers_ready()
            changed = (ok != self._controllers_ok) or (reason != self._controllers_reason)
            now = time.time()
            grace_base = self._controller_spawn_last_start or self._bridge_start_ts or 0.0
            in_grace = grace_base and (now - grace_base) < CONTROLLER_START_GRACE_SEC
            if ok:
                if self._controllers_state != "READY":
                    self._emit_log("[CTRL] state=READY")
                    self._controllers_state = "READY"
                self._controllers_ok = True
                self._controllers_reason = reason
            else:
                if in_grace:
                    if self._controllers_state != "STARTING":
                        self._emit_log("[CTRL] state=STARTING")
                        self._controllers_state = "STARTING"
                    # Mantener controllers_ok=false pero sin degradar el estado global.
                    self._controllers_ok = False
                    self._controllers_reason = reason
                else:
                    if self._controllers_state != "ERROR":
                        self._emit_log("[CTRL] state=ERROR")
                        self._controllers_state = "ERROR"
                    self._controllers_ok = False
                    self._controllers_reason = reason
            self._last_controller_check = time.time()
            self._controller_check_inflight = False
            if changed:
                if ok:
                    self._emit_log("[CTRL] controllers_ready=true reason=all_required_active")
                else:
                    if in_grace:
                        self._emit_log(f"[CTRL] controllers_ready=false reason=starting ({reason})")
                    else:
                        self._emit_log(f"[CTRL] controllers_ready=false reason={reason}")
                        self._log_warning(f"[PICK] controladores no listos ({reason})")
            self.signal_controllers_ready.emit(ok)
            self.signal_refresh_controls.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _update_ui_state(self):
        """Inicializar UI: arranque manual desde los controles del panel."""
        self._gz_running = False
        self._bridge_running = False
        self._bag_running = False
        self._moveit_running = False
        self._moveit_bridge_running = False
        self._auto_joint2_move_done = False
        
        # Gazebo controls
        self.btn_gz_start.setEnabled(True)
        self.btn_gz_stop.setEnabled(False)
        self.btn_debug_joints.setEnabled(False)
        self.btn_debug_joints.setChecked(False)
        self.world_combo.setEnabled(True)
        self.mode_combo.setEnabled(True)
        self.btn_world_browse.setEnabled(True)
        
        # Bridge controls
        self.btn_bridge_start.setEnabled(False)
        self.btn_bridge_stop.setEnabled(False)
        self.bridge_presets.setEnabled(False)
        self.bridge_edit.setEnabled(False)
        self.btn_bridge_browse.setEnabled(False)
        
        # Bag controls
        self.btn_bag_start.setEnabled(False)
        self.btn_bag_stop.setEnabled(False)
        self.bag_name.setEnabled(False)
        self.bag_topics.setEnabled(False)

        # MoveIt controls
        self.btn_moveit_start.setEnabled(True)
        self.btn_moveit_stop.setEnabled(False)
        self.btn_moveit_bridge_start.setEnabled(False)
        self.btn_moveit_bridge_stop.setEnabled(False)
        
        # Cámara (deshabilitada hasta que bridge esté activo)
        self.camera_topic_combo.setEnabled(False)
        self.btn_camera_refresh.setEnabled(False)
        self.btn_camera_connect.setEnabled(False)
        self.btn_calibrate.setEnabled(False)
        
        # Control manual (deshabilitado hasta que bridge esté activo)
        self.btn_send_joints.setEnabled(False)
        self.joint_time.setEnabled(False)
        self.chk_auto_joints.setEnabled(False)
        for slider in self.joint_sliders:
            slider.setEnabled(False)

        # Botones de movimiento (bloqueados hasta bridge)
        self.btn_home.setEnabled(False)
        self.btn_table.setEnabled(False)
        self.btn_basket.setEnabled(False)
        self.btn_gripper.setEnabled(False)
        self.btn_pick_demo.setEnabled(False)
        
        # Debug y otros
        self.btn_debug_joints.setEnabled(True)
        self.btn_debug_logs.setEnabled(True)
        self.btn_kill_hard.setEnabled(True)
        self.btn_close_terminal.setEnabled(True)

    def _effective_mode(self) -> str:
        m = self.mode_combo.currentText().strip().lower()
        if m.startswith("gui"):
            return "gui"
        if m.startswith("auto"):
            # En remoto fuerza headless; en local permite GUI si hay DISPLAY.
            if os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY"):
                return "headless"
            if os.environ.get("DISPLAY"):
                return "gui"
            return "headless"
        return "headless"

    def _apply_home_joint2_offset(self, retries: int = 2):
        """Mover a HOME ajustando joint2 un -20% (sentido negativo) al lanzar Gazebo."""
        if self._auto_joint2_move_done:
            return
        self._schedule_controller_check()
        if not self._bridge_running:
            if retries > 0:
                self._schedule_home_offset_retry(1500, retries - 1)
            return
        if not self._gz_running:
            if retries > 0:
                self._schedule_home_offset_retry(1500, retries - 1)
            return

        if not self._controllers_ok:
            if retries > 0:
                self._log("[AUTO] Controller manager no listo, reintentando en 2s")
                self._schedule_home_offset_retry(2000, retries - 1)
            else:
                self._log_warning("[AUTO] Controller manager no disponible (ajuste automático cancelado)")
            return

        home = load_home_pose()
        if len(home) < 6:
            self._log_warning("[AUTO] HOME inválido, no se aplica offset joint2")
            return

        target = list(home[:6])
        base = abs(target[1])
        if base < 1e-3:
            base = 0.25  # fallback pequeño si HOME era ~0
        offset = base * 0.20
        target[1] = -offset

        def worker():
            self._log(f"[AUTO] Ajuste joint2=-20% desde HOME -> {target[1]:.3f} rad")
            ok, info = self._publish_joint_trajectory(target, 3.0)
            if ok:
                self._auto_joint2_move_done = True
                self._ui_set_status("AUTO: joint2 ajustado (-20% HOME)")
            else:
                self._log_warning(f"[AUTO] Falló mover joint2 (-20%): {info}")
                if retries > 0:
                    self.signal_schedule_home_offset.emit(2000, retries - 1)

        threading.Thread(target=worker, daemon=True).start()

    @pyqtSlot(int, int)
    def _schedule_home_offset_retry(self, delay_ms: int, retries: int) -> None:
        QTimer.singleShot(delay_ms, lambda: self._apply_home_joint2_offset(retries=retries))

    def _go_home(self):
        self._log_button("Go HOME")
        if not self._require_manual_ready("HOME"):
            return
        self._log("[ROBOT] Iniciando movimiento a HOME")
        self._set_status("Moviendo a HOME…")
        self._set_motion_lock(True)
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        ok, info = self._publish_joint_trajectory(JOINT_HOME_POSE_RAD, move_sec)
        if ok:
            self._set_status("HOME ejecutado (JointTrajectory)")
            self._log(f"[ROBOT] HOME: JointTrajectory en {info}")
        else:
            self._set_status(f"HOME falló: {info}", error=True)
            self._log_warning(f"[ROBOT] HOME falló: {info}")
        self._set_motion_lock(False)

    def _go_table(self):
        self._log_button("Go Mesa")
        if not self._require_manual_ready("Mesa"):
            return
        self._log("[ROBOT] Iniciando movimiento a Mesa")
        self._set_status("Moviendo a Mesa…")
        self._set_motion_lock(True)
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        ok, info = self._publish_joint_trajectory(JOINT_TABLE_POSE_RAD, move_sec)
        if ok:
            self._set_status("Mesa ejecutado (JointTrajectory)")
            self._log(f"[ROBOT] Mesa: JointTrajectory en {info}")
        else:
            self._set_status(f"Mesa falló: {info}", error=True)
            self._log_warning(f"[ROBOT] Mesa falló: {info}")
        self._set_motion_lock(False)

    def _go_basket(self):
        self._log_button("Go Cesta")
        if not self._require_manual_ready("Cesta"):
            return
        self._set_status("Moviendo a Cesta…")
        self._set_motion_lock(True)
        move_sec = float(self.joint_time.value()) if self.joint_time else 3.0
        ok, info = self._publish_joint_trajectory(JOINT_BASKET_POSE_RAD, move_sec)
        if ok:
            self._set_status("Cesta ejecutado (JointTrajectory)")
            self._log(f"[ROBOT] Cesta: JointTrajectory en {info}")
        else:
            self._set_status(f"Cesta falló: {info}", error=True)
            self._log_warning(f"[ROBOT] Cesta falló: {info}")
        self._set_motion_lock(False)

    def _toggle_gripper_button(self, checked: bool):
        if not self._require_manual_ready("Gripper"):
            return
        self._gripper_closed = checked
        self.btn_gripper.setText("Abrir gripper" if checked else "Cerrar gripper")
        self._log_button(f"Gripper {'cerrar' if checked else 'abrir'}")
        if self._moveit_node is None:
            self._init_moveit_publisher()
        pub = self._get_gripper_publisher(GRIPPER_CMD_TOPIC)
        if pub is None:
            self._set_status("Gripper: publisher no disponible", error=True)
            self._log_warning("[GRIPPER] Publisher no disponible")
            return
        target = GRIPPER_CLOSED_RAD if checked else GRIPPER_OPEN_RAD
        msg = Float64MultiArray()
        msg.data = [float(target), float(target) * GRIPPER_JOINT2_SIGN]
        pub.publish(msg)
        self._set_status(f"Gripper -> {target:.3f} rad", error=False)
    
    def _on_camera_click(self, px: int, py: int):
        """Manejar click en la imagen de cámara."""
        if hasattr(self, "camera_view") and not self.camera_view.isEnabled():
            return
        if not self._pick_ui_allowed():
            return
        # Prioridad 1: Si está calibrando, manejar calibración
        if self._calibrating:
            self._handle_calibration_click(px, py)
            return
        
        # Prioridad 2: Si hay calibración válida, seleccionar objeto
        self._handle_object_selection_click(px, py)

    def _load_table_calibration(self):
        """Cargar calibración de tabla desde archivo (IGUAL A PANEL ONLY)."""
        from .panel_utils import load_table_calib, TABLE_CALIB_PATH
        
        self._emit_log("[CALIB] Intentando cargar calibración...")
        try:
            calib = load_table_calib()
            if calib:
                # Determinar tipo de calibración
                if isinstance(calib, dict):
                    msg = f"[CALIB] Calibración RECT cargada desde {TABLE_CALIB_PATH}"
                elif isinstance(calib, list):
                    if len(calib) == 3 and all(len(row) == 3 for row in calib):
                        msg = f"[CALIB] Calibración HOMOGRAFÍA cargada desde {TABLE_CALIB_PATH}"
                    else:
                        msg = f"[CALIB] Calibración AFINE cargada desde {TABLE_CALIB_PATH}"
                self._emit_log(msg)
                self._log(msg)
                self._set_status("✅ Calibración cargada", error=False)
            else:
                if os.path.isfile(TABLE_CALIB_PATH):
                    msg = f"[CALIB] ⚠️ Archivo calibración inválido: {TABLE_CALIB_PATH}"
                else:
                    msg = f"[CALIB] ℹ️ No hay calibración guardada. Click en 'Calibrar' para crear una."
                self._emit_log(msg)
                self._log(msg)
                self._set_status("Sin calibración - Click 'Calibrar'", error=False)
        except Exception as e:
            msg = f"[CALIB] ERROR cargando calibración: {e}"
            self._emit_log(msg)
            self._log(msg)

    def _refresh_objects_from_gz_async(self):
        threading.Thread(target=self._refresh_objects_from_gz, daemon=True).start()

    def _refresh_objects_from_gz(self):
        """Sincronizar poses de objetos desde Gazebo (igual a Panel Only)."""
        if not gz_sim_status()[0]:
            return
        self._ensure_pose_subscription()

        world_path = self.world_combo.currentText().strip()
        sdf_path = ""
        if world_path and os.path.isfile(world_path):
            sdf_path = world_path
        else:
            cand = os.path.join(WORLDS_DIR, world_path) if world_path else ""
            if cand and os.path.isfile(cand):
                sdf_path = cand
            elif cand and not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
                sdf_path = cand + ".sdf"
            if not sdf_path:
                for c in DEFAULT_WORLD_CANDIDATES:
                    if os.path.isfile(c):
                        sdf_path = c
                        break

        world_name = read_world_name(sdf_path) if sdf_path else GZ_WORLD
        poses = self._read_world_pose_info(world_name)
        if not poses:
            env_base = (
                "export GZ_SIM_RESOURCE_PATH='{}:{}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
                "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; "
                "export GZ_TRANSPORT_IP='{}' ; "
            ).format(
                MODELS_DIR,
                WORLDS_DIR,
                os.environ.get("GZ_TRANSPORT_IP", "127.0.0.1"),
            )
            partitions = []
            part = resolve_gz_partition(self.gz_partition)
            if part:
                partitions.append(part)
            partitions.append("")
            out = ""
            for _ in range(4):
                for p in partitions:
                    env = env_base + (f"export GZ_PARTITION='{p}' ; " if p else "")
                    cmd = (
                        bash_preamble(self.ws_dir)
                        + env
                        + f"gz topic -e -n 1 -t '/world/{world_name}/pose/info' --json-output"
                    )
                    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
                    poses = _parse_pose_json(res.stdout or "")
                    if poses:
                        out = res.stdout
                        break
                if out:
                    break
                time.sleep(0.6)
            if not out:
                self._log("[PICK] Objetos: no se pudo leer poses desde Gazebo")
                return
            poses = _parse_pose_json(out)
            if not poses:
                return
        targets = self._settle_targets()
        if self._objects_settled and targets:
            seen_targets = {pose.get("name") for pose in poses if pose.get("name") in targets}
            if seen_targets != targets:
                self._invalidate_settle("cambio de modelos en Gazebo", restart=True)
        updates = {}
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            name = pose.get("name")
            pos = pose.get("position") or {}
            if not name or not isinstance(pos, dict):
                continue
            if name not in get_object_positions():
                continue
            try:
                x = float(pos.get("x"))
                y = float(pos.get("y"))
                z = float(pos.get("z"))
            except (TypeError, ValueError):
                continue
            updates[name] = (x, y, z)

        updated = bulk_update_object_positions(updates)
        if updated:
            save_object_positions()
            self.signal_update_objects.emit()
            self._log(f"[PICK] Objetos sincronizados desde Gazebo ({updated}).")
        else:
            self._log("[PICK] Objetos: sin cambios desde Gazebo.")

    def _settle_targets(self) -> Set[str]:
        targets = set()
        for name in DYNAMIC_OBJECTS:
            if name in SETTLE_MANUAL:
                targets.add(name)
        return targets

    def _read_world_pose_info(self, world_name: str) -> Optional[List[Dict[str, object]]]:
        if not self._ros_worker_started:
            return None
        poses, ts = self.ros_worker.pose_snapshot()
        if not poses:
            # Fallback: use cached object positions synced from Gazebo.
            fallback = get_object_positions()
            if not fallback:
                return None
            out = []
            for name, (x, y, z) in fallback.items():
                out.append({"name": name, "position": {"x": x, "y": y, "z": z}})
            return out
        age = time.time() - ts if ts else float("inf")
        if age > POSE_INFO_MAX_AGE_SEC:
            return None
        out: List[Dict[str, object]] = []
        for name, (x, y, z) in poses.items():
            out.append(
                {
                    "name": name,
                    "position": {"x": x, "y": y, "z": z},
                }
            )
        return out

    def _start_calibration(self):
        """Iniciar/Desactivar calibración manual mostrando la grilla."""
        self._log_button("Calibrar")
        from .panel_utils import load_table_calib, TABLE_CALIB_PATH

        if not self._objects_settled:
            self._request_settle_snapshot("calibrar")
            self._log_calib_blocked("esperando caída/estabilidad de objetos")
            self._set_status("Esperando caída/estabilización de objetos", error=False)
            return
        if not self._pose_info_ok:
            self._log_calib_blocked("pose/info no disponible")
            self._set_status("Esperando pose/info", error=False)
            return
        if self._camera_required and not self._camera_stream_ok:
            self._log_calib_blocked("cámara no publica")
            self._set_status("Esperando cámara", error=False)
            return
        
        # Si está en modo calibración, desactivar
        if self._calibrating:
            self._calibrating = False
            self._calib_points = []
            self.btn_calibrate.setText("Calibrar")
            self._set_status("Calibración desactivada", error=False)
            self._log("[CALIB] Calibración desactivada")
            return

        # Si ya hay calibración en archivo (igual que Panel Only), no pedir clicks
        try:
            calib = load_table_calib()
            if calib:
                self._calibrating = False
                self._calib_points = []
                self.btn_calibrate.setText("Calibrar")
                self._set_status("✅ Calibración cargada desde archivo - sin clicks", error=False)
                self._log(f"[CALIB] Calibración ya cargada ({TABLE_CALIB_PATH}) - sin interacción")
                self._refresh_objects_from_gz_async()
                return
        except Exception as e:
            self._log(f"[CALIB] Aviso: no se pudo leer calibración guardada ({e}), se ofrece modo manual")
        
        # Iniciar calibración
        if not self._camera_subscribed:
            self._set_status("Conecta la cámara antes de calibrar", error=True)
            return

        self._calibrating = True
        self._calib_points = []
        self._selected_object = None
        self._selected_px = None
        self._selected_world = None
        self.calib_service.start_calibration(self.camera_topic, CalibrationMode.LINEAR_2PT)
        self.btn_calibrate.setText("✓ Calibrar (activo - click para desactivar)")
        self._set_status("CALIBRACIÓN: Click en 4 esquinas de la mesa (arriba-izq, arriba-der, abajo-der, abajo-izq)", error=False)
        self._log("[CALIB] Calibración manual 4 puntos (grid activo)")
    
    def _draw_calib_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar malla y puntos de calibración sobre la imagen."""
        from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
        from PyQt5.QtCore import Qt, QPointF
        
        # Copiar imagen para no modificar original
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        painter.setRenderHint(QPainter.Antialiasing)

        try:
            # Dibujar malla de calibración (grid) - IGUAL A PANEL ONLY
            # Usar coordenadas de mundo (0.025m steps) + tabla_xy_to_pixel para convertir a píxeles
            pen = QPen(QColor(30, 64, 175, 90))
            pen.setWidth(1)
            painter.setPen(pen)
            
            step_x = 0.025  # metros
            step_y = 0.025  # metros
            x_min = TABLE_CENTER_X - (TABLE_SIZE_X / 2.0)
            x_max = TABLE_CENTER_X + (TABLE_SIZE_X / 2.0)
            y_min = TABLE_CENTER_Y - (TABLE_SIZE_Y / 2.0)
            y_max = TABLE_CENTER_Y + (TABLE_SIZE_Y / 2.0)
            
            # Líneas verticales (X constante)
            x = x_min
            while x <= (x_max + 1e-6):
                p0 = table_xy_to_pixel(x, y_min, w, h)
                p1 = table_xy_to_pixel(x, y_max, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                x += step_x
            
            # Líneas horizontales (Y constante)
            y = y_min
            while y <= (y_max + 1e-6):
                p0 = table_xy_to_pixel(x_min, y, w, h)
                p1 = table_xy_to_pixel(x_max, y, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                y += step_y

            # Etiquetas básicas de ejes para visibilidad (mismo estilo Panel Only)
            painter.setPen(QPen(QColor(30, 64, 175, 160)))
            for label_x in (TABLE_CENTER_X - 0.4, TABLE_CENTER_X, TABLE_CENTER_X + 0.4):
                p = table_xy_to_pixel(label_x, y_min, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] + 12, f"x={label_x:.1f}")
            for label_y in (TABLE_CENTER_Y - 0.3, TABLE_CENTER_Y, TABLE_CENTER_Y + 0.3):
                p = table_xy_to_pixel(x_min, label_y, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] - 3, f"y={label_y:.1f}")

            # Dibujar cada punto de calibración
            for i, (px, py) in enumerate(self._calib_points):
                # Cruz roja
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                size = 10
                painter.drawLine(px - size, py, px + size, py)
                painter.drawLine(px, py - size, px, py + size)

                # Círculo exterior
                painter.setPen(QPen(QColor(255, 255, 0), 2))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(QPointF(px, py), 8, 8)

                # Texto con número
                painter.setPen(QPen(QColor(0, 255, 0), 1))
                painter.drawText(QPointF(px + 12, py - 8), f"P{i+1}")

            # Ejes XYZ para orientar la calibración
            def _world_to_pixel(wx: float, wy: float):
                calib = self.calib_service.get_calibration() if hasattr(self, "calib_service") else None
                if not calib or calib.matrix is None:
                    return None
                try:
                    if calib.mode == CalibrationMode.LINEAR_2PT:
                        sx = float(calib.matrix[0, 0])
                        sy = float(calib.matrix[1, 1])
                        tx = float(calib.matrix[0, 2])
                        ty = float(calib.matrix[1, 2])
                        if abs(sx) < 1e-9 or abs(sy) < 1e-9:
                            return None
                        px = (wx - tx) / sx
                        py = (wy - ty) / sy
                        return (px, py)
                    if calib.mode == CalibrationMode.HOMOGRAPHY:
                        inv = np.linalg.inv(calib.matrix)
                        vec = inv @ np.array([wx, wy, 1.0])
                        if abs(vec[2]) < 1e-9:
                            return None
                        return (float(vec[0] / vec[2]), float(vec[1] / vec[2]))
                except Exception:
                    return None
                return None

            def _draw_arrow(p0, p1, color: QColor, label: str):
                if not p0 or not p1:
                    return
                painter.setPen(QPen(color, 3))
                painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                vx = p1[0] - p0[0]
                vy = p1[1] - p0[1]
                norm = math.hypot(vx, vy)
                if norm > 1e-3:
                    ux = vx / norm
                    uy = vy / norm
                    size = 8.0
                    perp = (-uy, ux)
                    a1 = (p1[0] - ux * size + perp[0] * (size / 2.0), p1[1] - uy * size + perp[1] * (size / 2.0))
                    a2 = (p1[0] - ux * size - perp[0] * (size / 2.0), p1[1] - uy * size - perp[1] * (size / 2.0))
                    painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a1[0], a1[1]))
                    painter.drawLine(QPointF(p1[0], p1[1]), QPointF(a2[0], a2[1]))
                painter.setPen(QPen(color, 1))
                painter.drawText(QPointF(p1[0] + 4.0, p1[1] - 4.0), label)

            # Representación compacta de ejes en la esquina superior izquierda
            axis_len_px = max(18.0, min(w, h) * 0.05)
            margin = 18.0
            base = (margin, margin)
            x_tip = (base[0] + axis_len_px, base[1])
            y_tip = (base[0], base[1] - axis_len_px)
            z_tip = (base[0] - axis_len_px * 0.5, base[1] - axis_len_px * 0.7)

            _draw_arrow(base, x_tip, QColor(239, 68, 68), "X+")
            _draw_arrow(base, y_tip, QColor(34, 197, 94), "Y+")
            _draw_arrow(base, z_tip, QColor(59, 130, 246), "Z+")
        finally:
            painter.end()
        return img_copy
    
    def _draw_selection_overlay(self, qimg: QImage, w: int, h: int) -> QImage:
        """Dibujar selección actual sobre la imagen."""
        from PyQt5.QtGui import QPainter, QPen, QColor
        from PyQt5.QtCore import Qt, QPointF
        
        if not self._selected_px:
            return qimg
        
        px, py = self._selected_px
        img_copy = qimg.copy()
        painter = QPainter(img_copy)
        painter.setRenderHint(QPainter.Antialiasing)
        
        color = QColor(34, 197, 94)
        painter.setPen(QPen(color, 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(px, py), 9, 9)
        
        painter.drawLine(px - 10, py, px + 10, py)
        painter.drawLine(px, py - 10, px, py + 10)
        
        # Texto con coordenadas del mundo
        if self._selected_world:
            wx, wy, wz = self._selected_world
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.drawText(px + 22, py, f"({wx:.3f}, {wy:.3f})")
        
        painter.end()
        return img_copy

    def _run_pick_demo(self):
        """Publica una secuencia MoveIt-only para el DEMO de mesa → cesta."""
        self._log_button("PICK MESA → CESTA")
        # RELAXED GATING FOR DEMO PICK
        demo_ready = (
            self._gazebo_state() == "GAZEBO_READY"
            and self._controllers_ok
            and self._tf_ready_state
            and bool(self._ee_frame_effective)
            and self._camera_stream_ok
        )
        if not demo_ready:
            self._set_status("Sistema no listo; esperando pick", error=False)
            self._emit_log(f"[PICK] Bloqueado: estado={self._system_state.value}")
            return
        if not self._moveit_required:
            self._set_status("MoveIt deshabilitado; bloqueando pick", error=True)
            self._emit_log("[PICK] Bloqueado: MoveIt deshabilitado")
            return
        if self._moveit_state != MoveItState.READY:
            reason = self._moveit_state_reason or "MoveIt no listo"
            self._set_status(f"MoveIt no listo; esperando pick ({reason})", error=False)
            self._emit_log(f"[PICK] Bloqueado: MoveIt ({reason})")
            return
        self._emit_log("[PICK] Secuencia MoveIt-only iniciada")
        ready, reason = self._controllers_ready()
        if not ready:
            self._emit_log(f"[PICK] controladores no listos ({reason})")
            self._set_status("Controladores no listos; esperando", error=False)
            return
        self._emit_log("[PICK] target_source=DEMO")
        self._set_status("Pick demo: enviando poses a MoveIt…")
        self._set_motion_lock(True)

        # LEGACY: plan_pick_demo_ik + JointTrajectory han pasado a ser legacy.
        def worker():
            try:
                for label, pose_data, delay in PICK_SEQUENCE:
                    if self._moveit_state != MoveItState.READY:
                        raise RuntimeError("MoveIt no listo durante la secuencia")
                    position = pose_data.get("position", (0.0, 0.0, 0.0))
                    frame_id = pose_data.get("frame", BASE_FRAME or "base_link")
                    self._emit_log(
                        f"[PICK] Pose pick_target (BASE_FRAME): {position} frame_id={frame_id}"
                    )
                    self._publish_moveit_pose(label, pose_data)
                    time.sleep(delay)
                self._ui_set_status("Pick demo publicado (MoveIt maneja la ejecución)")
                self._emit_log("[PICK] Secuencia MoveIt publicada correctamente.")
            except Exception as exc:
                self._ui_set_status(f"Error en pick demo: {exc}", error=True)
                self._emit_log(f"[PICK] ✗ Error: {exc}")
            finally:
                self._set_motion_lock(False)

        threading.Thread(target=worker, daemon=True).start()
    
    def _get_object_world_position(self, obj_name: str) -> Optional[tuple]:
        """Obtener posición mundial del objeto desde poses actuales."""
        try:
            positions = get_object_positions()
            if positions and obj_name in positions:
                pos = positions[obj_name]
                # pos es una tupla (x, y, z)
                x, y, z = pos
                return (float(x), float(y), float(z))
        except Exception as e:
            self._log(f"[PICK] Error obteniendo posición: {e}")
        return None

    def _on_slider_change(self, idx: int, value: int):
        # Ignorar cambios si vienen de _on_joint_state (flag)
        if self._updating_sliders_from_joint_state:
            return
        
        deg = float(value) / JOINT_SLIDER_SCALE
        rad = math.radians(deg)
        self.joint_value_labels[idx].setText(f"{deg:.1f} deg / {rad:.3f} rad")
        
        # Bloquear actualizaciones de sliders por 2 segundos después de movimiento manual
        self._slider_update_blocked_until = time.time() + 2.0
        
        # Debug: mostrar cambio incremental cuando se mueve un slider
        if self._debug_logs_enabled:
            joint_name = UR5_JOINT_NAMES[idx]
            old_value = self._last_slider_values.get(idx, value)
            delta_deg = deg - (float(old_value) / JOINT_SLIDER_SCALE)
            direction = "↑" if delta_deg > 0 else "↓" if delta_deg < 0 else "→"
            if abs(delta_deg) > 0.5:  # Solo mostrar cambios significativos
                self._log(f"[SLIDER] {joint_name:25s} {direction} {deg:+7.1f}° (delta: {delta_deg:+.1f}°)")
            self._last_slider_values[idx] = value

    def _slider_to_deg(self, value: int) -> float:
        return float(value) / JOINT_SLIDER_SCALE

    def _current_joint_positions_rad(self):
        positions = [math.radians(self._slider_to_deg(s.value())) for s in self.joint_sliders]
        
        # Debug: mostrar valores de sliders
        if self._debug_logs_enabled:
            slider_values = [s.value() for s in self.joint_sliders]
            deg_values = [self._slider_to_deg(v) for v in slider_values]
            self._log(f"[SLIDER_READ] slider_raw={slider_values}")
            self._log(f"[SLIDER_READ] slider_deg={[f'{d:.1f}' for d in deg_values]}")
        
        return positions
    
    def _handle_calibration_click(self, px: int, py: int):
        """Manejar click durante calibración (IGUAL A PANEL ONLY - SIN DIÁLOGOS)."""
        if not self._calibrating:
            return
        
        # ✅ Simplemente guardar el píxel clickeado
        self._calib_points.append((px, py))
        self._log(f"[CALIB] Punto {len(self._calib_points)}: píxel ({px}, {py})")
        
        # Si tenemos 4 puntos, calcular calibración automáticamente
        if len(self._calib_points) >= 4:
            self._finish_calibration()
        else:
            # Mostrar progreso
            needed = 4 - len(self._calib_points)
            self._set_status(f"CALIBRACIÓN: {len(self._calib_points)}/4 puntos - Click {needed} más", error=False)
    
    def _finish_calibration(self):
        """Finalizar calibración calculando homografía automáticamente (IGUAL A PANEL ONLY)."""
        if len(self._calib_points) < 4:
            self._log("[CALIB] Calibración incompleta")
            return
        
        # ✅ Extraer píxeles clickeados
        p1_px, p1_py = self._calib_points[0]
        p2_px, p2_py = self._calib_points[1]
        p3_px, p3_py = self._calib_points[2]
        p4_px, p4_py = self._calib_points[3]
        
        # ✅ Calcular coordenadas del mundo basadas en la tabla
        # Asumimos que los 4 puntos son las esquinas de la mesa
        cx = TABLE_CENTER_X
        cy = TABLE_CENTER_Y
        sx = TABLE_SIZE_X / 2.0
        sy = TABLE_SIZE_Y / 2.0
        
        # Esquinas en coordenadas del mundo (arriba-izq, arriba-der, abajo-der, abajo-izq)
        w1 = (cx - sx, cy + sy)   # Arriba-izq
        w2 = (cx + sx, cy + sy)   # Arriba-der
        w3 = (cx + sx, cy - sy)   # Abajo-der
        w4 = (cx - sx, cy - sy)   # Abajo-izq
        
        # ✅ Validación mejorada: que los 4 puntos formen un cuadrilátero razonable
        # Chequea distancias entre puntos consecutivos y diagonales
        def dist(p1, p2):
            return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        
        points = [(p1_px, p1_py), (p2_px, p2_py), (p3_px, p3_py), (p4_px, p4_py)]
        
        # Verificar que cada par de puntos adyacentes estén separados >= 20px
        min_distance = 20
        dist_12 = dist(points[0], points[1])
        dist_23 = dist(points[1], points[2])
        dist_34 = dist(points[2], points[3])
        dist_41 = dist(points[3], points[0])
        
        self._log(f"[CALIB] Distancias: P1-P2={dist_12:.1f}px, P2-P3={dist_23:.1f}px, P3-P4={dist_34:.1f}px, P4-P1={dist_41:.1f}px")
        
        if min(dist_12, dist_23, dist_34, dist_41) < min_distance:
            self._log(f"[CALIB] ERROR: puntos muy cercanos (mínimo {min_distance}px). Repite calibración.")
            self._set_status(f"ERROR: puntos muy cercanos ({min_distance}px mín)", error=True)
            self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
            self._set_status("Calibración reiniciada - click en 4 esquinas", error=False)
            return
        
        # ✅ Calcular homografía
        try:
            pixel_points = [(float(p1_px), float(p1_py)), (float(p2_px), float(p2_py)),
                           (float(p3_px), float(p3_py)), (float(p4_px), float(p4_py))]
            world_points = [(float(w1[0]), float(w1[1])), (float(w2[0]), float(w2[1])),
                           (float(w3[0]), float(w3[1])), (float(w4[0]), float(w4[1]))]

            hom = None
            # Intentar usar graspnet si está instalado
            try:
                from graspnet.utils.homography import compute_homography as g_compute
                hom = g_compute(pixel_points, world_points)
            except Exception as e:
                self._log(f"[CALIB] Aviso: graspnet no disponible ({e}), usando OpenCV")
                try:
                    import numpy as np
                    import cv2
                    src = np.array(pixel_points, dtype=np.float32)
                    dst = np.array(world_points, dtype=np.float32)
                    hom, _ = cv2.findHomography(src, dst, method=0)
                    if hom is not None:
                        hom = hom.tolist()
                except Exception as e_cv:
                    raise RuntimeError(f"No se pudo calcular homografía: {e_cv}")

            if not hom:
                self._log("[CALIB] ERROR: homografía inválida. Repite calibración.")
                self._set_status("ERROR: homografía inválida", error=True)
                self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR
                return
            
            # ✅ Enviar calibración al servicio
            self.calib_service.set_homography(hom)
            self._calibrating = False
            self._calib_points = []  # ✅ LIMPIAR PUNTOS DESPUÉS DE ÉXITO
            self.btn_calibrate.setText("Calibrar")
            self._set_status(f"✅ Calibración completada ({dist_12:.0f}-{dist_34:.0f}px)", error=False)
            self._log(f"[CALIB] ✅ Homografía completada y almacenada en servicio")
            self._refresh_objects_from_gz_async()
            
        except Exception as e:
            self._log(f"[CALIB] ERROR calculando homografía: {e}")
            self._set_status(f"ERROR: {e}", error=True)
            self._calib_points = []  # ✅ LIMPIAR PUNTOS EN ERROR

    
    def _handle_object_selection_click(self, px: int, py: int):
        """Manejar click en cámara para seleccionar objeto (igual a Panel Only)."""
        if not self._pick_ui_allowed():
            return
        # Usar homografía global/table map para convertir a mundo
        w = getattr(self.camera_view, "_img_width", 0)
        h = getattr(self.camera_view, "_img_height", 0)
        world_x, world_y = pixel_to_table_xy(px, py, w, h)
        if world_x is None or world_y is None:
            self._log("[PICK] No hay calibración válida (pixel_to_table_xy) - carga table_pixel_map.json o calibra")
            self._set_status("⚠️ Calibra la cámara o carga table_pixel_map.json", error=True)
            return

        obj_name = nearest_table_object(world_x, world_y)
        obj_pos = get_object_position(obj_name) if obj_name else None
        world_z = obj_pos[2] if obj_pos else 0.0

        if obj_pos:
            # Recalcular con la altura del objeto para ajustar la proyección
            wx_adj, wy_adj = pixel_to_table_xy(px, py, w, h, z_target=obj_pos[2]) or (world_x, world_y)
            world_x, world_y = wx_adj, wy_adj
            dx = world_x - obj_pos[0]
            dy = world_y - obj_pos[1]
            snapped = math.hypot(dx, dy) <= SELECTION_SNAP_DIST
            if snapped:
                world_x, world_y = obj_pos[0], obj_pos[1]
                pix = world_xyz_to_pixel(world_x, world_y, obj_pos[2], w, h)
                if not pix:
                    pix = table_xy_to_pixel(world_x, world_y, w, h)
                if pix:
                    px, py = pix
            self._select_object(obj_name, px, py, world_x, world_y, world_z)
            return

        # Click en vacío (no objetos cerca)
        self._log(f"[PICK] Click en vacío: px=({px},{py}) → world=({world_x:.2f},{world_y:.2f})")
        self._selected_object = None
        self._selected_px = (px, py)
        self._selected_world = (world_x, world_y, 0.0)
        self._update_objects()
        self._log_selection_tf(self._selected_world)
    
    def _select_object(self, name: str, px: int, py: int, wx: float, wy: float, wz: float):
        """Seleccionar un objeto (desde click o desde lista)."""
        self._selected_object = name
        self._selected_px = (px, py)
        self._selected_world = (wx, wy, wz)
        self._selection_timestamp = time.time()
        px_label = f"px=({px},{py})" if px >= 0 and py >= 0 else "px=n/a"
        self._log(f"[PICK] Objeto seleccionado: {name} @ {px_label} world=({wx:.2f},{wy:.2f},{wz:.2f})")
        self._set_status(f"✓ Seleccionado: {name} ({wx:.2f},{wy:.2f})", error=False)
        self._update_objects()
        self._log_selection_tf(self._selected_world)

    def _log_selection_tf(self, world_pose: Tuple[float, float, float], frame: str = WORLD_FRAME or "world"):
        """Log selected world point and its TF-transformed base coordinates."""
        if not world_pose:
            return
        world_frame = frame or WORLD_FRAME or "world"
        self._last_selection_frame = world_frame
        self._last_selected_world_pose = (world_pose[0], world_pose[1], world_pose[2], world_frame)
        self._log(
            f"[PICK] selected_world=({world_pose[0]:.3f},{world_pose[1]:.3f},{world_pose[2]:.3f})"
            f" frame={world_frame}"
        )
        self._start_tf_diagnose_async(world_pose, world_frame)
        self._start_pick_tf_resolve(world_pose, world_frame)

    def _start_tf_diagnose_async(self, world_pose: Tuple[float, float, float], world_frame: str) -> None:
        def worker():
            tf_status = diagnose_tf_tree(world_pose, selection_frame=world_frame)
            self._last_tf_status = tf_status
            base_frame_label = tf_status.get("base_frame") or BASE_FRAME or "base"
            world_frame_label = tf_status.get("world_frame") or world_frame
            self._last_selection_frame = world_frame_label
            self._base_frame_effective = base_frame_label
            selected_base = tf_status.get("selected_base")
            if selected_base:
                self._last_selected_base_pose = (
                    selected_base[0],
                    selected_base[1],
                    selected_base[2],
                    base_frame_label,
                )
            else:
                self._last_selected_base_pose = None
            tf_lookup_status = "ok" if tf_status.get("ok") else "fail"
            if tf_status.get("ok"):
                self.signal_trace_ready.emit()
            selected_base = tf_status.get("selected_base")
            if selected_base:
                bx, by, bz = selected_base
                self._last_selected_base_pose = (bx, by, bz, base_frame_label)
            else:
                self._last_selected_base_pose = None
            transform = tf_status.get("transform")
            log_sig_parts = [
                "ok" if tf_status.get("ok") else "fail",
                base_frame_label,
                world_frame_label,
            ]
            if selected_base:
                log_sig_parts.append(
                    f"sel={selected_base[0]:.3f},{selected_base[1]:.3f},{selected_base[2]:.3f}"
                )
            if transform:
                t = transform.transform.translation
                yaw_deg = math.degrees(yaw_from_quaternion(transform.transform.rotation))
                log_sig_parts.append(f"t={t.x:.3f},{t.y:.3f},{t.z:.3f},{yaw_deg:.2f}")
            log_signature = "|".join(log_sig_parts)
            now = time.time()
            log_pick = True
            if tf_status.get("ok"):
                if (
                    log_signature == self._pick_log_last_sig
                    and (now - self._pick_log_last_ts) < PICK_LOG_MIN_INTERVAL_SEC
                ):
                    log_pick = False
                else:
                    self._pick_log_last_sig = log_signature
                    self._pick_log_last_ts = now
            else:
                self._pick_log_last_sig = log_signature
                self._pick_log_last_ts = now
            if log_pick or not tf_status.get("ok"):
                self._log(f"[PICK] TF_DISCOVERY: base={base_frame_label} world={world_frame_label}")
                self._log(
                    f"[PICK] tf_lookup: {tf_lookup_status} "
                    f"world={world_frame_label} base={base_frame_label} err={tf_status.get('error') or 'n/a'}"
                )
                if selected_base:
                    self._log(
                        f"[PICK] selected_base=({bx:.3f},{by:.3f},{bz:.3f}) frame={base_frame_label}"
                    )
                else:
                    self._log(f"[PICK] selected_base=None frame={base_frame_label}")
            if transform:
                if log_pick or not tf_status.get("ok"):
                    t = transform.transform.translation
                    yaw_deg = math.degrees(yaw_from_quaternion(transform.transform.rotation))
                    self._log(
                        f"[PICK] tf_world_to_base: translation=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                        f"yaw={yaw_deg:.2f}° frame={base_frame_label}"
                    )
            tf_topics_list = tf_status.get("tf_topics") or []
            if not tf_topics_list and self._trace_ready:
                tf_topics_list, _ = _list_tf_topics()
            tf_topics_str = ",".join(tf_topics_list) if tf_topics_list else "n/a"
            fallback_reason = tf_status.get("fallback") or "none"
            if self._trace_ready:
                fallback_reason = "none"
            if log_pick or not tf_status.get("ok"):
                self._log(f"[PICK] tf_topics={tf_topics_str} fallback={fallback_reason}")
                debug_tf, debug_err = debug_dump_tf(world_frame_label, base_frame_label)
                if debug_tf:
                    tx, ty, tz = debug_tf["translation"]
                    yaw_deg = math.degrees(debug_tf["yaw"])
                    self._log(
                        f"[PICK] tf_world_to_base: translation=({tx:.3f},{ty:.3f},{tz:.3f}), "
                        f"yaw={yaw_deg:.2f}°"
                    )
                else:
                    self._log(f"[PICK] tf_world_to_base: error={debug_err or 'unknown'}")

        threading.Thread(target=worker, daemon=True).start()

    def _start_pick_tf_resolve(self, world_pose: Tuple[float, float, float], world_frame: str) -> None:
        if self._pick_tf_inflight:
            return
        if not self._moveit_required:
            self._emit_log("[PICK] Bloqueado: MoveIt deshabilitado")
            self._ui_set_status("PICK en espera: MoveIt no activo", error=False)
            return
        if not self._state_ready_moveit():
            reason = self._system_state_reason or self._system_state.value
            self._emit_log(f"[PICK] Bloqueado: estado={self._system_state.value}")
            self._ui_set_status(f"PICK en espera: {reason}", error=False)
            return
        if not self._pose_info_ok:
            self._emit_log("[PICK] Bloqueado: pose/info no disponible")
            self._ui_set_status("PICK en espera: pose/info no disponible", error=False)
            return
        if not self._tf_ready_state:
            self._emit_log("[PICK] Bloqueado: TF world->base_link no disponible")
            self._ui_set_status("PICK en espera: TF world->base_link no disponible", error=False)
            return
        self._pick_tf_inflight = True
        base_frame = self._base_frame_effective or BASE_FRAME or "base_link"

        def worker():
            try:
                coords = None
                start = time.time()
                while (time.time() - start) < PICK_TF_TIMEOUT_SEC:
                    coords, _ = transform_point_to_frame(
                        world_pose,
                        base_frame,
                        source_frame=world_frame,
                        timeout_sec=PICK_TF_RETRY_SEC,
                    )
                    if coords:
                        break
                    time.sleep(PICK_TF_RETRY_SEC)
                if not coords:
                    self._log("[PICK] Bloqueado: TF world->base no disponible")
                    self._ui_set_status("PICK en espera: TF world->base no disponible", error=False)
                    return
                self._last_selected_base_pose = (coords[0], coords[1], coords[2], base_frame)
                pose_data = _make_pose_data(coords, frame=base_frame)
                self._publish_moveit_pose("PICK_CLICK", pose_data)
                self._ui_set_status("PICK click → /desired_grasp publicado", error=False)
            finally:
                self._pick_tf_inflight = False

        threading.Thread(target=worker, daemon=True).start()
    
    def _selection_candidate(self) -> Optional[Dict[str, object]]:
        """Return the last selected object/world tuple if still recent."""
        if not self._selected_object or not self._selected_world:
            return None
        age = time.time() - self._selection_timestamp
        if age > SELECTION_TIMEOUT_SEC:
            self._log(f"[PICK] Selección expiró (age={age:.1f}s) → fallback demo pick")
            return None
        frame = self._last_selection_frame or WORLD_FRAME or "world"
        return {
            "name": self._selected_object,
            "world": tuple(self._selected_world),
            "frame": frame,
            "age": age,
        }

    def _selection_to_base(self, world_pos: Tuple[float, float, float], source_frame: str) -> Optional[Dict[str, object]]:
        """Try transforming the selected point into a base frame (prefers effective frames)."""
        candidates: List[str] = []
        if self._base_frame_effective:
            candidates.append(self._base_frame_effective)
        if BASE_FRAME and BASE_FRAME not in candidates:
            candidates.append(BASE_FRAME)
        for fallback in ("base_link", "base"):
            if fallback not in candidates:
                candidates.append(fallback)
        for base_frame in candidates:
            coords, transform = transform_point_to_frame(world_pos, base_frame, source_frame)
            if coords and transform:
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                yaw = yaw_from_quaternion(transform.transform.rotation)
                return {
                    "coords": coords,
                    "frame": base_frame,
                    "transform": transform,
                    "translation": (tx, ty, tz),
                    "yaw": yaw,
                    "via_tf": True,
                }
        return None

    def _on_object_clicked(self, name: str):
        """Manejar click en objeto de la lista (IGUAL A PANEL ONLY)."""
        if hasattr(self, "obj_panel") and not self.obj_panel.isEnabled():
            return
        if not self._pick_ui_allowed():
            return
        # Obtener posición del objeto
        objects = get_object_positions()
        if name not in objects:
            self._log(f"[PICK] Objeto {name} no encontrado en Gazebo")
            return
        
        x, y, z = objects[name]
        self._log(f"[PICK] Click en objeto: {name} @ world ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # ✅ Convertir a píxel usando table_xy_to_pixel (IGUAL A PANEL ONLY)
        px, py = -1, -1
        # Usar las dimensiones de la última imagen recibida del CameraView
        w = getattr(self.camera_view, "_img_width", 0) if hasattr(self, "camera_view") else 0
        h = getattr(self.camera_view, "_img_height", 0) if hasattr(self, "camera_view") else 0
        
        if not (self._camera_stream_ok and self._pose_info_ok and w > 0 and h > 0):
            self._log("[PICK] Sin cámara/pose_info lista; omitiendo conversión a píxel")
        else:
            # Intentar convertir con coordenadas XYZ primero
            pix = world_xyz_to_pixel(x, y, z, w, h)
            if not pix:
                # Fallback a XY (igual a Panel Only)
                pix = table_xy_to_pixel(x, y, w, h)
            if pix:
                px, py = pix
                self._log(f"[PICK] Conversión XYZ→píxel OK: ({px}, {py})")
            else:
                self._log(f"[PICK] No se pudo convertir XYZ/XY→píxel, usando fallback (0,0)")
        
        self._select_object(name, px, py, x, y, z)
    
    def _update_objects(self):
        """Actualizar lista de objetos usando ObjectListPanel (mismo estilo que panel principal)."""
        if not hasattr(self, "obj_panel"):
            return

        objects = get_object_positions() if self._gz_running else {}
        pickable_map = {}
        pickable_allowed = self._state_ready_moveit()
        if objects:
            for name, (x, y, _z) in objects.items():
                pickable_map[name] = pickable_allowed and not object_out_of_reach(x, y)
        self.obj_panel.update_objects(objects, pickable=pickable_map)

        sel_text = "Selección: -"
        if self._selected_object and self._selected_world:
            wx, wy, _wz = self._selected_world
            sel_text = f"Selección: {self._selected_object} @ ({wx:.2f},{wy:.2f})"
            if not pickable_allowed:
                sel_text = f"{sel_text} · MoveIt no activo"
        self.obj_panel.set_selected(self._selected_object, sel_text)

    def _build_trace_group(self) -> QGroupBox:
        trace_group = QGroupBox("")
        trace_group.setStyleSheet(
            "QGroupBox {"
            "background:#f8fafc;"
            "border:1px solid #94a3b8;"
            "border-radius:8px;"
            "padding:8px;"
            "margin:0;"
            "}"
            "QLabel { color:#0f172a; }"
        )
        trace_layout = QVBoxLayout()
        trace_layout.setContentsMargins(6, 6, 6, 6)
        trace_layout.setSpacing(4)

        self.lbl_trace_frames = QLabel("Frames: world=..., base=..., ee=...")
        self.lbl_trace_frames.setStyleSheet("font-size: 11px; font-weight: 600;")
        trace_layout.addWidget(self.lbl_trace_frames)

        self.trace_table = QTableWidget(2, 8)
        self.trace_table.setHorizontalHeaderLabels(["frame", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.trace_table.setVerticalHeaderLabels(["Object", "TCP"])
        self.trace_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.trace_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.trace_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.trace_table.setSelectionMode(QTableWidget.NoSelection)
        self.trace_table.setStyleSheet(
            "font-size:10px;"
            "background:#ffffff;"
            "border:none;"
            "gridline-color:#e2e8f0;"
        )
        self.trace_table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.trace_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.trace_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.trace_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.trace_table.setFixedHeight(160)
        self.trace_table.setShowGrid(True)
        self.trace_table.setFrameStyle(QTableWidget.Box | QTableWidget.Plain)
        trace_layout.addWidget(self.trace_table)

        error_block = QWidget()
        error_layout = QGridLayout()
        error_layout.setContentsMargins(0, 0, 0, 0)
        error_layout.addWidget(QLabel("<b>Base</b> (m):"), 0, 0)
        self.lbl_trace_error_base = QLabel("--")
        error_layout.addWidget(self.lbl_trace_error_base, 0, 1)
        error_layout.addWidget(QLabel("<b>World</b> (m):"), 1, 0)
        self.lbl_trace_error_world = QLabel("--")
        error_layout.addWidget(self.lbl_trace_error_world, 1, 1)
        error_block.setLayout(error_layout)
        trace_layout.addWidget(error_block)

        tf_block = QWidget()
        tf_layout = QGridLayout()
        tf_layout.setContentsMargins(0, 0, 0, 0)
        tf_layout.addWidget(QLabel("translation:"), 0, 0)
        self.lbl_trace_tf_translation = QLabel("--")
        tf_layout.addWidget(self.lbl_trace_tf_translation, 0, 1)
        tf_layout.addWidget(QLabel("yaw:"), 1, 0)
        self.lbl_trace_tf_yaw = QLabel("--")
        tf_layout.addWidget(self.lbl_trace_tf_yaw, 1, 1)
        tf_block.setLayout(tf_layout)
        trace_layout.addWidget(tf_block)

        control_row = QHBoxLayout()
        self.chk_trace_freeze = QCheckBox("Freeze")
        control_row.addWidget(self.chk_trace_freeze)
        control_row.addStretch(1)
        self.btn_trace_diag = QPushButton("TRACE DIAG ONCE")
        self.btn_trace_diag.clicked.connect(self._run_trace_diag_once)
        self.btn_trace_diag.setStyleSheet("font-size:9px; padding:2px 8px;")
        control_row.addWidget(self.btn_trace_diag)
        self.btn_copy_trace = QPushButton("Copy trace")
        self.btn_copy_trace.clicked.connect(self._copy_trace_text)
        self.btn_copy_trace.setStyleSheet("font-size:9px; padding:2px 8px;")
        control_row.addWidget(self.btn_copy_trace)
        trace_layout.addLayout(control_row)

        trace_group.setLayout(trace_layout)
        return trace_group

    def _start_trace_timer(self):
        if self._trace_timer:
            return
        self._trace_timer = QTimer(self)
        self._trace_timer.setInterval(200)
        self._trace_timer.timeout.connect(self._refresh_trace_data)
        self._trace_timer.start()
        self._refresh_trace_data()

    def _resolve_trace_frames(self, world_frame: str) -> Tuple[str, Optional[str]]:
        detected_base, detected_ee = discover_base_and_ee_frames(world_frame)
        effective_base = detected_base or self._base_frame_effective or BASE_FRAME or "base_link"
        if detected_base and detected_base != self._base_frame_effective:
            self._base_frame_effective = detected_base
            effective_base = detected_base
        if not detected_base:
            helper = get_tf_helper()
            preferred_base = _preferred_base_frame(helper, world_frame)
            if preferred_base and preferred_base != effective_base:
                effective_base = preferred_base
                self._base_frame_effective = preferred_base
        if detected_ee and detected_ee != self._ee_frame_effective:
            self._ee_frame_effective = detected_ee
        effective_ee = detected_ee or self._ee_frame_effective
        if not effective_ee:
            helper = get_tf_helper()
            for candidate in ("tool0", "flange"):
                if helper and _can_transform_between(helper, effective_base, candidate, timeout_sec=0.05):
                    effective_ee = candidate
                    self._ee_frame_effective = candidate
                    self._emit_log(f"[TF] EE resolved via {candidate}")
                    break
        return effective_base, effective_ee

    def _refresh_trace_data(self):
        if self._closing or not self.trace_table or not self._bridge_running:
            return
        if self.chk_trace_freeze and self.chk_trace_freeze.isChecked():
            return
        now = time.monotonic()
        world_frame = self._last_selection_frame or WORLD_FRAME or "world"
        base_frame, ee_frame = self._resolve_trace_frames(world_frame)
        frames_text = f"Frames: world={world_frame} base={base_frame} ee={ee_frame or 'unavailable'}"
        if self.lbl_trace_frames:
            self.lbl_trace_frames.setText(frames_text)

        object_world_data = None
        object_base_data = None
        last_world = self._last_selected_world_pose
        if last_world:
            wx, wy, wz, stored_world_frame = last_world
            object_world_data = self._pose_dict((wx, wy, wz), (0.0, 0.0, 0.0, 1.0), world_frame)
            last_base = self._last_selected_base_pose
            if last_base and last_base[3] == base_frame:
                bx, by, bz, _ = last_base
                object_base_data = self._pose_dict((bx, by, bz), (0.0, 0.0, 0.0, 1.0), base_frame)
            else:
                source_frame_for_transform = stored_world_frame or world_frame
                coords = None
                try:
                    coords, _ = transform_point_to_frame(
                        (wx, wy, wz),
                        base_frame,
                        source_frame=source_frame_for_transform,
                        timeout_sec=0.05,
                    )
                except Exception as exc:
                    self._log_trace_transform_warning(f"transform_point_to_frame: {exc}")
                if coords:
                    object_base_data = self._pose_dict(coords, (0.0, 0.0, 0.0, 1.0), base_frame)
                    self._last_selected_base_pose = (coords[0], coords[1], coords[2], base_frame)

        tcp_world_data = None
        tcp_base_data = None
        if ee_frame:
            try:
                tcp_world_data, _ = get_pose(world_frame, ee_frame, timeout_sec=0.05)
                tcp_base_data, _ = get_pose(base_frame, ee_frame, timeout_sec=0.05)
            except Exception as exc:
                self._log_trace_transform_warning(f"get_pose: {exc}")
        else:
            if now - self._last_ee_warn_ts >= self._ee_warn_period:
                self._log("[TRACE] EE frame unavailable (retrying)")
                self._last_ee_warn_ts = now
            if self._tf_ready_state and now - self._last_ee_diag_ts >= self._ee_warn_period:
                helper = get_tf_helper()
                frames = helper.list_frames() if helper else set()
                candidates = [
                    f for f in sorted(frames)
                    if any(k in f.lower() for k in ("tool", "tcp", "ee", "flange", "wrist", "rg2", "hand", "ft"))
                ]
                sample = ", ".join(candidates[:8]) if candidates else "-"
                self._log(f"[TF] TF OK pero no hay EE transformable desde base_link. Candidatos={sample}. Bloqueando PICK.")
                self._last_ee_diag_ts = now

        self._set_trace_row(0, object_world_data, object_base_data, world_frame, base_frame)
        self._set_trace_row(1, tcp_world_data, tcp_base_data, world_frame, base_frame)
        self.trace_table.resizeRowsToContents()

        base_error = self._compute_error(object_base_data, tcp_base_data)
        world_error = self._compute_error(object_world_data, tcp_world_data)
        if self.lbl_trace_error_base:
            self.lbl_trace_error_base.setText(self._format_error_text(base_error))
        if self.lbl_trace_error_world:
            self.lbl_trace_error_world.setText(self._format_error_text(world_error))

        tf_transform = self._last_tf_status.get("transform") if self._last_tf_status else None
        if tf_transform and self.lbl_trace_tf_translation and self.lbl_trace_tf_yaw:
            t = tf_transform.transform.translation
            translation_text = f"{t.x:.3f}, {t.y:.3f}, {t.z:.3f}"
            yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
            self.lbl_trace_tf_translation.setText(translation_text)
            self.lbl_trace_tf_yaw.setText(f"{yaw_deg:.2f}°")
        else:
            if self.lbl_trace_tf_translation:
                self.lbl_trace_tf_translation.setText("--")
            if self.lbl_trace_tf_yaw:
                self.lbl_trace_tf_yaw.setText("--")

        self._trace_cached_text = self._build_trace_text(
            world_frame,
            base_frame,
            ee_frame,
            object_world_data,
            object_base_data,
            tcp_world_data,
            tcp_base_data,
            base_error,
            world_error,
            tf_transform,
        )
        self._maybe_log_trace(now)

    def _log_trace_transform_warning(self, message: str) -> None:
        now = time.monotonic()
        key = message.split(":", 1)[0]
        last = self._trace_transform_warn_last.get(key, 0.0)
        self._trace_transform_warn_count[key] = self._trace_transform_warn_count.get(key, 0) + 1
        if (now - last) < self._trace_transform_warn_period:
            return
        count = self._trace_transform_warn_count.get(key, 0)
        self._trace_transform_warn_count[key] = 0
        self._trace_transform_warn_last[key] = now
        self._emit_log(f"[TRACE][WARN] {message} ({count})")

    def _maybe_log_tf_not_ready(self):
        if self._tf_not_ready_logged:
            return
        now = time.monotonic()
        if self._bridge_start_ts and (time.time() - self._bridge_start_ts) < TF_INIT_GRACE_SEC:
            return
        if now - self._tf_ready_last_notice >= 1.0:
            self._log("[TRACE] TF not ready yet (waiting for transforms)")
            self._tf_ready_last_notice = now
            self._tf_not_ready_logged = True

    def _maybe_log_trace(self, now: float):
        if not self._trace_ready:
            return
        if "TF world→base: n/a" in (self._trace_cached_text or ""):
            return
        if now - self._last_trace_print_ts >= self._trace_print_period:
            dt = now - self._last_trace_print_ts
            if not self._trace_debug_logged:
                self._log(
                    f"[TRACE][DEBUG] now={now:.3f} last={self._last_trace_print_ts:.3f} dt={dt:.3f}"
                )
                self._trace_debug_logged = True
            self._last_trace_print_ts = now
            self._log("[TRACE] " + self._trace_cached_text.replace("\n", " | "))

    def _reset_trace_throttle(self, reason: str):
        now = time.monotonic()
        self._last_trace_print_ts = now - self._trace_print_period
        self._last_ee_warn_ts = now - self._ee_warn_period
        self._trace_debug_logged = False
        self._tf_not_ready_logged = False
        self._log(f"[TRACE] throttle reset ({reason})")

    def _run_trace_diag_once(self):
        topic_names: Set[str] = set()
        if self.ros_worker:
            topic_names = set(self.ros_worker.list_topic_names())
        if not topic_names and ROS_AVAILABLE:
            helper_node = get_tf_helper()._node if get_tf_helper() else None
            if helper_node:
                try:
                    topic_names = {name for name, _ in helper_node.get_topic_names_and_types()}
                except Exception:
                    pass
        joint_states_present = "/joint_states" in topic_names
        tf_present = "/tf" in topic_names
        tf_static_present = "/tf_static" in topic_names
        helper = get_tf_helper()
        frames: Set[str] = set()
        if helper:
            for attempt in range(4):
                frames = helper.list_frames()
                if frames:
                    break
                time.sleep(0.5)
            if not frames and helper._buffer:
                _log_tf_yaml_head_once(helper._buffer.all_frames_as_yaml())
        joint_payload: Optional[dict] = None
        if self.ros_worker:
            joint_payload, _ = self.ros_worker.get_last_joint_state()
        joint_received = joint_payload is not None
        names_len = len(joint_payload.get("name", [])) if joint_payload else 0
        position_len = len(joint_payload.get("position", [])) if joint_payload else 0
        robot_frames = [
            frame for frame in sorted(frames) if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
        ]
        sample = ", ".join(robot_frames[:10]) if robot_frames else "-"
        self._log(
            f"[TRACE][DIAG] topics: /joint_states={joint_states_present} /tf={tf_present} /tf_static={tf_static_present}"
        )
        self._log(
            f"[TRACE][DIAG] frames={len(frames)} robot_candidates={len(robot_frames)} sample={sample}"
        )
        self._log(
            f"[TRACE][DIAG] joint_states_msg_received={joint_received} names_len={names_len} position_len={position_len}"
        )
        if helper is None:
            self._log("[TRACE][DIAG] TF helper unavailable for transform checks")
            return

        tf_stats = helper.tf_listener_stats()
        tf_frames_set, tf_static_frames_set = helper.tf_frames_seen()
        combined_robot_frames = [
            frame
            for frame in sorted(tf_frames_set.union(tf_static_frames_set))
            if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
        ]
        tf_frames_sample = ", ".join(sorted(tf_frames_set)[:10]) if tf_frames_set else "-"
        tf_static_sample = ", ".join(sorted(tf_static_frames_set)[:10]) if tf_static_frames_set else "-"
        self._log(
            f"[TRACE][DIAG] tf_listener_msgs tf={tf_stats[0]} tf_static={tf_stats[1]}"
        )
        self._log(
            f"[TRACE][DIAG] tf_frames_seen_count={len(tf_frames_set)} sample={tf_frames_sample}"
        )
        self._log(
            f"[TRACE][DIAG] tf_static_frames_seen_count={len(tf_static_frames_set)} sample={tf_static_sample}"
        )
        self._log(
            f"[TRACE][DIAG] tf_robot_candidates_seen={len(combined_robot_frames)} sample={', '.join(combined_robot_frames[:10]) or '-'}"
        )
        if tf_stats[0] == 0 and not combined_robot_frames:
            self._log(
                "[TRACE][DIAG] No dynamic TF (/tf) received → robot TF missing (check robot_state_publisher / controllers)"
            )

        def diag_transform(label: str, frame_a: str, frame_b: str):
            ok = _can_transform_between(helper, frame_a, frame_b, timeout_sec=0.1)
            self._log(f"[TRACE][DIAG] {label} {frame_a}<->{frame_b} ok={ok}")
            return ok

        base_frame = BASE_FRAME or "base"
        diag_transform("base->base_link", base_frame, "base_link")

        def gather_candidates(substring_predicate):
            return [frame for frame in sorted(frames) if substring_predicate(frame.lower())]

        tool_like = gather_candidates(lambda text: text.endswith("tool0") or "tool0" in text)
        ee_like = gather_candidates(lambda text: "ee" in text and "link" in text)
        wrist_like = gather_candidates(lambda text: "wrist_3" in text)
        self._log(f"[TRACE][DIAG] tool_like={tool_like[:10]}")
        self._log(f"[TRACE][DIAG] ee_like={ee_like[:10]}")
        self._log(f"[TRACE][DIAG] wrist_like={wrist_like[:10]}")

        def diag_candidates(label, candidate_list):
            chosen = None
            for candidate in candidate_list[:3]:
                ok = diag_transform(label, "base_link", candidate)
                if ok and chosen is None:
                    chosen = candidate
            return chosen

        tool_candidate = diag_candidates("base_link-tool", tool_like)
        ee_candidate = diag_candidates("base_link-ee", ee_like)
        wrist_candidate = diag_candidates("base_link-wrist", wrist_like)
        recommended_ee = ee_candidate or tool_candidate or wrist_candidate
        if recommended_ee:
            self._log(f"[TRACE][DIAG] recommended EE candidate: {recommended_ee}")
        else:
            self._log("[TRACE][DIAG] no EE candidate transformable from base_link")

    def _try_mark_tf_ready(self):
        try:
            if self._closing:
                self._stop_tf_ready_timer()
                return
            if not self._bridge_running:
                self._stop_tf_ready_timer()
                return
            if self._trace_ready:
                self._stop_tf_ready_timer()
                return
            prev_state = self._tf_ready_state
            helper = get_tf_helper()
            now = time.monotonic()
            if helper is None:
                if (now - getattr(self, '_last_tf_diag_log', 0.0)) > 1.5:
                    self._emit_log("[TF][DIAG] helper no disponible, tf_ready_state=False")
                    self._last_tf_diag_log = now
                self._tf_ready_state = False
                self._maybe_log_tf_not_ready()
                if prev_state:
                    self.signal_refresh_controls.emit()
                return
            tf_stats = helper.tf_listener_stats()
            if tf_stats[0] == 0 and tf_stats[1] == 0:
                if (now - getattr(self, '_last_tf_diag_log', 0.0)) > 1.5:
                    self._emit_log("[TF][DIAG] No llegan mensajes a /tf o /tf_static. Falta robot_state_publisher o la cadena de publish TF. Bloqueando pick.")
                    self._last_tf_diag_log = now
                self._tf_no_msgs_logged = True
            elif tf_stats[0] > 0 or tf_stats[1] > 0:
                self._tf_no_msgs_logged = False
            world_frame = self._last_selection_frame or WORLD_FRAME or "world"
            final_base = self._wait_for_tf_ready(world_frame, helper)
            if final_base:
                if not self._tf_ready_state:
                    self._log(f"[TRACE] TF ready (base={final_base})")
                self._tf_ready_state = True
                self._tf_not_ready_logged = False
                self._base_frame_effective = final_base
                self._bridge_ready = True
                self.signal_trace_ready.emit()
                if not prev_state:
                    self.signal_tf_ready.emit(True)
                    self.signal_refresh_controls.emit()
                return
            # Diagnóstico EE frame
            frames = helper.list_frames() if helper else set()
            ee_candidates = [f for f in frames if any(k in f.lower() for k in ("tool", "tcp", "ee", "flange", "wrist", "rg2", "hand", "ft"))]
            if not ee_candidates and (now - getattr(self, '_last_tf_diag_log', 0.0)) > 1.5:
                self._emit_log("[TF][DIAG] No hay EE frame transformable desde base_link. Bloqueando PICK.")
                self._last_tf_diag_log = now
            self._tf_ready_state = False
            self._maybe_log_tf_not_ready()
            if prev_state:
                self.signal_tf_ready.emit(False)
                self.signal_refresh_controls.emit()
            elif self._system_error_reason:
                self.signal_refresh_controls.emit()
        except Exception as exc:
            self._log_error(f"[TRACE][ERROR] TF readiness error: {exc}")

    def _start_tf_ready_timer(self):
        if self._tf_ready_timer is None:
            self._tf_ready_timer = QTimer(self)
            self._tf_ready_timer.setInterval(500)
            self._tf_ready_timer.timeout.connect(self._try_mark_tf_ready)
        self._tf_ready_timer.start()

    def _wait_for_tf_ready(self, world_frame: str, helper: Optional["TfHelper"]) -> Optional[str]:
        if helper is None:
            return None
        tf_topics, tf_static = _list_tf_topics()
        if not tf_topics and not tf_static:
            return None
        frames = helper.list_frames()
        if "base_link" in frames and _can_transform_between(helper, "base_link", world_frame, timeout_sec=0.15):
            if self._tf_world_base_valid(helper, "base_link", world_frame):
                return "base_link"
            return None
        base_frame, _ = discover_base_and_ee_frames(world_frame)
        preferred_base = _preferred_base_frame(helper, world_frame, timeout_sec=0.25)
        final_base = preferred_base or base_frame
        if final_base and _can_transform_between(helper, final_base, world_frame, timeout_sec=0.15):
            if self._tf_world_base_valid(helper, final_base, world_frame):
                return final_base
        return None

    def _tf_world_base_valid(self, helper: "TfHelper", base_frame: str, world_frame: str) -> bool:
        transform = helper.lookup_transform(base_frame, world_frame, timeout_sec=0.15)
        if not transform:
            return False
        t = transform.transform.translation
        if abs(t.x) < 1e-3 and abs(t.y) < 1e-3 and abs(t.z) < 1e-3:
            if abs(UR5_BASE_X) > 0.1 or abs(UR5_BASE_Y) > 0.1 or abs(UR5_BASE_Z) > 0.1:
                msg = "[TF][ERROR] world->base es identidad; TF inválido para este mundo"
                if not self._tf_invalid:
                    self._emit_log(msg)
                    self._ui_set_status("TF inválido: world->base es identidad", error=True)
                self._tf_invalid = True
                if self._system_error_reason != msg:
                    self._system_error_reason = msg
                    self._set_system_state(SystemState.ERROR, msg)
                return False
        self._tf_invalid = False
        if self._system_error_reason == "[TF][ERROR] world->base es identidad; TF inválido para este mundo":
            self._system_error_reason = ""
        return True

    def _stop_tf_ready_timer(self):
        if self._tf_ready_timer:
            self._tf_ready_timer.stop()

    def _build_trace_text(
        self,
        world_frame: str,
        base_frame: str,
        ee_frame: Optional[str],
        object_world: Optional[Dict[str, object]],
        object_base: Optional[Dict[str, object]],
        tcp_world: Optional[Dict[str, object]],
        tcp_base: Optional[Dict[str, object]],
        base_error: Optional[Tuple[float, float, float, float]],
        world_error: Optional[Tuple[float, float, float, float]],
        tf_transform: Optional[object],
    ) -> str:
        lines = [
            f"Frames: world={world_frame} base={base_frame} ee={ee_frame or 'n/a'}",
            self._format_pose_summary("Object/world", object_world),
            self._format_pose_summary("Object/base", object_base),
            self._format_pose_summary("TCP/world", tcp_world),
            self._format_pose_summary("TCP/base", tcp_base),
            f"Error base (dx,dy,dz,dist): {self._format_error_tuple(base_error)}",
            f"Error world (dx,dy,dz,dist): {self._format_error_tuple(world_error)}",
        ]
        if tf_transform:
            t = tf_transform.transform.translation
            yaw_deg = math.degrees(yaw_from_quaternion(tf_transform.transform.rotation))
            lines.append(f"TF world→base translation: ({t.x:.3f},{t.y:.3f},{t.z:.3f})")
            lines.append(f"TF world→base yaw: {yaw_deg:.2f}°")
        else:
            lines.append("TF world→base: n/a")
        return "\n".join(lines)

    def _set_trace_row(
        self,
        row: int,
        world_data: Optional[Dict[str, object]],
        base_data: Optional[Dict[str, object]],
        world_frame: str,
        base_frame: str,
    ):
        if not self.trace_table:
            return
        frame_text = f"{world_frame}\n{base_frame}"
        self._set_trace_item(row, 0, frame_text)
        axes = ["x", "y", "z"]
        for idx, axis in enumerate(axes, start=1):
            world_val = self._value_from_pose(world_data, axis)
            base_val = self._value_from_pose(base_data, axis)
            self._set_trace_item(row, idx, self._format_dual_value(world_val, base_val))
        orientation_keys = ["qx", "qy", "qz", "qw"]
        for idx, key in enumerate(orientation_keys, start=4):
            world_val = self._value_from_pose(world_data, key)
            base_val = self._value_from_pose(base_data, key)
            self._set_trace_item(row, idx, self._format_dual_value(world_val, base_val))

    def _value_from_pose(self, data: Optional[Dict[str, object]], key: str) -> Optional[float]:
        if not data:
            return None
        if key in ("x", "y", "z"):
            axis = {"x": 0, "y": 1, "z": 2}[key]
            pos = data.get("position")
            if pos:
                return float(pos[axis])
        else:
            orient = data.get("orientation")
            if orient:
                if key == "qx":
                    return float(orient[0])
                if key == "qy":
                    return float(orient[1])
                if key == "qz":
                    return float(orient[2])
                if key == "qw":
                    return float(orient[3])
        return None

    def _set_trace_item(self, row: int, col: int, text: str):
        if not self.trace_table:
            return
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignCenter)
        item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
        self.trace_table.setItem(row, col, item)

    def _format_dual_value(self, first: Optional[float], second: Optional[float]) -> str:
        def fmt(value: Optional[float]) -> str:
            return f"{value:.3f}" if value is not None else "n/a"
        return f"{fmt(first)}\n{fmt(second)}"

    def _pose_dict(self, position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], frame: str) -> Dict[str, object]:
        return {"frame": frame, "position": position, "orientation": orientation}

    def _compute_error(
        self, source: Optional[Dict[str, object]], target: Optional[Dict[str, object]]
    ) -> Optional[Tuple[float, float, float, float]]:
        if not source or not target:
            return None
        src = source.get("position")
        tgt = target.get("position")
        if not src or not tgt:
            return None
        dx = float(tgt[0]) - float(src[0])
        dy = float(tgt[1]) - float(src[1])
        dz = float(tgt[2]) - float(src[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dx, dy, dz, dist

    def _format_error_text(self, error: Optional[Tuple[float, float, float, float]]) -> str:
        if not error:
            return "n/a"
        dx, dy, dz, dist = error
        return f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f}"

    def _format_error_tuple(self, error: Optional[Tuple[float, float, float, float]]) -> str:
        if not error:
            return "n/a"
        return " ".join(f"{value:.3f}" for value in error)

    def _format_pose_summary(self, label: str, data: Optional[Dict[str, object]]) -> str:
        if not data:
            return f"{label}: n/a"
        pos = data.get("position")
        ori = data.get("orientation")
        if not pos or not ori:
            return f"{label}: n/a"
        return (
            f"{label}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
            f"quat=({ori[0]:.3f},{ori[1]:.3f},{ori[2]:.3f},{ori[3]:.3f})"
        )

    def _copy_trace_text(self):
        if not self._trace_cached_text:
            self._set_status("Trace vacío", error=True)
            return
        QApplication.clipboard().setText(self._trace_cached_text)
        self._set_status("Trace copiado al portapapeles")

    def _step_joint(self, idx: int, direction: int):
        slider = self.joint_sliders[idx]
        slider.setValue(slider.value() + direction)

    def _maybe_send_auto(self):
        if self.chk_auto_joints.isChecked():
            self._send_joints()

    def _send_joints(self):
        self._log_button("Send joints")
        if not self._require_manual_ready("Movimiento manual"):
            return
        if self._manual_inflight:
            self._manual_pending = True
            self._set_status("Movimiento manual en curso…", error=False)
            return

        def worker():
            self._manual_inflight = True
            try:
                if not self._ros_worker_started:
                    self._ensure_ros_worker_started()
                if not self.ros_worker.node_ready():
                    self._ui_set_status("Nodo ROS no listo", error=True)
                    return
                ok, reason = self._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
                if not ok:
                    self._ui_set_status(f"Controladores no listos: {reason}", error=True)
                    return

                topic = self._select_traj_topic()
                if self._debug_logs_enabled:
                    self._log(f"[MANUAL] Topic: {topic}")
                positions = [round(p, 4) for p in self._current_joint_positions_rad()]
                tsec = float(self.joint_time.value())
                sec = max(0.0, tsec)
                sec_i = int(sec)
                nsec_i = int((sec - sec_i) * 1e9)
                
                # Debug: mostrar valores enviados
                if self._debug_logs_enabled:
                    pos_str = ", ".join([f"{p:+.3f}" for p in positions])
                    self._log(f"[MANUAL] Enviando joints: [{pos_str}] (t={sec:.1f}s)")
                
                pub = self._get_traj_publisher(topic)
                if not pub:
                    self._ui_set_status("Publisher JointTrajectory no disponible", error=True)
                    return
                self._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
                traj = JointTrajectory()
                traj.joint_names = list(UR5_JOINT_NAMES)
                try:
                    traj.header.stamp = Time().to_msg()
                except Exception:
                    pass
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start.sec = sec_i
                point.time_from_start.nanosec = nsec_i
                traj.points = [point]
                pub.publish(traj)
                self._ui_set_status(f"Trayectoria enviada a {topic}")
                if self._debug_logs_enabled:
                    self._log("[MANUAL] ✓ Publicación JointTrajectory")
            finally:
                self._manual_inflight = False
                if self._manual_pending:
                    self._manual_pending = False
                    # Emitir señal thread-safe en lugar de QTimer.singleShot()
                    self.retry_send_joints.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _send_joints_retry(self):
        """Retry de _send_joints después de 200ms (thread-safe desde worker)."""
        QTimer.singleShot(200, self._send_joints)

    def closeEvent(self, event):
        if self._shutdown_complete:
            event.accept()
            return
        self._closing = True
        self._emit_log("[TRACE] Shutdown: begin")
        self._bridge_running = False
        self._gz_running = False
        self._log("[TRACE] Shutdown: stopping timers")
        for timer in (
            self._trace_timer,
            self._tf_ready_timer,
            self._pose_debug_timer,
            self._pose_info_timer,
            getattr(self, "objects_timer", None),
            getattr(self, "joint_timer", None),
        ):
            if timer:
                timer.stop()
        self._trace_ready = False
        self._reset_trace_throttle("panel close")
        self._log("[TRACE] Shutdown: stopping RosWorker")
        self.ros_worker.stop_and_join()
        self._log("[TRACE] Shutdown: shutting down TF helper")
        shutdown_tf_helper()
        self._log("[TRACE] Shutdown: TF helper stopped")
        self._kill_proc(self.bag_proc, "ros2 bag record")
        self._kill_proc(self.bridge_proc, "parameter_bridge")
        self._kill_proc(self.release_service_proc, "release_objects_service")
        self._kill_proc(self.world_tf_proc, "world_tf_publisher")
        self._kill_proc(self.rsp_proc, "robot_state_publisher")
        self._kill_proc(self.gz_proc, "gz sim")
        self.bag_proc = None
        self.bridge_proc = None
        self.release_service_proc = None
        self.world_tf_proc = None
        self.rsp_proc = None
        self.gz_proc = None
        self._kill_proc(self.moveit_bridge_proc, "ur5_moveit_bridge")
        self._kill_proc(self.moveit_proc, "move_group")
        self.moveit_bridge_proc = None
        self.moveit_proc = None
        self._force_cleanup_leftovers()
        if self._moveit_node is not None:
            try:
                self._moveit_node.destroy_node()
            except Exception:
                pass
            self._moveit_node = None
            self._moveit_pose_pub = None
        try:
            self._log("[TRACE] Shutdown: calling rclpy.try_shutdown()")
            rclpy.try_shutdown()
        except Exception:
            pass
        self._emit_log("[TRACE] Shutdown: workers stopped")
        self._emit_log("[TRACE] Shutdown: done")
        self._shutdown_complete = True
        super().closeEvent(event)

    def _force_cleanup_leftovers(self) -> None:
        """Forzar cierre de procesos residuales del stack."""
        patterns = (
            "ros2 bag record",
            "ros_gz_bridge",
            "parameter_bridge",
            "gz sim",
            "gz-sim",
            "gzserver",
            "gzclient",
            "ign gazebo",
            "ros2 launch ur5_bringup",
            "ros2_control_node",
            "robot_state_publisher",
            "world_tf_publisher",
            "release_objects_service",
            "system_state_manager",
            "controller_manager",
            "spawner",
            "move_group",
        )
        for sig in ("-TERM", "-KILL"):
            for pat in patterns:
                try:
                    subprocess.run(["pkill", sig, "-f", pat], check=False)
                except Exception:
                    continue
            time.sleep(0.2)
        try:
            res = subprocess.run(
                [
                    "pgrep",
                    "-af",
                    "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|ign gazebo|ros2 launch ur5_bringup|ros2_control_node|robot_state_publisher|world_tf_publisher|controller_manager|spawner|move_group",
                ],
                check=False,
                text=True,
                capture_output=True,
            )
            if res.stdout:
                self._emit_log(f"[WARN] Procesos residuales tras cierre:\n{res.stdout.strip()}")
        except Exception:
            pass


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()


def _rot_to_rpy(rot):
    sy = math.sqrt((rot[0, 0] * rot[0, 0]) + (rot[1, 0] * rot[1, 0]))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def main():
    app = QApplication(sys.argv)
    panel = ControlPanelV2()
    panel.show()
    def _handle_signal(_signum, _frame):
        try:
            QTimer.singleShot(0, panel.close)
        except Exception:
            pass
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _handle_signal)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
