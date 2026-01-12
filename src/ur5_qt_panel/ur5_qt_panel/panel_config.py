#!/usr/bin/env python3
"""Configuración compartida y constantes globales del panel SUPER PRO."""
import os
import sys
from typing import Dict, List, Optional, Tuple

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

WS_DIR = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
os.environ.setdefault("WS_DIR", WS_DIR)
os.environ.setdefault("GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib")
SCRIPTS_DIR = os.path.join(WS_DIR, "scripts")
WORLDS_DIR = os.path.join(WS_DIR, "worlds")
MODELS_DIR = os.path.join(WS_DIR, "models")
LOG_DIR = os.path.join(WS_DIR, "log")
BAGS_DIR = os.path.join(WS_DIR, "bags")
FIG_DIR = os.path.join(WS_DIR, "experiments", "figures_memoria")
VISION_DIR = os.path.expanduser(os.environ.get("VISION_DIR", "~/TFM/agarre_inteligente"))
VISION_EXP_DIR = os.path.join(VISION_DIR, "experiments")
VISION_PLOTS_DIR = os.path.join(VISION_EXP_DIR, "plots")
VISION_SUMMARY = os.path.join(VISION_EXP_DIR, "summary_base.csv")
VISION_FIG_DIR = os.path.join(VISION_EXP_DIR, "figures_memoria")

TABLE_SIZE_X = 0.768
TABLE_SIZE_Y = 0.80
TABLE_CENTER_X = -0.17
TABLE_CENTER_Y = 0.0
TABLE_IMAGE_SWAP_XY = True
TABLE_IMAGE_FLIP_X = True
TABLE_IMAGE_FLIP_Y = True
TABLE_CALIB_PATH = os.path.join(SCRIPTS_DIR, "table_pixel_map.json")
TABLE_PIXEL_AFFINE: Optional[List[List[float]]] = None
TABLE_PIXEL_RECT: Optional[Dict[str, Tuple[float, float]]] = None
TABLE_PIXEL_HOMOGRAPHY: Optional[List[List[float]]] = None
TABLE_CAM_INFO: Optional[Dict[str, object]] = None
TABLE_OBJECT_XY_MARGIN = float(os.environ.get("TABLE_OBJECT_XY_MARGIN", "0.09"))
TABLE_OBJECT_Z_MIN = float(os.environ.get("TABLE_OBJECT_Z_MIN", "0.6"))
TABLE_OBJECT_Z_MAX = float(os.environ.get("TABLE_OBJECT_Z_MAX", "1.55"))
TABLE_OBJECT_WHITELIST: Optional[List[str]] = None
SELECTION_SNAP_DIST = float(os.environ.get("PANEL_SELECTION_SNAP_DIST", "0.0"))
OBJECT_POS_PATH = os.path.join(SCRIPTS_DIR, "object_positions.json")
UR5_BASE_X = -0.85
UR5_BASE_Y = 0.0
UR5_BASE_Z = 0.0
UR5_REACH_RADIUS = 0.85

TABLE_OBJECTS = {
    "cubo_rojo": (-0.02, 0.10, 0.80),
    "cilindro_verde": (-0.27, 0.05, 0.80),
    "caja_azul": (-0.17, -0.15, 0.80),
    "pieza_pick_mesa": (-0.42, 0.00, 0.81),
}
ATTACHABLE_OBJECTS = (
    "cubo_rojo",
    "cilindro_verde",
    "caja_azul",
    "pieza_pick_mesa",
)
EXTRA_OBJECTS = {
    "box_red": (-0.170, 0.000, 1.925),
    "box_blue": (-0.130, 0.000, 1.925),
    "box_green": (-0.210, 0.000, 1.925),
    "cyl_gray": (-0.170, 0.040, 1.925),
    "cyl_orange": (-0.170, -0.040, 1.925),
    "cyl_purple": (-0.120, 0.050, 1.925),
    "box_lightblue": (-0.220, 0.050, 1.925),
    "cyl_green": (-0.120, -0.050, 1.925),
    "box_yellow": (-0.220, -0.050, 1.925),
    "cross_cyan": (-0.090, 0.000, 1.925),
}
OBJECT_POSITIONS = {**TABLE_OBJECTS, **EXTRA_OBJECTS}
DYNAMIC_OBJECTS = set(OBJECT_POSITIONS.keys())
OBJECT_COLORS = {
    "cubo_rojo": "#ef4444",
    "cilindro_verde": "#22c55e",
    "caja_azul": "#3b82f6",
    "pieza_pick_mesa": "#f59e0b",
    "box_red": "#f97316",
    "box_blue": "#a855f7",
    "box_green": "#14b8a6",
    "cyl_gray": "#22c55e",
    "cyl_orange": "#06b6d4",
    "cyl_purple": "#84cc16",
    "box_lightblue": "#93c5fd",
    "cyl_green": "#22c55e",
    "box_yellow": "#10b981",
    "cross_cyan": "#06b6d4",
}

BASKET_DROP = (-1.30, 0.00, 0.82)
GZ_WORLD = "ur5_mesa_objetos"
GRIPPER_ATTACH_PREFIX = "/gripper"
GZ_PARTITION_FILE = os.path.join(LOG_DIR, "gz_partition.txt")

INFER_SCRIPT = os.path.join(VISION_DIR, "scripts", "infer_grasp_rgb.py")
INFER_CKPT = os.path.join(
    VISION_DIR,
    "experiments",
    "EXP1_SIMPLE_RGB_seed0",
    "checkpoints",
    "best.pth",
)
INFER_ROI_SIZE = max(0, int(os.environ.get("INFER_ROI_SIZE", "160")))
INFER_RETRY_ERR_PX = float(os.environ.get("INFER_RETRY_ERR_PX", "60.0"))
FASTRTPS_PROFILES = os.path.join(SCRIPTS_DIR, "fastdds_no_shm.xml")
UR5_CONTROLLERS_YAML = os.path.join(WS_DIR, "src", "ur5_description", "config", "ur5_controllers.yaml")

BRIDGE_BASE_YAML = os.path.join(SCRIPTS_DIR, "bridge_cameras.yaml")
EGL_VENDOR = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
AUTO_START_BRIDGE = bool(int(os.environ.get("PANEL_AUTO_BRIDGE", "1")))
AUTO_START_BRIDGE_DELAY_MS = int(os.environ.get("PANEL_AUTO_BRIDGE_DELAY_MS", "1200"))
AUTO_START_BRIDGE_MAX_RETRIES = int(os.environ.get("PANEL_AUTO_BRIDGE_MAX_RETRIES", "30"))

DEFAULT_WORLD_CANDIDATES = [
    os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"),
]

DEBUG_FRAME_LOG = bool(int(os.environ.get("PANEL_DEBUG_FRAMES", "0")))

BASE_FRAME = os.environ.get("PANEL_BASE_FRAME")
WORLD_FRAME = os.environ.get("PANEL_WORLD_FRAME")

ARM_TRAJ_TOPIC_DEFAULT = os.environ.get("ARM_TRAJ_TOPIC", "/joint_trajectory_controller/joint_trajectory")
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
GRIPPER_JOINT_NAMES = [
    "rg2_finger_joint1",
    "rg2_finger_joint2",
]
UR5_HOME_ENV = os.path.join(SCRIPTS_DIR, "ur5_home_pose.env")
UR5_HOME_DEFAULT = [0.054, 0.028, 0.016, 0.016, 0.028, 0.016]
UR5_MODEL_NAME = os.environ.get("UR5_MODEL_NAME", "ur5_rg2")
JOINT_SLIDER_DEG_MIN = -180.0
JOINT_SLIDER_DEG_MAX = 180.0
JOINT_SLIDER_SCALE = 10.0
DEFAULT_JOINT_MOVE_SEC = 2.0

ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from rosgraph_msgs.msg import Clock
    from cv_bridge import CvBridge
    import numpy as np
    import cv2
    ROS_AVAILABLE = True
except Exception as exc:  # pragma: no cover
    print(f"[WARN] ROS 2 / OpenCV no disponible en el panel: {exc}", file=sys.stderr)
