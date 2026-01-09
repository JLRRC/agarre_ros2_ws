#!/usr/bin/env python3
"""Configuración compartida y constantes globales del panel SUPER PRO."""
import os
import sys
from typing import Dict, List, Optional, Tuple

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

WS_DIR = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
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

TABLE_SIZE_X = 1.20
TABLE_SIZE_Y = 0.80
TABLE_CENTER_X = 0.0
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
    "cubo_rojo": (0.15, 0.10, 0.80),
    "cilindro_verde": (-0.10, 0.05, 0.80),
    "caja_azul": (0.00, -0.15, 0.80),
    "pieza_pick_mesa": (-0.45, 0.00, 0.81),
}
ATTACHABLE_OBJECTS = (
    "cubo_rojo",
    "cilindro_verde",
    "caja_azul",
    "pieza_pick_mesa",
)
EXTRA_OBJECTS = {
    "drop_obj_01_box_cube": (0.000, 0.000, 1.925),
    "drop_obj_02_box_flat": (0.040, 0.000, 1.925),
    "drop_obj_03_box_tall": (-0.040, 0.000, 1.925),
    "drop_obj_04_cyl_long": (0.000, 0.040, 1.925),
    "drop_obj_05_cyl_mid": (0.000, -0.040, 1.925),
    "drop_obj_06_cyl_puck": (0.050, 0.050, 1.925),
    "drop_obj_07_box_lightblue": (-0.050, 0.050, 1.925),
    "drop_obj_08_cyl_green": (0.050, -0.050, 1.925),
    "drop_obj_09_box_bar": (-0.050, -0.050, 1.925),
    "drop_obj_10_cross": (0.080, 0.000, 1.925),
}
OBJECT_POSITIONS = {**TABLE_OBJECTS, **EXTRA_OBJECTS}
DYNAMIC_OBJECTS = set(OBJECT_POSITIONS.keys())
OBJECT_COLORS = {
    "cubo_rojo": "#ef4444",
    "cilindro_verde": "#22c55e",
    "caja_azul": "#3b82f6",
    "pieza_pick_mesa": "#f59e0b",
    "drop_obj_01_box_cube": "#f97316",
    "drop_obj_02_box_flat": "#a855f7",
    "drop_obj_03_box_tall": "#14b8a6",
    "drop_obj_04_cyl_long": "#22c55e",
    "drop_obj_05_cyl_mid": "#06b6d4",
    "drop_obj_06_cyl_puck": "#84cc16",
    "drop_obj_07_box_lightblue": "#93c5fd",
    "drop_obj_08_cyl_green": "#22c55e",
    "drop_obj_09_box_bar": "#10b981",
    "drop_obj_10_cross": "#06b6d4",
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

DEFAULT_WORLD_CANDIDATES = [
    os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"),
]

DEBUG_FRAME_LOG = bool(int(os.environ.get("PANEL_DEBUG_FRAMES", "0")))

BASE_FRAME = os.environ.get("PANEL_BASE_FRAME")
WORLD_FRAME = os.environ.get("PANEL_WORLD_FRAME")

ARM_TRAJ_TOPIC_DEFAULT = os.environ.get("ARM_TRAJ_TOPIC", "/ur5_arm_joint_trajectory")
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
