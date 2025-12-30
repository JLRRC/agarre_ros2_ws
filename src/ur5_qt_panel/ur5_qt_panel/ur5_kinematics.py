#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py
# Summary: Minimal UR5 FK/IK (numeric) using standard DH parameters.
from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# Standard UR5 DH parameters (meters, radians).
_A = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
_D = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
_ALPHA = [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0]


def _dh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def fk_ur5(q: Iterable[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Forward kinematics. Returns (position_xyz, rotation_matrix)."""
    q = list(q)
    if len(q) != 6:
        raise ValueError("UR5 FK requiere 6 articulaciones.")
    t = np.eye(4, dtype=float)
    for i in range(6):
        t = t @ _dh(_A[i], _D[i], _ALPHA[i], q[i])
    pos = t[:3, 3].copy()
    rot = t[:3, :3].copy()
    return pos, rot


def ik_ur5(
    target_pos: Iterable[float],
    target_rot: np.ndarray,
    q0: Iterable[float],
    max_iter: int = 200,
    pos_weight: float = 1.0,
    rot_weight: float = 0.5,
) -> Tuple[np.ndarray, float, bool]:
    """Numeric IK for UR5. Returns (q, err_norm, success)."""
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    if target_rot.shape != (3, 3):
        raise ValueError("target_rot debe ser 3x3.")

    def _err(q: np.ndarray) -> np.ndarray:
        pos, rot = fk_ur5(q)
        dp = (pos - target_pos) * pos_weight
        r_err = rot.T @ target_rot
        rotvec = Rotation.from_matrix(r_err).as_rotvec() * rot_weight
        return np.concatenate([dp, rotvec])

    q0 = np.asarray(list(q0), dtype=float)
    if q0.size != 6:
        raise ValueError("q0 debe tener 6 articulaciones.")
    bounds = (-2.0 * math.pi) * np.ones(6), (2.0 * math.pi) * np.ones(6)
    res = least_squares(_err, q0, bounds=bounds, max_nfev=max_iter)
    err_norm = float(np.linalg.norm(res.fun))
    return res.x, err_norm, bool(res.success)


def rot_z(theta: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array([[ct, -st, 0.0], [st, ct, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def rot_x(theta: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array([[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]], dtype=float)

