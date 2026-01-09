#!/usr/bin/env python3
"""IK planner focused on the PICK MESA → CESTA demo used by panel_v2."""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence, Tuple

from .panel_config import BASKET_DROP, OBJECT_POSITIONS, UR5_HOME_DEFAULT
from .panel_utils import object_out_of_reach, world_to_base, base_to_world
from .ur5_kinematics import ik_ur5, rot_x, rot_z

LogFn = Callable[[str], None]


def _log(msg: str, fn: Optional[LogFn]) -> None:
    if fn:
        fn(msg)


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _yaw_candidates(base_yaw: float) -> List[float]:
    cands = [
        base_yaw,
        0.0,
        base_yaw + (math.pi / 2.0),
        base_yaw - (math.pi / 2.0),
        base_yaw + (math.pi / 4.0),
        base_yaw - (math.pi / 4.0),
        math.pi,
        -math.pi,
    ]
    out: List[float] = []
    for val in cands:
        normalized = _wrap_angle(val)
        if all(abs(normalized - prev) > 1e-4 for prev in out):
            out.append(normalized)
    return out


def _seed_candidates(seed: Sequence[float]) -> List[List[float]]:
    seeds: List[List[float]] = [list(seed)]
    delta = 0.08
    for idx in range(len(seed)):
        for sign in (-1.0, 1.0):
            cand = list(seed)
            cand[idx] = float(cand[idx]) + (sign * delta)
            seeds.append(cand)
    return seeds


def _call_ik(target: Tuple[float, float, float], rot, seed: Sequence[float]):
    try:
        q_res, err, ok = ik_ur5(target, rot, seed)
        return list(q_res), float(err), bool(ok), ""
    except Exception as exc:  # pragma: no cover
        return None, None, False, str(exc)


def _solve_pose(
    label: str,
    target: Tuple[float, float, float],
    seed: Sequence[float],
    log_fn: Optional[LogFn],
    yaw_base: float = 0.0,
) -> List[float]:
    seeds = _seed_candidates(seed)
    yaws = _yaw_candidates(yaw_base)
    z_offsets = [0.0]
    if label == "pregrasp":
        z_offsets.extend([0.02, 0.04, 0.06, 0.08, 0.10])

    best_reason: Optional[str] = None
    for z_offset in z_offsets:
        target_with_offset = (target[0], target[1], target[2] + z_offset)
        for yaw in yaws:
            rot = rot_z(yaw) @ rot_x(math.pi)
            for candidate in seeds:
                q_sol, err, ok, reason = _call_ik(target_with_offset, rot, candidate)
                if ok and q_sol is not None:
                    _log(f"[PICK] IK resuelto para {label} (yaw={yaw:.2f})", log_fn)
                    return q_sol
                if reason:
                    best_reason = reason

    raise RuntimeError(f"IK falla en {label} ({best_reason or 'sin solución'})")


def _resolve_world_target(obj_name: str, direct_pose: Optional[Sequence[float]], allow_fallback: bool, log_fn: Optional[LogFn]):
    if direct_pose is not None:
        try:
            return tuple(float(v) for v in direct_pose)
        except Exception:
            _log(f"[PICK] Ignorando pose inválida: {direct_pose}", log_fn)
    stored = OBJECT_POSITIONS.get(obj_name)
    if stored:
        return tuple(float(v) for v in stored)

    if allow_fallback:
        for alias in ("pick_demo", "pieza_pick_mesa"):
            candidate = OBJECT_POSITIONS.get(alias)
            if candidate:
                _log(f"[PICK] Usando objeto {alias} como fallback para {obj_name}", log_fn)
                return tuple(float(v) for v in candidate)
    return None


def _normalize_seed(seed: Optional[Sequence[float]]) -> List[float]:
    if seed and len(seed) == 6:
        return [float(v) for v in seed]
    return list(UR5_HOME_DEFAULT)


def plan_pick_demo_ik(
    target_obj: str,
    target_pose: Optional[Sequence[float]],
    q_seed: Optional[Sequence[float]],
    allow_fallback: bool = False,
    log_fn: Optional[LogFn] = None,
    target_frame: str = "world",
) -> List[Tuple[str, List[float]]]:
    """Planifica una secuencia mesa → cesta para un objeto dado."""
    target_world = None
    base_pose: Optional[Tuple[float, float, float]] = None
    if target_frame in ("base_link", "base"):
        if target_pose:
            try:
                base_pose = tuple(float(v) for v in target_pose)
            except Exception:
                base_pose = None
        if base_pose:
            target_world = base_to_world(*base_pose)
        else:
            _log(f"[PICK] target_frame={target_frame} requiere pose base válida", log_fn)
    if target_world is None:
        target_world = _resolve_world_target(target_obj, target_pose, allow_fallback, log_fn)
    if not target_world:
        raise RuntimeError(f"No se encontró la posición mundial para {target_obj}")

    wx, wy, wz = target_world
    if object_out_of_reach(wx, wy):
        if not allow_fallback:
            raise RuntimeError(f"{target_obj} fuera del alcance del UR5")
        fallback = OBJECT_POSITIONS.get("pick_demo") or OBJECT_POSITIONS.get("pieza_pick_mesa")
        if not fallback:
            raise RuntimeError(f"{target_obj} fuera del alcance y no hay fallback válido")
        wx, wy, wz = tuple(float(v) for v in fallback)
        _log(f"[PICK] Objeto fuera del alcance. Reubicando a pick_demo ({wx:.2f},{wy:.2f})", log_fn)
        if object_out_of_reach(wx, wy):
            raise RuntimeError("Fallback pick_demo también queda fuera del alcance")

    if base_pose:
        bx, by, bz = base_pose
    else:
        bx, by, bz = world_to_base(wx, wy, wz)
    basket_bx, basket_by, basket_bz = world_to_base(*BASKET_DROP)

    poses = [
        ("pregrasp", (bx, by, bz + 0.12)),
        ("grasp", (bx, by, bz + 0.02)),
        ("lift", (bx, by, bz + 0.15)),
        ("basket_pre", (basket_bx, basket_by, basket_bz + 0.12)),
        ("basket_drop", (basket_bx, basket_by, basket_bz + 0.03)),
        ("basket_pre_return", (basket_bx, basket_by, basket_bz + 0.12)),
    ]

    plan: List[Tuple[str, List[float]]] = []
    seed = _normalize_seed(q_seed)
    for label, pose in poses:
        _log(f"[PICK] Calculando IK para {label} en {pose}", log_fn)
        q_sol = _solve_pose(label, pose, seed, log_fn)
        plan.append((label, q_sol))
        seed = q_sol

    return plan
