#!/usr/bin/env python3
import argparse
import math
import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TFProbe(Node):
    def __init__(self, base_frame: str, ee_frame: str) -> None:
        super().__init__("tf_probe")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = base_frame
        self.ee_frame = ee_frame

    def wait_for_transform(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            try:
                if self.tf_buffer.can_transform(
                    self.base_frame,
                    self.ee_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                ):
                    return True
            except TransformException:
                pass
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def sample_pose(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.ee_frame,
            Time(),
            timeout=Duration(seconds=0.2),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        pos = (t.x, t.y, t.z)
        quat = (q.x, q.y, q.z, q.w)
        return pos, quat

    def spin_for(self, dt_sec: float) -> None:
        deadline = time.monotonic() + dt_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def quat_angle(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> float:
    dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
    dot = abs(dot)
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def format_pose(pos: Tuple[float, float, float], quat: Tuple[float, float, float, float]) -> str:
    px, py, pz = pos
    qx, qy, qz, qw = quat
    return f"pos=({px:.6f},{py:.6f},{pz:.6f}) quat=({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f})"


def run_probe(
    node: TFProbe,
    samples: int,
    dt_sec: float,
    min_total_m: float,
    min_step_m: float,
    min_rot_deg: float,
    timeout_sec: float,
) -> bool:
    if not node.wait_for_transform(timeout_sec):
        print(
            f"[TF_PROBE] ERROR: TF no disponible para {node.base_frame} -> {node.ee_frame} (timeout {timeout_sec:.2f}s)",
            file=sys.stderr,
        )
        return False

    samples = max(2, int(samples))
    dt_sec = max(0.01, float(dt_sec))
    poses: List[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = []
    t0 = time.monotonic()
    for idx in range(samples):
        try:
            poses.append(node.sample_pose())
        except TransformException as exc:
            print(f"[TF_PROBE] ERROR: lookup_transform falló en muestra {idx}: {exc}", file=sys.stderr)
            return False
        if idx < samples - 1:
            node.spin_for(dt_sec)

    if len(poses) < 2:
        print("[TF_PROBE] ERROR: muestras insuficientes", file=sys.stderr)
        return False

    total_dist = 0.0
    max_step = 0.0
    max_rot_step = 0.0
    for i in range(1, len(poses)):
        step = dist(poses[i - 1][0], poses[i][0])
        total_dist += step
        max_step = max(max_step, step)
        rot_step = quat_angle(poses[i - 1][1], poses[i][1])
        max_rot_step = max(max_rot_step, rot_step)

    pos_a, quat_a = poses[0]
    pos_b, quat_b = poses[-1]
    total_rot = quat_angle(quat_a, quat_b)
    total_time = max(1e-6, (len(poses) - 1) * dt_sec)
    avg_speed = total_dist / total_time

    # Umbrales físicos para filtrar jitter TF:
    # - 5 mm total: por debajo suele ser ruido numérico/TF.
    # - 2 mm entre muestras: evita falsos positivos por micro-variaciones.
    # - 1 grado: rotaciones menores suelen ser ruido en TF cuando el robot está quieto.
    min_rot_rad = math.radians(min_rot_deg)
    print(f"[TF_PROBE] base={node.base_frame} ee={node.ee_frame}")
    for idx, (pos, quat) in enumerate(poses):
        t_rel = t0 + idx * dt_sec
        print(f"[TF_PROBE] sample {idx:02d} t={t_rel:.3f} {format_pose(pos, quat)}")
    print(f"[TF_PROBE] total_dist={total_dist:.6f} m max_step={max_step:.6f} m avg_speed={avg_speed:.6f} m/s")
    print(
        f"[TF_PROBE] total_rot={math.degrees(total_rot):.3f} deg "
        f"max_rot_step={math.degrees(max_rot_step):.3f} deg"
    )
    print(
        "[TF_PROBE] thresholds: "
        f"total_dist>={min_total_m:.3f} m, "
        f"max_step>={min_step_m:.3f} m, "
        f"rot>={min_rot_deg:.1f} deg"
    )

    translation_moved = (total_dist >= min_total_m) and (max_step >= min_step_m)
    rotation_moved = (total_rot >= min_rot_rad) or (max_rot_step >= min_rot_rad)
    if translation_moved or rotation_moved:
        reason = "translation" if translation_moved else "rotation"
        print(f"[TF_PROBE] decision: movement detected via {reason}")
        print("✅ MOVEIT EJECUTA MOVIMIENTO REAL")
    else:
        print("[TF_PROBE] decision: only TF jitter/noise detected")
        print("❌ MOVEIT NO EJECUTA (SOLO RUIDO / ROBOT INMÓVIL)")
    sys.stdout.flush()
    return translation_moved or rotation_moved


def main() -> None:
    parser = argparse.ArgumentParser(description="TF probe for UR5 (base_link -> tool0)")
    parser.add_argument("--base-frame", type=str, default="base_link")
    parser.add_argument("--ee-frame", type=str, default="tool0")
    parser.add_argument("--samples", type=int, default=12, help="Número de muestras (>=10 recomendado)")
    parser.add_argument("--dt", type=float, default=0.25, help="Tiempo entre muestras (s)")
    parser.add_argument("--min-total-mm", type=float, default=5.0, help="Desplazamiento total mínimo (mm)")
    parser.add_argument("--min-step-mm", type=float, default=2.0, help="Desplazamiento mínimo entre muestras (mm)")
    parser.add_argument("--min-rot-deg", type=float, default=1.0, help="Cambio angular mínimo (deg)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Timeout TF (s)")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = TFProbe(args.base_frame, args.ee_frame)
    try:
        ok = run_probe(
            node,
            samples=args.samples,
            dt_sec=args.dt,
            min_total_m=args.min_total_mm / 1000.0,
            min_step_m=args.min_step_mm / 1000.0,
            min_rot_deg=args.min_rot_deg,
            timeout_sec=args.timeout,
        )
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
