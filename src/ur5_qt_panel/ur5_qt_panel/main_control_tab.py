#!/usr/bin/env python3

import os
import math
import json
import threading
import time
from typing import Dict, Tuple, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QGroupBox,
    QSizePolicy,
    QSplitter,
    QPushButton,
)

from panel_config import UR5_JOINT_NAMES, GRIPPER_JOINT_NAMES
from panel_utils import (
    RosWorker,
    CmdRunner,
    resolve_gz_partition,
    build_gz_env,
    run_cmd_output,
)
from robot_tab import RobotTab
from ur5_kinematics import fk_ur5


class CameraView(QLabel):
    """Large camera view that auto-scales to the widget size."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._qimg = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setText(title)
        self.setStyleSheet("background:#0b0f14; color:#94a3b8; border:1px solid #1f2937;")

    def set_frame(self, qimg):
        self._qimg = qimg
        self._update_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def _update_pixmap(self):
        if self._qimg is None:
            return
        pix = QPixmap.fromImage(self._qimg)
        pix = pix.scaled(self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(pix)


class JointStatePanel(QWidget):
    """Displays joint positions/velocities and basic FK/Dynamics metrics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._joint_rows: Dict[str, Tuple[QLabel, QLabel]] = {}
        self._gripper_rows: Dict[str, QLabel] = {}
        self._last_stamp = 0.0
        self._last_positions: Dict[str, float] = {}
        self._last_time: float = 0.0

        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self.status_lbl = QLabel("Joint states: esperando /joint_states ...")
        self.status_lbl.setStyleSheet("color:#64748b;")
        lay.addWidget(self.status_lbl)

        g_arm = QGroupBox("DOF UR5 (pos/vel)")
        g_arm.setFlat(True)
        g_arm.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid = QGridLayout()
        grid.setContentsMargins(6, 8, 6, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Posicion"), 0, 1)
        grid.addWidget(QLabel("Velocidad"), 0, 2)

        for row, joint in enumerate(UR5_JOINT_NAMES, start=1):
            name_lbl = QLabel(joint)
            pos_lbl = QLabel("--")
            vel_lbl = QLabel("--")
            self._joint_rows[joint] = (pos_lbl, vel_lbl)
            grid.addWidget(name_lbl, row, 0)
            grid.addWidget(pos_lbl, row, 1)
            grid.addWidget(vel_lbl, row, 2)

        g_arm.setLayout(grid)
        lay.addWidget(g_arm)

        g_grip = QGroupBox("Pinza RG2 (pos)")
        g_grip.setFlat(True)
        g_grip.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grip_grid = QGridLayout()
        grip_grid.setContentsMargins(6, 8, 6, 6)
        grip_grid.setHorizontalSpacing(6)
        grip_grid.setVerticalSpacing(2)
        grip_grid.addWidget(QLabel("Joint"), 0, 0)
        grip_grid.addWidget(QLabel("Apertura"), 0, 1)
        for row, joint in enumerate(GRIPPER_JOINT_NAMES, start=1):
            name_lbl = QLabel(joint)
            pos_lbl = QLabel("--")
            self._gripper_rows[joint] = pos_lbl
            grip_grid.addWidget(name_lbl, row, 0)
            grip_grid.addWidget(pos_lbl, row, 1)
        self.grip_total_lbl = QLabel("--")
        grip_grid.addWidget(QLabel("Apertura total"), len(GRIPPER_JOINT_NAMES) + 1, 0)
        grip_grid.addWidget(self.grip_total_lbl, len(GRIPPER_JOINT_NAMES) + 1, 1)
        g_grip.setLayout(grip_grid)
        lay.addWidget(g_grip)

        g_fk = QGroupBox("Cinematica (FK)")
        g_fk.setFlat(True)
        g_fk.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        fk_grid = QGridLayout()
        fk_grid.setContentsMargins(6, 8, 6, 6)
        fk_grid.setHorizontalSpacing(6)
        fk_grid.setVerticalSpacing(2)
        self.ee_pos_lbl = QLabel("--")
        self.ee_rpy_lbl = QLabel("--")
        fk_grid.addWidget(QLabel("TCP xyz [m]"), 0, 0)
        fk_grid.addWidget(self.ee_pos_lbl, 0, 1)
        fk_grid.addWidget(QLabel("RPY [deg]"), 1, 0)
        fk_grid.addWidget(self.ee_rpy_lbl, 1, 1)
        g_fk.setLayout(fk_grid)
        lay.addWidget(g_fk)

        g_dyn = QGroupBox("Dinamica (vel/eff)")
        g_dyn.setFlat(True)
        g_dyn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        dyn_grid = QGridLayout()
        dyn_grid.setContentsMargins(6, 8, 6, 6)
        dyn_grid.setHorizontalSpacing(6)
        dyn_grid.setVerticalSpacing(2)
        self.vel_norm_lbl = QLabel("--")
        self.vel_max_lbl = QLabel("--")
        self.eff_max_lbl = QLabel("--")
        dyn_grid.addWidget(QLabel("||qdot||"), 0, 0)
        dyn_grid.addWidget(self.vel_norm_lbl, 0, 1)
        dyn_grid.addWidget(QLabel("max |qdot|"), 1, 0)
        dyn_grid.addWidget(self.vel_max_lbl, 1, 1)
        dyn_grid.addWidget(QLabel("max |eff|"), 2, 0)
        dyn_grid.addWidget(self.eff_max_lbl, 2, 1)
        g_dyn.setLayout(dyn_grid)
        lay.addWidget(g_dyn)

        lay.addStretch(1)
        self.setLayout(lay)

    def update_state(self, payload: Dict[str, object]):
        names = payload.get("name") or []
        pos_list = payload.get("position") or []
        vel_list = payload.get("velocity") or []
        eff_list = payload.get("effort") or []
        stamp = float(payload.get("stamp") or 0.0)
        source = payload.get("source") or "ros"
        if stamp:
            self._last_stamp = stamp

        norm_names = [_normalize_joint_name(n) for n in names]
        pos_map = {n: p for n, p in zip(norm_names, pos_list)}
        vel_map = {n: v for n, v in zip(norm_names, vel_list)} if len(vel_list) == len(names) else {}
        eff_map = {n: e for n, e in zip(norm_names, eff_list)} if len(eff_list) == len(names) else {}
        now = stamp or time.time()
        missing_vel = any(vel is None for vel in vel_map.values()) if vel_map else True
        if pos_map:
            if missing_vel and self._last_time and now > self._last_time:
                dt = max(1e-6, now - self._last_time)
                for name, pos in pos_map.items():
                    prev = self._last_positions.get(name)
                    if prev is not None and vel_map.get(name) is None:
                        vel_map[name] = (pos - prev) / dt
            self._last_positions.update(pos_map)
            self._last_time = now

        for joint, (pos_lbl, vel_lbl) in self._joint_rows.items():
            pos = pos_map.get(joint)
            vel = vel_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                deg = math.degrees(pos)
                pos_lbl.setText(f"{deg:.2f} deg / {pos:.3f} rad")
            if vel is None:
                vel_lbl.setText("--")
            else:
                vel_lbl.setText(f"{vel:.3f} rad/s")

        grip_positions = []
        for joint, pos_lbl in self._gripper_rows.items():
            pos = pos_map.get(joint)
            if pos is None:
                pos_lbl.setText("--")
            else:
                pos_mm = pos * 1000.0
                pos_lbl.setText(f"{pos_mm:.1f} mm / {pos:.4f} m")
                grip_positions.append(pos)
        if len(grip_positions) == len(GRIPPER_JOINT_NAMES):
            opening = sum(abs(v) for v in grip_positions) * 1000.0
            self.grip_total_lbl.setText(f"{opening:.1f} mm")
        else:
            self.grip_total_lbl.setText("--")

        q = []
        for joint in UR5_JOINT_NAMES:
            if joint not in pos_map:
                q = []
                break
            q.append(pos_map[joint])
        if len(q) == 6:
            pos, rot = fk_ur5(q)
            roll, pitch, yaw = _rot_to_rpy(rot)
            self.ee_pos_lbl.setText(f"{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}")
            self.ee_rpy_lbl.setText(
                f"{math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}"
            )
        else:
            self.ee_pos_lbl.setText("--")
            self.ee_rpy_lbl.setText("--")

        vel_values = [vel_map[j] for j in UR5_JOINT_NAMES if j in vel_map]
        if vel_values:
            norm = math.sqrt(sum(v * v for v in vel_values))
            vmax = max(abs(v) for v in vel_values)
            self.vel_norm_lbl.setText(f"{norm:.3f} rad/s")
            self.vel_max_lbl.setText(f"{vmax:.3f} rad/s")
        else:
            self.vel_norm_lbl.setText("--")
            self.vel_max_lbl.setText("--")

        eff_values = [eff_map[j] for j in UR5_JOINT_NAMES if j in eff_map]
        if eff_values:
            emax = max(abs(v) for v in eff_values)
            self.eff_max_lbl.setText(f"{emax:.3f}")
        else:
            self.eff_max_lbl.setText("--")

        if self._last_stamp:
            self.status_lbl.setText(f"Joint states: ok ({len(names)} joints, {source})")
        else:
            self.status_lbl.setText("Joint states: esperando /joint_states ...")


class MainControlTab(QWidget):
    """Main control panel: large camera + robot controls + joint states."""

    joint_state_signal = pyqtSignal(object)

    def __init__(
        self,
        runner: CmdRunner,
        ros: RosWorker,
        log_fn,
        pick_place_fn,
        pose_cmd_fn=None,
        hold_pause_fn=None,
        parent=None,
    ):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn
        self.pick_place_fn = pick_place_fn
        self.pose_cmd_fn = pose_cmd_fn
        self.hold_pause_fn = hold_pause_fn
        self.camera_topic = os.environ.get("PANEL_MAIN_CAMERA_TOPIC", "/camera_north/image")
        self.joint_topic = os.environ.get("PANEL_JOINT_STATES_TOPIC", "/joint_states")
        self._camera_seen = False
        self._joint_seen = False
        self._camera_subscribed = False
        self._joint_subscribed = False
        self._last_joint_ts = 0.0
        self._gz_poll_inflight = False
        self._gz_joint_topic: Optional[str] = None
        self._debug_joints_to_stdout = False
        self._last_debug_ts = 0.0

        self._build_ui()
        self.ros.image.connect(self._on_image)
        if hasattr(self.ros, "joint_state"):
            self.ros.joint_state.connect(self._on_joint_state)

        self._connect_timer = QTimer(self)
        self._connect_timer.timeout.connect(self._auto_subscribe)
        self._connect_timer.start(800)

        self._gz_timer = QTimer(self)
        self._gz_timer.timeout.connect(self._maybe_poll_gz)
        self._gz_timer.start(900)

    def _build_ui(self):
        root = QVBoxLayout()
        root.setContentsMargins(2, 2, 2, 2)
        root.setSpacing(2)

        splitter = QSplitter(Qt.Horizontal)

        cam_wrap = QWidget()
        cam_lay = QVBoxLayout()
        cam_lay.setContentsMargins(0, 0, 0, 0)
        cam_lay.setSpacing(2)
        title = f"Camara principal: {self.camera_topic}"
        self.camera_view = CameraView(title)
        self.camera_info = QLabel("Esperando imagen...")
        self.camera_info.setStyleSheet("color:#64748b;")
        cam_lay.addWidget(self.camera_view, 1)
        cam_lay.addWidget(self.camera_info)
        cam_wrap.setLayout(cam_lay)

        right = QWidget()
        right_lay = QVBoxLayout()
        right_lay.setContentsMargins(0, 0, 0, 0)
        right_lay.setSpacing(6)

        debug_row = QHBoxLayout()
        self.btn_debug_joints = QPushButton("Debug joints/poses -> terminal")
        self.btn_debug_joints.setCheckable(True)
        self.btn_debug_joints.toggled.connect(self._on_debug_joints_toggled)
        debug_row.addWidget(self.btn_debug_joints)
        debug_row.addStretch(1)

        self.robot_controls = RobotTab(
            self.runner,
            self.log_fn,
            infer_grasp_fn=None,
            pick_place_fn=self.pick_place_fn,
            pose_cmd_fn=self.pose_cmd_fn,
            hold_pause_fn=self.hold_pause_fn,
            model_refresh_fn=None,
            manual_in_tab=True,
            controls_only=True,
        )
        self.joint_panel = JointStatePanel()

        right_lay.addLayout(debug_row)
        right_lay.addWidget(self.robot_controls)
        right_lay.addWidget(self.joint_panel, 1)
        right.setLayout(right_lay)

        splitter.addWidget(cam_wrap)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        root.addWidget(splitter)
        self.setLayout(root)

    def _auto_subscribe(self):
        if not self._camera_subscribed:
            self.ros.subscribe_image(self.camera_topic)
            self._camera_subscribed = True
        if not self._joint_subscribed and hasattr(self.ros, "subscribe_joint_states"):
            self.ros.subscribe_joint_states(self.joint_topic)
            self._joint_subscribed = True
        if self._camera_subscribed and self._joint_subscribed:
            self._connect_timer.stop()

    def _on_image(self, topic: str, qimg, w: int, h: int, fps: float):
        if topic != self.camera_topic:
            return
        self._camera_seen = True
        self.camera_view.set_frame(qimg)
        self.camera_info.setText(f"{topic} | {w}x{h} | fps {fps:.1f}")

    def _on_joint_state(self, payload: Dict[str, object]):
        self._joint_seen = True
        self._last_joint_ts = time.time()
        self.joint_panel.update_state(payload)
        self.robot_controls.update_joint_state(payload)
        self._maybe_debug_joint_pose(payload)
        self.joint_state_signal.emit(payload)

    def _maybe_poll_gz(self):
        if self._joint_seen and (time.time() - self._last_joint_ts) < 1.0:
            return
        if self._gz_poll_inflight:
            return
        self._gz_poll_inflight = True
        threading.Thread(target=self._poll_gz_joint_state, daemon=True).start()

    def _poll_gz_joint_state(self):
        try:
            payload = self._read_gz_joint_state()
            if payload:
                QTimer.singleShot(0, lambda: self._apply_gz_payload(payload))
        finally:
            self._gz_poll_inflight = False

    def _apply_gz_payload(self, payload: Dict[str, object]):
        self._joint_seen = True
        self._last_joint_ts = time.time()
        self.joint_panel.update_state(payload)
        self.robot_controls.update_joint_state(payload)
        self._maybe_debug_joint_pose(payload)
        self.joint_state_signal.emit(payload)

    def _read_gz_joint_state(self) -> Optional[Dict[str, object]]:
        part = resolve_gz_partition("")
        env = build_gz_env(part)
        if not self._gz_joint_topic:
            out, err = run_cmd_output(env + "gz topic -l", timeout_sec=1.5)
            if err:
                return None
            candidates = [ln.strip() for ln in out.splitlines() if "joint_state" in ln and "ur5" in ln]
            if not candidates:
                return None
            self._gz_joint_topic = candidates[0]
        topic = self._gz_joint_topic
        out, err = run_cmd_output(
            env + f"gz topic -e -n 1 -t '{topic}' --json-output",
            timeout_sec=1.8,
        )
        if err:
            return None
        raw = (out or "").strip()
        if not raw:
            return None
        try:
            data = json.loads(raw[raw.find("{"):])
        except Exception:
            return None
        pos_map, vel_map, eff_map = _parse_gz_joint_payload(data)
        if not pos_map:
            return None
        names = list(pos_map.keys())
        payload = {
            "stamp": time.time(),
            "source": "gz",
            "name": names,
            "position": [pos_map.get(n) for n in names],
            "velocity": [vel_map.get(n) for n in names] if vel_map else [],
            "effort": [eff_map.get(n) for n in names] if eff_map else [],
        }
        return payload

    def set_robot_state(self, running: bool, ros2_running: bool, gz_running: bool):
        self.robot_controls.set_robot_state(running, ros2_running, gz_running)

    def set_calibrated(self, calibrated: bool):
        self.robot_controls.set_calibrated(calibrated)

    def set_busy(self, busy: bool):
        self.robot_controls.set_busy(busy)

    def unlock_buttons(self):
        self.robot_controls.unlock_buttons()

    def _on_debug_joints_toggled(self, checked: bool):
        self._debug_joints_to_stdout = bool(checked)
        if checked:
            self.log_fn("[UI] Debug joints/poses -> terminal (ON)")
            self._last_debug_ts = 0.0
            return
        self.log_fn("[UI] Debug joints/poses -> terminal (OFF)")

    def _maybe_debug_joint_pose(self, payload: Dict[str, object]):
        if not self._debug_joints_to_stdout:
            return
        now = time.time()
        if (now - self._last_debug_ts) < 0.4:
            return
        self._last_debug_ts = now
        names = payload.get("name") or []
        positions = payload.get("position") or []
        if not isinstance(names, list) or not isinstance(positions, list) or not names:
            return
        pos_map = {}
        for name, pos in zip(names, positions):
            try:
                pos_map[_normalize_joint_name(name)] = float(pos)
            except Exception:
                continue
        if not pos_map:
            return
        joint_parts = []
        for joint in UR5_JOINT_NAMES:
            if joint in pos_map:
                joint_parts.append(f"{joint}={pos_map[joint]:.3f}rad")
        for joint in GRIPPER_JOINT_NAMES:
            if joint in pos_map:
                joint_parts.append(f"{joint}={pos_map[joint]:.4f}m")
        pose_txt = "tcp xyz=-- rpy=--"
        if all(j in pos_map for j in UR5_JOINT_NAMES):
            q = [pos_map[j] for j in UR5_JOINT_NAMES]
            pos, rot = fk_ur5(q)
            roll, pitch, yaw = _rot_to_rpy(rot)
            pose_txt = (
                f"tcp xyz=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"rpy=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})"
            )
        source = payload.get("source") or "ros"
        msg = f"[DEBUG] JOINTS ({source}): " + " ".join(joint_parts) + f" | {pose_txt}"
        print(msg, flush=True)


def _rot_to_rpy(rot) -> Tuple[float, float, float]:
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


def _parse_gz_joint_payload(data: Dict[str, object]):
    if "msg" in data and isinstance(data["msg"], dict):
        data = data["msg"]
    pos_map: Dict[str, float] = {}
    vel_map: Dict[str, float] = {}
    eff_map: Dict[str, float] = {}

    names = data.get("name")
    positions = data.get("position")
    velocities = data.get("velocity")
    efforts = data.get("effort")
    if isinstance(names, list) and isinstance(positions, list) and len(names) == len(positions):
        for name, pos in zip(names, positions):
            try:
                pos_map[_normalize_joint_name(name)] = float(pos)
            except Exception:
                continue
        if isinstance(velocities, list) and len(velocities) == len(names):
            for name, vel in zip(names, velocities):
                try:
                    vel_map[_normalize_joint_name(name)] = float(vel)
                except Exception:
                    continue
        if isinstance(efforts, list) and len(efforts) == len(names):
            for name, eff in zip(names, efforts):
                try:
                    eff_map[_normalize_joint_name(name)] = float(eff)
                except Exception:
                    continue
        if pos_map:
            return pos_map, vel_map, eff_map

    joints = data.get("joint")
    if isinstance(joints, list):
        for joint in joints:
            if not isinstance(joint, dict):
                continue
            name = joint.get("name")
            if not name:
                continue
            pos = _first_value(joint.get("position"))
            vel = _first_value(joint.get("velocity"))
            eff = _first_value(joint.get("effort"))
            if pos is None or vel is None or eff is None:
                axis1 = joint.get("axis1") or {}
                if isinstance(axis1, dict):
                    if pos is None:
                        pos = axis1.get("position")
                        if pos is None:
                            pos = axis1.get("angle")
                    if vel is None:
                        vel = axis1.get("velocity")
                    if eff is None:
                        eff = axis1.get("force") or axis1.get("effort")
            norm_name = _normalize_joint_name(name)
            if pos is not None:
                try:
                    pos_map[norm_name] = float(pos)
                except Exception:
                    pass
            if vel is not None:
                try:
                    vel_map[norm_name] = float(vel)
                except Exception:
                    pass
            if eff is not None:
                try:
                    eff_map[norm_name] = float(eff)
                except Exception:
                    pass
    return pos_map, vel_map, eff_map


def _first_value(val):
    if isinstance(val, list) and val:
        return val[0]
    if isinstance(val, (int, float)):
        return val
    return None


def _normalize_joint_name(name) -> str:
    text = str(name)
    if "::" in text:
        return text.split("::")[-1]
    return text
