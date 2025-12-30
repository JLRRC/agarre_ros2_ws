#!/usr/bin/env python3

import os
import math
import shlex
import json
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QGroupBox,
    QComboBox,
    QCheckBox,
    QSizePolicy,
    QSlider,
    QDoubleSpinBox,
)
from typing import Optional, List, Set
try:
    import yaml
except Exception:
    yaml = None
from panel_config import *
from panel_utils import *


class RobotTab(QWidget):
    """UI tab for robot scripts and diagnostics."""

    def __init__(
        self,
        runner: CmdRunner,
        log_fn,
        infer_grasp_fn,
        pick_place_fn,
        pose_cmd_fn=None,
        hold_pause_fn=None,
        model_refresh_fn=None,
        manual_in_tab: bool = True,
        controls_only: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
        self.infer_grasp_fn = infer_grasp_fn
        self.pick_place_fn = pick_place_fn
        self.pose_cmd_fn = pose_cmd_fn
        self.hold_pause_fn = hold_pause_fn
        self.model_refresh_fn = model_refresh_fn
        self._busy = False
        self._last_running = False
        self._last_ros2_running = False
        self._last_gz_running = False
        self._locked_buttons: Set[QPushButton] = set()
        self._manual_inflight = False
        self._manual_pending = False
        self._manual_in_tab = bool(manual_in_tab)
        self._controls_only = bool(controls_only)
        self._calibrated = False
        self._robot_buttons: List[QPushButton] = []
        self._model_ckpts: List[str] = []
        self.runner.finished.connect(self._on_runner_finished)
        self._build_ui()

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        ros2_only = os.environ.get("PANEL_ROS2_ONLY", "0") == "1"
        enable_infer = os.environ.get("PANEL_ENABLE_INFER", "0") == "1"
        if not enable_infer and os.path.isfile(INFER_SCRIPT):
            enable_infer = True

        g = QGroupBox("Robot / DEMO (scripts existentes)")
        g.setFlat(True)
        g.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        gl = QVBoxLayout()
        gl.setContentsMargins(1, 1, 1, 1)
        gl.setSpacing(1)

        self.btn_home = QPushButton("UR5 → HOME")
        self.btn_table = QPushButton("UR5 → Mesa")
        self.btn_basket = QPushButton("UR5 → Cesta")
        self.btn_pick = QPushButton("Pick & Place (demo)")
        self.btn_open = QPushButton("Cerrar gripper")
        self.btn_close = QPushButton("Abrir gripper")
        self.btn_test = QPushButton("Test corto")
        self.btn_diag = QPushButton("Diagnóstico robot")
        self.btn_infer_exp = QPushButton("Inferir (modelo)")
        self.btn_pick_sel = QPushButton("Pick & Place (selección)")
        self.btn_infer_pick = QPushButton("Inferir + Pick")
        self.btn_grasp_start = QPushButton("Start grasp node")
        self.btn_grasp_stop = QPushButton("Stop grasp node")
        self.btn_diag_full = QPushButton("Diagnóstico completo (IA/ROS)")

        self.btn_home.clicked.connect(lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")))
        self.btn_table.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_table_pose.sh"))
        )
        self.btn_basket.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_basket_pose.sh"))
        )
        self.btn_pick.clicked.connect(self.pick_place_fn)
        self.btn_open.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
        )
        self.btn_close.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_close_gripper.sh"))
        )
        self.btn_test.clicked.connect(self._run_robot_test)
        self.btn_diag.clicked.connect(self._diag_robot)
        self.btn_infer_exp.clicked.connect(self._run_infer_exp)
        self.btn_pick_sel.clicked.connect(self._run_pick_sel)
        self.btn_infer_pick.clicked.connect(self._run_infer_pick)
        self.btn_grasp_start.clicked.connect(self._start_grasp_node)
        self.btn_grasp_stop.clicked.connect(self._stop_grasp_node)
        self.btn_diag_full.clicked.connect(self._diag_full)

        if self._controls_only:
            grid = QGridLayout()
            grid.setHorizontalSpacing(2)
            grid.setVerticalSpacing(2)
            grid.addWidget(self.btn_home, 0, 0)
            grid.addWidget(self.btn_table, 0, 1)
            grid.addWidget(self.btn_basket, 0, 2)
            grid.addWidget(self.btn_open, 1, 0)
            grid.addWidget(self.btn_close, 1, 1)
            grid.addWidget(self.btn_pick, 1, 2)
            gl.addLayout(grid)
        else:
            grid = QGridLayout()
            grid.setHorizontalSpacing(1)
            grid.setVerticalSpacing(1)
            grid.addWidget(self.btn_home, 0, 0)
            grid.addWidget(self.btn_table, 0, 1)
            grid.addWidget(self.btn_basket, 0, 2)
            grid.addWidget(self.btn_open, 1, 0)
            grid.addWidget(self.btn_close, 1, 1)
            grid.addWidget(self.btn_pick, 1, 2)
            grid.addWidget(self.btn_test, 2, 0)
            grid.addWidget(self.btn_diag, 2, 1)
            if ros2_only and not enable_infer:
                pass
            gl.addLayout(grid)

            g_infer = QGroupBox("Inferencia + Pick (experimentos)")
            g_infer.setFlat(True)
            g_infer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
            infer_grid = QGridLayout()
            infer_grid.setContentsMargins(1, 1, 1, 1)
            infer_grid.setHorizontalSpacing(2)
            infer_grid.setVerticalSpacing(2)
            infer_grid.addWidget(self.btn_grasp_start, 0, 0)
            infer_grid.addWidget(self.btn_grasp_stop, 0, 1)
            infer_grid.addWidget(self.btn_diag_full, 0, 2)
            infer_grid.addWidget(self.btn_infer_exp, 1, 0)
            infer_grid.addWidget(self.btn_pick_sel, 1, 1)
            infer_grid.addWidget(self.btn_infer_pick, 1, 2)
            g_infer.setLayout(infer_grid)
            if not enable_infer:
                g_infer.setVisible(False)
            gl.addWidget(g_infer)

            model_row = QHBoxLayout()
            model_row.setSpacing(4)
            self.model_combo = QComboBox()
            self.model_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
            self.btn_refresh_models = QPushButton("Actualizar modelos")
            if self.model_refresh_fn:
                self.btn_refresh_models.clicked.connect(self.model_refresh_fn)
            else:
                self.btn_refresh_models.setVisible(False)
            model_label = QLabel("Modelo (inferir grasp)")
            model_row.addWidget(model_label)
            model_row.addWidget(self.model_combo, 1)
            model_row.addWidget(self.btn_refresh_models)
            if ros2_only and not enable_infer:
                model_label.setVisible(False)
                self.model_combo.setVisible(False)
                self.btn_refresh_models.setVisible(False)
            gl.addLayout(model_row)

        g.setLayout(gl)
        lay.addWidget(g)

        manual = QGroupBox("Control manual (6 DOF)")
        manual.setFlat(True)
        manual.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        mlay = QVBoxLayout()
        mlay.setContentsMargins(1, 1, 1, 1)
        mlay.setSpacing(1)

        manual_row = QHBoxLayout()
        manual_row.setSpacing(4)
        self.btn_send_joints = QPushButton("Mover articulaciones")
        self.btn_send_joints.clicked.connect(self._request_manual_send)
        self.chk_auto_joints = QCheckBox("Auto")
        self.chk_auto_joints.setChecked(True)
        self.joint_time = QDoubleSpinBox()
        self.joint_time.setDecimals(2)
        self.joint_time.setRange(0.5, 8.0)
        self.joint_time.setSingleStep(0.25)
        self.joint_time.setValue(DEFAULT_JOINT_MOVE_SEC)
        self.joint_time.setSuffix(" s")
        manual_row.addWidget(self.btn_send_joints)
        manual_row.addWidget(QLabel("t"))
        manual_row.addWidget(self.joint_time)
        manual_row.addWidget(self.chk_auto_joints)
        manual_row.addStretch(1)
        mlay.addLayout(manual_row)

        grid = QGridLayout()
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(2)
        grid.setColumnStretch(2, 1)
        self.joint_sliders = []
        self.joint_value_labels = []
        self.joint_step_buttons = []
        slider_min = int(JOINT_SLIDER_DEG_MIN * JOINT_SLIDER_SCALE)
        slider_max = int(JOINT_SLIDER_DEG_MAX * JOINT_SLIDER_SCALE)
        home_pose = load_home_pose()
        step_deg = 1.0 / JOINT_SLIDER_SCALE if JOINT_SLIDER_SCALE else 1.0
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            jlabel = QLabel(f"J{idx + 1}")
            jlabel.setToolTip(joint)
            btn_minus = QPushButton("-")
            btn_plus = QPushButton("+")
            btn_minus.setFixedWidth(22)
            btn_plus.setFixedWidth(22)
            btn_minus.setToolTip(f"Paso -{step_deg:.1f} deg")
            btn_plus.setToolTip(f"Paso +{step_deg:.1f} deg")
            btn_minus.clicked.connect(lambda _=False, i=idx: self._step_joint_slider(i, -1))
            btn_plus.clicked.connect(lambda _=False, i=idx: self._step_joint_slider(i, 1))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(slider_min, slider_max)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            value_lbl = QLabel("0.0 deg / 0.00 rad")
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider.valueChanged.connect(lambda v, i=idx: self._on_joint_slider_change(i, v))
            slider.sliderReleased.connect(self._maybe_send_joints_auto)
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_lbl)
            self.joint_step_buttons.append((btn_minus, btn_plus))
            if idx < len(home_pose):
                slider.setValue(int(round(math.degrees(home_pose[idx]) * JOINT_SLIDER_SCALE)))
            else:
                slider.setValue(0)
            grid.addWidget(jlabel, idx, 0)
            grid.addWidget(btn_minus, idx, 1)
            grid.addWidget(slider, idx, 2)
            grid.addWidget(btn_plus, idx, 3)
            grid.addWidget(value_lbl, idx, 4)
        mlay.addLayout(grid)
        manual.setLayout(mlay)
        self.manual_group = manual
        info = QLabel(
            "Nota: las barras se sincronizan con /joint_states; si no hay datos, revisa el nodo/controlador.\n"
            "Para control completo, arranca ros2_control del UR5 o Gazebo."
        )
        info.setWordWrap(True)
        info.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        manual_panel = QWidget()
        manual_layout = QVBoxLayout()
        manual_layout.setContentsMargins(0, 0, 0, 0)
        manual_layout.setSpacing(1)
        manual_layout.addWidget(manual)
        manual_layout.addWidget(info)
        manual_panel.setLayout(manual_layout)
        self.manual_panel = manual_panel
        if self._manual_in_tab:
            lay.addWidget(manual_panel)
        # no stretch: keep compact
        self.setLayout(lay)
        if self._controls_only:
            self._robot_buttons = [
                self.btn_home,
                self.btn_table,
                self.btn_basket,
                self.btn_pick,
                self.btn_open,
                self.btn_close,
            ]
        else:
            self._robot_buttons = [
                self.btn_home,
                self.btn_table,
                self.btn_basket,
                self.btn_pick,
                self.btn_open,
                self.btn_close,
                self.btn_test,
                self.btn_diag,
                self.btn_infer_exp,
                self.btn_pick_sel,
                self.btn_infer_pick,
                self.btn_grasp_start,
                self.btn_grasp_stop,
                self.btn_diag_full,
            ]
        self.set_robot_state(False, False, False)

    def set_model_options(self, options: List[tuple], default_label: Optional[str] = None):
        self._model_ckpts = []
        if not hasattr(self, "model_combo"):
            return
        self.model_combo.clear()
        if not options:
            self.model_combo.addItem("Sin modelos disponibles")
            self.model_combo.setEnabled(False)
            return
        for label, ckpt in options:
            self.model_combo.addItem(label)
            self._model_ckpts.append(ckpt)
        self.model_combo.setEnabled(True)
        if default_label:
            idx = self.model_combo.findText(default_label)
            if idx >= 0:
                self.model_combo.setCurrentIndex(idx)

    def get_selected_ckpt(self) -> str:
        if not hasattr(self, "model_combo"):
            return ""
        if not self._model_ckpts:
            return ""
        idx = self.model_combo.currentIndex()
        if idx < 0 or idx >= len(self._model_ckpts):
            return ""
        return self._model_ckpts[idx]

    def _on_runner_finished(self, tag: str, rc: int):
        if tag != "ROBOT-MANUAL":
            return
        self._manual_inflight = False
        if self._manual_pending:
            self._manual_pending = False
            self._request_manual_send()

    def _slider_to_deg(self, value: int) -> float:
        scale = JOINT_SLIDER_SCALE if JOINT_SLIDER_SCALE else 1.0
        return float(value) / float(scale)

    def _current_joint_positions_rad(self) -> List[float]:
        positions = []
        for s in self.joint_sliders:
            deg = self._slider_to_deg(s.value())
            positions.append(math.radians(deg))
        return positions

    def _step_joint_slider(self, idx: int, direction: int):
        if idx < 0 or idx >= len(self.joint_sliders):
            return
        slider = self.joint_sliders[idx]
        step = slider.singleStep() or 1
        new_val = slider.value() + int(direction) * int(step)
        new_val = max(slider.minimum(), min(slider.maximum(), new_val))
        if new_val == slider.value():
            return
        slider.setValue(new_val)
        self._maybe_send_joints_auto()

    def _on_joint_slider_change(self, idx: int, value: int):
        if idx < 0 or idx >= len(self.joint_value_labels):
            return
        deg = self._slider_to_deg(value)
        rad = math.radians(deg)
        self.joint_value_labels[idx].setText(f"{deg:.1f} deg / {rad:.3f} rad")

    def update_joint_state(self, payload: dict):
        if not payload:
            return
        if any(s.isSliderDown() for s in self.joint_sliders):
            return
        names = payload.get("name") or []
        positions = payload.get("position") or []
        if not isinstance(names, list) or not isinstance(positions, list):
            return
        pos_map = {}
        for name, pos in zip(names, positions):
            try:
                pos_map[_normalize_joint_name(name)] = float(pos)
            except Exception:
                continue
        if not pos_map:
            return
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            if joint not in pos_map:
                continue
            deg = math.degrees(pos_map[joint])
            slider = self.joint_sliders[idx]
            value = int(round(deg * JOINT_SLIDER_SCALE))
            value = max(slider.minimum(), min(slider.maximum(), value))
            if slider.value() != value:
                slider.setValue(value)

    def _maybe_send_joints_auto(self):
        if self.chk_auto_joints.isChecked():
            self._request_manual_send()

    def _request_manual_send(self):
        if self._manual_inflight:
            self._manual_pending = True
            return
        self._manual_pending = False
        self._send_joint_positions()

    def _send_joint_positions(self):
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            return
        if ros2_control_running() and not gz_sim_running():
            if not self._robot_ready(require_gripper=False):
                return
        positions = [round(p, 4) for p in self._current_joint_positions_rad()]
        tsec = float(self.joint_time.value())
        sec = max(0.0, tsec)
        sec_i = int(sec)
        nsec_i = int((sec - sec_i) * 1e9)
        msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": UR5_JOINT_NAMES,
            "points": [{
                "positions": positions,
                "time_from_start": {"sec": sec_i, "nanosec": nsec_i},
            }],
        }
        topic = detect_arm_trajectory_topic()
        cmd = (
            bash_preamble(WS_DIR)
            + "timeout 6 ros2 topic pub --once "
            + f"{shlex.quote(topic)} trajectory_msgs/msg/JointTrajectory "
            + shlex.quote(json.dumps(msg))
        )
        if self.hold_pause_fn:
            self.hold_pause_fn(max(6.0, tsec + 2.0))
        self._manual_inflight = True
        self.runner.run_stream("ROBOT-MANUAL", cmd)

    def _run_script(self, tag: str, script_path: str):
        self.log_fn(f"[UI] Botón: {tag} -> {os.path.basename(script_path)}")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        self.lock_button(self.sender() if isinstance(self.sender(), QPushButton) else None)
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            self.unlock_buttons()
            return
        if self.pose_cmd_fn and os.path.basename(script_path) in (
            "ur5_go_home.sh",
            "ur5_go_table_pose.sh",
            "ur5_go_basket_pose.sh",
            "ur5_go_test_pose.sh",
        ):
            self.pose_cmd_fn(script_path)
        if ros2_control_running() and not gz_sim_running():
            need_gripper = os.path.basename(script_path) in ("ur5_open_gripper.sh", "ur5_close_gripper.sh")
            if not self._robot_ready(require_gripper=need_gripper):
                self.unlock_buttons()
                return
        if not os.path.isfile(script_path):
            self.log_fn(f"[{tag}] ERROR: no existe {script_path}")
            self.unlock_buttons()
            return
        if not os.access(script_path, os.X_OK):
            self.log_fn(f"[{tag}] WARN: {script_path} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"timeout 12 '{script_path}' || true"
        self.runner.run_stream(tag, cmd)

    def set_robot_state(self, running: bool, ros2_running: bool, gz_running: bool):
        self._last_running = running
        self._last_ros2_running = ros2_running
        self._last_gz_running = gz_running
        self._refresh_robot_buttons()

    def set_calibrated(self, calibrated: bool):
        self._calibrated = bool(calibrated)
        self._refresh_robot_buttons()

    def _can_operate(self) -> bool:
        return self._calibrated and self._last_running and not self._busy

    def _apply_button_state(self, btn: QPushButton, enabled: bool):
        if not btn or btn in self._locked_buttons:
            return
        btn.setEnabled(enabled)

    def _refresh_robot_buttons(self):
        enabled = self._can_operate()
        for btn in self._robot_buttons:
            self._apply_button_state(btn, enabled)
        if self.manual_group is not None:
            self.manual_group.setEnabled(enabled)
        if self.manual_panel is not None:
            self.manual_panel.setEnabled(enabled)

    def set_busy(self, busy: bool):
        self._busy = bool(busy)
        self._refresh_robot_buttons()

    def lock_button(self, btn: Optional[QPushButton]):
        if not btn:
            return
        self._locked_buttons.add(btn)
        btn.setEnabled(False)

    def unlock_buttons(self):
        self._locked_buttons.clear()
        self._refresh_robot_buttons()

    def _diag_robot(self):
        self.log_fn("[UI] Botón: Diagnóstico robot")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        self.lock_button(self.sender() if isinstance(self.sender(), QPushButton) else None)
        self.log_fn("[ROBOT] Diagnóstico: topics/nodos/controladores")
        cmd_topics = bash_preamble(WS_DIR) + "timeout 2 ros2 topic list | egrep 'ur|gripper|joint|controller|trajectory' || true"
        cmd_nodes = bash_preamble(WS_DIR) + "timeout 2 ros2 node list | egrep 'ur|controller|moveit|gazebo|robot' || true"
        cmd_ctrl = bash_preamble(WS_DIR) + "timeout 2 ros2 control list_controllers || true"
        self.runner.run_stream("ROBOT-DIAG", cmd_topics)
        self.runner.run_stream("ROBOT-DIAG", cmd_nodes)
        self.runner.run_stream("ROBOT-DIAG", cmd_ctrl)

    def _run_infer_exp(self):
        self.log_fn("[UI] Botón: Inferir (modelo)")
        if not self.infer_grasp_fn:
            self.log_fn("[MODEL] Inference no disponible en este panel.")
            return
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        self.infer_grasp_fn()

    def _run_pick_sel(self):
        self.log_fn("[UI] Botón: Pick & Place (selección)")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        if not self.pick_place_fn:
            self.log_fn("[PICK] Acción no disponible en este panel.")
            return
        self.pick_place_fn()

    def _run_infer_pick(self):
        self.log_fn("[UI] Botón: Inferir + Pick")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        if self.infer_grasp_fn:
            self.infer_grasp_fn()
        if self.pick_place_fn:
            QTimer.singleShot(1200, self.pick_place_fn)

    def _start_grasp_node(self):
        self.log_fn("[UI] Botón: Start grasp node")
        script_path = os.path.join(SCRIPTS_DIR, "run_grasp_node.sh")
        if not os.path.isfile(script_path):
            self.log_fn(f"[GRASP] ERROR: no existe {script_path}")
            return
        if not os.access(script_path, os.X_OK):
            self.log_fn(f"[GRASP] WARN: {script_path} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"'{script_path}' || true"
        self.runner.run_stream("GRASP", cmd)

    def _stop_grasp_node(self):
        self.log_fn("[UI] Botón: Stop grasp node")
        cmd = "pkill -f 'grasp_pose_publisher|grasp_pose_node' || true"
        self.runner.run_stream("GRASP", cmd)

    def _diag_full(self):
        self.log_fn("[UI] Botón: Diagnóstico completo (IA/ROS)")
        cmd_topics = bash_preamble(WS_DIR) + (
            "ros2 topic list | egrep 'grasp|desired_grasp|camera|image|/clock|joint|controller' || true"
        )
        cmd_nodes = bash_preamble(WS_DIR) + (
            "ros2 node list | egrep 'grasp|camera|bridge|controller|gazebo|robot' || true"
        )
        cmd_ctrl = bash_preamble(WS_DIR) + "ros2 control list_controllers || true"
        self.runner.run_stream("DIAG", cmd_topics)
        self.runner.run_stream("DIAG", cmd_nodes)
        self.runner.run_stream("DIAG", cmd_ctrl)

    def _run_robot_test(self):
        self.log_fn("[UI] Botón: Test corto")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        self.lock_button(self.sender() if isinstance(self.sender(), QPushButton) else None)
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            self.unlock_buttons()
            return
        if ros2_control_running() and not gz_sim_running():
            active = self._robot_ready(require_gripper=False)
            if not active:
                self.unlock_buttons()
                return
        s_test = os.path.join(SCRIPTS_DIR, "ur5_quick_test.sh")
        if not os.path.isfile(s_test):
            self.log_fn(f"[ROBOT] ERROR: no existe {s_test}")
            self.unlock_buttons()
            return
        if not os.access(s_test, os.X_OK):
            self.log_fn(f"[ROBOT] WARN: {s_test} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"timeout 20 '{s_test}' || true"
        self.runner.run_stream("ROBOT-TEST", cmd)

    def _start_ur5_controllers(self):
        self.log_fn("[UI] Botón: Start UR5 controllers")
        if not ros2_control_running():
            self.log_fn("[ROBOT] ros2_control no está en ejecución. Pulsa 'Start UR5 ros2_control' primero.")
            return
        active, err = list_active_controllers()
        if err:
            self.log_fn(f"[ROBOT] ERROR comprobando controladores: {err}")
            return
        active = active or set()
        states, err = list_controllers_state()
        if err:
            self.log_fn(f"[ROBOT] ERROR comprobando estados: {err}")
            return
        states = states or {}

        cmds = []

        def ensure_controller(name: str):
            state = states.get(name)
            if state is None:
                cmds.append(
                    f"ros2 run controller_manager spawner {name} -c /controller_manager "
                    "--controller-manager-timeout 30 --switch-timeout 30"
                )
            elif state != "active":
                cmds.append(f"ros2 control set_controller_state {name} active")

        ensure_controller("joint_state_broadcaster")
        ensure_controller("joint_trajectory_controller")
        if gripper_controller_defined():
            ensure_controller("gripper_controller")
        else:
            self.log_fn("[ROBOT] gripper_controller no definido en ur5_controllers.yaml")

        if not cmds:
            self.log_fn("[ROBOT] Controladores ya activos.")
            return

        cmd = bash_preamble(WS_DIR) + " ; ".join(cmds) + " || true"
        self.runner.run_stream("ROBOT-CTRL", cmd)

    def _start_ur5_ros2_control(self):
        self.log_fn("[UI] Botón: Start UR5 ros2_control")
        if gz_sim_running():
            self.log_fn("[ROBOT] Gazebo en marcha. En sim no necesitas ros2_control.")
            return
        if ros2_control_running():
            self.log_fn("[ROBOT] ros2_control ya está en ejecución.")
            return
        cmd = bash_preamble(WS_DIR) + "ros2 launch ur5_bringup ur5_ros2_control.launch.py"
        self.runner.run_stream("ROBOT-CTRL", cmd)

    def _robot_ready(self, require_gripper: bool = False) -> bool:
        active, err = list_active_controllers()
        if err:
            self.log_fn(f"[ROBOT] ERROR comprobando controladores: {err}")
            return False
        if active is None or not active:
            self.log_fn("[ROBOT] No hay controladores activos.")
            return False
        if "joint_trajectory_controller" not in active:
            self.log_fn("[ROBOT] joint_trajectory_controller no activo.")
            self.log_fn("[ROBOT] Arranca el bringup/ros2_control antes de mover el robot.")
            return False
        if require_gripper and gripper_controller_defined() and "gripper_controller" not in active:
            self.log_fn("[ROBOT] gripper_controller no activo.")
            return False
        return True


def _normalize_joint_name(name) -> str:
    text = str(name)
    if "::" in text:
        return text.split("::")[-1]
    return text
