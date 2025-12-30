#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/main_panel.py
# Summary: Main Qt panel for UR5 simulation control and camera monitoring.
"""Qt control panel for UR5 simulation, bridge, cameras, and evidence capture."""
import os
import re
import sys
import math
import time
import shutil
import shlex
import json
import csv
import threading
import subprocess
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Optional, Dict, List, Tuple, Callable

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QGuiApplication, QImage
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QCheckBox,
    QAbstractButton,
    QMessageBox,
    QSplitter,
    QSizePolicy,
)

try:
    import yaml
except Exception:
    yaml = None

from panel_config import *
from panel_utils import *
from cameras_tab import CamerasTab, SelectedTarget
from simulation_tab import SimulationTab
from robot_tab import RobotTab
from main_control_tab import MainControlTab
from experiments_tab import ExperimentsTab
from evidences_tab import EvidencesTab, LogDialog
from tfm_evidence_tab import TfmEvidenceTab
from ur5_kinematics import fk_ur5, ik_ur5, rot_x, rot_z

class MainWindow(QMainWindow):
    """Main application window for the SUPER PRO panel."""

    log_signal = pyqtSignal(str)
    robot_state_signal = pyqtSignal(bool, bool)
    pick_start_signal = pyqtSignal(str)
    unlock_robot_buttons_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._ros2_only = os.environ.get("PANEL_ROS2_ONLY", "0") == "1"
        self._enable_infer = os.environ.get("PANEL_ENABLE_INFER", "0") == "1"
        if not self._enable_infer and os.path.isfile(INFER_SCRIPT):
            self._enable_infer = True
        if self._ros2_only:
            self.setWindowTitle("Panel ROS2/Gazebo — UR5 (ROS 2 Jazzy + Gazebo)")
        else:
            self.setWindowTitle("Panel SUPER PRO — Agarre inteligente (ROS 2 Jazzy + Gazebo)")

        ensure_dir(LOG_DIR)
        ensure_dir(BAGS_DIR)
        ensure_dir(FIG_DIR)

        self.runner = CmdRunner()
        self.runner.line.connect(self._append_log)
        self.runner.started.connect(self._on_cmd_started)
        self.runner.finished.connect(self._on_cmd_finished)

        self.log_dialog = LogDialog(self)
        self._log_buffer: List[str] = []
        self._log_history = deque(maxlen=5000)
        self._log_flush_timer = QTimer(self)
        self._log_flush_timer.setInterval(120)
        self._log_flush_timer.timeout.connect(self._flush_log_buffer)
        self._debug_logs_to_stdout = False
        self.alert_lbl = QLabel("OK")
        self.alert_lbl.setStyleSheet("background:#16a34a; color:white; padding:4px 10px; border-radius:8px;")

        self.ros = RosWorker()
        self.ros.log.connect(self._append_log)
        self.ros.image.connect(self._on_image)
        self.ros.start()
        load_object_positions()

        self.log_signal.connect(self._append_log_ui)
        self.robot_state_signal.connect(self._apply_robot_state)
        self.pick_start_signal.connect(self._start_pick_demo)
        self.selected_target: Optional[SelectedTarget] = None
        self._last_table_positions: Dict[str, Tuple[float, float, float]] = {}
        self._calib_active = False
        self._calib_queue: List[str] = []
        self._calib_points: List[Tuple[str, int, int]] = []
        self._last_obj_update = 0.0
        self._obj_update_interval = 0.5
        self._robot_busy_count = 0
        self._calibrated = False

        central = QWidget()
        root = QVBoxLayout()
        root.setContentsMargins(1, 1, 1, 1)
        root.setSpacing(1)

        self.tab_exp = None
        self.tab_tfm = None
        if not self._ros2_only:
            self.tab_exp = ExperimentsTab(self.runner, self._append_log)
            self.tab_tfm = TfmEvidenceTab(self._append_log)
        self.tab_sim = SimulationTab(self.runner, self.ros, self._append_log, show_evidences=False)
        self.tab_cam = CamerasTab(
            self.runner,
            self.ros,
            self._append_log,
            calib_start_fn=self._start_calibration,
            calib_click_fn=self._handle_calib_click,
            infer_grasp_fn=self.infer_grasp_model if self._enable_infer else None,
            show_top_bar=False,
        )
        self.tab_robot = RobotTab(
            self.runner,
            self._append_log,
            self.infer_grasp_model,
            self.pick_and_place_selected,
            pose_cmd_fn=self._set_hold_script,
            hold_pause_fn=self._pause_hold,
            model_refresh_fn=self._refresh_infer_models,
            manual_in_tab=False,
        )
        self.tab_main = MainControlTab(
            self.runner,
            self.ros,
            self._append_log,
            self.pick_and_place_selected,
            pose_cmd_fn=self._set_hold_script,
            hold_pause_fn=self._pause_hold,
        )
        self.tab_main.joint_state_signal.connect(self._on_joint_state_payload)
        self.tab_cam.set_manual_panel(self.tab_robot.manual_panel)
        self.tab_cam.set_status_panel(self.tab_sim.status_panel)
        self.unlock_robot_buttons_signal.connect(self.tab_robot.unlock_buttons)
        self.unlock_robot_buttons_signal.connect(self.tab_main.unlock_buttons)
        self._debug_buttons: List[Tuple[QAbstractButton, Callable[[], str]]] = []
        self._debug_active_button: Optional[QAbstractButton] = None
        self._debug_active_context: Optional[str] = None
        if self.tab_exp is not None:
            self._register_debug_button(
                self.tab_exp.btn_debug,
                lambda: f"Experimentos/{self.tab_exp.current_subtab_name()}",
            )
        self._register_debug_button(
            self.tab_sim.btn_debug,
            lambda: "Robot / ROS2-Gazebo",
        )
        if self.tab_tfm is not None:
            self._register_debug_button(
                self.tab_tfm.btn_debug,
                lambda: "Evidencias (TFM)",
            )
            self.tab_tfm.request_mesa_snapshot.connect(self.tab_cam.capture_mesa_snapshot)
        self.tab_sim.state_changed.connect(self._schedule_robot_state_check)
        self.tab_sim.bridge_started.connect(self._on_bridge_started)
        self.tab_cam.target_selected.connect(self._on_target_selected)
        if self.tab_tfm is not None and self.tab_exp is not None:
            self.tab_tfm.go_experiments.connect(lambda: self.tabs_widget.setCurrentWidget(self.tab_exp))

        if not self._ros2_only or self._enable_infer:
            self._refresh_infer_models()

        simulation_page = self._build_simulation_page()
        self.tabs_widget = QTabWidget()
        if self.tab_exp is not None:
            self.tabs_widget.addTab(self.tab_exp, "Experimentos")
        if self.tab_tfm is not None:
            self.tabs_widget.addTab(self.tab_tfm, "Evidencias (TFM)")
        self.tabs_widget.addTab(self.tab_main, "Panel principal")
        self.tabs_widget.addTab(simulation_page, "Robot / ROS2-Gazebo")
        self.tabs_widget.setCurrentWidget(simulation_page)
        root.addWidget(self.tabs_widget)

        central.setLayout(root)
        self.setCentralWidget(central)

        # COLD BOOT dentro del panel (si está activado)
        cold_boot_on = os.environ.get("PANEL_COLD_BOOT", "1") == "1"
        if cold_boot_on:
            QTimer.singleShot(0, self._run_cold_boot)
        self._log_term_procs: Dict[str, subprocess.Popen] = {}

        self._debug_logs_to_stdout = False
        self._robot_state_check_running = False
        self._robot_state_timer = QTimer(self)
        self._robot_state_timer.timeout.connect(self._schedule_robot_state_check)
        self._robot_state_timer.start(2000)
        self._auto_hold_enabled = os.environ.get("PANEL_AUTO_HOLD", "0") == "1"
        self._hold_timer = QTimer(self)
        self._hold_timer.timeout.connect(self._sim_hold_tick)
        if self._auto_hold_enabled:
            self._hold_timer.start(1600)
        self._hold_script: Optional[str] = os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")
        self._hold_pause_until = 0.0
        self._hold_proc: Dict[str, subprocess.Popen] = {}
        self._hold_last_run = 0.0
        self._auto_hold_interval = float(os.environ.get("PANEL_AUTO_HOLD_INTERVAL", "10"))
        self._last_bridge_camera_start = 0.0
        QTimer.singleShot(0, self._fit_to_screen)
        self._last_grasp_by_object: Dict[str, Dict[str, object]] = {}
        self._last_grasp_ts: Dict[str, float] = {}
        self._pickplace_dir = os.path.expanduser("~/TFM/reports/ros2_pickplace")
        ensure_dir(self._pickplace_dir)
        self._pickplace_log = os.path.join(self._pickplace_dir, f"log_{now_tag()}.txt")

        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        global TABLE_CAM_INFO
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
        TABLE_CAM_INFO = None
        calib = load_table_calib()
        if isinstance(calib, dict):
            TABLE_PIXEL_RECT = calib
        elif isinstance(calib, list):
            if len(calib) == 3 and all(len(row) == 3 for row in calib):
                TABLE_PIXEL_HOMOGRAPHY = calib
            else:
                TABLE_PIXEL_AFFINE = calib
        if TABLE_PIXEL_AFFINE or TABLE_PIXEL_RECT or TABLE_PIXEL_HOMOGRAPHY:
            self._append_log(f"[PICK] Calibración cargada: {TABLE_CALIB_PATH}")
            self._set_calibrated(True)
        else:
            if os.path.isfile(TABLE_CALIB_PATH):
                self._append_log("[PICK] Calibración inválida, se ignora. Recalibra.")
            self._set_calibrated(False)

    def _build_simulation_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        left = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(2)
        left_layout.setAlignment(Qt.AlignTop)
        self.tab_sim.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        self.tab_robot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        left_layout.addWidget(self.tab_sim)
        left_layout.addWidget(self.tab_robot)
        left_layout.addStretch(1)
        left.setLayout(left_layout)

        right = QWidget()
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(2)
        if hasattr(self.tab_cam, "top_bar_widget") and self.tab_cam.top_bar_widget is not None:
            right_layout.addWidget(self.tab_cam.top_bar_widget, 0)
        right_layout.addWidget(self.tab_cam)
        right.setLayout(right_layout)

        self.main_split = QSplitter(Qt.Horizontal)
        self.main_split.addWidget(left)
        self.main_split.addWidget(right)
        self.main_split.setStretchFactor(0, 1)
        self.main_split.setStretchFactor(1, 2)

        layout.addWidget(self._build_control_bar(), 0)
        layout.addWidget(self.main_split, 1)
        page.setLayout(layout)
        return page

    def _build_control_bar(self) -> QWidget:
        bar = QWidget()
        bottom = QHBoxLayout()
        bottom.setContentsMargins(1, 1, 1, 1)
        bottom.setSpacing(1)
        b_start = QPushButton("✅ START ALL")
        b_stop = QPushButton("🛑 STOP ALL")
        b_kill = QPushButton("💀 KILL HARD")
        b_logs = QPushButton("🧾 Logs")
        b_close = QPushButton("Cerrar panel")
        self.chk_logs = QCheckBox("Abrir logs")

        b_start.clicked.connect(self.start_all)
        b_stop.clicked.connect(self.stop_all)
        b_kill.clicked.connect(self.kill_hard)
        b_logs.clicked.connect(self.toggle_logs)
        b_close.clicked.connect(self.close_panel)

        bottom.addWidget(b_start)
        bottom.addWidget(self.chk_logs)
        bottom.addWidget(b_stop)
        bottom.addWidget(b_kill)
        bottom.addWidget(b_logs)
        bottom.addWidget(self.alert_lbl)
        bottom.addStretch(1)
        bottom.addWidget(b_close)
        bar.setLayout(bottom)
        return bar

    def _fit_to_screen(self):
        screen = QGuiApplication.primaryScreen()
        if not screen:
            return
        rect = screen.availableGeometry()
        w = max(600, int(rect.width() * 0.40))
        h = max(350, int(rect.height() * 0.40))
        self.resize(w, h)
        self.move(rect.x() + int(rect.width() * 0.22), rect.y() + int(rect.height() * 0.22))
        total_w = self.main_split.width()
        if total_w > 0:
            left_w = int(total_w * 0.34)
            self.main_split.setSizes([left_w, total_w - left_w])

    def _run_cold_boot(self):
        threading.Thread(target=cold_boot_kill, args=(self._append_log,), daemon=True).start()

    def _rotate_startup_logs(self):
        ensure_dir(LOG_DIR)
        for name in ("gz_server.log", "gz_gui.log", "ros_gz_bridge.log", "ros2_control.log"):
            rotate_log(os.path.join(LOG_DIR, name))
        self._append_log("[LOGS] Logs rotados y vaciados (START ALL).")

    def _open_log_terminals(self):
        if not os.environ.get("DISPLAY"):
            self._append_log("[LOGS] DISPLAY vacío; no se abren terminales de logs.")
            return
        self._append_log("[LOGS] Abriendo terminales de Gazebo y ROS2...")
        gz_filter = build_log_filter_cmd(GZ_LOG_FILTERS, unbuffered=True)
        gz_cmd = (
            "while true; do "
            f"if ls '{LOG_DIR}'/gz_*.log >/dev/null 2>&1; then "
            f"tail -n 200 -F '{LOG_DIR}'/gz_*.log"
            + (f" | {gz_filter}" if gz_filter else "")
            + "; "
            "else echo '[WAIT] No hay log de Gazebo aun'; sleep 1; fi; "
            "done"
        )
        ros_filter = build_log_filter_cmd(GZ_LOG_FILTERS, unbuffered=True)
        ros2_cmd = (
            "while true; do "
            f"if ls '{LOG_DIR}'/ros2_control.log '{LOG_DIR}'/ros_gz_bridge.log >/dev/null 2>&1; then "
            f"tail -n 200 -F '{LOG_DIR}'/ros2_control.log '{LOG_DIR}'/ros_gz_bridge.log"
            + (f" | {ros_filter}" if ros_filter else "")
            + "; "
            "else echo '[WAIT] No hay logs ROS2 aun'; sleep 1; fi; "
            "done"
        )
        self._open_log_terminal("gazebo", "Gazebo logs", gz_cmd)
        self._open_log_terminal("ros2", "ROS2 logs", ros2_cmd)

    def _open_log_terminal(self, key: str, title: str, shell_cmd: str):
        cmd = self._build_terminal_cmd(title, shell_cmd)
        if not cmd:
            self._append_log("[WARN] [LOGS] No se encuentra emulador de terminal.")
            return
        try:
            proc = subprocess.Popen(cmd)
            self._log_term_procs[key] = proc
            self._append_log(f"[LOGS] Terminal abierto -> {title}")
            QTimer.singleShot(1500, lambda: self._check_log_terminal(key, title))
        except Exception as e:
            self._append_log(f"[WARN] [LOGS] No pude abrir terminal ({title}): {e}")

    def _check_log_terminal(self, key: str, title: str):
        proc = self._log_term_procs.get(key)
        if not proc:
            return
        if proc.poll() is not None:
            self._append_log(f"[WARN] [LOGS] Terminal cerrado -> {title}")

    def _build_terminal_cmd(self, title: str, shell_cmd: str) -> Optional[List[str]]:
        for term in (
            "gnome-terminal",
            "x-terminal-emulator",
            "konsole",
            "xfce4-terminal",
            "mate-terminal",
            "terminator",
            "xterm",
        ):
            if shutil.which(term):
                if term == "gnome-terminal":
                    return [term, "--title", title, "--", "bash", "-lc", shell_cmd]
                if term == "x-terminal-emulator":
                    return [term, "-T", title, "-e", "bash", "-lc", shell_cmd]
                if term == "konsole":
                    return [term, "--new-tab", "-p", f"tabtitle={title}", "-e", "bash", "-lc", shell_cmd]
                if term == "xfce4-terminal":
                    return [term, "--title", title, "--command", f"bash -lc {shlex.quote(shell_cmd)}"]
                if term == "mate-terminal":
                    return [term, "--title", title, "--", "bash", "-lc", shell_cmd]
                if term == "terminator":
                    return [term, "--title", title, "-e", f"bash -lc {shlex.quote(shell_cmd)}"]
                if term == "xterm":
                    return [term, "-T", title, "-e", "bash", "-lc", shell_cmd]
        return None

    def _append_log(self, text: str):
        if QThread.currentThread() != QApplication.instance().thread():
            self.log_signal.emit(text)
            return
        self._append_log_ui(text)

    def _append_log_ui(self, text: str):
        self._log_history.append(text)
        if self.log_dialog.isVisible():
            self._log_buffer.append(text)
            if not self._log_flush_timer.isActive():
                self._log_flush_timer.start()
        if "[ERROR]" in text:
            self.alert_lbl.setText("ERROR")
            self.alert_lbl.setStyleSheet("background:#dc2626; color:white; padding:4px 10px; border-radius:8px;")
        elif "[WARN]" in text:
            self.alert_lbl.setText("WARN")
            self.alert_lbl.setStyleSheet("background:#f59e0b; color:white; padding:4px 10px; border-radius:8px;")
        if self._debug_logs_to_stdout:
            print(text, flush=True)

    def _flush_log_buffer(self):
        if not self._log_buffer:
            self._log_flush_timer.stop()
            return
        batch = "\n".join(self._log_buffer)
        self._log_buffer.clear()
        self.log_dialog.append(batch)

    def _register_debug_button(self, button: Optional[QAbstractButton], context_provider):
        if button is None:
            return
        button.setCheckable(True)
        self._debug_buttons.append((button, context_provider))
        button.toggled.connect(
            lambda checked, btn=button, ctx_fn=context_provider: self._on_debug_button_toggled(
                btn, ctx_fn, checked
            )
        )

    def _on_debug_button_toggled(self, button: QAbstractButton, context_provider, checked: bool):
        context = context_provider() if callable(context_provider) else str(context_provider)
        if checked:
            for other_btn, _ in self._debug_buttons:
                if other_btn is button:
                    continue
                if other_btn.isChecked():
                    other_btn.blockSignals(True)
                    other_btn.setChecked(False)
                    other_btn.blockSignals(False)
            self._debug_active_button = button
            self._debug_active_context = context
            self._set_debug_state(True, context)
            return
        if self._debug_active_button is button:
            self._set_debug_state(False, context)
            self._debug_active_button = None
            self._debug_active_context = None

    def _set_debug_state(self, enabled: bool, context: str):
        if enabled:
            self._debug_logs_to_stdout = True
            if self.tab_exp is not None:
                self.tab_exp.set_debug_enabled(context.startswith("Experimentos"))
            if self.tab_tfm is not None:
                self.tab_tfm.set_debug_enabled(context.startswith("Evidencias (TFM)"))
            self._append_log("[UI] Botón: Debug logs -> terminal (ON)")
            self._append_log(f"[DEBUG] Activado desde: {context}")
            return
        if self._debug_logs_to_stdout:
            self._append_log("[UI] Botón: Debug logs -> terminal (OFF)")
            self._append_log(f"[DEBUG] Desactivado desde: {context}")
        self._debug_logs_to_stdout = False
        if self.tab_exp is not None:
            self.tab_exp.set_debug_enabled(False)
        if self.tab_tfm is not None:
            self.tab_tfm.set_debug_enabled(False)

    def toggle_logs(self):
        if self.log_dialog.isVisible():
            self.log_dialog.hide()
        else:
            self._append_log("[UI] Botón: Logs")
            if self._log_history:
                self.log_dialog.text.setPlainText("\n".join(self._log_history))
                self.log_dialog.text.moveCursor(self.log_dialog.text.textCursor().End)
            self.log_dialog.show()
            self.log_dialog.raise_()
            self.log_dialog.activateWindow()

    def _on_cmd_started(self, tag: str):
        if tag in {"ROBOT", "ROBOT-TEST", "ROBOT-CTRL", "ROBOT-DIAG", "PICK"}:
            self._robot_busy_count += 1
            self.tab_robot.set_busy(True)
            self.tab_main.set_busy(True)

    def _on_cmd_finished(self, tag: str, rc: int):
        if tag in {"ROBOT", "ROBOT-TEST", "ROBOT-CTRL", "ROBOT-DIAG", "PICK"}:
            self._robot_busy_count = max(0, self._robot_busy_count - 1)
            if self._robot_busy_count == 0:
                self.tab_robot.set_busy(False)
                self.tab_robot.unlock_buttons()
                self.tab_main.set_busy(False)
                self.tab_main.unlock_buttons()

    def _schedule_robot_state_check(self):
        if self._robot_state_check_running:
            return
        self._robot_state_check_running = True

        def _worker():
            ros2_running = ros2_control_running()
            gz_running = gz_sim_running()
            self.robot_state_signal.emit(ros2_running, gz_running)
            self._robot_state_check_running = False

        threading.Thread(target=_worker, daemon=True).start()

    def _apply_robot_state(self, ros2_running: bool, gz_running: bool):
        running = ros2_running or (gz_running and ros_clock_available())
        self.tab_robot.set_robot_state(running, ros2_running, gz_running)
        self.tab_main.set_robot_state(running, ros2_running, gz_running)
        self.tab_sim.set_robot_leds(ros2_running, gz_running)

    def _on_joint_state_payload(self, payload: dict):
        if self.tab_robot is not None:
            self.tab_robot.update_joint_state(payload)

    def _on_target_selected(self, target: SelectedTarget):
        self.selected_target = target
        self.tab_cam.set_selected(
            target.object_name,
            f"Selección: {target.object_name} @ ({target.world_x:.2f},{target.world_y:.2f})"
        )
        self.tab_cam.update_selection_info(target)

    def _world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        for tile in self.tab_cam.tiles:
            if "/camera_overhead/image" not in (tile.frame.topic or ""):
                continue
            pix = table_xy_to_pixel(x, y, tile.frame.w, tile.frame.h)
            if pix:
                return pix
        return (0, 0)

    def _closest_table_target(self) -> Optional[SelectedTarget]:
        best = None
        best_dist = float("inf")
        for name, (ox, oy, oz) in OBJECT_POSITIONS.items():
            if not visible_table_object(name, (ox, oy, oz)):
                continue
            dx = ox - UR5_BASE_X
            dy = oy - UR5_BASE_Y
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best = (name, ox, oy, oz)
                best_dist = dist
        if not best:
            return None
        px, py = self._world_to_pixel(best[1], best[2])
        return SelectedTarget(
            px=px,
            py=py,
            world_x=best[1],
            world_y=best[2],
            world_z=best[3],
            object_name=best[0],
            object_x=best[1],
            object_y=best[2],
            object_z=best[3],
        )

    def _set_hold_script(self, script_path: str):
        self._hold_script = script_path
        self._hold_pause_until = time.time() + 2.0

    def _pause_hold(self, seconds: float):
        self._hold_pause_until = time.time() + seconds

    def _update_calibration_state(self) -> None:
        self.tab_robot.set_calibrated(self._calibrated)
        self.tab_main.set_calibrated(self._calibrated)
        self.tab_cam.obj_panel.setVisible(self._calibrated)
        if self._calibrated:
            self.tab_cam.update_objects()
        if not self._calibrated:
            self.tab_cam.obj_panel.sel_lbl.setText("Selecciona y calibra primero.")

    def _set_calibrated(self, state: bool) -> None:
        self._calibrated = state
        self._update_calibration_state()

    def _sim_hold_tick(self):
        now = time.time()
        if not self._auto_hold_enabled:
            return
        if now - self._hold_last_run < self._auto_hold_interval:
            return
        if now < self._hold_pause_until:
            return
        if not gz_sim_running() or ros2_control_running():
            return
        if not self._hold_script or not os.path.isfile(self._hold_script):
            return
        proc = self._hold_proc.get("hold_home")
        if proc and proc.poll() is None:
            return
        if proc and proc.poll() is not None:
            self._hold_proc.pop("hold_home", None)
        cmd = bash_preamble(WS_DIR) + f"HOME_QUIET=1 timeout 6 '{self._hold_script}' || true"
        self.runner.run_stream("ROBOT", cmd, proc_slot=self._hold_proc, key="hold_home")
        self._hold_last_run = now

    def _start_calibration(self):
        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        global TABLE_CAM_INFO
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
        TABLE_CAM_INFO = None
        self._calib_active = False
        self._calib_points = []
        self._calib_queue = []
        self._append_log("[PICK] Calibración automática: leyendo cámara/mesa desde SDF.")
        mesa_tile = next((t for t in self.tab_cam.tiles if t.name.lower().startswith("mesa")), None)
        if not mesa_tile:
            self._append_log("[PICK] Calibración: no hay tile 'Mesa'.")
            return
        if not mesa_tile.frame.qimg:
            self._append_log("[PICK] Calibración: la cámara Mesa no tiene imagen.")
            return
        mesa_tile.show_grid = True
        mesa_tile.update_frame(mesa_tile.frame.topic, mesa_tile.frame.qimg, mesa_tile.frame.w, mesa_tile.frame.h, mesa_tile.frame.fps)
        success = self._calib_from_gz()
        QTimer.singleShot(2000, self._hide_calib_grid)
        if success:
            self._set_calibrated(True)
        else:
            self._set_calibrated(False)

    def _hide_calib_grid(self):
        for t in self.tab_cam.tiles:
            if t.name.lower().startswith("mesa"):
                t.show_grid = False
                if t.frame.qimg:
                    t.update_frame(t.frame.topic, t.frame.qimg, t.frame.w, t.frame.h, t.frame.fps)
                break

    def _calib_from_gz(self):
        if not gz_sim_running():
            self._append_log("[PICK] Calibración: Gazebo no está activo.")
            return False
        world_path = self.tab_sim.world_combo.currentText().strip()
        sdf_path = ""
        if world_path and os.path.isfile(world_path):
            sdf_path = world_path
        else:
            if world_path:
                cand = os.path.join(WORLDS_DIR, world_path)
                if os.path.isfile(cand):
                    sdf_path = cand
                elif not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
                    sdf_path = cand + ".sdf"
            if not sdf_path:
                for cand in DEFAULT_WORLD_CANDIDATES:
                    if os.path.isfile(cand):
                        sdf_path = cand
                        break
        if sdf_path:
            world_name = read_world_name(sdf_path)
        else:
            world_name = GZ_WORLD
        if sdf_path:
            self._append_log(f"[PICK] Calibración: usando SDF {sdf_path}")
        env_base = (
            f"export GZ_IP='{os.environ.get('GZ_IP','127.0.0.1')}' ; "
            f"export GZ_TRANSPORT_IP='{os.environ.get('GZ_TRANSPORT_IP','127.0.0.1')}' ; "
        )
        partitions = []
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        if part:
            partitions.append(part)
        partitions.append("")  # fallback sin partición
        out = ""

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
                elif isinstance(data, list):
                    for item in data:
                        if isinstance(item, dict) and isinstance(item.get("pose"), list):
                            poses = item["pose"]
                            break
                if isinstance(poses, list) and poses:
                    return poses
            return []

        def _auto_calib_from_sdf(text: str) -> Optional[Dict[str, List[float]]]:
            def _parse_pose(elem: Optional[ET.Element]) -> Tuple[float, float, float, float, float, float]:
                if elem is None or not elem.text:
                    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                parts = elem.text.strip().split()
                if len(parts) < 6:
                    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                vals = [float(v) for v in parts[:6]]
                return tuple(vals)  # type: ignore[return-value]

            def _rpy_to_mat(roll: float, pitch: float, yaw: float) -> List[List[float]]:
                cr = math.cos(roll)
                sr = math.sin(roll)
                cp = math.cos(pitch)
                sp = math.sin(pitch)
                cyaw = math.cos(yaw)
                syaw = math.sin(yaw)
                return [
                    [cyaw * cp, cyaw * sp * sr - syaw * cr, cyaw * sp * cr + syaw * sr],
                    [syaw * cp, syaw * sp * sr + cyaw * cr, syaw * sp * cr - cyaw * sr],
                    [-sp, cp * sr, cp * cr],
                ]

            def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
                return [
                    [
                        a[0][0] * b[0][c] + a[0][1] * b[1][c] + a[0][2] * b[2][c]
                        for c in range(3)
                    ],
                    [
                        a[1][0] * b[0][c] + a[1][1] * b[1][c] + a[1][2] * b[2][c]
                        for c in range(3)
                    ],
                    [
                        a[2][0] * b[0][c] + a[2][1] * b[1][c] + a[2][2] * b[2][c]
                        for c in range(3)
                    ],
                ]

            def _mat_vec(mat: List[List[float]], vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
                return (
                    mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
                    mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
                    mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
                )

            try:
                root = ET.fromstring(text)
            except ET.ParseError:
                return None

            model = None
            for mdl in root.findall(".//model"):
                if mdl.get("name") == "camera_overhead":
                    model = mdl
                    break
            if model is None:
                return None

            model_pose = _parse_pose(model.find("pose"))
            model_R = _rpy_to_mat(model_pose[3], model_pose[4], model_pose[5])
            model_t = (model_pose[0], model_pose[1], model_pose[2])

            sensor = None
            link_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            for link in model.findall("link"):
                lp = _parse_pose(link.find("pose"))
                for s in link.findall(".//sensor"):
                    topic = (s.findtext("topic") or "").strip()
                    if topic == "/camera_overhead":
                        sensor = s
                        link_pose = lp
                        break
                if sensor is not None:
                    break
            if sensor is None:
                sensors = model.findall(".//sensor")
                sensor = sensors[0] if sensors else None
            if sensor is None:
                return None

            sensor_pose = _parse_pose(sensor.find("pose"))
            link_R = _rpy_to_mat(link_pose[3], link_pose[4], link_pose[5])
            link_t = (link_pose[0], link_pose[1], link_pose[2])
            sensor_R = _rpy_to_mat(sensor_pose[3], sensor_pose[4], sensor_pose[5])
            sensor_t = (sensor_pose[0], sensor_pose[1], sensor_pose[2])

            ml_R = _mat_mul(model_R, link_R)
            ml_t = tuple(a + b for a, b in zip(model_t, _mat_vec(model_R, link_t)))
            cam_R = _mat_mul(ml_R, sensor_R)
            cam_t = tuple(a + b for a, b in zip(ml_t, _mat_vec(ml_R, sensor_t)))

            camera = sensor.find("camera")
            if camera is None:
                return None
            hfov_text = camera.findtext("horizontal_fov")
            width_text = camera.findtext("image/width")
            height_text = camera.findtext("image/height")
            if not (hfov_text and width_text and height_text):
                return None
            hfov = float(hfov_text)
            width = int(float(width_text))
            height = int(float(height_text))
            if width <= 0 or height <= 0:
                return None
            vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * (height / float(width)))
            fx = width / (2.0 * math.tan(hfov / 2.0))
            fy = height / (2.0 * math.tan(vfov / 2.0))

            cam_x, cam_y, cam_z = cam_t
            def world_to_pixel(wx: float, wy: float, wz: float) -> Tuple[float, float]:
                vx = wx - cam_x
                vy = wy - cam_y
                vz = wz - cam_z
                # camera frame = R^T * v
                xcam = cam_R[0][0] * vx + cam_R[1][0] * vy + cam_R[2][0] * vz
                ycam = cam_R[0][1] * vx + cam_R[1][1] * vy + cam_R[2][1] * vz
                zcam = cam_R[0][2] * vx + cam_R[1][2] * vy + cam_R[2][2] * vz
                if xcam <= 1e-6:
                    return (0.0, 0.0)
                u = (width / 2.0) - (ycam / xcam) * fx
                v = (height / 2.0) - (zcam / xcam) * fy
                return (u, v)

            z_table = 0.775
            w1 = (-TABLE_SIZE_X / 2.0, TABLE_SIZE_Y / 2.0)
            w2 = (TABLE_SIZE_X / 2.0, TABLE_SIZE_Y / 2.0)
            w3 = (TABLE_SIZE_X / 2.0, -TABLE_SIZE_Y / 2.0)
            w4 = (-TABLE_SIZE_X / 2.0, -TABLE_SIZE_Y / 2.0)
            p1 = world_to_pixel(w1[0], w1[1], z_table)
            p2 = world_to_pixel(w2[0], w2[1], z_table)
            p3 = world_to_pixel(w3[0], w3[1], z_table)
            p4 = world_to_pixel(w4[0], w4[1], z_table)
            n1 = pixel_to_norm(p1[0], p1[1], width, height)
            n2 = pixel_to_norm(p2[0], p2[1], width, height)
            n3 = pixel_to_norm(p3[0], p3[1], width, height)
            n4 = pixel_to_norm(p4[0], p4[1], width, height)
            hom = compute_homography(
                [(n1[0], n1[1]), (n2[0], n2[1]), (n3[0], n3[1]), (n4[0], n4[1])],
                [(w1[0], w1[1]), (w2[0], w2[1]), (w3[0], w3[1]), (w4[0], w4[1])],
            )
            if not hom:
                return None
            return {
                "mode": "homography",
                "h": hom,
                "camera": {
                    "position": [cam_x, cam_y, cam_z],
                    "rotation": cam_R,
                    "fx": fx,
                    "fy": fy,
                    "cx": width / 2.0,
                    "cy": height / 2.0,
                    "width": width,
                    "height": height,
                },
            }

        # Intento 1: gz topic pose/info (JSON)
        for part in partitions:
            env = env_base + (f"export GZ_PARTITION='{part}' ; " if part else "")
            cmd = (
                bash_preamble(WS_DIR)
                + env
                + f"gz topic -e -n 1 -t '/world/{world_name}/pose/info' --json-output"
            )
            res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
            poses = _parse_pose_json(res.stdout)
            if poses:
                out = res.stdout
                break

        # Intento 2: fallback con gz service (si pose/info no responde)
        if not out:
            for part in partitions:
                env = env_base + (f"export GZ_PARTITION='{part}' ; " if part else "")
                cmd = (
                    bash_preamble(WS_DIR)
                    + env
                    + f"gz service -s /world/{world_name}/state "
                    "--reqtype gz.msgs.Empty --reptype gz.msgs.WorldState --timeout 2000 --req ''"
                )
                res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
                if res.returncode == 0 and res.stdout:
                    out = res.stdout
                    break

        calib_ok = False
        sdf_text = ""
        if sdf_path:
            try:
                sdf_text = Path(sdf_path).read_text(encoding="utf-8", errors="ignore")
            except Exception:
                sdf_text = ""
        if not sdf_text:
            self._append_log("[PICK] Calibración: SDF no disponible; revisa la ruta del mundo.")
        if sdf_text:
            calib = _auto_calib_from_sdf(sdf_text)
            if calib and calib.get("mode") == "homography":
                ensure_dir(os.path.dirname(TABLE_CALIB_PATH))
                with open(TABLE_CALIB_PATH, "w", encoding="utf-8") as f:
                    json.dump(calib, f, indent=2)
                load_table_calib()
                global TABLE_PIXEL_AFFINE
                global TABLE_PIXEL_RECT
                global TABLE_PIXEL_HOMOGRAPHY
                global TABLE_CAM_INFO
                TABLE_PIXEL_AFFINE = None
                TABLE_PIXEL_RECT = None
                TABLE_PIXEL_HOMOGRAPHY = calib["h"]
                TABLE_CAM_INFO = calib.get("camera") if isinstance(calib, dict) else None
                calib_ok = True
                self._append_log("[PICK] Calibración: homografía derivada del SDF.")
            else:
                self._append_log("[PICK] Calibración: no pude extraer cámara_overhead del SDF.")

        if not out:
            if sdf_text:
                updated = 0
                for name in list(OBJECT_POSITIONS.keys()):
                    pattern = (
                        r'<model\\s+name="' + re.escape(name) + r'"[^>]*>.*?<pose>\\s*'
                        r'([\\-0-9.eE]+)\\s+([\\-0-9.eE]+)\\s+([\\-0-9.eE]+)'
                    )
                    m = re.search(pattern, sdf_text, re.DOTALL)
                    if m:
                        x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                        OBJECT_POSITIONS[name] = (x, y, z)
                        updated += 1
                if updated:
                    save_object_positions()
                    self.tab_cam.update_objects()
                    self._append_log("[PICK] Calibración: usando poses estáticas del SDF.")
                    self._append_log(f"[PICK] Calibración: {updated} objetos actualizados.")
                    if not calib_ok:
                        self._append_log("[PICK] Calibración: homografía no disponible (SDF).")
                    return calib_ok
            if calib_ok:
                return True
            self._append_log("[PICK] Calibración: no pude leer estado del mundo.")
            return False
        updated = 0
        poses = _parse_pose_json(out)
        if poses:
            for pose in poses:
                if not isinstance(pose, dict):
                    continue
                name = pose.get("name")
                pos = pose.get("position") or {}
                if not name or not isinstance(pos, dict):
                    continue
                if name not in OBJECT_POSITIONS:
                    continue
                try:
                    x = float(pos.get("x"))
                    y = float(pos.get("y"))
                    z = float(pos.get("z"))
                except (TypeError, ValueError):
                    continue
                OBJECT_POSITIONS[name] = (x, y, z)
                updated += 1
        else:
            for name in list(OBJECT_POSITIONS.keys()):
                pattern = (
                    r'name: "' + re.escape(name) + r'"\\s*pose\\s*\\{[^}]*position\\s*\\{'
                    r'\\s*x: ([\\-0-9.eE]+)\\s*y: ([\\-0-9.eE]+)\\s*z: ([\\-0-9.eE]+)'
                )
                m = re.search(pattern, out)
                if m:
                    x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                    OBJECT_POSITIONS[name] = (x, y, z)
                    updated += 1
        if updated:
            save_object_positions()
            self.tab_cam.update_objects()
            self._append_log(f"[PICK] Calibración: {updated} objetos actualizados.")
            if not calib_ok:
                self._append_log("[PICK] Calibración: homografía no disponible (SDF).")
            return calib_ok
        if calib_ok:
            self._append_log("[PICK] Calibración: sin objetos dinámicos; se mantiene homografía.")
            return True
        self._append_log("[PICK] Calibración: no se encontraron objetos.")
        return False

    def _handle_calib_click(self, px: int, py: int) -> bool:
        if not self._calib_active:
            return False
        self._calib_points.append((f"p{len(self._calib_points)+1}", px, py))
        self._append_log(f"[PICK] Calibración fina: punto {len(self._calib_points)} = ({px},{py})")
        if len(self._calib_points) >= 4:
            self._finish_calibration()
            self._calib_active = False
        return True

    def _cancel_manual_calib(self):
        if self._calib_active and len(self._calib_points) < 4:
            self._calib_active = False
            self._append_log("[PICK] Calibración fina cancelada (timeout).")

    def _finish_calibration(self):
        self._calib_active = False
        if len(self._calib_points) < 4:
            self._append_log("[PICK] Calibración incompleta.")
            return
        p1 = self._calib_points[0][1:]
        p2 = self._calib_points[1][1:]
        p3 = self._calib_points[2][1:]
        p4 = self._calib_points[3][1:]
        w1 = (-TABLE_SIZE_X / 2.0, TABLE_SIZE_Y / 2.0)
        w2 = (TABLE_SIZE_X / 2.0, TABLE_SIZE_Y / 2.0)
        w3 = (TABLE_SIZE_X / 2.0, -TABLE_SIZE_Y / 2.0)
        w4 = (-TABLE_SIZE_X / 2.0, -TABLE_SIZE_Y / 2.0)
        if (
            abs(p2[0] - p1[0]) < 10
            or abs(p3[0] - p2[0]) < 10
            or abs(p4[0] - p3[0]) < 10
        ):
            self._append_log("[PICK] ERROR: puntos demasiado cercanos. Repite.")
            return
        hom = compute_homography(
            [(float(p1[0]), float(p1[1])), (float(p2[0]), float(p2[1])),
             (float(p3[0]), float(p3[1])), (float(p4[0]), float(p4[1]))],
            [(float(w1[0]), float(w1[1])), (float(w2[0]), float(w2[1])),
             (float(w3[0]), float(w3[1])), (float(w4[0]), float(w4[1]))],
        )
        if not hom:
            self._append_log("[PICK] ERROR: homografía inválida. Repite calibración.")
            return
        data = {
            "mode": "homography",
            "h": hom,
        }
        ensure_dir(os.path.dirname(TABLE_CALIB_PATH))
        with open(TABLE_CALIB_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = hom
        self._append_log(f"[PICK] Calibración guardada: {TABLE_CALIB_PATH}")
        for t in self.tab_cam.tiles:
            if t.name.lower().startswith("mesa"):
                t.show_grid = False
                if t.frame.qimg:
                    t.update_frame(t.frame.topic, t.frame.qimg, t.frame.w, t.frame.h, t.frame.fps)
                break

    def infer_grasp_model(self):
        tile = None
        for t in self.tab_cam.tiles:
            if "/camera_overhead/image" in (t.frame.topic or ""):
                tile = t
                break
        if tile is None or not tile.frame.qimg:
            self._append_log("[MODEL] No hay frame de /camera_overhead/image.")
            return
        target = self.selected_target
        if not target or not target.object_name:
            self._append_log("[MODEL] Selecciona un objeto antes de inferir.")
            return
        if not os.path.isfile(INFER_SCRIPT):
            self._append_log(f"[MODEL] ERROR: no existe {INFER_SCRIPT}")
            return
        venv_activate = os.path.join(VISION_DIR, ".venv", "bin", "activate")
        if not os.path.isfile(venv_activate):
            alt_activate = os.path.join(VISION_DIR, ".venv_ai", "bin", "activate")
            if os.path.isfile(alt_activate):
                venv_activate = alt_activate
            else:
                self._append_log(f"[MODEL] ERROR: no existe {venv_activate}")
                return
        ckpt_path = self._resolve_infer_ckpt()
        if not ckpt_path:
            self._append_log(
                "[MODEL] ERROR: no hay checkpoint disponible. "
                "Define INFER_CKPT o genera experiments/*/checkpoints/best.pth."
            )
            return
        if not os.path.isfile(ckpt_path):
            self._append_log(f"[MODEL] ERROR: no existe {ckpt_path}")
            return
        self._append_log(f"[MODEL] Usando checkpoint: {ckpt_path}")
        ensure_dir(LOG_DIR)
        img_path = os.path.join(LOG_DIR, "overhead_last.png")
        out_path = os.path.join(LOG_DIR, "overhead_grasp.json")
        tile.frame.qimg.save(img_path)

        def _worker():
            try:
                def _build_roi_args(roi_size: int) -> str:
                    if not target or roi_size <= 0:
                        return ""
                    px, py = self._pixel_for_target(tile, target)
                    if 0 <= px < tile.frame.w and 0 <= py < tile.frame.h:
                        return f" --roi-cx {px} --roi-cy {py} --roi-size {int(roi_size)}"
                    return ""

                def _run_infer(roi_size: int) -> Optional[Dict[str, object]]:
                    roi_args = _build_roi_args(roi_size)
                    cmd = (
                        f"source '{venv_activate}' && "
                        f"cd '{VISION_DIR}' && "
                        "export PYTHONPATH=\"$PWD/src:${PYTHONPATH:-}\" && "
                        f"python '{INFER_SCRIPT}' --image '{img_path}' --ckpt '{ckpt_path}' --out '{out_path}'{roi_args}"
                    )
                    res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
                    if res.returncode != 0:
                        self._append_log(f"[MODEL] ERROR infer: {res.stderr.strip() or res.stdout.strip()}")
                        return None
                    try:
                        with open(out_path, "r", encoding="utf-8") as f:
                            return json.load(f)
                    except Exception as exc:
                        self._append_log(f"[MODEL] ERROR leyendo grasp: {exc}")
                        return None

                data = _run_infer(INFER_ROI_SIZE)
                if not data:
                    return
                if target and INFER_ROI_SIZE > 0 and INFER_RETRY_ERR_PX > 0:
                    tpx, tpy = self._pixel_for_target(tile, target)
                    dx = float(data["cx"]) - float(tpx)
                    dy = float(data["cy"]) - float(tpy)
                    err = math.hypot(dx, dy)
                    if err > INFER_RETRY_ERR_PX:
                        retry_size = max(64, int(INFER_ROI_SIZE * 0.6))
                        retry = _run_infer(retry_size)
                        if retry:
                            data = retry
                            self._append_log(
                                f"[MODEL] ROI retry: {INFER_ROI_SIZE}px -> {retry_size}px (err {err:.1f}px)."
                            )
                z_target = None
                if target and target.object_z is not None:
                    z_target = target.object_z
                elif target and target.object_name in OBJECT_POSITIONS:
                    z_target = OBJECT_POSITIONS[target.object_name][2]
                world_xy = pixel_to_table_xy(
                    float(data["cx"]),
                    float(data["cy"]),
                    tile.frame.w,
                    tile.frame.h,
                    z_target=z_target,
                )
                corners = self._corners_from_grasp(
                    float(data["cx"]),
                    float(data["cy"]),
                    float(data["w"]),
                    float(data["h"]),
                    float(data["angle_deg"]),
                )
                conf = None
                if isinstance(data, dict):
                    conf = data.get("confidence", data.get("score"))
                grasp_info = {
                    "object_name": target.object_name,
                    "cx": float(data["cx"]),
                    "cy": float(data["cy"]),
                    "w": float(data["w"]),
                    "h": float(data["h"]),
                    "angle_deg": float(data["angle_deg"]),
                    "corners_px": corners,
                    "confidence": conf,
                }
                if world_xy:
                    grasp_info["world_x"] = float(world_xy[0])
                    grasp_info["world_y"] = float(world_xy[1])
                    grasp_info["world_z"] = float(z_target or 0.0)
                if target:
                    grasp_info["err_px"] = (
                        float(data["cx"]) - float(self._pixel_for_target(tile, target)[0]),
                        float(data["cy"]) - float(self._pixel_for_target(tile, target)[1]),
                    )
                    if world_xy:
                        grasp_info["err_xy"] = (
                            float(world_xy[0]) - float(target.world_x),
                            float(world_xy[1]) - float(target.world_y),
                        )
                tile.grasp_rect = (
                    float(data["cx"]),
                    float(data["cy"]),
                    float(data["w"]),
                    float(data["h"]),
                    float(data["angle_deg"]),
                )
                tile.set_grasp_info(grasp_info)
                self._store_last_grasp(target, tile, grasp_info)
                QTimer.singleShot(0, lambda: tile.update_frame(
                    tile.frame.topic, tile.frame.qimg, tile.frame.w, tile.frame.h, tile.frame.fps
                ))
                self._append_log(
                    f"[MODEL] Grasp: cx={data['cx']:.1f} cy={data['cy']:.1f} "
                    f"w={data['w']:.1f} h={data['h']:.1f} ang={data['angle_deg']:.1f}"
                )
                if world_xy:
                    self._append_log(
                        f"[MODEL] Grasp world: x={world_xy[0]:.3f} y={world_xy[1]:.3f} z={float(z_target or 0.0):.3f}"
                    )
                if target and world_xy:
                    self._append_log(
                        f"[MODEL] Err: px=({grasp_info['err_px'][0]:.1f},{grasp_info['err_px'][1]:.1f}) "
                        f"xy=({grasp_info['err_xy'][0]:.3f},{grasp_info['err_xy'][1]:.3f})"
                    )
            except Exception as e:
                self._append_log(f"[MODEL] ERROR leyendo grasp: {e}")

        threading.Thread(target=_worker, daemon=True).start()

    def _pixel_for_target(self, tile, target: SelectedTarget) -> Tuple[int, int]:
        px = int(target.px)
        py = int(target.py)
        if 0 <= px < tile.frame.w and 0 <= py < tile.frame.h:
            return px, py
        if target.object_x is not None and target.object_y is not None and target.object_z is not None:
            pix = world_xyz_to_pixel(target.object_x, target.object_y, target.object_z, tile.frame.w, tile.frame.h)
            if pix:
                return int(pix[0]), int(pix[1])
        pix = table_xy_to_pixel(target.world_x, target.world_y, tile.frame.w, tile.frame.h)
        if pix:
            return int(pix[0]), int(pix[1])
        return px, py

    def _corners_from_grasp(self, cx: float, cy: float, w: float, h: float, angle_deg: float):
        ang = math.radians(angle_deg)
        dx = w / 2.0
        dy = h / 2.0
        corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        pts = []
        for ox, oy in corners:
            x = cx + (ox * math.cos(ang) - oy * math.sin(ang))
            y = cy + (ox * math.sin(ang) + oy * math.cos(ang))
            pts.append([float(x), float(y)])
        return pts

    def _calib_used_label(self) -> str:
        if TABLE_PIXEL_HOMOGRAPHY:
            return "homography"
        if TABLE_PIXEL_AFFINE:
            return "affine"
        if TABLE_PIXEL_RECT:
            return "rect"
        return "none"

    def _store_last_grasp(self, target: SelectedTarget, tile, grasp_info: Dict[str, object]) -> None:
        obj = target.object_name
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        world_xy = None
        if "world_x" in grasp_info and "world_y" in grasp_info:
            world_xy = [float(grasp_info["world_x"]), float(grasp_info["world_y"])]
        payload = {
            "object_name": obj,
            "timestamp": ts,
            "image_topic": tile.frame.topic,
            "cx": float(grasp_info.get("cx", 0.0)),
            "cy": float(grasp_info.get("cy", 0.0)),
            "w": float(grasp_info.get("w", 0.0)),
            "h": float(grasp_info.get("h", 0.0)),
            "angle_deg": float(grasp_info.get("angle_deg", 0.0)),
            "corners_px": grasp_info.get("corners_px"),
            "confidence": grasp_info.get("confidence"),
            "world_xy_est": world_xy,
            "calib_used": self._calib_used_label(),
        }
        self._last_grasp_by_object[obj] = payload
        self._last_grasp_ts[obj] = time.time()
        report_dir = os.path.expanduser("~/TFM/reports/ros2_pickplace")
        ensure_dir(report_dir)
        out_path = os.path.join(report_dir, f"last_grasp_{obj}.json")
        try:
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except Exception as exc:
            self._append_log(f"[MODEL] ERROR guardando grasp: {exc}")

    def _resolve_infer_ckpt(self) -> str:
        selected = self.tab_robot.get_selected_ckpt() if self.tab_robot else ""
        if selected and os.path.isfile(selected):
            return selected
        env_ckpt = os.environ.get("INFER_CKPT", "").strip()
        if env_ckpt and os.path.isfile(env_ckpt):
            return env_ckpt
        if os.path.isfile(INFER_CKPT):
            return INFER_CKPT
        ckpt = self._best_ckpt_from_summary()
        if ckpt:
            return ckpt
        return self._find_latest_ckpt()

    def _refresh_infer_models(self):
        options = []
        default_label = ""
        if os.path.isfile(VISION_SUMMARY):
            try:
                with open(VISION_SUMMARY, "r", encoding="utf-8") as handle:
                    reader = csv.DictReader(handle)
                    for row in reader:
                        if not row:
                            continue
                        exp_id = (row.get("exp_id") or "").strip()
                        model_name = (row.get("model") or "").strip()
                        modality = (row.get("modality") or "").strip().upper()
                        ckpt = (row.get("ckpt_best") or "").strip()
                        if modality != "RGB":
                            continue
                        if not ckpt or not os.path.isfile(ckpt):
                            continue
                        label = f"{exp_id} ({model_name})" if exp_id else ckpt
                        options.append((label, ckpt))
                        if exp_id.startswith("EXP1_SIMPLE_RGB_seed0"):
                            default_label = label
                        elif not default_label and exp_id.startswith("EXP1_SIMPLE_RGB"):
                            default_label = label
            except Exception:
                options = []
        if not options:
            options = self._scan_rgb_checkpoints()
        if not options:
            self._append_log("[MODEL] No hay modelos RGB disponibles.")
        self.tab_robot.set_model_options(options, default_label or None)

    def _scan_rgb_checkpoints(self) -> List[tuple]:
        options = []
        root = os.path.join(VISION_DIR, "experiments")
        if not os.path.isdir(root):
            return options
        for dirpath, _dirnames, filenames in os.walk(root):
            if "BORRAR" in dirpath:
                continue
            if "checkpoints" not in dirpath:
                continue
            if "best.pth" not in filenames:
                continue
            path = os.path.join(dirpath, "best.pth")
            if not self._ckpt_matches_rgb(path):
                continue
            exp_dir = Path(path).resolve().parent.parent
            label = exp_dir.name
            options.append((label, path))
        options.sort(key=lambda x: x[0])
        return options

    def _best_ckpt_from_summary(self) -> str:
        if not os.path.isfile(VISION_SUMMARY):
            return ""
        try:
            with open(VISION_SUMMARY, "r", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                best = None
                for row in reader:
                    if not row:
                        continue
                    model_name = (row.get("model") or "").strip().lower()
                    modality = (row.get("modality") or "").strip().upper()
                    ckpt = (row.get("ckpt_best") or "").strip()
                    if model_name and model_name not in ("simple_cnn", "resnet18"):
                        continue
                    if modality != "RGB":
                        continue
                    try:
                        score = float(row.get("val_success") or "0")
                    except ValueError:
                        score = 0.0
                    if not ckpt or not os.path.isfile(ckpt):
                        continue
                    if best is None or score > best[0]:
                        best = (score, ckpt)
                if best:
                    return best[1]
        except Exception:
            return ""
        return ""

    def _find_latest_ckpt(self) -> str:
        root = os.path.join(VISION_DIR, "experiments")
        if not os.path.isdir(root):
            return ""
        latest_path = ""
        latest_time = -1.0
        for dirpath, _dirnames, filenames in os.walk(root):
            if "BORRAR" in dirpath:
                continue
            if "checkpoints" not in dirpath:
                continue
            if "best.pth" not in filenames:
                continue
            path = os.path.join(dirpath, "best.pth")
            if not self._ckpt_matches_rgb(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if mtime > latest_time:
                latest_time = mtime
                latest_path = path
        return latest_path

    def _ckpt_matches_rgb(self, ckpt_path: str) -> bool:
        exp_dir = Path(ckpt_path).resolve().parent.parent
        cfg_path = exp_dir / "config_used.yaml"
        if yaml is not None and cfg_path.is_file():
            try:
                payload = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
            except Exception:
                payload = {}
            data = payload.get("data", {}) if isinstance(payload, dict) else {}
            use_depth = data.get("use_depth")
            if use_depth is False or str(use_depth).lower() == "false":
                return True
        name = exp_dir.name.upper()
        if "RGBD" not in name:
            return True
        return False

    def pick_and_place_selected(self):
        if self.tab_robot._busy:
            self._append_log("[PICK] Ocupado. Espera a que termine la acción actual.")
            return
        self._append_log("[UI] Botón: Pick & Place (selección)")
        self.tab_robot.lock_button(self.tab_robot.btn_pick)
        target = self.selected_target
        if not target or not target.object_name:
            self._append_log("[PICK] No hay selección válida. Ajusta la cámara o recalibra.")
            self.unlock_robot_buttons_signal.emit()
            return
        obj = target.object_name
        grasp = self._last_grasp_by_object.get(obj)
        if not grasp:
            self._append_log(f"[PICK] No hay last_grasp para {obj}. Pulsa Inferir (modelo) primero.")
            self.unlock_robot_buttons_signal.emit()
            return

        def _worker():
            if not robot_control_available():
                self._append_log("[PICK] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
                self.unlock_robot_buttons_signal.emit()
                return
            if ros2_control_running() and not gz_sim_running():
                active, err = list_active_controllers()
                if err or not active or "joint_trajectory_controller" not in active:
                    self._append_log("[PICK] ros2_control sin controladores activos. Pulsa Start UR5 controllers.")
                    self.unlock_robot_buttons_signal.emit()
                    return
            self._run_pickplace_from_grasp(target, grasp)

        threading.Thread(target=_worker, daemon=True).start()

    def _start_pick_demo(self, obj: str):
        self._append_log(f"[PICK] Pick&Place demo -> {obj}")
        if not self._selection_matches_object(obj):
            self._append_log(
                "[PICK] Selección fuera de la zona esperada. No se inicia el demo para evitar teletransportar."
            )
            self.unlock_robot_buttons_signal.emit()
            return
        to_basket = not self._object_in_basket(obj)
        if to_basket:
            self._append_log("[PICK] Demo: mesa -> cesta")
        else:
            self._append_log("[PICK] Demo: cesta -> mesa")
        self._set_hold_script(os.path.join(SCRIPTS_DIR, "ur5_go_basket_pose.sh"))
        self._pause_hold(8.0)
        attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/attach"
        detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/detach"
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        gz_env = build_gz_env(part)
        attach_cmd = f"gz topic -t '{attach_topic}' -m gz.msgs.Empty -p 'unused: true'"
        detach_cmd = f"gz topic -t '{detach_topic}' -m gz.msgs.Empty -p 'unused: true'"
        if to_basket:
            steps = [
                f"'{SCRIPTS_DIR}/ur5_open_gripper.sh' || true",
                "sleep 0.6",
                f"'{SCRIPTS_DIR}/ur5_go_table_pose.sh' || true",
                "sleep 0.6",
                f"'{SCRIPTS_DIR}/ur5_close_gripper.sh' || true",
                "sleep 0.4",
                f"echo 'Attach -> {obj}'",
                f"{attach_cmd} || true",
                "sleep 0.3",
                "echo 'Attach (reintento)'",
                f"{attach_cmd} || true",
                "sleep 0.5",
                f"'{SCRIPTS_DIR}/ur5_go_basket_pose.sh' || true",
                "sleep 0.5",
                f"'{SCRIPTS_DIR}/ur5_open_gripper.sh' || true",
                "sleep 0.3",
                f"echo 'Detach -> {obj}'",
                f"{detach_cmd} || true",
                "sleep 0.25",
                "echo 'Detach (reintento)'",
                f"{detach_cmd} || true",
            ]
        else:
            steps = [
                f"'{SCRIPTS_DIR}/ur5_open_gripper.sh' || true",
                "sleep 0.6",
                f"'{SCRIPTS_DIR}/ur5_go_basket_pose.sh' || true",
                "sleep 0.6",
                f"'{SCRIPTS_DIR}/ur5_close_gripper.sh' || true",
                "sleep 0.4",
                f"echo 'Attach -> {obj}'",
                f"{attach_cmd} || true",
                "sleep 0.3",
                "echo 'Attach (reintento)'",
                f"{attach_cmd} || true",
                "sleep 0.5",
                f"'{SCRIPTS_DIR}/ur5_go_table_pose.sh' || true",
                "sleep 0.5",
                f"'{SCRIPTS_DIR}/ur5_open_gripper.sh' || true",
                "sleep 0.3",
                f"echo 'Detach -> {obj}'",
                f"{detach_cmd} || true",
                "sleep 0.25",
                "echo 'Detach (reintento)'",
                f"{detach_cmd} || true",
            ]
        cmd = bash_preamble(WS_DIR) + (gz_env if gz_env else "") + " ; ".join(steps)
        self.runner.run_stream("PICK", cmd)
        if to_basket:
            QTimer.singleShot(1500, lambda: self._move_object_to_basket(obj))
        else:
            QTimer.singleShot(1500, lambda: self._move_object_to_table(obj))
        QTimer.singleShot(12000, lambda: self._set_hold_script(
            os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")
        ))

    def _log_pickplace(self, text: str) -> None:
        try:
            with open(self._pickplace_log, "a", encoding="utf-8") as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {text}\n")
        except Exception:
            pass

    def _read_joint_positions(self) -> Optional[List[float]]:
        cmd_type = bash_preamble(WS_DIR) + "ros2 topic list -t | awk '/^\\/joint_states[[:space:]]/ {print $2}'"
        res_type = subprocess.run(["bash", "-lc", cmd_type], text=True, capture_output=True)
        msg_type = (res_type.stdout or "").strip().strip("[]")
        if not msg_type:
            return self._read_joint_positions_gz()
        cmd = bash_preamble(WS_DIR) + f"timeout 2 ros2 topic echo /joint_states {shlex.quote(msg_type)} --once"
        res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
        if res.returncode != 0:
            self._append_log(
                f"[PICK] ERROR leyendo /joint_states ({msg_type}): {res.stderr.strip() or res.stdout.strip()}"
            )
            return self._read_joint_positions_gz()
        if yaml is None:
            self._append_log("[PICK] ERROR: yaml no disponible para parsear joint_states.")
            return None
        try:
            data = yaml.safe_load(res.stdout)
        except Exception as exc:
            self._append_log(f"[PICK] ERROR parseando joint_states: {exc}")
            return None
        names = data.get("name") or []
        pos = data.get("position") or []
        if not names or not pos or len(names) != len(pos):
            self._append_log("[PICK] ERROR: joint_states incompleto.")
            return None
        mapping = dict(zip(names, pos))
        ordered = []
        for j in UR5_JOINT_NAMES:
            if j not in mapping:
                self._append_log(f"[PICK] ERROR: falta {j} en joint_states.")
                return None
            ordered.append(float(mapping[j]))
        return ordered

    def _read_joint_positions_gz(self) -> Optional[List[float]]:
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        gz_env = build_gz_env(part)
        cmd_list = gz_env + "gz topic -l"
        res_list = subprocess.run(["bash", "-lc", cmd_list], text=True, capture_output=True)
        if res_list.returncode != 0:
            self._append_log("[PICK] ERROR: no puedo listar topics de Gazebo.")
            return None
        candidates = []
        for line in (res_list.stdout or "").splitlines():
            if "joint_state" in line and "ur5" in line:
                candidates.append(line.strip())
        if not candidates:
            self._append_log("[PICK] ERROR: no hay joint_state en Gazebo.")
            return None
        topic = candidates[0]
        cmd = gz_env + f"gz topic -e -n 1 -t '{topic}' --json-output"
        res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
        if res.returncode != 0:
            self._append_log("[PICK] ERROR leyendo joint_state de Gazebo.")
            return None
        raw = (res.stdout or "").strip()
        if not raw:
            self._append_log("[PICK] ERROR: joint_state Gazebo vacío.")
            return None
        try:
            data = json.loads(raw[raw.find("{"):])
        except Exception as exc:
            self._append_log(f"[PICK] ERROR parseando joint_state Gazebo: {exc}")
            return None
        mapping = self._parse_joint_state_payload(data)
        if not mapping:
            self._append_log("[PICK] ERROR: joint_state Gazebo sin joints.")
            return None
        ordered = []
        for j in UR5_JOINT_NAMES:
            if j not in mapping:
                self._append_log(f"[PICK] ERROR: falta {j} en joint_state Gazebo.")
                return None
            ordered.append(float(mapping[j]))
        self._append_log("[PICK] joint_states desde Gazebo.")
        return ordered

    def _parse_joint_state_payload(self, data: Dict[str, object]) -> Dict[str, float]:
        if "msg" in data and isinstance(data["msg"], dict):
            data = data["msg"]
        mapping: Dict[str, float] = {}
        names = data.get("name")
        positions = data.get("position")
        if isinstance(names, list) and isinstance(positions, list) and len(names) == len(positions):
            for name, pos in zip(names, positions):
                try:
                    mapping[str(name)] = float(pos)
                except Exception:
                    continue
            return mapping
        joints = data.get("joint")
        if isinstance(joints, list):
            for joint in joints:
                if not isinstance(joint, dict):
                    continue
                name = joint.get("name")
                if not name:
                    continue
                pos = joint.get("position")
                if isinstance(pos, list) and pos:
                    pos = pos[0]
                if pos is None:
                    axis1 = joint.get("axis1") or {}
                    if isinstance(axis1, dict):
                        pos = axis1.get("position")
                        if pos is None:
                            pos = axis1.get("angle")
                try:
                    mapping[str(name)] = float(pos)
                except Exception:
                    continue
        return mapping

    def _send_joint_trajectory(self, positions: List[float], duration: float, tag: str) -> bool:
        sec = max(0.5, float(duration))
        sec_i = int(sec)
        nsec_i = int((sec - sec_i) * 1e9)
        msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": UR5_JOINT_NAMES,
            "points": [{
                "positions": [round(p, 5) for p in positions],
                "time_from_start": {"sec": sec_i, "nanosec": nsec_i},
            }],
        }
        topic = detect_arm_trajectory_topic()
        cmd = (
            bash_preamble(WS_DIR)
            + "timeout 8 ros2 topic pub --once "
            + f"{shlex.quote(topic)} trajectory_msgs/msg/JointTrajectory "
            + shlex.quote(json.dumps(msg))
        )
        res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
        if res.returncode != 0:
            self._append_log(f"[PICK] ERROR enviando trayectoria ({tag}): {res.stderr.strip() or res.stdout.strip()}")
            return False
        return True

    def _run_pickplace_from_grasp(self, target: SelectedTarget, grasp: Dict[str, object]) -> None:
        obj = target.object_name
        self._append_log(f"[PICK] Pick&Place (selección) -> {obj}")
        self._log_pickplace(f"[PICK] start obj={obj}")
        q_current = self._read_joint_positions()
        if not q_current:
            self._append_log("[PICK] ERROR: no hay joints actuales.")
            self.unlock_robot_buttons_signal.emit()
            return
        world_xy = grasp.get("world_xy_est")
        if not world_xy:
            world_xy = [target.world_x, target.world_y]
        wx, wy = float(world_xy[0]), float(world_xy[1])
        wz = float(target.object_z if target.object_z is not None else target.world_z)
        yaw = float(grasp.get("angle_deg", 0.0)) * math.pi / 180.0
        rot = rot_z(yaw) @ rot_x(math.pi)
        z_pre = wz + 0.12
        z_grasp = wz + 0.02
        z_lift = wz + 0.15
        basket_x, basket_y, basket_z = BASKET_DROP
        basket_pre = basket_z + 0.12
        basket_drop = basket_z + 0.03

        poses = [
            ("pregrasp", (wx, wy, z_pre)),
            ("grasp", (wx, wy, z_grasp)),
            ("lift", (wx, wy, z_lift)),
            ("basket_pre", (basket_x, basket_y, basket_pre)),
            ("basket_drop", (basket_x, basket_y, basket_drop)),
            ("basket_pre_2", (basket_x, basket_y, basket_pre)),
        ]
        q_plan = []
        q_seed = q_current
        for label, pos in poses:
            q_sol, err, ok = ik_ur5(pos, rot, q_seed)
            self._append_log(f"[PICK] IK {label}: err={err:.4f} ok={ok}")
            if not ok or err > 0.15:
                self._append_log(f"[PICK] ERROR: IK falla en {label} (err={err:.3f}).")
                self.unlock_robot_buttons_signal.emit()
                return
            q_plan.append((label, q_sol))
            q_seed = q_sol

        def _run_script(path: str, tag: str) -> bool:
            cmd = bash_preamble(WS_DIR) + f"timeout 8 '{path}' || true"
            res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
            if res.returncode != 0:
                self._append_log(f"[PICK] WARN script {tag}: {res.stderr.strip() or res.stdout.strip()}")
                return False
            return True

        if not _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"), "open_gripper"):
            self._append_log("[PICK] WARN: fallo al abrir gripper.")
        if not self._send_joint_trajectory(q_plan[0][1].tolist(), 4.0, "pregrasp"):
            self.unlock_robot_buttons_signal.emit()
            return
        if not self._send_joint_trajectory(q_plan[1][1].tolist(), 3.0, "grasp"):
            self.unlock_robot_buttons_signal.emit()
            return
        _run_script(os.path.join(SCRIPTS_DIR, "ur5_close_gripper.sh"), "close_gripper")
        attach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/attach"
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        gz_env = build_gz_env(part)
        attach_cmd = f"gz topic -t '{attach_topic}' -m gz.msgs.Empty -p 'unused: true'"
        subprocess.run(["bash", "-lc", gz_env + attach_cmd], text=True, capture_output=True)

        self._send_joint_trajectory(q_plan[2][1].tolist(), 3.0, "lift")
        self._send_joint_trajectory(q_plan[3][1].tolist(), 4.0, "basket_pre")
        self._send_joint_trajectory(q_plan[4][1].tolist(), 3.0, "basket_drop")
        _run_script(os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"), "open_gripper")
        detach_topic = f"{GRIPPER_ATTACH_PREFIX}/{obj}/detach"
        detach_cmd = f"gz topic -t '{detach_topic}' -m gz.msgs.Empty -p 'unused: true'"
        subprocess.run(["bash", "-lc", gz_env + detach_cmd], text=True, capture_output=True)
        self._send_joint_trajectory(q_plan[5][1].tolist(), 3.0, "basket_pre_2")
        QTimer.singleShot(1200, lambda: self._move_object_to_basket(obj))
        self._append_log(f"[PICK] SUCCESS: {obj} a cesta.")
        self._log_pickplace(f"[PICK] success obj={obj}")
        self.unlock_robot_buttons_signal.emit()

    def _selection_matches_object(self, obj: str) -> bool:
        target = self.selected_target
        if not target or obj not in OBJECT_POSITIONS:
            return False
        base_x, base_y, _ = OBJECT_POSITIONS[obj]
        dx = target.world_x - base_x
        dy = target.world_y - base_y
        dist = math.hypot(dx, dy)
        return dist <= 0.35

    def _move_object_to_basket(self, obj_name: str):
        if obj_name not in DYNAMIC_OBJECTS:
            self._append_log(f"[PICK] WARN: {obj_name} es estático, puede no moverse.")
        if obj_name in OBJECT_POSITIONS and not self._object_in_basket(obj_name):
            self._last_table_positions[obj_name] = OBJECT_POSITIONS[obj_name]
        x, y, z = BASKET_DROP
        if obj_name in OBJECT_POSITIONS:
            OBJECT_POSITIONS[obj_name] = (x, y, z)
        req = (
            f"name: '{obj_name}' "
            f"position {{x: {x}, y: {y}, z: {z}}} "
            f"orientation {{w: 1.0}}"
        )
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        env = build_gz_env(part)
        cmd = (
            env +
            f"gz service -s /world/{GZ_WORLD}/set_pose "
            "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
            f"--req \"{req}\""
        )
        self.runner.run_stream("PICK", cmd)

    def _move_object_to_table(self, obj_name: str):
        if obj_name not in DYNAMIC_OBJECTS:
            self._append_log(f"[PICK] WARN: {obj_name} es estático, puede no moverse.")
        target = self._last_table_positions.get(obj_name)
        if not target and obj_name in TABLE_OBJECTS:
            target = TABLE_OBJECTS[obj_name]
        if not target:
            self._append_log(f"[PICK] WARN: no hay posición de mesa para {obj_name}.")
            return
        x, y, z = target
        OBJECT_POSITIONS[obj_name] = (x, y, z)
        req = (
            f"name: '{obj_name}' "
            f"position {{x: {x}, y: {y}, z: {z}}} "
            f"orientation {{w: 1.0}}"
        )
        part = resolve_gz_partition(self.tab_sim.gz_partition)
        env = build_gz_env(part)
        cmd = (
            env +
            f"gz service -s /world/{GZ_WORLD}/set_pose "
            "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
            f"--req \"{req}\""
        )
        self.runner.run_stream("PICK", cmd)

    def _object_in_basket(self, obj_name: str) -> bool:
        pos = OBJECT_POSITIONS.get(obj_name)
        if not pos:
            return False
        dx = pos[0] - BASKET_DROP[0]
        dy = pos[1] - BASKET_DROP[1]
        return math.hypot(dx, dy) < 0.12

    def _on_image(self, topic: str, qimg: QImage, w: int, h: int, fps: float):
        # update tiles
        for t in self.tab_cam.tiles:
            t.update_frame(topic, qimg, w, h, fps)
        now = time.time()
        if (now - self._last_obj_update) >= self._obj_update_interval:
            self._last_obj_update = now
            self.tab_cam.update_objects()

    def _on_bridge_started(self):
        now = time.time()
        if now - self._last_bridge_camera_start < 6.0:
            return
        self._last_bridge_camera_start = now
        self._append_log("[AUTO] Bridge activo, re-lanzando cámaras front/tras.")
        QTimer.singleShot(900, self.tab_cam.auto_start_cameras)

    def start_all(self):
        self._append_log("[UI] Botón: START ALL")
        self._append_log("[START] Secuencia START ALL: stop -> start Gazebo -> start Bridge")
        self._rotate_startup_logs()
        if self.chk_logs.isChecked():
            self._open_log_terminals()
        self.stop_all()
        # Gazebo
        self.tab_sim.start_gazebo()
        # Bridge tras un pequeño delay
        QTimer.singleShot(1200, self.tab_sim.start_bridge)
        QTimer.singleShot(2200, self.tab_cam.auto_start_cameras)

    def _auto_hold_sim_pose(self):
        if not gz_sim_running() or ros2_control_running():
            return
        script_path = os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")
        if not os.path.isfile(script_path):
            self._append_log(f"[ROBOT] ERROR: no existe {script_path}")
            return
        cmd = bash_preamble(WS_DIR) + f"HOME_QUIET=1 timeout 12 '{script_path}' || true"
        self._append_log("[ROBOT] Auto: enviando HOME (sim) para estabilizar el brazo.")
        self.runner.run_stream("ROBOT", cmd)
        self._hold_last_run = time.time()

    def stop_all(self):
        self._append_log("[UI] Botón: STOP ALL")
        self._append_log("[STOP] Deteniendo BAG, BRIDGE, GAZEBO...")
        self.tab_sim.stop_bag()
        self.tab_sim.stop_bridge()
        self.tab_sim.stop_gazebo()

    def kill_hard(self):
        self._append_log("[UI] Botón: KILL HARD")
        self._append_log("[KILL] pkill hard (rosbag/bridge/gz)")
        subprocess.run(["bash","-lc",
            "pkill -f 'ros2 bag record' || true; "
            "pkill -f 'ros_gz_bridge' || true; "
            "pkill -f 'parameter_bridge' || true; "
            "pkill -f 'gz sim' || true; "
            "pkill -f 'gzserver' || true; "
            "pkill -f 'gzclient' || true; "
        ], check=False)

    def close_panel(self):
        reply = QMessageBox.question(
            self, "Cerrar panel",
            "¿Seguro que quieres cerrar el panel?\n(Recomendación: STOP ALL antes.)",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            try:
                self.ros.stop()
            except Exception:
                pass
            try:
                kill_script = os.path.join(SCRIPTS_DIR, "kill_all.sh")
                if os.path.isfile(kill_script):
                    subprocess.run(["bash", "-lc", f"'{kill_script}' || true"], check=False)
            except Exception:
                pass
            QApplication.instance().quit()

    def closeEvent(self, event):
        self.close_panel()
        event.ignore()

def main():
    app = QApplication(sys.argv)
    font = app.font()
    font.setPointSize(6)
    app.setFont(font)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
