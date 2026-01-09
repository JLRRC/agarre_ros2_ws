#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/main_panel.py
# Summary: Main Qt panel for UR5 simulation control and camera monitoring.
"""Qt control panel for UR5 simulation, bridge, cameras, and evidence capture."""
import os
import re
import sys
import math
import time
import signal
import shutil
import shlex
import json
import random
import threading
import subprocess
import csv
import xml.etree.ElementTree as ET
from collections import deque
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Dict, List, Tuple, Set, Callable

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, QPointF, QRectF, QProcess
from PyQt5.QtGui import QImage, QPixmap, QGuiApplication, QPainter, QPen, QColor, QPolygonF, QFont, QFontMetrics, QTextCursor
# from PyQt5 import ...
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QTabWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QTextEdit, QTextBrowser, QGroupBox,
    QComboBox, QLineEdit, QFileDialog, QCheckBox, QToolButton,
    QAbstractButton, QSizePolicy, QMessageBox, QScrollArea, QSplitter, QDialog,
    QSlider, QDoubleSpinBox, QSpinBox, QProgressBar, QPlainTextEdit
)

try:
    import yaml
except Exception:
    yaml = None

from panel_config import *
from panel_utils import *

# =========================
# UI: Camera tile
# =========================
@dataclass
class Frame:
    """Container for a single camera frame and metadata."""

    qimg: Optional[QImage] = None
    w: int = 0
    h: int = 0
    fps: float = 0.0
    topic: str = ""

@dataclass
class SelectedTarget:
    """Selection in image space and mapped world coordinates."""

    px: int
    py: int
    world_x: float
    world_y: float
    object_name: str

class ClickableLabel(QLabel):
    """QLabel that emits click coordinates."""

    clicked = pyqtSignal(int, int)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(event.x(), event.y())
        super().mousePressEvent(event)

class ObjectRow(QWidget):
    """Single object row with icon, coordinates and hover highlight."""

    clicked = pyqtSignal(str)
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.pos = (0.0, 0.0, 0.0)
        self.out = False
        self._hover = False
        self._selected = False
        self.setFixedHeight(16)
        self.setMouseTracking(True)
        self._font = QFont()
        self._font.setPointSize(7)
        self._font.setFamily("DejaVu Sans")

    def set_state(self, x: float, y: float, z: float, out: bool):
        self.pos = (x, y, z)
        self.out = out
        self.update()

    def set_selected(self, selected: bool):
        self._selected = selected
        self.update()

    def enterEvent(self, event):
        self._hover = True
        self.update()

    def leaveEvent(self, event):
        self._hover = False
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.name)
        super().mousePressEvent(event)

    def _shape_type(self) -> str:
        nm = self.name.lower()
        if "cilindro" in nm:
            return "circle"
        if "cubo" in nm or "caja" in nm:
            return "square"
        return "rect"

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bg = "#1f2937" if (self._hover or self._selected) else "transparent"
        painter.fillRect(self.rect(), QColor(bg))
        color = OBJECT_COLORS.get(self.name, "#e5e7eb")
        if self.out:
            color = "#9ca3af"
        painter.setPen(QPen(QColor("#0f172a"), 1))
        painter.setBrush(QColor(color))
        x0 = 4
        y0 = 3
        if self._shape_type() == "circle":
            painter.drawEllipse(QPointF(x0 + 5, y0 + 5), 4, 4)
        elif self._shape_type() == "square":
            painter.drawRect(x0, y0, 10, 10)
        else:
            painter.drawRoundedRect(x0, y0 + 2, 12, 6, 2, 2)
        painter.setPen(QColor(color))
        painter.setFont(self._font)
        x, y, _z = self.pos
        status = " (fuera)" if self.out else ""
        text = f"{self.name}: ({x:.2f},{y:.2f}){status}"
        metrics = QFontMetrics(self._font)
        text = metrics.elidedText(text, Qt.ElideRight, self.width() - 28)
        painter.drawText(20, 12, text)
        painter.end()

class ObjectListPanel(QWidget):
    """Stacked list of detected objects with hover highlight and reach status."""

    selected = pyqtSignal(str)
    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: Dict[str, ObjectRow] = {}
        lay = QVBoxLayout()
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(1)
        self.sel_lbl = QLabel("Selección: -")
        self.sel_lbl.setStyleSheet("color:#475569; font-size: 8px; font-weight: 600;")
        lay.addWidget(self.sel_lbl)
        self.list_box = QVBoxLayout()
        self.list_box.setSpacing(0)
        lay.addLayout(self.list_box)
        self.setLayout(lay)
        self.setFixedWidth(220)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Maximum)
        self.setStyleSheet(
            "ObjectListPanel {"
            "  background: #f8fafc;"
            "  border: 1px solid #e2e8f0;"
            "  border-radius: 6px;"
            "}"
        )

    def update_objects(self, objects: Dict[str, Tuple[float, float, float]]):
        if not self.isVisible():
            return
        for name, (x, y, z) in sorted(objects.items()):
            if not visible_table_object(name, (x, y, z)):
                continue
            row = self._rows.get(name)
            if not row:
                row = ObjectRow(name)
                self._rows[name] = row
                self.list_box.addWidget(row)
                row.clicked.connect(self.selected.emit)
            out = object_out_of_reach(x, y)
            row.set_state(x, y, z, out)

    def set_selected(self, name: Optional[str], text: str):
        for obj, row in self._rows.items():
            row.set_selected(obj == name)
        self.sel_lbl.setText(text)

class CameraTile(QWidget):
    """Widget that displays a single camera stream with controls."""

    clicked = pyqtSignal(int, int)

    def __init__(self, name: str, ros: RosWorker, log_fn, parent=None):
        super().__init__(parent)
        self.name = name
        self.ros = ros
        self.log_fn = log_fn
        self.frame = Frame()
        self.aspect_ratio = 0.75  # default 4:3 until first frame
        self.selected_px: Optional[Tuple[int, int]] = None
        self.grasp_rect: Optional[Tuple[float, float, float, float, float]] = None
        self._pulse_phase = 0.0
        self._last_ui_ts = 0.0
        try:
            max_fps = float(os.environ.get("PANEL_MAX_FPS", "12"))
            if max_fps <= 0:
                max_fps = 12.0
        except ValueError:
            max_fps = 12.0
        self._min_frame_interval = 1.0 / max_fps

        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        top = QHBoxLayout()
        top.setSpacing(1)
        self.combo = QComboBox()
        self.combo.setEditable(True)
        self.combo.setMinimumWidth(140)
        self.combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.btn_conn = QPushButton("Conn")
        self.btn_disc = QPushButton("Disc")
        self.btn_shot = QPushButton("Shot")
        for b in (self.btn_conn, self.btn_disc, self.btn_shot):
            b.setFixedWidth(22)
        self.btn_conn.setToolTip("Conectar")
        self.btn_disc.setToolTip("Desconectar")
        self.btn_shot.setToolTip("Evidencia")

        self.btn_conn.clicked.connect(self.on_connect)
        self.btn_disc.clicked.connect(self.on_disconnect)
        self.btn_shot.clicked.connect(self.on_shot)

        top.addWidget(QLabel(self.name))
        top.addWidget(self.combo, stretch=1)
        top.addWidget(self.btn_conn)
        top.addWidget(self.btn_disc)
        top.addWidget(self.btn_shot)

        lay.addLayout(top)

        self.lbl = ClickableLabel("sin imagen")
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet("background:#111; color:#aaa; border:1px solid #444;")
        self.lbl.setMinimumSize(140, 80)
        self.lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.lbl.clicked.connect(self._on_click)

        lay.addWidget(self.lbl)

        self.info = QLabel("Topic: - | 0x0 | fps 0.0")
        self.info.setStyleSheet("color:#666;")
        lay.addWidget(self.info)
        self.coord_lbl = QLabel("Cruce: - | Objetivo: -")
        self.coord_lbl.setStyleSheet("color:#4b5563; font-size:10px;")
        lay.addWidget(self.coord_lbl)

        self.setLayout(lay)

        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._pulse_tick)
        self._pulse_timer.start(140)

    def _pulse_tick(self):
        if not self.frame.qimg or not self.selected_px:
            return
        self._pulse_phase += 0.3
        if self._pulse_phase > (2 * math.pi):
            self._pulse_phase -= 2 * math.pi
        self.update_frame(self.frame.topic, self.frame.qimg, self.frame.w, self.frame.h, self.frame.fps, force=True)

    def _on_click(self, x: int, y: int):
        if not self.frame.qimg or self.frame.w <= 0 or self.frame.h <= 0:
            return
        lw = max(1, self.lbl.width())
        lh = max(1, self.lbl.height())
        px = int(x / lw * self.frame.w)
        py = int(y / lh * self.frame.h)
        px = max(0, min(self.frame.w - 1, px))
        py = max(0, min(self.frame.h - 1, py))
        self.selected_px = (px, py)
        self.clicked.emit(px, py)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_label_height()
        if self.frame.qimg:
            self.update_frame(self.frame.topic, self.frame.qimg, self.frame.w, self.frame.h, self.frame.fps)

    def _update_label_height(self):
        width = max(1, self.lbl.width())
        target_h = int(width * self.aspect_ratio)
        self.lbl.setFixedHeight(max(80, target_h))

    def set_topics(self, topics: List[str]):
        cur = self.combo.currentText().strip()
        self.combo.clear()
        for t in topics:
            self.combo.addItem(t)
        if cur:
            self.combo.setCurrentText(cur)

    def on_connect(self):
        t = self.combo.currentText().strip()
        if not t:
            return
        self.log_fn(f"[UI] Botón: Conectar {self.name} -> {t}")
        self.frame.topic = t
        self.ros.subscribe_image(t)
        self.log_fn(f"[ROS] Solicitud subscribe -> {t}")

    def on_disconnect(self):
        t = self.frame.topic or self.combo.currentText().strip()
        if t:
            self.log_fn(f"[UI] Botón: Desconectar {self.name} -> {t}")
            self.ros.unsubscribe_image(t)
        self.frame = Frame()
        self.lbl.setText("sin imagen")
        self.lbl.setPixmap(QPixmap())
        self.info.setText("Topic: - | 0x0 | fps 0.0")

    def update_frame(self, topic: str, qimg: QImage, w: int, h: int, fps: float, force: bool = False):
        # --- PROTECCIÓN: solo procesar el último frame, sin logs por frame ---
        if topic != self.frame.topic:
            return
        self.frame.qimg = qimg
        self.frame.w = w
        self.frame.h = h
        self.frame.fps = fps
        now = time.time()
        if not force and (now - self._last_ui_ts) < self._min_frame_interval:
            return
        self._last_ui_ts = now

        if w > 0 and h > 0:
            self.aspect_ratio = h / max(1, w)
        self._update_label_height()

        # --- No imprimir logs por frame, solo actualizar UI ---
        need_overlay = (
            (self.name.lower().startswith("mesa") and getattr(self, "show_grid", False))
            or self.selected_px
            or self.grasp_rect
        )
        draw = qimg.copy() if need_overlay else qimg
        if self.name.lower().startswith("mesa") and getattr(self, "show_grid", False):
            painter = QPainter(draw)
            painter.setRenderHint(QPainter.Antialiasing)
            pen = QPen(QColor(30, 64, 175, 90))
            pen.setWidth(1)
            painter.setPen(pen)
            step_x = 0.025
            step_y = 0.025
            x = -TABLE_SIZE_X / 2.0
            while x <= (TABLE_SIZE_X / 2.0 + 1e-6):
                p0 = table_xy_to_pixel(x, -TABLE_SIZE_Y / 2.0, w, h)
                p1 = table_xy_to_pixel(x, TABLE_SIZE_Y / 2.0, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                x += step_x
            y = -TABLE_SIZE_Y / 2.0
            while y <= (TABLE_SIZE_Y / 2.0 + 1e-6):
                p0 = table_xy_to_pixel(-TABLE_SIZE_X / 2.0, y, w, h)
                p1 = table_xy_to_pixel(TABLE_SIZE_X / 2.0, y, w, h)
                if p0 and p1:
                    painter.drawLine(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]))
                y += step_y
            painter.setPen(QPen(QColor(30, 64, 175, 160)))
            for label_x in (-0.4, 0.0, 0.4):
                p = table_xy_to_pixel(label_x, -TABLE_SIZE_Y / 2.0, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] + 12, f"x={label_x:.1f}")
            for label_y in (-0.3, 0.0, 0.3):
                p = table_xy_to_pixel(-TABLE_SIZE_X / 2.0, label_y, w, h)
                if p:
                    painter.drawText(p[0] + 3, p[1] - 3, f"y={label_y:.1f}")
            painter.end()
        if self.selected_px:
            painter = QPainter(draw)
            pen = QPen(QColor("#22c55e"))
            pen.setWidth(1)
            painter.setPen(pen)
            cx, cy = self.selected_px
            painter.drawLine(cx - 6, cy, cx + 6, cy)
            painter.drawLine(cx, cy - 6, cx, cy + 6)
            painter.setBrush(QColor(34, 197, 94, 40))
            painter.drawEllipse(QPointF(cx, cy), 6, 6)
            painter.end()

        if self.grasp_rect:
            gcx, gcy, gw, gh, gangle = self.grasp_rect
            painter = QPainter(draw)
            pen = QPen(QColor("#f59e0b"))
            pen.setWidth(2)
            painter.setPen(pen)
            ang = gangle * math.pi / 180.0
            dx = gw / 2.0
            dy = gh / 2.0
            corners = [
                (-dx, -dy),
                (dx, -dy),
                (dx, dy),
                (-dx, dy),
            ]
            pts = []
            for ox, oy in corners:
                x = gcx + (ox * float(math.cos(ang)) - oy * float(math.sin(ang)))
                y = gcy + (ox * float(math.sin(ang)) + oy * float(math.cos(ang)))
                pts.append(QPointF(x, y))
            painter.drawPolygon(QPolygonF(pts))
            painter.end()

        pix = QPixmap.fromImage(draw)
        pix = pix.scaled(self.lbl.width(), self.lbl.height(), Qt.IgnoreAspectRatio, Qt.FastTransformation)
        self.lbl.setPixmap(pix)
        self.info.setText(f"Topic: {topic} | {w}x{h} | fps {fps:.1f}")

    def set_selected_pixel(self, px: int, py: int):
        self.selected_px = (px, py)
        if self.frame.qimg:
            self.update_frame(self.frame.topic, self.frame.qimg, self.frame.w, self.frame.h, self.frame.fps, force=True)
        # keep coords label updated if world info present
        if self.coord_lbl and self.selected_px:
            # maintain placeholder until CamerasTab calls set_selection_info
            pass

    def set_selection_info(self, target: Optional["SelectedTarget"]) -> None:
        if not target:
            self.coord_lbl.setText("Cruce: - | Objetivo: -")
            return
        obj_pos = OBJECT_POSITIONS.get(target.object_name)
        obj_text = f"{target.object_name}"
        if obj_pos:
            obj_text += f" ({obj_pos[0]:.2f},{obj_pos[1]:.2f})"
        self.coord_lbl.setText(
            f"Cruce: ({target.world_x:.2f},{target.world_y:.2f}) | Objetivo: {obj_text}"
        )
    def on_shot(self):
        if not self.frame.qimg:
            self.log_fn(f"[EVID] {self.name}: no hay frame.")
            return
        self.log_fn(f"[UI] Botón: Evidencia {self.name}")
        ensure_dir(FIG_DIR)
        fn = f"{now_tag()}_{self.name}_{safe_topic_name(self.frame.topic)}.png"
        out = os.path.join(FIG_DIR, fn)
        self.frame.qimg.save(out)
        self.log_fn(f"[EVID] Guardado: {out}")

# =========================
# Tabs
# =========================
class SimulationTab(QWidget):
    """UI tab for Gazebo, bridge, and rosbag management."""

    bridge_started = pyqtSignal()
    state_changed = pyqtSignal()

    def __init__(self, runner: CmdRunner, ros: RosWorker, log_fn, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn

        self.procs: Dict[str, subprocess.Popen] = {}
        self.gz_partition: str = ""
        self._gz_running = False
        self._bridge_running = False
        self._bag_running = False
        self._sys_last_total: Optional[int] = None
        self._sys_last_idle: Optional[int] = None
        self._sys_logged_once = False

        self.led_gz = QLabel()
        self.led_bridge = QLabel()
        self.led_clock = QLabel()
        self.led_bag = QLabel()
        self.led_ros2 = QLabel()
        self.led_ur5 = QLabel()
        self.lbl_cpu = QLabel("N/A")
        self.lbl_mem = QLabel("N/A")
        self.lbl_load = QLabel("N/A")
        self.btn_debug = QPushButton("Debug logs -> terminal")
        self.btn_debug.setCheckable(True)

        self.world_combo = QComboBox()
        self.world_combo.setEditable(True)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Auto", "GUI", "Headless"])

        self.bridge_edit = QLineEdit(BRIDGE_BASE_YAML)
        self.lazy_chk = QComboBox()
        self.lazy_chk.addItems(["Lazy ON (recomendado)", "Lazy OFF"])

        self.bag_name = QLineEdit(f"demo_{now_tag()}")
        self.bag_topics = QLineEdit(
            "/camera_overhead/image /camera_north/image /camera_lateral/image "
            "/camera_east/image /camera_west/image"
        )

        self._build_ui()

        # LEDs a OFF por defecto
        set_led(self.led_gz, "off")
        set_led(self.led_bridge, "off")
        set_led(self.led_clock, "off")
        set_led(self.led_bag, "off")
        set_led(self.led_ros2, "off")
        set_led(self.led_ur5, "off")

        # Timer clock LED (real)
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._refresh_clock_led)
        self.clock_timer.start(400)
        self.sys_timer = QTimer(self)
        self.sys_timer.timeout.connect(self._refresh_sys_stats)
        self.sys_timer.start(2000)
        self._refresh_sys_stats()
        self._refresh_buttons()

    def _refresh_buttons(self):
        self.btn_gz_start.setEnabled(not self._gz_running)
        self.btn_gz_stop.setEnabled(self._gz_running)
        self.btn_gz_reset.setEnabled(self._gz_running)
        self.btn_br_start.setEnabled(self._gz_running and not self._bridge_running)
        self.btn_br_stop.setEnabled(self._bridge_running)
        self.btn_bag_start.setEnabled(self._bridge_running and not self._bag_running)
        self.btn_bag_stop.setEnabled(self._bag_running)
        self.state_changed.emit()

    def set_robot_leds(self, ros2_running: bool, gz_running: bool):
        ros2_ready = ros2_running or (gz_running and ros_clock_available())
        set_led(self.led_ros2, "on" if ros2_ready else "off")
        set_led(self.led_ur5, "on" if gz_running else "off")

    def _build_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(1, 1, 1, 1)
        layout.setSpacing(1)

        left = QVBoxLayout()
        left.setSpacing(1)

        # Mundo
        g_world = QGroupBox("Mundo (Gazebo)")
        g_world.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        wlay = QVBoxLayout()
        wlay.setContentsMargins(1, 1, 1, 1)
        wlay.setSpacing(1)
        self._fill_worlds()

        row = QHBoxLayout()
        row.setSpacing(1)
        row.addWidget(QLabel("Selecciona el mundo (SDF/WORLD):"))
        row.addStretch(1)
        wlay.addLayout(row)
        wlay.addWidget(self.world_combo)

        btn_browse = QPushButton("Buscar mundo...")
        btn_browse.clicked.connect(self._browse_world)
        wlay.addWidget(btn_browse)
        g_world.setLayout(wlay)
        left.addWidget(g_world)

        # Gazebo controls
        g_gz = QGroupBox("Gazebo")
        g_gz.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        gzlay = QVBoxLayout()
        gzlay.setContentsMargins(1, 1, 1, 1)
        gzlay.setSpacing(1)

        rowm = QHBoxLayout()
        rowm.setSpacing(1)
        rowm.addWidget(QLabel("Modo Gazebo:"))
        rowm.addWidget(self.mode_combo)
        gzlay.addLayout(rowm)

        rbtn = QHBoxLayout()
        rbtn.setSpacing(1)
        self.btn_gz_start = QPushButton("✅ Lanzar Gazebo")
        self.btn_gz_stop = QPushButton("🛑 Detener Gazebo")
        self.btn_gz_reset = QPushButton("🔁 Reset World")
        self.btn_gz_logs = QPushButton("📂 Logs")
        self.btn_gz_view = QPushButton("📝 Ver log Gazebo")
        self.btn_gz_start.clicked.connect(self.start_gazebo)
        self.btn_gz_stop.clicked.connect(self.stop_gazebo)
        self.btn_gz_reset.clicked.connect(self.reset_world)
        self.btn_gz_logs.clicked.connect(self.open_log)
        self.btn_gz_view.clicked.connect(self.open_gz_log)
        rbtn.addWidget(self.btn_gz_start)
        rbtn.addWidget(self.btn_gz_stop)
        gzlay.addLayout(rbtn)
        rbtn2 = QHBoxLayout()
        rbtn2.setSpacing(1)
        rbtn2.addWidget(self.btn_gz_reset)
        rbtn2.addWidget(self.btn_gz_logs)
        rbtn2.addWidget(self.btn_gz_view)
        gzlay.addLayout(rbtn2)

        g_gz.setLayout(gzlay)
        left.addWidget(g_gz)

        # Bridge
        g_br = QGroupBox("Bridge ROS 2 ↔ Gazebo (YAML)")
        g_br.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        brlay = QVBoxLayout()
        brlay.setContentsMargins(1, 1, 1, 1)
        brlay.setSpacing(1)
        brlay.addWidget(QLabel("YAML base de cámaras (parameter_bridge):"))
        brlay.addWidget(self.bridge_edit)

        rowb = QHBoxLayout()
        rowb.setSpacing(1)
        self.btn_br_start = QPushButton("✅ Lanzar bridge")
        self.btn_br_stop = QPushButton("🛑 Detener bridge")
        self.btn_br_start.clicked.connect(self.start_bridge)
        self.btn_br_stop.clicked.connect(self.stop_bridge)
        rowb.addWidget(self.btn_br_start)
        rowb.addWidget(self.btn_br_stop)
        brlay.addLayout(rowb)

        brlay.addWidget(self.lazy_chk)
        g_br.setLayout(brlay)
        left.addWidget(g_br)

        # Rosbag
        g_bag = QGroupBox("Rosbag PRO (evidencias)")
        g_bag.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        baglay = QVBoxLayout()
        baglay.setContentsMargins(1, 1, 1, 1)
        baglay.setSpacing(1)
        baglay.addWidget(QLabel("Nombre bag (se guardará en bags/):"))
        baglay.addWidget(self.bag_name)
        baglay.addWidget(QLabel("Tópicos a grabar (espacio separado):"))
        baglay.addWidget(self.bag_topics)

        rowbag = QHBoxLayout()
        rowbag.setSpacing(1)
        self.btn_bag_start = QPushButton("● Start bag")
        self.btn_bag_stop = QPushButton("■ Stop bag")
        self.btn_bag_start.clicked.connect(self.start_bag)
        self.btn_bag_stop.clicked.connect(self.stop_bag)
        rowbag.addWidget(self.btn_bag_start)
        rowbag.addWidget(self.btn_bag_stop)
        baglay.addLayout(rowbag)

        g_bag.setLayout(baglay)
        left.addWidget(g_bag)

        # no stretch: keep compact

        # Status panel (moved near cameras)
        self.status_panel = QWidget()
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(1)
        g_status = QGroupBox("Estado")
        g_status.setFlat(True)
        g_status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        slay = QGridLayout()
        slay.setContentsMargins(1, 1, 1, 1)
        slay.setHorizontalSpacing(3)
        slay.setVerticalSpacing(1)

        slay.addWidget(QLabel("Gazebo"), 0, 0)
        slay.addWidget(self.led_gz, 0, 1)

        slay.addWidget(QLabel("Bridge"), 1, 0)
        slay.addWidget(self.led_bridge, 1, 1)

        slay.addWidget(QLabel("/clock en ROS"), 2, 0)
        slay.addWidget(self.led_clock, 2, 1)

        slay.addWidget(QLabel("Rosbag grabando"), 3, 0)
        slay.addWidget(self.led_bag, 3, 1)

        slay.addWidget(QLabel("ros2_control"), 4, 0)
        slay.addWidget(self.led_ros2, 4, 1)

        slay.addWidget(QLabel("UR5 (sim)"), 5, 0)
        slay.addWidget(self.led_ur5, 5, 1)

        g_status.setLayout(slay)
        right.addWidget(g_status)

        g_sys = QGroupBox("Sistema")
        g_sys.setFlat(True)
        g_sys.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        syslay = QGridLayout()
        syslay.setContentsMargins(1, 1, 1, 1)
        syslay.setHorizontalSpacing(3)
        syslay.setVerticalSpacing(1)
        syslay.addWidget(QLabel("CPU"), 0, 0)
        syslay.addWidget(self.lbl_cpu, 0, 1)
        syslay.addWidget(QLabel("RAM"), 1, 0)
        syslay.addWidget(self.lbl_mem, 1, 1)
        syslay.addWidget(QLabel("Load"), 2, 0)
        syslay.addWidget(self.lbl_load, 2, 1)
        g_sys.setLayout(syslay)
        right.addWidget(g_sys)

        right.addWidget(self.btn_debug)

        g_diag = QGroupBox("Diagnóstico rápido")
        g_diag.setFlat(True)
        g_diag.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        dlay = QVBoxLayout()
        dlay.setContentsMargins(1, 1, 1, 1)
        dlay.setSpacing(1)

        b9  = QPushButton("Listar topics (grep camera/clock)")
        b10 = QPushButton("Echo /clock (1 msg)")
        b11 = QPushButton("Tail gz log (últimas 30 líneas)")
        b12 = QPushButton("Tail bridge log (últimas 30 líneas)")

        b9.clicked.connect(self.diag_topics)
        b10.clicked.connect(self.diag_clock_once)
        b11.clicked.connect(self.diag_tail_gz)
        b12.clicked.connect(self.diag_tail_bridge)

        dlay.addWidget(b9)
        dlay.addWidget(b10)
        dlay.addWidget(b11)
        dlay.addWidget(b12)

        g_diag.setLayout(dlay)
        right.addWidget(g_diag)
        self.status_panel.setLayout(right)

        layout.addLayout(left, 3)

        self.setLayout(layout)

    def _fill_worlds(self):
        self.world_combo.clear()
        # 1) candidatos
        for p in DEFAULT_WORLD_CANDIDATES:
            if os.path.isfile(p):
                self.world_combo.addItem(p)
        # 2) resto
        if os.path.isdir(WORLDS_DIR):
            for fn in sorted(os.listdir(WORLDS_DIR)):
                if fn.endswith(".sdf") or fn.endswith(".world"):
                    p = os.path.join(WORLDS_DIR, fn)
                    if p not in DEFAULT_WORLD_CANDIDATES:
                        self.world_combo.addItem(p)
        # 3) fallback
        if self.world_combo.count() == 0:
            self.world_combo.addItem(DEFAULT_WORLD_CANDIDATES[0])
        self.world_combo.setCurrentIndex(0)

    def _browse_world(self):
        p, _ = QFileDialog.getOpenFileName(self, "Selecciona mundo", WORLDS_DIR, "SDF/WORLD (*.sdf *.world)")
        if p:
            self.world_combo.setCurrentText(p)

    def _effective_mode(self) -> str:
        m = self.mode_combo.currentText().strip().lower()
        if m.startswith("auto"):
            # En remoto, forzar headless aunque exista DISPLAY (evita GUI por SSH).
            if os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_TTY"):
                return "headless"
            # GUI solo si hay DISPLAY local disponible.
            if os.environ.get("DISPLAY"):
                return "gui"
            return "headless"
        if m.startswith("gui"):
            return "gui"
        return "headless"

    def _refresh_clock_led(self):
        ok, age = self.ros.clock_alive()
        set_led(self.led_clock, "on" if ok else "off")

    def _set_stat_label(self, lbl: QLabel, text: str, level: str):
        colors = {
            "ok": "#16a34a",
            "warn": "#f59e0b",
            "error": "#ef4444",
            "na": "#6b7280",
        }
        lbl.setText(text)
        lbl.setStyleSheet(f"color:{colors.get(level, colors['na'])};")

    def _refresh_sys_stats(self):
        cpu_times = read_cpu_times()
        mem = read_meminfo_kb()
        load = read_loadavg()
        if not cpu_times or not mem or not load:
            self._set_stat_label(self.lbl_cpu, "N/A", "na")
            self._set_stat_label(self.lbl_mem, "N/A", "na")
            self._set_stat_label(self.lbl_load, "N/A", "na")
            return

        total, idle = cpu_times
        cpu_pct = None
        if self._sys_last_total is not None and self._sys_last_idle is not None:
            d_total = max(1, total - self._sys_last_total)
            d_idle = max(0, idle - self._sys_last_idle)
            cpu_pct = 100.0 * (1.0 - (d_idle / float(d_total)))
        self._sys_last_total = total
        self._sys_last_idle = idle

        mem_total, mem_avail = mem
        mem_used = mem_total - mem_avail
        mem_pct = 100.0 * (mem_used / float(mem_total)) if mem_total else 0.0
        load1, load5, load15 = load
        cores = os.cpu_count() or 1

        if cpu_pct is None:
            cpu_text = "..."
            cpu_level = "na"
        else:
            cpu_text = f"{cpu_pct:.0f}%"
            cpu_level = "error" if cpu_pct >= 90.0 else "warn" if cpu_pct >= 75.0 else "ok"

        mem_text = f"{kb_to_gb(mem_used):.1f}/{kb_to_gb(mem_total):.1f} GB ({mem_pct:.0f}%)"
        mem_level = "error" if mem_pct >= 90.0 else "warn" if mem_pct >= 80.0 else "ok"

        load_text = f"{load1:.2f} {load5:.2f} {load15:.2f}"
        load_level = (
            "error" if load1 >= cores * 0.95 else "warn" if load1 >= cores * 0.75 else "ok"
        )

        self._set_stat_label(self.lbl_cpu, cpu_text, cpu_level)
        self._set_stat_label(self.lbl_mem, mem_text, mem_level)
        self._set_stat_label(self.lbl_load, load_text, load_level)

        if not self._sys_logged_once and cpu_pct is not None:
            self._sys_logged_once = True
            self.log_fn(
                f"[SYS] CPU {cpu_pct:.0f}%, RAM {kb_to_gb(mem_used):.1f}/{kb_to_gb(mem_total):.1f} GB "
                f"({mem_pct:.0f}%), Load {load_text}"
            )

    def start_gazebo(self):
        self.log_fn("[UI] Botón: Lanzar Gazebo -> iniciar simulación")
        ensure_dir(LOG_DIR)
        world = self.world_combo.currentText().strip()
        mode = self._effective_mode()

        # NUEVA partición por arranque (y la reutilizamos en bridge)
        self.gz_partition = f"ur5pro_{int(time.time())}"
        os.environ["GZ_PARTITION"] = self.gz_partition
        try:
            with open(GZ_PARTITION_FILE, "w", encoding="utf-8") as f:
                f.write(self.gz_partition)
        except Exception as e:
            self.log_fn(f"[WARN] No pude guardar GZ_PARTITION: {e}")

        set_led(self.led_gz, "warn")
        gz_log = os.path.join(LOG_DIR, "gz_server.log" if mode != "gui" else "gz_gui.log")
        rotate_log(gz_log)

        # env fijo para evitar multicast “Network is unreachable”
        gz_ip = os.environ.get("GZ_IP", "").strip()
        gz_transport_ip = os.environ.get("GZ_TRANSPORT_IP", "").strip()
        ip_env = ""
        if gz_ip:
            ip_env += f"export GZ_IP='{gz_ip}' ; "
        if gz_transport_ip:
            ip_env += f"export GZ_TRANSPORT_IP='{gz_transport_ip}' ; "
        env = (
            ip_env +
            f"export GZ_PARTITION='{self.gz_partition}' ; "
            f"export GZ_SIM_RESOURCE_PATH='{MODELS_DIR}:{WORLDS_DIR}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
            "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; "
            "export QT_LOGGING_RULES='qt.qml.*=false'; "
        )

        if mode == "gui":
            gz_cmd = with_line_buffer(f"gz sim -r -v 1 '{world}'")
            filter_cmd = build_log_filter_cmd(GZ_LOG_FILTERS)
            cmd = (
                bash_preamble(WS_DIR) +
                env +
                "unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT QT_OPENGL QT_XCB_GL_INTEGRATION; "
                f"{log_to_file(gz_cmd, gz_log, filter_cmd)}"
            )
        else:
            gz_cmd = with_line_buffer(f"gz sim -s -r -v 1 --headless-rendering '{world}'")
            filter_cmd = build_log_filter_cmd(GZ_LOG_FILTERS)
            cmd = (
                bash_preamble(WS_DIR) +
                env +
                f"env -u DISPLAY __EGL_VENDOR_LIBRARY_FILENAMES='{EGL_VENDOR}' "
                "GZ_RENDER_ENGINE=ogre2 "
                f"{log_to_file(gz_cmd, gz_log, filter_cmd)}"
            )

        # usamos stream y guardamos proc
        self.runner.run_stream("SIM", cmd, proc_slot=self.procs, key="gazebo")
        set_led(self.led_gz, "on")
        self._gz_running = True
        self._refresh_buttons()
        QTimer.singleShot(2600, self._auto_detach_objects)

    def _auto_detach_objects(self):
        if not self._gz_running:
            return
        names = sorted(OBJECT_POSITIONS.keys())
        if not names:
            return
        for name in names:
            self.log_fn(f"[PICK] Detach -> {name}")
        part = resolve_gz_partition(self.gz_partition)
        env = build_gz_env(part)
        detach_cmds = " ; ".join(
            f"gz topic -t '{GRIPPER_ATTACH_PREFIX}/{name}/detach' -m gz.msgs.Empty -p 'unused: true' || true"
            for name in names
        )
        cmd = bash_preamble(WS_DIR) + env + f"for i in 1 2 3; do {detach_cmds}; sleep 0.25; done"
        self.log_fn("[SIM] Detach objetos: liberando piezas del gripper.")
        self.runner.run_stream("SIM", cmd)

    def stop_gazebo(self):
        self.log_fn("[UI] Botón: Detener Gazebo -> cerrar simulación")
        p = self.procs.get("gazebo")
        if p:
            kill_process_group(p, "Gazebo", self.log_fn)
            self.procs.pop("gazebo", None)
        subprocess.run(
            [
                "bash",
                "-lc",
                "pkill -f 'gz sim' || true; pkill -f gzserver || true; pkill -f gzclient || true",
            ],
            check=False,
        )
        set_led(self.led_gz, "off")
        self._gz_running = False
        self._bridge_running = False
        self._bag_running = False
        self._refresh_buttons()

    def start_bridge(self):
        self.log_fn("[UI] Botón: Lanzar bridge -> ros_gz_bridge")
        ensure_dir(LOG_DIR)
        if not self._gz_running:
            self.log_fn("[ERROR] Bridge requiere Gazebo activo. Arranca Gazebo primero.")
            set_led(self.led_bridge, "error")
            return
        base_yaml = self.bridge_edit.text().strip()
        if not os.path.isfile(base_yaml):
            self.log_fn(f"[ERROR] No existe YAML base: {base_yaml}")
            set_led(self.led_bridge, "error")
            return

        if not self.gz_partition:
            env_part = os.environ.get("GZ_PARTITION", "").strip()
            if env_part:
                self.gz_partition = env_part
                self.log_fn(f"[WARN] GZ_PARTITION vacío en panel; usando env: {env_part}")
            else:
                self.log_fn("[ERROR] Bridge requiere Gazebo activo (GZ_PARTITION). Arranca Gazebo primero.")
                set_led(self.led_bridge, "error")
                return

        world = self.world_combo.currentText().strip()
        world_name = read_world_name(world)

        runtime_yaml = os.path.join(LOG_DIR, "bridge_runtime.yaml")
        write_bridge_runtime_yaml(runtime_yaml, world_name, base_yaml)

        set_led(self.led_bridge, "warn")
        br_log = os.path.join(LOG_DIR, "ros_gz_bridge.log")
        rotate_log(br_log)

        gz_ip = os.environ.get("GZ_IP", "").strip()
        gz_transport_ip = os.environ.get("GZ_TRANSPORT_IP", "").strip()
        ip_env = ""
        if gz_ip:
            ip_env += f"export GZ_IP='{gz_ip}' ; "
        if gz_transport_ip:
            ip_env += f"export GZ_TRANSPORT_IP='{gz_transport_ip}' ; "
        ros_log_dir = os.path.join(LOG_DIR, "ros")
        ensure_dir(ros_log_dir)
        env = (
            ip_env +
            f"export GZ_PARTITION='{self.gz_partition or os.environ.get('GZ_PARTITION','')}' ; "
            f"export ROS_LOG_DIR='{ros_log_dir}' ; "
        )

        lazy_on = self.lazy_chk.currentIndex() == 0
        try:
            with open(base_yaml, "r", encoding="utf-8") as f:
                base_txt = f.read()
            if "ROS_TO_GZ" in base_txt and lazy_on:
                self.log_fn("[BRIDGE] Lazy OFF (ROS_TO_GZ requiere bridge activo).")
                lazy_on = False
        except Exception:
            pass
        lazy_arg = " -p lazy:=true " if lazy_on else ""

        bridge_cmd = with_line_buffer(
            "ros2 run ros_gz_bridge parameter_bridge --ros-args "
            + f"-p config_file:='{runtime_yaml}'{lazy_arg}"
        )

        cmd = (
            bash_preamble(WS_DIR)
            + env
            + f"{bridge_cmd} > '{br_log}' 2>&1"
        )

        self.runner.run_stream("BRIDGE", cmd, proc_slot=self.procs, key="bridge")
        set_led(self.led_bridge, "on")
        self.log_fn(f"[INFO] Bridge runtime YAML: {runtime_yaml}")
        self._bridge_running = True
        self.bridge_started.emit()
        self._refresh_buttons()

    def stop_bridge(self):
        self.log_fn("[UI] Botón: Detener bridge")
        p = self.procs.get("bridge")
        if p:
            kill_process_group(p, "Bridge", self.log_fn)
            self.procs.pop("bridge", None)
        subprocess.run(
            [
                "bash",
                "-lc",
                "pkill -f 'ros_gz_bridge' || true; pkill -f parameter_bridge || true",
            ],
            check=False,
        )
        set_led(self.led_bridge, "off")
        self._bridge_running = False
        self._bag_running = False
        self._refresh_buttons()

    def start_bag(self):
        self.log_fn("[UI] Botón: Start bag -> ros2 bag record")
        ensure_dir(BAGS_DIR)
        if not self._bridge_running:
            self.log_fn("[BAG] ERROR: el bridge no está activo.")
            set_led(self.led_bag, "error")
            return
        set_led(self.led_bag, "warn")
        name = self.bag_name.text().strip() or f"demo_{now_tag()}"
        topics_raw = self.bag_topics.text().strip()
        topics, invalid = parse_ros_topics(topics_raw)
        if invalid:
            self.log_fn(f"[BAG] ERROR: tópicos inválidos: {', '.join(invalid)}")
            set_led(self.led_bag, "error")
            return
        if not topics:
            self.log_fn("[BAG] ERROR: no hay tópicos para grabar.")
            set_led(self.led_bag, "error")
            return

        outdir = os.path.join(BAGS_DIR, name)
        if os.path.exists(outdir):
            suffix = 1
            while os.path.exists(f"{outdir}_{suffix}"):
                suffix += 1
            outdir = f"{outdir}_{suffix}"
            self.log_fn(f"[BAG] Carpeta existente, usando: {outdir}")
        cmd = (
            bash_preamble(WS_DIR) +
            "ros2 bag record -o "
            f"'{outdir}' --topics "
            + " ".join(shlex.quote(t) for t in topics)
        )
        self.runner.run_stream("BAG", cmd, proc_slot=self.procs, key="bag")
        set_led(self.led_bag, "on")
        self.log_fn(f"[BAG] Grabando -> {outdir}")
        self._bag_running = True
        self._refresh_buttons()

    def stop_bag(self):
        self.log_fn("[UI] Botón: Stop bag")
        p = self.procs.get("bag")
        if p:
            kill_process_group(p, "Rosbag", self.log_fn)
            self.procs.pop("bag", None)
        subprocess.run(["bash","-lc","pkill -f 'ros2 bag record' || true"], check=False)
        set_led(self.led_bag, "off")
        self._bag_running = False
        self._refresh_buttons()

    def open_log(self):
        self.log_fn("[UI] Botón: Abrir carpeta log")
        ensure_dir(LOG_DIR)
        subprocess.Popen(["bash","-lc", f"xdg-open '{LOG_DIR}' >/dev/null 2>&1 || true"])

    def open_gz_log(self):
        self.log_fn("[UI] Botón: Ver log Gazebo")
        ensure_dir(LOG_DIR)
        gz_server = os.path.join(LOG_DIR, "gz_server.log")
        gz_gui = os.path.join(LOG_DIR, "gz_gui.log")
        candidates = [p for p in (gz_server, gz_gui) if os.path.isfile(p)]
        if not candidates:
            self.log_fn("[ERROR] No existe log de Gazebo aún.")
            return
        log_path = max(candidates, key=os.path.getmtime)
        subprocess.Popen(["bash","-lc", f"xdg-open '{log_path}' >/dev/null 2>&1 || true"])

    def reset_world(self):
        self.log_fn("[UI] Botón: Reset World (STOP+START)")
        self.log_fn("[RESET] STOP+START (Gazebo + Bridge).")
        self.stop_bag()
        self.stop_bridge()
        self.stop_gazebo()
        QTimer.singleShot(400, self.start_gazebo)

    # Diags
    def diag_topics(self):
        self.log_fn("[UI] Botón: Listar topics (camera/clock)")
        cmd = bash_preamble(WS_DIR) + "ros2 topic list | egrep 'camera|/clock' || true"
        self.runner.run_stream("DIAG", cmd)

    def diag_clock_once(self):
        self.log_fn("[UI] Botón: Echo /clock (1 msg)")
        cmd = bash_preamble(WS_DIR) + "timeout 2 ros2 topic echo /clock --once || true"
        self.runner.run_stream("DIAG", cmd)

    def diag_tail_gz(self):
        self.log_fn("[UI] Botón: Tail gz log")
        cmd = (
            f"test -f '{LOG_DIR}/gz_server.log' && tail -n 30 '{LOG_DIR}/gz_server.log' "
            "|| echo 'No existe gz_server.log aún'"
        )
        self.runner.run_stream("DIAG", cmd)

    def diag_tail_bridge(self):
        self.log_fn("[UI] Botón: Tail bridge log")
        cmd = (
            f"test -f '{LOG_DIR}/ros_gz_bridge.log' && tail -n 30 '{LOG_DIR}/ros_gz_bridge.log' "
            "|| echo 'No existe ros_gz_bridge.log aún'"
        )
        self.runner.run_stream("DIAG", cmd)

class CamerasTab(QWidget):
    """UI tab for camera tiles and quick robot actions."""

    topics_found = pyqtSignal(list)
    target_selected = pyqtSignal(object)

    def __init__(
        self,
        runner: CmdRunner,
        ros: RosWorker,
        log_fn,
        calib_start_fn=None,
        calib_click_fn=None,
        show_top_bar: bool = True,
        parent=None,
    ):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn
        self.calib_start_fn = calib_start_fn
        self.calib_click_fn = calib_click_fn
        self._show_top_bar = show_top_bar
        self.obj_panel = ObjectListPanel()
        self._manual_panel = None
        self._status_panel = None
        self._status_placeholder = None
        self._cam_grid = None
        self._auto_connect_pending = False

        self.tiles: List[CameraTile] = []
        self.topics: List[str] = []
        self.obj_panel.selected.connect(self._on_object_selected)

        self.topics_found.connect(self._apply_topics)
        self._build_ui()
        self._selection_target: Optional[SelectedTarget] = None

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        self.top_bar_widget = QWidget()
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(1)
        b1 = QPushButton("Auto-descubrir topics (ROS 2)")
        b2 = QPushButton("Abrir evidencias")
        b3 = QPushButton("Calibrar")
        b3.setCheckable(True)
        b1.clicked.connect(self.discover_topics)
        b2.clicked.connect(self.open_evidences)
        b3.clicked.connect(self.start_calib)
        top.addWidget(b1)
        top.addWidget(b2)
        top.addWidget(b3)
        top.addStretch(1)
        self.top_bar_widget.setLayout(top)
        if self._show_top_bar:
            lay.addWidget(self.top_bar_widget)

        names = ["Mesa", "Frente", "Lateral"]
        for nm in names:
            t = CameraTile(nm, self.ros, self.log_fn)
            t.clicked.connect(lambda x, y, tile=t: self._on_tile_click(tile, x, y))
            self.tiles.append(t)

        # Ajuste inicial para proporciones 4:3, se recalcula al recibir imagen
        self.tiles[0].aspect_ratio = 0.75
        self.tiles[1].aspect_ratio = 0.75
        self.tiles[2].aspect_ratio = 0.75

        mesa_tile = self.tiles[0]
        front_tile = self.tiles[1]
        lateral_tile = self.tiles[2]
        cam_row = QHBoxLayout()
        cam_row.setSpacing(1)
        cam_row.addWidget(front_tile)
        cam_row.addWidget(lateral_tile)

        cam_grid = QGridLayout()
        cam_grid.setHorizontalSpacing(4)
        cam_grid.setVerticalSpacing(2)
        self._status_placeholder = QWidget()
        self._status_placeholder.setMinimumWidth(140)
        cam_grid.addWidget(self._status_placeholder, 0, 0, 2, 1, Qt.AlignTop)
        cam_grid.addWidget(mesa_tile, 0, 1)
        cam_grid.addLayout(cam_row, 1, 1)
        cam_grid.addWidget(self.obj_panel, 0, 2, Qt.AlignTop)
        cam_grid.setColumnStretch(0, 0)
        cam_grid.setColumnStretch(1, 1)
        cam_grid.setColumnStretch(2, 0)
        cam_grid.setRowStretch(0, 2)
        cam_grid.setRowStretch(1, 1)
        self._cam_grid = cam_grid
        lay.addLayout(cam_grid)

        self.setLayout(lay)

    def update_selection_info(self, target: Optional[SelectedTarget]) -> None:
        self._selection_target = target
        for tile in self.tiles:
            if "/camera_overhead/image" in (tile.frame.topic or ""):
                tile.set_selection_info(target)
                break

    def set_manual_panel(self, panel: Optional[QWidget]) -> None:
        if panel is None or self._cam_grid is None:
            return
        if self._manual_panel is panel:
            return
        if self._manual_panel is not None:
            self._cam_grid.removeWidget(self._manual_panel)
            self._manual_panel.setParent(None)
        self._manual_panel = panel
        self._cam_grid.addWidget(panel, 1, 2, Qt.AlignTop)

    def set_status_panel(self, panel: Optional[QWidget]) -> None:
        if panel is None or self._cam_grid is None:
            return
        if self._status_panel is panel:
            return
        if self._status_panel is not None:
            self._cam_grid.removeWidget(self._status_panel)
            self._status_panel.setParent(None)
        if self._status_placeholder is not None:
            self._cam_grid.removeWidget(self._status_placeholder)
            self._status_placeholder.setParent(None)
            self._status_placeholder = None
        self._status_panel = panel
        self._cam_grid.addWidget(panel, 0, 0, 2, 1, Qt.AlignTop)

    def open_evidences(self):
        self.log_fn("[UI] Botón: Abrir evidencias")
        ensure_dir(FIG_DIR)
        subprocess.Popen(["bash","-lc", f"xdg-open '{FIG_DIR}' >/dev/null 2>&1 || true"])

    def start_calib(self):
        if self.calib_start_fn:
            self.calib_start_fn()

    def discover_topics(self):
        def _worker():
            self.log_fn("[UI] Botón: Auto-descubrir topics (ROS 2)")
            self.log_fn("[CAMS] Auto-discovery: buscando topics...")
            cmd = bash_preamble(WS_DIR) + "ros2 topic list -t"
            try:
                out = subprocess.check_output(["bash","-lc", cmd], text=True, stderr=subprocess.STDOUT)
            except Exception as e:
                self.log_fn(f"[CAMS] ERROR discover: {e}")
                return

            topics = []
            for ln in out.splitlines():
                # formato típico: /camera_overhead/image [sensor_msgs/msg/Image]
                if "[sensor_msgs/msg/Image]" in ln:
                    t = ln.split(" [", 1)[0].strip()
                    if t:
                        topics.append(t)

            topics = sorted(set(topics))
            self.topics = topics
            self.log_fn(f"[CAMS] Auto-discovery: {len(topics)} tópicos de imagen")
            self.topics_found.emit(topics)

        threading.Thread(target=_worker, daemon=True).start()

    def _run_script(self, tag: str, script_path: str):
        self.log_fn(f"[UI] Botón: {tag} -> {os.path.basename(script_path)}")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            return
        if ros2_control_running():
            need_gripper = os.path.basename(script_path) in ("ur5_open_gripper.sh", "ur5_close_gripper.sh")
            if not self._robot_ready(require_gripper=need_gripper):
                return
        if not os.path.isfile(script_path):
            self.log_fn(f"[{tag}] ERROR: no existe {script_path}")
            return
        if not os.access(script_path, os.X_OK):
            self.log_fn(f"[{tag}] WARN: {script_path} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"timeout 12 '{script_path}' || true"
        self.runner.run_stream(tag, cmd)

    def _run_robot_test(self):
        self.log_fn("[UI] Botón: Test corto")
        if self._busy:
            self.log_fn("[ROBOT] Ocupado. Espera a que termine la acción actual.")
            return
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            return
        if ros2_control_running():
            active = self._robot_ready(require_gripper=False)
            if active is None:
                return
        s_test = os.path.join(SCRIPTS_DIR, "ur5_quick_test.sh")
        if not os.path.isfile(s_test):
            self.log_fn(f"[ROBOT] ERROR: no existe {s_test}")
            return
        if not os.access(s_test, os.X_OK):
            self.log_fn(f"[ROBOT] WARN: {s_test} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"timeout 20 '{s_test}' || true"
        self.runner.run_stream("ROBOT-TEST", cmd)

    def _robot_ready(self, require_gripper: bool = False) -> Optional[Set[str]]:
        active, err = list_active_controllers()
        if err:
            self.log_fn(f"[ROBOT] ERROR comprobando controladores: {err}")
            return None
        if active is None or not active:
            self.log_fn("[ROBOT] No hay controladores activos (controller_manager no está corriendo).")
            return None
        if "joint_trajectory_controller" not in active:
            self.log_fn("[ROBOT] joint_trajectory_controller no activo.")
            self.log_fn("[ROBOT] Arranca el bringup/ros2_control antes de mover el robot.")
            return None
        if require_gripper and gripper_controller_defined() and "gripper_controller" not in active:
            self.log_fn("[ROBOT] gripper_controller no activo.")
            return None
        return active

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
        if ros2_control_running():
            self.log_fn("[ROBOT] ros2_control ya está en ejecución.")
            return
        ensure_dir(LOG_DIR)
        ros2_log = os.path.join(LOG_DIR, "ros2_control.log")
        rotate_log(ros2_log)
        launch_cmd = with_line_buffer("ros2 launch ur5_bringup ur5_ros2_control.launch.py")
        tee_cmd = f"{STDBUF_PREFIX}tee -a '{ros2_log}'" if STDBUF_PREFIX else f"tee -a '{ros2_log}'"
        cmd = (
            bash_preamble(WS_DIR)
            + f"{launch_cmd} 2>&1 | {tee_cmd}"
        )
        self.runner.run_stream("ROBOT-CTRL", cmd)

    def _apply_topics(self, topics: List[str]):
        if topics:
            for tile in self.tiles:
                tile.set_topics(topics)
        else:
            self.log_fn("[CAMS] No se detectaron tópicos Image. ¿Está el bridge activo?")
            return
        # defaults “bonitos”
        lateral = "/camera_lateral/image" if "/camera_lateral/image" in topics else "/camera_south/image"
        defaults = [
            "/camera_overhead/image",
            "/camera_west/image",
            lateral,
        ]
        for i, tile in enumerate(self.tiles):
            if i < len(defaults) and defaults[i] in topics:
                tile.combo.setCurrentText(defaults[i])
            elif i < len(topics):
                tile.combo.setCurrentText(topics[i])
        if self._auto_connect_pending:
            self._auto_connect_pending = False
            for tile in self.tiles:
                tile.on_connect()

    def auto_start_cameras(self):
        self._auto_connect_pending = True
        self.discover_topics()

    def _on_tile_click(self, tile: CameraTile, px: int, py: int):
        if "/camera_overhead/image" not in (tile.frame.topic or ""):
            self.log_fn("[PICK] Selección solo disponible en cámara cenital (Mesa).")
            return
        if tile.frame.w <= 0 or tile.frame.h <= 0:
            return
        world_x, world_y = pixel_to_table_xy(px, py, tile.frame.w, tile.frame.h)
        if self.calib_click_fn and self.calib_click_fn(px, py):
            return
        obj_name = nearest_table_object(world_x, world_y)
        obj_pos = OBJECT_POSITIONS.get(obj_name)
        if obj_pos:
            dx = world_x - obj_pos[0]
            dy = world_y - obj_pos[1]
            if math.hypot(dx, dy) <= SELECTION_SNAP_DIST:
                # Snap to object center to keep selection aligned after minor mapping drift.
                world_x, world_y = obj_pos[0], obj_pos[1]
        target = SelectedTarget(px=px, py=py, world_x=world_x, world_y=world_y, object_name=obj_name)
        self.target_selected.emit(target)
        self.obj_panel.set_selected(obj_name, f"Selección: {obj_name} @ ({world_x:.2f},{world_y:.2f})")
        self.log_fn(
            f"[PICK] Selección: {obj_name} @ px=({px},{py}) -> world=({world_x:.2f},{world_y:.2f})"
        )

    def _on_object_selected(self, name: str):
        if name not in OBJECT_POSITIONS:
            return
        x, y, _z = OBJECT_POSITIONS[name]
        self.obj_panel.set_selected(name, f"Selección: {name} @ ({x:.2f},{y:.2f})")
        px = 0
        py = 0
        for t in self.tiles:
            if "/camera_overhead/image" in (t.frame.topic or ""):
                pix = table_xy_to_pixel(x, y, t.frame.w, t.frame.h)
                if pix:
                    px, py = pix
                    t.set_selected_pixel(px, py)
                break
        target = SelectedTarget(px=px, py=py, world_x=x, world_y=y, object_name=name)
        self.target_selected.emit(target)

    def set_selected(self, name: Optional[str], text: str):
        self.obj_panel.set_selected(name, text)

    def update_objects(self):
        self.obj_panel.update_objects(get_object_positions())

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
        manual_in_tab: bool = True,
        parent=None,
    ):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
        self.infer_grasp_fn = infer_grasp_fn
        self.pick_place_fn = pick_place_fn
        self.pose_cmd_fn = pose_cmd_fn
        self.hold_pause_fn = hold_pause_fn
        self._busy = False
        self._last_running = False
        self._last_ros2_running = False
        self._last_gz_running = False
        self._locked_buttons: Set[QPushButton] = set()
        self._manual_inflight = False
        self._manual_pending = False
        self._manual_in_tab = bool(manual_in_tab)
        self._calibrated = False
        self._robot_buttons: List[QPushButton] = []
        self.runner.finished.connect(self._on_runner_finished)
        self._build_ui()

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        g = QGroupBox("Robot / DEMO (scripts existentes)")
        g.setFlat(True)
        g.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        gl = QVBoxLayout()
        gl.setContentsMargins(1, 1, 1, 1)
        gl.setSpacing(1)

        self.btn_home = QPushButton("UR5 → HOME")
        self.btn_set_home = QPushButton("Fijar HOME (arranque)")
        self.btn_set_table = QPushButton("Fijar MESA")
        self.btn_set_basket = QPushButton("Fijar CESTA")
        self.btn_table = QPushButton("UR5 → Mesa")
        self.btn_basket = QPushButton("UR5 → Cesta")
        self.btn_infer = QPushButton("Inferir grasp (modelo)")
        self.btn_pick = QPushButton("Pick & Place (demo)")
        self.btn_open = QPushButton("Abrir gripper")
        self.btn_close = QPushButton("Cerrar gripper")
        self.btn_test = QPushButton("Test corto")
        self.btn_eval = QPushButton("Evaluar último experimento")
        self.btn_diag = QPushButton("Diagnóstico robot")

        self.btn_home.clicked.connect(lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")))
        self.btn_set_home.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_set_home_boot.sh"))
        )
        self.btn_set_table.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_set_table_pose.sh"))
        )
        self.btn_set_basket.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_set_basket_pose.sh"))
        )
        self.btn_table.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_table_pose.sh"))
        )
        self.btn_basket.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_basket_pose.sh"))
        )
        self.btn_infer.clicked.connect(self.infer_grasp_fn)
        self.btn_pick.clicked.connect(self.pick_place_fn)
        self.btn_open.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_open_gripper.sh"))
        )
        self.btn_close.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_close_gripper.sh"))
        )
        self.btn_test.clicked.connect(self._run_robot_test)
        self.btn_eval.clicked.connect(
            lambda: self._run_script("EXP", os.path.join(SCRIPTS_DIR, "eval_last_experiment.sh"))
        )
        self.btn_diag.clicked.connect(self._diag_robot)

        grid = QGridLayout()
        grid.setHorizontalSpacing(1)
        grid.setVerticalSpacing(1)
        grid.addWidget(self.btn_home, 0, 0)
        grid.addWidget(self.btn_table, 0, 1)
        grid.addWidget(self.btn_basket, 0, 2)
        grid.addWidget(self.btn_open, 1, 0)
        grid.addWidget(self.btn_close, 1, 1)
        grid.addWidget(self.btn_pick, 1, 2)
        grid.addWidget(self.btn_set_home, 2, 0)
        grid.addWidget(self.btn_set_table, 2, 1)
        grid.addWidget(self.btn_set_basket, 2, 2)
        grid.addWidget(self.btn_infer, 3, 0)
        grid.addWidget(self.btn_test, 3, 1)
        grid.addWidget(self.btn_diag, 3, 2)
        grid.addWidget(self.btn_eval, 4, 0, 1, 3)
        gl.addLayout(grid)

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
        self.joint_sliders = []
        self.joint_value_labels = []
        slider_min = int(JOINT_SLIDER_DEG_MIN * JOINT_SLIDER_SCALE)
        slider_max = int(JOINT_SLIDER_DEG_MAX * JOINT_SLIDER_SCALE)
        home_pose = load_home_pose()
        for idx, joint in enumerate(UR5_JOINT_NAMES):
            jlabel = QLabel(f"J{idx + 1}")
            jlabel.setToolTip(joint)
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
            if idx < len(home_pose):
                slider.setValue(int(round(math.degrees(home_pose[idx]) * JOINT_SLIDER_SCALE)))
            else:
                slider.setValue(0)
            grid.addWidget(jlabel, idx, 0)
            grid.addWidget(slider, idx, 1)
            grid.addWidget(value_lbl, idx, 2)
        mlay.addLayout(grid)
        manual.setLayout(mlay)
        self.manual_group = manual
        info = QLabel(
            "Nota: si un script queda en 'Waiting for matching subscription(s)...' falta el nodo/controlador.\n"
            "Para control completo hace falta bringup ROS2-control del UR5."
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
        self._robot_buttons = [
            self.btn_home,
            self.btn_table,
            self.btn_basket,
            self.btn_set_home,
            self.btn_set_table,
            self.btn_set_basket,
            self.btn_infer,
            self.btn_pick,
            self.btn_open,
            self.btn_close,
            self.btn_test,
            self.btn_eval,
            self.btn_diag,
        ]
        self.set_robot_state(False, False, False)

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

    def _on_joint_slider_change(self, idx: int, value: int):
        if idx < 0 or idx >= len(self.joint_value_labels):
            return
        deg = self._slider_to_deg(value)
        rad = math.radians(deg)
        self.joint_value_labels[idx].setText(f"{deg:.1f} deg / {rad:.3f} rad")

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

class ExperimentsTab(QWidget):
    """UI tab para lanzar entrenamientos y ver métricas de agarre_inteligente."""
    dataset_sample_ready = pyqtSignal(dict)
    dataset_sample_error = pyqtSignal(str)
    assistant_step_ready = pyqtSignal(str, dict)

    def __init__(self, runner: CmdRunner, log_fn, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
        self.ml_root = Path(VISION_DIR).expanduser().resolve()
        self._config_map: Dict[str, str] = {}
        self._busy = False
        self._metrics_text = ""
        self._proc: Optional[QProcess] = None
        self._proc_tag = ""
        self._proc_kind = ""
        self._proc_pid = None
        self._active_exp_dir: Optional[Path] = None
        self._active_cfg_path: Optional[Path] = None
        self._active_total_epochs: Optional[int] = None
        self._run_started_ts = 0.0
        self._progress_timer = QTimer(self)
        self._progress_timer.setInterval(1000)
        self._progress_timer.timeout.connect(self._refresh_status)
        self._dataset_busy = False
        self._exp_run_log_path: Optional[str] = None
        self._exp_debug_term = False
        self._exp_log_dir = Path(LOG_DIR) / "experiments"
        self._exp_log_dir.mkdir(parents=True, exist_ok=True)
        self._exp_log_path = str(self._exp_log_dir / "experiments_debug.log")
        self._exp_log_latest = str(self._exp_log_dir / "experiments_debug_latest.log")
        self._exp_log_ui_path: Optional[Path] = None
        self._assistant_results: Dict[str, str] = {}
        self._assistant_running = False
        self._assistant_ran_cleanup = False
        self._assistant_steps: Dict[str, Dict[str, object]] = {}
        self._exp_registry: Dict[str, Dict[str, str]] = {}
        self._exp_registry_path = self._resolve_registry_path()
        self._exp_registry_loaded = False
        self._panel_tmp_dir = (self.ml_root / "log" / "experiments_panel").resolve()
        self._panel_tmp_dir.mkdir(parents=True, exist_ok=True)
        self._ml_python = None
        self.dataset_sample_ready.connect(self._apply_dataset_sample)
        self.dataset_sample_error.connect(self._dataset_error)
        self.assistant_step_ready.connect(self._apply_assistant_step)
        self._prune_exp_logs(days=1)
        self._build_ui()
        self.lbl_log_path.setText(f"Log: {self._exp_log_path}")
        self._exp_registry_loaded = False
        self._load_exp_registry()
        self._refresh_configs()
        self._refresh_status()
        self._check_subset_integrity()
        self._run_assistant_checks(initial=True)

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(8)

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(4)
        self.btn_debug = QPushButton("Debug logs -> terminal")
        self.btn_debug.setCheckable(True)
        top_bar.addWidget(self.btn_debug)
        top_bar.addStretch(1)
        lay.addLayout(top_bar)

        self.g_assistant = QGroupBox("Asistente Experimentos")
        self.g_assistant.setFlat(True)
        self.g_assistant.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        assist_layout = QVBoxLayout()
        assist_layout.setContentsMargins(4, 4, 4, 4)
        assist_layout.setSpacing(6)

        assist_top = QHBoxLayout()
        assist_top.setContentsMargins(0, 0, 0, 0)
        assist_top.setSpacing(6)
        self.lbl_assistant_state = QLabel("Estado: ...")
        self.lbl_assistant_state.setStyleSheet("font-weight:bold;")
        assist_top.addWidget(self.lbl_assistant_state)
        assist_top.addStretch(1)
        self.btn_assistant_cleanup = QPushButton("🧹 Limpiar y reiniciar Experimentos")
        self.btn_assistant_cleanup.clicked.connect(
            lambda: (self.cleanup_experiments_panel(), self._run_assistant_checks(initial=False))
        )
        self.btn_assistant_recheck = QPushButton("Recomprobar todo")
        self.btn_assistant_recheck.clicked.connect(lambda: self._run_assistant_checks(initial=False))
        assist_top.addWidget(self.btn_assistant_cleanup)
        assist_top.addWidget(self.btn_assistant_recheck)
        assist_layout.addLayout(assist_top)

        self.assistant_steps_layout = QVBoxLayout()
        self.assistant_steps_layout.setSpacing(4)
        assist_layout.addLayout(self.assistant_steps_layout)
        self.g_assistant.setLayout(assist_layout)
        lay.addWidget(self.g_assistant)

        g_conf = QGroupBox("Experimentos — Configuraciones")
        g_conf.setFlat(True)
        g_conf.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        conf_layout = QGridLayout()
        conf_layout.setContentsMargins(4, 4, 4, 4)
        conf_layout.setHorizontalSpacing(6)
        conf_layout.setVerticalSpacing(6)

        conf_layout.addWidget(QLabel("Experimento:"), 0, 0)
        self.exp_registry_combo = QComboBox()
        self.exp_registry_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        conf_layout.addWidget(self.exp_registry_combo, 0, 1, 1, 3)

        conf_layout.addWidget(QLabel("Config YAML:"), 1, 0)
        self.config_combo = QComboBox()
        self.config_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        conf_layout.addWidget(self.config_combo, 1, 1)

        conf_layout.addWidget(QLabel("Seed:"), 1, 2)
        self.seed_spin = QSpinBox()
        self.seed_spin.setRange(0, 99)
        self.seed_spin.setValue(0)
        conf_layout.addWidget(self.seed_spin, 1, 3)

        self.btn_refresh = QPushButton("Refrescar configs")
        self.btn_refresh.clicked.connect(self._refresh_configs)
        conf_layout.addWidget(self.btn_refresh, 2, 0, 1, 2)

        g_conf.setLayout(conf_layout)
        lay.addWidget(g_conf)

        mid_row = QHBoxLayout()
        mid_row.setSpacing(8)

        self.g_desc = QGroupBox("Descripcion del experimento")
        self.g_desc.setFlat(True)
        self.g_desc.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        desc_layout = QVBoxLayout()
        desc_layout.setContentsMargins(4, 4, 4, 4)
        self.desc_view = QTextBrowser()
        self.desc_view.setReadOnly(True)
        self.desc_view.setOpenExternalLinks(False)
        self.desc_view.setPlaceholderText("Selecciona un YAML para ver la descripcion.")
        desc_layout.addWidget(self.desc_view, 1)
        self.btn_create_desc = QPushButton("Crear descripcion .md")
        self.btn_create_desc.clicked.connect(self._create_description_file)
        desc_layout.addWidget(self.btn_create_desc, 0, Qt.AlignRight)
        self.g_desc.setLayout(desc_layout)
        mid_row.addWidget(self.g_desc, 1)

        self.g_status = QGroupBox("Estado y metricas")
        self.g_status.setFlat(True)
        self.g_status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        status_col = QVBoxLayout()
        status_col.setContentsMargins(4, 4, 4, 4)
        status_col.setSpacing(6)

        status_layout = QGridLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setHorizontalSpacing(6)
        status_layout.setVerticalSpacing(4)

        self.lbl_state = QLabel("Estado: idle")
        self.lbl_exp_dir = QLabel("-")
        self.lbl_epoch = QLabel("Epoch: -/-")
        self.lbl_val_success = QLabel("val_success: -")
        self.lbl_val_loss = QLabel("val_loss: -")
        self.lbl_train_loss = QLabel("train_loss: -")
        self.progress_epoch = QProgressBar()
        self.progress_epoch.setRange(0, 100)
        self.progress_epoch.setValue(0)
        self.progress_epoch.setTextVisible(True)

        status_layout.addWidget(self.lbl_state, 0, 0, 1, 2)
        status_layout.addWidget(QLabel("Ruta EXP:"), 1, 0)
        status_layout.addWidget(self.lbl_exp_dir, 1, 1)
        status_layout.addWidget(self.lbl_epoch, 2, 0)
        status_layout.addWidget(self.lbl_val_success, 2, 1)
        status_layout.addWidget(self.lbl_val_loss, 3, 0)
        status_layout.addWidget(self.lbl_train_loss, 3, 1)
        status_layout.addWidget(self.progress_epoch, 4, 0, 1, 2)
        status_col.addLayout(status_layout)

        result_box = QGroupBox("Resultado final")
        result_box.setFlat(True)
        result_layout = QGridLayout()
        result_layout.setContentsMargins(4, 4, 4, 4)
        result_layout.setHorizontalSpacing(6)
        result_layout.setVerticalSpacing(4)
        self.lbl_result_status = QLabel("Estado: -")
        self.lbl_result_best_epoch = QLabel("best_epoch: -")
        self.lbl_result_best_metric = QLabel("best_val_success: -")
        self.lbl_result_exp = QLabel("Ruta EXP: -")
        self.lbl_result_metrics = QLabel("metrics.csv: -")
        self.lbl_result_plots = QLabel("Plots: -")
        self.lbl_result_summary = QLabel("Resumen: -")
        self.lbl_result_summary.setWordWrap(True)
        result_layout.addWidget(self.lbl_result_status, 0, 0, 1, 2)
        result_layout.addWidget(self.lbl_result_best_epoch, 1, 0)
        result_layout.addWidget(self.lbl_result_best_metric, 1, 1)
        result_layout.addWidget(self.lbl_result_exp, 2, 0, 1, 2)
        result_layout.addWidget(self.lbl_result_metrics, 3, 0, 1, 2)
        result_layout.addWidget(self.lbl_result_plots, 4, 0, 1, 2)
        result_layout.addWidget(self.lbl_result_summary, 5, 0, 1, 2)
        result_box.setLayout(result_layout)
        status_col.addWidget(result_box)

        g_preview = QGroupBox("Mini vista — plots")
        g_preview.setFlat(True)
        preview_layout = QHBoxLayout()
        preview_layout.setContentsMargins(4, 4, 4, 4)
        self.preview_label = QLabel("Sin figura")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setFixedSize(420, 260)
        self.preview_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.preview_label.setStyleSheet("border:1px solid #cbd5f5;")
        preview_layout.addWidget(self.preview_label, 1)
        g_preview.setLayout(preview_layout)
        status_col.addWidget(g_preview)

        self.g_status.setLayout(status_col)
        mid_row.addWidget(self.g_status, 2)
        self.subtabs = QTabWidget()
        self.subtabs.setTabPosition(QTabWidget.North)
        self.subtabs.setDocumentMode(True)
        self.subtabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.subtabs.tabBar().setExpanding(True)

        exp_page = QWidget()
        exp_layout = QVBoxLayout()
        exp_layout.setContentsMargins(0, 0, 0, 0)
        exp_layout.setSpacing(6)
        exp_layout.addLayout(mid_row, 3)

        g_run = QGroupBox("Controles de ejecucion")
        g_run.setFlat(True)
        g_run.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        run_layout = QGridLayout()
        run_layout.setContentsMargins(4, 4, 4, 4)
        run_layout.setHorizontalSpacing(6)
        run_layout.setVerticalSpacing(6)

        self.btn_smoke = QPushButton("▶ Smoke test (1 epoch)")
        self.btn_run_single = QPushButton("▶ Ejecutar (seed unica)")
        self.btn_run_multi = QPushButton("▶ Ejecutar (multi-seed: 0,1,2)")
        self.btn_stop_run = QPushButton("⏹ Detener")
        self.btn_stop_run.setEnabled(False)

        self.chk_deterministic = QCheckBox("Determinismo")
        self.chk_deterministic.setToolTip("Si esta activo, se añade CUBLAS_WORKSPACE_CONFIG=:4096:8")

        self.btn_ab_summary = QPushButton("📊 Generar resumen A/B")
        self.btn_export_tfm = QPushButton("📦 Exportar TFM (tablas y figuras)")

        run_layout.addWidget(self.btn_smoke, 0, 0)
        run_layout.addWidget(self.btn_run_single, 0, 1)
        run_layout.addWidget(self.btn_run_multi, 0, 2)
        run_layout.addWidget(self.btn_stop_run, 0, 3)
        run_layout.addWidget(self.chk_deterministic, 1, 0, 1, 2)
        run_layout.addWidget(self.btn_ab_summary, 1, 2)
        run_layout.addWidget(self.btn_export_tfm, 1, 3)

        g_run.setLayout(run_layout)
        exp_layout.addWidget(g_run)

        log_bar = QHBoxLayout()
        log_bar.setContentsMargins(0, 0, 0, 0)
        log_bar.setSpacing(6)
        self.btn_save_log = QPushButton("Guardar log")
        self.lbl_log_path = QLabel("Log: -")
        self.lbl_log_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        log_bar.addWidget(self.btn_save_log)
        log_bar.addWidget(self.lbl_log_path, 1)
        exp_layout.addLayout(log_bar)

        log_box = QPlainTextEdit()
        log_box.setReadOnly(True)
        log_box.setLineWrapMode(QPlainTextEdit.NoWrap)
        log_box.document().setMaximumBlockCount(800)
        log_box.setPlaceholderText("Logs de entrenamiento / evaluacion apareceran aqui.")
        log_box.setMaximumHeight(200)
        self.log_view = log_box
        exp_layout.addWidget(log_box, 1)

        self.subset_status = QLabel("")
        self.subset_status.setStyleSheet("color:#b91c1c;")
        exp_layout.addWidget(self.subset_status)
        exp_page.setLayout(exp_layout)
        self.subtabs.addTab(exp_page, "Experimento")

        dataset_page = QWidget()
        dataset_layout = QVBoxLayout()
        dataset_layout.setContentsMargins(0, 0, 0, 0)
        dataset_layout.setSpacing(6)

        self.g_dataset = QGroupBox("Dataset")
        self.g_dataset.setFlat(True)
        self.g_dataset.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        data_layout = QGridLayout()
        data_layout.setContentsMargins(4, 4, 4, 4)
        data_layout.setHorizontalSpacing(6)
        data_layout.setVerticalSpacing(4)

        data_layout.addWidget(QLabel("Split:"), 0, 0)
        self.dataset_split = QComboBox()
        self.dataset_split.addItems(["train", "val", "all"])
        data_layout.addWidget(self.dataset_split, 0, 1)
        self.btn_dataset_sample = QPushButton("Mostrar muestra")
        self.btn_dataset_sample.clicked.connect(self._load_dataset_sample)
        data_layout.addWidget(self.btn_dataset_sample, 0, 2)

        self.dataset_summary = QTextBrowser()
        self.dataset_summary.setReadOnly(True)
        self.dataset_summary.setOpenExternalLinks(False)
        self.dataset_summary.setPlaceholderText("Selecciona un YAML para ver info del dataset.")
        data_layout.addWidget(self.dataset_summary, 1, 0, 1, 3)

        self.dataset_preview = QLabel("Sin muestra")
        self.dataset_preview.setAlignment(Qt.AlignCenter)
        self.dataset_preview.setFixedSize(200, 160)
        self.dataset_preview.setStyleSheet("border:1px solid #cbd5f5;")
        data_layout.addWidget(self.dataset_preview, 2, 0, 1, 1)

        self.dataset_sample_meta = QTextBrowser()
        self.dataset_sample_meta.setReadOnly(True)
        self.dataset_sample_meta.setOpenExternalLinks(False)
        self.dataset_sample_meta.setPlaceholderText("Caracteristicas de la muestra.")
        data_layout.addWidget(self.dataset_sample_meta, 2, 1, 1, 2)

        self.g_dataset.setLayout(data_layout)
        dataset_layout.addWidget(self.g_dataset, 0, Qt.AlignTop)
        dataset_layout.addStretch(1)
        dataset_page.setLayout(dataset_layout)
        self.subtabs.addTab(dataset_page, "Dataset")

        cfg_page = QWidget()
        cfg_layout = QVBoxLayout()
        cfg_layout.setContentsMargins(0, 0, 0, 0)
        cfg_layout.setSpacing(6)

        cfg_bar = QHBoxLayout()
        cfg_bar.setContentsMargins(0, 0, 0, 0)
        cfg_bar.setSpacing(6)
        cfg_bar.addWidget(QLabel("Config YAML:"))
        self.config_editor_combo = QComboBox()
        self.config_editor_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        cfg_bar.addWidget(self.config_editor_combo, 1)
        self.btn_cfg_reload = QPushButton("Recargar")
        self.btn_cfg_save = QPushButton("Guardar")
        self.btn_cfg_delete = QPushButton("Borrar")
        cfg_bar.addWidget(self.btn_cfg_reload)
        cfg_bar.addWidget(self.btn_cfg_save)
        cfg_bar.addWidget(self.btn_cfg_delete)
        cfg_layout.addLayout(cfg_bar)

        self.cfg_editor = QPlainTextEdit()
        self.cfg_editor.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.cfg_editor.setPlaceholderText("Selecciona un YAML para editarlo.")
        cfg_layout.addWidget(self.cfg_editor, 1)
        cfg_page.setLayout(cfg_layout)
        self.subtabs.addTab(cfg_page, "Exp-Configuracion")

        lay.addWidget(self.subtabs, 1)

        self.setLayout(lay)
        self._init_assistant_steps()
        self.exp_registry_combo.currentTextChanged.connect(self._on_registry_selected)
        self.config_combo.currentTextChanged.connect(self._on_experiment_selected)
        self.seed_spin.valueChanged.connect(self._on_seed_changed)
        self.dataset_split.currentTextChanged.connect(self._update_dataset_summary)
        self.btn_smoke.clicked.connect(self._launch_smoke)
        self.btn_run_single.clicked.connect(self._launch_training)
        self.btn_run_multi.clicked.connect(self._launch_multi_seed)
        self.btn_stop_run.clicked.connect(self._stop_process)
        self.btn_ab_summary.clicked.connect(self._launch_summary_ab)
        self.btn_export_tfm.clicked.connect(self._export_tfm)
        self.btn_save_log.clicked.connect(self._save_log_snapshot)
        self.config_editor_combo.currentTextChanged.connect(self._load_config_editor)
        self.btn_cfg_reload.clicked.connect(self._reload_config_editor)
        self.btn_cfg_save.clicked.connect(self._save_config_editor)
        self.btn_cfg_delete.clicked.connect(self._delete_config_editor)

    def _init_assistant_steps(self) -> None:
        steps = [
            ("A", "Python/venv + imports", self._check_python_env, None),
            ("B", "Cornell PRO (dataset + index_files estrictos)", self._check_cornell_pro, None),
            ("C", "Registro de EXPs con descripcion", self._check_exp_registry, self._ensure_exp_registry),
            ("D", "Lanzador (smoke / 1 seed / multi-seed)", self._check_launcher_ready, None),
            ("E", "Parsing en vivo (epoch/metricas)", self._check_live_parsing, None),
            ("F", "Consolidacion A/B", self._check_summary_ab, self._launch_summary_ab),
            ("G", "Export TFM (tablas + figuras)", self._check_export_tfm, self._export_tfm),
        ]
        for key, title, check_fn, fix_fn in steps:
            self._add_assistant_step(key, title, check_fn, fix_fn)

    def _add_assistant_step(
        self,
        key: str,
        title: str,
        check_fn,
        fix_fn,
    ) -> None:
        row_box = QVBoxLayout()
        row_box.setContentsMargins(0, 0, 0, 0)
        row_box.setSpacing(2)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        icon = QLabel("⚪")
        icon.setFixedWidth(20)
        summary = QLabel(f"{title}: ...")
        summary.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        btn_action = QPushButton("Ejecutar" if fix_fn else "Comprobar")
        btn_details = QToolButton()
        btn_details.setText("Detalles")
        btn_details.setCheckable(True)

        row.addWidget(icon)
        row.addWidget(summary, 1)
        row.addWidget(btn_action)
        row.addWidget(btn_details)
        row_box.addLayout(row)

        details = QTextBrowser()
        details.setReadOnly(True)
        details.setOpenExternalLinks(False)
        details.setVisible(False)
        details.setMaximumHeight(120)
        row_box.addWidget(details)

        def _toggle_details(checked: bool, box=details):
            box.setVisible(checked)

        btn_details.toggled.connect(_toggle_details)

        def _run_action():
            if fix_fn:
                fix_fn()
                QTimer.singleShot(400, lambda: self._run_single_assistant_check(key))
            else:
                self._run_single_assistant_check(key)

        btn_action.clicked.connect(_run_action)
        self.assistant_steps_layout.addLayout(row_box)
        self._assistant_steps[key] = {
            "title": title,
            "icon": icon,
            "summary": summary,
            "details": details,
            "button": btn_action,
            "check_fn": check_fn,
            "fix_fn": fix_fn,
            "status": "orange",
        }

    def _run_assistant_checks(self, initial: bool = False) -> None:
        if self._assistant_running:
            return
        self._assistant_running = True
        if initial and not self._assistant_ran_cleanup:
            self.cleanup_experiments_panel()
            self._assistant_ran_cleanup = True

        def _worker():
            for key in list(self._assistant_steps.keys()):
                result = self._assistant_steps[key]["check_fn"]()
                self.assistant_step_ready.emit(key, result)
            self._assistant_running = False

        threading.Thread(target=_worker, daemon=True).start()

    def _run_single_assistant_check(self, key: str) -> None:
        if key not in self._assistant_steps:
            return

        def _worker():
            result = self._assistant_steps[key]["check_fn"]()
            self.assistant_step_ready.emit(key, result)

        threading.Thread(target=_worker, daemon=True).start()

    def _apply_assistant_step(self, key: str, result: Dict[str, str]) -> None:
        step = self._assistant_steps.get(key)
        if not step:
            return
        status = result.get("status", "orange")
        summary = result.get("summary", "")
        details = result.get("details", "")
        icon_map = {"green": "🟢", "orange": "🟠", "red": "🔴"}
        step["icon"].setText(icon_map.get(status, "🟠"))
        step["summary"].setText(f"{step['title']}: {summary}")
        step["details"].setPlainText(details)
        step["status"] = status
        self._assistant_results[key] = status
        self._update_assistant_global_state()

    def _update_assistant_global_state(self) -> None:
        statuses = list(self._assistant_results.values())
        if not statuses:
            self.lbl_assistant_state.setText("Estado: ...")
            return
        if "red" in statuses:
            self.lbl_assistant_state.setText("Estado: BLOQUEADO")
            self.lbl_assistant_state.setStyleSheet("font-weight:bold; color:#dc2626;")
        elif "orange" in statuses:
            self.lbl_assistant_state.setText("Estado: FALTAN COSAS")
            self.lbl_assistant_state.setStyleSheet("font-weight:bold; color:#f59e0b;")
        else:
            self.lbl_assistant_state.setText("Estado: LISTO PARA ENTRENAR")
            self.lbl_assistant_state.setStyleSheet("font-weight:bold; color:#16a34a;")

    def cleanup_experiments_panel(self) -> None:
        log_path = Path(LOG_DIR) / "panel_experimentos_cleanup.log"
        self._exp_log_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        removed = []
        for path in self._exp_log_dir.glob("*.log"):
            if path.name in ("experiments_debug.log", "experiments_debug_latest.log"):
                continue
            try:
                path.unlink()
                removed.append(str(path))
            except OSError:
                continue
        tmp_removed = []
        for path in self._panel_tmp_dir.glob("*"):
            if path.is_dir():
                shutil.rmtree(path, ignore_errors=True)
                tmp_removed.append(str(path))
            else:
                try:
                    path.unlink()
                    tmp_removed.append(str(path))
                except OSError:
                    continue
        try:
            ensure_dir(LOG_DIR)
            with log_path.open("a", encoding="utf-8") as handle:
                handle.write(f"[{ts}] Limpieza Experimentos\n")
                for name in removed:
                    handle.write(f"  - log: {name}\n")
                for name in tmp_removed:
                    handle.write(f"  - tmp: {name}\n")
        except Exception:
            pass
        self._append_log_line("[ML] Limpieza Experimentos completada.")

    def _resolve_registry_path(self) -> Path:
        exp_dir = self.ml_root / "experiments"
        exp_dir.mkdir(parents=True, exist_ok=True)
        if yaml is not None:
            return exp_dir / "experiments_registry.yaml"
        return exp_dir / "experiments_registry.json"

    def _load_exp_registry(self) -> None:
        if self._exp_registry_loaded:
            return
        path = self._exp_registry_path
        payload = {}
        if path.exists():
            try:
                if path.suffix == ".json":
                    payload = json.loads(path.read_text(encoding="utf-8"))
                else:
                    payload = yaml.safe_load(path.read_text(encoding="utf-8")) if yaml else {}
            except Exception:
                payload = {}
        if not payload:
            payload = self._build_registry_payload()
            self._write_exp_registry(payload)
        experiments = payload.get("experiments", {}) if isinstance(payload, dict) else {}
        self._exp_registry = {
            name: data for name, data in experiments.items()
            if isinstance(data, dict)
        }
        self._exp_registry_loaded = True

    def _write_exp_registry(self, payload: Dict[str, object]) -> None:
        path = self._exp_registry_path
        try:
            if path.suffix == ".json":
                path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            else:
                if yaml is None:
                    return
                path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        except Exception as exc:
            self.log_fn(f"[ML] WARN: no pude guardar registry: {exc}")

    def _build_registry_payload(self) -> Dict[str, object]:
        defaults = {
            "EXP1_SIMPLE_RGB": "config/exp1_simple_rgb.yaml",
            "EXP2_SIMPLE_RGBD": "config/exp2_simple_rgbd.yaml",
            "EXP3_RESNET18_RGB_AUGMENT": "config/exp3_resnet18_rgb_augment.yaml",
            "EXP3_RESNET18_RGBD": "config/exp3_resnet18_rgbd.yaml",
        }
        if (self.ml_root / "config" / "cornell_resnet18.yaml").exists():
            defaults.setdefault("EXP_RESNET18_RGB_NOAUG", "config/cornell_resnet18.yaml")
        experiments = {}
        for name, rel in defaults.items():
            cfg_path = self.ml_root / rel
            if not cfg_path.exists():
                continue
            desc = self._auto_describe_from_yaml(cfg_path) if yaml else "Descripcion no disponible (PyYAML)."
            experiments[name] = {
                "config": rel,
                "description": desc,
            }
        payload = {
            "version": 1,
            "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "experiments": experiments,
        }
        return payload

    def _ensure_exp_registry(self) -> None:
        payload = self._build_registry_payload()
        self._write_exp_registry(payload)
        self._exp_registry_loaded = False
        self._load_exp_registry()
        self._refresh_registry_combo()

    def _refresh_registry_combo(self) -> None:
        self.exp_registry_combo.blockSignals(True)
        self.exp_registry_combo.clear()
        for name in sorted(self._exp_registry.keys()):
            self.exp_registry_combo.addItem(name)
        self.exp_registry_combo.blockSignals(False)
        if self.exp_registry_combo.count():
            self.exp_registry_combo.setCurrentIndex(0)
            self._on_registry_selected(self.exp_registry_combo.currentText().strip())

    def _registry_entry_for_config(self, rel_path: str) -> Optional[Dict[str, str]]:
        if not rel_path:
            return None
        for _, entry in self._exp_registry.items():
            if entry.get("config") == rel_path:
                return entry
        return None

    def _registry_name_for_config(self, rel_path: str) -> Optional[str]:
        if not rel_path:
            return None
        for name, entry in self._exp_registry.items():
            if entry.get("config") == rel_path:
                return name
        return None

    def _on_registry_selected(self, name: str) -> None:
        entry = self._exp_registry.get(name, {})
        cfg_rel = entry.get("config", "")
        if cfg_rel and cfg_rel in self._config_map.values():
            for cfg_name, cfg_path in self._config_map.items():
                if cfg_path == cfg_rel:
                    self.config_combo.setCurrentText(cfg_name)
                    break
        desc = entry.get("description", "")
        if desc:
            self.desc_view.setPlainText(desc)

    def _resolve_ml_python(self) -> str:
        venv_py = self.ml_root / ".venv" / "bin" / "python"
        if venv_py.exists():
            return str(venv_py)
        for candidate in ("python3", "python"):
            if shutil.which(candidate):
                return candidate
        return ""

    def _check_python_env(self) -> Dict[str, str]:
        py = self._resolve_ml_python()
        self._ml_python = py or None
        if not py:
            return {
                "status": "red",
                "summary": "No se encontro python usable",
                "details": "No existe .venv/bin/python ni python en PATH.",
            }
        cmd = [
            py,
            "-c",
            (
                "import importlib\n"
                "mods=['torch','numpy','yaml','pandas']\n"
                "missing=[]\n"
                "for m in mods:\n"
                "    try:\n"
                "        importlib.import_module(m)\n"
                "    except Exception:\n"
                "        missing.append(m)\n"
                "print('missing=' + ','.join(missing))\n"
            ),
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        except Exception as exc:
            return {
                "status": "orange",
                "summary": "Python disponible, pero fallo el check",
                "details": f"Error ejecutando import check: {exc}",
            }
        stdout = proc.stdout.strip()
        stderr = proc.stderr.strip()
        missing = []
        if "missing=" in stdout:
            missing = [m for m in stdout.split("missing=", 1)[-1].split(",") if m]
        if proc.returncode != 0:
            return {
                "status": "orange",
                "summary": "Python ok, imports con errores",
                "details": f"stdout: {stdout}\nstderr: {stderr}",
            }
        if missing:
            return {
                "status": "orange",
                "summary": f"Faltan paquetes: {', '.join(missing)}",
                "details": f"Python: {py}\nstdout: {stdout}\nstderr: {stderr}",
            }
        return {
            "status": "green",
            "summary": "Python/venv listo",
            "details": f"Python: {py}\nstdout: {stdout}\nstderr: {stderr}",
        }

    def _check_cornell_pro(self) -> Dict[str, str]:
        if not yaml:
            return {
                "status": "red",
                "summary": "PyYAML no disponible",
                "details": "No se puede leer el YAML para validar Cornell PRO.",
            }
        if not self._active_cfg_path:
            return {
                "status": "red",
                "summary": "Sin config seleccionada",
                "details": "Selecciona un YAML para validar dataset.",
            }
        try:
            payload = yaml.safe_load(self._active_cfg_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            return {
                "status": "red",
                "summary": "No se pudo leer config",
                "details": f"ERROR: {exc}",
            }
        data_cfg = payload.get("data", {}) if isinstance(payload, dict) else {}
        root_dir_raw = data_cfg.get("root_dir", "")
        root_dir = self._resolve_dataset_root(root_dir_raw) if root_dir_raw else None
        expected_root = (self.ml_root / "data" / "cornell_raw").resolve()
        issues = []
        if not root_dir or not root_dir.exists():
            issues.append("root_dir no existe o esta vacio")
        elif root_dir.resolve() != expected_root:
            issues.append(f"root_dir no es data/cornell_raw ({root_dir})")
        index_files = data_cfg.get("index_files", {}) if isinstance(data_cfg, dict) else {}
        if not isinstance(index_files, dict) or not index_files:
            issues.append("index_files no definidos en config")
        else:
            for key in ("train", "val"):
                idx = index_files.get(key)
                if not idx:
                    issues.append(f"index_files.{key} ausente")
                    continue
                idx_path = (self.ml_root / idx).resolve() if not os.path.isabs(str(idx)) else Path(idx)
                if not idx_path.exists():
                    issues.append(f"{key} no existe: {idx_path}")
                elif idx_path.stat().st_size == 0:
                    issues.append(f"{key} vacio: {idx_path}")
        if issues:
            return {
                "status": "red",
                "summary": "Faltan requisitos Cornell PRO",
                "details": "\n".join(issues),
            }
        return {
            "status": "green",
            "summary": "Cornell PRO OK",
            "details": f"root_dir: {root_dir}\nindex_files: OK",
        }

    def _check_exp_registry(self) -> Dict[str, str]:
        required = [
            "EXP1_SIMPLE_RGB",
            "EXP2_SIMPLE_RGBD",
            "EXP3_RESNET18_RGB_AUGMENT",
            "EXP3_RESNET18_RGBD",
        ]
        missing = set()
        for name in required:
            entry = self._exp_registry.get(name)
            if not entry:
                missing.add(name)
                continue
            cfg = entry.get("config", "")
            if not cfg or not (self.ml_root / cfg).exists():
                missing.add(name)
            if not entry.get("description", "").strip():
                missing.add(name)
        if not self._exp_registry:
            return {
                "status": "red",
                "summary": "Registry vacio",
                "details": f"Ruta: {self._exp_registry_path}",
            }
        if missing:
            return {
                "status": "orange",
                "summary": f"Faltan {len(missing)} EXP",
                "details": "Faltan: " + ", ".join(sorted(missing)),
            }
        return {
            "status": "green",
            "summary": "Registry listo",
            "details": f"Ruta: {self._exp_registry_path}",
        }

    def _check_launcher_ready(self) -> Dict[str, str]:
        status_b = self._assistant_results.get("B")
        if status_b != "green":
            return {
                "status": "red",
                "summary": "Bloqueado por Cornell PRO",
                "details": "Corrige el paso B antes de lanzar entrenamientos.",
            }
        return {
            "status": "green",
            "summary": "Lanzador habilitado",
            "details": "Puedes ejecutar smoke/seed/multi-seed.",
        }

    def _check_live_parsing(self) -> Dict[str, str]:
        if self._busy:
            return {
                "status": "orange",
                "summary": "Ejecutando",
                "details": "Parsing en vivo activo durante la ejecucion.",
            }
        if self._active_exp_dir and (self._active_exp_dir / "metrics.csv").exists():
            return {
                "status": "green",
                "summary": "metrics.csv detectado",
                "details": f"{self._active_exp_dir / 'metrics.csv'}",
            }
        return {
            "status": "orange",
            "summary": "Sin metrics.csv",
            "details": "Ejecuta un experimento para generar metrics.csv.",
        }

    def _check_summary_ab(self) -> Dict[str, str]:
        exp_dir = self.ml_root / "experiments"
        if not exp_dir.exists():
            return {
                "status": "red",
                "summary": "No existe experiments/",
                "details": f"Ruta: {exp_dir}",
            }
        summary = self.ml_root / "experiments" / "summary_base.csv"
        pretty = self.ml_root / "experiments" / "summary_base_pretty.md"
        if summary.exists() and summary.stat().st_size > 0 and pretty.exists():
            return {
                "status": "green",
                "summary": "summary_base OK",
                "details": f"{summary}\n{pretty}",
            }
        return {
            "status": "orange",
            "summary": "Resumen A/B no generado",
            "details": "Ejecuta Generar resumen A/B.",
        }

    def _check_export_tfm(self) -> Dict[str, str]:
        tablas = self.ml_root / "reports" / "tfm_tablas" / "tabla_ab_resumen.csv"
        figuras = self.ml_root / "reports" / "tfm_figuras"
        memoria = self.ml_root / "experiments" / "figures_memoria" / "memoria_resumen.md"
        fig_ok = figuras / "comparativa_val_success.png"
        loss_ok = figuras / "comparativa_loss.png"
        if tablas.exists() and fig_ok.exists() and loss_ok.exists() and memoria.exists():
            return {
                "status": "green",
                "summary": "Export TFM OK",
                "details": f"{tablas}\n{fig_ok}\n{loss_ok}\n{memoria}",
            }
        return {
            "status": "orange",
            "summary": "Export TFM pendiente",
            "details": "Genera export TFM para crear tablas y figuras.",
        }


    def _refresh_configs(self):
        config_dir = self.ml_root / "config"
        self.config_combo.blockSignals(True)
        self.config_combo.clear()
        self._config_map.clear()
        if not config_dir.is_dir():
            self.log_fn(f"[ML] ERROR: no existe config_dir {config_dir}")
        else:
            for cfg in sorted(config_dir.glob("*.yaml")):
                name = cfg.name
                rel = os.path.relpath(cfg, start=self.ml_root)
                self._config_map[name] = rel
                self.config_combo.addItem(name)
        self.config_combo.blockSignals(False)
        self.config_editor_combo.blockSignals(True)
        self.config_editor_combo.clear()
        for name in self._config_map.keys():
            self.config_editor_combo.addItem(name)
        self.config_editor_combo.blockSignals(False)
        if self.config_combo.count():
            self.config_combo.setCurrentIndex(0)
        self._on_experiment_selected(self.config_combo.currentText().strip())
        if self.config_editor_combo.count():
            self.config_editor_combo.setCurrentIndex(0)
            self._load_config_editor(self.config_editor_combo.currentText().strip())
        self._load_exp_registry()
        self._refresh_registry_combo()

    def current_subtab_name(self) -> str:
        if not self.subtabs:
            return "Experimento"
        idx = self.subtabs.currentIndex()
        if idx < 0:
            return "Experimento"
        name = self.subtabs.tabText(idx).strip()
        return name or "Experimento"

    def _on_experiment_selected(self, value: str):
        rel = self._config_map.get(value, "")
        path = (self.ml_root / rel).resolve() if rel else None
        self._active_cfg_path = path
        if path:
            reg_entry = self._registry_entry_for_config(rel)
            desc = reg_entry.get("description") if reg_entry else ""
            if not desc:
                desc = self._load_description(path)
            self.desc_view.setPlainText(desc)
            reg_name = self._registry_name_for_config(rel)
            if reg_name and self.exp_registry_combo.currentText() != reg_name:
                self.exp_registry_combo.blockSignals(True)
                self.exp_registry_combo.setCurrentText(reg_name)
                self.exp_registry_combo.blockSignals(False)
            self._active_total_epochs = self._infer_total_epochs(path)
            self._active_exp_dir = self._infer_exp_dir(path, self.seed_spin.value())
            self._update_dataset_summary()
            self._clear_dataset_sample()
            self._run_single_assistant_check("B")
            self._run_single_assistant_check("D")
        else:
            self.desc_view.setPlainText("Sin configuracion seleccionada.")
            self._active_total_epochs = None
            self._active_exp_dir = None
            self.dataset_summary.setPlainText("Sin configuracion seleccionada.")
            self._clear_dataset_sample()
        self._refresh_status()
        self._sync_editor_selection(value)

    def _sync_editor_selection(self, value: str):
        if not value:
            return
        self.config_editor_combo.blockSignals(True)
        self.config_editor_combo.setCurrentText(value)
        self.config_editor_combo.blockSignals(False)

    def _on_seed_changed(self, _value: int):
        if self._active_cfg_path:
            self._active_total_epochs = self._infer_total_epochs(self._active_cfg_path)
            self._active_exp_dir = self._infer_exp_dir(self._active_cfg_path, self.seed_spin.value())
            self._refresh_status()

    def _load_description(self, cfg_path: Path) -> str:
        desc_dir = self.ml_root / "config" / "descriptions"
        desc_path = desc_dir / f"{cfg_path.stem}.md"
        if desc_path.exists():
            try:
                return desc_path.read_text(encoding="utf-8")
            except Exception:
                return self._auto_describe_from_yaml(cfg_path)
        return self._auto_describe_from_yaml(cfg_path)

    def _auto_describe_from_yaml(self, cfg_path: Path) -> str:
        rel = os.path.relpath(cfg_path, start=self.ml_root)
        seed = self.seed_spin.value()
        if not yaml:
            return (
                f"Experimento: {cfg_path.stem}\n"
                f"Config: {rel}\n\n"
                "Objetivo: entrenar un modelo de agarre (Cornell).\n"
                f"Ejecucion: ./scripts/run_one.sh {rel} {seed}\n"
            )
        try:
            with cfg_path.open("r", encoding="utf-8") as handle:
                payload = yaml.safe_load(handle) or {}
        except Exception:
            payload = {}

        data_cfg = payload.get("data", {}) if isinstance(payload, dict) else {}
        train_cfg = payload.get("train", {}) if isinstance(payload, dict) else {}
        model_cfg = payload.get("model", {}) if isinstance(payload, dict) else {}
        log_cfg = payload.get("logging", {}) if isinstance(payload, dict) else {}
        metrics_cfg = payload.get("metrics", {}) if isinstance(payload, dict) else {}

        exp_name = payload.get("experiment_name", cfg_path.stem)
        use_depth = bool(data_cfg.get("use_depth", False))
        modality = "RGBD" if use_depth else "RGB"
        model_name = str(model_cfg.get("name", "-"))
        lr = train_cfg.get("lr", train_cfg.get("learning_rate", "-"))
        batch = train_cfg.get("batch_size", "-")
        epochs = train_cfg.get("num_epochs", train_cfg.get("epochs", "-"))
        root_dir = data_cfg.get("root_dir", "-")
        index_files = data_cfg.get("index_files", {}) or {}
        aug_cfg = data_cfg.get("augmentation", data_cfg.get("augmentations", {})) or {}
        aug_on = [k for k, v in aug_cfg.items() if bool(v)]
        aug_text = ", ".join(aug_on) if aug_on else "none"
        deterministic = bool(train_cfg.get("deterministic", False))
        save_best_by = log_cfg.get("save_best_by", "val_success")
        dataset_name = data_cfg.get("dataset", "cornell")
        iou_thresh = metrics_cfg.get("iou_thresh", "-")
        angle_thresh = metrics_cfg.get("angle_thresh", "-")

        lines = [
            f"Experimento: {exp_name}",
            "",
            "Objetivo:",
            f"- Entrenar un modelo de agarre sobre {dataset_name} ({modality}).",
            "",
            "Como se ejecuta:",
            f"- ./scripts/run_one.sh {rel} {seed}",
            "",
            "Parametros importantes:",
            f"- Modelo: {model_name}",
            f"- Modalidad: {modality}",
            f"- lr: {lr} | batch_size: {batch} | epochs: {epochs} | seed: {seed}",
            f"- root_dir: {root_dir}",
            f"- index_files: train={index_files.get('train', '-')}, val={index_files.get('val', '-')}",
            f"- augmentations: {aug_text}",
            f"- save_best_by: {save_best_by}",
            f"- metricas: val_success, val_iou, val_angle | iou_thresh={iou_thresh}, angle_thresh={angle_thresh}",
            "",
            "Reproducibilidad:",
            f"- deterministic: {deterministic}",
            "- Si se usa CUDA y deterministic=True, requiere CUBLAS_WORKSPACE_CONFIG.",
            "- Usa subsets limpios si index_files apunta a reports/cornell_audit.",
            "",
            "Salidas esperadas:",
            "- metrics.csv (por experimento)",
            "- checkpoints/best.pth y last.pth",
            "- plots en experiments/plots/",
            "",
            "Notas:",
            "- El entrenamiento copia config_used.yaml dentro del experimento.",
        ]
        return "\n".join(lines)

    def _create_description_file(self):
        if not self._active_cfg_path:
            return
        desc_dir = self.ml_root / "config" / "descriptions"
        desc_dir.mkdir(parents=True, exist_ok=True)
        desc_path = desc_dir / f"{self._active_cfg_path.stem}.md"
        if desc_path.exists():
            self.log_fn(f"[ML] Descripcion ya existe: {desc_path}")
            return
        desc_text = self._auto_describe_from_yaml(self._active_cfg_path)
        desc_path.write_text(desc_text, encoding="utf-8")
        self.desc_view.setPlainText(desc_text)
        self.log_fn(f"[ML] Descripcion creada: {desc_path}")

    def _load_config_editor(self, value: str):
        rel = self._config_map.get(value, "")
        if not rel:
            self.cfg_editor.setPlainText("")
            return
        path = (self.ml_root / rel).resolve()
        try:
            text = path.read_text(encoding="utf-8")
        except Exception as exc:
            self.cfg_editor.setPlainText(f"# ERROR leyendo {path}: {exc}")
            return
        self.cfg_editor.setPlainText(text)
        self._sync_main_selection(value)

    def _reload_config_editor(self):
        self._load_config_editor(self.config_editor_combo.currentText().strip())

    def _save_config_editor(self):
        name = self.config_editor_combo.currentText().strip()
        rel = self._config_map.get(name, "")
        if not rel:
            return
        path = (self.ml_root / rel).resolve()
        text = self.cfg_editor.toPlainText()
        try:
            path.write_text(text, encoding="utf-8")
        except Exception as exc:
            self.log_fn(f"[ML] ERROR guardando {path}: {exc}")
            return
        self.log_fn(f"[ML] Config guardada: {path}")
        self._on_experiment_selected(name)

    def _delete_config_editor(self):
        name = self.config_editor_combo.currentText().strip()
        rel = self._config_map.get(name, "")
        if not rel:
            return
        path = (self.ml_root / rel).resolve()
        reply = QMessageBox.question(
            self,
            "Borrar config",
            f"¿Seguro que quieres borrar {path.name}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        try:
            path.unlink()
        except Exception as exc:
            self.log_fn(f"[ML] ERROR borrando {path}: {exc}")
            return
        self.log_fn(f"[ML] Config borrada: {path}")
        self.cfg_editor.setPlainText("")
        self._refresh_configs()

    def _sync_main_selection(self, value: str):
        if not value:
            return
        self.config_combo.blockSignals(True)
        self.config_combo.setCurrentText(value)
        self.config_combo.blockSignals(False)

    def _clear_dataset_sample(self):
        self.dataset_preview.setText("Sin muestra")
        self.dataset_preview.setPixmap(QPixmap())
        self.dataset_sample_meta.setPlainText("")

    def _update_dataset_summary(self):
        if not self._active_cfg_path:
            self.dataset_summary.setPlainText("Sin configuracion seleccionada.")
            return
        if not yaml:
            self.dataset_summary.setPlainText("PyYAML no disponible para leer el YAML.")
            return
        try:
            payload = yaml.safe_load(self._active_cfg_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            self.dataset_summary.setPlainText(f"ERROR leyendo YAML: {exc}")
            return

        data_cfg = payload.get("data", {}) if isinstance(payload, dict) else {}
        dataset_name = data_cfg.get("dataset", "cornell")
        root_dir_raw = str(data_cfg.get("root_dir", "-"))
        root_dir = self._resolve_dataset_root(root_dir_raw)
        img_size = data_cfg.get("img_size", "-")
        val_split = data_cfg.get("val_split", "-")
        use_depth = bool(data_cfg.get("use_depth", False))
        index_cfg = data_cfg.get("index_files", {}) or {}
        train_idx = self._resolve_optional_path(index_cfg.get("train", ""))
        val_idx = self._resolve_optional_path(index_cfg.get("val", ""))
        train_count = self._count_index_lines(train_idx) if train_idx else None
        val_count = self._count_index_lines(val_idx) if val_idx else None

        lines = [
            f"Dataset: {dataset_name}",
            f"root_dir: {root_dir if root_dir else root_dir_raw}",
            f"split seleccionado: {self.dataset_split.currentText()}",
            f"img_size: {img_size} | val_split: {val_split} | use_depth: {use_depth}",
        ]
        if train_idx:
            lines.append(f"index train: {train_idx} (n={train_count})")
        if val_idx:
            lines.append(f"index val: {val_idx} (n={val_count})")
        if not train_idx and not val_idx:
            lines.append("index_files: no definidos")
        self.dataset_summary.setPlainText("\n".join(lines))

    def _load_dataset_sample(self):
        if self._dataset_busy:
            return
        if not self._active_cfg_path:
            self.dataset_sample_meta.setPlainText("Selecciona un YAML primero.")
            return
        if not yaml:
            self.dataset_sample_meta.setPlainText("PyYAML no disponible.")
            return
        split = self.dataset_split.currentText().strip()
        self._dataset_busy = True
        self.btn_dataset_sample.setEnabled(False)
        self.dataset_sample_meta.setPlainText("Cargando muestra...")

        def _worker():
            try:
                sample = self._prepare_dataset_sample(self._active_cfg_path, split)
            except Exception as exc:
                self.dataset_sample_error.emit(str(exc))
                return
            self.dataset_sample_ready.emit(sample)

        threading.Thread(target=_worker, daemon=True).start()

    def _dataset_error(self, msg: str):
        self._dataset_busy = False
        self.btn_dataset_sample.setEnabled(True)
        self.dataset_sample_meta.setPlainText(f"ERROR: {msg}")

    def _apply_dataset_sample(self, sample: Dict[str, object]):
        self._dataset_busy = False
        self.btn_dataset_sample.setEnabled(True)
        rgb_path = Path(sample.get("rgb", ""))
        cpos_path = Path(sample.get("cpos", ""))
        depth_path = Path(sample.get("depth", ""))
        grasp_count = sample.get("grasp_count", "-")
        split = sample.get("split", "-")
        subset = sample.get("subset", False)
        idx = sample.get("index", "-")
        total = sample.get("total", "-")
        partial = sample.get("partial", False)

        pixmap = QPixmap(str(rgb_path)) if rgb_path.exists() else QPixmap()
        if pixmap and not pixmap.isNull():
            self.dataset_preview.setPixmap(
                pixmap.scaled(self.dataset_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        else:
            self.dataset_preview.setText("Sin imagen")
            self.dataset_preview.setPixmap(QPixmap())

        lines = [
            f"split: {split} | subset: {subset}",
            f"index: {idx} / {total}{' (parcial)' if partial else ''}",
            f"rgb: {rgb_path}",
            f"depth: {depth_path if depth_path.exists() else '-'}",
            f"cpos: {cpos_path}",
            f"grasps: {grasp_count}",
        ]
        self.dataset_sample_meta.setPlainText("\n".join(lines))

    def _prepare_dataset_sample(self, cfg_path: Path, split: str) -> Dict[str, object]:
        payload = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        data_cfg = payload.get("data", {}) if isinstance(payload, dict) else {}
        root_dir_raw = str(data_cfg.get("root_dir", "")).strip()
        root_dir = self._resolve_dataset_root(root_dir_raw)
        if not root_dir or not root_dir.exists():
            raise FileNotFoundError(f"root_dir no existe: {root_dir_raw}")
        val_split = float(data_cfg.get("val_split", 0.2))
        index_cfg = data_cfg.get("index_files", {}) or {}
        train_idx = self._resolve_optional_path(index_cfg.get("train", ""))
        val_idx = self._resolve_optional_path(index_cfg.get("val", ""))

        samples, partial = self._collect_dataset_samples(root_dir, time_budget_s=3.0)
        if not samples:
            raise RuntimeError("No se encontraron samples en el dataset.")

        subset = False
        if split != "all" and not partial:
            n_total = len(samples)
            n_val = int(round(n_total * val_split))
            n_train = n_total - n_val
            if split == "train":
                samples = samples[:n_train]
            else:
                samples = samples[n_train:]

        idx_file = None
        if split == "train":
            idx_file = train_idx
        elif split == "val":
            idx_file = val_idx

        if idx_file and idx_file.exists() and not partial:
            idx_vals = self._read_indices(idx_file)
            filtered = []
            for i in idx_vals:
                if 0 <= i < len(samples):
                    filtered.append(samples[i])
            samples = filtered
            subset = True

        if not samples:
            raise RuntimeError("Subset vacio: no hay muestras para este split.")

        pick_idx = random.randrange(len(samples))
        sample = samples[pick_idx]
        grasp_count = self._count_grasps(sample["cpos"])

        return {
            "rgb": sample["rgb"],
            "depth": sample["depth"],
            "cpos": sample["cpos"],
            "grasp_count": grasp_count,
            "split": split,
            "subset": subset,
            "index": pick_idx,
            "total": len(samples),
            "partial": partial,
        }

    def _resolve_optional_path(self, path_str: str) -> Optional[Path]:
        if not path_str:
            return None
        path = Path(path_str)
        if not path.is_absolute():
            path = (self.ml_root / path).resolve()
        return path

    def _resolve_dataset_root(self, root_dir: str) -> Optional[Path]:
        if not root_dir:
            return None
        path = Path(root_dir)
        if not path.is_absolute():
            path = (self.ml_root / path).resolve()
        if path.is_dir():
            return path
        base_name = path.name
        parent = path.parent
        candidates = []
        if base_name == "cornell":
            candidates.append(parent / "cornell_raw")
            candidates.append(parent / "cornell_processed")
        candidates.append(Path(str(path) + "_raw"))
        candidates.append(Path(str(path) + "_processed"))
        for cand in candidates:
            if cand.is_dir():
                return cand
        return None

    def _collect_dataset_samples(self, root_dir: Path, time_budget_s: float = 3.0) -> Tuple[List[Dict[str, str]], bool]:
        samples: List[Dict[str, str]] = []
        partial = False
        start = time.time()
        for dirpath, dirnames, filenames in os.walk(root_dir):
            if time_budget_s and (time.time() - start) > time_budget_s:
                partial = True
                break
            dirnames.sort()
            filenames.sort()
            cpos_files = [f for f in filenames if f.endswith("cpos.txt")]
            for cpos_name in cpos_files:
                cpos_path = os.path.join(dirpath, cpos_name)
                base = cpos_path[:-8]
                rgb_candidates = [
                    base + "r.png",
                    base + "r.jpg",
                    base + "r.jpeg",
                ]
                depth_candidates = [
                    base + "d.tiff",
                    base + "d.tif",
                    base + "d.png",
                    base + "d.jpg",
                ]
                rgb_path = next((p for p in rgb_candidates if os.path.exists(p)), None)
                depth_path = next((p for p in depth_candidates if os.path.exists(p)), None)
                if not rgb_path or not depth_path:
                    continue
                samples.append({
                    "rgb": rgb_path,
                    "depth": depth_path,
                    "cpos": cpos_path,
                })
                if time_budget_s and (time.time() - start) > time_budget_s:
                    partial = True
                    return samples, partial
        return samples, partial

    def _read_indices(self, path: Path) -> List[int]:
        values = []
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                values.append(int(line))
            except ValueError:
                continue
        return values

    def _count_index_lines(self, path: Path) -> int:
        try:
            return len([line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()])
        except Exception:
            return 0

    def _count_grasps(self, cpos_path: str) -> int:
        try:
            lines = [line.strip() for line in Path(cpos_path).read_text(encoding="utf-8").splitlines() if line.strip()]
        except Exception:
            return 0
        return len(lines) // 4

    def _launch_training(self):
        if self._busy:
            self.log_fn("[ML] Ya hay un comando en curso.")
            return
        if not self._can_launch():
            return
        config = self._current_config_rel()
        if not config:
            self.log_fn("[ML] ERROR: sin config seleccionada.")
            return
        seed = self.seed_spin.value()
        env = self._determinism_env()
        cmd = f"cd '{self.ml_root}' && {env}PYTHONUNBUFFERED=1 ./scripts/run_one.sh '{config}' {seed}"
        exp_dir = None
        if self._active_cfg_path:
            exp_dir = self._infer_exp_dir(self._active_cfg_path, seed)
        self._start_process("ML-TRAIN", cmd, exp_dir, kind="train", clear_logs=True)

    def _launch_eval(self):
        self._launch_summary_ab()

    def _launch_smoke(self):
        if self._busy:
            self.log_fn("[ML] Ya hay un comando en curso.")
            return
        if not self._can_launch():
            return
        if not yaml:
            self.log_fn("[ML] ERROR: PyYAML no disponible para smoke test.")
            return
        config = self._current_config_rel()
        if not config:
            self.log_fn("[ML] ERROR: sin config seleccionada.")
            return
        seed = 0
        smoke_cfg = self._build_smoke_config(config)
        if not smoke_cfg:
            self.log_fn("[ML] ERROR: no pude generar config de smoke.")
            return
        rel_smoke = os.path.relpath(smoke_cfg, start=self.ml_root)
        env = self._determinism_env()
        cmd = f"cd '{self.ml_root}' && {env}PYTHONUNBUFFERED=1 ./scripts/run_one.sh '{rel_smoke}' {seed}"
        exp_dir = self._infer_exp_dir(Path(smoke_cfg), seed)
        self._start_process("ML-SMOKE", cmd, exp_dir, kind="smoke", clear_logs=True)

    def _launch_multi_seed(self):
        if self._busy:
            self.log_fn("[ML] Ya hay un comando en curso.")
            return
        if not self._can_launch():
            return
        config = self._current_config_rel()
        if not config:
            self.log_fn("[ML] ERROR: sin config seleccionada.")
            return
        env = self._determinism_env()
        cmd = (
            f"cd '{self.ml_root}' && "
            "if [ -f '.venv/bin/activate' ]; then source .venv/bin/activate; fi && "
            f"{env}PYTHONUNBUFFERED=1 ./scripts/run_seeds.sh '{config}' 0 1 2"
        )
        self._active_exp_dir = None
        self._start_process("ML-MULTI", cmd, None, kind="multi", clear_logs=True)

    def _launch_summary_ab(self):
        if self._busy:
            self.log_fn("[ML] Ya hay un comando en curso.")
            return
        env = self._determinism_env()
        cmd = (
            f"cd '{self.ml_root}' && "
            "if [ -x '.venv/bin/python' ]; then PY='.venv/bin/python'; else PY='python3'; fi && "
            f"{env}PYTHONUNBUFFERED=1 $PY scripts/analyze_experiments.py --root . --output experiments/summary_base.csv && "
            f"{env}PYTHONUNBUFFERED=1 $PY src/graspnet/metrics/make_day2_report.py --root experiments"
        )
        self._start_process("ML-SUMMARY", cmd, None, kind="summary", clear_logs=False)

    def _export_tfm(self):
        if self._busy:
            self.log_fn("[ML] Ya hay un comando en curso.")
            return
        env = self._determinism_env()
        cmd = (
            f"cd '{self.ml_root}' && "
            "if [ -x '.venv/bin/python' ]; then PY='.venv/bin/python'; else PY='python3'; fi && "
            f"{env}PYTHONUNBUFFERED=1 $PY scripts/analyze_experiments.py --root . --output experiments/summary_base.csv && "
            f"{env}PYTHONUNBUFFERED=1 $PY src/graspnet/metrics/make_day2_report.py --root experiments && "
            f"{env}PYTHONUNBUFFERED=1 $PY - <<'PY'\n"
            "import csv\n"
            "import statistics\n"
            "from pathlib import Path\n"
            "root = Path('.').resolve()\n"
            "exp_dir = root / 'experiments'\n"
            "reports = root / 'reports'\n"
            "tfm_tablas = reports / 'tfm_tablas'\n"
            "tfm_figs = reports / 'tfm_figuras'\n"
            "tfm_tablas.mkdir(parents=True, exist_ok=True)\n"
            "tfm_figs.mkdir(parents=True, exist_ok=True)\n"
            "summary = exp_dir / 'summary_base.csv'\n"
            "rows = []\n"
            "if summary.exists():\n"
            "    with summary.open() as f:\n"
            "        reader = csv.DictReader(f)\n"
            "        for r in reader:\n"
            "            rows.append(r)\n"
            "out_csv = tfm_tablas / 'tabla_ab_resumen.csv'\n"
            "with out_csv.open('w', newline='') as f:\n"
            "    fieldnames = ['exp','seed','best_epoch','val_success','val_loss','params']\n"
            "    writer = csv.DictWriter(f, fieldnames=fieldnames)\n"
            "    writer.writeheader()\n"
            "    for r in rows:\n"
            "        writer.writerow({\n"
            "            'exp': r.get('exp_id',''),\n"
            "            'seed': r.get('seed',''),\n"
            "            'best_epoch': r.get('best_epoch',''),\n"
            "            'val_success': r.get('val_success',''),\n"
            "            'val_loss': r.get('val_loss',''),\n"
            "            'params': r.get('params',''),\n"
            "        })\n"
            "plots_dir = exp_dir / 'plots'\n"
            "success_plot = tfm_figs / 'comparativa_val_success.png'\n"
            "loss_plot = tfm_figs / 'comparativa_loss.png'\n"
            "try:\n"
            "    import pandas as pd\n"
            "    import matplotlib.pyplot as plt\n"
            "    if rows:\n"
            "        df = pd.DataFrame(rows)\n"
            "        if 'val_success' in df.columns:\n"
            "            df['val_success'] = pd.to_numeric(df['val_success'], errors='coerce')\n"
            "            by_exp = df.groupby('exp_id')['val_success'].mean().sort_values(ascending=False)\n"
            "            by_exp.plot(kind='bar')\n"
            "            plt.title('Comparativa val_success (media)')\n"
            "            plt.ylabel('val_success')\n"
            "            plt.tight_layout()\n"
            "            plt.savefig(success_plot, dpi=160)\n"
            "            plt.close()\n"
            "        if 'val_loss' in df.columns:\n"
            "            df['val_loss'] = pd.to_numeric(df['val_loss'], errors='coerce')\n"
            "            by_exp_loss = df.groupby('exp_id')['val_loss'].mean().sort_values(ascending=True)\n"
            "            by_exp_loss.plot(kind='bar')\n"
            "            plt.title('Comparativa val_loss (media)')\n"
            "            plt.ylabel('val_loss')\n"
            "            plt.tight_layout()\n"
            "            plt.savefig(loss_plot, dpi=160)\n"
            "            plt.close()\n"
            "except Exception:\n"
            "    pass\n"
            "fig_mem = exp_dir / 'figures_memoria'\n"
            "fig_mem.mkdir(parents=True, exist_ok=True)\n"
            "winner_md = fig_mem / 'memoria_resumen.md'\n"
            "top = None\n"
            "if rows:\n"
            "    best = {}\n"
            "    for r in rows:\n"
            "        exp = r.get('exp_id','')\n"
            "        try:\n"
            "            val = float(r.get('val_success','nan'))\n"
            "        except Exception:\n"
            "            continue\n"
            "        if exp not in best:\n"
            "            best[exp] = []\n"
            "        best[exp].append(val)\n"
            "    if best:\n"
            "        avg = {k: statistics.mean(v) for k,v in best.items() if v}\n"
            "        top = max(avg.items(), key=lambda x: x[1])[0] if avg else None\n"
            "entries = []\n"
            "if top:\n"
            "    entries = list(plots_dir.glob(f'{top}*__val_success.png'))\n"
            "    entries += list(plots_dir.glob(f'{top}*__loss.png'))\n"
            "    for p in entries:\n"
            "        (fig_mem / p.name).write_bytes(p.read_bytes())\n"
            "md_lines = [\n"
            "    '# Memoria - Figuras ganadoras',\n"
            "    f'Experimento ganador: {top or \"N/A\"}',\n"
            "    '',\n"
            "    '## Figuras',\n"
            "]\n"
            "for p in entries:\n"
            "    md_lines.append(f'- {p.name}')\n"
            "winner_md.write_text('\\n'.join(md_lines) + '\\n', encoding='utf-8')\n"
            "print('[EXPORT] OK')\n"
            "PY"
        )
        self._start_process("ML-EXPORT", cmd, None, kind="export", clear_logs=False)

    def _current_config_rel(self) -> str:
        name = self.config_combo.currentText().strip()
        return self._config_map.get(name, "")

    def _determinism_env(self) -> str:
        if not self.chk_deterministic.isChecked():
            return ""
        return "export CUBLAS_WORKSPACE_CONFIG=':4096:8' ; "

    def _can_launch(self) -> bool:
        result = self._check_cornell_pro()
        if result.get("status") != "green":
            self._apply_assistant_step("B", result)
            self.log_fn("[ML] Bloqueado: Cornell PRO no valido. Revisa Asistente.")
            return False
        return True

    def _build_smoke_config(self, base_rel: str) -> Optional[str]:
        if not yaml:
            return None
        base_path = (self.ml_root / base_rel).resolve()
        if not base_path.exists():
            return None
        try:
            cfg = yaml.safe_load(base_path.read_text(encoding="utf-8")) or {}
        except Exception:
            return None
        cfg = dict(cfg)
        base_name = str(cfg.get("experiment_name", base_path.stem))
        cfg["experiment_name"] = f"SMOKE_{base_name}"
        cfg.setdefault("train", {})
        cfg["train"]["num_epochs"] = 1
        cfg.setdefault("logging", {})
        cfg["logging"]["base_dir"] = str(self._panel_tmp_dir / "smoke_runs")
        cfg_path = self._panel_tmp_dir / "configs"
        cfg_path.mkdir(parents=True, exist_ok=True)
        out_path = cfg_path / f"smoke_{base_path.stem}_{now_tag()}.yaml"
        out_path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
        return str(out_path)

    def _save_log_snapshot(self) -> None:
        ensure_dir(LOG_DIR)
        out_dir = self._exp_log_dir
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"experimentos_ui_{now_tag()}.log"
        try:
            out_path.write_text(self.log_view.toPlainText(), encoding="utf-8")
            self.lbl_log_path.setText(f"Log: {out_path}")
            self._append_log_line(f"[ML] Log guardado: {out_path}")
        except Exception as exc:
            self.log_fn(f"[ML] ERROR guardando log: {exc}")

    def _start_process(self, tag: str, cmd: str, exp_dir: Optional[Path], kind: str, clear_logs: bool = True):
        if self._proc and self._proc.state() != QProcess.NotRunning:
            self.log_fn("[ML] Proceso en curso. Espera a que termine.")
            return
        self._busy = True
        self._proc_tag = tag
        self._proc_kind = kind
        self._active_exp_dir = exp_dir
        self._run_started_ts = time.time()
        self._start_run_log(tag)
        if clear_logs:
            self.log_view.clear()
        self._reset_result_panel()
        self._append_log_line(f"[{tag}] $ {cmd}")
        if self.chk_deterministic.isChecked():
            self._append_log_line("[ML] Determinismo activo: CUBLAS_WORKSPACE_CONFIG=:4096:8")
        self._set_state("running")
        self._set_run_buttons_enabled(False)
        self.btn_stop_run.setEnabled(True)
        # debug a terminal gestionado globalmente

        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(self._on_proc_ready)
        proc.finished.connect(self._on_proc_finished)
        proc.start("bash", ["-lc", cmd])
        self._proc = proc
        try:
            self._proc_pid = int(proc.processId())
        except Exception:
            self._proc_pid = None
        self._progress_timer.start()
        self._refresh_status()

    def _set_run_buttons_enabled(self, enabled: bool) -> None:
        self.btn_smoke.setEnabled(enabled)
        self.btn_run_single.setEnabled(enabled)
        self.btn_run_multi.setEnabled(enabled)
        self.btn_ab_summary.setEnabled(enabled)
        self.btn_export_tfm.setEnabled(enabled)
        self.btn_save_log.setEnabled(True)

    def _stop_process(self):
        if not self._proc or self._proc.state() == QProcess.NotRunning:
            self.log_fn("[ML] No hay proceso activo.")
            return
        self.log_fn("[ML] Deteniendo proceso en curso...")
        self._proc.terminate()
        QTimer.singleShot(2000, self._kill_process_if_needed)

    def _kill_process_if_needed(self):
        if not self._proc or self._proc.state() == QProcess.NotRunning:
            return
        self._proc.kill()

    def _on_proc_ready(self):
        if not self._proc:
            return
        raw = bytes(self._proc.readAllStandardOutput()).decode("utf-8", errors="ignore")
        if not raw:
            return
        for line in raw.splitlines():
            self._append_log_line(f"[{self._proc_tag}] {line}")
            match = re.search(r"\[Epoch\s+(\d+)/(\d+)\]", line)
            if match:
                try:
                    epoch = int(match.group(1))
                    total = int(match.group(2))
                    self.lbl_epoch.setText(f"Epoch: {epoch}/{total}")
                    self.progress_epoch.setRange(0, total)
                    self.progress_epoch.setValue(epoch)
                except Exception:
                    pass
            if "Resultados en:" in line:
                _, _, tail = line.partition("Resultados en:")
                exp_path = tail.strip()
                if exp_path:
                    self._active_exp_dir = Path(exp_path).expanduser()
                    self._refresh_status()

    def _on_proc_finished(self, rc: int, _status=None):
        tag = self._proc_tag
        self._busy = False
        self._set_run_buttons_enabled(True)
        self.btn_stop_run.setEnabled(False)
        self._progress_timer.stop()
        self._set_state("finished" if rc == 0 else "failed")
        self._append_log_line(f"[{tag}] [EXIT] rc={rc}")
        if self._proc_kind in ("summary", "export"):
            self._update_summary_panel(rc=rc)
        else:
            self._update_result_panel(rc=rc)
        if self._proc_kind in ("train", "smoke", "multi"):
            self._run_single_assistant_check("E")
        if self._proc_kind == "summary":
            self._run_single_assistant_check("F")
        if self._proc_kind == "export":
            self._run_single_assistant_check("G")
        self._refresh_status()
        self._proc_kind = ""
        self._proc = None
        self._proc_pid = None

    def _append_log_line(self, text: str):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {text}"
        self.log_view.appendPlainText(line)
        self.log_view.moveCursor(QTextCursor.End)
        self._write_exp_log(text)
        if self._exp_debug_term:
            print(f"[EXP] {line}", flush=True)

    def _set_state(self, state: str):
        self.lbl_state.setText(f"Estado: {state}")

    def _write_exp_log(self, text: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {text}"
        try:
            self._exp_log_dir.mkdir(parents=True, exist_ok=True)
            with open(self._exp_log_path, "a", encoding="utf-8") as handle:
                handle.write(line + "\n")
            if self._exp_run_log_path:
                with open(self._exp_run_log_path, "a", encoding="utf-8") as handle:
                    handle.write(line + "\n")
        except Exception as exc:
            self.log_fn(f"[ML] WARN: no pude escribir log debug: {exc}")

    def _start_run_log(self, tag: str):
        self._exp_log_dir.mkdir(parents=True, exist_ok=True)
        run_name = f"experiments_debug_{tag.lower()}_{now_tag()}.log"
        self._exp_run_log_path = str(self._exp_log_dir / run_name)
        try:
            Path(self._exp_log_path).touch(exist_ok=True)
            Path(self._exp_run_log_path).touch(exist_ok=True)
            try:
                if os.path.islink(self._exp_log_latest) or os.path.exists(self._exp_log_latest):
                    os.unlink(self._exp_log_latest)
                os.symlink(self._exp_run_log_path, self._exp_log_latest)
            except Exception:
                pass
        except Exception as exc:
            self.log_fn(f"[ML] WARN: no pude preparar log debug: {exc}")
        self._append_log_line(f"[ML-DEBUG] Log debug: {self._exp_log_path}")
        self._append_log_line(f"[ML-DEBUG] Log ejecucion: {self._exp_run_log_path}")
        if self._exp_run_log_path:
            self._exp_log_ui_path = Path(self._exp_run_log_path)
            self.lbl_log_path.setText(f"Log: {self._exp_run_log_path}")

    def set_debug_enabled(self, enabled: bool) -> None:
        self._exp_debug_term = bool(enabled)

    def _prune_exp_logs(self, days: int = 1):
        self._exp_log_dir.mkdir(parents=True, exist_ok=True)
        if days <= 0:
            return
        cutoff = time.time() - (days * 86400)
        prefix = "experiments_debug_"
        for name in os.listdir(self._exp_log_dir):
            if not name.startswith(prefix):
                continue
            if name in ("experiments_debug.log", "experiments_debug_latest.log"):
                continue
            path = os.path.join(str(self._exp_log_dir), name)
            if os.path.islink(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if mtime < cutoff:
                try:
                    os.remove(path)
                except OSError:
                    pass

    def _open_exp_log_terminal(self):
        ensure_dir(LOG_DIR)
        target = self._exp_log_path
        try:
            Path(self._exp_log_path).touch(exist_ok=True)
            if os.path.exists(self._exp_log_latest):
                target = self._exp_log_latest
        except Exception:
            pass
        tail_cmd = f"tail -n 200 -F '{target}'"
        tail_q = shlex.quote(tail_cmd)
        title = "EXP logs"
        geometry = "110x24"
        term_cmd = (
            f"gnome-terminal --title '{title}' --geometry={geometry} -- bash -lc {tail_q} || "
            f"xterm -T '{title}' -geometry {geometry} -e bash -lc {tail_q} || "
            f"konsole --new-tab --geometry {geometry} -p tabtitle={title} -e bash -lc {tail_q} || "
            f"xfce4-terminal --title '{title}' --geometry={geometry} -e bash -lc {tail_q} || "
            f"x-terminal-emulator -geometry {geometry} -e bash -lc {tail_q}"
        )
        try:
            subprocess.Popen(["bash", "-lc", term_cmd])
        except Exception as exc:
            self._append_log_line(f"[ML] WARN: no pude abrir terminal: {exc}")
    def _reset_result_panel(self):
        self.lbl_result_status.setText("Estado: -")
        self.lbl_result_best_epoch.setText("best_epoch: -")
        self.lbl_result_best_metric.setText("best_val_success: -")
        self.lbl_result_exp.setText("Ruta EXP: -")
        self.lbl_result_metrics.setText("metrics.csv: -")
        self.lbl_result_plots.setText("Plots: -")
        self.lbl_result_summary.setText("Resumen: -")

    def _refresh_status(self):
        exp_dir = self._active_exp_dir
        if exp_dir:
            self.lbl_exp_dir.setText(str(exp_dir))
        else:
            self.lbl_exp_dir.setText("-")
        total_epochs = self._active_total_epochs or 0
        metrics = self._read_metrics_latest(exp_dir) if exp_dir else None
        if metrics:
            epoch = metrics.get("epoch")
            if epoch is not None and total_epochs:
                self.lbl_epoch.setText(f"Epoch: {epoch}/{total_epochs}")
                self.progress_epoch.setRange(0, total_epochs)
                self.progress_epoch.setValue(int(epoch))
            elif epoch is not None:
                self.lbl_epoch.setText(f"Epoch: {epoch}/-")
                self.progress_epoch.setRange(0, 100)
                self.progress_epoch.setValue(0)
            self.lbl_val_success.setText(f"val_success: {metrics.get('val_success', '-')}")
            self.lbl_val_loss.setText(f"val_loss: {metrics.get('val_loss', '-')}")
            self.lbl_train_loss.setText(f"train_loss: {metrics.get('train_loss', '-')}")
        else:
            self.lbl_epoch.setText("Epoch: -/-")
            self.lbl_val_success.setText("val_success: -")
            self.lbl_val_loss.setText("val_loss: -")
            self.lbl_train_loss.setText("train_loss: -")
            self.progress_epoch.setRange(0, 100)
            self.progress_epoch.setValue(0)
        self._update_preview()

    def _read_metrics_latest(self, exp_dir: Optional[Path]) -> Optional[Dict[str, str]]:
        if not exp_dir:
            return None
        metrics_path = exp_dir / "metrics.csv"
        if not metrics_path.exists():
            return None
        try:
            with metrics_path.open("r", newline="", encoding="utf-8") as handler:
                rows = list(csv.DictReader(handler))
        except Exception:
            return None
        if not rows:
            return None
        return rows[-1]

    def _infer_total_epochs(self, cfg_path: Path) -> Optional[int]:
        if not yaml:
            return None
        try:
            with cfg_path.open("r", encoding="utf-8") as handle:
                payload = yaml.safe_load(handle) or {}
        except Exception:
            return None
        train_cfg = payload.get("train", {}) if isinstance(payload, dict) else {}
        epochs = train_cfg.get("num_epochs", train_cfg.get("epochs"))
        try:
            return int(epochs)
        except (TypeError, ValueError):
            return None

    def _infer_exp_dir(self, cfg_path: Path, seed: int) -> Optional[Path]:
        if not yaml:
            return None
        try:
            with cfg_path.open("r", encoding="utf-8") as handle:
                payload = yaml.safe_load(handle) or {}
        except Exception:
            return None
        exp_name = payload.get("experiment_name", cfg_path.stem)
        if "_seed" not in str(exp_name):
            exp_name = f"{exp_name}_seed{seed}"
        log_cfg = payload.get("logging", {}) if isinstance(payload, dict) else {}
        output_dir = (
            log_cfg.get("output_dir")
            or log_cfg.get("base_dir")
            or log_cfg.get("save_dir")
            or "experiments"
        )
        out_path = Path(output_dir)
        if not out_path.is_absolute():
            out_path = (self.ml_root / out_path).resolve()
        return out_path / str(exp_name)

    def _find_plot_paths(self, exp_id: str) -> Tuple[Optional[Path], Optional[Path]]:
        plots_dir = self.ml_root / "experiments" / "plots"
        if not plots_dir.exists():
            return None, None
        exp_base = exp_id.split("_seed")[0]
        def pick_plot(suffix: str) -> Optional[Path]:
            candidates = [
                plots_dir / f"{exp_id}__{suffix}.png",
                plots_dir / f"{exp_base}__{suffix}.png",
            ]
            for cand in candidates:
                if cand.exists():
                    return cand
            for cand in plots_dir.glob(f"{exp_id}*__{suffix}.png"):
                return cand
            for cand in plots_dir.glob(f"{exp_base}*__{suffix}.png"):
                return cand
            return None
        return pick_plot("loss"), pick_plot("val_success")

    def _update_result_panel(self, rc: Optional[int] = None):
        exp_dir = self._active_exp_dir
        if not exp_dir:
            self.lbl_result_status.setText("Estado: -")
            return
        exp_id = exp_dir.name
        summary_path = self.ml_root / "experiments" / "summary_base.csv"
        entry = None
        if summary_path.exists():
            for row in self._load_csv(summary_path):
                if row.get("exp_id") == exp_id:
                    entry = row
                    break
        best_epoch = "-"
        best_val = "-"
        if entry:
            best_epoch = entry.get("best_epoch", "-")
            best_val = entry.get("val_success", "-")
        else:
            metrics = self._read_metrics_latest(exp_dir)
            if metrics and metrics.get("epoch"):
                best_epoch = metrics.get("epoch", "-")
                best_val = metrics.get("val_success", "-")

        if rc is None:
            status = "OK" if entry else "OK"
        else:
            status = "OK" if rc == 0 else "FAIL"
        self.lbl_result_status.setText(f"Estado: {status}")
        self.lbl_result_best_epoch.setText(f"best_epoch: {best_epoch}")
        self.lbl_result_best_metric.setText(f"best_val_success: {best_val}")
        self.lbl_result_exp.setText(f"Ruta EXP: {exp_dir}")
        metrics_path = exp_dir / "metrics.csv"
        if metrics_path.exists():
            self.lbl_result_metrics.setText(f"metrics.csv: {metrics_path}")
        else:
            self.lbl_result_metrics.setText("metrics.csv: -")
        loss_plot, success_plot = self._find_plot_paths(exp_id)
        plots = []
        if loss_plot:
            plots.append(str(loss_plot))
        if success_plot:
            plots.append(str(success_plot))
        self.lbl_result_plots.setText(f"Plots: {', '.join(plots) if plots else '-'}")
        pretty_md = self.ml_root / "experiments" / "summary_base_pretty.md"
        if pretty_md.exists():
            self.lbl_result_summary.setText(f"Resumen: {pretty_md}")
        else:
            self.lbl_result_summary.setText("Resumen: -")

    def _update_summary_panel(self, rc: int):
        status = "OK" if rc == 0 else "FAIL"
        summary_path = self.ml_root / "experiments" / "summary_base.csv"
        pretty_md = self.ml_root / "experiments" / "summary_base_pretty.md"
        by_seed_md = self.ml_root / "experiments" / "summary_by_seed.md"
        plots_dir = self.ml_root / "experiments" / "plots"
        summary_bits = []
        if summary_path.exists():
            summary_bits.append(str(summary_path))
        if pretty_md.exists():
            summary_bits.append(str(pretty_md))
        if by_seed_md.exists():
            summary_bits.append(str(by_seed_md))
        self.lbl_result_status.setText(f"Estado: {status}")
        self.lbl_result_best_epoch.setText("best_epoch: -")
        self.lbl_result_best_metric.setText("best_val_success: -")
        self.lbl_result_exp.setText("Ruta EXP: -")
        self.lbl_result_metrics.setText("metrics.csv: -")
        if plots_dir.exists():
            self.lbl_result_plots.setText(f"Plots: {plots_dir}")
        else:
            self.lbl_result_plots.setText("Plots: -")
        self.lbl_result_summary.setText(f"Resumen: {', '.join(summary_bits) if summary_bits else '-'}")

    def _load_csv(self, path: Path) -> List[Dict[str, str]]:
        if not path.exists():
            return []
        with path.open("r", newline="", encoding="utf-8") as handler:
            rows = list(csv.DictReader(handler))
        return rows

    def _update_preview(self):
        pixmap = None
        exp_dir = self._active_exp_dir
        if exp_dir:
            loss_plot, success_plot = self._find_plot_paths(exp_dir.name)
            pick = success_plot or loss_plot
            if pick and pick.exists():
                pixmap = QPixmap(str(pick))
        if pixmap is None or pixmap.isNull():
            base_figs = self.ml_root / "experiments" / "figures_memoria"
            for candidate in ["winner_val_success.png", "winner_loss.png"]:
                path = base_figs / candidate
                if path.exists():
                    pixmap = QPixmap(str(path))
                    break
        if pixmap and not pixmap.isNull():
            self.preview_label.setPixmap(pixmap.scaled(self.preview_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.preview_label.setText("Sin figura")
            self.preview_label.setPixmap(QPixmap())

    def _check_subset_integrity(self):
        audit_dir = self.ml_root / "reports" / "cornell_audit"
        required = [
            "clean_idx_train_v2.txt",
            "clean_idx_val.txt",
        ]
        missing = [name for name in required if not (audit_dir / name).exists()]
        if missing:
            self.subset_status.setText(
                f"[ML] ¡Faltan índices limpios: {', '.join(missing)}! No entrenes hasta auditarlos."
            )
        else:
            self.subset_status.setText("[ML] Auditoría Cornell presente.")

class EvidencesTab(QWidget):
    """UI tab for evidence folders, experiments, and snapshots."""

    def __init__(self, runner: CmdRunner, log_fn, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
        self._build_ui()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(4)
        self.btn_debug = QPushButton("Debug logs -> terminal")
        self.btn_debug.setCheckable(True)
        top_bar.addWidget(self.btn_debug)
        top_bar.addStretch(1)
        lay.addLayout(top_bar)

        g_export = QGroupBox("Capturas y exportación")
        g_export.setFlat(True)
        g_export.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        elay = QHBoxLayout()
        elay.setContentsMargins(1, 1, 1, 1)
        elay.setSpacing(3)

        btn_snapshot = QPushButton("Capturar evidencia (snapshot)")
        btn_graphs = QPushButton("Exportar gráficas")
        btn_tables = QPushButton("Exportar tablas")
        btn_folder = QPushButton("Abrir carpeta evidencias")
        elay.addWidget(btn_snapshot)
        elay.addWidget(btn_graphs)
        elay.addWidget(btn_tables)
        elay.addWidget(btn_folder)

        btn_snapshot.clicked.connect(self.snapshot)
        btn_graphs.clicked.connect(self.export_graphs)
        btn_tables.clicked.connect(self.export_tables)
        btn_folder.clicked.connect(self.open_evidencias_folder)

        g_export.setLayout(elay)
        lay.addWidget(g_export)

        g_exp = QGroupBox("Experimentos IA (agarre_inteligente)")
        g_exp.setFlat(True)
        g_exp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        xlay = QGridLayout()
        xlay.setContentsMargins(1, 1, 1, 1)
        xlay.setHorizontalSpacing(1)
        xlay.setVerticalSpacing(1)

        b4 = QPushButton("Start grasp node")
        b5 = QPushButton("Stop grasp node")
        b6 = QPushButton("Run EXP RGB")
        b7 = QPushButton("Run EXP RGB-D")
        b8 = QPushButton("Abrir experiments/")
        b9 = QPushButton("Abrir plots/")
        b10 = QPushButton("Abrir summary_base.csv")
        b11 = QPushButton("Abrir figures_memoria (IA)")
        b12 = QPushButton("Diagnóstico completo (IA/ROS)")

        b4.clicked.connect(self.start_grasp_node)
        b5.clicked.connect(self.stop_grasp_node)
        b6.clicked.connect(lambda: self.run_exp("run_experiment_rgb.sh"))
        b7.clicked.connect(lambda: self.run_exp("run_experiment_rgbd.sh"))
        b8.clicked.connect(self.open_vision_experiments)
        b9.clicked.connect(self.open_vision_plots)
        b10.clicked.connect(self.open_vision_summary)
        b11.clicked.connect(self.open_vision_figures)
        b12.clicked.connect(self.diag_full)

        xlay.addWidget(b4, 0, 0)
        xlay.addWidget(b5, 0, 1)
        xlay.addWidget(b6, 0, 2)
        xlay.addWidget(b7, 1, 0)
        xlay.addWidget(b8, 1, 1)
        xlay.addWidget(b9, 1, 2)
        xlay.addWidget(b10, 2, 0)
        xlay.addWidget(b11, 2, 1)
        xlay.addWidget(b12, 2, 2)
        g_exp.setLayout(xlay)
        lay.addWidget(g_exp)
        lay.addStretch(1)
        self.setLayout(lay)

    def export_graphs(self):
        self._run_export("figures", "[EVID] Exportando gráficas...")

    def export_tables(self):
        self._run_export("tables", "[EVID] Exportando tablas...")

    def _run_export(self, only_mode: str, message: str):
        ml_root = VISION_DIR
        if not os.path.isdir(ml_root):
            self.log_fn(f"[EVID] ERROR: workspace {ml_root} no disponible.")
            return
        docs_arg = os.path.join("docs", "tfm")
        cmd = (
            bash_preamble(WS_DIR)
            + f"cd '{ml_root}' && "
            "if [ -f '.venv/bin/activate' ]; then source .venv/bin/activate >/dev/null 2>&1; fi && "
            f"python scripts/export_evidencias.py --only {only_mode} --docs-root {docs_arg} || true"
        )
        self.log_fn(f"{message} (modo: {only_mode})")
        self.runner.run_stream("EVID-EXPORT", cmd)

    def open_evidencias_folder(self):
        path = os.path.join(VISION_DIR, "docs", "tfm")
        ensure_dir(path)
        subprocess.Popen(["bash", "-lc", f"xdg-open '{path}' >/dev/null 2>&1 || true"])

    def snapshot(self):
        ensure_dir(LOG_DIR)
        ensure_dir(FIG_DIR)
        dst = os.path.join(LOG_DIR, f"figures_memoria_snapshot_{now_tag()}")
        ensure_dir(dst)
        try:
            for fn in os.listdir(FIG_DIR):
                if fn.lower().endswith((".png", ".jpg", ".jpeg")):
                    shutil.copy2(os.path.join(FIG_DIR, fn), os.path.join(dst, fn))
            self.log_fn(f"[EVID] Snapshot -> {dst}")
        except Exception as e:
            self.log_fn(f"[EVID] ERROR snapshot: {e}")

    def start_grasp_node(self):
        self.run_exp("run_grasp_node.sh", tag="GRASP")

    def stop_grasp_node(self):
        cmd = "pkill -f 'grasp_pose_publisher|grasp_pose_node' || true"
        self.runner.run_stream("GRASP", cmd)

    def run_exp(self, script_name: str, tag: str = "EXP"):
        script_path = os.path.join(SCRIPTS_DIR, script_name)
        if not os.path.isfile(script_path):
            self.log_fn(f"[{tag}] ERROR: no existe {script_path}")
            return
        if not os.access(script_path, os.X_OK):
            self.log_fn(f"[{tag}] WARN: {script_path} no es ejecutable (chmod +x)")
        cmd = bash_preamble(WS_DIR) + f"'{script_path}' || true"
        self.runner.run_stream(tag, cmd)

    def open_vision_experiments(self):
        ensure_dir(VISION_EXP_DIR)
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_EXP_DIR}' >/dev/null 2>&1 || true"])

    def open_vision_plots(self):
        ensure_dir(VISION_PLOTS_DIR)
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_PLOTS_DIR}' >/dev/null 2>&1 || true"])

    def open_vision_summary(self):
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_SUMMARY}' >/dev/null 2>&1 || true"])

    def open_vision_figures(self):
        ensure_dir(VISION_FIG_DIR)
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_FIG_DIR}' >/dev/null 2>&1 || true"])

    def diag_full(self):
        self.log_fn("[DIAG] Diagnóstico completo (percepción + control)")
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

# =========================
# Logs Dialog
# =========================
class LogDialog(QDialog):
    """Popup dialog that shows the live logs."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Logs — Panel SUPER PRO")
        self.resize(900, 360)
        layout = QVBoxLayout()
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        self.text.setLineWrapMode(QTextEdit.NoWrap)
        # Limit total lines to keep UI responsive under heavy logging.
        self.text.document().setMaximumBlockCount(2000)
        layout.addWidget(self.text)
        self.setLayout(layout)

    def append(self, text: str):
        self.text.append(text)
        self.text.moveCursor(self.text.textCursor().End)

# =========================
# Main Window
# =========================
class MainWindow(QMainWindow):
    """Main application window for the SUPER PRO panel."""

    log_signal = pyqtSignal(str)
    robot_state_signal = pyqtSignal(bool, bool)
    pick_start_signal = pyqtSignal(str)
    unlock_robot_buttons_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
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

        self.tab_exp = ExperimentsTab(self.runner, self._append_log)
        self.tab_evid = EvidencesTab(self.runner, self._append_log)
        self.tab_sim = SimulationTab(self.runner, self.ros, self._append_log)
        self.tab_cam = CamerasTab(
            self.runner,
            self.ros,
            self._append_log,
            calib_start_fn=self._start_calibration,
            calib_click_fn=self._handle_calib_click,
            show_top_bar=False,
        )
        self.tab_robot = RobotTab(
            self.runner,
            self._append_log,
            self.infer_grasp_model,
            self.pick_and_place_selected,
            pose_cmd_fn=self._set_hold_script,
            hold_pause_fn=self._pause_hold,
            manual_in_tab=False,
        )
        self.tab_cam.set_manual_panel(self.tab_robot.manual_panel)
        self.tab_cam.set_status_panel(self.tab_sim.status_panel)
        self.unlock_robot_buttons_signal.connect(self.tab_robot.unlock_buttons)
        self._debug_buttons: List[Tuple[QAbstractButton, Callable[[], str]]] = []
        self._debug_active_button: Optional[QAbstractButton] = None
        self._debug_active_context: Optional[str] = None
        self._register_debug_button(
            self.tab_exp.btn_debug,
            lambda: f"Experimentos/{self.tab_exp.current_subtab_name()}",
        )
        self._register_debug_button(
            self.tab_sim.btn_debug,
            lambda: "Robot / ROS2-Gazebo",
        )
        self._register_debug_button(
            self.tab_evid.btn_debug,
            lambda: "Evidencias TFM",
        )
        self.tab_sim.state_changed.connect(self._schedule_robot_state_check)
        self.tab_sim.bridge_started.connect(self._on_bridge_started)
        self.tab_cam.target_selected.connect(self._on_target_selected)

        simulation_page = self._build_simulation_page()
        self.tabs_widget = QTabWidget()
        self.tabs_widget.addTab(self.tab_exp, "Experimentos")
        self.tabs_widget.addTab(simulation_page, "Robot / ROS2-Gazebo")
        self.tabs_widget.addTab(self.tab_evid, "Evidencias TFM")
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

        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        calib = load_table_calib()
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
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
        layout.setContentsMargins(1, 1, 1, 1)
        layout.setSpacing(1)

        left = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(1, 1, 1, 1)
        left_layout.setSpacing(1)
        left_layout.addWidget(self.tab_sim)
        left_bottom = QHBoxLayout()
        left_bottom.setContentsMargins(0, 0, 0, 0)
        left_bottom.setSpacing(1)
        left_bottom.setAlignment(Qt.AlignTop)
        left_bottom.addWidget(self.tab_robot)
        left_layout.addLayout(left_bottom)
        left.setLayout(left_layout)

        right = QWidget()
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(1, 1, 1, 1)
        right_layout.setSpacing(1)
        right_layout.addWidget(self.tab_cam)
        right.setLayout(right_layout)

        self.main_split = QSplitter(Qt.Horizontal)
        self.main_split.addWidget(left)
        self.main_split.addWidget(right)
        self.main_split.setStretchFactor(0, 1)
        self.main_split.setStretchFactor(1, 2)

        layout.addWidget(self._build_control_bar(), 0)
        if hasattr(self.tab_cam, "top_bar_widget") and self.tab_cam.top_bar_widget is not None:
            layout.addWidget(self.tab_cam.top_bar_widget, 0)
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
            self.tab_exp.set_debug_enabled(context.startswith("Experimentos"))
            self._append_log("[UI] Botón: Debug logs -> terminal (ON)")
            self._append_log(f"[DEBUG] Activado desde: {context}")
            return
        if self._debug_logs_to_stdout:
            self._append_log("[UI] Botón: Debug logs -> terminal (OFF)")
            self._append_log(f"[DEBUG] Desactivado desde: {context}")
        self._debug_logs_to_stdout = False
        self.tab_exp.set_debug_enabled(False)

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

    def _on_cmd_finished(self, tag: str, rc: int):
        if tag in {"ROBOT", "ROBOT-TEST", "ROBOT-CTRL", "ROBOT-DIAG", "PICK"}:
            self._robot_busy_count = max(0, self._robot_busy_count - 1)
            if self._robot_busy_count == 0:
                self.tab_robot.set_busy(False)
                self.tab_robot.unlock_buttons()

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
        self.tab_sim.set_robot_leds(ros2_running, gz_running)

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
        return SelectedTarget(px=px, py=py, world_x=best[1], world_y=best[2], object_name=best[0])

    def _set_hold_script(self, script_path: str):
        self._hold_script = script_path
        self._hold_pause_until = time.time() + 2.0

    def _pause_hold(self, seconds: float):
        self._hold_pause_until = time.time() + seconds

    def _update_calibration_state(self) -> None:
        self.tab_robot.set_calibrated(self._calibrated)
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
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
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

            cx, cy, cz = cam_t
            def world_to_pixel(wx: float, wy: float, wz: float) -> Tuple[float, float]:
                vx = wx - cx
                vy = wy - cy
                vz = wz - cz
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
                global TABLE_PIXEL_AFFINE
                global TABLE_PIXEL_RECT
                global TABLE_PIXEL_HOMOGRAPHY
                TABLE_PIXEL_AFFINE = None
                TABLE_PIXEL_RECT = None
                TABLE_PIXEL_HOMOGRAPHY = calib["h"]
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
        if not os.path.isfile(INFER_SCRIPT):
            self._append_log(f"[MODEL] ERROR: no existe {INFER_SCRIPT}")
            return
        venv_activate = os.path.join(VISION_DIR, ".venv", "bin", "activate")
        if not os.path.isfile(venv_activate):
            self._append_log(f"[MODEL] ERROR: no existe {venv_activate}")
            return
        if not os.path.isfile(INFER_CKPT):
            self._append_log(f"[MODEL] ERROR: no existe {INFER_CKPT}")
            return
        ensure_dir(LOG_DIR)
        img_path = os.path.join(LOG_DIR, "overhead_last.png")
        out_path = os.path.join(LOG_DIR, "overhead_grasp.json")
        tile.frame.qimg.save(img_path)

        def _worker():
            cmd = (
                f"source '{venv_activate}' && "
                f"cd '{VISION_DIR}' && "
                "export PYTHONPATH=\"$PWD/src:${PYTHONPATH:-}\" && "
                f"python '{INFER_SCRIPT}' --image '{img_path}' --ckpt '{INFER_CKPT}' --out '{out_path}'"
            )
            res = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
            if res.returncode != 0:
                self._append_log(f"[MODEL] ERROR infer: {res.stderr.strip() or res.stdout.strip()}")
                return
            try:
                with open(out_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                tile.grasp_rect = (
                    float(data["cx"]),
                    float(data["cy"]),
                    float(data["w"]),
                    float(data["h"]),
                    float(data["angle_deg"]),
                )
                QTimer.singleShot(0, lambda: tile.update_frame(
                    tile.frame.topic, tile.frame.qimg, tile.frame.w, tile.frame.h, tile.frame.fps
                ))
                self._append_log(
                    f"[MODEL] Grasp: cx={data['cx']:.1f} cy={data['cy']:.1f} "
                    f"w={data['w']:.1f} h={data['h']:.1f} ang={data['angle_deg']:.1f}"
                )
            except Exception as e:
                self._append_log(f"[MODEL] ERROR leyendo grasp: {e}")

        threading.Thread(target=_worker, daemon=True).start()

    def pick_and_place_selected(self):
        if self.tab_robot._busy:
            self._append_log("[PICK] Ocupado. Espera a que termine la acción actual.")
            return
        self._append_log("[UI] Botón: Pick & Place (demo)")
        self.tab_robot.lock_button(self.tab_robot.btn_pick)
        demo_obj = "pieza_pick_mesa"
        if demo_obj in OBJECT_POSITIONS:
            x, y, _z = OBJECT_POSITIONS[demo_obj]
            px, py = self._world_to_pixel(x, y)
            self.selected_target = SelectedTarget(px=px, py=py, world_x=x, world_y=y, object_name=demo_obj)
            self.tab_cam.set_selected(
                demo_obj,
                f"Selección demo: {demo_obj} @ ({x:.2f},{y:.2f})"
            )
        obj = self.selected_target.object_name if self.selected_target else demo_obj

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
            target = self.selected_target
            if not target:
                self._append_log("[PICK] No hay selección válida. Ajusta la cámara o recalibra.")
                self.unlock_robot_buttons_signal.emit()
                return
            self.pick_start_signal.emit(target.object_name)

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
        self._append_log("[SAFETY] Teleport detectado: bloqueado (move_object_to_basket)")
        return

    def _move_object_to_table(self, obj_name: str):
        self._append_log("[SAFETY] Teleport detectado: bloqueado (move_object_to_table)")
        return

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
