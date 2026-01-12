#!/usr/bin/env python3

import os
import math
import time
import threading
import subprocess
from PyQt5.QtCore import (
    Qt,
    QTimer,
    pyqtSignal,
    QPointF,
)
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QPainter,
    QPen,
    QColor,
    QPolygonF,
    QFont,
    QFontMetrics,
)
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QCheckBox,
    QComboBox,
    QSizePolicy,
)
from dataclasses import dataclass
from typing import (
    Optional,
    Dict,
    List,
    Tuple,
    Set,
)
try:
    import yaml
except Exception:
    yaml = None
from .panel_config import *
from .panel_utils import *


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
    world_z: float = 0.0
    object_x: Optional[float] = None
    object_y: Optional[float] = None
    object_z: Optional[float] = None

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
        self.pickable = True
        self._hover = False
        self._selected = False
        self.setFixedHeight(16)
        self.setMouseTracking(True)
        self._font = QFont()
        self._font.setPointSize(7)
        self._font.setFamily("DejaVu Sans")

    def set_state(self, x: float, y: float, z: float, out: bool, pickable: bool = True):
        self.pos = (x, y, z)
        self.out = out
        self.pickable = pickable
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
        if self.out or not self.pickable:
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
        lock = "🔒 " if not self.pickable else ""
        text = f"{lock}{self.name}: ({x:.2f},{y:.2f}){status}"
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

    def update_objects(
        self,
        objects: Dict[str, Tuple[float, float, float]],
        pickable: Optional[Dict[str, bool]] = None,
    ):
        if not self.isVisible():
            return
        pickable = pickable or {}
        for name, (x, y, z) in sorted(objects.items()):
            visible = visible_table_object(name, (x, y, z))
            row = self._rows.get(name)
            if not row:
                row = ObjectRow(name)
                self._rows[name] = row
                self.list_box.addWidget(row)
                row.clicked.connect(self.selected.emit)
            out = object_out_of_reach(x, y) or not visible
            pickable_flag = pickable.get(name, True) and visible
            if pickable.get(name) is True:
                out = False
            row.set_state(x, y, z, out, pickable_flag)

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
        self.grasp_info: Optional[Dict[str, object]] = None
        self.show_grasp_overlay = True
        self._selection_target: Optional[SelectedTarget] = None
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
        self.grasp_lbl = QLabel("Grasp: -")
        self.grasp_lbl.setStyleSheet("color:#6b7280; font-size:10px;")
        self.grasp_lbl.setWordWrap(True)
        lay.addWidget(self.grasp_lbl)

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

        need_overlay = (
            (self.name.lower().startswith("mesa") and getattr(self, "show_grid", False))
            or self.selected_px
            or (self.grasp_rect and self.show_grasp_overlay)
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

        if self.grasp_rect and self.show_grasp_overlay:
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
            center_pen = QPen(QColor("#f59e0b"))
            center_pen.setWidth(1)
            painter.setPen(center_pen)
            painter.drawLine(QPointF(gcx - 6.0, gcy), QPointF(gcx + 6.0, gcy))
            painter.drawLine(QPointF(gcx, gcy - 6.0), QPointF(gcx, gcy + 6.0))
            painter.setBrush(QColor(245, 158, 11, 60))
            painter.drawEllipse(QPointF(gcx, gcy), 4, 4)
            painter.end()
        if self.selected_px and self.grasp_rect and self.show_grasp_overlay:
            scx, scy = self.selected_px
            gcx, gcy, _, _, _ = self.grasp_rect
            painter = QPainter(draw)
            pen = QPen(QColor("#ef4444"))
            pen.setWidth(1)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(QPointF(scx, scy), QPointF(gcx, gcy))
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
        self._selection_target = target
        if not target:
            self.coord_lbl.setText("Cruce: - | Objetivo: -")
            return
        obj_text = f"{target.object_name}"
        if target.object_x is not None and target.object_y is not None and target.object_z is not None:
            obj_text += f" ({target.object_x:.2f},{target.object_y:.2f},{target.object_z:.2f})"
        self.coord_lbl.setText(
            f"Cruce px=({target.px},{target.py}) "
            f"world=({target.world_x:.2f},{target.world_y:.2f},{target.world_z:.2f}) "
            f"| Obj: {obj_text}"
        )

    def set_grasp_info(self, info: Optional[Dict[str, object]]) -> None:
        self.grasp_info = info
        if not info:
            self.grasp_lbl.setText("Grasp: -")
            return
        line1 = (
            f"Cornell: cx={info.get('cx', 0.0):.1f} cy={info.get('cy', 0.0):.1f} "
            f"w={info.get('w', 0.0):.1f} h={info.get('h', 0.0):.1f} "
            f"ang={info.get('angle_deg', 0.0):.1f}"
        )
        corners = info.get("corners_px")
        if isinstance(corners, list) and corners:
            short = " ".join([f"({c[0]:.0f},{c[1]:.0f})" for c in corners[:4] if isinstance(c, (list, tuple)) and len(c) >= 2])
            line1 = f"{line1} | corners: {short}"
        if "world_x" in info and "world_y" in info:
            line2 = (
                f"World: x={info.get('world_x', 0.0):.3f} "
                f"y={info.get('world_y', 0.0):.3f} z={info.get('world_z', 0.0):.3f}"
            )
        else:
            line2 = "World: -"
        err_px = info.get("err_px")
        err_xy = info.get("err_xy")
        if err_px is not None and err_xy is not None:
            line3 = (
                f"Err: px=({err_px[0]:.1f},{err_px[1]:.1f}) "
                f"xy=({err_xy[0]:.3f},{err_xy[1]:.3f})"
            )
        elif err_px is not None:
            line3 = f"Err: px=({err_px[0]:.1f},{err_px[1]:.1f})"
        else:
            line3 = "Err: -"
        self.grasp_lbl.setText(f"{line1}\n{line2}\n{line3}")
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
        if self.name.lower().startswith("mesa"):
            try:
                manual_dir = os.path.join(WS_DIR, "reports", "tfm_evidencias", "manual")
                ensure_dir(manual_dir)
                for old in os.listdir(manual_dir):
                    if old.startswith("ilustracion_1_1_a_gazebo."):
                        try:
                            os.unlink(os.path.join(manual_dir, old))
                        except OSError:
                            pass
                dst = os.path.join(manual_dir, "ilustracion_1_1_a_gazebo.png")
                shutil.copy2(out, dst)
                self.log_fn(f"[TFM] Ilustracion 1.1 (a) actualizada: {dst}")
            except Exception as e:
                self.log_fn(f"[TFM] WARN: no pude actualizar ilustracion 1.1 (a): {e}")

# =========================
# Tabs
# =========================

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
        infer_grasp_fn=None,
        show_top_bar: bool = True,
        parent=None,
    ):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn
        self.calib_start_fn = calib_start_fn
        self.calib_click_fn = calib_click_fn
        self.infer_grasp_fn = infer_grasp_fn
        self._show_top_bar = show_top_bar
        ros2_only = os.environ.get("PANEL_ROS2_ONLY", "0") == "1"
        self._single_cam = os.environ.get("PANEL_SINGLE_CAM", "1" if ros2_only else "0") == "1"
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

    def capture_mesa_snapshot(self) -> bool:
        if not self.tiles:
            self.log_fn("[EVID] Mesa: no hay tiles.")
            return False
        mesa = self.tiles[0]
        if not mesa.frame.qimg:
            self.log_fn("[EVID] Mesa: no hay frame.")
            return False
        mesa.on_shot()
        return True

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
        self.btn_infer = QPushButton("Inferir grasp (modelo)")
        self.btn_infer.setToolTip("Infere el grasp y dibuja la representacion Cornell en Mesa.")
        self.btn_clear_grasp = QPushButton("Limpiar grasp")
        self.btn_clear_grasp.setToolTip("Quita la ultima inferencia dibujada.")
        self.chk_show_grasp = QCheckBox("Mostrar grasp (Cornell)")
        self.chk_show_grasp.setChecked(True)
        b1.clicked.connect(self.discover_topics)
        b2.clicked.connect(self.open_evidences)
        b3.clicked.connect(self.start_calib)
        self.btn_infer.clicked.connect(self._infer_grasp)
        self.btn_clear_grasp.clicked.connect(self._clear_grasp_overlay)
        self.chk_show_grasp.toggled.connect(self._toggle_grasp_overlay)
        if not self.infer_grasp_fn:
            self.btn_infer.setEnabled(False)
        top.addWidget(b1)
        top.addWidget(b2)
        top.addWidget(b3)
        top.addWidget(self.btn_infer)
        top.addWidget(self.btn_clear_grasp)
        top.addWidget(self.chk_show_grasp)
        top.addStretch(1)
        self.top_bar_widget.setLayout(top)
        if self._show_top_bar:
            lay.addWidget(self.top_bar_widget)

        names = ["Mesa"] if self._single_cam else ["Mesa", "Frente", "Lateral"]
        for nm in names:
            t = CameraTile(nm, self.ros, self.log_fn)
            t.clicked.connect(lambda x, y, tile=t: self._on_tile_click(tile, x, y))
            self.tiles.append(t)

        # Ajuste inicial para proporciones 4:3, se recalcula al recibir imagen
        for tile in self.tiles:
            tile.aspect_ratio = 0.75

        mesa_tile = self.tiles[0]
        cam_grid = QGridLayout()
        cam_grid.setContentsMargins(0, 0, 0, 0)
        cam_grid.setHorizontalSpacing(4)
        cam_grid.setVerticalSpacing(2)
        self._status_placeholder = QWidget()
        self._status_placeholder.setMinimumWidth(140)
        cam_grid.addWidget(self._status_placeholder, 0, 0, 2, 1, Qt.AlignTop)
        if self._single_cam:
            cam_grid.addWidget(mesa_tile, 0, 1, 2, 1, Qt.AlignTop)
        else:
            front_tile = self.tiles[1]
            lateral_tile = self.tiles[2]
            cam_row = QHBoxLayout()
            cam_row.setSpacing(1)
            cam_row.addWidget(front_tile)
            cam_row.addWidget(lateral_tile)
            cam_grid.addWidget(mesa_tile, 0, 1, Qt.AlignTop)
            cam_grid.addLayout(cam_row, 1, 1, Qt.AlignTop)
        cam_grid.addWidget(self.obj_panel, 0, 2, Qt.AlignTop)
        cam_grid.setColumnStretch(0, 0)
        cam_grid.setColumnStretch(1, 1)
        cam_grid.setColumnStretch(2, 0)
        cam_grid.setRowStretch(0, 0)
        cam_grid.setRowStretch(1, 0)
        self._cam_grid = cam_grid
        lay.addLayout(cam_grid)
        lay.addStretch(1)

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

    def _infer_grasp(self):
        if not self.infer_grasp_fn:
            self.log_fn("[MODEL] Inference no disponible en este panel.")
            return
        self.infer_grasp_fn()

    def _toggle_grasp_overlay(self, enabled: bool):
        tile = self._mesa_tile()
        if not tile:
            return
        tile.show_grasp_overlay = enabled
        if tile.frame.qimg:
            tile.update_frame(
                tile.frame.topic,
                tile.frame.qimg,
                tile.frame.w,
                tile.frame.h,
                tile.frame.fps,
                force=True,
            )

    def _clear_grasp_overlay(self):
        tile = self._mesa_tile()
        if not tile:
            return
        tile.grasp_rect = None
        tile.set_grasp_info(None)
        if tile.frame.qimg:
            tile.update_frame(
                tile.frame.topic,
                tile.frame.qimg,
                tile.frame.w,
                tile.frame.h,
                tile.frame.fps,
                force=True,
            )
        self.log_fn("[MODEL] Grasp limpiado.")

    def _mesa_tile(self) -> Optional[CameraTile]:
        for tile in self.tiles:
            if tile.name.lower().startswith("mesa"):
                return tile
        return None

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
        if self._single_cam:
            defaults = ["/camera_overhead/image"]
        else:
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
            world_x, world_y = pixel_to_table_xy(px, py, tile.frame.w, tile.frame.h, z_target=obj_pos[2])
        obj_x = obj_y = obj_z = None
        world_z = 0.0
        if obj_pos:
            obj_x, obj_y, obj_z = obj_pos
            world_z = obj_z
        if obj_pos:
            dx = world_x - obj_pos[0]
            dy = world_y - obj_pos[1]
            if math.hypot(dx, dy) <= SELECTION_SNAP_DIST:
                # Snap to object center to keep selection aligned after minor mapping drift.
                world_x, world_y = obj_pos[0], obj_pos[1]
        target = SelectedTarget(
            px=px,
            py=py,
            world_x=world_x,
            world_y=world_y,
            world_z=world_z,
            object_name=obj_name,
            object_x=obj_x,
            object_y=obj_y,
            object_z=obj_z,
        )
        self.target_selected.emit(target)
        self.obj_panel.set_selected(obj_name, f"Selección: {obj_name} @ ({world_x:.2f},{world_y:.2f})")
        self.log_fn(
            f"[PICK] Selección: {obj_name} @ px=({px},{py}) -> world=({world_x:.2f},{world_y:.2f})"
        )

    def _on_object_selected(self, name: str):
        if name not in OBJECT_POSITIONS:
            return
        x, y, z = OBJECT_POSITIONS[name]
        self.obj_panel.set_selected(name, f"Selección: {name} @ ({x:.2f},{y:.2f})")
        px = 0
        py = 0
        for t in self.tiles:
            if "/camera_overhead/image" in (t.frame.topic or ""):
                pix = world_xyz_to_pixel(x, y, z, t.frame.w, t.frame.h)
                if not pix:
                    pix = table_xy_to_pixel(x, y, t.frame.w, t.frame.h)
                if pix:
                    px, py = pix
                    t.set_selected_pixel(px, py)
                break
        target = SelectedTarget(
            px=px,
            py=py,
            world_x=x,
            world_y=y,
            world_z=z,
            object_name=name,
            object_x=x,
            object_y=y,
            object_z=z,
        )
        self.target_selected.emit(target)

    def set_selected(self, name: Optional[str], text: str):
        self.obj_panel.set_selected(name, text)

    def update_objects(self):
        self.obj_panel.update_objects(get_object_positions())
