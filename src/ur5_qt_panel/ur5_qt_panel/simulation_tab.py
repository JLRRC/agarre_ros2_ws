#!/usr/bin/env python3

import os
import time
import shlex
import json
import math
import subprocess
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QToolButton,
    QGroupBox,
    QComboBox,
    QLineEdit,
    QFileDialog,
    QSizePolicy,
)
from typing import Optional, Dict
try:
    import yaml
except Exception:
    yaml = None
from panel_config import *
from panel_utils import *
from evidences_tab import EvidencesTab


class SimulationTab(QWidget):
    """UI tab for Gazebo, bridge, and rosbag management."""

    bridge_started = pyqtSignal()
    state_changed = pyqtSignal()

    def __init__(self, runner: CmdRunner, ros: RosWorker, log_fn, parent=None, show_evidences: bool = True):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn
        self._ros2_only = os.environ.get("PANEL_ROS2_ONLY", "0") == "1"
        self._show_evidences = bool(show_evidences)

        self.procs: Dict[str, subprocess.Popen] = {}
        self.gz_partition: str = ""
        self._gz_running = False
        self._bridge_running = False
        self._bag_running = False
        self._sys_last_total: Optional[int] = None
        self._sys_last_idle: Optional[int] = None
        self._sys_logged_once = False
        self.evid_panel = None

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
        self.lbl_ur5_pose = QLabel("UR5: -")
        self.btn_ur5_pose = QPushButton("Actualizar pose UR5")

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
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        left = QVBoxLayout()
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(2)
        left.setAlignment(Qt.AlignTop)

        # Mundo
        g_world = QGroupBox("Mundo (Gazebo)")
        g_world.setFlat(True)
        g_world.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        wlay = QVBoxLayout()
        wlay.setContentsMargins(6, 8, 6, 6)
        wlay.setSpacing(2)
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
        g_gz.setFlat(True)
        g_gz.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        gzlay = QVBoxLayout()
        gzlay.setContentsMargins(6, 8, 6, 6)
        gzlay.setSpacing(2)

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
        g_br.setFlat(True)
        g_br.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        brlay = QVBoxLayout()
        brlay.setContentsMargins(6, 8, 6, 6)
        brlay.setSpacing(2)
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
        g_bag.setFlat(True)
        g_bag.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        baglay = QVBoxLayout()
        baglay.setContentsMargins(6, 8, 6, 6)
        baglay.setSpacing(2)
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

        if self._show_evidences and not self._ros2_only:
            self.evid_panel = EvidencesTab(self.runner, self.log_fn, parent=self, show_top_bar=False, compact=True)
            left.addWidget(self.evid_panel)

        # no stretch: keep compact

        # Status panel (moved near cameras)
        self.status_panel = QWidget()
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(2)
        g_status = QGroupBox("Estado")
        g_status.setFlat(True)
        g_status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        slay = QGridLayout()
        slay.setContentsMargins(6, 8, 6, 6)
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
        syslay.setContentsMargins(6, 8, 6, 6)
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

        g_pose = QGroupBox("Pose UR5 (Gazebo)")
        g_pose.setFlat(True)
        g_pose.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        play = QVBoxLayout()
        play.setContentsMargins(6, 8, 6, 6)
        play.setSpacing(2)
        self.lbl_ur5_pose.setWordWrap(True)
        self.btn_ur5_pose.clicked.connect(self._refresh_ur5_pose)
        play.addWidget(self.lbl_ur5_pose)
        play.addWidget(self.btn_ur5_pose)
        g_pose.setLayout(play)
        right.addWidget(g_pose)

        right.addWidget(self.btn_debug)

        g_diag = QGroupBox("Diagnóstico rápido")
        g_diag.setFlat(True)
        g_diag.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        dlay = QVBoxLayout()
        dlay.setContentsMargins(6, 6, 6, 6)
        dlay.setSpacing(2)

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
        right.addWidget(self._wrap_collapsible("Diagnóstico rápido", g_diag, checked=False))
        self.status_panel.setLayout(right)

        layout.addLayout(left, 3)
        self.setLayout(layout)
        self.setStyleSheet(
            "QGroupBox { margin-top: 10px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px; }"
        )

    def _current_world_name(self) -> str:
        world = self.world_combo.currentText().strip()
        if not world:
            return GZ_WORLD
        if os.path.isfile(world):
            return read_world_name(world)
        cand = os.path.join(WORLDS_DIR, world)
        if os.path.isfile(cand):
            return read_world_name(cand)
        if not cand.endswith(".sdf") and os.path.isfile(cand + ".sdf"):
            return read_world_name(cand + ".sdf")
        return GZ_WORLD

    def _parse_pose_json(self, raw: str):
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
            if isinstance(poses, list) and poses:
                return poses
        return []

    def _quat_to_rpy(self, x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _refresh_ur5_pose(self):
        if not self._gz_running:
            self.lbl_ur5_pose.setText("UR5: N/A (Gazebo apagado)")
            return
        world_name = self._current_world_name()
        part = resolve_gz_partition(self.gz_partition)
        env = build_gz_env(part)
        cmd = (
            bash_preamble(WS_DIR)
            + env
            + f"gz topic -e -n 1 -t '/world/{world_name}/pose/info' --json-output"
        )
        raw, err = run_cmd_output(cmd, timeout_sec=2.0)
        if err:
            self.lbl_ur5_pose.setText(f"UR5: N/A ({err})")
            return
        poses = self._parse_pose_json(raw)
        if not poses:
            self.lbl_ur5_pose.setText("UR5: N/A (sin poses)")
            return
        candidates = ("ur5_rg2", "ur5")
        target = None
        found_name = ""
        for name in candidates:
            for pose in poses:
                if not isinstance(pose, dict):
                    continue
                if pose.get("name") == name:
                    target = pose
                    found_name = name
                    break
            if target:
                break
        if target is None:
            for pose in poses:
                if not isinstance(pose, dict):
                    continue
                nm = str(pose.get("name") or "")
                if nm.startswith("ur5_rg2::") or nm.startswith("ur5::"):
                    target = pose
                    found_name = nm
                    break
        if target is None:
            self.lbl_ur5_pose.setText("UR5: no encontrado en pose/info")
            return
        pos = target.get("position") or {}
        ori = target.get("orientation") or {}
        try:
            x = float(pos.get("x"))
            y = float(pos.get("y"))
            z = float(pos.get("z"))
        except (TypeError, ValueError):
            self.lbl_ur5_pose.setText("UR5: pose inválida")
            return
        ox = float(ori.get("x", 0.0) or 0.0)
        oy = float(ori.get("y", 0.0) or 0.0)
        oz = float(ori.get("z", 0.0) or 0.0)
        ow = float(ori.get("w", 1.0) or 1.0)
        roll, pitch, yaw = self._quat_to_rpy(ox, oy, oz, ow)
        self.lbl_ur5_pose.setText(
            f"UR5 ({found_name}) x={x:.3f} y={y:.3f} z={z:.3f} | "
            f"rpy={math.degrees(roll):.1f}/{math.degrees(pitch):.1f}/{math.degrees(yaw):.1f} deg"
        )

    def _wrap_collapsible(self, title: str, widget: QWidget, checked: bool = True) -> QWidget:
        header = QToolButton()
        header.setCheckable(True)
        header.setChecked(checked)
        header.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        header.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        header.setText(title)
        widget.setVisible(checked)

        def _toggle(state: bool):
            widget.setVisible(state)
            header.setArrowType(Qt.DownArrow if state else Qt.RightArrow)

        header.toggled.connect(_toggle)
        container = QWidget()
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)
        vbox.addWidget(header)
        vbox.addWidget(widget)
        container.setLayout(vbox)
        return container

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
