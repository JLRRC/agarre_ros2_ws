#!/usr/bin/env python3

import os
import shutil
import subprocess
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QTextEdit,
    QToolButton,
    QGroupBox,
    QSizePolicy,
    QDialog,
)
try:
    import yaml
except Exception:
    yaml = None
from panel_config import *
from panel_utils import *


class EvidencesTab(QWidget):
    """UI tab for evidence folders, experiments, and snapshots."""

    def __init__(self, runner: CmdRunner, log_fn, parent=None, show_top_bar: bool = True, compact: bool = False):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
        self._show_top_bar = show_top_bar
        self._compact = compact
        self._build_ui()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2 if self._compact else 1)

        if self._show_top_bar:
            top_bar = QHBoxLayout()
            top_bar.setContentsMargins(0, 0, 0, 0)
            top_bar.setSpacing(4)
            self.btn_debug = QPushButton("Debug logs -> terminal")
            self.btn_debug.setCheckable(True)
            top_bar.addWidget(self.btn_debug)
            top_bar.addStretch(1)
            self.btn_close_panel = QPushButton("Cerrar panel")
            self.btn_close_panel.clicked.connect(self._close_panel)
            top_bar.addWidget(self.btn_close_panel)
            lay.addLayout(top_bar)
        else:
            self.btn_debug = None

        g_exp = QGroupBox("Experimentos IA (agarre_inteligente)")
        g_exp.setFlat(True)
        g_exp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        xlay = QGridLayout()
        xlay.setContentsMargins(0, 0, 0, 0)
        xlay.setHorizontalSpacing(2)
        xlay.setVerticalSpacing(2)

        b4 = QPushButton("Start grasp node")
        b5 = QPushButton("Stop grasp node")
        b10 = QPushButton("Abrir summary_base.csv")
        b11 = QPushButton("Abrir figures_memoria (IA)")
        b12 = QPushButton("Diagnóstico completo (IA/ROS)")

        b4.clicked.connect(self.start_grasp_node)
        b5.clicked.connect(self.stop_grasp_node)
        b10.clicked.connect(self.open_vision_summary)
        b11.clicked.connect(self.open_vision_figures)
        b12.clicked.connect(self.diag_full)

        xlay.addWidget(b4, 0, 0)
        xlay.addWidget(b5, 0, 1)
        xlay.addWidget(b10, 0, 2)
        xlay.addWidget(b11, 1, 0)
        xlay.addWidget(b12, 1, 1)
        g_exp.setLayout(xlay)
        if self._compact:
            g_exp.setTitle("")
            lay.addWidget(self._wrap_collapsible("Experimentos IA (agarre_inteligente)", g_exp, checked=False))
        else:
            lay.addWidget(g_exp)
            if self._show_top_bar:
                lay.addStretch(1)
        self.setLayout(lay)
        if self._compact:
            self.setStyleSheet("QPushButton { padding:2px 6px; }")

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

    def export_graphs(self):
        self._run_export("figures", "[EVID] Exportando gráficas...")

    def export_tables(self):
        self._run_export("tables", "[EVID] Exportando tablas...")

    def _close_panel(self) -> None:
        self.window().close()

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
