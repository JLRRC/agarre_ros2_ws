#!/usr/bin/env python3

import os
import re
import time
import shutil
import shlex
import json
import random
import threading
import subprocess
import csv
from PyQt5.QtCore import (
    Qt,
    QTimer,
    pyqtSignal,
    QProcess,
)
from PyQt5.QtGui import QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QTextBrowser,
    QGroupBox,
    QDialog,
    QComboBox,
    QCheckBox,
    QToolButton,
    QSizePolicy,
    QMessageBox,
    QSpinBox,
    QProgressBar,
    QPlainTextEdit,
)
from pathlib import Path
from typing import (
    Optional,
    Dict,
    List,
    Tuple,
)
try:
    import yaml
except Exception:
    yaml = None
from panel_config import *
from panel_utils import *


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
        self._assistant_thread: Optional[threading.Thread] = None
        self._assistant_ran_cleanup = False
        self._assistant_steps: Dict[str, Dict[str, object]] = {}
        self._exp_registry: Dict[str, Dict[str, str]] = {}
        self._exp_registry_path = self._resolve_registry_path()
        self._exp_registry_loaded = False
        self._auto_pipeline_running = False
        self._auto_pipeline_steps: List[Tuple[str, object]] = []
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
        QTimer.singleShot(200, lambda: self._auto_summary_ab("inicio"))

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
        self.btn_auto_pipeline = QPushButton("🚀 Auto EXPs")
        self.btn_auto_pipeline.clicked.connect(self._start_auto_pipeline)
        top_bar.addWidget(self.btn_auto_pipeline)
        self.chk_auto_summary = QCheckBox("Auto resumen A/B")
        self.chk_auto_summary.setChecked(True)
        top_bar.addWidget(self.chk_auto_summary)
        self.btn_open_summary = QPushButton("Abrir summary_base.csv")
        self.btn_open_summary.clicked.connect(self._open_summary_base)
        top_bar.addWidget(self.btn_open_summary)
        self.btn_open_figures = QPushButton("Abrir figures_memoria (IA)")
        self.btn_open_figures.clicked.connect(self._open_figures_memoria)
        top_bar.addWidget(self.btn_open_figures)
        top_bar.addStretch(1)
        self.btn_close_panel = QPushButton("Cerrar panel")
        self.btn_close_panel.clicked.connect(self._close_panel)
        top_bar.addWidget(self.btn_close_panel)
        lay.addLayout(top_bar)

        self.g_assistant_compact = QGroupBox("Asistente")
        self.g_assistant_compact.setFlat(True)
        self.g_assistant_compact.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        self.g_assistant_compact.setMaximumHeight(42)
        compact_layout = QHBoxLayout()
        compact_layout.setContentsMargins(2, 2, 2, 2)
        compact_layout.setSpacing(4)
        self.lbl_assistant_state = QLabel("Estado: ...")
        self.lbl_assistant_state.setStyleSheet("font-weight:600; font-size:10px;")
        compact_layout.addWidget(self.lbl_assistant_state)
        compact_layout.addStretch(1)
        self.btn_assistant_cleanup = QPushButton("🧹 Limpiar")
        self.btn_assistant_cleanup.clicked.connect(self._on_assistant_cleanup_clicked)
        self.btn_assistant_recheck = QPushButton("Recomprobar")
        self.btn_assistant_recheck.clicked.connect(self._on_assistant_recheck_clicked)
        self.btn_assistant_show = QPushButton("Ver checklist")
        self.btn_assistant_show.clicked.connect(self._show_assistant_dialog)
        compact_layout.addWidget(self.btn_assistant_cleanup)
        compact_layout.addWidget(self.btn_assistant_recheck)
        compact_layout.addWidget(self.btn_assistant_show)
        self.g_assistant_compact.setLayout(compact_layout)
        self.g_assistant_compact.setStyleSheet(
            "QGroupBox { margin-top: 2px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 2px; }"
        )
        lay.addWidget(self.g_assistant_compact)

        self.assistant_dialog = QDialog(self)
        self.assistant_dialog.setWindowTitle("Asistente Experimentos")
        self.assistant_dialog.resize(900, 360)
        dialog_layout = QVBoxLayout()
        dialog_layout.setContentsMargins(6, 6, 6, 6)
        dialog_layout.setSpacing(6)
        self.lbl_assistant_state_dialog = QLabel("Estado: ...")
        self.lbl_assistant_state_dialog.setStyleSheet("font-weight:bold;")
        dialog_layout.addWidget(self.lbl_assistant_state_dialog)
        self.g_assistant = QGroupBox("Checklist")
        self.g_assistant.setFlat(True)
        self.g_assistant.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        assist_layout = QVBoxLayout()
        assist_layout.setContentsMargins(4, 4, 4, 4)
        assist_layout.setSpacing(6)
        self.assistant_steps_layout = QVBoxLayout()
        self.assistant_steps_layout.setSpacing(4)
        assist_layout.addLayout(self.assistant_steps_layout)
        self.g_assistant.setLayout(assist_layout)
        dialog_layout.addWidget(self.g_assistant)
        self.assistant_dialog.setLayout(dialog_layout)

        conf_header = QHBoxLayout()
        conf_header.setContentsMargins(0, 0, 0, 0)
        conf_header.setSpacing(4)
        conf_title = QLabel("Experimentos — Configuraciones")
        conf_title.setStyleSheet("font-weight:600;")
        self.btn_toggle_conf = QToolButton()
        self.btn_toggle_conf.setCheckable(True)
        self.btn_toggle_conf.setChecked(True)
        self.btn_toggle_conf.setArrowType(Qt.DownArrow)
        self.btn_toggle_conf.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.btn_toggle_conf.setText("Ocultar")
        conf_header.addWidget(conf_title)
        conf_header.addStretch(1)
        conf_header.addWidget(self.btn_toggle_conf)
        lay.addLayout(conf_header)

        g_conf = QGroupBox("")
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
        self.btn_toggle_conf.toggled.connect(
            lambda checked: self._toggle_config_panel(checked, g_conf)
        )

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
        status_col.setContentsMargins(2, 2, 2, 2)
        status_col.setSpacing(4)

        status_layout = QGridLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setHorizontalSpacing(4)
        status_layout.setVerticalSpacing(2)

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

        result_box = QGroupBox("Resultado final")
        result_box.setFlat(True)
        result_layout = QGridLayout()
        result_layout.setContentsMargins(2, 2, 2, 2)
        result_layout.setHorizontalSpacing(4)
        result_layout.setVerticalSpacing(2)
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

        g_preview = QGroupBox("Mini vista — plots")
        g_preview.setFlat(True)
        preview_layout = QHBoxLayout()
        preview_layout.setContentsMargins(2, 2, 2, 2)
        self.preview_label = QLabel("Sin figura")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(360, 220)
        self.preview_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.preview_label.setStyleSheet("border:1px solid #cbd5f5;")
        preview_layout.addWidget(self.preview_label, 1)
        g_preview.setLayout(preview_layout)
        g_preview.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        status_grid_widget = QWidget()
        status_grid_widget.setLayout(status_layout)
        status_grid_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

        status_col.addWidget(status_grid_widget)

        result_row = QHBoxLayout()
        result_row.setContentsMargins(0, 0, 0, 0)
        result_row.setSpacing(6)
        result_row.addWidget(g_preview, 3)
        result_row.addWidget(result_box, 2)
        status_col.addLayout(result_row)

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

    def _toggle_config_panel(self, checked: bool, panel: QGroupBox) -> None:
        panel.setVisible(bool(checked))
        if hasattr(self, "btn_toggle_conf") and self.btn_toggle_conf:
            self.btn_toggle_conf.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
            self.btn_toggle_conf.setText("Ocultar" if checked else "Mostrar")

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
        icon = QLabel("")
        icon.setFixedSize(12, 12)
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet("background:#94a3b8; border-radius:6px;")
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
            if self._assistant_thread and not self._assistant_thread.is_alive():
                self._assistant_running = False
            else:
                self._append_log_line("[ML] Asistente: comprobacion en curso.")
                return
        self._assistant_running = True
        self._set_assistant_state_text("Estado: comprobando...", "#64748b")
        self._append_log_line("[ML] Asistente: recomprobando todo.")
        if initial and not self._assistant_ran_cleanup:
            self.cleanup_experiments_panel()
            self._assistant_ran_cleanup = True

        def _worker():
            for key in list(self._assistant_steps.keys()):
                result = self._assistant_steps[key]["check_fn"]()
                self.assistant_step_ready.emit(key, result)
            self._assistant_running = False
            self._assistant_thread = None

        self._assistant_thread = threading.Thread(target=_worker, daemon=True)
        self._assistant_thread.start()

    def _run_single_assistant_check(self, key: str) -> None:
        if key not in self._assistant_steps:
            return
        if self._assistant_running:
            if self._assistant_thread and not self._assistant_thread.is_alive():
                self._assistant_running = False
            else:
                self._append_log_line("[ML] Asistente: comprobacion en curso.")
                return
        self._assistant_running = True
        self._set_assistant_state_text("Estado: comprobando...", "#64748b")
        self._append_log_line(f"[ML] Asistente: comprobando {key}.")

        def _worker():
            result = self._assistant_steps[key]["check_fn"]()
            self.assistant_step_ready.emit(key, result)
            self._assistant_running = False
            self._assistant_thread = None

        self._assistant_thread = threading.Thread(target=_worker, daemon=True)
        self._assistant_thread.start()

    def _on_assistant_cleanup_clicked(self) -> None:
        self._append_log_line("[ML] Asistente: limpieza solicitada.")
        self.cleanup_experiments_panel()
        QTimer.singleShot(80, lambda: self._run_assistant_checks(initial=False))

    def _on_assistant_recheck_clicked(self) -> None:
        self._append_log_line("[ML] Asistente: recomprobacion solicitada.")
        QTimer.singleShot(0, lambda: self._run_assistant_checks(initial=False))

    def _apply_assistant_step(self, key: str, result: Dict[str, str]) -> None:
        step = self._assistant_steps.get(key)
        if not step:
            return
        status = result.get("status", "orange")
        summary = result.get("summary", "")
        details = result.get("details", "")
        self._set_assistant_icon(step["icon"], status)
        step["summary"].setText(f"{step['title']}: {summary}")
        step["details"].setPlainText(details)
        step["status"] = status
        self._assistant_results[key] = status
        self._update_assistant_global_state()

    def _set_assistant_icon(self, label: QLabel, status: str) -> None:
        color_map = {
            "green": "#16a34a",
            "orange": "#f59e0b",
            "red": "#dc2626",
            "grey": "#94a3b8",
        }
        color = color_map.get(status, color_map["orange"])
        label.setStyleSheet(f"background:{color}; border-radius:6px;")

    def _set_assistant_state_text(self, text: str, color: str):
        self.lbl_assistant_state.setText(text)
        self.lbl_assistant_state.setStyleSheet(f"font-weight:bold; color:{color};")
        if hasattr(self, "lbl_assistant_state_dialog"):
            self.lbl_assistant_state_dialog.setText(text)
            self.lbl_assistant_state_dialog.setStyleSheet(f"font-weight:bold; color:{color};")

    def _update_assistant_global_state(self) -> None:
        statuses = list(self._assistant_results.values())
        if not statuses:
            self.lbl_assistant_state.setText("Estado: ...")
            if hasattr(self, "lbl_assistant_state_dialog"):
                self.lbl_assistant_state_dialog.setText("Estado: ...")
            return
        if "red" in statuses:
            text = "Estado: BLOQUEADO"
            style = "font-weight:bold; color:#dc2626;"
        elif "orange" in statuses:
            text = "Estado: FALTAN COSAS"
            style = "font-weight:bold; color:#f59e0b;"
        else:
            text = "Estado: LISTO PARA ENTRENAR"
            style = "font-weight:bold; color:#16a34a;"
        self.lbl_assistant_state.setText(text)
        self.lbl_assistant_state.setStyleSheet(style)
        if hasattr(self, "lbl_assistant_state_dialog"):
            self.lbl_assistant_state_dialog.setText(text)
            self.lbl_assistant_state_dialog.setStyleSheet(style)

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
                "mods=['torch','numpy','yaml','pandas','graspnet']\n"
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
            env = os.environ.copy()
            env["PYTHONPATH"] = f"{self.ml_root}:{self.ml_root / 'src'}:{env.get('PYTHONPATH','')}"
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8, cwd=self.ml_root, env=env)
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
        summary = exp_dir / "summary_base.csv"
        pretty = exp_dir / "summary_base_pretty.md"
        needs_refresh, detail = self._summary_needs_refresh()
        if not needs_refresh and summary.exists() and summary.stat().st_size > 0 and pretty.exists():
            return {
                "status": "green",
                "summary": "summary_base OK",
                "details": f"{summary}\n{pretty}",
            }
        if summary.exists():
            return {
                "status": "orange",
                "summary": "Resumen A/B desactualizado",
                "details": detail or "Ejecuta Generar resumen A/B.",
            }
        return {
            "status": "orange",
            "summary": "Resumen A/B no generado",
            "details": "Ejecuta Generar resumen A/B.",
        }

    def _summary_needs_refresh(self) -> Tuple[bool, str]:
        exp_dir = self.ml_root / "experiments"
        if not exp_dir.exists():
            return False, "experiments/ no existe"
        summary = exp_dir / "summary_base.csv"
        pretty = exp_dir / "summary_base_pretty.md"
        if not summary.exists():
            return True, "summary_base.csv ausente"
        if not pretty.exists():
            return True, "summary_base_pretty.md ausente"
        try:
            summary_mtime = summary.stat().st_mtime
        except Exception:
            return True, "no se pudo leer summary_base.csv"
        latest_metrics = 0.0
        for metrics in exp_dir.glob("*/metrics.csv"):
            try:
                latest_metrics = max(latest_metrics, metrics.stat().st_mtime)
            except Exception:
                continue
        if latest_metrics and latest_metrics > summary_mtime:
            return True, "metrics.csv mas reciente"
        return False, ""

    def _auto_summary_ab(self, reason: str) -> None:
        if not getattr(self, "chk_auto_summary", None) or not self.chk_auto_summary.isChecked():
            return
        if self._busy:
            return
        if self._proc and self._proc.state() != QProcess.NotRunning:
            return
        if self._auto_pipeline_running:
            return
        if self._proc_kind == "summary":
            return
        needs_refresh, detail = self._summary_needs_refresh()
        if not needs_refresh:
            return
        msg = f"[ML] Auto resumen A/B ({reason}): {detail}"
        self._append_log_line(msg)
        self._launch_summary_ab()

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
        env = self._build_env_prefix()
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
        env = self._build_env_prefix()
        cmd = f"cd '{self.ml_root}' && {env}PYTHONUNBUFFERED=1 ./scripts/run_one.sh '{rel_smoke}' {seed}"
        exp_dir = self._infer_exp_dir(Path(smoke_cfg), seed)
        self._active_total_epochs = 1
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
        env = self._build_env_prefix()
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
        env = self._build_env_prefix()
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
        env = self._build_env_prefix()
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

    def _build_env_prefix(self) -> str:
        root = shlex.quote(str(self.ml_root))
        root_src = shlex.quote(str(self.ml_root / "src"))
        env = f"export PYTHONPATH={root}:{root_src}:${{PYTHONPATH:-}} ; "
        env += self._determinism_env()
        return env

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
        if self._auto_pipeline_running:
            self._append_log_line("[ML-AUTO] Pipeline cancelado por el usuario.")
            self._auto_pipeline_running = False
            self._auto_pipeline_steps.clear()
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
        kind = self._proc_kind
        self._busy = False
        self._set_run_buttons_enabled(True)
        self.btn_stop_run.setEnabled(False)
        self._progress_timer.stop()
        self._set_state("finished" if rc == 0 else "failed")
        self._append_log_line(f"[{tag}] [EXIT] rc={rc}")
        if kind in ("summary", "export"):
            self._update_summary_panel(rc=rc)
        else:
            self._update_result_panel(rc=rc)
        if kind in ("train", "smoke", "multi"):
            self._run_single_assistant_check("E")
        if kind == "summary":
            self._run_single_assistant_check("F")
        if kind == "export":
            self._run_single_assistant_check("G")
        if kind in ("train", "smoke", "multi") and rc == 0:
            QTimer.singleShot(200, lambda: self._auto_summary_ab("post-run"))
        self._refresh_status()
        self._proc_kind = ""
        self._proc = None
        self._proc_pid = None
        if self._auto_pipeline_running:
            if rc != 0:
                self._append_log_line("[ML-AUTO] Pipeline detenido: hubo un error.")
                self._auto_pipeline_running = False
                self._auto_pipeline_steps.clear()
            else:
                QTimer.singleShot(100, self._auto_pipeline_next)

    def _append_log_line(self, text: str):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {text}"
        self.log_view.appendPlainText(line)
        self.log_view.moveCursor(QTextCursor.End)
        self._write_exp_log(text)
        if self._exp_debug_term:
            print(f"[EXP] {line}", flush=True)

    def _show_assistant_dialog(self):
        if self.assistant_dialog.isVisible():
            self.assistant_dialog.raise_()
            self.assistant_dialog.activateWindow()
        else:
            self.assistant_dialog.show()
            self.assistant_dialog.raise_()
            self.assistant_dialog.activateWindow()

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

    def _close_panel(self) -> None:
        self.window().close()

    def _open_summary_base(self):
        if not os.path.isfile(VISION_SUMMARY):
            self.log_fn(f"[EXP] ERROR: no existe {VISION_SUMMARY}")
            return
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_SUMMARY}' >/dev/null 2>&1 || true"])

    def _open_figures_memoria(self):
        ensure_dir(VISION_FIG_DIR)
        subprocess.Popen(["bash", "-lc", f"xdg-open '{VISION_FIG_DIR}' >/dev/null 2>&1 || true"])

    def start_auto_pipeline(self) -> bool:
        return self._start_auto_pipeline()

    def _start_auto_pipeline(self) -> bool:
        if self._auto_pipeline_running:
            self._append_log_line("[ML-AUTO] Ya hay un pipeline en curso.")
            return False
        if self._proc and self._proc.state() != QProcess.NotRunning:
            self._append_log_line("[ML-AUTO] Hay un proceso activo. Detenlo antes de lanzar el pipeline.")
            return False
        exp_names, missing = self._resolve_tfm_experiments()
        if missing:
            self._append_log_line("[ML-AUTO] ERROR: faltan experimentos TFM requeridos.")
            for msg in missing:
                self._append_log_line(f"[ML-AUTO] FALTA: {msg}")
            return False
        if not exp_names:
            self._append_log_line("[ML-AUTO] ERROR: no hay experimentos TFM disponibles.")
            return False
        self._auto_pipeline_steps = []
        for name in exp_names:
            self._auto_pipeline_steps.append((f"EXP {name}", lambda n=name: self._pipeline_run_multi(n)))
        self._auto_pipeline_steps.append(("Summary A/B", self._pipeline_run_summary))
        self._auto_pipeline_steps.append(("Export TFM", self._pipeline_run_export))
        self._auto_pipeline_running = True
        self._append_log_line(f"[ML-AUTO] Pipeline TFM iniciado ({len(exp_names)} experiments).")
        self._auto_pipeline_next()
        return True

    def _auto_pipeline_next(self) -> None:
        if not self._auto_pipeline_running:
            return
        if self._proc and self._proc.state() != QProcess.NotRunning:
            return
        while self._auto_pipeline_steps:
            label, step_fn = self._auto_pipeline_steps.pop(0)
            self._append_log_line(f"[ML-AUTO] Paso: {label}")
            started = False
            try:
                started = bool(step_fn())
            except Exception as exc:
                self._append_log_line(f"[ML-AUTO] ERROR en {label}: {exc}")
                started = False
            if started and self._proc is not None:
                return
            self._append_log_line(f"[ML-AUTO] Paso omitido: {label}")
        self._append_log_line("[ML-AUTO] Pipeline completado.")
        self._auto_pipeline_running = False

    def _pipeline_run_multi(self, name: str) -> bool:
        if not self._select_registry_experiment(name):
            self._append_log_line(f"[ML-AUTO] WARN: {name} sin config valida.")
            return False
        if not self._can_launch():
            self._append_log_line("[ML-AUTO] ERROR: precondiciones no OK. Cancelando pipeline.")
            self._auto_pipeline_running = False
            self._auto_pipeline_steps.clear()
            return False
        self._append_log_line(f"[ML-AUTO] Ejecutando {name} (multi-seed 0,1,2).")
        self._launch_multi_seed()
        return self._proc is not None

    def _pipeline_run_summary(self) -> bool:
        self._append_log_line("[ML-AUTO] Generando resumen A/B.")
        self._launch_summary_ab()
        return self._proc is not None

    def _pipeline_run_export(self) -> bool:
        self._append_log_line("[ML-AUTO] Exportando TFM.")
        self._export_tfm()
        return self._proc is not None

    def _resolve_tfm_experiments(self) -> Tuple[List[str], List[str]]:
        required = [
            ("EXP1_SIMPLE_RGB", "config/exp1_simple_rgb.yaml"),
            ("EXP2_SIMPLE_RGBD", "config/exp2_simple_rgbd.yaml"),
            ("EXP3_RESNET18_RGB_AUGMENT", "config/exp3_resnet18_rgb_augment.yaml"),
            ("EXP3_RESNET18_RGBD", "config/exp3_resnet18_rgbd.yaml"),
        ]
        exp_names: List[str] = []
        missing: List[str] = []
        configs = set(self._config_map.values())
        for name, rel in required:
            if name in self._exp_registry:
                exp_names.append(name)
                continue
            if rel in configs:
                mapped = self._registry_name_for_config(rel)
                if mapped:
                    exp_names.append(mapped)
                else:
                    missing.append(f"{name} (registry sin entrada para {rel})")
                continue
            missing.append(f"{name} (no existe {rel})")
        return exp_names, missing

    def _select_registry_experiment(self, name: str) -> bool:
        entry = self._exp_registry.get(name)
        if not entry:
            return False
        cfg_rel = entry.get("config", "")
        if cfg_rel:
            for cfg_name, cfg_path in self._config_map.items():
                if cfg_path == cfg_rel:
                    if self.config_combo.currentText() != cfg_name:
                        self.config_combo.setCurrentText(cfg_name)
                    break
        if self.exp_registry_combo.currentText() != name:
            self.exp_registry_combo.setCurrentText(name)
        return bool(self._current_config_rel())

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

    def _is_panel_exp_dir(self, exp_dir: Path) -> bool:
        try:
            exp_dir.resolve().relative_to(self._panel_tmp_dir.resolve())
            return True
        except Exception:
            return False

    def _update_result_panel(self, rc: Optional[int] = None):
        exp_dir = self._active_exp_dir
        if not exp_dir:
            self.lbl_result_status.setText("Estado: -")
            return
        exp_dir = exp_dir.resolve()
        is_panel = self._is_panel_exp_dir(exp_dir)
        exp_id = exp_dir.name
        summary_path = self.ml_root / "experiments" / "summary_base.csv"
        entry = None
        if not is_panel and summary_path.exists():
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
        loss_plot, success_plot = (None, None) if is_panel else self._find_plot_paths(exp_id)
        plots = []
        if loss_plot:
            plots.append(str(loss_plot))
        if success_plot:
            plots.append(str(success_plot))
        self.lbl_result_plots.setText(f"Plots: {', '.join(plots) if plots else '-'}")
        if is_panel:
            self.lbl_result_summary.setText("Resumen: -")
        else:
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
        if (pixmap is None or pixmap.isNull()) and exp_dir is None:
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
