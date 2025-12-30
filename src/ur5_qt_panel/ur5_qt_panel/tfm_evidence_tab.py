#!/usr/bin/env python3
"""Pestana TFM para evidencias reales (figuras y tablas)."""
import csv
import os
import shutil
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt, QProcess, QProcessEnvironment, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTextBrowser,
    QGroupBox,
    QSplitter,
    QSizePolicy,
    QScrollArea,
    QPlainTextEdit,
    QTableWidget,
    QTableWidgetItem,
    QFileDialog,
)

try:
    import yaml
except Exception:
    yaml = None

from panel_config import VISION_DIR, WS_DIR, FIG_DIR
from panel_utils import ensure_dir


@dataclass
class EvidenceEntry:
    evidence_id: str
    ev_type: str
    label: str
    number: str
    title: str
    explanation: str
    footnote: str
    inputs_required: List[str]
    artifact_path: str
    generator: List[str]
    checks: Dict[str, object]


class TfmEvidenceTab(QWidget):
    """Tab de Evidencias (TFM) con generacion real y visor."""

    go_experiments = pyqtSignal()
    request_mesa_snapshot = pyqtSignal()

    def __init__(self, log_fn, parent=None):
        super().__init__(parent)
        self.log_fn = log_fn
        self._compact_view = True
        self.data_root = Path(VISION_DIR).expanduser().resolve()
        self.ws_root = Path(WS_DIR).expanduser().resolve()
        self.registry_root = self._select_registry_root()
        self.root = self.registry_root
        self.alt_root = self.data_root if self.registry_root != self.data_root else self.ws_root
        self.registry_dir = self.registry_root / "reports" / "tfm_evidencias"
        self.registry_path = self.registry_dir / "tfm_evidences_registry.yaml"
        self.fig_dir = self.registry_dir / "figuras"
        self.table_dir = self.registry_dir / "tablas"
        self.logs_dir = self.registry_dir / "logs"
        self.manual_dir = self.registry_dir / "manual"
        self._entries: List[EvidenceEntry] = []
        self._entry_widgets: Dict[str, QPushButton] = {}
        self._entry_rows: Dict[str, QWidget] = {}
        self._current_entry: Optional[EvidenceEntry] = None
        self._proc: Optional[QProcess] = None
        self._gen_queue: List[EvidenceEntry] = []
        self._gen_cmds: List[str] = []
        self._log_path = self.logs_dir / "generate_all.log"
        self._debug_term = False
        self._last_exec_rc: Optional[int] = None
        self._last_exec_python: str = ""
        self._last_exec_cmd: str = ""
        self._last_exec_cwd: str = ""
        self._build_ui()
        self._ensure_registry()
        self._load_registry()
        self._refresh_list()
        self._refresh_states()

    def set_debug_enabled(self, enabled: bool) -> None:
        self._debug_term = bool(enabled)

    def _select_registry_root(self) -> Path:
        ws_candidate = self.ws_root / "reports" / "tfm_evidencias" / "tfm_evidences_registry.yaml"
        if ws_candidate.exists():
            return self.ws_root
        data_candidate = self.data_root / "reports" / "tfm_evidencias" / "tfm_evidences_registry.yaml"
        if data_candidate.exists():
            return self.data_root
        return self.data_root

    def _resolve_ml_repo_root(self) -> Path:
        return self.data_root

    def _resolve_tfm_python(self) -> Optional[str]:
        candidate = self._resolve_ml_repo_root() / ".venv" / "bin" / "python"
        if candidate.exists():
            return str(candidate)
        alt = shutil.which("python3")
        if alt:
            return alt
        return None

    def _close_panel(self) -> None:
        self.window().close()

    def _build_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        top_bar = QHBoxLayout()
        self.btn_debug = QPushButton("Debug logs -> terminal")
        self.btn_debug.setCheckable(True)
        self.btn_refresh = QPushButton("\U0001f501 Actualizar estados")
        self.btn_generate = QPushButton("\U0001f9ea Generar/actualizar TODO (solo datos reales)")
        self.btn_export = QPushButton("\U0001f4e6 Exportar pack TFM")
        self.btn_smoke = QPushButton("\U0001f9ea Smoke test Evidencias")
        self.btn_close_panel = QPushButton("Cerrar panel")
        self.btn_refresh.clicked.connect(self._refresh_states)
        self.btn_generate.clicked.connect(self._start_generate_all)
        self.btn_export.clicked.connect(self._export_pack)
        self.btn_smoke.clicked.connect(self._smoke_test)
        top_bar.addWidget(self.btn_debug)
        top_bar.addWidget(self.btn_refresh)
        top_bar.addWidget(self.btn_generate)
        top_bar.addWidget(self.btn_export)
        top_bar.addWidget(self.btn_smoke)
        top_bar.addStretch(1)
        top_bar.addWidget(self.btn_close_panel)
        self.btn_close_panel.clicked.connect(self._close_panel)
        layout.addLayout(top_bar)

        splitter = QSplitter(Qt.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        self.group_fig = QGroupBox("Ilustraciones")
        fig_layout = QVBoxLayout()
        fig_layout.setContentsMargins(4, 4, 4, 4)
        fig_layout.setSpacing(4)
        self.fig_list_container = QWidget()
        self.fig_list_layout = QVBoxLayout()
        self.fig_list_layout.setContentsMargins(0, 0, 0, 0)
        self.fig_list_layout.setSpacing(4)
        self.fig_list_container.setLayout(self.fig_list_layout)
        fig_layout.addWidget(self.fig_list_container)
        self.group_fig.setLayout(fig_layout)

        self.group_tab = QGroupBox("Tablas")
        tab_layout = QVBoxLayout()
        tab_layout.setContentsMargins(4, 4, 4, 4)
        tab_layout.setSpacing(4)
        self.tab_list_container = QWidget()
        self.tab_list_layout = QVBoxLayout()
        self.tab_list_layout.setContentsMargins(0, 0, 0, 0)
        self.tab_list_layout.setSpacing(4)
        self.tab_list_container.setLayout(self.tab_list_layout)
        tab_layout.addWidget(self.tab_list_container)
        self.group_tab.setLayout(tab_layout)

        left_layout.addWidget(self.group_fig)
        left_layout.addWidget(self.group_tab)
        left_layout.addStretch(1)
        left_widget.setLayout(left_layout)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_widget)
        left_scroll.setMinimumWidth(280)
        splitter.addWidget(left_scroll)

        right_widget = QWidget()
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)

        self.explain_box = QGroupBox("Explicación previa")
        explain_layout = QVBoxLayout()
        explain_layout.setContentsMargins(4, 4, 4, 4)
        self.explain_text = QTextBrowser()
        self.explain_text.setReadOnly(True)
        explain_layout.addWidget(self.explain_text)
        self.explain_box.setLayout(explain_layout)
        right_layout.addWidget(self.explain_box)

        self.viewer_label = QLabel("Selecciona una evidencia")
        self.viewer_label.setAlignment(Qt.AlignCenter)
        self.viewer_label.setStyleSheet("border:1px solid #cbd5f5; background:#f8fafc;")
        self.viewer_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.viewer_label.setMinimumHeight(240)
        right_layout.addWidget(self.viewer_label, 2)

        self.table_widget = QTableWidget()
        self.table_widget.setVisible(False)
        self.table_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout.addWidget(self.table_widget, 2)

        self.table_btns_widget = QWidget()
        table_btns = QHBoxLayout()
        table_btns.setContentsMargins(0, 0, 0, 0)
        self.btn_copy_md = QPushButton("Copiar como Markdown")
        self.btn_copy_csv = QPushButton("Copiar como CSV")
        self.btn_save_table = QPushButton("Guardar")
        self.btn_copy_md.clicked.connect(self._copy_markdown)
        self.btn_copy_csv.clicked.connect(self._copy_csv)
        self.btn_save_table.clicked.connect(self._save_table)
        self.btn_copy_md.setVisible(False)
        self.btn_copy_csv.setVisible(False)
        self.btn_save_table.setVisible(False)
        table_btns.addWidget(self.btn_copy_md)
        table_btns.addWidget(self.btn_copy_csv)
        table_btns.addWidget(self.btn_save_table)
        table_btns.addStretch(1)
        self.table_btns_widget.setLayout(table_btns)
        right_layout.addWidget(self.table_btns_widget)

        self.label_footer = QLabel("")
        self.label_footer.setStyleSheet("font-weight:bold;")
        right_layout.addWidget(self.label_footer)

        self.footnote_label = QLabel("")
        self.footnote_label.setStyleSheet("color:#6b7280;")
        self.footnote_label.setWordWrap(True)
        right_layout.addWidget(self.footnote_label)

        self.sources_box = QGroupBox("Fuentes reales")
        sources_layout = QVBoxLayout()
        sources_layout.setContentsMargins(4, 4, 4, 4)
        self.sources_text = QTextBrowser()
        self.sources_text.setReadOnly(True)
        sources_layout.addWidget(self.sources_text)
        self.sources_box.setLayout(sources_layout)
        right_layout.addWidget(self.sources_box)

        self.status_box = QGroupBox("Estado / Por que")
        status_layout = QVBoxLayout()
        status_layout.setContentsMargins(4, 4, 4, 4)
        self.status_banner = QLabel("")
        self.status_banner.setWordWrap(True)
        self.status_banner.setStyleSheet("color:#dc2626; font-weight:bold;")
        self.status_text = QTextBrowser()
        self.status_text.setReadOnly(True)
        status_layout.addWidget(self.status_banner)
        status_layout.addWidget(self.status_text)
        self.status_box.setLayout(status_layout)
        right_layout.addWidget(self.status_box)

        self.exec_box = QGroupBox("Ejecucion")
        exec_layout = QVBoxLayout()
        exec_layout.setContentsMargins(4, 4, 4, 4)
        self.exec_py = QLabel("Python: -")
        self.exec_cwd = QLabel("CWD: -")
        self.exec_cmd = QTextBrowser()
        self.exec_cmd.setReadOnly(True)
        self.exec_cmd.setPlaceholderText("Comando...")
        self.exec_rc = QLabel("RC: -")
        exec_layout.addWidget(self.exec_py)
        exec_layout.addWidget(self.exec_cwd)
        exec_layout.addWidget(self.exec_cmd)
        exec_layout.addWidget(self.exec_rc)
        self.exec_box.setLayout(exec_layout)
        right_layout.addWidget(self.exec_box)

        self.action_row_widget = QWidget()
        action_row = QHBoxLayout()
        action_row.setContentsMargins(0, 0, 0, 0)
        self.btn_go_exp = QPushButton("Ir a Experimentos")
        self.btn_gen_summary = QPushButton("Generar summary_base ahora")
        self.btn_go_exp.clicked.connect(lambda: self.go_experiments.emit())
        self.btn_gen_summary.clicked.connect(self._generate_summary_now)
        action_row.addWidget(self.btn_go_exp)
        action_row.addWidget(self.btn_gen_summary)
        action_row.addStretch(1)
        self.action_row_widget.setLayout(action_row)
        right_layout.addWidget(self.action_row_widget)

        self.manual_row_widget = QWidget()
        manual_row = QHBoxLayout()
        manual_row.setContentsMargins(0, 0, 0, 0)
        manual_row.setSpacing(6)
        self.btn_use_gazebo = QPushButton("Usar captura Mesa (a)")
        self.btn_upload_real = QPushButton("Subir foto real (c)")
        self.btn_gen_fig_1_2 = QPushButton("Generar 1.2 ahora")
        self.btn_upload_pipeline = QPushButton("Subir diagrama 1.3")
        self.btn_open_manual = QPushButton("Abrir carpeta manual")
        self.btn_use_gazebo.clicked.connect(self._use_latest_gazebo_capture)
        self.btn_upload_real.clicked.connect(self._upload_real_photo)
        self.btn_gen_fig_1_2.clicked.connect(self._generate_current_entry)
        self.btn_upload_pipeline.clicked.connect(self._upload_pipeline_diagram)
        self.btn_open_manual.clicked.connect(self._open_manual_dir)
        manual_row.addWidget(self.btn_use_gazebo)
        manual_row.addWidget(self.btn_upload_real)
        manual_row.addWidget(self.btn_gen_fig_1_2)
        manual_row.addWidget(self.btn_upload_pipeline)
        manual_row.addWidget(self.btn_open_manual)
        manual_row.addStretch(1)
        self.manual_row_widget.setLayout(manual_row)
        self.manual_row_widget.setVisible(False)
        right_layout.addWidget(self.manual_row_widget)

        right_widget.setLayout(right_layout)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter, 1)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(600)
        self.log_view.setPlaceholderText("Logs de generacion de evidencias.")
        self.log_view.setMaximumHeight(140)
        layout.addWidget(self.log_view)

        self.setLayout(layout)
        self._set_compact_view(self._compact_view)

    def _set_compact_view(self, enabled: bool) -> None:
        show_extras = not enabled
        for widget in (
            self.table_btns_widget,
            self.sources_box,
            self.status_box,
            self.exec_box,
            self.action_row_widget,
            self.manual_row_widget,
            self.log_view,
        ):
            widget.setVisible(show_extras)

    def _ensure_registry(self) -> None:
        self.registry_dir.mkdir(parents=True, exist_ok=True)
        self.fig_dir.mkdir(parents=True, exist_ok=True)
        self.table_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.manual_dir.mkdir(parents=True, exist_ok=True)
        if self.registry_path.exists():
            return
        payload = {
            "version": 1,
            "evidences": [
                {
                    "id": "fig_val_success",
                    "type": "figure",
                    "label": "Ilustracion",
                    "number": "4.1",
                    "title": "Comparativa val_success (media)",
                    "explanation": "Comparativa media de val_success por experimento. Se obtiene de experiments/summary_base.csv o metrics.csv reales.",
                    "footnote": "",
                    "inputs_required": [["experiments/summary_base.csv", "experiments/*/metrics.csv"]],
                    "artifact_path": "reports/tfm_evidencias/figuras/ilustracion_4_1_val_success.png",
                    "generator": [
                        "python scripts/tfm/gen_fig_val_success.py --root . --out reports/tfm_evidencias/figuras/ilustracion_4_1_val_success.png"
                    ],
                    "checks": {"type": "figure"},
                },
                {
                    "id": "fig_val_loss",
                    "type": "figure",
                    "label": "Ilustracion",
                    "number": "4.2",
                    "title": "Comparativa val_loss (media)",
                    "explanation": "Comparativa media de val_loss por experimento. Se obtiene de summary_base.csv o metrics reales.",
                    "footnote": "",
                    "inputs_required": [["experiments/summary_base.csv", "experiments/*/metrics.csv"]],
                    "artifact_path": "reports/tfm_evidencias/figuras/ilustracion_4_2_val_loss.png",
                    "generator": [
                        "python scripts/tfm/gen_fig_loss.py --root . --out reports/tfm_evidencias/figuras/ilustracion_4_2_val_loss.png"
                    ],
                    "checks": {"type": "figure"},
                },
                {
                    "id": "tabla_ab_resumen",
                    "type": "table",
                    "label": "Tabla",
                    "number": "4.1",
                    "title": "Resumen A/B (best epoch)",
                    "explanation": "Tabla resumen por experimento y seed. Se genera desde summary_base.csv o metrics reales.",
                    "footnote": "",
                    "inputs_required": [["experiments/summary_base.csv", "experiments/*/metrics.csv"]],
                    "artifact_path": "reports/tfm_evidencias/tablas/tabla_4_1_ab_resumen.csv",
                    "generator": [
                        "python scripts/tfm/gen_tabla_ab_resumen.py --root . --out_csv reports/tfm_evidencias/tablas/tabla_4_1_ab_resumen.csv --out_md reports/tfm_evidencias/tablas/tabla_4_1_ab_resumen.md"
                    ],
                    "checks": {"type": "table", "expected_columns": ["exp_id", "seed", "best_epoch", "val_success", "val_loss"]},
                },
                {
                    "id": "tabla_hiperparams",
                    "type": "table",
                    "label": "Tabla",
                    "number": "4.2",
                    "title": "Hiperparametros por experimento",
                    "explanation": "Tabla con hiperparametros leidos de config/*.yaml.",
                    "footnote": "",
                    "inputs_required": ["config/*.yaml"],
                    "artifact_path": "reports/tfm_evidencias/tablas/tabla_4_2_hiperparams.csv",
                    "generator": [
                        "python scripts/tfm/gen_tabla_hiperparams.py --root . --out_csv reports/tfm_evidencias/tablas/tabla_4_2_hiperparams.csv --out_md reports/tfm_evidencias/tablas/tabla_4_2_hiperparams.md"
                    ],
                    "checks": {"type": "table", "expected_columns": ["exp", "model", "lr", "batch_size", "epochs", "use_depth"]},
                },
            ],
        }
        if yaml is None:
            self.registry_path.write_text("", encoding="utf-8")
            return
        self.registry_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    def _load_registry(self) -> None:
        self._entries = []
        if not self.registry_path.exists():
            return
        if yaml is None:
            return
        try:
            payload = yaml.safe_load(self.registry_path.read_text(encoding="utf-8")) or {}
        except Exception:
            return
        for entry in payload.get("evidences", []):
            if not isinstance(entry, dict):
                continue
            generator = self._normalize_generator(entry.get("generator"))
            self._entries.append(
                EvidenceEntry(
                    evidence_id=str(entry.get("id", "")),
                    ev_type=str(entry.get("type", "")),
                    label=str(entry.get("label", "")),
                    number=str(entry.get("number", "")),
                    title=str(entry.get("title", "")),
                    explanation=str(entry.get("explanation", "")),
                    footnote=str(entry.get("footnote", "")) if entry.get("footnote") else "",
                    inputs_required=list(entry.get("inputs_required", []) or []),
                    artifact_path=str(entry.get("artifact_path", "")),
                    generator=generator,
                    checks=entry.get("checks", {}) or {},
                )
            )

    def _normalize_generator(self, generator_field) -> List[str]:
        if not generator_field:
            return []
        if isinstance(generator_field, list):
            return [str(cmd) for cmd in generator_field if cmd]
        if isinstance(generator_field, dict):
            cmd = generator_field.get("cmd")
            if cmd:
                return [str(cmd)]
            return []
        return [str(generator_field)]

    def _generator_available(self, entry: EvidenceEntry) -> Tuple[bool, str]:
        if not entry.generator:
            return False, "sin generador"
        cwd = Path(self._resolve_ml_repo_root())
        artifact_path = str((self.root / entry.artifact_path).resolve()) if entry.artifact_path else ""
        for raw_cmd in entry.generator:
            fmt_cmd = self._format_cmd(raw_cmd, artifact_path)
            argv = self._prepare_command(fmt_cmd)
            if not argv:
                return False, "comando invalido o python no disponible"
            if len(argv) > 1 and argv[1].endswith(".py"):
                script_path = Path(argv[1])
                if not script_path.is_absolute():
                    script_path = cwd / script_path
                if not script_path.exists():
                    return False, f"falta script: {script_path}"
        return True, ""

    def _refresh_list(self) -> None:
        for layout in (self.fig_list_layout, self.tab_list_layout):
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget:
                    widget.deleteLater()
        self._entry_widgets.clear()
        self._entry_rows.clear()
        for entry in self._entries:
            row = QWidget()
            row_lay = QHBoxLayout()
            row_lay.setContentsMargins(0, 0, 0, 0)
            row_lay.setSpacing(6)
            btn = QPushButton()
            btn.setCheckable(True)
            btn.clicked.connect(lambda _checked, e=entry: self._select_entry(e))
            btn.setStyleSheet("text-align:left; padding:6px;")
            row_lay.addWidget(btn, 1)
            if entry.ev_type == "figure":
                auto_btn = QPushButton("Auto")
                auto_btn.clicked.connect(lambda _checked=False, e=entry: self._auto_entry(e))
                row_lay.addWidget(auto_btn, 0)
            row.setLayout(row_lay)
            self._entry_widgets[entry.evidence_id] = btn
            self._entry_rows[entry.evidence_id] = row
            if entry.ev_type == "figure":
                self.fig_list_layout.addWidget(row)
            else:
                self.tab_list_layout.addWidget(row)
        self.fig_list_layout.addStretch(1)
        self.tab_list_layout.addStretch(1)

    def _refresh_states(self) -> None:
        for entry in self._entries:
            status, title = self._status_for_entry(entry)
            btn = self._entry_widgets.get(entry.evidence_id)
            if not btn:
                continue
            btn.setText(f"{status} {entry.label} {entry.number} — {entry.title}")
        if self._current_entry:
            self._select_entry(self._current_entry)

    def _status_for_entry(self, entry: EvidenceEntry) -> Tuple[str, str]:
        inputs, missing = self._resolve_inputs(entry)
        artifact_path = self._get_artifact_path(entry)
        artifact_ok = artifact_path is not None
        if missing:
            return "\U0001f534", "NO DISPONIBLE"
        if not artifact_ok:
            return "\U0001f7e0", "PENDIENTE"
        if artifact_path and self._artifact_outdated(artifact_path, inputs):
            return "\U0001f7e0", "ACTUALIZAR"
        if entry.ev_type == "table":
            expected = entry.checks.get("expected_columns") if entry.checks else None
            if expected and artifact_path and not self._table_has_columns(artifact_path, expected):
                return "\U0001f7e0", "INCOMPLETA"
        return "\U0001f7e2", "OK"

    def _resolve_inputs(self, entry: EvidenceEntry) -> Tuple[List[Path], List[str]]:
        inputs: List[Path] = []
        missing: List[str] = []
        items = entry.inputs_required
        if items and not entry.generator and all(isinstance(item, str) for item in items):
            items = [items]
        for item in items:
            if isinstance(item, list):
                group_matches: List[Path] = []
                group_patterns: List[str] = []
                for pattern in item:
                    group_patterns.append(pattern)
                    matches = self._glob_pattern(pattern)
                    if matches:
                        group_matches.extend(matches)
                if not group_matches:
                    missing.append(" | ".join(group_patterns))
                else:
                    inputs.extend(group_matches)
                continue
            pattern = str(item)
            matches = self._glob_pattern(pattern)
            if not matches:
                missing.append(pattern)
            else:
                inputs.extend(matches)
        return inputs, missing

    def _glob_pattern(self, pattern: str) -> List[Path]:
        if not pattern:
            return []
        if os.path.isabs(pattern):
            base = Path(pattern).parent
            name = Path(pattern).name
            return list(base.glob(name))
        matches = list(self.root.glob(pattern))
        if self.alt_root:
            matches.extend(self.alt_root.glob(pattern))
        seen: Dict[str, Path] = {}
        for match in matches:
            seen[str(match)] = match
        return list(seen.values())

    def _artifact_candidates(self, entry: EvidenceEntry) -> List[Path]:
        rel = Path(entry.artifact_path)
        candidates = []
        if self.root:
            candidates.append(self.root / rel)
        if self.alt_root and self.alt_root != self.root:
            candidates.append(self.alt_root / rel)
        return candidates

    def _get_artifact_path(self, entry: EvidenceEntry, allow_missing: bool = False) -> Optional[Path]:
        for path in self._artifact_candidates(entry):
            if path.exists() and path.stat().st_size > 0:
                return path
        if allow_missing:
            candidates = self._artifact_candidates(entry)
            return candidates[0] if candidates else None
        return None

    def _artifact_outdated(self, artifact: Path, inputs: List[Path]) -> bool:
        try:
            art_time = artifact.stat().st_mtime
        except OSError:
            return True
        for inp in inputs:
            try:
                if inp.stat().st_mtime > art_time:
                    return True
            except OSError:
                continue
        return False

    def _table_has_columns(self, path: Path, expected: List[str]) -> bool:
        try:
            with path.open("r", encoding="utf-8") as handle:
                reader = csv.reader(handle)
                header = next(reader, [])
        except Exception:
            return False
        return all(col in header for col in expected)

    def _select_entry(self, entry: EvidenceEntry) -> None:
        self._current_entry = entry
        for btn in self._entry_widgets.values():
            btn.setChecked(False)
        if entry.evidence_id in self._entry_widgets:
            self._entry_widgets[entry.evidence_id].setChecked(True)
        self._update_manual_controls(entry)

        status_icon, status_text = self._status_for_entry(entry)
        inputs, missing = self._resolve_inputs(entry)
        artifact = self._get_artifact_path(entry, allow_missing=True)
        artifact_exists = bool(artifact and artifact.exists())

        self.explain_text.setPlainText(entry.explanation or "-")
        self.footnote_label.setText(entry.footnote or "")
        self.label_footer.setText(f"{entry.label} {entry.number} — {entry.title}")

        # Viewer
        if entry.ev_type == "figure":
            self.table_widget.setVisible(False)
            self.viewer_label.setVisible(True)
            self.btn_copy_md.setVisible(False)
            self.btn_copy_csv.setVisible(False)
            self.btn_save_table.setVisible(False)
            if artifact_exists:
                pixmap = QPixmap(str(artifact))
                if not pixmap.isNull():
                    self.viewer_label.setPixmap(pixmap.scaled(self.viewer_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
                    self.viewer_label.setText("")
                else:
                    self.viewer_label.setText("NO DISPONIBLE")
            else:
                self.viewer_label.setText("NO DISPONIBLE")
        else:
            self.viewer_label.setVisible(False)
            self.table_widget.setVisible(True)
            self.btn_copy_md.setVisible(True)
            self.btn_copy_csv.setVisible(True)
            self.btn_save_table.setVisible(True)
            self._load_table(artifact if artifact_exists else None)

        # Sources
        source_lines = []
        for inp in inputs:
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(inp.stat().st_mtime)) if inp.exists() else "-"
            source_lines.append(f"{inp} | {ts}")
        if artifact_exists:
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(artifact.stat().st_mtime))
            source_lines.append(f"Artefacto: {artifact} | {ts}")
        elif artifact:
            source_lines.append(f"Artefacto esperado: {artifact}")
        for miss in missing:
            source_lines.append(f"FALTA: {miss}")
        self.sources_text.setPlainText("\n".join(source_lines) if source_lines else "-")

        # Estado
        if status_icon == "\U0001f534":
            self.status_banner.setStyleSheet("color:#dc2626; font-weight:bold;")
            self.status_banner.setText("NO DISPONIBLE: no se puede obtener del trabajo realizado")
            reason = "Faltan inputs requeridos:\n" + "\n".join(missing)
        elif status_icon == "\U0001f7e0":
            self.status_banner.setStyleSheet("color:#f59e0b; font-weight:bold;")
            self.status_banner.setText("PARCIAL / ACTUALIZAR")
            reason = "Artefacto ausente o desactualizado. Ejecuta Generar/actualizar TODO."
        else:
            self.status_banner.setText("OK")
            self.status_banner.setStyleSheet("color:#16a34a; font-weight:bold;")
            reason = "Evidencia disponible y validada."
        self.status_text.setPlainText(reason)

    def _update_manual_controls(self, entry: Optional[EvidenceEntry]) -> None:
        if not self.manual_row_widget:
            return
        if self._compact_view:
            self.manual_row_widget.setVisible(False)
            return
        if not entry:
            self.manual_row_widget.setVisible(False)
            return
        self.btn_use_gazebo.setVisible(entry.evidence_id == "fig_1_1")
        self.btn_upload_real.setVisible(entry.evidence_id == "fig_1_1")
        self.btn_gen_fig_1_2.setVisible(entry.evidence_id == "fig_1_2")
        self.btn_upload_pipeline.setVisible(entry.evidence_id == "fig_1_3")
        self.btn_open_manual.setVisible(entry.evidence_id in ("fig_1_1", "fig_1_3"))
        self.manual_row_widget.setVisible(entry.evidence_id in ("fig_1_1", "fig_1_2", "fig_1_3"))

    def _auto_entry(self, entry: EvidenceEntry) -> None:
        self._select_entry(entry)
        if entry.evidence_id == "fig_1_1":
            if not self._use_latest_gazebo_capture():
                self._append_log("[TFM] Auto fig_1_1: solicitando captura Mesa (a).")
                self.request_mesa_snapshot.emit()
                QTimer.singleShot(800, self._continue_auto_fig1_1)
                return
            return self._finish_auto_fig1_1(entry)
        if entry.evidence_id == "fig_1_3":
            ok = self._upload_pipeline_diagram()
            if not ok:
                self._append_log("[TFM] Auto fig_1_3: cancelado (sin diagrama).")
            return
        self._generate_entry(entry)

    def _continue_auto_fig1_1(self) -> None:
        entry = self._current_entry
        if not entry or entry.evidence_id != "fig_1_1":
            return
        if not self._use_latest_gazebo_capture():
            self._append_log("[TFM] Auto fig_1_1: sigue sin captura Mesa (a).")
            return
        self._finish_auto_fig1_1(entry)

    def _finish_auto_fig1_1(self, entry: EvidenceEntry) -> None:
        ok_c = self._upload_real_photo()
        if not ok_c:
            self._append_log("[TFM] Auto fig_1_1: cancelado (sin foto real).")
            return
        self._generate_entry(entry)

    def _manual_path(self, prefix: str, ext: str) -> Path:
        return self.manual_dir / f"{prefix}{ext}"

    def _cleanup_manual_prefix(self, prefix: str) -> None:
        if not self.manual_dir.exists():
            return
        for item in self.manual_dir.glob(f"{prefix}.*"):
            try:
                item.unlink()
            except OSError:
                continue

    def _use_latest_gazebo_capture(self) -> bool:
        self.manual_dir.mkdir(parents=True, exist_ok=True)
        fig_root = Path(FIG_DIR).expanduser()
        patterns = ["*Mesa*.png", "*Mesa*.jpg", "*Mesa*.jpeg", "*camera_overhead*.png", "*camera_overhead*.jpg"]
        candidates: List[Path] = []
        for pattern in patterns:
            candidates.extend(fig_root.glob(pattern))
        if not candidates:
            self._append_log("[TFM] No hay capturas Mesa en experiments/figures_memoria.")
            return False
        latest = max(candidates, key=lambda p: p.stat().st_mtime)
        ext = latest.suffix.lower()
        self._cleanup_manual_prefix("ilustracion_1_1_a_gazebo")
        dst = self._manual_path("ilustracion_1_1_a_gazebo", ext)
        shutil.copy2(str(latest), str(dst))
        self._append_log(f"[TFM] Captura Mesa -> {dst}")
        self._refresh_states()
        if self._current_entry:
            self._select_entry(self._current_entry)
        return True

    def _upload_real_photo(self) -> bool:
        self.manual_dir.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Selecciona foto real (clutter)",
            str(self.manual_dir),
            "Imagenes (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)",
        )
        if not path:
            return False
        src = Path(path)
        if not src.exists():
            self._append_log(f"[TFM] Foto real no existe: {src}")
            return False
        ext = src.suffix.lower() or ".png"
        self._cleanup_manual_prefix("ilustracion_1_1_c_real")
        dst = self._manual_path("ilustracion_1_1_c_real", ext)
        shutil.copy2(str(src), str(dst))
        self._append_log(f"[TFM] Foto real -> {dst}")
        self._refresh_states()
        if self._current_entry:
            self._select_entry(self._current_entry)
        return True

    def _upload_pipeline_diagram(self) -> bool:
        self.manual_dir.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Selecciona diagrama pipeline (1.3)",
            str(self.manual_dir),
            "Imagenes (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)",
        )
        if not path:
            return False
        src = Path(path)
        if not src.exists():
            self._append_log(f"[TFM] Diagrama no existe: {src}")
            return False
        ext = src.suffix.lower() or ".png"
        self._cleanup_manual_prefix("ilustracion_1_3_pipeline")
        manual_dst = self._manual_path("ilustracion_1_3_pipeline", ext)
        shutil.copy2(str(src), str(manual_dst))
        artifact_dst = self._get_artifact_path(
            EvidenceEntry(
                evidence_id="fig_1_3",
                ev_type="figure",
                label="Ilustración",
                number="1.3",
                title="",
                explanation="",
                footnote="",
                inputs_required=[],
                artifact_path="reports/tfm_evidencias/figuras/ilustracion_1_3_pipeline.png",
                generator=[],
                checks={},
            ),
            allow_missing=True,
        )
        if artifact_dst:
            artifact_dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(src), str(artifact_dst))
        self._append_log(f"[TFM] Diagrama 1.3 -> {manual_dst}")
        self._refresh_states()
        if self._current_entry:
            self._select_entry(self._current_entry)
        return True

    def _generate_current_entry(self) -> None:
        entry = self._current_entry
        if entry is None:
            return
        self._generate_entry(entry)

    def _generate_entry(self, entry: EvidenceEntry) -> None:
        if self._proc and self._proc.state() != QProcess.NotRunning:
            self._append_log("[TFM] Generacion en curso.")
            return
        inputs, missing = self._resolve_inputs(entry)
        if missing:
            self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} (faltan inputs)")
            return
        ok_gen, gen_reason = self._generator_available(entry)
        if not ok_gen:
            reason = gen_reason or "sin generador"
            self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} ({reason})")
            return
        self._gen_queue = [entry]
        self._run_next_entry()

    def _open_manual_dir(self) -> None:
        self.manual_dir.mkdir(parents=True, exist_ok=True)
        os.system(f"xdg-open '{self.manual_dir}' >/dev/null 2>&1 || true")

        # Acciones contextuales
        needs_summary = any("summary_base.csv" in m for m in missing)
        needs_exp = any("experiments/" in m for m in missing)
        self.btn_go_exp.setVisible(needs_exp)
        self.btn_gen_summary.setVisible(needs_summary)

    def _load_table(self, path: Optional[Path]) -> None:
        self.table_widget.clear()
        if not path or not path.exists() or path.is_dir():
            self.table_widget.setRowCount(0)
            self.table_widget.setColumnCount(0)
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                reader = csv.reader(handle)
                rows = list(reader)
        except Exception:
            self.table_widget.setRowCount(0)
            self.table_widget.setColumnCount(0)
            return
        if not rows:
            self.table_widget.setRowCount(0)
            self.table_widget.setColumnCount(0)
            return
        header = rows[0]
        data = rows[1:]
        self.table_widget.setColumnCount(len(header))
        self.table_widget.setRowCount(len(data))
        self.table_widget.setHorizontalHeaderLabels(header)
        for r_idx, row in enumerate(data):
            for c_idx, value in enumerate(row):
                self.table_widget.setItem(r_idx, c_idx, QTableWidgetItem(value))

    def _copy_markdown(self) -> None:
        if not self._current_entry:
            return
        artifact = self._get_artifact_path(self._current_entry, allow_missing=False)
        if artifact is None:
            return
        md_path = artifact.with_suffix(".md")
        if md_path.exists():
            text = md_path.read_text(encoding="utf-8")
        else:
            text = self._table_to_markdown()
        if text:
            QApplication.clipboard().setText(text)

    def _copy_csv(self) -> None:
        if not self._current_entry:
            return
        artifact = self._get_artifact_path(self._current_entry, allow_missing=False)
        if artifact and artifact.exists():
            QApplication.clipboard().setText(artifact.read_text(encoding="utf-8"))

    def _save_table(self) -> None:
        if not self._current_entry:
            return
        artifact = self._get_artifact_path(self._current_entry, allow_missing=False)
        if not artifact or not artifact.exists():
            return
        out_path, _ = QFileDialog.getSaveFileName(self, "Guardar tabla", str(artifact), "CSV (*.csv)")
        if not out_path:
            return
        shutil.copyfile(str(artifact), out_path)

    def _table_to_markdown(self) -> str:
        rows = []
        for r in range(self.table_widget.rowCount()):
            row = []
            for c in range(self.table_widget.columnCount()):
                item = self.table_widget.item(r, c)
                row.append(item.text() if item else "")
            rows.append(row)
        header = [self.table_widget.horizontalHeaderItem(c).text() for c in range(self.table_widget.columnCount())]
        if not header:
            return ""
        md = ["| " + " | ".join(header) + " |", "|" + "|".join([" --- "] * len(header)) + "|"]
        for row in rows:
            md.append("| " + " | ".join(row) + " |")
        return "\n".join(md)

    def _append_log(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {text}"
        self.log_view.appendPlainText(line)
        if self._debug_term:
            print(f"[TFM] {line}", flush=True)
        try:
            ensure_dir(self.logs_dir)
            with self._log_path.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")
        except Exception:
            pass

    def _update_exec_info(self, py_path: str, cwd: str, cmd: str, rc: Optional[int] = None) -> None:
        self._last_exec_python = py_path
        self._last_exec_cwd = cwd
        self._last_exec_cmd = cmd
        if rc is not None:
            self._last_exec_rc = rc
        self.exec_py.setText(f"Python: {py_path or '-'}")
        self.exec_cwd.setText(f"CWD: {cwd or '-'}")
        self.exec_cmd.setPlainText(cmd or "-")
        self.exec_rc.setText(f"RC: {self._last_exec_rc if self._last_exec_rc is not None else '-'}")

    def _process_env(self) -> QProcessEnvironment:
        env = QProcessEnvironment.systemEnvironment()
        root = str(self._resolve_ml_repo_root())
        src = str(self._resolve_ml_repo_root() / "src")
        current = env.value("PYTHONPATH", "")
        parts = [root, src]
        if current:
            parts.append(current)
        env.insert("PYTHONPATH", ":".join(parts))
        return env

    def _prepare_command(self, cmd: str) -> Optional[List[str]]:
        argv = shlex.split(cmd)
        if not argv:
            return None
        py = self._resolve_tfm_python()
        if not py:
            return None
        if argv[0] in ("python", "python3") or argv[0].endswith("/python"):
            argv[0] = py
        return argv

    def _start_generate_all(self) -> None:
        if self._proc and self._proc.state() != QProcess.NotRunning:
            self._append_log("[TFM] Generacion en curso.")
            return
        self._gen_queue = []
        py = self._resolve_tfm_python()
        if not py:
            self._append_log("[TFM] ERROR: no se encontro python3 ni .venv/bin/python.")
            self._update_exec_info("", str(self._resolve_ml_repo_root()), "", rc=1)
            return
        self._append_log(f"[TFM] Python usado: {py}")
        for entry in self._entries:
            inputs, missing = self._resolve_inputs(entry)
            if missing:
                self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} (faltan inputs)")
                continue
            ok_gen, gen_reason = self._generator_available(entry)
            if not ok_gen:
                reason = gen_reason or "sin generador"
                self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} ({reason})")
                continue
            self._gen_queue.append(entry)
        self._append_log("[TFM] Generar/actualizar TODO")
        self._run_next_entry()

    def _smoke_test(self) -> None:
        if self._proc and self._proc.state() != QProcess.NotRunning:
            self._append_log("[TFM] Generacion en curso.")
            return
        py = self._resolve_tfm_python()
        if not py:
            self._append_log("[TFM] ERROR: no se encontro python3 ni .venv/bin/python.")
            self._update_exec_info("", str(self._resolve_ml_repo_root()), "", rc=1)
            return
        candidate = None
        for entry in self._entries:
            if "summary_base.csv" in " ".join(entry.inputs_required):
                candidate = entry
                break
        if candidate is None and self._entries:
            candidate = self._entries[0]
        if candidate is None:
            self._append_log("[TFM] Smoke: no hay evidencias registradas.")
            return
        inputs, missing = self._resolve_inputs(candidate)
        if missing:
            self._append_log(f"[TFM] NO DISPONIBLE: {candidate.evidence_id} (faltan inputs)")
            self._update_exec_info(py, str(self._resolve_ml_repo_root()), "", rc=1)
            return
        if not candidate.generator:
            self._append_log(f"[TFM] NO DISPONIBLE: {candidate.evidence_id} (sin generador)")
            self._update_exec_info(py, str(self._resolve_ml_repo_root()), "", rc=1)
            return
        self._append_log(f"[TFM] Smoke: generando {candidate.evidence_id}")
        self._gen_queue = [candidate]
        self._run_next_entry()

    def _run_next_entry(self) -> None:
        if not self._gen_queue:
            self._append_log("[TFM] Generacion terminada")
            self._refresh_states()
            return
        entry = self._gen_queue.pop(0)
        self._current_entry = entry
        inputs, missing = self._resolve_inputs(entry)
        if missing:
            self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} (faltan inputs)")
            self._refresh_states()
            self._run_next_entry()
            return
        ok_gen, gen_reason = self._generator_available(entry)
        if not ok_gen:
            reason = gen_reason or "sin generador"
            self._append_log(f"[TFM] NO DISPONIBLE: {entry.evidence_id} ({reason})")
            self._refresh_states()
            self._run_next_entry()
            return
        self._gen_cmds = list(entry.generator)
        self._append_log(f"[TFM] Generando {entry.evidence_id}")
        self._run_next_cmd()

    def _run_next_cmd(self) -> None:
        if not self._gen_cmds:
            self._refresh_states()
            self._run_next_entry()
            return
        cmd = self._gen_cmds.pop(0)
        artifact_path = ""
        if self._current_entry:
            artifact_path = str((self.root / self._current_entry.artifact_path).resolve())
        fmt_cmd = self._format_cmd(cmd, artifact_path)
        argv = self._prepare_command(fmt_cmd)
        py = self._resolve_tfm_python() or ""
        cwd = str(self._resolve_ml_repo_root())
        if not argv:
            self._append_log("[TFM] ERROR: comando invalido o python no disponible.")
            self._update_exec_info(py, cwd, fmt_cmd, rc=1)
            self._run_next_entry()
            return
        self._append_log(f"[TFM] Python: {py}")
        self._append_log(f"[TFM] CWD: {cwd}")
        self._append_log(f"[TFM] $ {cwd} :: {' '.join(argv)}")
        self._update_exec_info(py, cwd, " ".join(argv))
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.setWorkingDirectory(cwd)
        proc.setProcessEnvironment(self._process_env())
        proc.readyReadStandardOutput.connect(lambda: self._proc_output(proc))
        proc.finished.connect(lambda rc, _status=None: self._proc_done(rc))
        proc.start(argv[0], argv[1:])
        self._proc = proc

    def _proc_output(self, proc: QProcess) -> None:
        raw = bytes(proc.readAllStandardOutput()).decode("utf-8", errors="ignore")
        for line in raw.splitlines():
            self._append_log(line)

    def _proc_done(self, rc: int) -> None:
        self._append_log(f"[TFM] [EXIT] rc={rc}")
        self._update_exec_info(self._last_exec_python, self._last_exec_cwd, self._last_exec_cmd, rc=rc)
        self._proc = None
        self._run_next_cmd()

    def _format_cmd(self, cmd: str, artifact_path: str) -> str:
        if "{artifact_path}" in cmd or "{artifact_dir}" in cmd or "{root}" in cmd:
            artifact_dir = str(Path(artifact_path).parent) if artifact_path else ""
            return cmd.format(
                artifact_path=artifact_path,
                artifact_dir=artifact_dir,
                root=str(self.data_root),
                registry_root=str(self.registry_root),
            )
        return cmd

    def _export_pack(self) -> None:
        export_dir = self.root / "reports" / "tfm_export"
        ensure_dir(export_dir)
        ok_entries = []
        missing_entries = []
        for entry in self._entries:
            status, _ = self._status_for_entry(entry)
            if status == "\U0001f7e2":
                ok_entries.append(entry)
            else:
                missing_entries.append(entry)
        for entry in ok_entries:
            artifact = self._get_artifact_path(entry, allow_missing=False)
            if artifact and artifact.exists():
                shutil.copyfile(str(artifact), str(export_dir / artifact.name))
        md_path = export_dir / "indice_evidencias.md"
        lines = ["# Indice de evidencias", "", "## Disponibles"]
        for entry in ok_entries:
            artifact = Path(entry.artifact_path).name
            lines.append(f"- {entry.label} {entry.number} — {entry.title} -> {artifact}")
        lines.append("")
        lines.append("## No disponibles")
        for entry in missing_entries:
            status, _ = self._status_for_entry(entry)
            lines.append(f"- {entry.label} {entry.number} — {entry.title} ({status})")
        md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self._append_log(f"[TFM] Exportado pack en {export_dir}")

    def _generate_summary_now(self) -> None:
        if self._proc and self._proc.state() != QProcess.NotRunning:
            return
        py = self._resolve_tfm_python()
        if not py:
            self._append_log("[TFM] ERROR: no se encontro python3 ni .venv/bin/python.")
            self._update_exec_info("", str(self._resolve_ml_repo_root()), "", rc=1)
            return
        cmd = [py, "scripts/analyze_experiments.py", "--root", ".", "--output", "experiments/summary_base.csv"]
        cwd = str(self._resolve_ml_repo_root())
        self._append_log("[TFM] Generando summary_base.csv")
        self._append_log(f"[TFM] $ {cwd} :: {' '.join(cmd)}")
        self._update_exec_info(py, cwd, " ".join(cmd))
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.setWorkingDirectory(cwd)
        proc.setProcessEnvironment(self._process_env())
        proc.readyReadStandardOutput.connect(lambda: self._proc_output(proc))
        proc.finished.connect(lambda rc, _status=None: self._after_summary(rc))
        proc.start(cmd[0], cmd[1:])
        self._proc = proc

    def _after_summary(self, rc: int) -> None:
        self._append_log(f"[TFM] summary_base finalizado rc={rc}")
        self._update_exec_info(self._last_exec_python, self._last_exec_cwd, self._last_exec_cmd, rc=rc)
        self._proc = None
        self._refresh_states()
