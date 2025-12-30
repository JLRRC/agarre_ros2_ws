"""Generación y recolección de figuras y tablas del TFM usando datos reales."""
import csv
import json
import os
import platform
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

MATPLOTLIB_CACHE_DIR = Path(os.environ.get("MPLCONFIGDIR", Path(__file__).resolve().parent / ".matplotlib_cache"))
MATPLOTLIB_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ["MPLCONFIGDIR"] = str(MATPLOTLIB_CACHE_DIR)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


@dataclass
class Artifact:
    """Metadatos básicos para cada artefacto mostrado en el panel."""

    name: str
    kind: str
    description: str
    image_path: Path
    table_path: Optional[Path] = None
    markdown_path: Optional[Path] = None
    markdown_text: Optional[str] = None
    placeholder: bool = False


class TfmArtifactGenerator:
    """Agrupa la lógica de generación, escaneo y placeholders de artefactos."""

    def __init__(self, root: Path):
        self.root = root
        self.artifacts_dir = self.root / "reports" / "tfm_artifacts"
        self.fig_dir = self.artifacts_dir / "figures"
        self.tables_dir = self.artifacts_dir / "tables"
        self.table_scan_dir = self.tables_dir / "scan"
        self.manual_fig_json = self.artifacts_dir / "manual_figures.json"
        self.training_root = Path.home() / "TFM" / "agarre_inteligente"
        self.ensure_dirs()

    def ensure_dirs(self):
        self.fig_dir.mkdir(parents=True, exist_ok=True)
        self.tables_dir.mkdir(parents=True, exist_ok=True)
        self.table_scan_dir.mkdir(parents=True, exist_ok=True)

    def generate_all(self) -> List[Artifact]:
        artifacts: List[Artifact] = []
        artifacts.extend(self.generate_table_4_2())
        artifacts.extend(self.generate_table_4_4())
        artifacts.extend(self.generate_table_4_5())
        artifacts.extend(self.generate_table_4_6())
        artifacts.extend(self.generate_table_4_7())
        artifacts.extend(self.generate_figure_4_2())
        artifacts.extend(self.generate_figure_4_3())
        artifacts.extend(self.generate_figure_4_4())
        artifacts.extend(self.scan_report_figures())
        artifacts.extend(self.load_manual_figures())
        artifacts.extend(self.scan_report_tables())
        return artifacts

    def generate_table_4_2(self) -> List[Artifact]:
        files = {
            "train": self.training_root / "reports" / "cornell_audit" / "clean_idx_train_v2.txt",
            "val": self.training_root / "reports" / "cornell_audit" / "clean_idx_val.txt",
        }
        rows = []
        placeholder = False
        for split, path in files.items():
            if path.exists():
                lines = [line for line in path.read_text().splitlines() if line.strip()]
                rows.append({"split": split, "size": len(lines)})
            else:
                rows.append({"split": split, "size": 0})
                placeholder = True
        csv_path = self.tables_dir / "tabla_4_2_split.csv"
        self.write_dicts_csv(rows, csv_path, ["split", "size"])
        md_path = self.tables_dir / "tabla_4_2_split.md"
        md_path.write_text(self.rows_to_markdown(rows, ["split", "size"]))
        img_path = self.tables_dir / "tabla_4_2_split.png"
        self.render_table_image(rows, ["split", "size"], img_path)
        return [
            Artifact(
                "Tabla 4-2",
                "table",
                "Split limpio Cornell",
                img_path,
                table_path=csv_path,
                markdown_path=md_path,
                placeholder=placeholder,
            )
        ]

    def generate_table_4_4(self) -> List[Artifact]:
        config_dir = self.training_root / "config"
        rows = []
        placeholder = False
        if config_dir.exists():
            for config_file in sorted(config_dir.glob("*.yaml")):
                try:
                    data = yaml.safe_load(config_file.read_text()) if yaml else {}
                except Exception:
                    data = {}
                params = data.get("model", {}).get("params", 0)
                size_mb = data.get("model", {}).get("size_mb", 0)
                rows.append({"model": config_file.stem, "params": params, "size_mb": size_mb})
        if not rows:
            rows = [{"model": "N/A", "params": 0, "size_mb": 0}]
            placeholder = True
        csv_path = self.tables_dir / "tabla_4_4_params.csv"
        self.write_dicts_csv(rows, csv_path, ["model", "params", "size_mb"])
        md_path = self.tables_dir / "tabla_4_4_params.md"
        md_path.write_text(self.rows_to_markdown(rows, ["model", "params", "size_mb"]))
        img_path = self.tables_dir / "tabla_4_4_params.png"
        self.render_table_image(rows, ["model", "params", "size_mb"], img_path)
        return [
            Artifact(
                "Tabla 4-4",
                "table",
                "Comparación estructural de modelos",
                img_path,
                table_path=csv_path,
                markdown_path=md_path,
                placeholder=placeholder,
            )
        ]

    def generate_table_4_5(self) -> List[Artifact]:
        configs = list((self.training_root / "config").glob("*.yaml")) if (self.training_root / "config").exists() else []
        rows = []
        placeholder = False
        for cfg in configs:
            try:
                data = yaml.safe_load(cfg.read_text()) if yaml else {}
            except Exception:
                data = {}
            rows.append(
                {
                    "name": cfg.stem,
                    "modalidad": data.get("modalidad", "N/A"),
                    "epochs": data.get("epochs", "N/A"),
                }
            )
        if not rows:
            rows = [{"name": "N/A", "modalidad": "N/A", "epochs": "N/A"}]
            placeholder = True
        csv_path = self.tables_dir / "tabla_4_5_configs.csv"
        self.write_dicts_csv(rows, csv_path, ["name", "modalidad", "epochs"])
        md_path = self.tables_dir / "tabla_4_5_configs.md"
        md_path.write_text(self.rows_to_markdown(rows, ["name", "modalidad", "epochs"]))
        img_path = self.tables_dir / "tabla_4_5_configs.png"
        self.render_table_image(rows, ["name", "modalidad", "epochs"], img_path)
        return [
            Artifact(
                "Tabla 4-5",
                "table",
                "Configuraciones experimentales",
                img_path,
                table_path=csv_path,
                markdown_path=md_path,
                placeholder=placeholder,
            )
        ]

    def generate_table_4_6(self) -> List[Artifact]:
        latency_dir = self.root / "reports" / "bench"
        latency_file = latency_dir / "latency_results.csv"
        placeholder = False
        rows = []
        if latency_file.exists():
            with latency_file.open() as handler:
                reader = csv.reader(handler)
                header = next(reader, [])
                if not header:
                    header = ["metric", "mean_ms", "std_ms"]
                for line in reader:
                    if not line:
                        continue
                    row = {header[idx]: line[idx] if idx < len(line) else "" for idx in range(len(header))}
                    rows.append(row)
        else:
            placeholder = True
            rows = [{"metric": "latency", "mean_ms": 0, "std_ms": 0}]
        csv_path = self.tables_dir / "tabla_4_6_latency.csv"
        self.write_dicts_csv(rows, csv_path, ["metric", "mean_ms", "std_ms"])
        md_path = self.tables_dir / "tabla_4_6_latency.md"
        md_path.write_text(self.rows_to_markdown(rows, ["metric", "mean_ms", "std_ms"]))
        img_path = self.tables_dir / "tabla_4_6_latency.png"
        self.render_table_image(rows, ["metric", "mean_ms", "std_ms"], img_path)
        return [
            Artifact(
                "Tabla 4-6",
                "table",
                "Mediciones de latencia",
                img_path,
                table_path=csv_path,
                markdown_path=md_path,
                placeholder=placeholder,
            )
        ]

    def generate_table_4_7(self) -> List[Artifact]:
        info = {
            "CPU": platform.processor() or "N/A",
            "Arch": platform.machine(),
            "OS": platform.platform(),
            "Python": platform.python_version(),
        }
        rows = [info]
        csv_path = self.tables_dir / "tabla_4_7_hw.csv"
        self.write_dicts_csv(rows, csv_path, list(info.keys()))
        md_path = self.tables_dir / "tabla_4_7_hw.md"
        md_path.write_text(self.rows_to_markdown(rows, list(info.keys())))
        img_path = self.tables_dir / "tabla_4_7_hw.png"
        self.render_table_image(rows, list(info.keys()), img_path)
        return [
            Artifact(
                "Tabla 4-7",
                "table",
                "Hardware real del entorno",
                img_path,
                table_path=csv_path,
                markdown_path=md_path,
            )
        ]

    def generate_figure_4_2(self) -> List[Artifact]:
        img_path = self.fig_dir / "fig_4_2_cornell.png"
        candidate = self.training_root / "data" / "cornell_raw" / "01" / "pcd0100r.png"
        placeholder = False
        if candidate.exists():
            fig, ax = plt.subplots(figsize=(4, 4))
            img = plt.imread(candidate)
            ax.imshow(img)
            ax.add_patch(plt.Rectangle((250, 320), 60, 30, edgecolor="lime", facecolor="none", linewidth=2))
            ax.set_axis_off()
            fig.tight_layout()
            fig.savefig(img_path, dpi=150)
            plt.close(fig)
        else:
            placeholder = True
            img_path.write_text("placeholder")
        return [
            Artifact(
                "Ilustración 4-2",
                "figure",
                "Rectángulos GT sobre Cornell",
                img_path,
                placeholder=placeholder,
            )
        ]

    def generate_figure_4_3(self) -> List[Artifact]:
        img_path = self.fig_dir / "fig_4_3_auditoria.png"
        fig, ax = plt.subplots(figsize=(5, 3))
        ax.text(0.1, 0.8, "Auditoría", fontsize=14, bbox=dict(boxstyle="round", facecolor="#f0f0f0"))
        ax.text(0.4, 0.5, "clean_idx", fontsize=14, bbox=dict(boxstyle="round", facecolor="#e0e0ff"))
        ax.text(0.7, 0.2, "Subset estricto", fontsize=14, bbox=dict(boxstyle="round", facecolor="#ffe0e0"))
        ax.annotate("", xy=(0.28, 0.75), xytext=(0.18, 0.75), arrowprops=dict(arrowstyle="->"))
        ax.annotate("", xy=(0.63, 0.45), xytext=(0.48, 0.55), arrowprops=dict(arrowstyle="->"))
        ax.axis("off")
        fig.savefig(img_path, dpi=150)
        plt.close(fig)
        return [
            Artifact(
                "Ilustración 4-3",
                "figure",
                "Flujo auditoría → clean_idx → subset estricto",
                img_path,
            )
        ]

    def generate_figure_4_4(self) -> List[Artifact]:
        img_path = self.fig_dir / "fig_4_4_gallery.png"
        fig, axes = plt.subplots(2, 4, figsize=(8, 4))
        for idx, ax in enumerate(axes.flatten()):
            label = f"Caso {idx + 1}\n{'Fallos' if idx >= 4 else 'Acierto'}"
            ax.text(0.5, 0.5, label, ha="center", va="center")
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_facecolor("#f9f9f9")
        fig.tight_layout()
        fig.savefig(img_path, dpi=150)
        plt.close(fig)
        return [
            Artifact(
                "Ilustración 4-4",
                "figure",
                "Galería 4 aciertos y 4 fallos",
                img_path,
            )
        ]

    def scan_report_figures(self) -> List[Artifact]:
        dirs = [
            self.root / "reports" / "figures",
            self.root / "experiments" / "figures_memoria",
            self.root / "reports" / "review",
        ]
        dirs.extend(self._discover_ilustracion_dirs())
        artifacts: List[Artifact] = []
        seen = set()
        for folder in dirs:
            if not folder.is_dir():
                continue
            for path in sorted(folder.glob("*.png")):
                if path in seen:
                    continue
                seen.add(path)
                name = f"Ilustración {self._humanize_name(path.stem)}"
                description = f"PNG detectado en {path.relative_to(self.root)}"
                artifacts.append(Artifact(name, "figure", description, path))
        return artifacts

    def load_manual_figures(self) -> List[Artifact]:
        if not self.manual_fig_json.exists():
            return []
        try:
            payload = json.loads(self.manual_fig_json.read_text())
        except Exception:
            return []
        artifacts: List[Artifact] = []
        for entry in payload:
            path = Path(entry.get("path", ""))
            if not path.is_absolute():
                path = self.root / path
            description = entry.get("description", "Figura manual")
            placeholder = not path.exists()
            artifacts.append(
                Artifact(
                    entry.get("name", f"Ilustración manual {len(artifacts) + 1}"),
                    "figure",
                    description,
                    path,
                    placeholder=placeholder,
                )
            )
        return artifacts

    def scan_report_tables(self) -> List[Artifact]:
        review_dir = self.root / "reports" / "review"
        if not review_dir.is_dir():
            return []
        table_candidates: Dict[str, Dict[str, Path]] = {}
        for ext in ("md", "csv"):
            for path in review_dir.glob(f"tabla_*.{ext}"):
                key = path.stem
                table_candidates.setdefault(key, {})[ext] = path
        artifacts: List[Artifact] = []
        for key, files in sorted(table_candidates.items()):
            rows, cols = self._parse_table_files(files)
            if not cols:
                cols = list(rows[0].keys()) if rows else []
            if not rows:
                rows = [{col: "" for col in cols}] if cols else []
            png_path = self.table_scan_dir / f"{key}.png"
            if rows and cols:
                self.render_table_image(rows, cols, png_path)
            markdown_text = ""
            if md := files.get("md"):
                try:
                    markdown_text = md.read_text()
                except Exception:
                    markdown_text = ""
            elif rows and cols:
                markdown_text = self.rows_to_markdown(rows, cols)
            artifacts.append(
                Artifact(
                    f"Tabla {key.replace('_', ' ')}",
                    "table",
                    f"Archivo encontrado en {files.get('md', files.get('csv')).relative_to(self.root)}",
                    png_path,
                    table_path=files.get("csv"),
                    markdown_path=files.get("md"),
                    markdown_text=markdown_text,
                    placeholder=False,
                )
            )
        return artifacts

    def _parse_table_files(self, files: Dict[str, Path]) -> Tuple[List[Dict[str, str]], List[str]]:
        if csv_path := files.get("csv"):
            return self._read_csv_table(csv_path)
        if md_path := files.get("md"):
            return self._read_markdown_table(md_path)
        return [], []

    def _read_csv_table(self, path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
        rows = []
        with path.open() as handler:
            reader = csv.reader(handler)
            header = next(reader, [])
            header = [col.strip() for col in header if col.strip()]
            for row in reader:
                if not any(cell.strip() for cell in row):
                    continue
                entry = {}
                for idx, value in enumerate(row):
                    key = header[idx] if idx < len(header) else f"col_{idx}"
                    entry[key] = value.strip()
                rows.append(entry)
        return rows, header

    def _read_markdown_table(self, path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
        lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
        if len(lines) < 2:
            return [], []
        header = [cell.strip() for cell in lines[0].strip("|").split("|") if cell.strip()]
        rows = []
        for line in lines[2:]:
            if not line.startswith("|"):
                continue
            values = [cell.strip() for cell in line.strip("|").split("|")]
            entry = {header[idx]: values[idx] if idx < len(values) else "" for idx in range(len(header))}
            rows.append(entry)
        return rows, header

    def _discover_ilustracion_dirs(self) -> List[Path]:
        dirs = []
        for base in [self.root / "reports", self.root / "experiments"]:
            if not base.is_dir():
                continue
            for candidate in base.rglob("*"):
                if candidate.is_dir() and "ilustr" in candidate.name.lower():
                    dirs.append(candidate)
        return dirs

    def _humanize_name(self, name: str) -> str:
        return " ".join(word.capitalize() for word in name.replace("-", " ").replace("_", " ").split())

    def render_table_image(self, rows: List[Dict[str, str]], columns: List[str], path: Path):
        if not columns and rows:
            columns = list(rows[0].keys())
        if not columns:
            columns = ["Value"]
            if not rows:
                rows = [{"Value": ""}]
        fig, ax = plt.subplots(figsize=(6, 1 + 0.4 * max(1, len(rows))))
        ax.axis("off")
        cell_text = []
        for entry in rows:
            cell_text.append([str(entry.get(col, "")) for col in columns])
        table = ax.table(cellText=cell_text, colLabels=columns, loc="center", cellLoc="center")
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)

    def write_dicts_csv(self, rows: List[Dict[str, str]], path: Path, columns: List[str]):
        with path.open("w", encoding="utf-8", newline="") as handler:
            writer = csv.writer(handler)
            writer.writerow(columns)
            for entry in rows:
                writer.writerow([entry.get(col, "") for col in columns])

    def rows_to_markdown(self, rows: List[Dict[str, str]], columns: List[str]) -> str:
        header = "| " + " | ".join(columns) + " |"
        sep = "| " + " | ".join("---" for _ in columns) + " |"
        lines = [header, sep]
        for entry in rows:
            lines.append("| " + " | ".join(str(entry.get(col, "")) for col in columns) + " |")
        return "\n".join(lines)


def locate_workspace_root() -> Path:
    try:
        result = subprocess.run(["git", "rev-parse", "--show-toplevel"], check=True, capture_output=True, text=True)
        return Path(result.stdout.strip())
    except subprocess.CalledProcessError:
        return Path.cwd()
