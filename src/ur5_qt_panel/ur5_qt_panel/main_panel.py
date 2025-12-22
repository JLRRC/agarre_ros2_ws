#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/main_panel.py
# Summary: Main Qt panel for UR5 simulation control and camera monitoring.
"""Qt control panel for UR5 simulation, bridge, cameras, and evidence capture."""
import os
import re
import sys
import time
import signal
import shutil
import threading
import subprocess
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple, Set

# Disable FastDDS SHM early to avoid noisy startup errors.
os.environ.setdefault("RMW_FASTRTPS_USE_SHM", "0")

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QImage, QPixmap, QGuiApplication
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QTextEdit, QGroupBox,
    QComboBox, QLineEdit, QFileDialog,
    QSizePolicy, QMessageBox, QScrollArea, QSplitter, QDialog
)

# =========================
# CONFIG SUPER PRO
# =========================
WS_DIR = os.path.expanduser(os.environ.get("WS_DIR", "~/TFM/agarre_ros2_ws"))
SCRIPTS_DIR = os.path.join(WS_DIR, "scripts")
WORLDS_DIR = os.path.join(WS_DIR, "worlds")
MODELS_DIR = os.path.join(WS_DIR, "models")
LOG_DIR = os.path.join(WS_DIR, "log")
BAGS_DIR = os.path.join(WS_DIR, "bags")
FIG_DIR = os.path.join(WS_DIR, "experiments", "figures_memoria")
FASTRTPS_PROFILES = os.path.join(SCRIPTS_DIR, "fastdds_no_shm.xml")
UR5_CONTROLLERS_YAML = os.path.join(WS_DIR, "src", "ur5_description", "config", "ur5_controllers.yaml")

BRIDGE_BASE_YAML = os.path.join(SCRIPTS_DIR, "bridge_cameras.yaml")
EGL_VENDOR = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"

DEFAULT_WORLD_CANDIDATES = [
    os.path.join(WORLDS_DIR, "ur5_mesa_objetos_pro.sdf"),
    os.path.join(WORLDS_DIR, "ur5_mesa_objetos.sdf"),
]

DEBUG_FRAME_LOG = bool(int(os.environ.get("PANEL_DEBUG_FRAMES", "0")))

# =========================
# ROS 2 imports
# =========================
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from cv_bridge import CvBridge
    import numpy as np
    import cv2
    ROS_AVAILABLE = True
except Exception as e:
    print(f"[WARN] ROS 2 / OpenCV no disponible en el panel: {e}", file=sys.stderr)

# =========================
# Utils
# =========================
def ensure_dir(p: str):
    """Create a directory if it does not exist."""
    os.makedirs(p, exist_ok=True)

def now_tag() -> str:
    """Return a timestamp string suitable for filenames."""
    return time.strftime("%Y%m%d_%H%M%S")

def safe_topic_name(t: str) -> str:
    """Sanitize a topic name for use in filenames."""
    return re.sub(r"[^a-zA-Z0-9_]+", "_", t.strip("/"))

def set_led(lbl: QLabel, state: str):
    """Update a small LED label with a status color."""
    # state: off,on,warn,error
    colors = {
        "off": "#6b7280",   # gray
        "on": "#22c55e",    # green
        "warn": "#f59e0b",  # amber
        "error": "#ef4444", # red
    }
    c = colors.get(state, colors["off"])
    lbl.setFixedSize(14, 14)
    lbl.setStyleSheet(f"background:{c}; border-radius:7px; border:1px solid #374151;")

def bash_preamble(ws_dir: str) -> str:
    """Build a shell preamble that sources ROS 2 and workspace overlays."""
    # Evita el error AMENT_TRACE_SETUP_FILES con "set -u"
    return (
        "set +u; "
        "export AMENT_TRACE_SETUP_FILES=\"${AMENT_TRACE_SETUP_FILES:-}\"; "
        f"export FASTRTPS_DEFAULT_PROFILES_FILE='{FASTRTPS_PROFILES}'; "
        "export RMW_FASTRTPS_USE_SHM=0; "
        "if [ -z \"${RMW_IMPLEMENTATION:-}\" ]; then "
        "  if [ -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/librmw_cyclonedds_cpp.so ] "
        "|| [ -f /usr/lib/x86_64-linux-gnu/librmw_cyclonedds_cpp.so ]; then "
        "    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
        "  fi; "
        "fi; "
        "source /opt/ros/jazzy/setup.bash; "
        f"if [ -f '{ws_dir}/install/setup.bash' ]; then source '{ws_dir}/install/setup.bash'; fi; "
        "set -u; "
        f"cd '{ws_dir}'; "
    )

def kill_process_group(proc: subprocess.Popen, name: str, log_fn):
    """Terminate a process group with a graceful SIGTERM and SIGKILL fallback."""
    if proc is None:
        return
    try:
        if proc.poll() is not None:
            return
        pgid = os.getpgid(proc.pid)
        log_fn(f"[STOP] Matando {name} (pgid={pgid}) ...")
        os.killpg(pgid, signal.SIGTERM)
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            log_fn(f"[STOP] {name} no murió con SIGTERM -> SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
    except Exception as e:
        log_fn(f"[WARN] kill_process_group({name}) falló: {e}")

def cold_boot_kill(log_fn):
    """Kill common ROS/Gazebo processes to ensure a clean launch state."""
    log_fn("[BOOT] Enforce COLD BOOT: matando procesos antes de mostrar el panel...")

    # 1) kill_all.sh si existe
    ka = os.path.join(SCRIPTS_DIR, "kill_all.sh")
    if os.path.isfile(ka) and os.access(ka, os.X_OK):
        try:
            subprocess.run(["bash", "-lc", f"'{ka}'"], check=False)
        except Exception:
            pass

    # 2) fallback pkill
    subprocess.run(["bash","-lc",
        "pkill -f 'ros2 bag record' || true; "
        "pkill -f 'ros_gz_bridge' || true; "
        "pkill -f 'parameter_bridge' || true; "
        "pkill -f 'gz sim' || true; "
        "pkill -f 'gzserver' || true; "
        "pkill -f 'gzclient' || true; "
        "pkill -f 'rqt_image_view' || true; "
    ], check=False)

    # 3) espera
    for _ in range(40):
        r = subprocess.run(
            [
                "bash",
                "-lc",
                "pgrep -f \"ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gzserver|gzclient\" "
                ">/dev/null 2>&1",
            ],
            check=False,
        )
        if r.returncode != 0:
            break
        time.sleep(0.15)

    log_fn("[OK] Panel listo. Nada se arranca automáticamente. Usa START ALL.")

def read_world_name(world_path: str) -> str:
    """Read the world name from an SDF file, returning a default if absent."""
    # Busca: <world name="...">
    try:
        with open(world_path, "r", encoding="utf-8", errors="ignore") as f:
            head = f.read(20000)
        m = re.search(r"<world\s+name\s*=\s*\"([^\"]+)\"", head)
        if m:
            return m.group(1).strip()
    except Exception:
        pass
    # fallback
    return "default"

def write_bridge_runtime_yaml(runtime_path: str, world_name: str, base_yaml_path: str):
    """Generate a runtime YAML for ros_gz_bridge with /clock and camera rules."""
    ensure_dir(os.path.dirname(runtime_path))

    # Cabecera + clock
    lines = []
    lines.append("# AUTO-GENERATED by Panel SUPER PRO\n")
    lines.append(f"# world_name = {world_name}\n")
    lines.append("\n")
    lines.append("- ros_topic_name: /clock\n")
    lines.append(f"  gz_topic_name: /world/{world_name}/clock\n")
    lines.append("  ros_type_name: rosgraph_msgs/msg/Clock\n")
    lines.append("  gz_type_name: gz.msgs.Clock\n")
    lines.append("  direction: GZ_TO_ROS\n\n")

    # Añadimos el base yaml (asumimos que ya es lista correcta)
    try:
        with open(base_yaml_path, "r", encoding="utf-8") as f:
            base = f.read().strip()
        if base:
            lines.append("\n")
            lines.append(base)
            lines.append("\n")
    except Exception as e:
        # Si falla, al menos dejamos clock (panel avisará)
        lines.append(f"\n# WARN: no pude leer base_yaml: {e}\n")

    with open(runtime_path, "w", encoding="utf-8") as f:
        f.write("".join(lines))

def gripper_controller_defined() -> bool:
    """Return True when the gripper controller is configured."""
    try:
        with open(UR5_CONTROLLERS_YAML, "r", encoding="utf-8", errors="ignore") as f:
            return "gripper_controller:" in f.read()
    except Exception:
        return False

def list_active_controllers() -> Tuple[Optional[Set[str]], Optional[str]]:
    """Return active controller names and any error string."""
    states, err = list_controllers_state()
    if err or states is None:
        return None, err
    active = {name for name, st in states.items() if st == "active"}
    return active, None

def list_controllers_state() -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    """Return controller states (name -> state) and any error string."""
    cmd = bash_preamble(WS_DIR) + "ros2 control list_controllers || true"
    try:
        out = subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)
    except Exception as e:
        return None, str(e)
    states: Dict[str, str] = {}
    for ln in out.splitlines():
        parts = ln.split()
        if len(parts) >= 3:
            name = parts[0]
            state = parts[-1]
            states[name] = state
    return states, None

def ros2_control_running() -> bool:
    """Check whether ros2_control is running by querying nodes."""
    cmd = bash_preamble(WS_DIR) + "ros2 service list || true"
    try:
        out = subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)
    except Exception:
        return False
    if "/controller_manager/list_controllers" in out:
        return True
    cmd_nodes = bash_preamble(WS_DIR) + "ros2 node list || true"
    try:
        nodes = subprocess.check_output(["bash", "-lc", cmd_nodes], text=True, stderr=subprocess.STDOUT)
    except Exception:
        return False
    return "/controller_manager" in nodes

def gz_sim_running() -> bool:
    """Check whether Gazebo sim is running."""
    try:
        r = subprocess.run(
            ["bash", "-lc", "pgrep -f \"gz sim\" >/dev/null 2>&1"],
            check=False,
        )
        return r.returncode == 0
    except Exception:
        return False

def ros_clock_available() -> bool:
    """Return True when /clock is visible in ROS (bridge running)."""
    cmd = bash_preamble(WS_DIR) + "ros2 topic list || true"
    try:
        out = subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)
    except Exception:
        return False
    return "/clock" in out

def robot_control_available() -> bool:
    """Return True when either ros2_control or Gazebo sim is available."""
    return ros2_control_running() or (gz_sim_running() and ros_clock_available())

# =========================
# Command runner (stream)
# =========================
class CmdRunner(QObject):
    """Run shell commands and stream output to the UI."""
    line = pyqtSignal(str)

    def run_stream(self, tag: str, cmd: str, proc_slot: Optional[dict]=None, key: Optional[str]=None):
        """
        Ejecuta cmd (bash -lc) y streamea salida al log.
        Si proc_slot+key se pasan, guarda Popen ahí.
        """
        def _worker():
            self.line.emit(f"[{tag}] $ {cmd}")
            try:
                p = subprocess.Popen(
                    ["bash","-lc", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid
                )
                if proc_slot is not None and key is not None:
                    proc_slot[key] = p

                if p.stdout:
                    for ln in p.stdout:
                        self.line.emit(f"[{tag}] {ln.rstrip()}")
                rc = p.wait()
                self.line.emit(f"[{tag}] [EXIT] rc={rc}")
            except Exception as e:
                self.line.emit(f"[{tag}] [ERROR] {e}")

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

# =========================
# ROS Worker
# =========================
class RosWorker(QObject):
    """ROS 2 worker that subscribes to images and emits Qt signals."""
    log = pyqtSignal(str)
    image = pyqtSignal(str, QImage, int, int, float)  # topic,qimg,w,h,fps

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._running = False
        self._node = None
        self._exec = None
        self._bridge = None
        self._subs: Dict[str, object] = {}
        self._fps: Dict[str, List[float]] = {}  # topic -> [count,last_ts,fps]
        self._last_clock_wall: float = 0.0

    def start(self):
        if not ROS_AVAILABLE:
            self.log.emit("[ROS] No disponible (faltan deps).")
            return
        with self._lock:
            if self._running:
                return
            self._running = True
        threading.Thread(target=self._thread_main, daemon=True).start()

    def stop(self):
        with self._lock:
            self._running = False
        try:
            if self._node and self._subs:
                for tpc, sub in list(self._subs.items()):
                    try:
                        self._node.destroy_subscription(sub)
                    except Exception:
                        pass
                self._subs.clear()
            if self._exec and self._node:
                try:
                    self._exec.remove_node(self._node)
                except Exception:
                    pass
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass
        self._node = None
        self._exec = None
        self._bridge = None
        try:
            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def clock_alive(self, max_age_sec: float = 1.5) -> Tuple[bool, float]:
        with self._lock:
            t = self._last_clock_wall
        if t <= 0:
            return (False, 1e9)
        age = time.time() - t
        return (age <= max_age_sec, age)

    def subscribe_image(self, topic: str):
        if not ROS_AVAILABLE:
            return
        topic = topic.strip()
        if not topic:
            return
        with self._lock:
            if not self._node:
                self.log.emit("[ROS] Nodo no listo todavía.")
                return
            # recrea si ya existe
            if topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception:
                    pass
                self._subs.pop(topic, None)

            def cb(msg, tpc=topic):
                self._on_image(msg, tpc)

            try:
                sub = self._node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
                self._subs[topic] = sub
                self._fps[topic] = [0.0, time.time(), 0.0]
                self.log.emit(f"[ROS] Suscrito a {topic}")
            except Exception as e:
                self.log.emit(f"[ROS] ERROR suscribiendo {topic}: {e}")

    def unsubscribe_image(self, topic: str):
        if not ROS_AVAILABLE:
            return
        topic = topic.strip()
        with self._lock:
            if self._node and topic in self._subs:
                try:
                    self._node.destroy_subscription(self._subs[topic])
                except Exception:
                    pass
                self._subs.pop(topic, None)
                self.log.emit(f"[ROS] Unsubscribe {topic}")

    def _thread_main(self):
        # init
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            try:
                rclpy.init(args=None)
            except Exception as e:
                self.log.emit(f"[ROS] ERROR rclpy.init: {e}")
                return

        try:
            self._node = rclpy.create_node("panel_superpro")
            self._bridge = CvBridge()
            self._exec = MultiThreadedExecutor(num_threads=2)
            self._exec.add_node(self._node)

            # /clock subscription (para LED real)
            def cb_clock(_msg):
                with self._lock:
                    self._last_clock_wall = time.time()

            self._node.create_subscription(Clock, "/clock", cb_clock, qos_profile_sensor_data)

            self.log.emit("[ROS] OK: nodo listo.")
        except Exception as e:
            self.log.emit(f"[ROS] ERROR creando nodo/executor: {e}")
            return

        # spin
        while True:
            with self._lock:
                if not self._running:
                    break
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception as e:
                self.log.emit(f"[ROS] WARN spin_once: {e}")
                time.sleep(0.1)

    def _decode_depth_to_bgr(self, cv_depth):
        # cv_depth: float32/uint16 1ch
        import numpy as np
        import cv2
        a = cv_depth.astype("float32")
        a[~np.isfinite(a)] = 0.0
        # recorta percentiles para evitar outliers
        lo = float(np.percentile(a, 1.0))
        hi = float(np.percentile(a, 99.0))
        if hi <= lo:
            hi = lo + 1e-3
        a = (a - lo) / (hi - lo)
        a = np.clip(a, 0.0, 1.0)
        u8 = (a * 255.0).astype("uint8")
        col = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
        return col

    def _on_image(self, msg: "Image", topic: str):
        if not ROS_AVAILABLE or not self._bridge:
            return
        try:
            import cv2
            import numpy as np

            enc = (msg.encoding or "").lower()

            # Depth -> visualización coloreada
            if "32fc1" in enc or "16uc1" in enc or "mono16" in enc or "depth" in topic:
                try:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except Exception:
                    cv_depth = self._bridge.imgmsg_to_cv2(msg)
                if cv_depth.ndim == 3:
                    cv_depth = cv_depth[:, :, 0]
                bgr = self._decode_depth_to_bgr(cv_depth)
            else:
                # RGB
                try:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except Exception:
                    # fallback si encoding viene vacío
                    w = int(msg.width)
                    h = int(msg.height)
                    step = int(msg.step)
                    data = msg.data
                    if step == w * 3 and len(data) >= h * step:
                        arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
                        # asumimos RGB
                        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    else:
                        # último intento
                        bgr = self._bridge.imgmsg_to_cv2(msg)

            h, w = bgr.shape[:2]

            # FPS
            rec = self._fps.get(topic, [0.0, time.time(), 0.0])
            rec[0] += 1.0
            dt = max(1e-6, time.time() - rec[1])
            if dt >= 1.0:
                rec[2] = rec[0] / dt
                rec[0] = 0.0
                rec[1] = time.time()
            self._fps[topic] = rec
            fps = float(rec[2])

            # cruz demo
            cv2.drawMarker(bgr, (w//2, h//2), (0,0,255), markerType=cv2.MARKER_CROSS, markerSize=26, thickness=2)

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            qimg = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
            self.image.emit(topic, qimg, w, h, fps)

        except Exception as e:
            if DEBUG_FRAME_LOG:
                self.log.emit(f"[ROS] ERROR frame {topic}: {e}")

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

class CameraTile(QWidget):
    """Widget that displays a single camera stream with controls."""
    def __init__(self, name: str, ros: RosWorker, log_fn, parent=None):
        super().__init__(parent)
        self.name = name
        self.ros = ros
        self.log_fn = log_fn
        self.frame = Frame()
        self.aspect_ratio = 0.75  # default 4:3 until first frame

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

        self.lbl = QLabel("sin imagen")
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet("background:#111; color:#aaa; border:1px solid #444;")
        self.lbl.setMinimumSize(140, 80)
        self.lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay.addWidget(self.lbl)

        self.info = QLabel("Topic: - | 0x0 | fps 0.0")
        self.info.setStyleSheet("color:#666;")
        lay.addWidget(self.info)

        self.setLayout(lay)

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

    def update_frame(self, topic: str, qimg: QImage, w: int, h: int, fps: float):
        if topic != self.frame.topic:
            return
        self.frame.qimg = qimg
        self.frame.w = w
        self.frame.h = h
        self.frame.fps = fps

        if w > 0 and h > 0:
            self.aspect_ratio = h / max(1, w)
        self._update_label_height()

        pix = QPixmap.fromImage(qimg)
        pix = pix.scaled(self.lbl.width(), self.lbl.height(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        self.lbl.setPixmap(pix)
        self.info.setText(f"Topic: {topic} | {w}x{h} | fps {fps:.1f}")

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
    debug_toggled = pyqtSignal(bool)
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

        self.led_gz = QLabel()
        self.led_bridge = QLabel()
        self.led_clock = QLabel()
        self.led_bag = QLabel()
        self.led_ros2 = QLabel()
        self.led_ur5 = QLabel()
        self.btn_debug = QPushButton("🪵 Debug logs -> terminal")
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
            "/camera_overhead/image /camera_north/image /camera_south/image "
            "/camera_east/image /camera_west/image /camera_lateral/image"
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
        set_led(self.led_ros2, "on" if ros2_running else "off")
        set_led(self.led_ur5, "on" if gz_running else "off")

    def _build_ui(self):
        layout = QHBoxLayout()
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
        self.btn_gz_start.clicked.connect(self.start_gazebo)
        self.btn_gz_stop.clicked.connect(self.stop_gazebo)
        self.btn_gz_reset.clicked.connect(self.reset_world)
        self.btn_gz_logs.clicked.connect(self.open_log)
        rbtn.addWidget(self.btn_gz_start)
        rbtn.addWidget(self.btn_gz_stop)
        gzlay.addLayout(rbtn)
        rbtn2 = QHBoxLayout()
        rbtn2.setSpacing(1)
        rbtn2.addWidget(self.btn_gz_reset)
        rbtn2.addWidget(self.btn_gz_logs)
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

        # Right status + diag
        right = QVBoxLayout()
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

        self.btn_debug.clicked.connect(self.debug_toggled.emit)
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
        # no stretch: keep compact

        layout.addLayout(left, 3)
        layout.addLayout(right, 2)

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
            # GUI si hay DISPLAY disponible (incluye XQuartz vía SSH)
            if os.environ.get("DISPLAY"):
                return "gui"
            return "headless"
        if m.startswith("gui"):
            return "gui"
        return "headless"

    def _refresh_clock_led(self):
        ok, age = self.ros.clock_alive()
        set_led(self.led_clock, "on" if ok else "off")

    def start_gazebo(self):
        self.log_fn("[UI] Botón: Lanzar Gazebo -> iniciar simulación")
        ensure_dir(LOG_DIR)
        world = self.world_combo.currentText().strip()
        mode = self._effective_mode()

        # NUEVA partición por arranque (y la reutilizamos en bridge)
        self.gz_partition = f"ur5pro_{int(time.time())}"

        set_led(self.led_gz, "warn")
        gz_log = os.path.join(LOG_DIR, "gz_server.log" if mode != "gui" else "gz_gui.log")

        # env fijo para evitar multicast “Network is unreachable”
        env = (
            f"export GZ_IP='{os.environ.get('GZ_IP','127.0.0.1')}' ; "
            f"export GZ_TRANSPORT_IP='{os.environ.get('GZ_TRANSPORT_IP','127.0.0.1')}' ; "
            f"export GZ_PARTITION='{self.gz_partition}' ; "
            f"export GZ_SIM_RESOURCE_PATH='{MODELS_DIR}:{WORLDS_DIR}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
        )

        if mode == "gui":
            cmd = (
                bash_preamble(WS_DIR) +
                env +
                "unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT QT_OPENGL QT_XCB_GL_INTEGRATION; "
                f"gz sim -r -v 3 '{world}' > '{gz_log}' 2>&1"
            )
        else:
            cmd = (
                bash_preamble(WS_DIR) +
                env +
                f"env -u DISPLAY __EGL_VENDOR_LIBRARY_FILENAMES='{EGL_VENDOR}' "
                "GZ_RENDER_ENGINE=ogre2 "
                f"gz sim -s -r -v 3 --headless-rendering '{world}' > '{gz_log}' 2>&1"
            )

        # usamos stream y guardamos proc
        self.runner.run_stream("SIM", cmd, proc_slot=self.procs, key="gazebo")
        set_led(self.led_gz, "on")
        self._gz_running = True
        self._refresh_buttons()

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

        env = (
            f"export GZ_IP='{os.environ.get('GZ_IP','127.0.0.1')}' ; "
            f"export GZ_TRANSPORT_IP='{os.environ.get('GZ_TRANSPORT_IP','127.0.0.1')}' ; "
            f"export GZ_PARTITION='{self.gz_partition or os.environ.get('GZ_PARTITION','')}' ; "
        )

        lazy_on = self.lazy_chk.currentIndex() == 0
        lazy_arg = " -p lazy:=true " if lazy_on else ""

        cmd = (
            bash_preamble(WS_DIR)
            + env
            + "ros2 run ros_gz_bridge parameter_bridge --ros-args "
            + f"-p config_file:='{runtime_yaml}'{lazy_arg} > '{br_log}' 2>&1"
        )

        self.runner.run_stream("BRIDGE", cmd, proc_slot=self.procs, key="bridge")
        set_led(self.led_bridge, "on")
        self.log_fn(f"[INFO] Bridge runtime YAML: {runtime_yaml}")
        self._bridge_running = True
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
        topics = self.bag_topics.text().strip()
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
            f"ros2 bag record -o '{outdir}' --topics {topics}"
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

    def __init__(self, runner: CmdRunner, ros: RosWorker, log_fn, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.ros = ros
        self.log_fn = log_fn

        self.tiles: List[CameraTile] = []
        self.topics: List[str] = []

        self.topics_found.connect(self._apply_topics)
        self._build_ui()

    def _build_ui(self):
        lay = QVBoxLayout()
        lay.setContentsMargins(1, 1, 1, 1)
        lay.setSpacing(1)

        top = QHBoxLayout()
        top.setSpacing(1)
        b1 = QPushButton("Auto-descubrir topics (ROS 2)")
        b2 = QPushButton("Abrir evidencias")
        b1.clicked.connect(self.discover_topics)
        b2.clicked.connect(self.open_evidences)
        top.addWidget(b1)
        top.addWidget(b2)
        top.addStretch(1)
        lay.addLayout(top)

        names = ["Mesa", "Frente", "Cesta"]
        for nm in names:
            t = CameraTile(nm, self.ros, self.log_fn)
            self.tiles.append(t)

        # Ajuste inicial para proporciones 4:3, se recalcula al recibir imagen
        self.tiles[0].aspect_ratio = 0.75
        self.tiles[1].aspect_ratio = 0.75
        self.tiles[2].aspect_ratio = 0.75

        stack = QVBoxLayout()
        stack.setSpacing(1)
        for t in self.tiles:
            stack.addWidget(t)
        lay.addLayout(stack)

        self.setLayout(lay)

    def open_evidences(self):
        self.log_fn("[UI] Botón: Abrir evidencias")
        ensure_dir(FIG_DIR)
        subprocess.Popen(["bash","-lc", f"xdg-open '{FIG_DIR}' >/dev/null 2>&1 || true"])

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
        cmd = bash_preamble(WS_DIR) + "ros2 launch ur5_bringup ur5_ros2_control.launch.py"
        self.runner.run_stream("ROBOT-CTRL", cmd)

    def _apply_topics(self, topics: List[str]):
        if topics:
            for tile in self.tiles:
                tile.set_topics(topics)
        else:
            self.log_fn("[CAMS] No se detectaron tópicos Image. ¿Está el bridge activo?")
            return
        # defaults “bonitos”
        defaults = [
            "/camera_overhead/image",
            "/camera_west/image",
            "/camera_east/image",
        ]
        for i, tile in enumerate(self.tiles):
            if i < len(defaults) and defaults[i] in topics:
                tile.combo.setCurrentText(defaults[i])
            elif i < len(topics):
                tile.combo.setCurrentText(topics[i])

class RobotTab(QWidget):
    """UI tab for robot scripts and diagnostics."""
    def __init__(self, runner: CmdRunner, log_fn, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.log_fn = log_fn
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
        self.btn_table = QPushButton("UR5 → Mesa")
        self.btn_basket = QPushButton("UR5 → Cesta")
        self.btn_open = QPushButton("Abrir gripper")
        self.btn_close = QPushButton("Cerrar gripper")
        self.btn_test = QPushButton("Test corto")
        self.btn_eval = QPushButton("Evaluar último experimento")
        self.btn_ctrl = QPushButton("Start UR5 controllers")
        self.btn_ros2 = QPushButton("Start UR5 ros2_control")
        self.btn_diag = QPushButton("Diagnóstico robot")

        self.btn_home.clicked.connect(lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_home.sh")))
        self.btn_set_home.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_set_home_boot.sh"))
        )
        self.btn_table.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_table_pose.sh"))
        )
        self.btn_basket.clicked.connect(
            lambda: self._run_script("ROBOT", os.path.join(SCRIPTS_DIR, "ur5_go_basket_pose.sh"))
        )
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
        self.btn_ctrl.clicked.connect(self._start_ur5_controllers)
        self.btn_ros2.clicked.connect(self._start_ur5_ros2_control)
        self.btn_diag.clicked.connect(self._diag_robot)

        grid = QGridLayout()
        grid.setHorizontalSpacing(1)
        grid.setVerticalSpacing(1)
        grid.addWidget(self.btn_home, 0, 0)
        grid.addWidget(self.btn_open, 0, 1)
        grid.addWidget(self.btn_close, 0, 2)
        grid.addWidget(self.btn_test, 1, 0)
        grid.addWidget(self.btn_ctrl, 1, 1)
        grid.addWidget(self.btn_ros2, 1, 2)
        grid.addWidget(self.btn_set_home, 2, 0)
        grid.addWidget(self.btn_table, 2, 1)
        grid.addWidget(self.btn_basket, 2, 2)
        grid.addWidget(self.btn_eval, 3, 0)
        grid.addWidget(self.btn_diag, 3, 1)
        gl.addLayout(grid)

        g.setLayout(gl)
        lay.addWidget(g)

        info = QLabel(
            "Nota: si un script queda en 'Waiting for matching subscription(s)...' falta el nodo/controlador.\n"
            "Para control completo hace falta bringup ROS2-control del UR5."
        )
        info.setWordWrap(True)
        info.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        lay.addWidget(info)
        # no stretch: keep compact
        self.setLayout(lay)
        self.set_robot_state(False, False)

    def set_robot_state(self, running: bool, ros2_running: bool):
        self.btn_home.setEnabled(running)
        self.btn_set_home.setEnabled(running)
        self.btn_table.setEnabled(running)
        self.btn_basket.setEnabled(running)
        self.btn_open.setEnabled(running)
        self.btn_close.setEnabled(running)
        self.btn_test.setEnabled(running)
        self.btn_ctrl.setEnabled(ros2_running)
        self.btn_ros2.setEnabled(not ros2_running)
        self.btn_eval.setEnabled(True)
        self.btn_diag.setEnabled(True)

    def _run_script(self, tag: str, script_path: str):
        self.log_fn(f"[UI] Botón: {tag} -> {os.path.basename(script_path)}")
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

    def _diag_robot(self):
        self.log_fn("[UI] Botón: Diagnóstico robot")
        self.log_fn("[ROBOT] Diagnóstico: topics/nodos/controladores")
        cmd_topics = bash_preamble(WS_DIR) + "ros2 topic list | egrep 'ur|gripper|joint|controller|trajectory' || true"
        cmd_nodes = bash_preamble(WS_DIR) + "ros2 node list | egrep 'ur|controller|moveit|gazebo|robot' || true"
        cmd_ctrl = bash_preamble(WS_DIR) + "ros2 control list_controllers || true"
        self.runner.run_stream("ROBOT-DIAG", cmd_topics)
        self.runner.run_stream("ROBOT-DIAG", cmd_nodes)
        self.runner.run_stream("ROBOT-DIAG", cmd_ctrl)

    def _run_robot_test(self):
        self.log_fn("[UI] Botón: Test corto")
        if not robot_control_available():
            self.log_fn("[ROBOT] Ni Gazebo ni ros2_control están activos. Arranca START ALL.")
            return
        if ros2_control_running():
            active = self._robot_ready(require_gripper=False)
            if not active:
                return
        s_test = os.path.join(SCRIPTS_DIR, "ur5_quick_test.sh")
        if not os.path.isfile(s_test):
            self.log_fn(f"[ROBOT] ERROR: no existe {s_test}")
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
        cmd = bash_preamble(WS_DIR) + (
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "-c /controller_manager --activate --controller-manager-timeout 30 --switch-timeout 30 || true; "
            "ros2 run controller_manager spawner joint_trajectory_controller "
            "-c /controller_manager --activate --controller-manager-timeout 30 --switch-timeout 30 || true; "
        )
        if gripper_controller_defined():
            cmd += (
                "ros2 run controller_manager spawner gripper_controller "
                "-c /controller_manager --activate --controller-manager-timeout 30 --switch-timeout 30 || true; "
            )
        else:
            self.log_fn("[ROBOT] gripper_controller no definido en ur5_controllers.yaml")
        self.runner.run_stream("ROBOT-CTRL", cmd)

    def _start_ur5_ros2_control(self):
        self.log_fn("[UI] Botón: Start UR5 ros2_control")
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

class EvidencesTab(QWidget):
    """UI tab for evidence folders and snapshots."""
    def __init__(self, log_fn, parent=None):
        super().__init__(parent)
        self.log_fn = log_fn
        self._build_ui()

    def _build_ui(self):
        lay = QVBoxLayout()

        b1 = QPushButton("Abrir figures_memoria/")
        b2 = QPushButton("Abrir bags/")
        b3 = QPushButton("Copiar figures_memoria → log (snapshot)")
        b1.clicked.connect(self.open_fig)
        b2.clicked.connect(self.open_bags)
        b3.clicked.connect(self.snapshot)

        lay.addWidget(b1)
        lay.addWidget(b2)
        lay.addWidget(b3)
        # no stretch: keep compact
        self.setLayout(lay)

    def open_fig(self):
        ensure_dir(FIG_DIR)
        subprocess.Popen(["bash","-lc", f"xdg-open '{FIG_DIR}' >/dev/null 2>&1 || true"])

    def open_bags(self):
        ensure_dir(BAGS_DIR)
        subprocess.Popen(["bash","-lc", f"xdg-open '{BAGS_DIR}' >/dev/null 2>&1 || true"])

    def snapshot(self):
        ensure_dir(LOG_DIR)
        ensure_dir(FIG_DIR)
        dst = os.path.join(LOG_DIR, f"figures_memoria_snapshot_{now_tag()}")
        ensure_dir(dst)
        try:
            for fn in os.listdir(FIG_DIR):
                if fn.lower().endswith((".png",".jpg",".jpeg")):
                    shutil.copy2(os.path.join(FIG_DIR, fn), os.path.join(dst, fn))
            self.log_fn(f"[EVID] Snapshot -> {dst}")
        except Exception as e:
            self.log_fn(f"[EVID] ERROR snapshot: {e}")

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

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel SUPER PRO — Agarre inteligente (ROS 2 Jazzy + Gazebo)")

        ensure_dir(LOG_DIR)
        ensure_dir(BAGS_DIR)
        ensure_dir(FIG_DIR)

        self.runner = CmdRunner()
        self.runner.line.connect(self._append_log)

        self.ros = RosWorker()
        self.ros.log.connect(self._append_log)
        self.ros.image.connect(self._on_image)
        self.ros.start()

        self.log_signal.connect(self._append_log_ui)
        self.robot_state_signal.connect(self._apply_robot_state)

        central = QWidget()
        root = QVBoxLayout()
        root.setContentsMargins(1, 1, 1, 1)
        root.setSpacing(1)

        self.tab_sim = SimulationTab(self.runner, self.ros, self._append_log)
        self.tab_cam = CamerasTab(self.runner, self.ros, self._append_log)
        self.tab_robot = RobotTab(self.runner, self._append_log)
        self.tab_evid = EvidencesTab(self._append_log)

        self.tab_sim.debug_toggled.connect(self.toggle_debug_logs)
        self.tab_sim.state_changed.connect(self._schedule_robot_state_check)
        self.log_dialog = LogDialog(self)

        left = QWidget()
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(1, 1, 1, 1)
        left_layout.setSpacing(1)
        left_layout.addWidget(self.tab_sim)
        left_bottom = QHBoxLayout()
        left_bottom.setContentsMargins(0, 0, 0, 0)
        left_bottom.setSpacing(1)
        left_bottom.addWidget(self.tab_robot)
        left_bottom.addWidget(self.tab_evid)
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

        root.addWidget(self.main_split)

        bottom = QHBoxLayout()
        bottom.setContentsMargins(1, 1, 1, 1)
        bottom.setSpacing(1)
        b_start = QPushButton("✅ START ALL")
        b_stop = QPushButton("🛑 STOP ALL")
        b_kill = QPushButton("💀 KILL HARD")
        b_logs = QPushButton("🧾 Logs")
        b_close = QPushButton("Cerrar panel")

        b_start.clicked.connect(self.start_all)
        b_stop.clicked.connect(self.stop_all)
        b_kill.clicked.connect(self.kill_hard)
        b_logs.clicked.connect(self.toggle_logs)
        b_close.clicked.connect(self.close_panel)

        self.alert_lbl = QLabel("OK")
        self.alert_lbl.setStyleSheet("background:#16a34a; color:white; padding:4px 10px; border-radius:8px;")

        bottom.addWidget(b_start)
        bottom.addWidget(b_stop)
        bottom.addWidget(b_kill)
        bottom.addWidget(b_logs)
        bottom.addWidget(self.alert_lbl)
        bottom.addStretch(1)
        bottom.addWidget(b_close)

        root.addLayout(bottom)

        central.setLayout(root)
        self.setCentralWidget(central)

        # COLD BOOT dentro del panel (si está activado)
        if os.environ.get("PANEL_COLD_BOOT", "1") == "1":
            QTimer.singleShot(0, self._run_cold_boot)

        self._debug_logs_to_stdout = False
        self._robot_state_check_running = False
        self._robot_state_timer = QTimer(self)
        self._robot_state_timer.timeout.connect(self._schedule_robot_state_check)
        self._robot_state_timer.start(2000)
        QTimer.singleShot(0, self._fit_to_screen)

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

    def _append_log(self, text: str):
        if QThread.currentThread() != QApplication.instance().thread():
            self.log_signal.emit(text)
            return
        self._append_log_ui(text)

    def _append_log_ui(self, text: str):
        self.log_dialog.append(text)
        if "[ERROR]" in text:
            self.alert_lbl.setText("ERROR")
            self.alert_lbl.setStyleSheet("background:#dc2626; color:white; padding:4px 10px; border-radius:8px;")
        elif "[WARN]" in text:
            self.alert_lbl.setText("WARN")
            self.alert_lbl.setStyleSheet("background:#f59e0b; color:white; padding:4px 10px; border-radius:8px;")
        if self._debug_logs_to_stdout:
            print(text, flush=True)

    def toggle_debug_logs(self, checked: bool):
        self._debug_logs_to_stdout = bool(checked)
        state = "ON" if self._debug_logs_to_stdout else "OFF"
        self._append_log(f"[DEBUG] Logs en terminal: {state}")

    def toggle_logs(self):
        if self.log_dialog.isVisible():
            self.log_dialog.hide()
        else:
            self.log_dialog.show()
            self.log_dialog.raise_()
            self.log_dialog.activateWindow()

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
        self.tab_robot.set_robot_state(running, ros2_running)
        self.tab_sim.set_robot_leds(ros2_running, gz_running)

    def _on_image(self, topic: str, qimg: QImage, w: int, h: int, fps: float):
        # update tiles
        for t in self.tab_cam.tiles:
            t.update_frame(topic, qimg, w, h, fps)

    def start_all(self):
        self._append_log("[UI] Botón: START ALL")
        self._append_log("[START] Secuencia START ALL: stop -> start Gazebo -> start Bridge")
        self.stop_all()
        # Gazebo
        self.tab_sim.start_gazebo()
        # Bridge tras un pequeño delay
        QTimer.singleShot(1200, self.tab_sim.start_bridge)

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
