#!/usr/bin/env python3
import os
import sys
import subprocess
from typing import Optional, Callable

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QLabel,
    QTextEdit,
    QGroupBox,
    QComboBox,
    QTabWidget,
    QSizePolicy,
    QMessageBox,
)

# --- Integración ROS 2 + OpenCV (para mostrar imágenes) ---
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    ROS_AVAILABLE = True
except Exception as e:
    print(f"[AVISO] ROS 2 / cv_bridge no disponible en el panel: {e}", file=sys.stderr)


class ClickableLabel(QLabel):
    clicked = pyqtSignal(int, int)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x = event.pos().x()
            y = event.pos().y()
            self.clicked.emit(x, y)
        super().mousePressEvent(event)


class RealCamerasTab(QWidget):
    def __init__(self, run_cmd: Callable[[str, Optional[str]], None], parent=None):
        super().__init__(parent)
        self.run_cmd = run_cmd
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout()

        group_real = QGroupBox("Cámaras reales (hardware físico)")
        real_layout = QVBoxLayout()

        label_info = QLabel(
            "Aquí se configurarán las cámaras FÍSICAS conectadas al sistema.\n"
            "En el MeLE, normalmente no las usarás, pero dejamos todo preparado."
        )
        label_info.setWordWrap(True)

        btn_start_real = QPushButton("Iniciar cámaras reales (placeholder)")
        btn_stop_real = QPushButton("Detener cámaras reales (placeholder)")

        btn_start_real.clicked.connect(self.start_real_cameras)
        btn_stop_real.clicked.connect(self.stop_real_cameras)

        real_layout.addWidget(label_info)
        real_layout.addWidget(btn_start_real)
        real_layout.addWidget(btn_stop_real)
        group_real.setLayout(real_layout)

        layout.addWidget(group_real)

        placeholder = QLabel(
            "🔧 Aquí más adelante conectaremos scripts reales de cámaras físicas.\n"
            "Por ahora, esta pestaña es informativa."
        )
        placeholder.setWordWrap(True)
        layout.addWidget(placeholder)

        layout.addStretch()
        self.setLayout(layout)

    def start_real_cameras(self):
        self.run_cmd(
            "echo 'TODO: lanzar nodos de cámaras REALES (no configurado todavía)'",
            context="[Reales]",
        )

    def stop_real_cameras(self):
        self.run_cmd(
            "echo 'TODO: detener nodos de cámaras REALES (no configurado todavía)'",
            context="[Reales]",
        )


class SyntheticImagesTab(QWidget):
    def __init__(self, run_cmd: Callable[[str, Optional[str]], None], parent=None):
        super().__init__(parent)
        self.run_cmd = run_cmd

        self.status_gazebo = QLabel()
        self.status_bridge = QLabel()
        self.status_cameras = QLabel()

        self._build_ui()

    def _build_ui(self):
        main_layout = QHBoxLayout()

        controls_group = QGroupBox("Simulación y bridge")
        controls_layout = QVBoxLayout()

        # --- Gazebo ---
        gazebo_group = QGroupBox("Gazebo – UR5 + mesa + objetos")
        gazebo_layout = QVBoxLayout()

        label_info = QLabel(
            "Desde aquí lanzaremos todo el entorno SIMULADO:\n"
            "- Gazebo con el mundo ur5_mesa_objetos.sdf\n"
            "- Nodos ROS 2 asociados\n"
            "- Cámaras sintéticas (tópicos /camera/...)"
        )
        label_info.setWordWrap(True)
        gazebo_layout.addWidget(label_info)

        btn_launch_gazebo = QPushButton("Lanzar Gazebo (UR5 + mesa + objetos)")
        btn_stop_gazebo = QPushButton("Detener Gazebo")

        btn_launch_gazebo.clicked.connect(self.launch_gazebo_ur5_world)
        btn_stop_gazebo.clicked.connect(self.stop_gazebo)

        gazebo_layout.addWidget(btn_launch_gazebo)
        gazebo_layout.addWidget(btn_stop_gazebo)
        gazebo_group.setLayout(gazebo_layout)

        # --- Bridge ---
        bridge_group = QGroupBox("Bridge ROS 2 ↔ Gazebo")
        bridge_layout = QVBoxLayout()

        label_bridge = QLabel(
            "Bridge ros_gz_bridge para exponer los tópicos de Gazebo en ROS 2.\n"
            "Se apoya en el script run_gz_ros_bridge.sh."
        )
        label_bridge.setWordWrap(True)

        btn_launch_bridge = QPushButton("Lanzar bridge")
        btn_stop_bridge = QPushButton("Detener bridge")

        btn_launch_bridge.clicked.connect(self.launch_bridge)
        btn_stop_bridge.clicked.connect(self.stop_bridge)

        bridge_layout.addWidget(label_bridge)
        bridge_layout.addWidget(btn_launch_bridge)
        bridge_layout.addWidget(btn_stop_bridge)
        bridge_group.setLayout(bridge_layout)

        # --- Cámaras sintéticas ---
        cameras_group = QGroupBox("Cámaras sintéticas (tópicos ROS 2)")
        cam_layout = QVBoxLayout()

        label_cam = QLabel(
            "Aquí podrás:\n"
            "- Comprobar los tópicos /camera_lateral/image, /camera_overhead/image, etc.\n"
            "- Ver si el bridge está funcionando correctamente."
        )
        label_cam.setWordWrap(True)

        btn_check_cameras = QPushButton("Comprobar tópicos de cámara (ROS 2)")
        btn_check_cameras.clicked.connect(self.check_cameras_topics)

        cam_layout.addWidget(label_cam)
        cam_layout.addWidget(btn_check_cameras)
        cameras_group.setLayout(cam_layout)

        controls_layout.addWidget(gazebo_group)
        controls_layout.addWidget(bridge_group)
        controls_layout.addWidget(cameras_group)
        controls_layout.addStretch()
        controls_group.setLayout(controls_layout)

        # --- Columna de estado ---
        status_group = QGroupBox("Estado de la simulación")
        status_layout = QGridLayout()

        self._setup_led(status_layout, 0, "Gazebo", self.status_gazebo)
        self._setup_led(status_layout, 1, "Bridge ROS 2 ↔ Gazebo", self.status_bridge)
        self._setup_led(status_layout, 2, "Cámaras sintéticas (ROS 2)", self.status_cameras)

        status_group.setLayout(status_layout)

        main_layout.addWidget(controls_group, stretch=3)
        main_layout.addWidget(status_group, stretch=2)
        self.setLayout(main_layout)

    def _setup_led(self, layout: QGridLayout, row: int, text: str, label: QLabel):
        label.setText("●")
        self._set_led_color(label, "off")
        desc = QLabel(text)
        desc.setWordWrap(True)
        layout.addWidget(desc, row, 0)
        layout.addWidget(label, row, 1)

    def _set_led_color(self, label: QLabel, state: str):
        color_map = {
            "off": "#555555",
            "on": "#00cc44",
            "warn": "#ff9900",
            "error": "#cc0000",
        }
        color = color_map.get(state, "#555555")
        label.setStyleSheet(f"color: {color}; font-size: 18px;")

    def launch_gazebo_ur5_world(self):
        self._set_led_color(self.status_gazebo, "warn")
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/run_ur5_world.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Gazebo lanzado en background (run_ur5_world.sh en el MeLE)'",
                context="[Sintéticas:Gazebo]",
            )
            self._set_led_color(self.status_gazebo, "on")
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al lanzar Gazebo: {e}'",
                context="[Sintéticas:Gazebo]",
            )
            self._set_led_color(self.status_gazebo, "off")

    def stop_gazebo(self):
        cmd = "pkill -f 'gz sim' || echo 'Gazebo no estaba ejecutándose'"
        self.run_cmd(cmd, context="[Sintéticas:Gazebo]")
        self._set_led_color(self.status_gazebo, "off")

    def launch_bridge(self):
        self._set_led_color(self.status_bridge, "warn")
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/run_gz_ros_bridge.sh"

        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Bridge ros_gz_bridge lanzado en background desde run_gz_ros_bridge.sh'",
                context="[Sintéticas:Bridge]",
            )
            self._set_led_color(self.status_bridge, "on")
        except Exception as e:
            err_msg = f"ERROR al lanzar el bridge: {e}"
            self.run_cmd(f"echo '{err_msg}'", context="[Sintéticas:Bridge]")
            self._set_led_color(self.status_bridge, "off")

    def stop_bridge(self):
        cmd = "pkill -f 'ros_gz_bridge' || echo 'Bridge no estaba ejecutándose'"
        self.run_cmd(cmd, context="[Sintéticas:Bridge]")
        self._set_led_color(self.status_bridge, "off")

    def check_cameras_topics(self):
        self._set_led_color(self.status_cameras, "warn")
        cmd = (
            "cd ~/TFM/agarre_ros2_ws && "
            "source /opt/ros/jazzy/setup.bash && "
            "source install/setup.bash && "
            "ros2 topic list | grep camera || echo 'No hay tópicos /camera activos'"
        )
        self.run_cmd(cmd, context="[Sintéticas:Cámaras]")
        self._set_led_color(self.status_cameras, "on")


class ControlTab(QWidget):
    def __init__(self, run_cmd: Callable[[str, Optional[str]], None], parent=None):
        super().__init__(parent)
        self.run_cmd = run_cmd

        self.ros_node: Optional[Node] = None
        self.bridge: Optional[CvBridge] = None
        self.image_sub = None
        self.image_timer: Optional[QTimer] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.current_camera_topic: Optional[str] = None

        if ROS_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
            except Exception:
                try:
                    rclpy.init(args=None)
                except Exception:
                    pass

            try:
                if rclpy.ok():
                    self.ros_node = rclpy.create_node("qt_control_panel")
                    self.bridge = CvBridge()
                    self.executor = SingleThreadedExecutor()
                    self.executor.add_node(self.ros_node)

                    self.image_timer = QTimer(self)
                    self.image_timer.timeout.connect(self._spin_ros_once)
                    self.image_timer.start(50)  # ~20 Hz
            except Exception as e:
                print(f"[AVISO] No se pudo crear el nodo ROS del panel: {e}", file=sys.stderr)
                self.ros_node = None
                self.bridge = None
                self.executor = None
                self.image_timer = None

        self.lbl_robot_pose: Optional[QLabel] = None
        self.lbl_object_pixel: Optional[QLabel] = None
        self.lbl_mode: Optional[QLabel] = None

        self.selection_mode = "manual"
        self.last_clicked_pixel = None  # (x, y)

        self._build_ui()

    def _build_ui(self):
        layout = QHBoxLayout()

        # --------- Vista de cámara ----------
        view_group = QGroupBox("Vista de cámara / simulación")
        view_layout = QVBoxLayout()

        self.image_placeholder = ClickableLabel("Aquí se mostrará la imagen de cámara")
        self.image_placeholder.setAlignment(Qt.AlignCenter)
        self.image_placeholder.setStyleSheet(
            "background-color: #202020; color: #aaaaaa; border: 1px solid #555555;"
        )
        self.image_placeholder.setMinimumSize(400, 300)
        self.image_placeholder.clicked.connect(self._on_image_clicked)

        view_layout.addWidget(self.image_placeholder)

        cam_select_layout = QHBoxLayout()
        cam_select_layout.addWidget(QLabel("Tópico de cámara:"))

        self.combo_camera = QComboBox()
        # Por defecto, ponemos primero la cenital del mundo simulado
        self.combo_camera.addItems([
            "/camera_overhead/image",
            "/camera_north/image",
            "/camera_south/image",
            "/camera_east/image",
            "/camera_west/image",
            "/camera_lateral/image",
            "/camera/color/image_raw",
        ])


        cam_select_layout.addWidget(self.combo_camera)

        btn_connect_cam = QPushButton("Conectar cámara")
        btn_disconnect_cam = QPushButton("Desconectar cámara")

        btn_connect_cam.clicked.connect(self._on_connect_camera)
        btn_disconnect_cam.clicked.connect(self._on_disconnect_camera)

        cam_select_layout.addWidget(btn_connect_cam)
        cam_select_layout.addWidget(btn_disconnect_cam)

        view_layout.addLayout(cam_select_layout)

        # Modo de selección
        mode_layout = QHBoxLayout()
        btn_manual = QPushButton("Selección MANUAL")
        btn_auto = QPushButton("Selección AUTO (IA)")

        btn_manual.setCheckable(True)
        btn_auto.setCheckable(True)
        btn_manual.setChecked(True)

        def set_manual():
            btn_manual.setChecked(True)
            btn_auto.setChecked(False)
            self.selection_mode = "manual"
            self._set_mode_text("Modo de selección: MANUAL (clic en la imagen)")

        def set_auto():
            btn_manual.setChecked(False)
            btn_auto.setChecked(True)
            self.selection_mode = "auto"
            self._set_mode_text("Modo de selección: AUTO (coordenadas desde IA)")

        btn_manual.clicked.connect(set_manual)
        btn_auto.clicked.connect(set_auto)

        mode_layout.addWidget(btn_manual)
        mode_layout.addWidget(btn_auto)
        view_layout.addLayout(mode_layout)

        view_group.setLayout(view_layout)

        # --------- Lado derecho: estado + UR5 + experimentos ----------
        right_layout = QVBoxLayout()

        status_group = QGroupBox("Estado del sistema")
        status_layout = QVBoxLayout()

        self.lbl_robot_pose = QLabel("UR5: posición desconocida")
        self.lbl_object_pixel = QLabel("Objeto (px): sin seleccionar")
        self.lbl_mode = QLabel("Modo de selección: MANUAL (clic en la imagen)")

        for lbl in (self.lbl_robot_pose, self.lbl_object_pixel, self.lbl_mode):
            lbl.setWordWrap(True)
            status_layout.addWidget(lbl)

        status_group.setLayout(status_layout)

        ur5_group = QGroupBox("Controles UR5 (ROS 2)")
        ur5_layout = QVBoxLayout()

        btn_home = QPushButton("Ir a posición HOME")
        btn_test_pose = QPushButton("Ir a posición de PRUEBA")
        btn_open_gripper = QPushButton("Abrir gripper")
        btn_close_gripper = QPushButton("Cerrar gripper")

        btn_home.clicked.connect(self.go_home_pose)
        btn_test_pose.clicked.connect(self.go_test_pose)
        btn_open_gripper.clicked.connect(self.open_gripper)
        btn_close_gripper.clicked.connect(self.close_gripper)

        ur5_layout.addWidget(btn_home)
        ur5_layout.addWidget(btn_test_pose)
        ur5_layout.addWidget(btn_open_gripper)
        ur5_layout.addWidget(btn_close_gripper)
        ur5_group.setLayout(ur5_layout)

        exp_group = QGroupBox("Experimentos de agarre (scripts TFM)")
        exp_layout = QVBoxLayout()

        btn_run_rgb = QPushButton("Lanzar experimento RGB")
        btn_run_rgbd = QPushButton("Lanzar experimento RGB-D")
        btn_eval_last = QPushButton("Evaluar último experimento")

        btn_run_rgb.clicked.connect(self.run_experiment_rgb)
        btn_run_rgbd.clicked.connect(self.run_experiment_rgbd)
        btn_eval_last.clicked.connect(self.eval_last_experiment)

        exp_layout.addWidget(btn_run_rgb)
        exp_layout.addWidget(btn_run_rgbd)
        exp_layout.addWidget(btn_eval_last)
        exp_group.setLayout(exp_layout)

        right_layout.addWidget(status_group)
        right_layout.addWidget(ur5_group)
        right_layout.addWidget(exp_group)
        right_layout.addStretch()

        layout.addWidget(view_group, stretch=3)
        layout.addLayout(right_layout, stretch=2)
        self.setLayout(layout)

    # --------- ROS 2: executor y cámara ----------
    def _spin_ros_once(self):
        if self.executor is not None:
            try:
                self.executor.spin_once(timeout_sec=0.0)
            except Exception as e:
                print(f"[AVISO] Error en executor.spin_once: {e}", file=sys.stderr)

    def _on_connect_camera(self):
        topic = self.combo_camera.currentText().strip()
        if not ROS_AVAILABLE or self.ros_node is None or self.bridge is None:
            self.run_cmd(
                "echo 'ROS 2 / cv_bridge no están disponibles en este panel (revisa instalación)'",
                context="[Control:Cámara]",
            )
            return

        if self.image_sub is not None:
            try:
                self.ros_node.destroy_subscription(self.image_sub)
            except Exception:
                pass
            self.image_sub = None

        self.current_camera_topic = topic
        try:
            # QoS específico para sensores (cámaras, lidars, etc.)
            self.image_sub = self.ros_node.create_subscription(
                Image,
                topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
            self.run_cmd(
                f"echo 'Suscrito a tópico de cámara: {topic}'",
                context="[Control:Cámara]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al suscribirse al tópico {topic}: {e}'",
                context="[Control:Cámara]",
            )

    def _on_disconnect_camera(self):
        if self.image_sub is not None and self.ros_node is not None:
            try:
                self.ros_node.destroy_subscription(self.image_sub)
            except Exception:
                pass
            self.image_sub = None
            self.current_camera_topic = None
            self.run_cmd(
                "echo 'Suscripción a cámara eliminada'",
                context="[Control:Cámara]",
            )

    def _image_callback(self, msg: Image):
        """Callback de la imagen: convierte a QImage y la pinta en el QLabel."""
        if not ROS_AVAILABLE or self.bridge is None:
            return
        try:
            # 1) ROS -> OpenCV (BGR)
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 2) DEBUG: dibujar una cruz roja en el centro
            h, w, ch = cv_img.shape
            center = (w // 2, h // 2)
            cv2.drawMarker(
                cv_img,
                center,
                (0, 0, 255),           # rojo en BGR
                markerType=cv2.MARKER_CROSS,
                markerSize=30,
                thickness=2,
            )

            # 3) BGR -> RGB para Qt
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

            # 4) Log de depuración
            self.run_cmd(
                f"echo 'Frame de cámara recibido: {w}x{h}'",
                context="[Control:Cámara]",
            )

            # 5) OpenCV -> QImage -> QPixmap
            bytes_per_line = ch * w
            qimg = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            qimg = qimg.copy()

            pix = QPixmap.fromImage(qimg)

            # 6) Escalar a tamaño del QLabel manteniendo proporción
            label_w = max(1, self.image_placeholder.width())
            label_h = max(1, self.image_placeholder.height())
            pix = pix.scaled(
                label_w,
                label_h,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )

            # 7) Mostrar en el QLabel
            self.image_placeholder.setPixmap(pix)

        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR en _image_callback: {e}'",
                context="[Control:Cámara]",
            )


    # --------- Clic en imagen ----------
    def _on_image_clicked(self, x: int, y: int):
        if self.selection_mode != "manual":
            self.run_cmd(
                "echo 'Ignorando clic: modo de selección AUTO (IA)'",
                context="[Control:Selección]",
            )
            return

        self.last_clicked_pixel = (x, y)
        self._set_object_pixel_text(f"Objeto (px): ({x}, {y})")
        self.run_cmd(
            f"echo 'Píxel de objeto seleccionado manualmente: ({x}, {y})'",
            context="[Control:Selección]",
        )

    # --------- Helpers de estado ----------
    def _set_robot_pose_text(self, text: str):
        if self.lbl_robot_pose is not None:
            self.lbl_robot_pose.setText(text)

    def _set_object_pixel_text(self, text: str):
        if self.lbl_object_pixel is not None:
            self.lbl_object_pixel.setText(text)

    def _set_mode_text(self, text: str):
        if self.lbl_mode is not None:
            self.lbl_mode.setText(text)

    # --------- UR5 ----------
    def go_home_pose(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/ur5_go_home.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd("echo 'Comando enviado: UR5 → HOME'", context="[Control:UR5]")
            self._set_robot_pose_text("UR5: posición HOME")
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al mover UR5 HOME: {e}'", context="[Control:UR5]"
            )

    def go_test_pose(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/ur5_go_test_pose.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Comando enviado: UR5 → POSICIÓN DE PRUEBA'",
                context="[Control:UR5]",
            )
            self._set_robot_pose_text("UR5: posición de PRUEBA")
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al mover UR5 a prueba: {e}'", context="[Control:UR5]"
            )

    def open_gripper(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/ur5_open_gripper.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Comando enviado: ABRIR gripper'",
                context="[Control:UR5]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al abrir gripper: {e}'", context="[Control:UR5]"
            )

    def close_gripper(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/ur5_close_gripper.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Comando enviado: CERRAR gripper'",
                context="[Control:UR5]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al cerrar gripper: {e}'", context="[Control:UR5]"
            )

    # --------- Experimentos ----------
    def run_experiment_rgb(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/run_experiment_rgb.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Experimento RGB lanzado en background'",
                context="[Control:ExpRGB]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al lanzar experimento RGB: {e}'",
                context="[Control:ExpRGB]",
            )

    def run_experiment_rgbd(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/run_experiment_rgbd.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Experimento RGB-D lanzado en background'",
                context="[Control:ExpRGBD]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al lanzar experimento RGB-D: {e}'",
                context="[Control:ExpRGBD]",
            )

    def eval_last_experiment(self):
        bash_cmd = "cd ~/TFM/agarre_ros2_ws && ./scripts/eval_last_experiment.sh"
        try:
            subprocess.Popen(["bash", "-lc", bash_cmd])
            self.run_cmd(
                "echo 'Evaluación del último experimento lanzada en background'",
                context="[Control:ExpEval]",
            )
        except Exception as e:
            self.run_cmd(
                f"echo 'ERROR al evaluar último experimento: {e}'",
                context="[Control:ExpEval]",
            )

    # --------- Cierre ordenado de ROS ----------
    def shutdown_ros(self):
        if hasattr(self, "image_timer") and self.image_timer is not None:
            try:
                self.image_timer.stop()
            except Exception:
                pass

        if hasattr(self, "image_sub") and self.image_sub is not None and \
           hasattr(self, "ros_node") and self.ros_node is not None:
            try:
                self.ros_node.destroy_subscription(self.image_sub)
            except Exception:
                pass
            self.image_sub = None

        if self.executor is not None:
            try:
                if self.ros_node is not None:
                    self.executor.remove_node(self.ros_node)
            except Exception:
                pass
            try:
                self.executor.shutdown()
            except Exception:
                pass
            self.executor = None

        if hasattr(self, "ros_node") and self.ros_node is not None:
            try:
                self.ros_node.destroy_node()
            except Exception:
                pass
            self.ros_node = None

        try:
            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


class MainPanel(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Panel de control – Agarre inteligente (ROS 2 + Gazebo)")
        self.resize(1100, 700)

        central = QWidget()
        main_layout = QVBoxLayout()

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setLineWrapMode(QTextEdit.NoWrap)
        self.log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.config_tabs = QTabWidget()
        self.config_tabs.setTabPosition(QTabWidget.North)
        self.config_tabs.setMovable(False)

        self.real_cameras_tab = RealCamerasTab(run_cmd=self.run_command)
        self.synthetic_tab = SyntheticImagesTab(run_cmd=self.run_command)

        self.config_tabs.addTab(self.real_cameras_tab, "Cámaras reales")
        self.config_tabs.addTab(self.synthetic_tab, "Imágenes sintéticas")

        config_container = QWidget()
        config_layout = QVBoxLayout()
        config_layout.addWidget(self.config_tabs)
        config_container.setLayout(config_layout)

        self.control_tab = ControlTab(run_cmd=self.run_command)

        self.main_tabs = QTabWidget()
        self.main_tabs.setTabPosition(QTabWidget.North)
        self.main_tabs.setMovable(False)

        self.main_tabs.addTab(config_container, "Configuración")
        self.main_tabs.addTab(self.control_tab, "Control y experimentos")

        btn_close = QPushButton("Cerrar panel")
        btn_close.clicked.connect(self.confirm_close)

        main_layout.addWidget(self.main_tabs)
        main_layout.addWidget(QLabel("Salida / Log de comandos:"))
        main_layout.addWidget(self.log)
        main_layout.addWidget(btn_close, alignment=Qt.AlignRight)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def run_command(self, command: str, context: Optional[str] = None):
        prefix = f"{context} " if context else ""
        self.append_log(f"{prefix}$ {command}")

        try:
            completed = subprocess.run(
                ["bash", "-lc", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            output = completed.stdout
        except Exception as e:
            output = f"[ERROR al ejecutar comando: {e}]"

        if output:
            for line in output.splitlines():
                self.append_log(f"{prefix}{line}")

    def append_log(self, text: str):
        self.log.append(text)
        self.log.moveCursor(self.log.textCursor().End)

    def confirm_close(self):
        reply = QMessageBox.question(
            self,
            "Cerrar panel",
            "¿Seguro que quieres cerrar el panel?\n"
            "Se cerrará la interfaz, pero los procesos que ya estén lanzados "
            "seguirán ejecutándose salvo que los detengas explícitamente.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            try:
                self.control_tab.shutdown_ros()
            except Exception:
                pass
            QApplication.instance().quit()

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self,
            "Cerrar panel",
            "¿Seguro que quieres cerrar el panel?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            try:
                self.control_tab.shutdown_ros()
            except Exception:
                pass
            event.accept()
        else:
            event.ignore()


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    app = QApplication(sys.argv)
    window = MainPanel()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

