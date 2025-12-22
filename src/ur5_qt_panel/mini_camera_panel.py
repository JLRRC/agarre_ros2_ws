#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/mini_camera_panel.py
# Summary: Mini Qt panel that displays multiple camera topics.
"""Minimal Qt panel for visualizing multiple camera topics."""
import sys
import threading

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QGridLayout,
    QVBoxLayout,
    QPushButton,
    QHBoxLayout,
)


# QLabel que se puede actualizar desde callbacks ROS usando señales
class CameraLabel(QLabel):
    """QLabel that can be updated from ROS callbacks via Qt signals."""
    image_signal = pyqtSignal(QImage)

    def __init__(self, topic_name: str):
        super().__init__()
        self.topic_name = topic_name
        self.setText(f"Esperando imágenes de\n{topic_name}")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background-color: #404040; color: white;")
        self.image_signal.connect(self._on_new_image)

    def _on_new_image(self, qimg: QImage):
        pixmap = QPixmap.fromImage(qimg)
        pixmap = pixmap.scaled(
            self.width(),
            self.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.setPixmap(pixmap)
        self.setStyleSheet("background-color: black;")  # ya hay imagen


# Nodo ROS 2 que se suscribe a todas las cámaras
class CamerasNode(Node):
    """ROS 2 node that subscribes to camera image topics."""
    def __init__(self, labels_by_topic):
        super().__init__("mini_camera_panel")
        self.bridge = CvBridge()
        self.labels_by_topic = labels_by_topic
        # OJO: no usar self.subscriptions (propiedad interna de rclpy.Node)
        self._subscriptions = []

        for topic, label in labels_by_topic.items():
            self.get_logger().info(f"Suscribiéndose a {topic}")
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                10,
            )
            self._subscriptions.append(sub)

    def image_callback(self, msg: Image, topic: str):
        label = self.labels_by_topic.get(topic)
        if label is None:
            return

        try:
            # Convertir sensor_msgs/Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qimg = QImage(
                cv_image.data,
                w,
                h,
                bytes_per_line,
                QImage.Format_RGB888,
            )

            # Emitimos señal para actualizar el QLabel en el hilo de GUI
            label.image_signal.emit(qimg)

        except Exception as e:
            self.get_logger().warn(f"Error procesando imagen de {topic}: {e}")


# Ventana principal del mini-panel
class MiniCameraPanel(QWidget):
    """Main widget that lays out multiple camera labels."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mini Camera Panel – Comprobación cámaras Gazebo/ROS2")
        self.resize(1200, 800)

        # Crear los labels para cada cámara
        self.label_overhead = CameraLabel("/camera_overhead/image")
        self.label_north = CameraLabel("/camera_north/image")
        self.label_south = CameraLabel("/camera_south/image")
        self.label_east = CameraLabel("/camera_east/image")
        self.label_west = CameraLabel("/camera_west/image")

        # Layout en grid (2 filas x 3 columnas, dejando hueco si hace falta)
        grid = QGridLayout()
        grid.addWidget(self.label_overhead, 0, 0, 1, 3)  # cenital ocupa toda la fila

        grid.addWidget(self.label_north, 1, 0)
        grid.addWidget(self.label_east, 1, 1)
        grid.addWidget(self.label_west, 1, 2)

        # Ponemos la south en la fila de abajo
        grid.addWidget(self.label_south, 2, 0, 1, 3)

        # Botón de cierre con confirmación
        btn_close = QPushButton("Cerrar mini panel")
        btn_close.clicked.connect(self.confirm_close)

        footer = QHBoxLayout()
        footer.addStretch()
        footer.addWidget(btn_close)
        footer.addStretch()

        main_layout = QVBoxLayout()
        main_layout.addLayout(grid)
        main_layout.addLayout(footer)

        self.setLayout(main_layout)

    def confirm_close(self):
        from PyQt5.QtWidgets import QMessageBox

        reply = QMessageBox.question(
            self,
            "Cerrar mini panel",
            "¿Seguro que quieres cerrar el mini panel de cámaras?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            QApplication.instance().quit()


def ros_spin_thread(node: CamerasNode):
    """Spin a ROS 2 node in a background thread."""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    """Entry point for the mini camera panel."""
    # Iniciar ROS 2
    rclpy.init(args=None)

    # Iniciar Qt
    app = QApplication(sys.argv)
    window = MiniCameraPanel()
    window.show()

    # Mapear tópicos -> labels
    labels_by_topic = {
        "/camera_overhead/image": window.label_overhead,
        "/camera_north/image": window.label_north,
        "/camera_south/image": window.label_south,
        "/camera_east/image": window.label_east,
        "/camera_west/image": window.label_west,
    }

    # Crear nodo ROS y lanzarlo en un hilo aparte
    node = CamerasNode(labels_by_topic)
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    # Ejecutar bucle Qt
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
