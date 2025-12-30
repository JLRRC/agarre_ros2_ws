#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/fake_cameras.py
# Summary: ROS 2 node that publishes synthetic camera images.
"""ROS 2 node that publishes synthetic camera images for testing."""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FakeCamerasNode(Node):
    """Publish synthetic RGB images on configured camera topics."""

    def __init__(self):
        super().__init__('fake_cameras')

        # QoS similar al de imagen (fiable, profundidad 1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores en los mismos tópicos que usamos en el panel
        self.pub_color = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            qos
        )
        self.pub_overhead = self.create_publisher(
            Image,
            '/camera_overhead/image',
            qos
        )

        # Temporizador a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Para animar un poco la imagen sintética
        self.start_time = time.time()

        self.get_logger().info('FakeCamerasNode iniciado. '
                               'Publicando imágenes sintéticas en '
                               '/camera/color/image_raw y /camera_overhead/image')

    def _make_image(self, width: int, height: int, t: float, mode: str) -> Image:
        """
        Genera una imagen RGB8 sintética.

        - mode='color': gradiente horizontal con banda que se mueve
        - mode='overhead': cuadrícula simple
        """
        img = np.zeros((height, width, 3), dtype=np.uint8)

        if mode == 'color':
            # Gradiente horizontal
            x = np.linspace(0, 255, width, dtype=np.uint8)
            grad = np.tile(x, (height, 1))
            img[:, :, 0] = grad  # canal R
            img[:, :, 1] = np.roll(grad, int(t * 20) % width, axis=1)  # canal G desplazado
            img[:, :, 2] = 128  # canal B fijo
        else:
            # Cuadrícula simple
            step = 40
            for y in range(0, height, step):
                img[y:y+2, :, :] = 255
            for x in range(0, width, step):
                img[:, x:x+2, :] = 255

        msg = Image()
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = img.tobytes()
        return msg

    def timer_callback(self):
        t = time.time() - self.start_time

        # Cámara "color" sintética
        color_img = self._make_image(640, 480, t, mode='color')
        self.pub_color.publish(color_img)

        # Cámara cenital sintética
        overhead_img = self._make_image(640, 480, t, mode='overhead')
        self.pub_overhead.publish(overhead_img)


def main(args=None):
    """Entry point for the fake cameras node."""
    rclpy.init(args=args)
    node = FakeCamerasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
