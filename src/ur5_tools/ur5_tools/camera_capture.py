#!/usr/bin/env python3
import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture')

        # Parámetros
        self.declare_parameter('output_dir', '/home/laboratorio/capturas_ur5')
        self.declare_parameter('topic', '/camera/color/image_raw')

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.callback,
            10
        )
        self.get_logger().info(
            f'Guardando imágenes del topic [{topic}] en [{output_dir}]'
        )

    def callback(self, msg: Image):
        # Convertir imagen ROS a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = os.path.join(self.output_dir, f'frame_{ts}.png')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Guardado {filename}')


def main():
    rclpy.init()
    node = CameraCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
