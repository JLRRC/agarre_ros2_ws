#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture_overhead")
        self.bridge = CvBridge()
        self.frame_saved = False

        # Nos suscribimos al tópico de la cámara cenital
        self.sub = self.create_subscription(
            Image,
            "/camera_overhead/image",
            self.callback,
            10,
        )

    def callback(self, msg: Image):
        if self.frame_saved:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out_path = "/tmp/overhead.png"
            cv2.imwrite(out_path, cv_img)
            self.get_logger().info(f"Frame guardado en {out_path}")
            self.frame_saved = True
        except Exception as e:
            self.get_logger().error(f"Error guardando frame: {e}")
        finally:
            # Cerramos el nodo después del primer frame
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CaptureNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
