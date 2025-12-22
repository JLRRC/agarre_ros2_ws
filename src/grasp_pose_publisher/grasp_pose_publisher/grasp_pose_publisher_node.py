# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/grasp_pose_publisher/grasp_pose_publisher/grasp_pose_publisher_node.py
# Summary: Publishes a fixed grasp pose at a steady rate.
"""ROS 2 node that publishes a fixed grasp pose at a steady rate."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GraspPosePublisher(Node):
    """
    Nodo simple que publica una pose de agarre fija en /desired_grasp.

    Más adelante:
    - Cambiar las coordenadas por la salida real de tu modelo de deep learning.
    - Ajustar frame_id al frame de cámara o de la escena.
    """

    def __init__(self):
        super().__init__('grasp_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'desired_grasp', 10)
        # Publicamos a 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Nodo grasp_pose_publisher inicializado.')

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Usamos el frame base del brazo UR (ajustable según lo que dé la sim)
        msg.header.frame_id = 'base_link'

        # EJEMPLO: una pose "frontal" delante del robot
        msg.pose.position.x = 0.4  # 40 cm delante
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.3  # 30 cm por encima del plano base

        # Orientación identidad (sin rotación) -> mano apuntando "hacia abajo"
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().debug('Publicando pose de agarre en /desired_grasp')


def main(args=None):
    """Entry point for the fixed grasp pose publisher."""
    rclpy.init(args=args)
    node = GraspPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
