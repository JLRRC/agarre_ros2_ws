# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/grasp_pose_publisher/grasp_pose_publisher/grasp_pose_node.py
# Summary: Subscribes to overhead camera and publishes a dummy grasp pose.
"""ROS 2 node that publishes a placeholder grasp pose from camera input."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge


class GraspPoseNode(Node):
    """
    Nodo ROS 2 muy sencillo que:

    - Se suscribe a /camera_overhead/image (sensor_msgs/msg/Image),
      que viene de Gazebo a través de ros_gz_bridge.
    - Cada vez que recibe una imagen, publica una pose de agarre "dummy"
      en /grasp_pose (geometry_msgs/msg/PoseStamped).

    Más adelante, aquí se integrará el modelo entrenado de Cornell.
    """

    def __init__(self):
        super().__init__('grasp_pose_node')

        self.bridge = CvBridge()

        # Suscripción a la cámara cenital con QoS de sensor
        self.image_sub = self.create_subscription(
            Image,
            '/camera_overhead/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publicador de la pose de agarre
        self.grasp_pub = self.create_publisher(
            PoseStamped,
            'grasp_pose',
            10
        )

        self.get_logger().info(
            'GraspPoseNode inicializado. Esperando imágenes en /camera_overhead/image...'
        )

        self._frame_count = 0

    def image_callback(self, msg: Image):
        self._frame_count += 1

        # Convertimos a OpenCV solo para comprobar que la imagen se recibe bien.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo la imagen: {e}')
            return

        height, width, _ = cv_image.shape

        if self._frame_count % 30 == 0:
            # Log cada 30 frames para no saturar
            self.get_logger().info(f'Imagen recibida: {width}x{height}')

        # Pose "dummy" cerca del cilindro verde
        pose_msg = PoseStamped()

        # Timestamp (copiamos el de la imagen)
        pose_msg.header.stamp = Time(
            sec=msg.header.stamp.sec,
            nanosec=msg.header.stamp.nanosec
        )
        pose_msg.header.frame_id = 'world'

        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.20
        pose_msg.pose.position.z = 0.82

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.grasp_pub.publish(pose_msg)

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'Publicado grasp_pose: '
                f'[{pose_msg.pose.position.x:.2f}, '
                f'{pose_msg.pose.position.y:.2f}, '
                f'{pose_msg.pose.position.z:.2f}]'
            )


def main(args=None):
    """Entry point for the grasp pose node."""
    rclpy.init(args=args)
    node = GraspPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
