"""Node that bridges a grasp pose to MoveIt planning/execution for the UR5."""
from __future__ import annotations

import sys

import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot_trajectory import RobotTrajectory
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


class UR5MoveItBridge(Node):
    """Subscribes to grasp poses and drives MoveIt planning/execution."""

    def __init__(self) -> None:
        super().__init__("ur5_moveit_bridge")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = MoveGroupCommander("manipulator")
        self.move_group.set_pose_reference_frame("base_link")
        self.subscriptions = []
        for topic in ("/desired_grasp", "/grasp_pose"):
            self.subscriptions.append(
                self.create_subscription(PoseStamped, topic, self._pose_callback, 10)
            )
        self.get_logger().info("UR5 MoveIt bridge listo.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        target = self._ensure_base_frame(msg)
        if target is None:
            return

        self.move_group.set_pose_target(target.pose)
        plan = self.move_group.plan()
        trajectory = self._extract_trajectory(plan)
        if trajectory is None:
            self.get_logger().warning("Planificación con MoveIt fallida.")
            self.move_group.clear_pose_targets()
            return

        self.get_logger().info("Planificación con MoveIt OK.")
        success = self.move_group.execute(trajectory, wait=True)
        if success:
            self.get_logger().info("Ejecución MoveIt completada.")
        else:
            self.get_logger().warn("Ejecución MoveIt fallida.")
        self.move_group.clear_pose_targets()

    def _ensure_base_frame(self, msg: PoseStamped) -> PoseStamped | None:
        if msg.header.frame_id == "base_link":
            return msg

        try:
            now = Time()
            transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, now)
            return do_transform_pose(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(f"No se pudo transformar pose a base_link: {exc}")
            return None

    @staticmethod
    def _extract_trajectory(plan) -> RobotTrajectory | None:
        if plan is None:
            return None
        if isinstance(plan, tuple):
            outcome, trajectory = plan
            if outcome:
                return trajectory
            return None
        if isinstance(plan, RobotTrajectory):
            return plan
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    moveit_commander.roscpp_initialize(sys.argv)
    node = UR5MoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UR5 MoveIt bridge detenido por usuario.")
    finally:
        node.destroy_node()
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
