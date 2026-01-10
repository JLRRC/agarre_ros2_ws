"""Node that bridges a grasp pose to MoveIt planning/execution for the UR5."""
from __future__ import annotations

import sys

try:
    from moveit.planning import MoveItPy, PlanningComponent  # type: ignore
except Exception as exc:  # pragma: no cover
    MoveItPy = None  # type: ignore
    PlanningComponent = None  # type: ignore
    _MOVEIT_PY_IMPORT_ERROR = exc
else:
    _MOVEIT_PY_IMPORT_ERROR = None

try:
    import moveit_commander  # type: ignore
    from moveit_commander.move_group import MoveGroupCommander  # type: ignore
    from moveit_commander.robot_trajectory import RobotTrajectory  # type: ignore
except Exception as exc:  # pragma: no cover
    moveit_commander = None  # type: ignore
    MoveGroupCommander = None  # type: ignore
    RobotTrajectory = None  # type: ignore
    _MOVEIT_COMMANDER_IMPORT_ERROR = exc
else:
    _MOVEIT_COMMANDER_IMPORT_ERROR = None
import threading
import time
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


class UR5MoveItBridge(Node):
    """Subscribes to grasp poses and drives MoveIt planning/execution."""

    def __init__(self) -> None:
        super().__init__("ur5_moveit_bridge")
        self._backend = None
        self._moveit_py = None
        self._planning_component = None
        self._move_group = None
        self._moveit_py_ready = False
        self._moveit_py_init_error = None
        if MoveItPy is not None and PlanningComponent is not None:
            self._backend = "moveit_py"
            self.get_logger().info("MoveItPy backend seleccionado; inicializando...")
        elif moveit_commander is not None:
            self._backend = "moveit_commander"
            self._move_group = MoveGroupCommander("manipulator")
            self._move_group.set_pose_reference_frame("base_link")
            self.get_logger().info("moveit_commander backend activo.")
        else:
            self.get_logger().error(
                "MoveIt Python no disponible. "
                f"moveit_py: {_MOVEIT_PY_IMPORT_ERROR} "
                f"moveit_commander: {_MOVEIT_COMMANDER_IMPORT_ERROR}"
            )
            raise RuntimeError("MoveIt Python no disponible")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._subscriptions = []
        for topic in ("/desired_grasp", "/grasp_pose"):
            self._subscriptions.append(
                self.create_subscription(PoseStamped, topic, self._pose_callback, 10)
            )
        if self._backend == "moveit_py":
            threading.Thread(target=self._init_moveit_py, daemon=True).start()
        self.get_logger().info("UR5 MoveIt bridge listo.")

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f"Pose recibida frame={msg.header.frame_id or 'n/a'}"
        )
        target = self._ensure_base_frame(msg)
        if target is None:
            return

        if self._backend == "moveit_py":
            self._plan_with_moveit_py(target)
        else:
            self._plan_with_moveit_commander(target)

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

    def _plan_with_moveit_py(self, target: PoseStamped) -> None:
        if not self._planning_component or not self._moveit_py:
            if self._moveit_py_init_error:
                self.get_logger().warning(
                    f"MoveItPy no inicializado: {self._moveit_py_init_error}"
                )
            else:
                self.get_logger().warning("MoveItPy inicializando; reintenta en unos segundos.")
            return
        try:
            self._planning_component.set_start_state_to_current_state()
            self._planning_component.set_goal_state(target, "tool0")
            plan = self._planning_component.plan()
        except Exception as exc:
            self.get_logger().warning(f"Planificación MoveItPy fallida: {exc}")
            return
        if plan is None:
            self.get_logger().warning("Planificación MoveItPy fallida (plan vacío).")
            return
        trajectory = getattr(plan, "trajectory", None)
        success = getattr(plan, "success", None)
        if success is False or trajectory is None:
            self.get_logger().warning("Planificación MoveItPy fallida (sin trayectoria).")
            return
        self.get_logger().info("Planificación MoveItPy OK.")
        try:
            result = self._moveit_py.execute(trajectory, controllers=[])
        except TypeError:
            result = self._moveit_py.execute(trajectory)
        if bool(result):
            self.get_logger().info("Ejecución MoveItPy completada.")
        else:
            self.get_logger().warning("Ejecución MoveItPy fallida.")

    def _init_moveit_py(self) -> None:
        try:
            moveit_share = get_package_share_directory("ur5_moveit_config")
            ur5_description_share = get_package_share_directory("ur5_description")
            srdf_path = Path(moveit_share) / "config" / "ur5.srdf"
            moveit_config = (
                MoveItConfigsBuilder("ur5_rg2", package_name="ur5_moveit_config")
                .robot_description(
                    file_path=str(Path(ur5_description_share) / "urdf" / "ur5.urdf.xacro"),
                    mappings={"ur_type": "ur5", "name": "ur5_rg2"},
                )
                .robot_description_semantic(file_path=str(srdf_path))
                .robot_description_kinematics(file_path=str(Path(moveit_share) / "config" / "kinematics.yaml"))
                .joint_limits(file_path=str(Path(moveit_share) / "config" / "joint_limits.yaml"))
                .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
                .trajectory_execution(file_path=str(Path(moveit_share) / "config" / "moveit_controllers.yaml"))
                .to_moveit_configs()
            )
            config_dict = moveit_config.to_dict()
            pipeline_names = config_dict.get("planning_pipelines", [])
            if isinstance(pipeline_names, list):
                config_dict["planning_pipelines"] = {
                    "pipeline_names": pipeline_names,
                    "namespace": "",
                }
                for pipeline in pipeline_names:
                    pipeline_cfg = config_dict.get(pipeline)
                    if (
                        isinstance(pipeline_cfg, dict)
                        and "planning_plugin" not in pipeline_cfg
                        and "planning_plugins" in pipeline_cfg
                        and isinstance(pipeline_cfg["planning_plugins"], list)
                        and pipeline_cfg["planning_plugins"]
                    ):
                        pipeline_cfg["planning_plugin"] = pipeline_cfg["planning_plugins"][0]
            self._moveit_py = MoveItPy(node_name="ur5_moveit_py", config_dict=config_dict)
            self._planning_component = PlanningComponent("manipulator", self._moveit_py)
            self._moveit_py_ready = True
            self.get_logger().info("MoveItPy backend activo.")
        except Exception as exc:
            self._moveit_py_init_error = exc
            self.get_logger().error(f"MoveItPy init fallida: {exc}")

    def _plan_with_moveit_commander(self, target: PoseStamped) -> None:
        if not self._move_group:
            self.get_logger().error("moveit_commander no inicializado.")
            return
        self._move_group.set_pose_target(target.pose)
        plan = self._move_group.plan()
        trajectory = self._extract_trajectory(plan)
        if trajectory is None:
            self.get_logger().warning("Planificación con MoveIt fallida.")
            self._move_group.clear_pose_targets()
            return

        self.get_logger().info("Planificación con MoveIt OK.")
        success = self._move_group.execute(trajectory, wait=True)
        if success:
            self.get_logger().info("Ejecución MoveIt completada.")
        else:
            self.get_logger().warning("Ejecución MoveIt fallida.")
        self._move_group.clear_pose_targets()

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
    if MoveItPy is None and moveit_commander is None:
        raise SystemExit("MoveIt Python no disponible. Instala ros-jazzy-moveit-py.")
    if moveit_commander is not None:
        moveit_commander.roscpp_initialize(sys.argv)
    node = UR5MoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UR5 MoveIt bridge detenido por usuario.")
    finally:
        node.destroy_node()
        if moveit_commander is not None:
            moveit_commander.roscpp_shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
