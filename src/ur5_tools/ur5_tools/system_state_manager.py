#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/system_state_manager.py
# Summary: Publishes a global system state for the UR5 stack.
"""Publish global system readiness state for the UR5 stack."""

from __future__ import annotations

import json
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock


class SystemStateManager(Node):
    """Track system readiness and publish /system_state + /system_diag."""

    def __init__(self) -> None:
        super().__init__("system_state_manager")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("model_name", "ur5_rg2")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("camera_topic", "/camera_overhead/image")
        self.declare_parameter("required_controllers", [
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "gripper_controller",
        ])
        self.declare_parameter("moveit_required", False)
        self.declare_parameter("moveit_service_names", [
            "/get_planning_scene",
            "/move_group/get_planning_scene",
        ])
        self.declare_parameter("clock_timeout_sec", 1.5)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("camera_timeout_sec", 1.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("controller_check_sec", 1.0)
        self.declare_parameter("state_publish_hz", 2.0)

        self._world_name = str(self.get_parameter("world_name").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._ee_frame = str(self.get_parameter("ee_frame").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        self._pose_topic = pose_topic or f"/world/{self._world_name}/pose/info"
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._required_controllers = list(self.get_parameter("required_controllers").value)
        self._moveit_required = bool(self.get_parameter("moveit_required").value)
        self._moveit_services = list(self.get_parameter("moveit_service_names").value)
        self._clock_timeout = float(self.get_parameter("clock_timeout_sec").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self._camera_timeout = float(self.get_parameter("camera_timeout_sec").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._controller_check_sec = float(self.get_parameter("controller_check_sec").value)
        self._state_hz = float(self.get_parameter("state_publish_hz").value)

        self._last_clock_wall = 0.0
        self._pose_last_wall = 0.0
        self._pose_model_seen = False
        self._camera_last_wall = 0.0
        self._camera_frames = 0
        self._controllers_state: Dict[str, str] = {}
        self._controllers_ready = False
        self._last_controller_check = 0.0
        self._controller_future = None
        self._moveit_ready = False
        self._ever_ready = False
        self._state = "BOOTING"
        self._reason = "boot"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state_pub = self.create_publisher(String, "/system_state", 10)
        self._diag_pub = self.create_publisher(String, "/system_diag", 10)

        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, self._pose_topic, self._on_pose_info, qos_profile_sensor_data)
        if self._camera_topic:
            self.create_subscription(Image, self._camera_topic, self._on_camera_image, qos_profile_sensor_data)

        self._controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        period = 1.0 / max(0.5, self._state_hz)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"SystemStateManager listo: pose={self._pose_topic} camera={self._camera_topic} "
            f"ee={self._ee_frame} controllers={len(self._required_controllers)} moveit={self._moveit_required}"
        )

    def _on_clock(self, _msg: Clock) -> None:
        self._last_clock_wall = time.time()

    def _on_pose_info(self, msg: TFMessage) -> None:
        now = time.time()
        found = False
        for tf in getattr(msg, "transforms", []):
            name = self._name_from_tf(tf)
            if not name:
                continue
            if name == self._model_name or name.startswith(f"{self._model_name}::"):
                found = True
                break
            if self._model_name and self._model_name in name:
                found = True
                break
        if found:
            self._pose_model_seen = True
        if getattr(msg, "transforms", []):
            self._pose_last_wall = now

    def _on_camera_image(self, _msg: Image) -> None:
        self._camera_last_wall = time.time()
        self._camera_frames += 1

    def _name_from_tf(self, tf: TransformStamped) -> str:
        child = getattr(tf, "child_frame_id", "") or ""
        if child:
            return child
        header = getattr(tf, "header", None)
        return getattr(header, "frame_id", "") if header else ""

    def _clock_ok(self) -> Tuple[bool, float]:
        if self._last_clock_wall <= 0.0:
            return False, float("inf")
        age = time.time() - self._last_clock_wall
        return age <= self._clock_timeout, age

    def _pose_ok(self) -> Tuple[bool, float]:
        if self._pose_last_wall <= 0.0:
            return False, float("inf")
        age = time.time() - self._pose_last_wall
        return self._pose_model_seen and age <= self._pose_timeout, age

    def _camera_ok(self) -> Tuple[bool, float]:
        if self._camera_last_wall <= 0.0:
            return False, float("inf")
        age = time.time() - self._camera_last_wall
        return age <= self._camera_timeout, age

    def _update_controllers(self) -> None:
        now = time.time()
        if (now - self._last_controller_check) < self._controller_check_sec:
            return
        self._last_controller_check = now
        if not self._controller_client.service_is_ready():
            self._controllers_ready = False
            return
        if self._controller_future is not None and not self._controller_future.done():
            return
        req = ListControllers.Request()
        self._controller_future = self._controller_client.call_async(req)

    def _consume_controllers(self) -> None:
        future = self._controller_future
        if future is None or not future.done():
            return
        self._controller_future = None
        try:
            result = future.result()
        except Exception:
            self._controllers_ready = False
            return
        states: Dict[str, str] = {}
        for ctrl in result.controller:
            states[str(ctrl.name)] = str(ctrl.state)
        self._controllers_state = states
        if not self._required_controllers:
            self._controllers_ready = True
            return
        self._controllers_ready = all(states.get(name) == "active" for name in self._required_controllers)

    def _tf_ok(self) -> Tuple[bool, str]:
        timeout = Duration(seconds=self._tf_timeout)
        try:
            if not self._tf_buffer.can_transform(
                self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
            ):
                return False, "world->base"
            if self._ee_frame:
                if not self._tf_buffer.can_transform(
                    self._base_frame, self._ee_frame, rclpy.time.Time(), timeout=timeout
                ):
                    return False, "base->ee"
        except Exception:
            return False, "tf_error"
        return True, "ok"

    def _moveit_ok(self) -> bool:
        if not self._moveit_required:
            return True
        try:
            services = self.get_service_names_and_types()
        except Exception:
            return False
        for name, _types in services:
            if name in self._moveit_services:
                return True
            if name.endswith("/get_planning_scene"):
                return True
        return False

    def _set_state(self, state: str, reason: str) -> None:
        if state == self._state and reason == self._reason:
            return
        self._state = state
        self._reason = reason
        if state == "READY":
            self._ever_ready = True
        self.get_logger().info(f"STATE {state} ({reason})")

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._state))
        diag = {"reason": self._reason}
        self._diag_pub.publish(String(data=json.dumps(diag)))

    def _tick(self) -> None:
        self._update_controllers()
        self._consume_controllers()
        clock_ok, clock_age = self._clock_ok()
        pose_ok, pose_age = self._pose_ok()
        camera_ok, camera_age = self._camera_ok()
        tf_ok, tf_reason = self._tf_ok()
        self._moveit_ready = self._moveit_ok()

        if not clock_ok:
            next_state = "WAITING_GAZEBO"
            next_reason = f"/clock age={clock_age:.2f}s"
        elif not pose_ok:
            next_state = "WAITING_BRIDGE"
            next_reason = f"pose/info age={pose_age:.2f}s model={self._pose_model_seen}"
        elif not self._controllers_ready:
            missing = [c for c in self._required_controllers if self._controllers_state.get(c) != "active"]
            next_state = "WAITING_CONTROLLERS"
            next_reason = "controllers activos" if not missing else f"controllers missing: {', '.join(missing)}"
        elif not tf_ok:
            next_state = "WAITING_TF"
            next_reason = f"tf={tf_reason}"
        elif not camera_ok:
            next_state = "WAITING_CAMERA"
            next_reason = f"camera age={camera_age:.2f}s"
        elif self._moveit_required and not self._moveit_ready:
            next_state = "WAITING_MOVEIT"
            next_reason = "move_group no listo"
        else:
            next_state = "READY"
            next_reason = "Sistema listo"

        if self._ever_ready and next_state != "READY":
            self._set_state("ERROR", f"drop: {next_reason}")
        else:
            self._set_state(next_state, next_reason)
        self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemStateManager()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
