#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/controller_bootstrap.py
# Summary: Load/configure/activate ros2_control controllers once using controller_manager services.
"""Bootstrap ros2_control controllers without respawning active ones."""

from __future__ import annotations

import time
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)


class ControllerBootstrap(Node):
    """One-shot controller loader/activator."""

    def __init__(self) -> None:
        super().__init__("controller_bootstrap")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("required_controllers", [
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "gripper_controller",
        ])
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("clock_timeout_sec", 10.0)
        self.declare_parameter("service_timeout_sec", 5.0)

        self._cm = str(self.get_parameter("controller_manager").value)
        self._required = list(self.get_parameter("required_controllers").value)
        self._wait_clock = bool(self.get_parameter("wait_for_clock").value)
        self._clock_timeout = float(self.get_parameter("clock_timeout_sec").value)
        self._service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self._last_clock_wall = 0.0

        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)

        self._list_client = self.create_client(ListControllers, f"{self._cm}/list_controllers")
        self._load_client = self.create_client(LoadController, f"{self._cm}/load_controller")
        self._configure_client = self.create_client(ConfigureController, f"{self._cm}/configure_controller")
        self._switch_client = self.create_client(SwitchController, f"{self._cm}/switch_controller")

    def _on_clock(self, _msg: Clock) -> None:
        self._last_clock_wall = time.time()

    def _clock_ok(self) -> bool:
        if not self._wait_clock:
            return True
        if self._last_clock_wall <= 0.0:
            return False
        return (time.time() - self._last_clock_wall) <= 1.5

    def _wait_for_services(self) -> bool:
        deadline = time.time() + self._service_timeout
        while time.time() < deadline:
            if (
                self._list_client.service_is_ready()
                and self._load_client.service_is_ready()
                and self._configure_client.service_is_ready()
                and self._switch_client.service_is_ready()
            ):
                return True
            time.sleep(0.2)
        return False

    def _call(self, client, req, timeout: float) -> bool:
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return False
        try:
            result = future.result()
            if hasattr(result, "ok"):
                return bool(result.ok)
            if hasattr(result, "success"):
                return bool(result.success)
            return bool(result)
        except Exception:
            return False

    def _list_controllers(self) -> Dict[str, str]:
        req = ListControllers.Request()
        future = self._list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return {}
        try:
            resp = future.result()
        except Exception:
            return {}
        states = {}
        for ctrl in resp.controller:
            states[str(ctrl.name)] = str(ctrl.state)
        return states

    def _activate(self, name: str) -> bool:
        req = SwitchController.Request()
        req.activate_controllers = [name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 2
        return self._call(self._switch_client, req, 3.0)

    def _configure(self, name: str) -> bool:
        req = ConfigureController.Request()
        req.name = name
        return self._call(self._configure_client, req, 2.0)

    def _load(self, name: str) -> bool:
        req = LoadController.Request()
        req.name = name
        return self._call(self._load_client, req, 2.0)

    def run_once(self) -> None:
        start = time.time()
        while self._wait_clock and not self._clock_ok():
            if (time.time() - start) > self._clock_timeout:
                self.get_logger().error("/clock no disponible; abortando bootstrap")
                return
            time.sleep(0.2)

        if not self._wait_for_services():
            self.get_logger().error("controller_manager services no disponibles")
            return

        state_map = self._list_controllers()
        if not self._required:
            self.get_logger().info("Sin controladores requeridos; bootstrap omitido.")
            return

        for name in self._required:
            state = state_map.get(name)
            if state == "active":
                self.get_logger().info(f"[CTRL] {name} ya activo; skip")
                continue
            if state is None:
                self.get_logger().info(f"[CTRL] {name} no cargado; cargando")
                if not self._load(name):
                    self.get_logger().error(f"[CTRL] load failed: {name}")
                    continue
                if not self._configure(name):
                    self.get_logger().error(f"[CTRL] configure failed: {name}")
                    continue
            elif state in ("unconfigured", "inactive", "configured"):
                if state == "unconfigured":
                    if not self._configure(name):
                        self.get_logger().error(f"[CTRL] configure failed: {name}")
                        continue
            else:
                self.get_logger().warn(f"[CTRL] estado inesperado {name}: {state}")
            if not self._activate(name):
                self.get_logger().error(f"[CTRL] activate failed: {name}")
                continue
            self.get_logger().info(f"[CTRL] {name} activo")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerBootstrap()
    try:
        node.run_once()
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
