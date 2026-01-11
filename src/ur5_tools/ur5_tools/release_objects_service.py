#!/usr/bin/env python3
"""ROS 2 service to detach drop objects from the gripper in Gazebo."""
from __future__ import annotations

import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

DEFAULT_OBJECTS = [
    "cubo_rojo",
    "cilindro_verde",
    "caja_azul",
    "pieza_pick_mesa",
]

class ReleaseObjectsService(Node):
    def __init__(self) -> None:
        super().__init__("release_objects_service")
        self.declare_parameter("gripper_prefix", "/gripper")
        self.declare_parameter("object_names", DEFAULT_OBJECTS)
        self.declare_parameter("attempts", 1)
        self.declare_parameter("attempt_sleep", 0.1)
        self._service = self.create_service(Trigger, "release_objects", self._handle_release)
        self._pubs: Dict[str, object] = {}

    def _handle_release(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        prefix = str(self.get_parameter("gripper_prefix").value)
        names = list(self.get_parameter("object_names").value or [])
        if not names:
            response.success = False
            response.message = "object_names vacío"
            return response

        attempts = int(self.get_parameter("attempts").value or 1)
        sleep_s = float(self.get_parameter("attempt_sleep").value or 0.0)
        attempts = max(1, attempts)
        for _idx in range(attempts):
            for name in names:
                topic = f"{prefix}/{name}/detach"
                pub = self._pubs.get(topic)
                if pub is None:
                    pub = self.create_publisher(Empty, topic, 10)
                    self._pubs[topic] = pub
                pub.publish(Empty())
            if sleep_s > 0.0:
                time.sleep(sleep_s)
        response.success = True
        response.message = f"detach published ({attempts}x, {len(names)} objs)"
        return response


def main() -> None:
    rclpy.init()
    node = ReleaseObjectsService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
