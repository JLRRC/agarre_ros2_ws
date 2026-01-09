from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node

from .panel_config import BASE_FRAME
from .panel_utils import get_tf_helper, transform_point_to_frame, yaw_from_quaternion
from ur5_qt_panel.srv import SelectionTf


class SelectionTfNode(Node):
    def __init__(self):
        super().__init__("selection_tf_service")
        self.srv = self.create_service(SelectionTf, "selection_tf", self.handle_selection)
        self._helper = get_tf_helper()

    def handle_selection(self, request: SelectionTf.Request, response: SelectionTf.Response) -> SelectionTf.Response:
        if not self._helper:
            response.ok = False
            response.target_source = "DEMO"
            response.message = "TF helper unavailable"
            return response

        world_point = request.world_point
        source_frame = request.source_frame or "world"
        base_candidates = [BASE_FRAME or "base_link", "base_link", "base"]
        chosen: Optional[Tuple[str, Tuple[float, float, float], object]] = None

        for base_frame in base_candidates:
            if not base_frame:
                continue
            coords, transform = transform_point_to_frame(
                (world_point.x, world_point.y, world_point.z), base_frame, source_frame
            )
            if coords and transform:
                chosen = (base_frame, coords, transform)
                break

        if not chosen:
            response.ok = False
            response.target_source = "DEMO"
            response.message = f"TF lookup {source_frame}->{base_candidates} failed"
            return response

        base_frame, coords, transform = chosen
        bx, by, bz = coords
        planar_dist = math.hypot(bx, by)
        z_ok = 0.0 <= bz <= 2.0
        via_tf = planar_dist <= 2.0 and z_ok

        response.selected_world = Point(x=world_point.x, y=world_point.y, z=world_point.z)
        response.selected_base = Point(x=bx, y=by, z=bz)
        translation = transform.transform.translation
        response.tf_translation = Vector3(x=translation.x, y=translation.y, z=translation.z)
        response.tf_yaw = yaw_from_quaternion(transform.transform.rotation)
        response.target_source = "SELECTION" if via_tf else "DEMO"
        response.ok = via_tf
        response.message = "TF OK" if via_tf else "Selection outside reach (fallback demo)"
        return response


def main():
    rclpy.init()
    node = SelectionTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
