#!/usr/bin/env python3
import argparse
import math
import sys
from typing import Optional, Tuple

import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class TFProbe(Node):
    def __init__(self, obj_frame: str, obj_xyz: Tuple[float, float, float]):
        super().__init__('tf_probe')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obj_frame = obj_frame
        self.obj_xyz = obj_xyz
        self.tf_count = 0
        self.tf_static_count = 0
        self.base_frame: Optional[str] = None
        self.ee_frame: Optional[str] = None
        self.frames = set()
        self.robot_frames = set()
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 10)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_callback, 10)
        self.create_timer(1.0, self._timer_cb)

    def _tf_callback(self, msg: TFMessage):
        self.tf_count += len(msg.transforms)

    def _tf_static_callback(self, msg: TFMessage):
        self.tf_static_count += len(msg.transforms)

    def _timer_cb(self):
        self._refresh_frames()
        self._discover_base_and_ee()
        obj_pose = self._make_pose(self.obj_frame, self.obj_xyz)
        obj_base = self._transform_pose(obj_pose, self.base_frame)
        tcp_base = self._lookup_frame_pose(self.base_frame, self.ee_frame)
        tcp_world = self._lookup_frame_pose(self.obj_frame, self.ee_frame)
        world_msg = f"{self.obj_frame}"
        base_msg = f"{self.base_frame or 'n/a'}"
        ee_msg = f"{self.ee_frame or 'n/a'}"
        dist_base = self._distance(obj_base, tcp_base)
        dist_world = self._distance(obj_pose, tcp_world)
        print(
            f"[PROBE] tf_counts tf={self.tf_count} static={self.tf_static_count} | "
            f"base={base_msg} ee={ee_msg} | "
            f"obj_world={(self.obj_frame, *self.obj_xyz)} | "
            f"obj_base={self._format_pose(obj_base)} | "
            f"tcp_base={self._format_pose(tcp_base)} dist_base={dist_base:.3f} | "
            f"tcp_world={self._format_pose(tcp_world)} dist_world={dist_world:.3f}"
        )
        sys.stdout.flush()

    def _refresh_frames(self):
        try:
            yaml_text = self.tf_buffer.all_frames_as_yaml()
            if not yaml_text:
                return
            data = yaml.safe_load(yaml_text) or {}
            transforms = data.get('transforms', [])
            frames = set()
            for entry in transforms:
                if frame := entry.get('child_frame_id'):
                    frames.add(frame)
                if parent := entry.get('frame_id'):
                    frames.add(parent)
            self.frames = frames
        except Exception:
            pass

    def _discover_base_and_ee(self):
        if not self.frames:
            return
        world_frame = self.obj_frame
        robot_keywords = ('wrist', 'tool', 'tcp', 'ee', 'flange', 'shoulder', 'elbow', 'rg2')
        candidate_frames = [f for f in self.frames if any(k in f.lower() for k in robot_keywords)]
        if self.base_frame and self._can_transform(world_frame, self.base_frame):
            pass
        else:
            for candidate in ['base_link', 'base', 'ur5_base_link']:
                if not candidate in self.frames and candidate != 'base':
                    continue
                if self._can_transform(world_frame, candidate) and any(
                    self._can_transform(candidate, other) for other in candidate_frames
                ):
                    self.base_frame = candidate
                    break
        if not self.base_frame:
            for candidate in ['base_link', 'base', 'ur5_base_link']:
                if candidate in self.frames and self._can_transform(world_frame, candidate):
                    self.base_frame = candidate
                    break
        if not self.base_frame:
            return
        if self.ee_frame and self._can_transform(self.base_frame, self.ee_frame):
            return
        ee_candidates = []
        keepers = ('tool0', 'tcp', 'ee_link', 'flange', 'wrist_3_link', 'ft_frame')
        for frame in self.frames:
            lframe = frame.lower()
            if frame == self.base_frame or frame == self.obj_frame:
                continue
            if frame in keepers or any(k in lframe for k in keepers):
                ee_candidates.append(frame)
        if not ee_candidates:
            leaves = self._leaf_frames()
            ee_candidates.extend(leaves)
        for candidate in ee_candidates:
            if self._can_transform(self.base_frame, candidate):
                self.ee_frame = candidate
                break

    def _leaf_frames(self):
        parents = set()
        children = set()
        try:
            yaml_text = self.tf_buffer.all_frames_as_yaml()
            data = yaml.safe_load(yaml_text) or {}
            for entry in data.get('transforms', []):
                parent = entry.get('frame_id')
                child = entry.get('child_frame_id')
                if parent:
                    parents.add(parent)
                if child:
                    children.add(child)
            leaves = [f for f in children if f not in parents]
            return leaves
        except Exception:
            return []

    def _can_transform(self, target, source, timeout_sec=0.2) -> bool:
        if not target or not source:
            return False
        try:
            return self.tf_buffer.can_transform(target, source, Time(), timeout=Duration(seconds=timeout_sec))
        except TransformException:
            return False

    def _transform_pose(self, pose: PoseStamped, target_frame: Optional[str]) -> Optional[PoseStamped]:
        if not target_frame:
            return None
        try:
            return self.tf_buffer.transform(pose, target_frame, timeout=Duration(seconds=0.2))
        except TransformException:
            return None

    def _lookup_frame_pose(self, base_frame: Optional[str], frame: Optional[str]) -> Optional[PoseStamped]:
        if not base_frame or not frame:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, frame, Time(), timeout=Duration(seconds=0.2))
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException:
            return None

    def _make_pose(self, frame: str, xyz: Tuple[float, float, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        pose.pose.orientation.w = 1.0
        return pose

    def _distance(self, a: Optional[PoseStamped], b: Optional[PoseStamped]) -> float:
        if not a or not b:
            return float('nan')
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.hypot(math.hypot(dx, dy), dz)

    def _format_pose(self, pose: Optional[PoseStamped]) -> str:
        if not pose:
            return 'n/a'
        p = pose.pose.position
        return f"({p.x:.3f},{p.y:.3f},{p.z:.3f})"


def main():
    parser = argparse.ArgumentParser(description="TF probe for UR5")
    parser.add_argument("--obj-x", type=float, default=0.5)
    parser.add_argument("--obj-y", type=float, default=0.0)
    parser.add_argument("--obj-z", type=float, default=0.2)
    parser.add_argument("--obj-frame", type=str, default="world")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = TFProbe(args.obj_frame, (args.obj_x, args.obj_y, args.obj_z))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
