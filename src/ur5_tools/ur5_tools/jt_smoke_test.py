#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/jt_smoke_test.py
# Summary: Minimal JointTrajectory smoke test with joint state verification.
"""Publish a minimal JointTrajectory and verify joint state change."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@dataclass
class JointSnapshot:
    positions: Dict[str, float]
    stamp_ns: int


class JointTrajectorySmokeTest(Node):
    """Send a single-point JointTrajectory and confirm state changes."""

    def __init__(self) -> None:
        super().__init__("jt_smoke_test")
        self.declare_parameter("traj_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("joints", DEFAULT_JOINTS)
        self.declare_parameter("delta_rad", 0.2)
        self.declare_parameter("move_sec", 2.0)
        self.declare_parameter("timeout_sec", 5.0)
        self.declare_parameter("min_change_rad", 0.02)

        self._traj_topic = str(self.get_parameter("traj_topic").value)
        self._joints: List[str] = list(self.get_parameter("joints").value)
        self._delta = float(self.get_parameter("delta_rad").value)
        self._move_sec = float(self.get_parameter("move_sec").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._min_change = float(self.get_parameter("min_change_rad").value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._pub = self.create_publisher(JointTrajectory, self._traj_topic, qos)
        self._sub = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._timer = self.create_timer(0.1, self._on_timer)

        self._initial: JointSnapshot | None = None
        self._last_state: JointSnapshot | None = None
        self._sent_state: JointSnapshot | None = None
        self._sent = False
        self._sent_time: Time | None = None

        self.get_logger().info(
            f"JT smoke test ready (topic={self._traj_topic}, joints={len(self._joints)})"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            positions[name] = pos
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        snapshot = JointSnapshot(positions=positions, stamp_ns=stamp_ns)
        self._last_state = snapshot
        if self._initial is None and self._has_all_joints(snapshot):
            self._initial = snapshot

    def _has_all_joints(self, snapshot: JointSnapshot) -> bool:
        return all(joint in snapshot.positions for joint in self._joints)

    def _build_target(self, snapshot: JointSnapshot) -> List[float]:
        positions = [snapshot.positions[joint] for joint in self._joints]
        if positions:
            positions[0] += self._delta
        return positions

    def _publish_trajectory(self) -> None:
        if self._initial is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._joints)
        point = JointTrajectoryPoint()
        point.positions = self._build_target(self._initial)
        point.time_from_start.sec = int(self._move_sec)
        point.time_from_start.nanosec = int((self._move_sec % 1.0) * 1e9)
        traj.points = [point]
        self._pub.publish(traj)
        self._sent = True
        self._sent_time = self.get_clock().now()
        self._sent_state = self._last_state
        self.get_logger().info("JointTrajectory published")

    def _detect_movement(self) -> bool:
        baseline = self._sent_state or self._initial
        if baseline is None or self._last_state is None:
            return False
        if not self._has_all_joints(self._last_state) or not self._has_all_joints(baseline):
            return False
        for joint in self._joints:
            before = baseline.positions[joint]
            after = self._last_state.positions[joint]
            if abs(after - before) >= self._min_change:
                return True
        return False

    def _on_timer(self) -> None:
        if not self._sent:
            if self._initial is None:
                return
            self._publish_trajectory()
            return

        if self._detect_movement():
            self.get_logger().info("JointTrajectory executed (joint_states changed)")
            rclpy.shutdown()
            return

        if self._sent_time is not None:
            elapsed = (self.get_clock().now() - self._sent_time).nanoseconds * 1e-9
            if elapsed > self._timeout_sec:
                self.get_logger().warn("JointTrajectory timeout: no joint_states change")
                rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = JointTrajectorySmokeTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
