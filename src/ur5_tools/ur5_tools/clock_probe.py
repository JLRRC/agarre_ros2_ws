#!/usr/bin/env python3
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/clock_probe.py
# Summary: /clock monitor for monotonicity and sim time coherence.
"""Monitor /clock for monotonicity and sim time coherence."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rosgraph_msgs.msg import Clock


@dataclass
class ClockState:
    last_clock_ns: int | None = None
    last_msg_wall_ns: int | None = None
    last_warn_ns: int = 0
    msgs: int = 0


class ClockProbe(Node):
    """Probe /clock for monotonicity and drift."""

    def __init__(self) -> None:
        super().__init__("clock_probe")
        self.declare_parameter("warn_every_sec", 2.0)
        self.declare_parameter("max_drift_sec", 0.05)
        self.declare_parameter("stale_sec", 1.0)

        self._warn_every = float(self.get_parameter("warn_every_sec").value)
        self._max_drift = float(self.get_parameter("max_drift_sec").value)
        self._stale_sec = float(self.get_parameter("stale_sec").value)

        self._state = ClockState()
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._sub = self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self._timer = self.create_timer(0.5, self._on_timer)

        self.get_logger().info(
            f"Clock probe active (max_drift={self._max_drift:.3f}s, stale={self._stale_sec:.2f}s)"
        )
        if not bool(self.get_parameter("use_sim_time").value):
            self.get_logger().warn("use_sim_time is false for clock_probe")

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_clock(self, msg: Clock) -> None:
        clock_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        now_ns = self._now_ns()
        state = self._state

        if state.last_clock_ns is not None and clock_ns < state.last_clock_ns:
            delta = (state.last_clock_ns - clock_ns) * 1e-9
            self.get_logger().warn(f"/clock moved backwards by {delta:.6f}s")

        drift = abs(now_ns - clock_ns) * 1e-9
        if drift > self._max_drift:
            self.get_logger().warn(f"/clock drift vs node clock: {drift:.3f}s")

        state.last_clock_ns = clock_ns
        state.last_msg_wall_ns = now_ns
        state.msgs += 1

    def _on_timer(self) -> None:
        now_ns = self._now_ns()
        state = self._state
        if state.last_msg_wall_ns is None:
            self.get_logger().warn("No /clock messages received yet")
            return

        since = (now_ns - state.last_msg_wall_ns) * 1e-9
        if since > self._stale_sec:
            self.get_logger().warn(f"/clock stale for {since:.2f}s")
        elif now_ns - state.last_warn_ns >= int(self._warn_every * 1e9):
            self.get_logger().info(f"/clock ok, msgs={state.msgs}")
            state.last_warn_ns = now_ns


def main() -> None:
    rclpy.init()
    node = ClockProbe()
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
