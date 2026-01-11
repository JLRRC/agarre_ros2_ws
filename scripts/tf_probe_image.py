#!/usr/bin/env python3
import argparse
import os
import sys
import time
from threading import Event
from typing import Optional, Tuple


def _bootstrap_ros_env() -> None:
    if os.environ.get("TF_PROBE_BOOTSTRAPPED") == "1":
        return
    ws_dir = os.environ.get("WS_DIR", "/home/laboratorio/TFM/agarre_ros2_ws")
    ros_setup = "/opt/ros/jazzy/setup.bash"
    ws_setup = os.path.join(ws_dir, "install", "setup.bash")
    cmd = [
        "bash",
        "-lc",
        "set -e; "
        f"source '{ros_setup}'; "
        f"if [ -f '{ws_setup}' ]; then source '{ws_setup}'; fi; "
        "export TF_PROBE_BOOTSTRAPPED=1; "
        f"exec python3 '{os.path.abspath(__file__)}' {' '.join(map(str, sys.argv[1:]))}",
    ]
    os.execv("/bin/bash", cmd)


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except Exception as exc:
    if "librcl_action" in str(exc) or "rclpy" in str(exc):
        _bootstrap_ros_env()
    raise


def _reliability_name(profile) -> str:
    value = getattr(profile, "reliability", None)
    if value is None:
        return "reliability=?"
    try:
        from rclpy.qos import ReliabilityPolicy

        if value == ReliabilityPolicy.RELIABLE:
            return "RELIABLE"
        if value == ReliabilityPolicy.BEST_EFFORT:
            return "BEST_EFFORT"
    except Exception:
        pass
    return str(value)


def _format_qos(profile) -> str:
    if profile is None:
        return "QoS=default"
    depth = getattr(profile, "depth", None)
    depth_txt = f"depth={depth}" if depth is not None else "depth=?"
    return f"{_reliability_name(profile)}@{depth_txt}"


class ImageProbe(Node):
    def __init__(self, topic: str, msg_type: str) -> None:
        super().__init__("tf_probe_image")
        self.topic = topic
        self.msg_type = msg_type
        self._count = 0
        self._last_wall = 0.0
        self._first_event = Event()
        self._last_info: Optional[str] = None
        self._sub = self._create_subscription()

    def _create_subscription(self):
        normalized = (self.msg_type or "image").strip().lower()
        if "compressed" in normalized:
            from sensor_msgs.msg import CompressedImage as MsgType
        else:
            from sensor_msgs.msg import Image as MsgType
        return self.create_subscription(
            MsgType,
            self.topic,
            self._on_msg,
            qos_profile_sensor_data,
        )

    def _on_msg(self, msg) -> None:
        self._count += 1
        self._last_wall = time.time()
        if self._count == 1:
            self._last_info = self._format_msg_info(msg)
            self._first_event.set()

    def _format_msg_info(self, msg) -> str:
        if hasattr(msg, "width") and hasattr(msg, "height"):
            enc = getattr(msg, "encoding", "?")
            return f"{msg.width}x{msg.height} {enc}"
        if hasattr(msg, "format"):
            return f"compressed format={msg.format}"
        return "unknown image format"

    def wait_first(self, timeout_sec: float) -> bool:
        if timeout_sec < 0:
            self._first_event.wait()
            return True
        return self._first_event.wait(timeout=timeout_sec)

    def has_first(self) -> bool:
        return self._first_event.is_set()

    def status(self) -> Tuple[int, float, Optional[str]]:
        count = self._count
        age = time.time() - self._last_wall if self._last_wall else float("inf")
        return count, age, self._last_info


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe image topic and wait for first frame.")
    parser.add_argument("--topic", type=str, default="/camera_overhead/image")
    parser.add_argument("--msg-type", type=str, default="image", help="image or compressed")
    parser.add_argument("--wait-sec", type=float, default=10.0, help="Timeout for first frame (0 = infinite)")
    parser.add_argument("--report-sec", type=float, default=1.0, help="Status log interval (s)")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = ImageProbe(args.topic, args.msg_type)
    qos_summary = _format_qos(getattr(node._sub, "qos_profile", None))
    print(f"[IMG_PROBE] Subscribed to {args.topic} ({qos_summary}) type={args.msg_type}")
    sys.stdout.flush()

    deadline = None if args.wait_sec <= 0 else time.monotonic() + max(0.1, args.wait_sec)
    next_report = time.monotonic() + max(0.1, args.report_sec)
    ok = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.has_first():
                ok = True
                break
            now = time.monotonic()
            if now >= next_report:
                count, age, _ = node.status()
                age_txt = f"{age:.2f}s" if age != float("inf") else "inf"
                print(f"[IMG_PROBE] frames={count} last_age={age_txt}")
                sys.stdout.flush()
                next_report = now + max(0.1, args.report_sec)
            if deadline is not None and now >= deadline:
                break
    except KeyboardInterrupt:
        ok = False
    finally:
        count, age, info = node.status()
        age_txt = f"{age:.2f}s" if age != float("inf") else "inf"
        if ok:
            info_txt = info or "n/a"
            print(f"[IMG_PROBE] OK first frame received ({info_txt})")
        else:
            print(f"[IMG_PROBE] TIMEOUT no frames (frames={count} last_age={age_txt})")
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
