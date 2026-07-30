#!/usr/bin/env python3

import argparse
import json
import os
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


STATE_FIELDS = (
    "state",
    "ready",
    "reason",
    "control_enabled",
    "control_mode",
    "gear",
    "engaged",
)


class RuntimeStateWatch(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("autoracer_rc_runtime_state_watch")
        self._output_path = output_path
        self._temporary_path = output_path.with_name(
            f".{output_path.name}.{os.getpid()}.tmp"
        )
        self._last_payload = ""
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/system/race_runtime/state",
            self._on_state,
            qos,
        )

    def _on_state(self, message: String) -> None:
        try:
            document = json.loads(message.data)
            snapshot = {name: document.get(name) for name in STATE_FIELDS}
            payload = json.dumps(snapshot, sort_keys=True, separators=(",", ":"))
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"invalid race runtime state: {error}")
            return

        if payload == self._last_payload:
            return

        try:
            self._temporary_path.write_text(f"{payload}\n", encoding="utf-8")
            self._temporary_path.chmod(0o600)
            os.replace(self._temporary_path, self._output_path)
        except OSError as error:
            self.get_logger().error(f"cannot publish runtime snapshot: {error}")
            return
        self._last_payload = payload

    def clean_temporary_file(self) -> None:
        self._temporary_path.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    output_path = arguments.output
    if not output_path.parent.is_dir():
        raise SystemExit(f"output directory does not exist: {output_path.parent}")
    output_path.unlink(missing_ok=True)

    rclpy.init()
    node = RuntimeStateWatch(output_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.clean_temporary_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
