from __future__ import annotations

import json
from pathlib import Path
import sys

from autoware_control_msgs.msg import Control
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from autoracer_safety.race_contract import COMMAND_QOS, STATE_QOS


class RaceStackMonitor(Node):
    def __init__(self) -> None:
        super().__init__("race_stack_monitor")
        self.declare_parameter("scenario", "normal")
        self.declare_parameter("summary_path", "/tmp/race_stack_summary.json")
        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("drop_after_sec", 5.0)
        self.declare_parameter("drop_duration_sec", 0.0)
        self._scenario = str(self.get_parameter("scenario").value)
        self._start = self._now()
        self._runtime = None
        self._final_times: list[float] = []
        self._emergency_seen = False
        self._emergency_time = None
        self._fault_time = None
        self._finished = False
        self._exit_code = 1
        self._reset_future = None
        self.create_subscription(
            String, "/system/race_runtime/state", self._on_runtime, STATE_QOS
        )
        self._reset_client = self.create_client(Trigger, "/autoracer/race/reset")
        self.create_subscription(
            Control, "/control/command/control_cmd", self._on_final, COMMAND_QOS
        )
        self.create_subscription(
            VehicleEmergencyStamped,
            "/control/command/emergency_cmd",
            self._on_emergency,
            COMMAND_QOS,
        )
        self.create_timer(0.1, self._on_timer)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_runtime(self, msg: String) -> None:
        self._runtime = json.loads(msg.data)
        if self._runtime.get("state") == "FAULT" and self._fault_time is None:
            self._fault_time = self._now()

    def _on_final(self, msg: Control) -> None:
        del msg
        self._final_times.append(self._now())

    def _on_emergency(self, msg: VehicleEmergencyStamped) -> None:
        if bool(msg.emergency) and not self._emergency_seen:
            self._emergency_time = self._now()
        self._emergency_seen = self._emergency_seen or bool(msg.emergency)

    def _on_timer(self) -> None:
        elapsed = self._now() - self._start
        if self._scenario == "normal":
            if (
                elapsed >= 4.0
                and self._runtime is not None
                and self._runtime.get("state") == "ACTIVE"
            ):
                self._finish(True, "ACTIVE_CONFIRMED")
                return
        elif self._scenario == "controller_recover":
            recovery_check = (
                float(self.get_parameter("drop_after_sec").value)
                + float(self.get_parameter("drop_duration_sec").value)
                + 1.0
            )
            if (
                elapsed >= recovery_check
                and self._fault_time is not None
                and self._emergency_seen
                and self._runtime is not None
                and self._runtime.get("state") == "FAULT"
            ):
                self._finish(True, "FAULT_REMAINS_LATCHED_AFTER_INPUT_RECOVERY")
                return
        elif self._scenario == "controller_reset":
            reset_at = (
                float(self.get_parameter("drop_after_sec").value)
                + float(self.get_parameter("drop_duration_sec").value)
                + 1.0
            )
            if (
                elapsed >= reset_at
                and self._fault_time is not None
                and self._reset_future is None
                and self._reset_client.service_is_ready()
            ):
                self._reset_future = self._reset_client.call_async(Trigger.Request())
            if (
                self._reset_future is not None
                and self._reset_future.done()
                and self._reset_future.result() is not None
                and self._reset_future.result().success
                and self._runtime is not None
                and self._runtime.get("state") == "ACTIVE"
            ):
                self._finish(True, "EXPLICIT_RESET_REARMED")
                return
        elif self._emergency_seen:
            if self._scenario == "manager":
                self._finish(True, "RUNTIME_MANAGER_HEARTBEAT_TIMEOUT")
                return
            if self._fault_time is not None:
                self._finish(True, str(self._runtime.get("reason", "FAULT_CONFIRMED")))
                return
        if elapsed >= float(self.get_parameter("timeout_sec").value):
            self._finish(False, "TIMEOUT")

    def _finish(self, passed: bool, reason: str) -> None:
        if self._finished:
            return
        self._finished = True
        duration = max(1.0e-9, self._final_times[-1] - self._final_times[0]) if len(self._final_times) > 1 else 0.0
        rate = (len(self._final_times) - 1) / duration if duration > 0.0 else 0.0
        publishers = {
            topic: len(self.get_publishers_info_by_topic(topic))
            for topic in (
                "/control/command/control_cmd",
                "/control/command/gear_cmd",
                "/control/command/emergency_cmd",
            )
        }
        if rate < 30.0 or any(count != 1 for count in publishers.values()):
            passed = False
            reason = f"TOPIC_CONTRACT rate={rate:.3f} publishers={publishers}"
        injection_time = self._start + float(self.get_parameter("drop_after_sec").value)
        fault_time = self._emergency_time if self._scenario == "manager" else self._fault_time
        fault_latency = None if fault_time is None else fault_time - injection_time
        emergency_latency = (
            None if self._emergency_time is None else self._emergency_time - injection_time
        )
        if self._scenario != "normal" and (
            fault_latency is None
            or emergency_latency is None
            or fault_latency > 0.60
            or emergency_latency > 1.00
        ):
            passed = False
            reason = (
                f"FAULT_LATENCY fault={fault_latency} emergency={emergency_latency}"
            )
        summary = {
            "result": "PASS" if passed else "FAIL",
            "scenario": self._scenario,
            "reason": reason,
            "final_control_rate_hz": rate,
            "publishers": publishers,
            "runtime_manager": self._runtime,
            "emergency_seen": self._emergency_seen,
            "fault_latency_sec": fault_latency,
            "emergency_latency_sec": emergency_latency,
            "runtime_fault_time_sec": self._fault_time,
        }
        path = Path(str(self.get_parameter("summary_path").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        self._exit_code = 0 if passed else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceStackMonitor()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        code = node._exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
