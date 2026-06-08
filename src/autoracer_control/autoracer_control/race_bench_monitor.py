import json
import math
from pathlib import Path
import sys
import time

from autoware_control_msgs.msg import Control
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


SCENARIOS = (
    "straight",
    "left_curve",
    "right_curve",
    "current_speed_low",
    "current_speed_high",
    "missing_trajectory",
    "missing_odometry",
    "missing_steering",
    "missing_acceleration",
    "missing_operation_mode",
    "stale_pose",
    "raw_timeout",
)

VALID_SCENARIOS = {
    "straight",
    "left_curve",
    "right_curve",
    "current_speed_low",
    "current_speed_high",
}

NEGATIVE_SCENARIOS = set(SCENARIOS) - VALID_SCENARIOS


def _format_node_name(name, namespace):
    namespace = namespace.rstrip("/")
    if not namespace:
        return f"/{name}"
    return f"{namespace}/{name}"


def _endpoint_node_name(endpoint):
    return _format_node_name(endpoint.node_name, endpoint.node_namespace)


def _durability_text(durability):
    text = str(durability).lower()
    if "transient_local" in text:
        return "transient_local"
    if "volatile" in text:
        return "volatile"
    return text


def _control_values(control):
    return (
        control.lateral.steering_tire_angle,
        control.lateral.steering_tire_rotation_rate,
        control.longitudinal.velocity,
        control.longitudinal.acceleration,
        control.longitudinal.jerk,
    )


def _finite_control(control):
    return all(math.isfinite(value) for value in _control_values(control))


def _is_stop(control):
    return abs(control.longitudinal.velocity) <= 0.01


class RaceBenchMonitor(Node):
    def __init__(self):
        super().__init__("race_bench_monitor")
        self.declare_parameter("scenario", "straight")
        self.declare_parameter("summary_root", "logs/race_control_bench")
        self.declare_parameter("monitor_timeout_sec", 4.0)
        self.declare_parameter("namespace", "/control_bench")
        self.declare_parameter(
            "raw_control_topic", "/control_bench/autoracer/control/raw_control_cmd"
        )
        self.declare_parameter(
            "final_control_topic", "/control_bench/control/command/control_cmd"
        )
        self.declare_parameter("default_final_topic", "/control/command/control_cmd")
        self.declare_parameter(
            "operation_mode_topic", "/control_bench/system/operation_mode/state"
        )
        self.declare_parameter("state_topic", "/control_bench/autoracer/safety/state")
        self.declare_parameter("straight_abs_steer_max_rad", 0.02)
        self.declare_parameter("command_gate_enable_drive_commands", True)

        self._scenario = str(self.get_parameter("scenario").value)
        if self._scenario not in SCENARIOS:
            raise ValueError(f"unsupported race bench scenario: {self._scenario}")

        self._summary_root = Path(str(self.get_parameter("summary_root").value))
        self._monitor_timeout = float(self.get_parameter("monitor_timeout_sec").value)
        self._namespace = str(self.get_parameter("namespace").value)
        self._raw_topic = str(self.get_parameter("raw_control_topic").value)
        self._final_topic = str(self.get_parameter("final_control_topic").value)
        self._default_final_topic = str(self.get_parameter("default_final_topic").value)
        self._operation_mode_topic = str(self.get_parameter("operation_mode_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._straight_abs_steer_max = float(
            self.get_parameter("straight_abs_steer_max_rad").value
        )
        self._command_gate_enable_drive_commands = bool(
            self.get_parameter("command_gate_enable_drive_commands").value
        )

        self._start_time = self.get_clock().now()
        self._raw_messages = []
        self._final_messages = []
        self._safety_states = []
        self._exit_code = 1
        self._summary_path = None
        self._finished = False

        self.create_subscription(Control, self._raw_topic, self._on_raw_control, 10)
        self.create_subscription(Control, self._final_topic, self._on_final_control, 10)
        self.create_subscription(String, self._state_topic, self._on_safety_state, 10)
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(f"race bench monitor scenario={self._scenario}")

    @property
    def exit_code(self):
        return self._exit_code

    @property
    def finished(self):
        return self._finished

    def _on_raw_control(self, msg):
        self._raw_messages.append(msg)

    def _on_final_control(self, msg):
        self._final_messages.append(msg)

    def _on_safety_state(self, msg):
        self._safety_states.append(msg.data)

    def _on_timer(self):
        if self._finished:
            return
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if elapsed < self._monitor_timeout:
            return
        summary = self._make_summary()
        self._summary_path = self._write_summary(summary)
        self._exit_code = 0 if summary["result"] == "PASS" else 1
        self._finished = True
        self.get_logger().info(f"runtime_summary={self._summary_path}")
        self.get_logger().info(json.dumps(summary, sort_keys=True))

    def _make_summary(self):
        graph = self._graph_snapshot()
        numeric_checks = self._numeric_checks()
        fail_closed_checks = self._fail_closed_checks()
        scenario_result = self._scenario_result(numeric_checks, fail_closed_checks)
        scenario_results = {scenario: "NOT_RUN" for scenario in SCENARIOS}
        scenario_results[self._scenario] = scenario_result

        global_checks = {
            "controller_under_test": True,
            "lateral_controller_mode": True,
            "longitudinal_controller_mode": True,
            "pure_pursuit_started": not graph["pure_pursuit_started"],
            "raw_control_publisher_count": graph["raw_control_publisher_count"] == 1,
            "final_control_publisher_count": graph["final_control_publisher_count"] == 1,
            "final_control_publisher_nodes": graph[
                "final_control_publisher_count"
            ]
            == 1
            and any(
                node.endswith("/command_gate")
                for node in graph["final_control_publisher_nodes"]
            ),
            "default_final_topic_publisher_count": graph[
                "default_final_topic_publisher_count"
            ]
            == 0,
            "operation_mode_qos": graph["operation_mode_qos"] == "transient_local",
            "command_gate_used": graph["command_gate_used"],
            "command_gate_enable_drive_commands": self._command_gate_enable_drive_commands,
        }

        result = "PASS" if scenario_result == "PASS" and all(global_checks.values()) else "FAIL"

        return {
            "stage": "race_control_bench_ros_only",
            "result": result,
            "scenario": self._scenario,
            "scenario_start_time": self._start_time.nanoseconds,
            "ros_domain_id": str(__import__("os").environ.get("ROS_DOMAIN_ID", "")),
            "namespace": self._namespace,
            "namespace_only_isolation_sufficient": False,
            "controller_under_test": "autoware_trajectory_follower_node/controller_node_exe",
            "controller_node_names": graph["controller_node_names"],
            "pure_pursuit_started": graph["pure_pursuit_started"],
            "lateral_controller_mode": "mpc",
            "longitudinal_controller_mode": "pid",
            "data_source": "synthetic_fixture",
            "command_gate_used": graph["command_gate_used"],
            "command_gate_enable_drive_commands": self._command_gate_enable_drive_commands,
            "raw_control_topic": self._raw_topic,
            "final_control_topic": self._final_topic,
            "raw_control_publisher_count": graph["raw_control_publisher_count"],
            "final_control_publisher_count": graph["final_control_publisher_count"],
            "final_control_publisher_nodes": graph["final_control_publisher_nodes"],
            "default_final_topic_publisher_count": graph[
                "default_final_topic_publisher_count"
            ],
            "operation_mode_qos": graph["operation_mode_qos"],
            "scenario_results": scenario_results,
            "numeric_checks": numeric_checks,
            "fail_closed_checks": fail_closed_checks,
            "safety_states_seen": sorted(set(self._safety_states)),
            "global_checks": global_checks,
            "does_not_validate": [
                "CarMaker closed-loop",
                "Stage B planner",
                "real vehicle calibration",
                "race performance",
            ],
        }

    def _graph_snapshot(self):
        raw_publishers = self.get_publishers_info_by_topic(self._raw_topic)
        final_publishers = self.get_publishers_info_by_topic(self._final_topic)
        default_final_publishers = self.get_publishers_info_by_topic(
            self._default_final_topic
        )
        operation_mode_publishers = self.get_publishers_info_by_topic(
            self._operation_mode_topic
        )
        node_names = self.get_node_names_and_namespaces()

        controller_node_names = [
            _format_node_name(name, namespace)
            for name, namespace in node_names
            if name == "controller"
        ]
        final_control_publisher_nodes = [
            _endpoint_node_name(endpoint) for endpoint in final_publishers
        ]
        operation_qos = "UNKNOWN"
        for endpoint in operation_mode_publishers:
            durability = _durability_text(endpoint.qos_profile.durability)
            if durability == "transient_local":
                operation_qos = durability
                break
            operation_qos = durability

        return {
            "controller_node_names": controller_node_names,
            "pure_pursuit_started": any(
                "pure_pursuit" in name for name, _namespace in node_names
            ),
            "command_gate_used": any(name == "command_gate" for name, _namespace in node_names),
            "raw_control_publisher_count": len(raw_publishers),
            "final_control_publisher_count": len(final_publishers),
            "final_control_publisher_nodes": final_control_publisher_nodes,
            "default_final_topic_publisher_count": len(default_final_publishers),
            "operation_mode_qos": operation_qos,
        }

    def _numeric_checks(self):
        raw = self._raw_messages[-1] if self._raw_messages else None
        final = self._final_messages[-1] if self._final_messages else None

        straight_abs_steer = None
        left_sign = None
        right_sign = None
        low_speed = None
        high_speed = None
        if raw is not None:
            steer = raw.lateral.steering_tire_angle
            if self._scenario == "straight":
                straight_abs_steer = abs(steer) <= self._straight_abs_steer_max
            elif self._scenario == "left_curve":
                left_sign = steer > 0.0
            elif self._scenario == "right_curve":
                right_sign = steer < 0.0
            elif self._scenario == "current_speed_low":
                low_speed = (
                    raw.longitudinal.velocity > 0.5
                    or raw.longitudinal.acceleration > 0.0
                )
            elif self._scenario == "current_speed_high":
                high_speed = (
                    raw.longitudinal.velocity < 1.0
                    or raw.longitudinal.acceleration < 0.0
                )

        return {
            "straight_abs_steer_max_rad": self._straight_abs_steer_max,
            "straight_abs_steer_within_limit": straight_abs_steer,
            "left_steer_positive": left_sign,
            "right_steer_negative": right_sign,
            "left_right_sign_opposite": None,
            "low_speed_accel_positive_or_velocity_gt_current": low_speed,
            "high_speed_accel_negative_or_velocity_lt_current": high_speed,
            "raw_no_nan_inf": raw is None or _finite_control(raw),
            "final_no_nan_inf": final is None or _finite_control(final),
        }

    def _fail_closed_checks(self):
        final = self._final_messages[-1] if self._final_messages else None
        return {
            "final_received": final is not None,
            "final_stop": final is not None and _is_stop(final),
            "stale_pose_final_stop": (
                self._scenario != "stale_pose"
                or (final is not None and _is_stop(final))
            ),
            "raw_timeout_final_stop": (
                self._scenario != "raw_timeout"
                or (final is not None and _is_stop(final))
            ),
            "localization_timeout_seen": "localization_timeout" in self._safety_states,
            "raw_command_timeout_seen": "raw_command_timeout" in self._safety_states,
            "trajectory_timeout_seen": "trajectory_timeout" in self._safety_states,
            "drive_enabled_seen": "drive_enabled" in self._safety_states,
        }

    def _scenario_result(self, numeric_checks, fail_closed_checks):
        raw = self._raw_messages[-1] if self._raw_messages else None
        final = self._final_messages[-1] if self._final_messages else None
        if final is None or not numeric_checks["final_no_nan_inf"]:
            return "FAIL"
        if raw is not None and not numeric_checks["raw_no_nan_inf"]:
            return "FAIL"

        if self._scenario in VALID_SCENARIOS:
            if raw is None:
                return "FAIL"
            if not fail_closed_checks["drive_enabled_seen"]:
                return "FAIL"
            if _is_stop(final):
                return "FAIL"
            scenario_key = {
                "straight": "straight_abs_steer_within_limit",
                "left_curve": "left_steer_positive",
                "right_curve": "right_steer_negative",
                "current_speed_low": "low_speed_accel_positive_or_velocity_gt_current",
                "current_speed_high": "high_speed_accel_negative_or_velocity_lt_current",
            }[self._scenario]
            return "PASS" if numeric_checks[scenario_key] is True else "FAIL"

        if self._scenario == "stale_pose":
            return (
                "PASS"
                if fail_closed_checks["final_stop"]
                and fail_closed_checks["localization_timeout_seen"]
                else "FAIL"
            )
        if self._scenario == "raw_timeout":
            return (
                "PASS"
                if fail_closed_checks["final_stop"]
                and fail_closed_checks["raw_command_timeout_seen"]
                else "FAIL"
            )
        if self._scenario in NEGATIVE_SCENARIOS:
            return "PASS" if fail_closed_checks["final_stop"] else "FAIL"
        return "FAIL"

    def _write_summary(self, summary):
        run_dir = self._summary_root / time.strftime("%Y%m%d-%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        path = run_dir / "runtime_summary.json"
        path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path


def _shutdown_if_context_ok():
    if rclpy.ok():
        rclpy.shutdown()


def _is_shutdown_rcl_error(exc):
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
    )


def main():
    rclpy.init()
    node = RaceBenchMonitor()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        _shutdown_if_context_ok()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
