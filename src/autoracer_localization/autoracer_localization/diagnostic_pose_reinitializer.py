from __future__ import annotations

import copy
from typing import Iterable

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_localization_msgs.srv import InitializeLocalization
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from autoracer_localization.pose_stream_qos import latest_pose_qos
from autoracer_localization.startup_pose_initializer_once import startup_initialize_method_to_request

DIAGNOSTIC_ERROR_LEVEL = 2
POSE_INSTABILITY_PLANAR_ERROR_STATUS_KEYS = (
    "diff_position_x:status",
    "diff_position_y:status",
    "diff_angle_z:status",
)


def diagnostic_level_to_int(level) -> int:
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, byteorder="little", signed=False)
    return int(level)


def diagnostic_name_matches_target(status_name: str, target_name: str) -> bool:
    return status_name == target_name or status_name.endswith(f": {target_name}")


def diagnostic_name_is_pose_instability(status_name: str) -> bool:
    return diagnostic_name_matches_target(status_name, "pose_instability_detector")


def diagnostic_name_is_ekf_localizer(status_name: str) -> bool:
    return diagnostic_name_matches_target(status_name, "ekf_localizer")


def diagnostic_status_value_as_int(status: DiagnosticStatus, key: str) -> int | None:
    for value in status.values:
        if value.key != key:
            continue
        try:
            return int(value.value)
        except ValueError:
            return None
    return None


def diagnostic_pose_instability_status_has_planar_error(status: DiagnosticStatus) -> bool:
    value_by_key = {value.key: value.value for value in status.values}
    return any(
        value_by_key.get(key) == "ERROR" for key in POSE_INSTABILITY_PLANAR_ERROR_STATUS_KEYS
    )


def diagnostic_ekf_status_has_pose_no_update_error(status: DiagnosticStatus) -> bool:
    pose_no_update_count = diagnostic_status_value_as_int(status, "pose_no_update_count")
    pose_no_update_threshold = diagnostic_status_value_as_int(
        status, "pose_no_update_count_threshold_error"
    )
    if pose_no_update_count is None or pose_no_update_threshold is None:
        return False
    return pose_no_update_count >= pose_no_update_threshold


def diagnostic_measurement_after_initialization_allows_lost_reinitialization(
    *,
    last_measurement_stamp_sec: float | None,
    last_initialized_stamp_sec: float | None,
) -> bool:
    if last_measurement_stamp_sec is None or last_initialized_stamp_sec is None:
        return False
    return float(last_measurement_stamp_sec) > float(last_initialized_stamp_sec)


def diagnostic_stamp_allows_reinitialization(
    *, diagnostic_stamp_sec: float, min_diagnostic_stamp_sec: float
) -> bool:
    return float(diagnostic_stamp_sec) >= float(min_diagnostic_stamp_sec)


def diagnostic_reinitializer_has_required_seed(
    *, initialize_method: int, has_latest_gnss_pose: bool
) -> bool:
    return int(initialize_method) != InitializeLocalization.Request.DIRECT or has_latest_gnss_pose


def diagnostic_pose_stamp_is_fresh(
    *, pose_stamp_sec: float, reference_stamp_sec: float, max_age_sec: float
) -> bool:
    return abs(float(reference_stamp_sec) - float(pose_stamp_sec)) <= float(max_age_sec)


def diagnostic_initialization_state_allows_reinitialization(
    initialization_state: int | None,
) -> bool:
    return int(initialization_state or 0) == LocalizationInitializationState.INITIALIZED


def diagnostic_post_initialization_grace_allows_reinitialization(
    *,
    diagnostic_stamp_sec: float,
    last_initialized_stamp_sec: float | None,
    post_initialization_grace_sec: float,
) -> bool:
    if last_initialized_stamp_sec is None:
        return True
    return (
        float(diagnostic_stamp_sec) - float(last_initialized_stamp_sec)
        >= float(post_initialization_grace_sec)
    )


def diagnostic_sustained_trigger_allows_reinitialization(
    *,
    diagnostic_stamp_sec: float,
    first_trigger_stamp_sec: float,
    min_trigger_duration_sec: float,
) -> bool:
    return (
        float(diagnostic_stamp_sec) - float(first_trigger_stamp_sec)
        >= float(min_trigger_duration_sec)
    )


def diagnostic_update_sustained_trigger_start(
    trigger_starts: dict[str, float],
    *,
    trigger_name: str,
    is_triggering: bool,
    diagnostic_stamp_sec: float,
) -> float | None:
    if not is_triggering:
        trigger_starts.pop(trigger_name, None)
        return None
    if trigger_name not in trigger_starts:
        trigger_starts[trigger_name] = float(diagnostic_stamp_sec)
    return trigger_starts[trigger_name]


def diagnostic_status_should_trigger_reinitialization(
    status: DiagnosticStatus,
    target_status_names: Iterable[str],
    *,
    min_level: int = DIAGNOSTIC_ERROR_LEVEL,
) -> bool:
    if diagnostic_level_to_int(status.level) < int(min_level):
        return False
    if not any(diagnostic_name_matches_target(status.name, target) for target in target_status_names):
        return False
    if diagnostic_name_is_pose_instability(status.name):
        return diagnostic_pose_instability_status_has_planar_error(status)
    if diagnostic_name_is_ekf_localizer(status.name):
        return diagnostic_ekf_status_has_pose_no_update_error(status)
    return True


class DiagnosticPoseReinitializer(Node):
    def __init__(self) -> None:
        super().__init__("diagnostic_pose_reinitializer")
        self._diagnostics_topic = (
            self.declare_parameter("diagnostics_topic", "/diagnostics")
            .get_parameter_value()
            .string_value
        )
        self._initialize_service = (
            self.declare_parameter("initialize_service", "/localization/initialize")
            .get_parameter_value()
            .string_value
        )
        self._gnss_pose_topic = (
            self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
            .get_parameter_value()
            .string_value
        )
        self._initialization_state_topic = (
            self.declare_parameter(
                "initialization_state_topic", "/localization/initialization_state"
            )
            .get_parameter_value()
            .string_value
        )
        self._direct_pose_topic = (
            self.declare_parameter("direct_pose_topic", "/initialpose3d")
            .get_parameter_value()
            .string_value
        )
        self._pose_observation_topic = (
            self.declare_parameter(
                "pose_observation_topic", "/localization/pose_estimator/pose_with_covariance"
            )
            .get_parameter_value()
            .string_value
        )
        self._initialize_method = startup_initialize_method_to_request(
            self.declare_parameter("initialize_method", "auto").get_parameter_value().string_value
        )
        self._target_status_names = (
            self.declare_parameter(
                "target_status_names",
                ["localization: pose_instability_detector", "localization: ekf_localizer"],
            )
            .get_parameter_value()
            .string_array_value
        )
        self._cooldown_sec = (
            self.declare_parameter("cooldown_sec", 10.0).get_parameter_value().double_value
        )
        self._min_level = (
            self.declare_parameter("min_level", DIAGNOSTIC_ERROR_LEVEL)
            .get_parameter_value()
            .integer_value
        )
        self._min_diagnostic_stamp_sec = (
            self.declare_parameter("min_diagnostic_stamp_sec", 0.0)
            .get_parameter_value()
            .double_value
        )
        self._post_initialization_grace_sec = (
            self.declare_parameter("post_initialization_grace_sec", 5.0)
            .get_parameter_value()
            .double_value
        )
        self._min_trigger_duration_sec = (
            self.declare_parameter("min_trigger_duration_sec", 1.0)
            .get_parameter_value()
            .double_value
        )
        self._max_gnss_pose_age_sec = (
            self.declare_parameter("max_gnss_pose_age_sec", 0.5)
            .get_parameter_value()
            .double_value
        )
        self._last_trigger_time: Time | None = None
        self._first_trigger_stamp_by_name: dict[str, float] = {}
        self._latest_gnss_pose: PoseWithCovarianceStamped | None = None
        self._latest_initialization_state: int | None = None
        self._last_initialized_stamp_sec: float | None = None
        self._last_measurement_stamp_sec: float | None = None
        self._client = self.create_client(InitializeLocalization, self._initialize_service)
        self._direct_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self._direct_pose_topic, 1
        )
        self._subscription = self.create_subscription(
            DiagnosticArray,
            self._diagnostics_topic,
            self._on_diagnostics,
            latest_pose_qos(),
        )
        self._gnss_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._gnss_pose_topic,
            self._on_gnss_pose,
            latest_pose_qos(),
        )
        self._initialization_state_subscription = self.create_subscription(
            LocalizationInitializationState,
            self._initialization_state_topic,
            self._on_initialization_state,
            latest_pose_qos(),
        )
        self._pose_observation_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._pose_observation_topic,
            self._on_pose_observation,
            latest_pose_qos(),
        )

    def _on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_gnss_pose = msg

    def _on_pose_observation(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_measurement_stamp_sec = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )

    def _on_initialization_state(self, msg: LocalizationInitializationState) -> None:
        self._latest_initialization_state = int(msg.state)
        if msg.state == LocalizationInitializationState.INITIALIZED:
            self._last_initialized_stamp_sec = (
                float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
            )

    def _publish_direct_initialpose(self) -> bool:
        if self._latest_gnss_pose is None:
            return False
        self._direct_pose_publisher.publish(copy.deepcopy(self._latest_gnss_pose))
        return True

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        diagnostic_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if not diagnostic_stamp_allows_reinitialization(
            diagnostic_stamp_sec=diagnostic_stamp_sec,
            min_diagnostic_stamp_sec=self._min_diagnostic_stamp_sec,
        ):
            return

        if not diagnostic_initialization_state_allows_reinitialization(
            self._latest_initialization_state
        ):
            return

        if not diagnostic_post_initialization_grace_allows_reinitialization(
            diagnostic_stamp_sec=diagnostic_stamp_sec,
            last_initialized_stamp_sec=self._last_initialized_stamp_sec,
            post_initialization_grace_sec=self._post_initialization_grace_sec,
        ):
            return

        matching_target_statuses = [
            status
            for status in msg.status
            if any(
                diagnostic_name_matches_target(status.name, target)
                for target in self._target_status_names
            )
        ]
        if not matching_target_statuses:
            return

        eligible_trigger = False
        for status in matching_target_statuses:
            is_triggering = diagnostic_status_should_trigger_reinitialization(
                status, self._target_status_names, min_level=self._min_level
            )
            if is_triggering and diagnostic_name_is_ekf_localizer(status.name):
                is_triggering = (
                    diagnostic_measurement_after_initialization_allows_lost_reinitialization(
                        last_measurement_stamp_sec=self._last_measurement_stamp_sec,
                        last_initialized_stamp_sec=self._last_initialized_stamp_sec,
                    )
                )
            first_trigger_stamp_sec = diagnostic_update_sustained_trigger_start(
                self._first_trigger_stamp_by_name,
                trigger_name=status.name,
                is_triggering=is_triggering,
                diagnostic_stamp_sec=diagnostic_stamp_sec,
            )
            if first_trigger_stamp_sec is None:
                continue
            if diagnostic_sustained_trigger_allows_reinitialization(
                diagnostic_stamp_sec=diagnostic_stamp_sec,
                first_trigger_stamp_sec=first_trigger_stamp_sec,
                min_trigger_duration_sec=self._min_trigger_duration_sec,
            ):
                eligible_trigger = True
                break
        if not eligible_trigger:
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if self._last_trigger_time is not None:
            elapsed = (now - self._last_trigger_time).nanoseconds * 1e-9
            if elapsed < self._cooldown_sec:
                return

        if not diagnostic_reinitializer_has_required_seed(
            initialize_method=self._initialize_method,
            has_latest_gnss_pose=self._latest_gnss_pose is not None,
        ):
            self.get_logger().warn(
                "Skip localization reinitialization: latest GNSS pose is not available"
            )
            return
        if self._initialize_method == InitializeLocalization.Request.DIRECT:
            latest_gnss_stamp_sec = (
                float(self._latest_gnss_pose.header.stamp.sec)
                + float(self._latest_gnss_pose.header.stamp.nanosec) * 1e-9
            )
            if not diagnostic_pose_stamp_is_fresh(
                pose_stamp_sec=latest_gnss_stamp_sec,
                reference_stamp_sec=now_sec,
                max_age_sec=self._max_gnss_pose_age_sec,
            ):
                self.get_logger().warn(
                    "Skip localization reinitialization: latest GNSS pose is stale "
                    f"(gnss={latest_gnss_stamp_sec:.3f}, now={now_sec:.3f})"
                )
                return

            if self._publish_direct_initialpose():
                self._last_trigger_time = now
                self._first_trigger_stamp_by_name.clear()
                self.get_logger().warn(
                    "Published direct GNSS localization reinitialization from diagnostics ERROR"
                )
                return

        if not self._client.service_is_ready():
            self.get_logger().warn(
                f"Skip localization reinitialization: service not ready: {self._initialize_service}"
            )
            return

        request = InitializeLocalization.Request()
        request.method = self._initialize_method
        if self._initialize_method == InitializeLocalization.Request.DIRECT:
            request.pose_with_covariance = [self._latest_gnss_pose]
        self._client.call_async(request)
        self._last_trigger_time = now
        self._first_trigger_stamp_by_name.clear()
        self.get_logger().warn("Requested localization reinitialization from diagnostics ERROR")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DiagnosticPoseReinitializer()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
