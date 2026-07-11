import threading
import time
from math import atan2

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.msg import ResponseStatus as AdapiResponseStatus
from autoware_adapi_v1_msgs.srv import InitializeLocalization as AdapiInitialize
from autoware_localization_msgs.srv import InitializeLocalization as LocalInitialize
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class LocalizationAdapiBridge(Node):
    def __init__(self):
        super().__init__("localization_adapi_bridge")
        self.declare_parameter("api_initialize_service", "/api/localization/initialize")
        self.declare_parameter("local_initialize_service", "/localization/initialize")
        self.declare_parameter(
            "api_initialization_state_topic", "/api/localization/initialization_state"
        )
        self.declare_parameter(
            "local_initialization_state_topic", "/localization/initialization_state"
        )
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("min_initialize_stamp_sec", 0.0)
        self.declare_parameter("max_auto_retry_attempts", 1)
        self.declare_parameter("auto_retry_delay_sec", 0.2)
        self.declare_parameter("auto_retry_initialpose_timeout_sec", 3.0)
        self.declare_parameter("auto_retry_max_gnss_xy_m", 0.0)
        self.declare_parameter("auto_retry_max_gnss_yaw_deg", 0.0)
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("initialpose_topic", "/initialpose3d")

        api_service = self._string_param("api_initialize_service")
        local_service = self._string_param("local_initialize_service")
        api_state_topic = self._string_param("api_initialization_state_topic")
        local_state_topic = self._string_param("local_initialization_state_topic")
        self._service_timeout_sec = (
            self.get_parameter("service_timeout_sec").get_parameter_value().double_value
        )
        self._min_initialize_stamp_sec = (
            self.get_parameter("min_initialize_stamp_sec").get_parameter_value().double_value
        )
        self._max_auto_retry_attempts = max(
            1,
            self.get_parameter("max_auto_retry_attempts")
            .get_parameter_value()
            .integer_value,
        )
        self._auto_retry_delay_sec = max(
            0.0,
            self.get_parameter("auto_retry_delay_sec").get_parameter_value().double_value,
        )
        self._auto_retry_initialpose_timeout_sec = max(
            0.0,
            self.get_parameter("auto_retry_initialpose_timeout_sec")
            .get_parameter_value()
            .double_value,
        )
        self._auto_retry_max_gnss_xy_m = (
            self.get_parameter("auto_retry_max_gnss_xy_m").get_parameter_value().double_value
        )
        self._auto_retry_max_gnss_yaw_deg = (
            self.get_parameter("auto_retry_max_gnss_yaw_deg")
            .get_parameter_value()
            .double_value
        )
        self._auto_retry_enabled = (
            self._auto_retry_max_gnss_xy_m > 0.0
            or self._auto_retry_max_gnss_yaw_deg > 0.0
        )

        qos_state = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._group = ReentrantCallbackGroup()
        self._pub_state = self.create_publisher(
            LocalizationInitializationState, api_state_topic, qos_state
        )
        self._sub_state = self.create_subscription(
            LocalizationInitializationState,
            local_state_topic,
            self._on_state,
            qos_state,
            callback_group=self._group,
        )
        self._sub_gnss = self.create_subscription(
            PoseWithCovarianceStamped,
            self._string_param("gnss_pose_topic"),
            self._on_gnss,
            10,
            callback_group=self._group,
        )
        self._sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped,
            self._string_param("initialpose_topic"),
            self._on_initialpose,
            10,
            callback_group=self._group,
        )
        self._client = self.create_client(
            LocalInitialize, local_service, callback_group=self._group
        )
        self._service = self.create_service(
            AdapiInitialize, api_service, self._on_initialize, callback_group=self._group
        )
        self._lock = threading.Lock()
        self._latest_gnss = None
        self._latest_initialpose = None
        self._initialpose_event = threading.Event()
        self._initialization_in_progress = False
        self._accepted_initialized = False
        self._last_local_state = None

        if self._auto_retry_enabled:
            self.get_logger().info(
                "AUTO initialization consistency retry enabled: "
                f"attempts={self._max_auto_retry_attempts} "
                f"max_gnss_xy={self._auto_retry_max_gnss_xy_m:.3f}m "
                f"max_gnss_yaw={self._auto_retry_max_gnss_yaw_deg:.3f}deg"
            )

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_state(self, msg: LocalizationInitializationState) -> None:
        with self._lock:
            self._last_local_state = msg
            suppress_initialized = (
                self._auto_retry_enabled
                and self._initialization_in_progress
                and not self._accepted_initialized
                and msg.state == LocalizationInitializationState.INITIALIZED
            )
        if suppress_initialized:
            gated = LocalizationInitializationState()
            gated.stamp = msg.stamp
            gated.state = LocalizationInitializationState.INITIALIZING
            self._pub_state.publish(gated)
            return
        self._pub_state.publish(msg)

    def _on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._latest_gnss = msg

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._latest_initialpose = msg
            self._initialpose_event.set()

    def _on_initialize(self, request, response):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._min_initialize_stamp_sec > 0.0 and now_sec < self._min_initialize_stamp_sec:
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                (
                    "AUTO localization initialization is gated until "
                    f"{self._min_initialize_stamp_sec:.2f}s; current={now_sec:.2f}s"
                ),
            )
            return response

        if not self._client.wait_for_service(timeout_sec=1.0):
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                "local /localization/initialize service is not ready",
            )
            return response

        with self._lock:
            self._initialization_in_progress = True
            self._accepted_initialized = False
        try:
            last_status = None
            for attempt in range(1, self._max_auto_retry_attempts + 1):
                local_response = self._call_local_initialize(request)
                last_status = local_response.status
                if not local_response.status.success:
                    break
                consistency = self._check_initialpose_gnss_consistency()
                if consistency["accepted"]:
                    self._accept_initialized_state()
                    response.status = self._status(
                        True,
                        local_response.status.code,
                        self._append_retry_message(local_response.status.message, attempt, consistency),
                    )
                    return response

                self.get_logger().warn(
                    "AUTO initialization attempt "
                    f"{attempt}/{self._max_auto_retry_attempts} rejected by "
                    f"GNSS consistency gate: {consistency['message']}"
                )
                if attempt < self._max_auto_retry_attempts and self._auto_retry_delay_sec > 0.0:
                    time.sleep(self._auto_retry_delay_sec)

            if last_status is None:
                response.status = self._status(
                    False,
                    AdapiInitialize.Response.ERROR_ESTIMATION,
                    "local /localization/initialize service was not called",
                )
            elif last_status.success:
                response.status = self._status(
                    False,
                    AdapiInitialize.Response.ERROR_ESTIMATION,
                    (
                        "AUTO localization initialization did not satisfy GNSS "
                        f"consistency after {self._max_auto_retry_attempts} attempts"
                    ),
                )
            else:
                response.status = self._status(
                    False,
                    last_status.code,
                    last_status.message,
                )
            return response
        except Exception as exc:  # noqa: BLE001 - service callbacks must report failures as status.
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                f"local /localization/initialize service failed: {exc}",
            )
            return response
        finally:
            with self._lock:
                self._initialization_in_progress = False

    def _status(self, success: bool, code: int, message: str) -> AdapiResponseStatus:
        status = AdapiResponseStatus()
        status.success = success
        status.code = code
        status.message = message
        return status

    def _call_local_initialize(self, request):
        with self._lock:
            self._latest_initialpose = None
            self._initialpose_event.clear()

        local_request = LocalInitialize.Request()
        local_request.method = LocalInitialize.Request.AUTO
        local_request.pose_with_covariance = list(request.pose)

        done = threading.Event()
        future = self._client.call_async(local_request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._service_timeout_sec):
            raise RuntimeError("local /localization/initialize service timed out")
        return future.result()

    def _check_initialpose_gnss_consistency(self) -> dict:
        if not self._auto_retry_enabled:
            return {"accepted": True, "message": "GNSS consistency retry disabled"}

        if self._auto_retry_initialpose_timeout_sec > 0.0:
            self._initialpose_event.wait(self._auto_retry_initialpose_timeout_sec)

        with self._lock:
            initialpose = self._latest_initialpose
            gnss = self._latest_gnss
        if initialpose is None:
            return {"accepted": False, "message": "no /initialpose3d observed"}
        if gnss is None:
            return {"accepted": False, "message": "no GNSS pose observed"}

        dx = initialpose.pose.pose.position.x - gnss.pose.pose.position.x
        dy = initialpose.pose.pose.position.y - gnss.pose.pose.position.y
        xy_m = (dx * dx + dy * dy) ** 0.5
        yaw_deg = abs(
            self._normalize_angle(
                self._yaw_from_pose(initialpose) - self._yaw_from_pose(gnss)
            )
        ) * 180.0 / 3.141592653589793

        failed = []
        if self._auto_retry_max_gnss_xy_m > 0.0 and xy_m > self._auto_retry_max_gnss_xy_m:
            failed.append(f"xy={xy_m:.3f}m>{self._auto_retry_max_gnss_xy_m:.3f}m")
        if (
            self._auto_retry_max_gnss_yaw_deg > 0.0
            and yaw_deg > self._auto_retry_max_gnss_yaw_deg
        ):
            failed.append(
                f"yaw={yaw_deg:.3f}deg>{self._auto_retry_max_gnss_yaw_deg:.3f}deg"
            )
        message = f"xy={xy_m:.3f}m yaw={yaw_deg:.3f}deg"
        if failed:
            message += " (" + ", ".join(failed) + ")"
        return {"accepted": not failed, "message": message, "xy_m": xy_m, "yaw_deg": yaw_deg}

    def _accept_initialized_state(self) -> None:
        with self._lock:
            self._accepted_initialized = True
            state = self._last_local_state
        if state is not None:
            self._pub_state.publish(state)

    def _append_retry_message(self, message: str, attempt: int, consistency: dict) -> str:
        if not self._auto_retry_enabled:
            return message
        suffix = (
            f"AUTO GNSS consistency accepted on attempt {attempt}/"
            f"{self._max_auto_retry_attempts}: {consistency['message']}"
        )
        return f"{message}; {suffix}" if message else suffix

    def _yaw_from_pose(self, msg: PoseWithCovarianceStamped) -> float:
        q = msg.pose.pose.orientation
        return atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _normalize_angle(self, value: float) -> float:
        while value > 3.141592653589793:
            value -= 2.0 * 3.141592653589793
        while value < -3.141592653589793:
            value += 2.0 * 3.141592653589793
        return value


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationAdapiBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
