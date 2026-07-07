import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_adapi_v1_msgs.msg import ResponseStatus as AdapiResponseStatus
from autoware_adapi_v1_msgs.srv import InitializeLocalization as AdapiInitialize
from autoware_localization_msgs.srv import InitializeLocalization as LocalInitialize
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState


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
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("service_timeout_sec", 30.0)

        api_service = self._string_param("api_initialize_service")
        local_service = self._string_param("local_initialize_service")
        api_state_topic = self._string_param("api_initialization_state_topic")
        local_state_topic = self._string_param("local_initialization_state_topic")
        gnss_pose_topic = self._string_param("gnss_pose_topic")
        self._service_timeout_sec = (
            self.get_parameter("service_timeout_sec").get_parameter_value().double_value
        )
        self._latest_gnss_pose = None
        self._lock = threading.Lock()

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
        self._sub_gnss_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            gnss_pose_topic,
            self._on_gnss_pose,
            1,
            callback_group=self._group,
        )
        self._client = self.create_client(
            LocalInitialize, local_service, callback_group=self._group
        )
        self._service = self.create_service(
            AdapiInitialize, api_service, self._on_initialize, callback_group=self._group
        )

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_state(self, msg: LocalizationInitializationState) -> None:
        self._pub_state.publish(msg)

    def _on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._latest_gnss_pose = msg

    def _on_initialize(self, request, response):
        if not self._client.wait_for_service(timeout_sec=1.0):
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                "local /localization/initialize service is not ready",
            )
            return response

        local_request = LocalInitialize.Request()
        if len(request.pose) > 0:
            local_request.method = LocalInitialize.Request.AUTO
            local_request.pose_with_covariance = list(request.pose)
        else:
            with self._lock:
                gnss_pose = self._latest_gnss_pose
            if gnss_pose is None:
                response.status = self._status(
                    False,
                    AdapiInitialize.Response.ERROR_GNSS,
                    "GNSS pose has not arrived",
                )
                return response
            local_request.method = LocalInitialize.Request.DIRECT
            local_request.pose_with_covariance = [gnss_pose]

        done = threading.Event()
        future = self._client.call_async(local_request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._service_timeout_sec):
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                "local /localization/initialize service timed out",
            )
            return response

        try:
            local_response = future.result()
        except Exception as exc:  # noqa: BLE001 - propagate service failure as API status.
            response.status = self._status(
                False,
                AdapiInitialize.Response.ERROR_ESTIMATION,
                f"local /localization/initialize service failed: {exc}",
            )
            return response

        response.status = self._status(
            local_response.status.success,
            local_response.status.code,
            local_response.status.message,
        )
        return response

    def _status(self, success: bool, code: int, message: str) -> AdapiResponseStatus:
        status = AdapiResponseStatus()
        status.success = success
        status.code = code
        status.message = message
        return status


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
