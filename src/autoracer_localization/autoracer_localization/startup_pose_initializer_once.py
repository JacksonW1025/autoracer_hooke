import rclpy
from autoware_localization_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from autoracer_localization.pose_stream_qos import latest_pose_qos


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def startup_initialize_should_attempt(
    *,
    gnss_stamp_sec: float,
    min_gnss_stamp_sec: float,
    request_in_flight: bool,
    initialized: bool,
) -> bool:
    return (
        not initialized
        and not request_in_flight
        and float(gnss_stamp_sec) >= float(min_gnss_stamp_sec)
    )


def startup_initialize_method_to_request(method: str) -> int:
    normalized = method.strip().lower()
    if normalized == "auto":
        return InitializeLocalization.Request.AUTO
    if normalized == "direct":
        return InitializeLocalization.Request.DIRECT
    raise ValueError(f"unsupported initialize_method: {method}")


class StartupPoseInitializerOnce(Node):
    def __init__(self):
        super().__init__("startup_pose_initializer_once")
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("initialize_service", "/localization/initialize")
        self.declare_parameter("initialize_method", "auto")
        self.declare_parameter("min_gnss_stamp_sec", 2.0)

        self._min_gnss_stamp_sec = float(self.get_parameter("min_gnss_stamp_sec").value)
        self._initialize_method = startup_initialize_method_to_request(
            str(self.get_parameter("initialize_method").value)
        )
        initialize_service = str(self.get_parameter("initialize_service").value)
        gnss_pose_topic = str(self.get_parameter("gnss_pose_topic").value)

        self._client = self.create_client(InitializeLocalization, initialize_service)
        self._request_in_flight = False
        self._initialized = False
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            gnss_pose_topic,
            self._on_gnss_pose,
            latest_pose_qos(),
        )

    def _on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if not startup_initialize_should_attempt(
            gnss_stamp_sec=stamp_to_sec(msg.header.stamp),
            min_gnss_stamp_sec=self._min_gnss_stamp_sec,
            request_in_flight=self._request_in_flight,
            initialized=self._initialized,
        ):
            return
        if not self._client.service_is_ready():
            return

        request = InitializeLocalization.Request()
        request.method = self._initialize_method
        request.pose_with_covariance = [msg]
        self._request_in_flight = True
        future = self._client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        self._request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive ROS callback logging
            self.get_logger().warn(f"startup initialize service call failed: {exc}")
            return
        if response.status.success:
            self._initialized = True
            self.get_logger().info("startup initialize succeeded via pose_initializer AUTO")
        else:
            self.get_logger().warn(
                "startup initialize failed: "
                f"code={response.status.code} message={response.status.message}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StartupPoseInitializerOnce()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
