import copy
import csv
import math

import rclpy
from autoware_localization_msgs.srv import InitializeLocalization
from geometry_msgs.msg import Quaternion
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


def load_route_xy_samples(csv_path: str) -> list[tuple[float, float]]:
    if not csv_path:
        return []
    samples: list[tuple[float, float]] = []
    with open(csv_path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                samples.append((float(row["x"]), float(row["y"])))
            except (KeyError, TypeError, ValueError):
                continue
    return samples


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = 0.5 * float(yaw)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


def route_heading_for_xy(
    samples: list[tuple[float, float]],
    *,
    x: float,
    y: float,
    max_distance_m: float,
    neighbor_stride: int,
    prefer_start_within_m: float = 0.0,
) -> tuple[float, float, int] | None:
    if len(samples) < 2:
        return None
    stride = max(1, int(neighbor_stride))
    start_distance = math.hypot(samples[0][0] - x, samples[0][1] - y)
    if float(prefer_start_within_m) > 0.0 and start_distance <= float(prefer_start_within_m):
        nearest_index = 0
    else:
        nearest_index = min(
            range(len(samples)),
            key=lambda idx: (samples[idx][0] - x) ** 2 + (samples[idx][1] - y) ** 2,
        )
    nearest_x, nearest_y = samples[nearest_index]
    nearest_distance = math.hypot(nearest_x - x, nearest_y - y)
    if nearest_distance > float(max_distance_m):
        return None

    prev_index = max(0, nearest_index - stride)
    next_index = min(len(samples) - 1, nearest_index + stride)
    if prev_index == next_index:
        return None
    prev_x, prev_y = samples[prev_index]
    next_x, next_y = samples[next_index]
    if math.hypot(next_x - prev_x, next_y - prev_y) <= 1e-6:
        return None
    yaw = math.atan2(next_y - prev_y, next_x - prev_x)
    return yaw, nearest_distance, nearest_index


def replace_startup_pose_yaw_from_route(
    msg: PoseWithCovarianceStamped,
    route_samples: list[tuple[float, float]],
    *,
    enabled: bool,
    max_distance_m: float,
    neighbor_stride: int,
    yaw_variance: float,
    snap_xy_to_route: bool = False,
    prefer_start_within_m: float = 0.0,
) -> tuple[PoseWithCovarianceStamped, bool, float | None, int | None]:
    if not enabled:
        return msg, False, None, None
    match = route_heading_for_xy(
        route_samples,
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        max_distance_m=max_distance_m,
        neighbor_stride=neighbor_stride,
        prefer_start_within_m=prefer_start_within_m,
    )
    if match is None:
        return msg, False, None, None
    yaw, distance, route_index = match
    updated = copy.deepcopy(msg)
    if snap_xy_to_route:
        route_x, route_y = route_samples[route_index]
        updated.pose.pose.position.x = route_x
        updated.pose.pose.position.y = route_y
    updated.pose.pose.orientation = _yaw_to_quaternion(yaw)
    if float(yaw_variance) > 0.0:
        updated.pose.covariance[35] = float(yaw_variance)
    return updated, True, distance, route_index


class StartupPoseInitializerOnce(Node):
    def __init__(self):
        super().__init__("startup_pose_initializer_once")
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter("initialize_service", "/localization/initialize")
        self.declare_parameter("initialize_method", "auto")
        self.declare_parameter("min_gnss_stamp_sec", 2.0)
        self.declare_parameter("enable_route_heading_startup", False)
        self.declare_parameter("route_heading_startup_samples_csv", "")
        self.declare_parameter("route_heading_startup_max_distance_m", 20.0)
        self.declare_parameter("route_heading_startup_neighbor_stride", 4)
        self.declare_parameter("route_heading_startup_yaw_variance", 0.007615435494667714)
        self.declare_parameter("route_heading_startup_snap_xy", False)
        self.declare_parameter("route_heading_startup_prefer_start_within_m", 0.0)

        self._min_gnss_stamp_sec = float(self.get_parameter("min_gnss_stamp_sec").value)
        self._initialize_method = startup_initialize_method_to_request(
            str(self.get_parameter("initialize_method").value)
        )
        self._enable_route_heading_startup = bool(
            self.get_parameter("enable_route_heading_startup").value
        )
        self._route_heading_max_distance_m = float(
            self.get_parameter("route_heading_startup_max_distance_m").value
        )
        self._route_heading_neighbor_stride = int(
            self.get_parameter("route_heading_startup_neighbor_stride").value
        )
        self._route_heading_yaw_variance = float(
            self.get_parameter("route_heading_startup_yaw_variance").value
        )
        self._route_heading_snap_xy = bool(
            self.get_parameter("route_heading_startup_snap_xy").value
        )
        self._route_heading_prefer_start_within_m = float(
            self.get_parameter("route_heading_startup_prefer_start_within_m").value
        )
        self._route_samples = load_route_xy_samples(
            str(self.get_parameter("route_heading_startup_samples_csv").value)
        )
        if self._enable_route_heading_startup:
            self.get_logger().info(
                "route-heading startup enabled: "
                f"samples={len(self._route_samples)} "
                f"max_distance={self._route_heading_max_distance_m:.1f}m "
                f"snap_xy={self._route_heading_snap_xy} "
                f"prefer_start_within={self._route_heading_prefer_start_within_m:.1f}m"
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

        initial_pose, route_heading_applied, route_distance, route_index = (
            replace_startup_pose_yaw_from_route(
                msg,
                self._route_samples,
                enabled=self._enable_route_heading_startup,
                max_distance_m=self._route_heading_max_distance_m,
                neighbor_stride=self._route_heading_neighbor_stride,
                yaw_variance=self._route_heading_yaw_variance,
                snap_xy_to_route=self._route_heading_snap_xy,
                prefer_start_within_m=self._route_heading_prefer_start_within_m,
            )
        )
        if route_heading_applied:
            self.get_logger().info(
                "startup initialize using route heading: "
                f"nearest_route_index={route_index} distance={route_distance:.2f}m"
            )

        request = InitializeLocalization.Request()
        request.method = self._initialize_method
        request.pose_with_covariance = [initial_pose]
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
