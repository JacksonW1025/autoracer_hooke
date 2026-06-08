import copy
from dataclasses import dataclass
import math

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class LocalPlannerConfig:
    lookahead_distance_m: float = 40.0
    backward_distance_m: float = 2.0
    resample_interval_m: float = 0.5
    min_point_distance_m: float = 0.05
    max_speed_mps: float = 1.5
    max_lateral_accel_mps2: float = 1.5
    max_accel_mps2: float = 0.8
    max_decel_mps2: float = -1.5
    goal_tolerance_m: float = 1.0
    nearest_search_backward_distance_m: float = 5.0
    nearest_search_forward_distance_m: float = 120.0


def sanitize_points(points, min_point_distance_m: float):
    sanitized = []
    for point in points:
        if not _is_finite_point(point):
            continue
        if sanitized and _point_distance(sanitized[-1], point) < min_point_distance_m:
            continue
        sanitized.append(copy.deepcopy(point))
    return sanitized


def slice_points_around_ego(
    points,
    ego_pose: Pose,
    backward_distance_m: float,
    lookahead_distance_m: float,
):
    sliced, includes_goal, _ = slice_points_around_ego_with_nearest(
        points,
        ego_pose,
        backward_distance_m,
        lookahead_distance_m,
    )
    return sliced, includes_goal


def slice_points_around_ego_with_nearest(
    points,
    ego_pose: Pose,
    backward_distance_m: float,
    lookahead_distance_m: float,
    previous_nearest_index=None,
    max_backward_distance_m: float = 5.0,
    max_forward_distance_m: float = 120.0,
):
    if len(points) < 2:
        return [], False, None

    distances = _cumulative_distances(points)
    nearest_index = select_monotonic_nearest_index(
        points,
        ego_pose,
        previous_nearest_index,
        max_backward_distance_m,
        max_forward_distance_m,
        distances,
    )
    ego_s = distances[nearest_index]
    start_s = max(0.0, ego_s - max(backward_distance_m, 0.0))
    end_s = min(distances[-1], ego_s + max(lookahead_distance_m, 0.0))

    start_index = nearest_index
    while start_index > 0 and distances[start_index] > start_s:
        start_index -= 1

    end_index = nearest_index
    while end_index < len(points) - 1 and distances[end_index] < end_s:
        end_index += 1

    sliced = [copy.deepcopy(point) for point in points[start_index : end_index + 1]]
    return sliced, end_index == len(points) - 1, nearest_index


def select_monotonic_nearest_index(
    points,
    ego_pose: Pose,
    previous_nearest_index=None,
    max_backward_distance_m: float = 5.0,
    max_forward_distance_m: float = 120.0,
    distances=None,
):
    if not points:
        return None

    if distances is None:
        distances = _cumulative_distances(points)

    if previous_nearest_index is None or not (0 <= previous_nearest_index < len(points)):
        candidates = range(len(points))
    else:
        previous_s = distances[previous_nearest_index]
        min_s = previous_s - max(max_backward_distance_m, 0.0)
        max_s = previous_s + max(max_forward_distance_m, 0.0)
        candidates = [
            index
            for index, distance in enumerate(distances)
            if min_s <= distance <= max_s
        ]
        if not candidates:
            candidates = range(len(points))

    return min(
        candidates,
        key=lambda index: _pose_distance(points[index].pose, ego_pose),
    )


def resample_points(points, interval_m: float):
    if len(points) < 2:
        return [copy.deepcopy(point) for point in points]

    interval = max(interval_m, 0.01)
    distances = _cumulative_distances(points)
    total_distance = distances[-1]
    if total_distance <= 0.0:
        return [copy.deepcopy(points[0])]

    samples = []
    target_s = 0.0
    while target_s < total_distance:
        samples.append(_interpolate_at(points, distances, target_s))
        target_s += interval
    samples.append(copy.deepcopy(points[-1]))
    _update_orientations(samples)
    return samples


def build_local_trajectory(
    global_trajectory: Trajectory,
    ego_pose: Pose,
    current_speed_mps: float,
    config: LocalPlannerConfig,
) -> Trajectory:
    trajectory, _ = build_local_trajectory_with_nearest(
        global_trajectory,
        ego_pose,
        current_speed_mps,
        config,
    )
    return trajectory


def build_local_trajectory_with_nearest(
    global_trajectory: Trajectory,
    ego_pose: Pose,
    current_speed_mps: float,
    config: LocalPlannerConfig,
    previous_nearest_index=None,
):
    output = Trajectory()
    output.header = copy.deepcopy(global_trajectory.header)

    points = sanitize_points(global_trajectory.points, config.min_point_distance_m)
    points, includes_goal, nearest_index = slice_points_around_ego_with_nearest(
        points,
        ego_pose,
        config.backward_distance_m,
        config.lookahead_distance_m,
        previous_nearest_index,
        config.nearest_search_backward_distance_m,
        config.nearest_search_forward_distance_m,
    )
    points = resample_points(points, config.resample_interval_m)
    if len(points) < 2:
        return output, nearest_index

    _apply_velocity_profile(points, current_speed_mps, config, includes_goal)
    _update_time_from_start(points)
    output.points = points
    return output, nearest_index


class LocalTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__("local_trajectory_planner")
        self.declare_parameter("global_trajectory_topic", "/planning/global_trajectory")
        self.declare_parameter("pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("marker_topic", "/planning/local_trajectory_marker")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("lookahead_distance_m", 40.0)
        self.declare_parameter("backward_distance_m", 2.0)
        self.declare_parameter("resample_interval_m", 0.5)
        self.declare_parameter("min_point_distance_m", 0.05)
        self.declare_parameter("max_speed_mps", 1.5)
        self.declare_parameter("max_lateral_accel_mps2", 1.5)
        self.declare_parameter("max_accel_mps2", 0.8)
        self.declare_parameter("max_decel_mps2", -1.5)
        self.declare_parameter("goal_tolerance_m", 1.0)
        self.declare_parameter("nearest_search_backward_distance_m", 5.0)
        self.declare_parameter("nearest_search_forward_distance_m", 120.0)
        self.declare_parameter("diagnostic_log_period_sec", 1.0)

        self._global_trajectory = None
        self._pose = None
        self._speed = 0.0
        self._nearest_index = None
        self._last_diagnostic_time = None

        self._trajectory_pub = self.create_publisher(
            Trajectory, self.get_parameter("trajectory_topic").value, 1
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 1
        )

        self.create_subscription(
            Trajectory,
            self.get_parameter("global_trajectory_topic").value,
            self._on_global_trajectory,
            1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            10,
        )
        self.create_subscription(
            VelocityReport,
            self.get_parameter("velocity_topic").value,
            self._on_velocity,
            10,
        )

        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(period, self._on_timer)

    def _on_global_trajectory(self, msg):
        self._global_trajectory = msg
        self._nearest_index = None

    def _on_pose(self, msg):
        self._pose = msg.pose.pose

    def _on_velocity(self, msg):
        self._speed = float(msg.longitudinal_velocity)

    def _on_timer(self):
        if self._global_trajectory is None or self._pose is None:
            return

        trajectory, nearest_index = build_local_trajectory_with_nearest(
            self._global_trajectory,
            self._pose,
            self._speed,
            self._config(),
            self._nearest_index,
        )
        if nearest_index is not None:
            self._nearest_index = nearest_index
        trajectory.header.stamp = self.get_clock().now().to_msg()
        self._maybe_log_diagnostics(trajectory, nearest_index)
        self._trajectory_pub.publish(trajectory)
        self._publish_marker(trajectory)

    def _config(self):
        return LocalPlannerConfig(
            lookahead_distance_m=float(self.get_parameter("lookahead_distance_m").value),
            backward_distance_m=float(self.get_parameter("backward_distance_m").value),
            resample_interval_m=float(self.get_parameter("resample_interval_m").value),
            min_point_distance_m=float(self.get_parameter("min_point_distance_m").value),
            max_speed_mps=float(self.get_parameter("max_speed_mps").value),
            max_lateral_accel_mps2=float(self.get_parameter("max_lateral_accel_mps2").value),
            max_accel_mps2=float(self.get_parameter("max_accel_mps2").value),
            max_decel_mps2=float(self.get_parameter("max_decel_mps2").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            nearest_search_backward_distance_m=float(
                self.get_parameter("nearest_search_backward_distance_m").value
            ),
            nearest_search_forward_distance_m=float(
                self.get_parameter("nearest_search_forward_distance_m").value
            ),
        )

    def _maybe_log_diagnostics(self, trajectory, nearest_index):
        now = self.get_clock().now()
        period = float(self.get_parameter("diagnostic_log_period_sec").value)
        if period <= 0.0:
            return
        if self._last_diagnostic_time is not None:
            elapsed = (now - self._last_diagnostic_time).nanoseconds * 1e-9
            if elapsed < period:
                return

        self._last_diagnostic_time = now
        speeds = [point.longitudinal_velocity_mps for point in trajectory.points]
        if speeds:
            speed_range = (min(speeds), max(speeds))
        else:
            speed_range = (0.0, 0.0)

        self.get_logger().info(
            "Stage B local planner diagnostics: "
            f"nearest_index={nearest_index} "
            f"trajectory_points={len(trajectory.points)} "
            f"speed_range_mps=({speed_range[0]:.2f},{speed_range[1]:.2f}) "
            f"ego_speed_mps={self._speed:.2f}"
        )

    def _publish_marker(self, trajectory):
        marker = Marker()
        marker.header = trajectory.header
        marker.ns = "autoracer_local_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.1
        marker.color.a = 0.95
        for trajectory_point in trajectory.points:
            point = Point()
            point.x = trajectory_point.pose.position.x
            point.y = trajectory_point.pose.position.y
            point.z = trajectory_point.pose.position.z + 0.35
            marker.points.append(point)
        self._marker_pub.publish(MarkerArray(markers=[marker]))


def _is_finite_point(point):
    values = (
        point.pose.position.x,
        point.pose.position.y,
        point.pose.position.z,
        point.pose.orientation.x,
        point.pose.orientation.y,
        point.pose.orientation.z,
        point.pose.orientation.w,
        point.longitudinal_velocity_mps,
    )
    return all(math.isfinite(float(value)) for value in values)


def _point_distance(a, b):
    return _pose_distance(a.pose, b.pose)


def _pose_distance(a, b):
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


def _cumulative_distances(points):
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(distances[-1] + _point_distance(previous, current))
    return distances


def _interpolate_at(points, distances, target_s):
    for index in range(len(distances) - 1):
        next_s = distances[index + 1]
        if target_s <= next_s:
            prev_s = distances[index]
            ratio = 0.0 if next_s == prev_s else (target_s - prev_s) / (next_s - prev_s)
            return _interpolate_point(points[index], points[index + 1], ratio)
    return copy.deepcopy(points[-1])


def _interpolate_point(start, end, ratio):
    point = copy.deepcopy(start)
    point.pose.position.x = _lerp(start.pose.position.x, end.pose.position.x, ratio)
    point.pose.position.y = _lerp(start.pose.position.y, end.pose.position.y, ratio)
    point.pose.position.z = _lerp(start.pose.position.z, end.pose.position.z, ratio)
    point.longitudinal_velocity_mps = _lerp(
        start.longitudinal_velocity_mps,
        end.longitudinal_velocity_mps,
        ratio,
    )
    return point


def _lerp(start, end, ratio):
    return float(start) + (float(end) - float(start)) * ratio


def _update_orientations(points):
    if len(points) < 2:
        return
    for index, point in enumerate(points):
        if index < len(points) - 1:
            other = points[index + 1]
        else:
            other = points[index - 1]
        yaw = math.atan2(
            other.pose.position.y - point.pose.position.y,
            other.pose.position.x - point.pose.position.x,
        )
        point.pose.orientation = _yaw_to_quaternion(yaw)


def _yaw_to_quaternion(yaw):
    pose = Pose()
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose.orientation


def _apply_velocity_profile(points, current_speed_mps, config, includes_goal):
    speeds = []
    for index, point in enumerate(points):
        speed = config.max_speed_mps
        if point.longitudinal_velocity_mps > 0.0:
            speed = min(speed, float(point.longitudinal_velocity_mps))
        curvature = _curvature_at(points, index)
        if curvature > 1e-6:
            speed = min(speed, math.sqrt(max(config.max_lateral_accel_mps2, 0.0) / curvature))
        speeds.append(max(0.0, speed))

    speeds[0] = min(speeds[0], max(0.0, current_speed_mps))
    for index in range(1, len(points)):
        ds = _point_distance(points[index - 1], points[index])
        limit = math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2.0 * config.max_accel_mps2 * ds))
        speeds[index] = min(speeds[index], limit)

    if includes_goal or _pose_distance(points[-1].pose, points[-2].pose) <= config.goal_tolerance_m:
        speeds[-1] = 0.0
    decel = abs(config.max_decel_mps2)
    for index in range(len(points) - 2, -1, -1):
        ds = _point_distance(points[index], points[index + 1])
        limit = math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2.0 * decel * ds))
        speeds[index] = min(speeds[index], limit)

    for index, point in enumerate(points):
        point.longitudinal_velocity_mps = float(speeds[index])
        if index == 0:
            point.acceleration_mps2 = 0.0
            continue
        ds = max(_point_distance(points[index - 1], point), 1e-3)
        point.acceleration_mps2 = (speeds[index] ** 2 - speeds[index - 1] ** 2) / (2.0 * ds)


def _curvature_at(points, index):
    if index <= 0 or index >= len(points) - 1:
        return 0.0
    previous = points[index - 1].pose.position
    current = points[index].pose.position
    next_point = points[index + 1].pose.position
    a = math.hypot(current.x - previous.x, current.y - previous.y)
    b = math.hypot(next_point.x - current.x, next_point.y - current.y)
    c = math.hypot(next_point.x - previous.x, next_point.y - previous.y)
    if min(a, b, c) < 1e-6:
        return 0.0
    area2 = abs(
        (current.x - previous.x) * (next_point.y - previous.y)
        - (current.y - previous.y) * (next_point.x - previous.x)
    )
    return 2.0 * area2 / (a * b * c)


def _update_time_from_start(points):
    elapsed = 0.0
    points[0].time_from_start = _duration_from_seconds(elapsed)
    for previous, current in zip(points, points[1:]):
        ds = _point_distance(previous, current)
        average_speed = max(
            (previous.longitudinal_velocity_mps + current.longitudinal_velocity_mps) * 0.5,
            0.1,
        )
        elapsed += ds / average_speed
        current.time_from_start = _duration_from_seconds(elapsed)


def _duration_from_seconds(seconds):
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
    return duration


def _shutdown_if_context_ok():
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise


def _is_shutdown_rcl_error(exc: RCLError) -> bool:
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "publisher's context is invalid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
    )


def main():
    rclpy.init()
    node = LocalTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise
    finally:
        node.destroy_node()
        _shutdown_if_context_ok()


if __name__ == "__main__":
    main()
