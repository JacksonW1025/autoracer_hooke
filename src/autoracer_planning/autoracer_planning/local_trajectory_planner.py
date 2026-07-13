import bisect
import copy
from dataclasses import dataclass
import math
import time

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_planning_msgs.msg import RouteState, Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class LocalPlannerConfig:
    lookahead_distance_m: float = 40.0
    backward_distance_m: float = 2.0
    resample_interval_m: float = 0.5
    min_point_distance_m: float = 0.05
    max_speed_mps: float = 5.0
    max_lateral_accel_mps2: float = 1.5
    max_accel_mps2: float = 0.8
    max_decel_mps2: float = -1.5
    goal_tolerance_m: float = 1.0
    nearest_search_backward_distance_m: float = 0.0
    nearest_search_forward_distance_m: float = 3.0
    nearest_search_forward_time_sec: float = 0.35
    initial_search_distance_m: float = 30.0
    nearest_position_gate_m: float = 3.0
    heading_gate_rad: float = math.radians(60.0)
    z_gate_m: float = 3.0
    command_latency_sec: float = 0.2
    stopping_margin_m: float = 5.0
    departure_speed_mps: float = 0.1


@dataclass(frozen=True)
class PreparedTrajectory:
    header: object
    points: tuple
    distances: tuple


def source_stamp_is_fresh(
    source_stamp_sec: float,
    now_sec: float,
    max_age_sec: float,
    future_tolerance_sec: float = 0.05,
) -> bool:
    age_sec = now_sec - source_stamp_sec
    return (
        math.isfinite(source_stamp_sec)
        and math.isfinite(now_sec)
        and math.isfinite(max_age_sec)
        and max_age_sec >= 0.0
        and -future_tolerance_sec <= age_sec <= max_age_sec
    )


def sanitize_points(points, min_point_distance_m: float):
    sanitized = []
    for point in points:
        if not _is_finite_point(point):
            continue
        if sanitized and _point_distance(sanitized[-1], point) < min_point_distance_m:
            continue
        sanitized.append(copy.deepcopy(point))
    return sanitized


def prepare_global_trajectory(
    global_trajectory: Trajectory, min_point_distance_m: float = 0.05
) -> PreparedTrajectory:
    points = sanitize_points(global_trajectory.points, min_point_distance_m)
    if len(points) < 2:
        raise ValueError("global trajectory has fewer than two valid points")
    _update_orientations(points)
    return PreparedTrajectory(
        header=copy.deepcopy(global_trajectory.header),
        points=tuple(points),
        distances=tuple(_cumulative_distances(points)),
    )


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
    max_backward_distance_m: float = 0.0,
    max_forward_distance_m: float = 15.0,
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
    start_index = max(0, bisect.bisect_right(distances, start_s) - 1)
    end_index = min(len(points) - 1, bisect.bisect_left(distances, end_s))
    return (
        [copy.deepcopy(point) for point in points[start_index : end_index + 1]],
        end_index == len(points) - 1,
        nearest_index,
    )


def select_monotonic_nearest_index(
    points,
    ego_pose: Pose,
    previous_nearest_index=None,
    max_backward_distance_m: float = 0.0,
    max_forward_distance_m: float = 15.0,
    distances=None,
):
    del max_backward_distance_m
    if not points:
        return None
    if distances is None:
        distances = _cumulative_distances(points)
    if previous_nearest_index is None or not (0 <= previous_nearest_index < len(points)):
        candidates = range(len(points))
    else:
        max_s = distances[previous_nearest_index] + max(max_forward_distance_m, 0.0)
        end_index = bisect.bisect_right(distances, max_s)
        candidates = range(previous_nearest_index, max(previous_nearest_index + 1, end_index))
    return min(candidates, key=lambda index: _pose_distance(points[index].pose, ego_pose))


def select_progress_index(
    prepared: PreparedTrajectory,
    ego_pose: Pose,
    previous_index,
    config: LocalPlannerConfig,
    forward_distance_m=None,
):
    if previous_index is None:
        end_index = max(
            1,
            bisect.bisect_right(prepared.distances, config.initial_search_distance_m),
        )
        candidates = range(0, min(end_index, len(prepared.points)))
    else:
        previous_index = min(max(int(previous_index), 0), len(prepared.points) - 1)
        forward_distance = (
            config.nearest_search_forward_distance_m
            if forward_distance_m is None
            else max(float(forward_distance_m), 0.0)
        )
        max_s = (
            prepared.distances[previous_index]
            + forward_distance
        )
        end_index = bisect.bisect_right(prepared.distances, max_s)
        candidates = range(previous_index, max(previous_index + 1, end_index))

    ego_yaw = _yaw_from_quaternion(ego_pose.orientation)
    gated = []
    for index in candidates:
        point = prepared.points[index]
        dz = abs(point.pose.position.z - ego_pose.position.z)
        position_error = _pose_distance(point.pose, ego_pose)
        point_yaw = _yaw_from_quaternion(point.pose.orientation)
        heading_error = abs(_normalize_angle(point_yaw - ego_yaw))
        if (
            position_error <= config.nearest_position_gate_m
            and dz <= config.z_gate_m
            and heading_error <= config.heading_gate_rad
        ):
            gated.append(index)
    if not gated:
        return None
    return min(gated, key=lambda index: _pose_distance(prepared.points[index].pose, ego_pose))


def progress_search_forward_distance(current_speed_mps, config: LocalPlannerConfig):
    return max(
        config.nearest_search_forward_distance_m,
        max(float(current_speed_mps), 0.0) * config.nearest_search_forward_time_sec,
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


def dynamic_horizon_distance(current_speed_mps: float, config: LocalPlannerConfig) -> float:
    speed = max(0.0, current_speed_mps)
    decel = max(abs(config.max_decel_mps2), 1e-3)
    stopping_distance = speed * speed / (2.0 * decel)
    latency_distance = speed * max(config.command_latency_sec, 0.0)
    return max(
        config.lookahead_distance_m,
        stopping_distance + latency_distance + max(config.stopping_margin_m, 0.0),
    )


def arrival_conditions_met(
    prepared: PreparedTrajectory,
    nearest_index: int,
    ego_pose: Pose,
    speed_mps: float,
    distance_limit_m: float,
    speed_limit_mps: float,
) -> bool:
    endpoint = prepared.points[-1].pose.position
    endpoint_distance = math.hypot(
        endpoint.x - ego_pose.position.x,
        endpoint.y - ego_pose.position.y,
    )
    remaining = prepared.distances[-1] - prepared.distances[nearest_index]
    return (
        remaining <= distance_limit_m
        and endpoint_distance <= distance_limit_m
        and speed_mps < speed_limit_mps
    )


def build_local_from_prepared(
    prepared: PreparedTrajectory,
    ego_pose: Pose,
    current_speed_mps: float,
    config: LocalPlannerConfig,
    previous_nearest_index=None,
    velocity_limit_mps=None,
):
    output = Trajectory()
    output.header = copy.deepcopy(prepared.header)
    nearest_index = select_progress_index(
        prepared,
        ego_pose,
        previous_nearest_index,
        config,
        progress_search_forward_distance(current_speed_mps, config),
    )
    if nearest_index is None:
        return output, None, False

    ego_s = prepared.distances[nearest_index]
    start_s = max(0.0, ego_s - max(config.backward_distance_m, 0.0))
    end_s = min(
        prepared.distances[-1],
        ego_s + dynamic_horizon_distance(current_speed_mps, config),
    )
    start_index = max(0, bisect.bisect_right(prepared.distances, start_s) - 1)
    end_index = min(
        len(prepared.points) - 1,
        bisect.bisect_left(prepared.distances, end_s),
    )
    includes_goal = end_index == len(prepared.points) - 1
    points = [
        copy.deepcopy(point)
        for point in prepared.points[start_index : end_index + 1]
    ]
    if len(points) < 2:
        return output, nearest_index, includes_goal
    _apply_local_velocity_envelope(
        points,
        current_speed_mps,
        config,
        includes_goal,
        velocity_limit_mps,
    )
    _update_time_from_start(points)
    output.points = points
    return output, nearest_index, includes_goal


def build_local_trajectory(
    global_trajectory: Trajectory,
    ego_pose: Pose,
    current_speed_mps: float,
    config: LocalPlannerConfig,
) -> Trajectory:
    trajectory, _ = build_local_trajectory_with_nearest(
        global_trajectory, ego_pose, current_speed_mps, config
    )
    return trajectory


def build_local_trajectory_with_nearest(
    global_trajectory: Trajectory,
    ego_pose: Pose,
    current_speed_mps: float,
    config: LocalPlannerConfig,
    previous_nearest_index=None,
):
    try:
        prepared = prepare_global_trajectory(
            global_trajectory, config.min_point_distance_m
        )
    except ValueError:
        output = Trajectory()
        output.header = copy.deepcopy(global_trajectory.header)
        return output, previous_nearest_index
    trajectory, nearest_index, _ = build_local_from_prepared(
        prepared,
        ego_pose,
        current_speed_mps,
        config,
        previous_nearest_index,
    )
    return trajectory, nearest_index


class LocalTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__("local_trajectory_planner")
        self.declare_parameter("global_trajectory_topic", "/planning/global_trajectory")
        self.declare_parameter("odometry_topic", "/localization/kinematic_state")
        self.declare_parameter(
            "localization_state_topic", "/api/localization/initialization_state"
        )
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("route_state_topic", "/planning/route_state")
        self.declare_parameter("marker_topic", "/planning/local_trajectory_marker")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("lookahead_distance_m", 40.0)
        self.declare_parameter("backward_distance_m", 2.0)
        self.declare_parameter("resample_interval_m", 0.5)
        self.declare_parameter("min_point_distance_m", 0.05)
        self.declare_parameter("max_speed_mps", 5.0)
        self.declare_parameter("max_lateral_accel_mps2", 1.5)
        self.declare_parameter("max_accel_mps2", 0.8)
        self.declare_parameter("max_decel_mps2", -1.5)
        self.declare_parameter("goal_tolerance_m", 1.0)
        self.declare_parameter("nearest_search_forward_distance_m", 3.0)
        self.declare_parameter("nearest_search_forward_time_sec", 0.35)
        self.declare_parameter("initial_search_distance_m", 30.0)
        self.declare_parameter("nearest_position_gate_m", 3.0)
        self.declare_parameter("heading_gate_rad", math.radians(60.0))
        self.declare_parameter("z_gate_m", 3.0)
        self.declare_parameter("command_latency_sec", 0.2)
        self.declare_parameter("stopping_margin_m", 5.0)
        self.declare_parameter("departure_speed_mps", 0.1)
        self.declare_parameter("arrived_distance_m", 2.0)
        self.declare_parameter("arrived_speed_mps", 0.1)
        self.declare_parameter("arrived_dwell_sec", 2.0)
        self.declare_parameter("odometry_timeout_sec", 0.35)
        self.declare_parameter("publish_markers", False)
        self.declare_parameter("diagnostic_log_period_sec", 1.0)

        self._config = LocalPlannerConfig(
            lookahead_distance_m=float(self.get_parameter("lookahead_distance_m").value),
            backward_distance_m=float(self.get_parameter("backward_distance_m").value),
            resample_interval_m=float(self.get_parameter("resample_interval_m").value),
            min_point_distance_m=float(self.get_parameter("min_point_distance_m").value),
            max_speed_mps=float(self.get_parameter("max_speed_mps").value),
            max_lateral_accel_mps2=float(
                self.get_parameter("max_lateral_accel_mps2").value
            ),
            max_accel_mps2=float(self.get_parameter("max_accel_mps2").value),
            max_decel_mps2=float(self.get_parameter("max_decel_mps2").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            nearest_search_forward_distance_m=float(
                self.get_parameter("nearest_search_forward_distance_m").value
            ),
            nearest_search_forward_time_sec=float(
                self.get_parameter("nearest_search_forward_time_sec").value
            ),
            initial_search_distance_m=float(
                self.get_parameter("initial_search_distance_m").value
            ),
            nearest_position_gate_m=float(
                self.get_parameter("nearest_position_gate_m").value
            ),
            heading_gate_rad=float(self.get_parameter("heading_gate_rad").value),
            z_gate_m=float(self.get_parameter("z_gate_m").value),
            command_latency_sec=float(self.get_parameter("command_latency_sec").value),
            stopping_margin_m=float(self.get_parameter("stopping_margin_m").value),
            departure_speed_mps=float(self.get_parameter("departure_speed_mps").value),
        )
        self._prepared = None
        self._odometry = None
        self._nearest_index = None
        self._last_odom_stamp = -1.0
        self._route_state = RouteState.UNSET
        self._localization_initialized = False
        self._arrival_candidate_since = None
        self._last_diagnostic_time = None

        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trajectory_pub = self.create_publisher(
            Trajectory, str(self.get_parameter("trajectory_topic").value), volatile_qos
        )
        self._route_state_pub = self.create_publisher(
            RouteState, str(self.get_parameter("route_state_topic").value), state_qos
        )
        self._marker_pub = None
        if bool(self.get_parameter("publish_markers").value):
            self._marker_pub = self.create_publisher(
                MarkerArray, str(self.get_parameter("marker_topic").value), volatile_qos
            )

        self.create_subscription(
            Trajectory,
            str(self.get_parameter("global_trajectory_topic").value),
            self._on_global_trajectory,
            state_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(
            LocalizationInitializationState,
            str(self.get_parameter("localization_state_topic").value),
            self._on_localization_state,
            state_qos,
        )
        period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(period, self._on_timer)
        self._publish_route_state(RouteState.UNSET)

    def _on_global_trajectory(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().error(
                f"Rejected global trajectory frame: {msg.header.frame_id!r}"
            )
            self._publish_route_state(RouteState.ABORTED)
            return
        try:
            self._prepared = prepare_global_trajectory(
                msg, self._config.min_point_distance_m
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._publish_route_state(RouteState.ABORTED)
            return
        self._nearest_index = None
        self._arrival_candidate_since = None
        self._publish_route_state(RouteState.SET)
        self.get_logger().info(
            f"Cached fixed course once: points={len(self._prepared.points)} "
            f"length_m={self._prepared.distances[-1]:.3f}"
        )

    def _on_odometry(self, msg):
        stamp = _stamp_seconds(msg.header.stamp)
        if msg.header.frame_id != "map" or (
            msg.child_frame_id and msg.child_frame_id != "base_link"
        ):
            return
        if not _is_finite_pose(msg.pose.pose) or not math.isfinite(
            float(msg.twist.twist.linear.x)
        ):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if stamp < self._last_odom_stamp or stamp > now + 0.05:
            return
        self._last_odom_stamp = stamp
        self._odometry = msg

    def _on_localization_state(self, msg):
        self._localization_initialized = (
            msg.state == LocalizationInitializationState.INITIALIZED
        )

    def _on_timer(self):
        if (
            self._prepared is None
            or self._odometry is None
            or not self._localization_initialized
        ):
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if not source_stamp_is_fresh(
            self._last_odom_stamp,
            now_sec,
            float(self.get_parameter("odometry_timeout_sec").value),
        ):
            return
        started = time.perf_counter()
        speed = abs(float(self._odometry.twist.twist.linear.x))
        effective_limit = self._config.max_speed_mps
        trajectory, nearest_index, _ = build_local_from_prepared(
            self._prepared,
            self._odometry.pose.pose,
            speed,
            self._config,
            self._nearest_index,
            effective_limit,
        )
        if nearest_index is not None:
            self._nearest_index = nearest_index
        if len(trajectory.points) < 2:
            return
        trajectory.header.stamp = self.get_clock().now().to_msg()
        self._trajectory_pub.publish(trajectory)
        if self._marker_pub is not None:
            self._publish_marker(trajectory)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._update_arrival(speed)
        self._maybe_log_diagnostics(trajectory, effective_limit, elapsed_ms)

    def _update_arrival(self, speed):
        if self._prepared is None or self._nearest_index is None:
            return
        ready = arrival_conditions_met(
            self._prepared,
            self._nearest_index,
            self._odometry.pose.pose,
            speed,
            float(self.get_parameter("arrived_distance_m").value),
            float(self.get_parameter("arrived_speed_mps").value),
        )
        now = self.get_clock().now().nanoseconds * 1e-9
        if not ready:
            self._arrival_candidate_since = None
            if self._route_state == RouteState.ARRIVED:
                self._publish_route_state(RouteState.SET)
            return
        if self._arrival_candidate_since is None:
            self._arrival_candidate_since = now
            return
        if now - self._arrival_candidate_since >= float(
            self.get_parameter("arrived_dwell_sec").value
        ):
            self._publish_route_state(RouteState.ARRIVED)

    def _publish_route_state(self, state):
        if state == self._route_state and state != RouteState.UNSET:
            return
        self._route_state = state
        message = RouteState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = state
        self._route_state_pub.publish(message)

    def _maybe_log_diagnostics(self, trajectory, effective_limit, elapsed_ms):
        now = self.get_clock().now()
        period = float(self.get_parameter("diagnostic_log_period_sec").value)
        if self._last_diagnostic_time is not None:
            elapsed = (now - self._last_diagnostic_time).nanoseconds * 1e-9
            if elapsed < period:
                return
        self._last_diagnostic_time = now
        speeds = [point.longitudinal_velocity_mps for point in trajectory.points]
        self.get_logger().info(
            "Planning shadow: "
            f"progress_index={self._nearest_index} points={len(trajectory.points)} "
            f"speed_mps=({min(speeds):.2f},{max(speeds):.2f}) "
            f"limit_mps={effective_limit:.2f} cycle_ms={elapsed_ms:.2f}"
        )

    def _publish_marker(self, trajectory):
        marker = Marker()
        marker.header = trajectory.header
        marker.ns = "autoracer_local_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.color.r = 0.65
        marker.color.g = 0.1
        marker.color.b = 1.0
        marker.color.a = 0.95
        for trajectory_point in trajectory.points:
            point = Point()
            point.x = trajectory_point.pose.position.x
            point.y = trajectory_point.pose.position.y
            point.z = trajectory_point.pose.position.z + 0.15
            marker.points.append(point)
        self._marker_pub.publish(MarkerArray(markers=[marker]))


def _is_finite_point(point):
    return _is_finite_pose(point.pose) and all(
        math.isfinite(float(value))
        for value in (
            point.longitudinal_velocity_mps,
            point.acceleration_mps2,
        )
    )


def _is_finite_pose(pose):
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    return all(math.isfinite(float(value)) for value in values) and norm > 1e-6


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
    index = bisect.bisect_right(distances, target_s) - 1
    index = min(max(index, 0), len(points) - 2)
    start_s = distances[index]
    end_s = distances[index + 1]
    ratio = 0.0 if end_s <= start_s else (target_s - start_s) / (end_s - start_s)
    return _interpolate_point(points[index], points[index + 1], ratio)


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
    point.acceleration_mps2 = _lerp(
        start.acceleration_mps2,
        end.acceleration_mps2,
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
            dx = points[index + 1].pose.position.x - point.pose.position.x
            dy = points[index + 1].pose.position.y - point.pose.position.y
        else:
            dx = point.pose.position.x - points[index - 1].pose.position.x
            dy = point.pose.position.y - points[index - 1].pose.position.y
        point.pose.orientation = _yaw_to_quaternion(math.atan2(dy, dx))


def _yaw_to_quaternion(yaw):
    pose = Pose()
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose.orientation


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _apply_local_velocity_envelope(
    points,
    current_speed_mps,
    config,
    includes_goal,
    velocity_limit_mps,
):
    effective_limit = config.max_speed_mps
    if velocity_limit_mps is not None:
        effective_limit = min(effective_limit, max(0.0, velocity_limit_mps))
    speeds = [
        min(effective_limit, max(0.0, float(point.longitudinal_velocity_mps)))
        for point in points
    ]
    speeds[0] = min(speeds[0], max(0.0, current_speed_mps))
    if speeds[0] <= 1e-3:
        next_positive_speed = next((speed for speed in speeds[1:] if speed > 1e-3), 0.0)
        if next_positive_speed > 0.0:
            speeds[0] = min(
                next_positive_speed,
                effective_limit,
                max(max(0.0, current_speed_mps), config.departure_speed_mps),
            )
    if includes_goal:
        speeds[-1] = 0.0
    decel = abs(config.max_decel_mps2)
    for _ in range(2):
        for index in range(1, len(points)):
            ds = _point_distance(points[index - 1], points[index])
            limit = math.sqrt(
                max(0.0, speeds[index - 1] ** 2 + 2.0 * config.max_accel_mps2 * ds)
            )
            speeds[index] = min(speeds[index], limit)
        if includes_goal:
            speeds[-1] = 0.0
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
        point.acceleration_mps2 = (
            speeds[index] ** 2 - speeds[index - 1] ** 2
        ) / (2.0 * ds)


def _update_time_from_start(points):
    elapsed = 0.0
    points[0].time_from_start = _duration_from_seconds(elapsed)
    for previous, current in zip(points, points[1:]):
        ds = _point_distance(previous, current)
        average_speed = 0.5 * (
            previous.longitudinal_velocity_mps + current.longitudinal_velocity_mps
        )
        elapsed += ds / max(average_speed, 0.1)
        current.time_from_start = _duration_from_seconds(elapsed)


def _duration_from_seconds(seconds):
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1_000_000_000))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000
    return duration


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _shutdown_if_context_ok():
    if not rclpy.ok():
        return
    try:
        rclpy.shutdown()
    except RCLError as exc:
        if not _is_shutdown_rcl_error(exc):
            raise


def _is_shutdown_rcl_error(exc: Exception) -> bool:
    text = str(exc)
    return (
        "rcl_shutdown already called" in text
        or "context is not valid" in text
        or "rcl_init() was not called or rcl_shutdown() was called" in text
        or "publisher's context is invalid" in text
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
