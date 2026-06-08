"""Deterministic trajectory and operation-mode publisher for the closed-loop bench."""

from __future__ import annotations

import math

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from autoracer_control.control_closed_loop_geometry import PathPoint, compute_stations
from autoracer_control.control_closed_loop_scenarios import (
    SCENARIO_SPECS,
    ScenarioSpec,
    get_scenario_spec,
    velocity_at_s,
)


REFERENCE_TRAJECTORY_TOPIC = "/control_bench/planning/trajectory"
OPERATION_MODE_TOPIC = "/control_bench/system/operation_mode/state"
SCENARIOS = set(SCENARIO_SPECS)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _duration_from_seconds(seconds: float):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    point = TrajectoryPoint()
    point.time_from_start.sec = sec
    point.time_from_start.nanosec = nanosec
    return point.time_from_start


def _finite(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("trajectory fixture generated a non-finite value")
    return value


def _point(x: float, y: float, yaw: float, v: float, curvature: float, t: float) -> TrajectoryPoint:
    point = TrajectoryPoint()
    point.pose.position.x = _finite(x)
    point.pose.position.y = _finite(y)
    point.pose.position.z = 0.0
    point.pose.orientation = _yaw_to_quaternion(_finite(yaw))
    point.longitudinal_velocity_mps = _finite(v)
    point.lateral_velocity_mps = 0.0
    point.acceleration_mps2 = 0.0
    point.heading_rate_rps = _finite(v * curvature)
    point.front_wheel_angle_rad = _finite(math.atan(1.9 * curvature))
    point.rear_wheel_angle_rad = 0.0
    point.time_from_start = _duration_from_seconds(t)
    return point


def _yaw_from_neighbors(points: list[tuple[float, float]], index: int) -> float:
    if index == 0:
        start, end = points[0], points[1]
    elif index == len(points) - 1:
        start, end = points[-2], points[-1]
    else:
        start, end = points[index - 1], points[index + 1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def _curvature_from_neighbors(points: list[tuple[float, float]], index: int) -> float:
    if index == 0 or index == len(points) - 1:
        return 0.0
    x1, y1 = points[index - 1]
    x2, y2 = points[index]
    x3, y3 = points[index + 1]
    a = math.hypot(x2 - x1, y2 - y1)
    b = math.hypot(x3 - x2, y3 - y2)
    c = math.hypot(x3 - x1, y3 - y1)
    denom = a * b * c
    if denom <= 1e-9:
        return 0.0
    signed_area2 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
    return 2.0 * signed_area2 / denom


def _sample_count(length: float, spacing: float) -> int:
    return max(2, int(math.ceil(length / spacing)) + 1)


def _straight_points(spec: ScenarioSpec) -> list[PathPoint]:
    count = _sample_count(spec.path_length_m, spec.point_spacing_m)
    return [
        PathPoint(spec.path_length_m * index / (count - 1), 0.0)
        for index in range(count)
    ]


def _arc_points(spec: ScenarioSpec) -> list[PathPoint]:
    arc_length = spec.radius_m * spec.arc_angle_rad
    count = _sample_count(arc_length, spec.point_spacing_m)
    points = []
    for index in range(count):
        fraction = index / (count - 1)
        theta = -math.pi * 0.5 + fraction * spec.arc_angle_rad
        points.append(
            PathPoint(
                spec.radius_m * math.cos(theta),
                spec.radius_m + spec.radius_m * math.sin(theta),
            )
        )
    return points


def _s_curve_points(spec: ScenarioSpec) -> list[PathPoint]:
    count = _sample_count(spec.path_length_m, spec.point_spacing_m)
    return [
        PathPoint(
            spec.path_length_m * index / (count - 1),
            spec.s_curve_amplitude_m
            * math.sin(
                2.0
                * math.pi
                * spec.path_length_m
                * index
                / ((count - 1) * spec.s_curve_wavelength_m)
            ),
        )
        for index in range(count)
    ]


def _scenario_points(spec: ScenarioSpec) -> list[PathPoint]:
    if spec.path_kind == "arc_left":
        return _arc_points(spec)
    if spec.path_kind == "s_curve":
        return _s_curve_points(spec)
    return _straight_points(spec)


class ControlClosedLoopFixturePublisher(Node):
    def __init__(self) -> None:
        super().__init__("control_closed_loop_fixture_publisher")
        self.declare_parameter("scenario", "straight_lateral_offset")
        self.declare_parameter("reference_trajectory_topic", REFERENCE_TRAJECTORY_TOPIC)
        self.declare_parameter("operation_mode_topic", OPERATION_MODE_TOPIC)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._scenario = str(self.get_parameter("scenario").value)
        self._scenario_spec = get_scenario_spec(self._scenario)

        self._trajectory_pub = self.create_publisher(
            Trajectory, str(self.get_parameter("reference_trajectory_topic").value), 10
        )
        operation_mode_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._operation_mode_pub = self.create_publisher(
            OperationModeState,
            str(self.get_parameter("operation_mode_topic").value),
            operation_mode_qos,
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(f"closed-loop fixture scenario={self._scenario}")

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        self._trajectory_pub.publish(self._make_trajectory(now))
        self._operation_mode_pub.publish(self._make_operation_mode(now))

    def _make_trajectory(self, now) -> Trajectory:
        path_points = _scenario_points(self._scenario_spec)
        stations = compute_stations(path_points)

        trajectory = Trajectory()
        trajectory.header.stamp = now.to_msg()
        trajectory.header.frame_id = "map"

        elapsed = 0.0
        prev_station = stations[0]
        xy_points = [(point.x_m, point.y_m) for point in path_points]
        for index, point_xy in enumerate(path_points):
            if index > 0:
                ds = stations[index] - prev_station
                elapsed += ds / max(velocity_at_s(self._scenario_spec, stations[index]), 0.1)
            yaw = _yaw_from_neighbors(xy_points, index)
            curvature = _curvature_from_neighbors(xy_points, index)
            velocity = velocity_at_s(self._scenario_spec, stations[index])
            trajectory.points.append(
                _point(point_xy.x_m, point_xy.y_m, yaw, velocity, curvature, elapsed)
            )
            prev_station = stations[index]

        return trajectory

    def _make_operation_mode(self, now) -> OperationModeState:
        operation_mode = OperationModeState()
        operation_mode.stamp = now.to_msg()
        operation_mode.mode = OperationModeState.AUTONOMOUS
        operation_mode.is_autoware_control_enabled = True
        return operation_mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlClosedLoopFixturePublisher()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
