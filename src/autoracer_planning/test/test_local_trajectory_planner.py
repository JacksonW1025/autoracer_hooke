import math

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose

from autoracer_planning.local_trajectory_planner import (
    LocalPlannerConfig,
    build_local_trajectory,
    _is_shutdown_rcl_error,
    resample_points,
    sanitize_points,
    select_monotonic_nearest_index,
    slice_points_around_ego,
)


def _pose(x, y, yaw=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def _point(x, y, velocity=1.5, yaw=0.0):
    point = TrajectoryPoint()
    point.pose = _pose(x, y, yaw)
    point.longitudinal_velocity_mps = float(velocity)
    return point


def _trajectory(points):
    msg = Trajectory()
    msg.header.frame_id = "map"
    msg.points = [_point(*point) for point in points]
    return msg


def _duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def test_sanitize_points_removes_invalid_and_duplicate_points():
    points = [
        _point(0.0, 0.0),
        _point(0.01, 0.0),
        _point(math.nan, 0.0),
        _point(1.0, 0.0),
    ]

    sanitized = sanitize_points(points, min_point_distance_m=0.05)

    assert [(p.pose.position.x, p.pose.position.y) for p in sanitized] == [
        (0.0, 0.0),
        (1.0, 0.0),
    ]


def test_slice_points_around_ego_keeps_backward_and_forward_window():
    points = [_point(float(x), 0.0) for x in range(11)]

    sliced, includes_goal = slice_points_around_ego(
        points,
        ego_pose=_pose(5.1, 0.0),
        backward_distance_m=2.0,
        lookahead_distance_m=3.0,
    )

    assert [round(p.pose.position.x) for p in sliced] == [3, 4, 5, 6, 7, 8]
    assert includes_goal is False


def test_select_monotonic_nearest_index_ignores_far_duplicate_crossing():
    points = [
        _point(91.4, -9.0),
        _point(91.4, -6.6),
        _point(101.7, 4.4),
        _point(120.0, 4.4),
        _point(104.1, 184.4),
        _point(101.4, 4.4),
        _point(91.5, 194.5),
    ]

    nearest = select_monotonic_nearest_index(
        points,
        ego_pose=_pose(98.0, 0.4),
        previous_nearest_index=1,
        max_backward_distance_m=2.0,
        max_forward_distance_m=80.0,
    )

    assert nearest == 2


def test_resample_points_uses_fixed_distance_interval():
    points = [_point(0.0, 0.0), _point(3.0, 0.0)]

    resampled = resample_points(points, interval_m=1.0)

    assert [round(p.pose.position.x, 3) for p in resampled] == [0.0, 1.0, 2.0, 3.0]


def test_build_local_trajectory_limits_speed_and_stops_at_goal():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        resample_interval_m=1.0,
        max_speed_mps=2.0,
        max_accel_mps2=0.5,
        max_decel_mps2=-1.0,
    )
    global_trajectory = _trajectory([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])

    local = build_local_trajectory(
        global_trajectory,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=0.0,
        config=config,
    )

    assert local.header.frame_id == "map"
    assert len(local.points) > 2
    assert local.points[-1].longitudinal_velocity_mps == 0.0
    assert max(p.longitudinal_velocity_mps for p in local.points) <= 2.0
    assert all(
        _duration_seconds(curr.time_from_start) >= _duration_seconds(prev.time_from_start)
        for prev, curr in zip(local.points, local.points[1:])
    )


def test_build_local_trajectory_reduces_speed_on_curves():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        resample_interval_m=1.0,
        max_speed_mps=5.0,
        max_lateral_accel_mps2=0.5,
    )
    global_trajectory = _trajectory(
        [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (3.0, 3.0), (3.0, 5.0)]
    )

    local = build_local_trajectory(
        global_trajectory,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=0.0,
        config=config,
    )

    speeds = [p.longitudinal_velocity_mps for p in local.points]
    assert min(speeds[1:-1]) < config.max_speed_mps


def test_shutdown_rcl_error_matches_invalid_publisher_context():
    exc = RuntimeError("Failed to publish: publisher's context is invalid")

    assert _is_shutdown_rcl_error(exc)
