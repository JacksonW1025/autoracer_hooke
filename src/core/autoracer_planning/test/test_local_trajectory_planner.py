import math

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose

from autoracer_planning.local_trajectory_planner import (
    LocalPlannerConfig,
    _is_shutdown_rcl_error,
    arrival_conditions_met,
    build_local_from_prepared,
    prepare_global_trajectory,
    progress_search_forward_distance,
    sanitize_points,
    select_progress_index,
    source_stamp_is_fresh,
)


def _pose(x, y, yaw=0.0, z=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
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


def _build_local(global_trajectory, ego_pose, current_speed_mps, config):
    prepared = prepare_global_trajectory(
        global_trajectory, config.min_point_distance_m
    )
    local, _, _ = build_local_from_prepared(
        prepared, ego_pose, current_speed_mps, config
    )
    return local


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


def test_progress_selection_fails_closed_outside_position_or_heading_gate():
    prepared = prepare_global_trajectory(
        _trajectory([(float(x), 0.0, 5.0, 0.0) for x in range(40)])
    )
    config = LocalPlannerConfig(
        initial_search_distance_m=30.0,
        nearest_position_gate_m=3.0,
        heading_gate_rad=math.radians(30.0),
    )

    assert select_progress_index(prepared, _pose(0.0, 0.0, math.pi), None, config) is None
    assert select_progress_index(prepared, _pose(100.0, 0.0), None, config) is None


def test_progress_search_window_scales_with_speed_without_relaxing_pose_gate():
    prepared = prepare_global_trajectory(
        _trajectory([(float(x), 0.0, 40.0, 0.0) for x in range(80)])
    )
    config = LocalPlannerConfig(
        nearest_search_forward_distance_m=3.0,
        nearest_search_forward_time_sec=0.35,
        nearest_position_gate_m=3.0,
    )

    assert progress_search_forward_distance(40.0, config) == 14.0
    local, nearest, _ = build_local_from_prepared(
        prepared,
        ego_pose=_pose(10.0, 0.0),
        current_speed_mps=40.0,
        config=config,
        previous_nearest_index=0,
    )
    assert nearest == 10
    assert len(local.points) >= 2
    assert select_progress_index(
        prepared,
        _pose(10.0, 5.0),
        0,
        config,
        progress_search_forward_distance(40.0, config),
    ) is None


def test_progress_tracking_uses_strict_height_only_for_initial_layer_selection():
    trajectory = _trajectory(
        [(float(x), 0.0, 4.0, 0.0) for x in range(40)]
    )
    for index, point in enumerate(trajectory.points):
        point.pose.position.z = 100.0 - 0.2 * index
    prepared = prepare_global_trajectory(trajectory)
    config = LocalPlannerConfig(
        nearest_search_forward_distance_m=3.0,
        nearest_position_gate_m=3.0,
        z_gate_m=3.0,
    )

    assert select_progress_index(
        prepared, _pose(0.0, 0.0, z=95.0), None, config
    ) is None

    previous_index = select_progress_index(
        prepared, _pose(0.0, 0.0, z=100.0), None, config
    )
    assert previous_index == 0
    for index in range(1, 21):
        previous_index = select_progress_index(
            prepared,
            _pose(float(index), 0.0, z=100.0),
            previous_index,
            config,
        )
        assert previous_index == index

    assert abs(
        prepared.points[previous_index].pose.position.z - 100.0
    ) > config.z_gate_m


def test_build_local_trajectory_limits_speed_and_stops_at_goal():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        max_speed_mps=2.0,
        max_accel_mps2=0.5,
        max_decel_mps2=-1.0,
    )
    global_trajectory = _trajectory([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)])

    local = _build_local(
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


def test_local_trajectory_preserves_precomputed_course_speed():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        max_speed_mps=5.0,
    )
    global_trajectory = _trajectory(
        [
            (0.0, 0.0, 5.0),
            (2.0, 0.0, 2.0),
            (3.0, 1.0, 2.0),
            (3.0, 3.0, 5.0),
            (3.0, 5.0, 5.0),
        ]
    )

    local = _build_local(
        global_trajectory,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=5.0,
        config=config,
    )

    speeds = [p.longitudinal_velocity_mps for p in local.points]
    assert min(speeds[1:-1]) < config.max_speed_mps


def test_local_acceleration_is_assigned_to_outgoing_segment_at_speed_valley():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        max_speed_mps=5.0,
        max_accel_mps2=10.0,
        max_decel_mps2=-10.0,
    )
    global_trajectory = _trajectory(
        [(0.0, 0.0, 2.0), (1.0, 0.0, 1.0)]
        + [(float(x), 0.0, 2.0) for x in range(2, 101)]
    )

    local = _build_local(
        global_trajectory,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=2.0,
        config=config,
    )

    assert local.points[0].acceleration_mps2 == -1.5
    assert local.points[1].acceleration_mps2 == 1.5


def test_local_trajectory_seeds_departure_without_removing_terminal_stop():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        departure_speed_mps=0.1,
    )
    global_trajectory = _trajectory(
        [(0.0, 0.0, 0.0), (0.5, 0.0, 0.9), (10.0, 0.0, 2.0)]
    )

    local = _build_local(
        global_trajectory,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=0.0,
        config=config,
    )

    assert math.isclose(local.points[0].longitudinal_velocity_mps, 0.1)
    assert local.points[-1].longitudinal_velocity_mps == 0.0


def test_nonterminal_local_window_keeps_forward_yaw_and_nonzero_end_speed():
    config = LocalPlannerConfig(
        lookahead_distance_m=20.0,
        backward_distance_m=0.0,
        max_speed_mps=5.0,
    )
    global_trajectory = _trajectory(
        [(float(x), 0.0, 5.0, 0.0) for x in range(101)]
    )
    prepared = prepare_global_trajectory(global_trajectory)

    local, nearest, includes_goal = build_local_from_prepared(
        prepared,
        ego_pose=_pose(0.0, 0.0),
        current_speed_mps=5.0,
        config=config,
    )

    assert nearest == 0
    assert includes_goal is False
    assert local.points[-1].longitudinal_velocity_mps > 0.0
    assert abs(local.points[-1].pose.orientation.z) < 1e-9
    assert local.points[-1].pose.orientation.w > 0.0


def test_arrival_requires_endpoint_position_progress_and_low_speed():
    prepared = prepare_global_trajectory(
        _trajectory([(float(x), 0.0, 1.0, 0.0) for x in range(11)])
    )

    assert arrival_conditions_met(prepared, 10, _pose(10.0, 0.0), 0.05, 2.0, 0.1)
    assert not arrival_conditions_met(
        prepared, 10, _pose(10.0, 0.0), 1.0, 2.0, 0.1
    )
    assert not arrival_conditions_met(
        prepared, 8, _pose(5.0, 0.0), 0.05, 2.0, 0.1
    )


def test_shutdown_rcl_error_matches_invalid_publisher_context():
    exc = RuntimeError("Failed to publish: publisher's context is invalid")

    assert _is_shutdown_rcl_error(exc)


def test_source_stamp_freshness_is_fail_closed():
    assert source_stamp_is_fresh(9.8, 10.0, 0.35)
    assert source_stamp_is_fresh(10.04, 10.0, 0.35)
    assert not source_stamp_is_fresh(9.6, 10.0, 0.35)
    assert not source_stamp_is_fresh(10.06, 10.0, 0.35)
    assert not source_stamp_is_fresh(math.nan, 10.0, 0.35)
