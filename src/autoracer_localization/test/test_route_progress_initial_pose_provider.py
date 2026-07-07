import csv
import math
from pathlib import Path

from autoracer_localization.pure_lidar_fixed_lag_tracker import Pose2D, RoutePath
from autoracer_localization.route_progress_initial_pose_provider import (
    ProgressFilter,
    propagate_progress,
    route_progress_pose_from_base,
    update_progress_measurement,
)


def _write_route(path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["x", "y"])
        writer.writeheader()
        for x in range(0, 101, 10):
            writer.writerow({"x": x, "y": 0.0})


def test_progress_filter_uses_weak_measurement_without_snapping() -> None:
    state = ProgressFilter(progress_m=100.0, variance_m2=4.0, last_stamp_sec=10.0)
    state.velocity_mps = 10.0

    propagate_progress(state, stamp_sec=10.1, process_noise_m2ps=0.1)
    before = state.progress_m
    update_progress_measurement(
        state,
        observed_progress_m=110.0,
        measurement_variance_m2=25.0,
        innovation_gate_m=12.0,
    )

    assert before > 100.9
    assert before < state.progress_m < 110.0


def test_progress_filter_rejects_large_gnss_progress_jump() -> None:
    state = ProgressFilter(progress_m=50.0, variance_m2=1.0, last_stamp_sec=1.0)

    updated, accepted = update_progress_measurement(
        state,
        observed_progress_m=90.0,
        measurement_variance_m2=25.0,
        innovation_gate_m=12.0,
    )

    assert updated.progress_m == 50.0
    assert not accepted


def test_route_progress_pose_preserves_base_yaw_and_cross(tmp_path: Path) -> None:
    route_csv = tmp_path / "route.csv"
    _write_route(route_csv)
    route = RoutePath.from_csv(route_csv)
    base = Pose2D(stamp_sec=1.0, x=12.0, y=1.5, yaw=0.42)

    corrected = route_progress_pose_from_base(
        route_path=route,
        base_pose=base,
        progress_m=30.0,
        predicted_progress_m=12.0,
        route_search_radius_m=30.0,
        max_abs_cross_m=3.0,
    )

    assert math.isclose(corrected.x, 30.0, abs_tol=1e-6)
    assert math.isclose(corrected.y, 1.5, abs_tol=1e-6)
    assert math.isclose(corrected.yaw, base.yaw, abs_tol=1e-6)
