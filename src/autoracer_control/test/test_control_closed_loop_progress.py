from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from autoracer_control.control_closed_loop_geometry import (
    PathPoint,
    compute_stations,
    monotonic_progress,
    project_to_path,
)
from autoracer_control.control_closed_loop_scenarios import (
    FULL_VALIDATION_SCENARIOS,
    SMOKE_SCENARIOS,
    get_scenario_spec,
)


def test_compute_stations_accumulates_arc_length():
    stations = compute_stations(
        [
            PathPoint(0.0, 0.0),
            PathPoint(3.0, 4.0),
            PathPoint(6.0, 4.0),
        ]
    )

    assert stations == pytest.approx([0.0, 5.0, 8.0])


def test_project_to_path_returns_segment_progress_distance():
    points = [PathPoint(0.0, 0.0), PathPoint(10.0, 0.0), PathPoint(10.0, 10.0)]
    stations = compute_stations(points)

    projection = project_to_path(points, stations, 10.0, 3.0)

    assert projection.nearest_segment_idx == 1
    assert projection.nearest_idx == 1
    assert projection.progress_distance_m == pytest.approx(13.0)
    assert projection.projected_x_m == pytest.approx(10.0)
    assert projection.projected_y_m == pytest.approx(3.0)
    assert projection.segment_yaw_rad == pytest.approx(1.57079632679)


def test_monotonic_progress_clamps_backtracking_projection():
    assert monotonic_progress(previous_progress_m=12.0, projected_progress_m=8.0) == pytest.approx(
        12.0
    )
    assert monotonic_progress(previous_progress_m=12.0, projected_progress_m=14.0) == pytest.approx(
        14.0
    )


def test_scenario_specs_keep_smoke_and_add_v15_full_validation():
    assert {
        "straight_lateral_offset",
        "straight_heading_offset",
        "constant_radius_left",
        "s_curve",
        "longitudinal_speed_step",
        "speed_regime_sweep",
    } <= set(SMOKE_SCENARIOS)
    assert {
        "straight_120m_v1",
        "arc_r20_90deg_v1",
        "s_curve_100m_v1",
        "speed_step_120m_v1",
    } <= set(FULL_VALIDATION_SCENARIOS)

    straight = get_scenario_spec("straight_120m_v1")
    speed_step = get_scenario_spec("speed_step_120m_v1")

    assert straight.scenario_type == "full_validation"
    assert straight.completion_threshold == pytest.approx(0.98)
    assert straight.path_length_m == pytest.approx(120.0)
    assert speed_step.scenario_type == "full_validation"
    assert len(speed_step.segments) >= 3
    assert [segment.reference_velocity_mps for segment in speed_step.segments] == pytest.approx(
        [0.5, 1.0, 2.0]
    )
