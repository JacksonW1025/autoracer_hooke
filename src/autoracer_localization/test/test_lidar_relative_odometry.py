import math

from autoracer_localization.lidar_relative_odometry import (
    estimate_scan_to_scan_motion_2d,
)


def test_scan_to_scan_recovers_forward_motion_on_structured_points():
    previous = [
        (2.0, -1.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (4.0, -0.5),
        (4.0, 0.5),
        (7.0, -1.0),
        (7.0, 0.0),
        (7.0, 1.0),
    ]
    current = [(x - 1.2, y) for x, y in previous]

    estimate = estimate_scan_to_scan_motion_2d(
        previous,
        current,
        initial_forward_m=1.0,
        initial_lateral_m=0.0,
        initial_yaw_rad=0.0,
    )

    assert estimate.is_valid
    assert abs(estimate.forward_m - 1.2) < 0.15
    assert abs(estimate.lateral_m) < 0.15
    assert abs(estimate.yaw_rad) < math.radians(1.0)


def test_scan_to_scan_marks_along_collinear_structure_degenerate():
    previous = [(float(x), 0.0) for x in range(2, 30)]
    current = [(x - 1.0, y) for x, y in previous]

    estimate = estimate_scan_to_scan_motion_2d(
        previous,
        current,
        initial_forward_m=1.0,
        initial_lateral_m=0.0,
        initial_yaw_rad=0.0,
        reject_degenerate=True,
    )

    assert not estimate.is_valid
    assert estimate.along_degenerate
    assert estimate.forward_variance_m2 > estimate.lateral_variance_m2


def test_scan_to_scan_rejects_bad_match_residual():
    previous = [
        (2.0, -1.0),
        (2.0, 1.0),
        (5.0, -1.0),
        (5.0, 1.0),
    ]
    current = [(50.0, 50.0), (51.0, 50.0), (50.0, 51.0), (51.0, 51.0)]

    estimate = estimate_scan_to_scan_motion_2d(
        previous,
        current,
        initial_forward_m=0.0,
        initial_lateral_m=0.0,
        initial_yaw_rad=0.0,
        max_match_distance_m=2.0,
    )

    assert not estimate.is_valid
    assert estimate.quality < 0.5
