import math

from autoracer_localization.pure_lidar_axis_remapper import (
    elevation_corrected_along_pose,
    intensity_corrected_along_pose,
    planar_covariance_variance,
    profile_corrected_along_pose,
    propagate_pose,
    reflector_spatial_corrected_along_pose,
    remap_ndt_pose_for_degeneracy,
    reject_ndt_pose_for_consistency,
    route_progress_innovation_m,
    route_progress_held_pose,
    route_cross_held_pose,
    runtime_localizability_degeneracy_metrics,
    scan_point_count_degeneracy_metrics,
    scan_point_count_persistence_metrics,
    scan_geometry_degeneracy_metrics,
    scan_route_geometry_degeneracy_metrics,
    stable_route_cross_target_candidate,
    update_twist_bias_from_progress_delta,
    update_twist_bias_from_progress_innovation,
    update_route_progress_anchor,
    update_route_cross_target,
    scan_submap_along_corrected_pose,
    scan_submap_yaw_corrected_pose,
    should_apply_route_hold,
)
from autoracer_localization.pure_lidar_fixed_lag_tracker import (
    LightweightScanSubmap,
    MotionDelta,
    Pose2D,
    RoutePath,
)


def test_non_degenerate_axis_remapper_accepts_ndt_pose_unchanged():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 14.0, 1.5, 0.2)

    out = remap_ndt_pose_for_degeneracy(
        route_path=route,
        predicted_pose=predicted,
        ndt_pose=ndt,
        along_degenerate=False,
        keep_predicted_yaw=False,
        search_radius_m=20.0,
    )

    assert out == ndt


def test_degenerate_axis_remapper_uses_predicted_along_and_ndt_cross_yaw():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    predicted = Pose2D(1.0, 10.0, 0.0, math.radians(10.0))
    ndt = Pose2D(1.0, 14.0, 1.5, math.radians(2.0))

    out = remap_ndt_pose_for_degeneracy(
        route_path=route,
        predicted_pose=predicted,
        ndt_pose=ndt,
        along_degenerate=True,
        keep_predicted_yaw=False,
        search_radius_m=20.0,
    )

    assert abs(out.x - 10.0) < 1e-9
    assert abs(out.y - 1.5) < 1e-9
    assert abs(out.yaw - math.radians(2.0)) < 1e-9


def test_degenerate_axis_remapper_can_keep_predicted_yaw():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    predicted = Pose2D(1.0, 10.0, 0.0, math.radians(10.0))
    ndt = Pose2D(1.0, 14.0, 1.5, math.radians(2.0))

    out = remap_ndt_pose_for_degeneracy(
        route_path=route,
        predicted_pose=predicted,
        ndt_pose=ndt,
        along_degenerate=True,
        keep_predicted_yaw=True,
        search_radius_m=20.0,
    )

    assert abs(out.yaw - math.radians(10.0)) < 1e-9


def test_axis_remapper_propagates_pose_with_dead_reckoning_motion():
    pose = Pose2D(1.0, 1.0, 2.0, math.pi / 2.0)

    out = propagate_pose(
        pose,
        MotionDelta(dt_sec=0.5, forward_m=2.0, lateral_m=0.0, yaw_rad=0.1),
    )

    assert abs(out.stamp_sec - 1.5) < 1e-9
    assert abs(out.x - 1.0) < 1e-9
    assert abs(out.y - 4.0) < 1e-9
    assert abs(out.yaw - (math.pi / 2.0 + 0.1)) < 1e-9


def test_planar_covariance_variance_projects_along_axis():
    covariance = [0.0] * 36
    covariance[0] = 4.0
    covariance[7] = 1.0

    assert abs(planar_covariance_variance(covariance, 0.0) - 4.0) < 1e-9
    assert abs(planar_covariance_variance(covariance, math.pi / 2.0) - 1.0) < 1e-9


def test_profile_corrected_along_pose_selects_best_route_progress_offset():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(12.0, -2.0), (12.5, -2.0), (13.0, -2.0)])
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 14.0, -1.0, 0.0)
    scan = [(0.0, -1.0), (0.5, -1.0), (1.0, -1.0)]

    result = profile_corrected_along_pose(
        route_path=route,
        map_submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        ndt_pose=ndt,
        forward_offsets_m=(-1.0, 0.0, 2.0, 4.0),
        search_radius_m=20.0,
        max_points=16,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.5,
        max_residual_m=0.25,
        min_improvement_m=0.1,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 12.0) < 1e-9
    assert abs(pose.y + 1.0) < 1e-9
    assert diag["profile_best_offset_m"] == 2.0


def test_scan_submap_yaw_corrected_pose_selects_causal_yaw_offset():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(0.0, y * 0.5) for y in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, 0.0) for x in range(8)]

    result = scan_submap_yaw_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        yaw_offsets_rad=(0.0, math.radians(90.0)),
        max_points=32,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.yaw - math.radians(90.0)) < 1e-9
    assert diag["scan_submap_yaw_best_quality"] >= 0.8


def test_scan_submap_yaw_corrected_pose_rejects_weak_improvement():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(x * 0.5, 0.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, 0.0) for x in range(8)]

    result = scan_submap_yaw_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        yaw_offsets_rad=(0.0, math.radians(2.0)),
        max_points=32,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
    )

    assert result is None


def test_scan_submap_along_corrected_pose_selects_causal_progress_offset():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(2.0 + x * 0.5, -1.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, -1.0) for x in range(8)]

    result = scan_submap_along_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        forward_offsets_m=(0.0, 1.0, 2.0),
        max_points=32,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 2.0) < 1e-9
    assert diag["scan_submap_along_best_offset_m"] == 2.0


def test_scan_submap_along_corrected_pose_rejects_weak_improvement():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(x * 0.5, 0.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, 0.0) for x in range(8)]

    result = scan_submap_along_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        forward_offsets_m=(0.0, 1.0),
        max_points=32,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
    )

    assert result is None


def test_scan_submap_along_corrected_pose_rejects_boundary_best_when_guarded():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(3.0 + x * 0.5, -1.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, -1.0) for x in range(8)]

    result = scan_submap_along_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        forward_offsets_m=(0.0, 1.0, 2.0, 3.0),
        max_points=32,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
        reject_boundary_best=True,
    )

    assert result is None


def test_scan_submap_along_corrected_pose_rejects_ambiguous_second_best():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(x * 0.5, 0.0) for x in range(8)])
    submap.add_world_points([(2.0 + x * 0.5, 0.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, 0.0) for x in range(8)]

    result = scan_submap_along_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        forward_offsets_m=(0.0, 1.0, 2.0, 3.0),
        max_points=32,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.0,
        min_second_best_margin_m=0.1,
    )

    assert result is None


def test_scan_submap_along_corrected_pose_accepts_unique_interior_minimum_when_guarded():
    submap = LightweightScanSubmap(voxel_size_m=0.2, max_cells=1000)
    submap.add_world_points([(2.0 + x * 0.5, -1.0) for x in range(8)])
    predicted = Pose2D(1.0, 0.0, 0.0, 0.0)
    scan = [(x * 0.5, -1.0) for x in range(8)]

    result = scan_submap_along_corrected_pose(
        submap=submap,
        scan_points=scan,
        predicted_pose=predicted,
        forward_offsets_m=(0.0, 1.0, 2.0, 3.0),
        max_points=32,
        max_profile_cells=1000,
        lateral_bin_m=0.5,
        min_quality=0.8,
        max_residual_m=0.25,
        min_improvement_m=0.2,
        reject_boundary_best=True,
        min_second_best_margin_m=0.1,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 2.0) < 1e-9
    assert diag["scan_submap_along_best_offset_m"] == 2.0
    assert diag["scan_submap_along_second_best_residual_m"] > diag[
        "scan_submap_along_best_residual_m"
    ]


def test_elevation_corrected_along_pose_selects_height_consistent_offset():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    map_xyz = [(12.0, 0.0, 10.0), (12.5, 0.0, 10.5), (13.0, 0.0, 11.0)] * 3
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 14.0, 0.0, 0.0)
    scan_xyz = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.5), (1.0, 0.0, 1.0)] * 4

    result = elevation_corrected_along_pose(
        route_path=route,
        map_xyz=map_xyz,
        scan_xyz=scan_xyz,
        predicted_pose=predicted,
        ndt_pose=ndt,
        ndt_z_m=10.0,
        forward_offsets_m=(-1.0, 0.0, 2.0, 4.0),
        search_radius_m=20.0,
        max_points=16,
        max_map_xy_distance_m=0.25,
        min_quality=0.5,
        max_rmse_m=0.1,
        min_improvement_m=0.1,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 12.0) < 1e-9
    assert diag["elevation_best_offset_m"] == 2.0


def test_intensity_corrected_along_pose_selects_reflectivity_consistent_offset():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    map_xyi = [(12.0, 0.0, 1.0), (12.5, 0.0, 5.0), (13.0, 0.0, 2.0)] * 3
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 14.0, 0.0, 0.0)
    scan_xyi = [(0.0, 0.0, 1.0), (0.5, 0.0, 5.0), (1.0, 0.0, 2.0)] * 4

    result = intensity_corrected_along_pose(
        route_path=route,
        map_xyi=map_xyi,
        scan_xyi=scan_xyi,
        predicted_pose=predicted,
        ndt_pose=ndt,
        forward_offsets_m=(-1.0, 0.0, 2.0, 4.0),
        search_radius_m=20.0,
        max_points=16,
        max_map_xy_distance_m=0.25,
        min_quality=0.5,
        max_residual=0.05,
        min_improvement=0.0,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 12.0) < 1e-9
    assert diag["intensity_best_offset_m"] == 2.0


def test_reflector_spatial_corrected_along_pose_selects_sparse_reflector_offset():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    map_xyi = [(12.0, 1.0, 250.0), (12.5, 1.0, 250.0), (13.0, 1.0, 250.0)] * 3
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 14.0, 1.0, 0.0)
    scan_xyi = [(0.0, 0.0, 255.0), (0.5, 0.0, 255.0), (1.0, 0.0, 255.0)] * 4

    result = reflector_spatial_corrected_along_pose(
        route_path=route,
        map_xyi=map_xyi,
        scan_xyi=scan_xyi,
        predicted_pose=predicted,
        ndt_pose=ndt,
        forward_offsets_m=(-1.0, 0.0, 2.0, 4.0),
        search_radius_m=20.0,
        max_points=16,
        min_map_intensity=200.0,
        min_scan_intensity=200.0,
        max_match_distance_m=0.25,
        min_quality=0.5,
        min_improvement_m=0.0,
    )

    assert result is not None
    pose, diag = result
    assert abs(pose.x - 12.0) < 1e-9
    assert abs(pose.y - 1.0) < 1e-9
    assert diag["reflector_best_offset_m"] == 2.0


def test_consistency_gate_rejects_large_yaw_innovation_to_prediction():
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 10.4, 0.1, math.radians(25.0))

    rejected, diag = reject_ndt_pose_for_consistency(
        predicted_pose=predicted,
        ndt_pose=ndt,
        max_xy_innovation_m=5.0,
        max_yaw_innovation_rad=math.radians(8.0),
    )

    assert rejected
    assert diag["consistency_reject_reason"] == "yaw_innovation"
    assert diag["consistency_yaw_innovation_deg"] > 20.0


def test_consistency_gate_accepts_small_causal_innovation():
    predicted = Pose2D(1.0, 10.0, 0.0, 0.0)
    ndt = Pose2D(1.0, 10.4, 0.1, math.radians(2.0))

    rejected, diag = reject_ndt_pose_for_consistency(
        predicted_pose=predicted,
        ndt_pose=ndt,
        max_xy_innovation_m=5.0,
        max_yaw_innovation_rad=math.radians(8.0),
    )

    assert not rejected
    assert diag["consistency_reject_reason"] == ""


def test_route_cross_hold_corrects_lateral_drift_without_changing_progress():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    pose = Pose2D(1.0, 10.0, 3.0, math.radians(4.0))

    corrected, diag = route_cross_held_pose(
        route_path=route,
        pose=pose,
        target_cross_m=0.2,
        gain=1.0,
        yaw_gain=0.0,
        gate_m=5.0,
        predicted_progress_m=10.0,
        search_radius_m=20.0,
    )

    assert corrected is not None
    assert abs(corrected.x - 10.0) < 1e-9
    assert abs(corrected.y - 0.2) < 1e-9
    assert abs(corrected.yaw - pose.yaw) < 1e-9
    assert diag["route_cross_hold_applied"] is True
    assert abs(diag["route_cross_before_m"] - 3.0) < 1e-9
    assert abs(diag["route_cross_target_m"] - 0.2) < 1e-9


def test_route_progress_hold_uses_causal_progress_not_nearest_branch():
    route = RoutePath(
        [
            (0.0, 0.0),
            (100.0, 0.0),
            (100.0, 10.0),
            (0.0, 10.0),
        ]
    )
    # The pose is spatially closer to the return branch near progress 205m,
    # but a causal tracker at progress 5m must stay on the outbound branch.
    pose = Pose2D(1.0, 5.0, 8.8, 0.0)

    corrected, diag = route_progress_held_pose(
        route_path=route,
        pose=pose,
        progress_m=5.0,
        target_cross_m=0.2,
        gain=1.0,
        yaw_gain=0.0,
        gate_m=20.0,
    )

    assert corrected is not None
    assert abs(corrected.x - 5.0) < 1e-9
    assert abs(corrected.y - 0.2) < 1e-9
    assert diag["route_progress_hold_applied"] is True
    assert abs(diag["route_progress_hold_progress_m"] - 5.0) < 1e-9


def test_route_progress_hold_rejects_when_causal_progress_too_far():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    pose = Pose2D(1.0, 40.0, 5.0, 0.0)

    corrected, diag = route_progress_held_pose(
        route_path=route,
        pose=pose,
        progress_m=5.0,
        target_cross_m=0.0,
        gain=1.0,
        yaw_gain=0.0,
        gate_m=5.0,
    )

    assert corrected is None
    assert diag["route_progress_hold_applied"] is False
    assert diag["route_progress_hold_distance_m"] > 30.0


def test_route_hold_stale_gate_allows_recent_ndt_when_disabled():
    assert should_apply_route_hold(
        pose_stamp_sec=10.0,
        last_ndt_update_stamp_sec=9.95,
        degenerate_until_sec=0.0,
        only_when_stale_or_degenerate=False,
        stale_sec=0.5,
    )


def test_route_hold_stale_gate_blocks_recent_non_degenerate_ndt():
    assert not should_apply_route_hold(
        pose_stamp_sec=10.0,
        last_ndt_update_stamp_sec=9.95,
        degenerate_until_sec=0.0,
        only_when_stale_or_degenerate=True,
        stale_sec=0.5,
    )


def test_route_hold_stale_gate_allows_ndt_gap_or_degeneracy():
    assert should_apply_route_hold(
        pose_stamp_sec=10.0,
        last_ndt_update_stamp_sec=9.0,
        degenerate_until_sec=0.0,
        only_when_stale_or_degenerate=True,
        stale_sec=0.5,
    )
    assert should_apply_route_hold(
        pose_stamp_sec=10.0,
        last_ndt_update_stamp_sec=9.95,
        degenerate_until_sec=10.5,
        only_when_stale_or_degenerate=True,
        stale_sec=0.5,
    )


def test_route_hold_stale_gate_allows_recent_ndt_inside_route_hold_window():
    assert should_apply_route_hold(
        pose_stamp_sec=10.0,
        last_ndt_update_stamp_sec=9.99,
        degenerate_until_sec=10.5,
        only_when_stale_or_degenerate=True,
        stale_sec=0.5,
    )


def test_route_cross_target_iir_learning_is_bounded_and_converges():
    target = 2.0
    for observed in (-1.0, -1.0, -1.0, -1.0):
        target = update_route_cross_target(
            current_target_m=target,
            observed_cross_m=observed,
            alpha=0.5,
            max_step_m=1.0,
            max_abs_m=3.0,
        )

    assert target < -0.6
    assert target > -1.0


def test_route_cross_target_learning_ignores_invalid_observation():
    assert update_route_cross_target(
        current_target_m=0.2,
        observed_cross_m=math.nan,
        alpha=0.5,
        max_step_m=1.0,
        max_abs_m=3.0,
    ) == 0.2


def test_stable_route_cross_target_requires_consistent_window():
    assert stable_route_cross_target_candidate(
        observations=(1.9, -1.0, -1.1, -1.05),
        min_count=4,
        max_range_m=0.3,
    ) is None
    candidate = stable_route_cross_target_candidate(
        observations=(-1.0, -1.1, -1.05, -0.98),
        min_count=4,
        max_range_m=0.3,
    )
    assert candidate is not None
    assert abs(candidate + 1.025) < 0.1


def test_route_progress_innovation_measures_ndt_along_jump():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    ndt = Pose2D(1.0, 16.0, 0.0, 0.0)

    innovation = route_progress_innovation_m(
        route_path=route,
        predicted_progress_m=10.0,
        ndt_pose=ndt,
        search_radius_m=20.0,
    )

    assert innovation is not None
    assert abs(innovation - 6.0) < 1e-9


def test_route_progress_anchor_blends_small_ndt_innovation():
    updated, applied = update_route_progress_anchor(
        current_progress_m=100.0,
        observed_progress_m=100.8,
        gate_m=1.0,
        gain=0.5,
        max_step_m=0.3,
    )

    assert applied
    assert abs(updated - 100.3) < 1e-9


def test_route_progress_anchor_rejects_wrong_basin_jump():
    updated, applied = update_route_progress_anchor(
        current_progress_m=100.0,
        observed_progress_m=104.0,
        gate_m=1.0,
        gain=0.5,
        max_step_m=0.3,
    )

    assert not applied
    assert updated == 100.0


def test_twist_bias_learning_applies_small_route_progress_innovation():
    updated, delta, applied = update_twist_bias_from_progress_innovation(
        current_bias_mps=-0.2,
        progress_innovation_m=0.4,
        dt_sec=2.0,
        alpha=0.5,
        max_step_mps=0.05,
        max_abs_mps=0.25,
        max_progress_innovation_m=1.0,
    )

    assert applied
    assert abs(delta - 0.025) < 1e-9
    assert abs(updated + 0.175) < 1e-9


def test_twist_bias_learning_rejects_wrong_basin_progress_jump():
    updated, delta, applied = update_twist_bias_from_progress_innovation(
        current_bias_mps=-0.2,
        progress_innovation_m=7.0,
        dt_sec=2.0,
        alpha=0.5,
        max_step_mps=0.05,
        max_abs_mps=0.25,
        max_progress_innovation_m=1.0,
    )

    assert not applied
    assert delta is None
    assert updated == -0.2


def test_twist_bias_learning_respects_asymmetric_bounds():
    updated, delta, applied = update_twist_bias_from_progress_innovation(
        current_bias_mps=-0.08,
        progress_innovation_m=0.4,
        dt_sec=2.0,
        alpha=0.5,
        max_step_mps=0.05,
        max_abs_mps=0.25,
        max_progress_innovation_m=1.0,
        min_bias_mps=-0.25,
        max_bias_mps=-0.07,
    )

    assert applied
    assert abs(updated + 0.07) < 1e-9
    assert abs(delta - 0.01) < 1e-9


def test_twist_bias_delta_learning_estimates_absolute_bias_from_progress_window():
    updated, delta, applied = update_twist_bias_from_progress_delta(
        current_bias_mps=0.0,
        observed_progress_delta_m=19.2,
        raw_forward_delta_m=20.0,
        dt_sec=10.0,
        alpha=1.0,
        max_step_mps=0.2,
        max_abs_mps=0.25,
        max_progress_residual_m=2.0,
    )

    assert applied
    assert abs(updated + 0.08) < 1e-9
    assert abs(delta + 0.08) < 1e-9


def test_twist_bias_delta_learning_rejects_large_progress_residual():
    updated, delta, applied = update_twist_bias_from_progress_delta(
        current_bias_mps=-0.05,
        observed_progress_delta_m=10.0,
        raw_forward_delta_m=20.0,
        dt_sec=10.0,
        alpha=1.0,
        max_step_mps=0.2,
        max_abs_mps=0.25,
        max_progress_residual_m=2.0,
    )

    assert not applied
    assert delta is None
    assert updated == -0.05


def test_twist_bias_delta_learning_limits_step_and_bounds():
    updated, delta, applied = update_twist_bias_from_progress_delta(
        current_bias_mps=0.0,
        observed_progress_delta_m=18.0,
        raw_forward_delta_m=20.0,
        dt_sec=10.0,
        alpha=1.0,
        max_step_mps=0.05,
        max_abs_mps=0.25,
        max_progress_residual_m=3.0,
        min_bias_mps=-0.12,
        max_bias_mps=0.05,
    )

    assert applied
    assert abs(updated + 0.05) < 1e-9
    assert abs(delta + 0.05) < 1e-9


def test_scan_geometry_degeneracy_detects_one_sided_scan():
    points = [(10.0 + i * 0.1, 3.0 + 0.01 * i) for i in range(120)]

    metrics = scan_geometry_degeneracy_metrics(
        points,
        min_total_side_points=50,
        min_side_points=8,
        min_abs_lateral_m=2.0,
        min_forward_m=2.0,
        max_forward_m=80.0,
        min_side_fraction=0.08,
    )

    assert metrics["scan_geometry_degenerate"] is True
    assert metrics["scan_geometry_left_count"] == 120
    assert metrics["scan_geometry_right_count"] == 0


def test_scan_geometry_degeneracy_rejects_balanced_scan():
    points = []
    for i in range(80):
        points.append((10.0 + i * 0.1, 3.0))
        points.append((10.0 + i * 0.1, -3.0))

    metrics = scan_geometry_degeneracy_metrics(
        points,
        min_total_side_points=50,
        min_side_points=8,
        min_abs_lateral_m=2.0,
        min_forward_m=2.0,
        max_forward_m=80.0,
        min_side_fraction=0.08,
    )

    assert metrics["scan_geometry_degenerate"] is False
    assert metrics["scan_geometry_left_count"] == 80
    assert metrics["scan_geometry_right_count"] == 80


def test_scan_point_count_degeneracy_detects_sparse_scan():
    metrics = scan_point_count_degeneracy_metrics(5200, max_points=9000)

    assert metrics["scan_point_count_degenerate"] is True
    assert metrics["scan_point_count"] == 5200


def test_scan_point_count_degeneracy_rejects_dense_scan():
    metrics = scan_point_count_degeneracy_metrics(18000, max_points=9000)

    assert metrics["scan_point_count_degenerate"] is False
    assert metrics["scan_point_count"] == 18000


def test_scan_point_count_persistence_waits_until_min_duration():
    state = None
    first = scan_point_count_persistence_metrics(
        is_degenerate=True,
        stamp_sec=10.0,
        first_degenerate_stamp_sec=state,
        min_duration_sec=6.0,
    )
    assert first["scan_point_count_persistent_degenerate"] is False
    assert first["scan_point_count_degenerate_duration_sec"] == 0.0
    state = first["scan_point_count_first_degenerate_stamp_sec"]

    later = scan_point_count_persistence_metrics(
        is_degenerate=True,
        stamp_sec=15.9,
        first_degenerate_stamp_sec=state,
        min_duration_sec=6.0,
    )
    assert later["scan_point_count_persistent_degenerate"] is False

    ready = scan_point_count_persistence_metrics(
        is_degenerate=True,
        stamp_sec=16.0,
        first_degenerate_stamp_sec=state,
        min_duration_sec=6.0,
    )
    assert ready["scan_point_count_persistent_degenerate"] is True
    assert ready["scan_point_count_degenerate_duration_sec"] == 6.0


def test_scan_point_count_persistence_resets_on_dense_scan():
    sparse = scan_point_count_persistence_metrics(
        is_degenerate=True,
        stamp_sec=10.0,
        first_degenerate_stamp_sec=None,
        min_duration_sec=3.0,
    )

    dense = scan_point_count_persistence_metrics(
        is_degenerate=False,
        stamp_sec=11.0,
        first_degenerate_stamp_sec=sparse["scan_point_count_first_degenerate_stamp_sec"],
        min_duration_sec=3.0,
    )

    assert dense["scan_point_count_persistent_degenerate"] is False
    assert dense["scan_point_count_first_degenerate_stamp_sec"] is None
    assert dense["scan_point_count_degenerate_duration_sec"] == 0.0


def test_runtime_localizability_degeneracy_metrics_extends_hold_window():
    payload = {
        "stamp_sec": 12.0,
        "spread_covariance_ambiguous": True,
        "spread_covariance_along_m2": 9.0,
        "spread_covariance_cross_m2": 0.25,
        "spread_covariance_contender_count": 4,
    }

    metrics = runtime_localizability_degeneracy_metrics(
        payload,
        min_along_variance_m2=4.0,
        min_along_to_cross_ratio=4.0,
        hold_sec=3.0,
        current_degenerate_until_sec=10.0,
    )

    assert metrics["runtime_localizability_along_degenerate"] is True
    assert metrics["runtime_localizability_degenerate_until_sec"] == 15.0
    assert metrics["spread_covariance_ambiguous"] is True
    assert metrics["spread_covariance_contender_count"] == 4


def test_runtime_localizability_degeneracy_metrics_preserves_current_window_when_not_degenerate():
    payload = {
        "stamp_sec": 12.0,
        "spread_covariance_ambiguous": True,
        "spread_covariance_along_m2": 1.0,
        "spread_covariance_cross_m2": 0.5,
    }

    metrics = runtime_localizability_degeneracy_metrics(
        payload,
        min_along_variance_m2=4.0,
        min_along_to_cross_ratio=4.0,
        hold_sec=3.0,
        current_degenerate_until_sec=20.0,
    )

    assert metrics["runtime_localizability_along_degenerate"] is False
    assert metrics["runtime_localizability_degenerate_until_sec"] == 20.0


def test_scan_route_geometry_degeneracy_detects_one_sided_route_frame_scan():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    pose = Pose2D(10.0, 0.0, 0.0, 0.0)
    points = [(5.0 + i * 0.1, 3.0) for i in range(120)]

    metrics = scan_route_geometry_degeneracy_metrics(
        points,
        route_path=route,
        pose=pose,
        predicted_progress_m=10.0,
        search_radius_m=50.0,
        min_total_side_points=50,
        min_side_points=8,
        min_abs_cross_m=2.0,
        min_side_fraction=0.08,
    )

    assert metrics["scan_route_geometry_degenerate"] is True
    assert metrics["scan_route_left_count"] == 120
    assert metrics["scan_route_right_count"] == 0


def test_scan_route_geometry_degeneracy_rejects_balanced_route_frame_scan():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    pose = Pose2D(10.0, 0.0, 0.0, 0.0)
    points = []
    for i in range(80):
        points.append((5.0 + i * 0.1, 3.0))
        points.append((5.0 + i * 0.1, -3.0))

    metrics = scan_route_geometry_degeneracy_metrics(
        points,
        route_path=route,
        pose=pose,
        predicted_progress_m=10.0,
        search_radius_m=50.0,
        min_total_side_points=50,
        min_side_points=8,
        min_abs_cross_m=2.0,
        min_side_fraction=0.08,
    )

    assert metrics["scan_route_geometry_degenerate"] is False
    assert metrics["scan_route_left_count"] == 80
    assert metrics["scan_route_right_count"] == 80


def test_runtime_localizability_can_hold_route_without_ndt_remap():
    metrics = runtime_localizability_degeneracy_metrics(
        {
            "stamp_sec": 100.0,
            "selected_output_covariance_along_m2": 6.0,
            "selected_output_covariance_cross_m2": 0.5,
        },
        min_along_variance_m2=1.0,
        min_along_to_cross_ratio=4.0,
        hold_sec=3.0,
        current_degenerate_until_sec=90.0,
        runtime_localizability_remaps_ndt=False,
    )

    assert metrics["runtime_localizability_along_degenerate"] is True
    assert metrics["runtime_localizability_degenerate_until_sec"] == 90.0
    assert metrics["runtime_localizability_route_hold_until_sec"] == 103.0
