import math

from autoracer_localization.pure_lidar_fixed_lag_tracker import (
    FixedLagMultiHypothesisTracker,
    LightweightScanSubmap,
    MotionDelta,
    NdtCandidate,
    Pose2D,
    RelativeResidual,
    RoutePath,
    RoutePrior,
    RouteCrossTargetLearner,
    WeakPriorPenalty,
    apply_degenerate_velocity_scale,
    apply_lro_forward_correction,
    motion_dt_is_usable,
    needs_pointcloud_subscription,
    scan_geometry_certificate,
    route_frame_scan_geometry_certificate,
    route_offset_pose,
    route_remap_candidate_along_to_prediction,
    candidate_residual_consistency_is_valid,
    submap_candidate_consistency_is_valid,
    update_twist_bias_estimate,
    TrackerConfig,
    candidate_confidence_summary,
    candidate_can_refresh_scan_submap_anchor,
    candidate_is_usable,
    candidate_indicates_along_degeneracy,
    candidates_from_runtime_multistart,
    payload_indicates_along_degeneracy,
    normalize_angle,
    planar_delta,
    route_filter_candidates,
    scan_submap_anchor_is_valid,
    scan_point_count_indicates_degeneracy,
)
from autoracer_localization.lidar_relative_odometry import RelativeOdometryEstimate


def test_propagate_is_causal_planar_motion():
    tracker = FixedLagMultiHypothesisTracker(Pose2D(10.0, 1.0, 2.0, math.pi / 2.0))

    tracker.propagate(MotionDelta(dt_sec=0.5, forward_m=2.0, lateral_m=0.5, yaw_rad=0.1))

    pose = tracker.best().pose
    assert pose.stamp_sec == 10.5
    assert abs(pose.x - 0.5) < 1e-9
    assert abs(pose.y - 4.0) < 1e-9
    assert abs(pose.yaw - normalize_angle(math.pi / 2.0 + 0.1)) < 1e-9


def test_motion_dt_gate_tolerates_configured_replay_stride_jitter():
    assert not motion_dt_is_usable(0.0, 1.2)
    assert motion_dt_is_usable(1.000000001, 1.2)
    assert not motion_dt_is_usable(1.3, 1.2)


def test_scan_submap_anchor_is_default_off_and_causal():
    assert not scan_submap_anchor_is_valid(
        enabled=False,
        anchor_stamp_sec=10.0,
        current_stamp_sec=11.0,
        max_age_sec=8.0,
    )
    assert scan_submap_anchor_is_valid(
        enabled=True,
        anchor_stamp_sec=10.0,
        current_stamp_sec=11.0,
        max_age_sec=8.0,
    )
    assert not scan_submap_anchor_is_valid(
        enabled=True,
        anchor_stamp_sec=10.0,
        current_stamp_sec=9.9,
        max_age_sec=8.0,
    )
    assert not scan_submap_anchor_is_valid(
        enabled=True,
        anchor_stamp_sec=10.0,
        current_stamp_sec=18.1,
        max_age_sec=8.0,
    )


def test_scan_submap_anchor_refresh_requires_converged_usable_ndt():
    config = TrackerConfig(
        enable_not_converged_partial_candidates=True,
        not_converged_partial_min_nvtl=0.9,
    )
    partial = NdtCandidate(
        Pose2D(1.0, 0.0, 0.0, 0.0),
        score=1.0,
        converged=False,
        rejection_reason="not_converged",
        nearest_voxel_transformation_likelihood=0.95,
    )
    assert candidate_is_usable(partial, config)
    assert not candidate_can_refresh_scan_submap_anchor(partial, config)
    converged = NdtCandidate(
        Pose2D(1.0, 0.0, 0.0, 0.0),
        score=1.0,
        converged=True,
    )
    assert candidate_can_refresh_scan_submap_anchor(converged, config)


def test_tracker_keeps_multiple_ndt_basin_hypotheses():
    config = TrackerConfig(max_hypotheses=4)
    tracker = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0), config)
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [
            NdtCandidate(Pose2D(1.0, 10.0, 0.0, 0.0), score=1.0, initial_to_result_m=0.2),
            NdtCandidate(Pose2D(1.0, 15.0, 0.0, 0.0), score=0.95, initial_to_result_m=0.4),
            NdtCandidate(Pose2D(1.0, 5.0, 0.0, 0.0), score=0.9, initial_to_result_m=0.5),
        ]
    )

    poses = [hypothesis.pose.x for hypothesis in tracker.hypotheses]
    assert len(poses) == 4
    assert {5.0, 10.0, 15.0}.issubset(set(poses))


def test_weak_prior_penalty_suppresses_far_candidate_without_direct_pose_override():
    config = TrackerConfig(max_hypotheses=3, gnss_weak_prior_weight=1.0)
    tracker = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0), config)

    tracker.update(
        [
            NdtCandidate(Pose2D(1.0, 20.0, 0.0, 0.0), score=1.0, initial_to_result_m=0.1),
            NdtCandidate(Pose2D(1.0, 1.0, 0.0, 0.0), score=0.9, initial_to_result_m=0.1),
        ],
        candidate_penalties={
            0: WeakPriorPenalty(penalty=8.0, distance_m=20.0),
            1: WeakPriorPenalty(penalty=0.02, distance_m=1.0),
        },
    )

    best = tracker.best().pose
    assert abs(best.x - 1.0) < 1e-9
    assert abs(best.y) < 1e-9


def test_candidate_confidence_summary_is_diagnostic_only_and_counts_sources():
    config = TrackerConfig(
        max_candidate_initial_to_result_m=2.0,
        enable_candidate_low_score_along_remap=True,
        candidate_low_score_along_remap_threshold=1.5,
    )
    candidates = [
        NdtCandidate(
            Pose2D(1.0, 1.0, 0.0, 0.0),
            score=0.5,
            initial_to_result_m=0.2,
            nearest_voxel_transformation_likelihood=1.0,
            source="ndt",
        ),
        NdtCandidate(
            Pose2D(1.0, 2.0, 0.0, 0.1),
            score=2.0,
            converged=False,
            rejection_reason="score_below_threshold",
            initial_to_result_m=3.0,
            nearest_voxel_transformation_likelihood=2.5,
            source="global_map_local_ndt",
        ),
    ]

    summary = candidate_confidence_summary(candidates, config)

    assert summary["candidate_usable_count"] == 1
    assert summary["candidate_converged_count"] == 1
    assert summary["candidate_rejected_count"] == 1
    assert summary["candidate_top_reject_reason"] == "score_below_threshold"
    assert summary["candidate_along_degenerate_count"] == 1
    assert summary["candidate_source_ndt_count"] == 1
    assert summary["candidate_source_global_map_local_ndt_count"] == 1
    assert summary["candidate_nvtl_min"] == 1.0
    assert summary["candidate_i2r_max_m"] == 3.0


def test_not_converged_partial_candidates_are_default_rejected():
    candidate = NdtCandidate(
        Pose2D(1.0, 1.0, 0.0, 0.0),
        score=1.2,
        converged=False,
        rejection_reason="not_converged",
        nearest_voxel_transformation_likelihood=1.2,
    )

    assert not candidate_is_usable(candidate, TrackerConfig())


def test_not_converged_partial_candidates_require_explicit_switch_and_nvtl():
    candidate = NdtCandidate(
        Pose2D(1.0, 1.0, 0.0, 0.0),
        score=1.2,
        converged=False,
        rejection_reason="not_converged",
        nearest_voxel_transformation_likelihood=1.2,
    )

    assert candidate_is_usable(
        candidate,
        TrackerConfig(
            enable_not_converged_partial_candidates=True,
            not_converged_partial_min_nvtl=1.0,
        ),
    )
    assert not candidate_is_usable(
        candidate,
        TrackerConfig(
            enable_not_converged_partial_candidates=True,
            not_converged_partial_min_nvtl=1.5,
        ),
    )


def test_candidate_residual_consistency_gate_is_default_off():
    assert candidate_residual_consistency_is_valid(
        relative_residual=None,
        baseline_residual=None,
        config=TrackerConfig(),
    )


def test_candidate_residual_consistency_gate_requires_improvement():
    config = TrackerConfig(
        enable_candidate_residual_consistency_gate=True,
        candidate_residual_max_m=2.0,
        candidate_residual_min_improvement_m=0.2,
    )

    assert candidate_residual_consistency_is_valid(
        relative_residual=RelativeResidual(0.5, 0.0, 0.8),
        baseline_residual=RelativeResidual(1.0, 0.0, 0.8),
        config=config,
    )
    assert not candidate_residual_consistency_is_valid(
        relative_residual=RelativeResidual(0.95, 0.0, 0.8),
        baseline_residual=RelativeResidual(1.0, 0.0, 0.8),
        config=config,
    )
    assert not candidate_residual_consistency_is_valid(
        relative_residual=RelativeResidual(2.5, 0.0, 0.8),
        baseline_residual=RelativeResidual(4.0, 0.0, 0.8),
        config=config,
    )


def test_route_progress_penalizes_wrong_hairpin_branch():
    config = TrackerConfig(max_hypotheses=2, non_monotonic_progress_penalty=20.0)
    tracker = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0), config)
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [
            NdtCandidate(Pose2D(1.0, 11.0, 0.0, 0.0), score=0.8, initial_to_result_m=0.2),
            NdtCandidate(Pose2D(1.0, 2.0, 0.0, 0.0), score=5.0, initial_to_result_m=0.2),
        ]
    )

    assert tracker.best().pose.x == 11.0


def test_route_progress_update_can_be_age_limited():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(route_progress_update_gain=1.0, route_progress_update_max_age_sec=1.0),
        initial_route_progress_m=0.0,
    )

    tracker.update(
        [NdtCandidate(Pose2D(0.5, 1.0, 0.0, 0.0), score=1.0)],
        route_priors={0: RoutePrior(progress_m=10.0, cross_track_m=0.0, yaw_error_rad=0.0)},
    )
    early_progress = tracker.best().route_progress_m

    tracker.update(
        [NdtCandidate(Pose2D(2.0, 2.0, 0.0, 0.0), score=1.0)],
        route_priors={tracker.best().id: RoutePrior(progress_m=100.0, cross_track_m=0.0, yaw_error_rad=0.0)},
    )

    assert early_progress == 10.0
    assert tracker.best().route_progress_m < 20.0


def test_tracker_can_add_same_pose_with_distinct_route_progress():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 10.0, 0.0, 0.0),
        TrackerConfig(max_hypotheses=4),
        initial_route_progress_m=10.0,
    )

    tracker.add_startup_hypothesis(
        Pose2D(0.0, 10.0, 0.0, 0.0),
        route_progress_m=100.0,
        score=-1.0,
        min_route_progress_separation_m=20.0,
    )

    progresses = sorted(round(item.route_progress_m, 3) for item in tracker.hypotheses)
    assert progresses == [10.0, 100.0]


def test_route_cross_target_learner_uses_only_trusted_non_degenerate_samples():
    learner = RouteCrossTargetLearner(alpha=0.5, gate_m=2.0, abs_limit_m=3.0)

    assert learner.update(1.0, along_degenerate=True) is None
    assert learner.update(4.0, along_degenerate=False) is None

    assert learner.update(-1.0, along_degenerate=False) == -1.0
    assert abs(learner.update(-1.4, along_degenerate=False) + 1.2) < 1e-9


def test_route_cross_target_learner_reports_missing_target_until_sample():
    learner = RouteCrossTargetLearner(alpha=0.5, gate_m=1.0, abs_limit_m=3.0)

    assert learner.has_value is False
    assert learner.value_or_none() is None
    learner.update(2.0, along_degenerate=False)
    assert learner.has_value is False
    learner.update(-0.5, along_degenerate=False)
    assert learner.has_value is True
    assert learner.value_or_none() == -0.5


def test_scan_point_count_degeneracy_is_default_off_and_thresholded():
    assert not scan_point_count_indicates_degeneracy(
        sampled_point_count=100,
        enabled=False,
        min_sampled_points=200,
    )
    assert scan_point_count_indicates_degeneracy(
        sampled_point_count=199,
        enabled=True,
        min_sampled_points=200,
    )
    assert not scan_point_count_indicates_degeneracy(
        sampled_point_count=200,
        enabled=True,
        min_sampled_points=200,
    )


def test_pointcloud_subscription_is_required_for_degeneracy_or_lro():
    assert not needs_pointcloud_subscription(
        enable_scan_submap_residual=False,
        enable_scan_point_count_degeneracy=False,
        enable_lro_forward_correction=False,
        enable_scan_geometry_degeneracy=False,
    )
    assert needs_pointcloud_subscription(
        enable_scan_submap_residual=True,
        enable_scan_point_count_degeneracy=False,
        enable_lro_forward_correction=False,
    )
    assert needs_pointcloud_subscription(
        enable_scan_submap_residual=False,
        enable_scan_point_count_degeneracy=True,
        enable_lro_forward_correction=False,
    )
    assert needs_pointcloud_subscription(
        enable_scan_submap_residual=False,
        enable_scan_point_count_degeneracy=False,
        enable_lro_forward_correction=True,
    )
    assert needs_pointcloud_subscription(
        enable_scan_submap_residual=False,
        enable_scan_point_count_degeneracy=False,
        enable_lro_forward_correction=False,
        enable_scan_geometry_degeneracy=True,
    )


def test_scan_geometry_certificate_detects_one_sided_along_corridor():
    points = [(float(x), 2.0 + 0.2 * ((x % 3) - 1)) for x in range(5, 55)]

    cert = scan_geometry_certificate(
        points,
        min_points=20,
        min_cross_side_points=5,
        min_along_to_cross_ratio=3.0,
        min_along_span_m=20.0,
        max_cross_span_m=4.0,
    )

    assert cert.along_degenerate
    assert cert.one_sided
    assert cert.right_count == 0
    assert cert.along_to_cross_ratio > 3.0


def test_scan_geometry_certificate_accepts_two_sided_structure():
    points = []
    for x in range(5, 35):
        points.append((float(x), -4.0))
        points.append((float(x), 4.0))

    cert = scan_geometry_certificate(
        points,
        min_points=20,
        min_cross_side_points=5,
        min_along_to_cross_ratio=3.0,
        min_along_span_m=20.0,
        max_cross_span_m=10.0,
    )

    assert not cert.one_sided
    assert not cert.along_degenerate


def test_scan_geometry_certificate_detects_one_sided_low_along_span():
    points = [(0.1 * float(i % 40), 2.0 + float(i // 40)) for i in range(400)]

    cert = scan_geometry_certificate(
        points,
        min_points=20,
        min_cross_side_points=5,
        min_along_to_cross_ratio=0.20,
        min_along_span_m=8.0,
        max_cross_span_m=80.0,
    )

    assert cert.one_sided
    assert cert.along_span_m < 8.0
    assert cert.along_degenerate


def test_route_frame_scan_geometry_certificate_uses_route_cross_axis():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    pose = Pose2D(1.0, 0.0, 0.0, 0.0)
    points = [(float(x), 3.0) for x in range(5, 55)]

    cert = route_frame_scan_geometry_certificate(
        points,
        pose,
        route,
        route_progress_m=0.0,
        min_points=20,
        min_cross_side_points=5,
        min_along_to_cross_ratio=3.0,
        min_along_span_m=20.0,
        max_cross_span_m=4.0,
    )

    assert cert.along_degenerate
    assert cert.one_sided
    assert cert.right_count == 0


def test_degenerate_velocity_scale_is_default_off_and_bounded():
    assert apply_degenerate_velocity_scale(
        10.0,
        degenerate=True,
        enabled=False,
        scale=0.5,
    ) == 10.0
    assert apply_degenerate_velocity_scale(
        10.0,
        degenerate=False,
        enabled=True,
        scale=0.5,
    ) == 10.0
    assert apply_degenerate_velocity_scale(
        10.0,
        degenerate=True,
        enabled=True,
        scale=0.8,
    ) == 8.0
    assert apply_degenerate_velocity_scale(
        10.0,
        degenerate=True,
        enabled=True,
        scale=2.0,
    ) == 10.0


def test_lro_forward_correction_is_default_off_and_bounded():
    pose = Pose2D(1.0, 10.0, 2.0, 0.0)
    estimate = RelativeOdometryEstimate(
        forward_m=1.4,
        lateral_m=0.0,
        yaw_rad=0.0,
        residual_m=0.1,
        quality=0.8,
        forward_variance_m2=0.1,
        lateral_variance_m2=0.1,
        yaw_variance_rad2=0.01,
        along_degenerate=False,
        is_valid=True,
    )

    assert apply_lro_forward_correction(
        pose,
        estimate,
        predicted_forward_m=1.0,
        enabled=False,
        gain=1.0,
        max_correction_m=1.0,
    ) == pose

    corrected = apply_lro_forward_correction(
        pose,
        estimate,
        predicted_forward_m=1.0,
        enabled=True,
        gain=0.5,
        max_correction_m=0.3,
    )
    assert abs(corrected.x - 10.2) < 1e-9
    assert corrected.y == pose.y


def test_lro_forward_correction_rejects_invalid_or_degenerate_estimate():
    pose = Pose2D(1.0, 10.0, 2.0, math.pi / 2.0)
    invalid = RelativeOdometryEstimate(
        forward_m=2.0,
        lateral_m=0.0,
        yaw_rad=0.0,
        residual_m=10.0,
        quality=0.0,
        forward_variance_m2=100.0,
        lateral_variance_m2=0.1,
        yaw_variance_rad2=0.01,
        along_degenerate=True,
        is_valid=False,
    )

    assert apply_lro_forward_correction(
        pose,
        invalid,
        predicted_forward_m=1.0,
        enabled=True,
        gain=1.0,
        max_correction_m=1.0,
    ) == pose


def test_twist_bias_learning_estimates_synthetic_positive_velocity_bias():
    result = update_twist_bias_estimate(
        current_bias_mps=0.0,
        anchor_pose=Pose2D(0.0, 0.0, 0.0, 0.0),
        current_pose=Pose2D(5.0, 10.25, 0.0, 0.0),
        integrated_forward_m=10.0,
        dt_sec=5.0,
        alpha=1.0,
        max_abs_mps=0.2,
        max_step_mps=0.2,
        min_dt_sec=2.0,
        max_dt_sec=8.0,
        max_lateral_m=0.5,
        max_yaw_rad=math.radians(2.0),
    )

    assert result.updated
    assert result.reason == "updated"
    assert abs(result.bias_mps - 0.05) < 1e-9


def test_twist_bias_learning_rejects_low_confidence_motion():
    common = dict(
        current_bias_mps=0.0,
        anchor_pose=Pose2D(0.0, 0.0, 0.0, 0.0),
        integrated_forward_m=10.0,
        dt_sec=5.0,
        alpha=1.0,
        max_abs_mps=0.2,
        max_step_mps=0.2,
        min_dt_sec=2.0,
        max_dt_sec=8.0,
        max_lateral_m=0.5,
        max_yaw_rad=math.radians(2.0),
    )

    lateral = update_twist_bias_estimate(
        current_pose=Pose2D(5.0, 10.0, 1.0, 0.0),
        **common,
    )
    yaw = update_twist_bias_estimate(
        current_pose=Pose2D(5.0, 10.0, 0.0, math.radians(5.0)),
        **common,
    )

    assert not lateral.updated
    assert lateral.reason == "lateral_too_large"
    assert not yaw.updated
    assert yaw.reason == "yaw_too_large"


def test_route_offset_pose_uses_pose_projection_progress():
    route = RoutePath([(0.0, 0.0), (10.0, 0.0)])
    pose = Pose2D(1.0, 5.0, 2.0, 0.3)

    corrected = route_offset_pose(
        route,
        pose,
        target_cross_m=1.0,
        gain=0.5,
        yaw_gain=0.0,
        gate_m=10.0,
        predicted_progress_m=None,
        search_radius_m=10.0,
    )

    assert corrected is not None
    assert abs(corrected.x - 5.0) < 1e-9
    assert abs(corrected.y - 1.5) < 1e-9
    assert abs(corrected.yaw - 0.3) < 1e-9


def test_degenerate_along_remap_keeps_predicted_forward_and_uses_lateral_yaw():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(enable_degenerate_along_remap=True),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 14.0, 2.0, 0.25), score=5.0)],
        along_degenerate=True,
    )

    best = tracker.best().pose
    assert abs(best.x - 10.0) < 1e-9
    assert abs(best.y - 2.0) < 1e-9
    assert abs(best.yaw - 0.25) < 1e-9


def test_candidate_localizability_can_trigger_along_remap_without_global_degeneracy():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(
            enable_degenerate_along_remap=True,
            enable_candidate_localizability_along_remap=True,
            candidate_localizability_along_min_variance_m2=4.0,
            candidate_localizability_along_min_ratio=4.0,
        ),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [
            NdtCandidate(
                Pose2D(1.0, 14.0, 2.0, 0.25),
                score=5.0,
                localizability_along_variance_m2=9.0,
                localizability_cross_variance_m2=0.5,
            )
        ],
        along_degenerate=False,
    )

    best = tracker.best().pose
    assert abs(best.x - 10.0) < 1e-9
    assert abs(best.y - 2.0) < 1e-9
    assert abs(best.yaw - 0.25) < 1e-9


def test_candidate_localizability_certificate_is_default_off():
    candidate = NdtCandidate(
        Pose2D(1.0, 14.0, 2.0, 0.25),
        score=5.0,
        localizability_along_variance_m2=9.0,
        localizability_cross_variance_m2=0.5,
    )

    assert not candidate_indicates_along_degeneracy(candidate, TrackerConfig())
    assert candidate_indicates_along_degeneracy(
        candidate,
        TrackerConfig(
            enable_candidate_localizability_along_remap=True,
            candidate_localizability_along_min_variance_m2=4.0,
            candidate_localizability_along_min_ratio=4.0,
        ),
    )


def test_degenerate_along_remap_can_keep_predicted_yaw():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.10),
        TrackerConfig(
            enable_degenerate_along_remap=True,
            degenerate_remap_keep_predicted_yaw=True,
        ),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.05))

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 14.0, 2.0, -0.60), score=5.0)],
        along_degenerate=True,
    )

    best = tracker.best().pose
    predicted = Pose2D(1.0, math.cos(0.10) * 10.0, math.sin(0.10) * 10.0, 0.15)
    forward, lateral, yaw_delta = planar_delta(predicted, best)
    _, ndt_lateral, _ = planar_delta(predicted, Pose2D(1.0, 14.0, 2.0, -0.60))
    assert abs(forward) < 1e-9
    assert abs(lateral - ndt_lateral) < 1e-9
    assert abs(yaw_delta) < 1e-9
    assert abs(best.yaw - 0.15) < 1e-9


def test_degenerate_mode_can_keep_predicted_yaw_without_remapping_xy():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.10),
        TrackerConfig(degenerate_keep_predicted_yaw_only=True),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.05))

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 11.0, 2.0, -0.60), score=5.0)],
        along_degenerate=True,
    )

    best = tracker.best().pose
    assert abs(best.x - 11.0) < 1e-9
    assert abs(best.y - 2.0) < 1e-9
    assert abs(best.yaw - 0.15) < 1e-9


def test_route_frame_degenerate_remap_keeps_predicted_progress_and_ndt_cross():
    route = RoutePath([(0.0, 0.0), (100.0, 0.0)])
    predicted = Pose2D(1.0, 10.0, 0.2, math.radians(20.0))
    ndt = Pose2D(1.0, 14.0, 1.5, math.radians(2.0))

    remapped = route_remap_candidate_along_to_prediction(
        route,
        predicted,
        ndt,
        keep_predicted_yaw=False,
        search_radius_m=20.0,
    )

    assert remapped is not None
    assert abs(remapped.x - 10.0) < 1e-9
    assert abs(remapped.y - 1.5) < 1e-9
    assert abs(remapped.yaw - math.radians(2.0)) < 1e-9


def test_low_score_candidate_can_trigger_along_remap_without_rejection():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(
            enable_degenerate_along_remap=True,
            enable_candidate_low_score_along_remap=True,
            candidate_low_score_along_remap_threshold=2.3,
            max_candidate_initial_to_result_m=10.0,
            min_candidate_score=-math.inf,
            missed_update_penalty=0.0,
        ),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 14.0, 2.0, 0.1), score=1.7, initial_to_result_m=0.5)]
    )

    best = tracker.best().pose
    assert abs(best.x - 10.0) < 1e-9
    assert abs(best.y - 2.0) < 1e-9
    assert abs(best.yaw - 0.1) < 1e-9


def test_low_score_along_remap_uses_nvtl_not_total_score_when_available():
    config = TrackerConfig(
        enable_candidate_low_score_along_remap=True,
        candidate_low_score_along_remap_threshold=2.3,
    )
    high_nvtl = NdtCandidate(
        Pose2D(1.0, 14.0, 2.0, 0.1),
        score=-30.0,
        nearest_voxel_transformation_likelihood=3.0,
    )
    low_nvtl = NdtCandidate(
        Pose2D(1.0, 14.0, 2.0, 0.1),
        score=30.0,
        nearest_voxel_transformation_likelihood=1.7,
    )

    assert not candidate_indicates_along_degeneracy(high_nvtl, config)
    assert candidate_indicates_along_degeneracy(low_nvtl, config)


def test_degenerate_mode_can_skip_ndt_candidates_and_coast():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(degenerate_skip_ndt_candidates=True),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.1))

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 30.0, 4.0, 0.9), score=10.0)],
        along_degenerate=True,
    )

    best = tracker.best().pose
    assert abs(best.x - 10.0) < 1e-9
    assert abs(best.y) < 1e-9
    assert abs(best.yaw - 0.1) < 1e-9


def test_submap_candidate_consistency_gate_is_default_off():
    hypothesis = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0)).best()
    candidate = NdtCandidate(
        Pose2D(0.0, 0.0, 5.0, math.radians(20.0)),
        score=1.0,
        source="scan_submap_local_ndt",
    )

    assert submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=None,
        relative_residual=None,
        config=TrackerConfig(),
    )


def test_submap_candidate_consistency_gate_rejects_unverified_synthetic_candidate():
    hypothesis = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0)).best()
    candidate = NdtCandidate(
        Pose2D(0.0, 0.5, 0.2, math.radians(1.0)),
        score=1.0,
        source="global_map_local_ndt",
    )

    assert not submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=RoutePrior(0.5, 0.2, math.radians(1.0)),
        relative_residual=None,
        config=TrackerConfig(enable_submap_candidate_consistency_gate=True),
    )


def test_submap_candidate_consistency_gate_accepts_supported_synthetic_candidate():
    hypothesis = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0)).best()
    candidate = NdtCandidate(
        Pose2D(0.0, 0.5, 0.2, math.radians(1.0)),
        score=1.0,
        source="scan_submap_local_ndt",
    )

    assert submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=RoutePrior(0.5, 0.2, math.radians(1.0)),
        relative_residual=RelativeResidual(0.3, math.radians(0.5), 0.8, True),
        config=TrackerConfig(enable_submap_candidate_consistency_gate=True),
    )


def test_submap_candidate_consistency_gate_requires_residual_improvement_when_configured():
    hypothesis = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0)).best()
    candidate = NdtCandidate(
        Pose2D(0.0, 0.5, 0.2, math.radians(1.0)),
        score=1.0,
        source="scan_submap_local_ndt",
    )
    config = TrackerConfig(
        enable_submap_candidate_consistency_gate=True,
        submap_candidate_min_residual_improvement_m=0.2,
    )

    assert not submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=RoutePrior(0.5, 0.2, math.radians(1.0)),
        relative_residual=RelativeResidual(0.95, 0.0, 0.8, True),
        baseline_residual=RelativeResidual(1.0, 0.0, 0.8, True),
        config=config,
    )
    assert submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=RoutePrior(0.5, 0.2, math.radians(1.0)),
        relative_residual=RelativeResidual(0.5, 0.0, 0.8, True),
        baseline_residual=RelativeResidual(1.0, 0.0, 0.8, True),
        config=config,
    )


def test_submap_candidate_consistency_gate_does_not_filter_raw_ndt_candidate():
    hypothesis = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0)).best()
    candidate = NdtCandidate(
        Pose2D(0.0, 0.0, 5.0, math.radians(20.0)),
        score=1.0,
        source="ndt",
    )

    assert submap_candidate_consistency_is_valid(
        hypothesis,
        candidate,
        route_prior=None,
        relative_residual=None,
        config=TrackerConfig(enable_submap_candidate_consistency_gate=True),
    )


def test_payload_along_degeneracy_uses_spread_covariance_ratio():
    assert payload_indicates_along_degeneracy(
        {
            "spread_covariance_ambiguous": True,
            "spread_covariance_along_m2": 9.0,
            "spread_covariance_cross_m2": 0.25,
        },
        min_along_variance_m2=4.0,
        min_along_to_cross_ratio=4.0,
    )
    assert not payload_indicates_along_degeneracy(
        {
            "spread_covariance_ambiguous": True,
            "spread_covariance_along_m2": 1.0,
            "spread_covariance_cross_m2": 0.25,
        },
        min_along_variance_m2=4.0,
        min_along_to_cross_ratio=4.0,
    )


def test_payload_along_degeneracy_uses_single_candidate_covariance():
    assert payload_indicates_along_degeneracy(
        {
            "spread_covariance_ambiguous": False,
            "candidates": [
                {
                    "localizability_along_variance_m2": 9.0,
                    "localizability_cross_variance_m2": 0.5,
                }
            ],
        },
        min_along_variance_m2=1.0,
        min_along_to_cross_ratio=4.0,
    )


def test_payload_along_degeneracy_uses_selected_output_covariance():
    assert payload_indicates_along_degeneracy(
        {
            "selected_output_covariance_along_m2": 6.0,
            "selected_output_covariance_cross_m2": 0.4,
        },
        min_along_variance_m2=1.0,
        min_along_to_cross_ratio=4.0,
    )


def test_payload_along_degeneracy_uses_candidate_innovation_spread():
    payload = {
        "candidates": [
            {"innovation_along_m": -1.2, "innovation_cross_m": 0.05},
            {"innovation_along_m": 0.0, "innovation_cross_m": 0.00},
            {"innovation_along_m": 1.2, "innovation_cross_m": -0.05},
        ]
    }

    assert payload_indicates_along_degeneracy(
        payload,
        min_along_variance_m2=1.0,
        min_along_to_cross_ratio=4.0,
    )


def test_payload_along_degeneracy_rejects_cross_spread():
    payload = {
        "candidates": [
            {"innovation_along_m": -1.2, "innovation_cross_m": -1.0},
            {"innovation_along_m": 0.0, "innovation_cross_m": 0.0},
            {"innovation_along_m": 1.2, "innovation_cross_m": 1.0},
        ]
    }

    assert not payload_indicates_along_degeneracy(
        payload,
        min_along_variance_m2=1.0,
        min_along_to_cross_ratio=4.0,
    )


def test_degenerate_along_rejects_large_lateral_candidate():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 10.0, 0.0, 0.0),
        TrackerConfig(
            enable_degenerate_along_remap=True,
            degenerate_max_lateral_m=0.5,
            missed_update_penalty=0.0,
            ndt_score_weight=10.0,
        ),
    )

    tracker.update(
        [NdtCandidate(Pose2D(1.0, 10.0, 2.0, 0.0), score=100.0)],
        along_degenerate=True,
    )

    best = tracker.best().pose
    assert best.x == 10.0
    assert best.y == 0.0


def test_route_path_projects_pose_with_signed_cross_track_and_progress():
    route = RoutePath([(0.0, 0.0), (10.0, 0.0), (20.0, 10.0)])

    projection = route.project(Pose2D(0.0, 5.0, 2.0, 0.1), predicted_progress_m=5.0)

    assert abs(projection.progress_m - 5.0) < 1e-9
    assert abs(projection.cross_track_m - 2.0) < 1e-9
    assert abs(projection.yaw_error_rad - 0.1) < 1e-9


def test_route_path_can_return_multiple_nearby_branch_projections():
    route = RoutePath(
        [
            (0.0, 0.0),
            (10.0, 0.0),
            (20.0, 0.0),
            (20.0, 10.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.2),
            (10.0, 0.2),
            (20.0, 0.2),
        ]
    )

    projections = route.project_candidates(
        Pose2D(0.0, 10.0, 0.1, 0.0),
        max_distance_m=0.5,
        max_candidates=3,
    )

    assert len(projections) >= 2
    progresses = [item.progress_m for item in projections]
    assert min(progresses) < 12.0
    assert max(progresses) > 60.0


def test_tracker_keeps_coasting_branch_when_candidate_is_plausible_but_wrong():
    config = TrackerConfig(
        max_hypotheses=2,
        missed_update_penalty=0.1,
        ndt_score_weight=1.0,
        initial_to_result_penalty_weight=0.1,
        lateral_jump_weight=0.1,
    )
    tracker = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0), config)
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [
            NdtCandidate(Pose2D(1.0, 10.0, 2.0, 0.0), score=0.8, initial_to_result_m=0.4),
        ]
    )

    poses = {(round(h.pose.x, 6), round(h.pose.y, 6)) for h in tracker.hypotheses}
    assert (10.0, 0.0) in poses
    assert (10.0, 2.0) in poses


def test_relative_and_route_residuals_can_override_single_frame_score():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(
            max_hypotheses=2,
            route_cross_weight=5.0,
            relative_residual_weight=5.0,
            lateral_jump_weight=5.0,
        ),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=10.0, lateral_m=0.0, yaw_rad=0.0))

    candidates = [
        NdtCandidate(Pose2D(1.0, 10.0, 0.0, 0.0), score=1.0, initial_to_result_m=0.2),
        NdtCandidate(Pose2D(1.0, 10.0, 3.0, 0.0), score=3.0, initial_to_result_m=0.2),
    ]
    tracker.update(candidates)

    # Both candidates are kept after the first ambiguous frame.
    assert len(tracker.hypotheses) == 2

    # The next update gives the temporally/route-consistent basin lower raw NDT
    # score but much better residuals, so it becomes best.
    tracker.update(
        candidates,
        route_priors={
            tracker.hypotheses[0].id: RoutePrior(10.0, 0.0, 0.0),
            tracker.hypotheses[1].id: RoutePrior(10.0, 3.0, 0.0),
        },
        relative_residuals={
            tracker.hypotheses[0].id: RelativeResidual(0.1, 0.0, 1.0),
            tracker.hypotheses[1].id: RelativeResidual(3.0, 0.0, 0.1),
        },
    )

    assert abs(tracker.best().pose.y) < 1e-9


def test_candidate_specific_scan_submap_residual_selects_temporally_consistent_basin():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    scan = [(2.0, 0.0), (3.0, 0.2), (4.0, -0.1), (5.0, 0.0)]
    submap.add_scan(scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    current_scan = [(1.0, 0.0), (2.0, 0.2), (3.0, -0.1), (4.0, 0.0)]
    correct = NdtCandidate(Pose2D(1.0, 1.0, 0.0, 0.0), score=1.0, initial_to_result_m=0.2)
    wrong = NdtCandidate(Pose2D(1.0, 1.0, 5.0, 0.0), score=3.0, initial_to_result_m=0.2)
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(max_hypotheses=2, relative_residual_weight=5.0),
    )
    tracker.propagate(MotionDelta(dt_sec=1.0, forward_m=1.0, lateral_m=0.0, yaw_rad=0.0))

    tracker.update(
        [correct, wrong],
        relative_residuals={
            (tracker.best().id, 0): submap.residual(current_scan, correct.pose),
            (tracker.best().id, 1): submap.residual(current_scan, wrong.pose),
        },
    )

    assert abs(tracker.best().pose.y) < 1e-9


def test_scan_submap_refine_pose_corrects_local_coasting_offset():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [(2.0, 0.0), (3.0, 0.2), (4.0, -0.1), (5.0, 0.0)]
    current_scan = [(1.0, 0.0), (2.0, 0.2), (3.0, -0.1), (4.0, 0.0)]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    seed = Pose2D(1.0, 1.0, 0.8, 0.0)
    refined, residual = submap.refine_pose(
        current_scan,
        seed,
        forward_offsets_m=(-0.5, 0.0, 0.5),
        lateral_offsets_m=(-1.0, 0.0, 1.0),
        yaw_offsets_rad=(0.0,),
    )

    assert abs(refined.x - 1.0) < 1e-9
    assert abs(refined.y) < 0.25
    assert residual.is_valid


def test_scan_submap_refine_pose_profile_score_corrects_along_offset():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, -0.3),
        (2.0, 0.0),
        (2.0, 0.3),
        (4.0, -0.3),
        (4.0, 0.0),
        (4.0, 0.3),
        (7.0, -0.3),
        (7.0, 0.0),
        (7.0, 0.3),
    ]
    current_scan = [
        (1.0, -0.3),
        (1.0, 0.0),
        (1.0, 0.3),
        (3.0, -0.3),
        (3.0, 0.0),
        (3.0, 0.3),
        (6.0, -0.3),
        (6.0, 0.0),
        (6.0, 0.3),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    refined, residual = submap.refine_pose(
        current_scan,
        Pose2D(1.0, 2.0, 0.0, 0.0),
        forward_offsets_m=(-1.0, 0.0, 1.0),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        profile_score_weight=2.0,
    )

    assert residual.is_valid
    assert abs(refined.x - 1.0) < abs(2.0 - 1.0)


def test_profile_along_correction_requires_interior_margin_certificate():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, -0.3),
        (2.0, 0.0),
        (2.0, 0.3),
        (4.0, -0.3),
        (4.0, 0.0),
        (4.0, 0.3),
        (7.0, -0.3),
        (7.0, 0.0),
        (7.0, 0.3),
    ]
    current_scan = [
        (1.0, -0.3),
        (1.0, 0.0),
        (1.0, 0.3),
        (3.0, -0.3),
        (3.0, 0.0),
        (3.0, 0.3),
        (6.0, -0.3),
        (6.0, 0.0),
        (6.0, 0.3),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    accepted = submap.profile_along_correction(
        current_scan,
        Pose2D(1.0, 2.0, 0.0, 0.0),
        forward_offsets_m=(-2.0, -1.0, 0.0, 1.0, 2.0),
        min_second_best_margin_m=0.05,
    )
    assert accepted is not None
    corrected_pose, residual = accepted
    assert residual.is_valid
    assert abs(corrected_pose.x - 1.0) < abs(2.0 - 1.0)

    rejected_boundary = submap.profile_along_correction(
        current_scan,
        Pose2D(1.0, 3.0, 0.0, 0.0),
        forward_offsets_m=(-2.0, -1.0, 0.0),
        min_second_best_margin_m=0.0,
    )
    assert rejected_boundary is None


def test_longitudinal_profile_residual_penalizes_along_shift():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0)]
    current_scan = [(1.0, 0.0), (3.0, 0.0), (5.0, 0.0), (7.0, 0.0)]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    correct = submap.longitudinal_profile_residual(
        current_scan,
        Pose2D(1.0, 1.0, 0.0, 0.0),
        lateral_bin_m=0.5,
    )
    shifted = submap.longitudinal_profile_residual(
        current_scan,
        Pose2D(1.0, 2.0, 0.0, 0.0),
        lateral_bin_m=0.5,
    )

    assert correct.is_valid
    assert shifted.is_valid
    assert correct.xy_m < shifted.xy_m


def test_local_icp_residual_penalizes_required_pose_correction():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, 0.0),
        (3.0, 0.5),
        (4.0, -0.4),
        (5.0, 0.2),
        (6.0, -0.1),
    ]
    current_scan = [
        (1.0, 0.0),
        (2.0, 0.5),
        (3.0, -0.4),
        (4.0, 0.2),
        (5.0, -0.1),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    correct = submap.local_icp_residual(
        current_scan,
        Pose2D(1.0, 1.0, 0.0, 0.0),
        correction_penalty_weight=1.0,
    )
    shifted = submap.local_icp_residual(
        current_scan,
        Pose2D(1.0, 2.0, 0.0, 0.0),
        correction_penalty_weight=1.0,
    )

    assert correct.is_valid
    assert shifted.is_valid
    assert correct.xy_m < shifted.xy_m


def test_local_icp_pose_candidate_returns_bounded_correction():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, 0.0),
        (3.0, 0.5),
        (4.0, -0.4),
        (5.0, 0.2),
        (6.0, -0.1),
    ]
    current_scan = [
        (1.0, 0.0),
        (2.0, 0.5),
        (3.0, -0.4),
        (4.0, 0.2),
        (5.0, -0.1),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    result = submap.local_icp_pose_candidate(
        current_scan,
        Pose2D(1.0, 1.0, 0.7, 0.0),
        max_correction_m=1.0,
        min_quality=0.2,
    )

    assert result is not None
    pose, residual = result
    assert residual.is_valid
    assert residual.quality >= 0.2
    assert abs(pose.x - 1.0) < 0.2
    assert abs(pose.y) < 0.4


def test_local_icp_pose_candidate_rejects_large_correction():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [(2.0, 0.0), (3.0, 0.5), (4.0, -0.4), (5.0, 0.2)]
    current_scan = [(1.0, 0.0), (2.0, 0.5), (3.0, -0.4), (4.0, 0.2)]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    result = submap.local_icp_pose_candidate(
        current_scan,
        Pose2D(1.0, 3.0, 0.0, 0.0),
        max_correction_m=0.3,
        min_quality=0.2,
    )

    assert result is None


def test_local_ndt_pose_candidates_score_bounded_offsets():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, -0.4),
        (2.0, 0.0),
        (2.0, 0.4),
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
        (5.0, -0.4),
        (5.0, 0.0),
        (5.0, 0.4),
    ]
    current_scan = [
        (1.0, -0.4),
        (1.0, 0.0),
        (1.0, 0.4),
        (2.0, -0.4),
        (2.0, 0.0),
        (2.0, 0.4),
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    candidates = submap.local_ndt_pose_candidates(
        current_scan,
        Pose2D(1.0, 0.2, 0.2, 0.0),
        forward_offsets_m=(-0.5, 0.0, 0.5),
        lateral_offsets_m=(-0.5, 0.0, 0.5),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
        max_candidates=3,
    )

    assert candidates
    best_pose, best_residual = candidates[0]
    assert best_residual.is_valid
    assert best_residual.quality >= 0.2
    assert abs(best_pose.x - 1.0) <= 0.6
    assert abs(best_pose.y) <= 0.6


def test_local_ndt_pose_candidates_reuses_gaussian_cell_cache():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    scan = [
        (2.0, -0.4),
        (2.0, 0.0),
        (2.0, 0.4),
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
    ]
    submap.add_scan(scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    first = submap.local_ndt_pose_candidates(
        scan,
        Pose2D(1.0, 0.0, 0.0, 0.0),
        forward_offsets_m=(0.0,),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
    )
    assert first
    assert len(submap._gaussian_cell_cache) == 1
    cached = next(iter(submap._gaussian_cell_cache.values()))

    second = submap.local_ndt_pose_candidates(
        scan,
        Pose2D(1.0, 0.0, 0.0, 0.0),
        forward_offsets_m=(0.0,),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
    )
    assert second
    assert next(iter(submap._gaussian_cell_cache.values())) is cached

    submap.add_world_points([(10.0, 10.0)])
    assert submap._gaussian_cell_cache == {}


def test_local_ndt_pose_candidates_can_use_longitudinal_profile_score():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (2.0, -0.3),
        (2.0, 0.0),
        (2.0, 0.3),
        (4.0, -0.3),
        (4.0, 0.0),
        (4.0, 0.3),
        (7.0, -0.3),
        (7.0, 0.0),
        (7.0, 0.3),
    ]
    current_scan = [
        (1.0, -0.3),
        (1.0, 0.0),
        (1.0, 0.3),
        (3.0, -0.3),
        (3.0, 0.0),
        (3.0, 0.3),
        (6.0, -0.3),
        (6.0, 0.0),
        (6.0, 0.3),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    candidates = submap.local_ndt_pose_candidates(
        current_scan,
        Pose2D(1.0, 1.8, 0.0, 0.0),
        forward_offsets_m=(-1.0, 0.0, 1.0),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
        max_candidates=3,
        profile_score_weight=2.0,
    )

    assert candidates
    best_pose, best_residual = candidates[0]
    assert best_residual.is_valid
    assert abs(best_pose.x - 1.0) < abs(best_pose.x - 2.8)


def test_local_ndt_pose_candidates_reject_boundary_best_when_guarded():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    previous_scan = [
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
        (5.0, -0.4),
        (5.0, 0.0),
        (5.0, 0.4),
        (6.0, -0.4),
        (6.0, 0.0),
        (6.0, 0.4),
    ]
    current_scan = [
        (1.0, -0.4),
        (1.0, 0.0),
        (1.0, 0.4),
        (2.0, -0.4),
        (2.0, 0.0),
        (2.0, 0.4),
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
    ]
    submap.add_scan(previous_scan, Pose2D(0.0, 0.0, 0.0, 0.0))

    candidates = submap.local_ndt_pose_candidates(
        current_scan,
        Pose2D(1.0, 0.0, 0.0, 0.0),
        forward_offsets_m=(0.0, 1.0, 2.0),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
        reject_boundary_best=True,
    )

    assert candidates == []


def test_local_ndt_pose_candidates_reject_ambiguous_second_best_when_guarded():
    submap = LightweightScanSubmap(voxel_size_m=0.5, neighbor_radius_cells=1)
    template = [
        (1.0, -0.4),
        (1.0, 0.0),
        (1.0, 0.4),
        (2.0, -0.4),
        (2.0, 0.0),
        (2.0, 0.4),
        (3.0, -0.4),
        (3.0, 0.0),
        (3.0, 0.4),
        (4.0, -0.4),
        (4.0, 0.0),
        (4.0, 0.4),
    ]
    submap.add_scan(template, Pose2D(0.0, 0.0, 0.0, 0.0))
    submap.add_scan(template, Pose2D(0.0, 1.0, 0.0, 0.0))

    candidates = submap.local_ndt_pose_candidates(
        template,
        Pose2D(1.0, 0.0, 0.0, 0.0),
        forward_offsets_m=(0.0, 1.0),
        lateral_offsets_m=(0.0,),
        yaw_offsets_rad=(0.0,),
        min_quality=0.2,
        min_cell_points=2,
        min_second_best_score_margin=0.5,
    )

    assert candidates == []


def test_rejected_or_large_jump_candidates_are_not_used():
    tracker = FixedLagMultiHypothesisTracker(Pose2D(0.0, 0.0, 0.0, 0.0))
    tracker.update(
        [
            NdtCandidate(Pose2D(0.1, 1.0, 0.0, 0.0), score=10.0, converged=False),
            NdtCandidate(
                Pose2D(0.1, 2.0, 0.0, 0.0),
                score=10.0,
                initial_to_result_m=99.0,
            ),
            NdtCandidate(Pose2D(0.1, 3.0, 0.0, 0.0), score=10.0, rejection_reason="bad"),
        ]
    )

    assert tracker.best().pose.x == 0.0
    assert tracker.best().missed_updates == 1


def test_best_hypothesis_uses_switch_margin_to_delay_commitment():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(0.0, 0.0, 0.0, 0.0),
        TrackerConfig(best_switch_score_margin=2.0),
    )

    tracker.update(
        [
            NdtCandidate(
                Pose2D(0.1, 1.0, 0.0, 0.0),
                score=1.0,
                initial_to_result_m=0.0,
            )
        ]
    )

    assert tracker.best().pose.x == 0.0

    tracker.update(
        [
            NdtCandidate(
                Pose2D(0.2, 2.0, 0.0, 0.0),
                score=10.0,
                initial_to_result_m=0.0,
            )
        ]
    )

    assert tracker.best().pose.x == 2.0


def test_candidate_usable_helper_matches_tracker_rejection_rules():
    config = TrackerConfig(max_candidate_initial_to_result_m=2.0, min_candidate_score=1.0)

    assert candidate_is_usable(
        NdtCandidate(Pose2D(0.0, 0.0, 0.0, 0.0), score=1.1, initial_to_result_m=1.0),
        config,
    )
    assert not candidate_is_usable(
        NdtCandidate(Pose2D(0.0, 0.0, 0.0, 0.0), score=10.0, converged=False),
        config,
    )
    assert not candidate_is_usable(
        NdtCandidate(
            Pose2D(0.0, 0.0, 0.0, 0.0),
            score=10.0,
            initial_to_result_m=1.0,
            rejection_reason="score_below_threshold",
        ),
        config,
    )
    assert not candidate_is_usable(
        NdtCandidate(Pose2D(0.0, 0.0, 0.0, 0.0), score=0.9, initial_to_result_m=1.0),
        config,
    )
    assert not candidate_is_usable(
        NdtCandidate(Pose2D(0.0, 0.0, 0.0, 0.0), score=10.0, initial_to_result_m=2.1),
        config,
    )


def test_runtime_multistart_payload_is_converted_to_tracker_candidates():
    payload = {
        "stamp_sec": 12.5,
        "candidates": [
            {
                "result_x": 1.0,
                "result_y": 2.0,
                "result_yaw_deg": 90.0,
                "total_score": 3.5,
                "converged": True,
                "initial_to_result_distance_m": 0.4,
                "innovation_yaw_deg": 1.5,
                "reject_reason": "",
            },
            {
                "result_x": 5.0,
                "result_y": 6.0,
                "result_yaw_deg": 0.0,
                "nearest_voxel_transformation_likelihood": 2.0,
                "converged": False,
                "initial_to_result_distance_m": 0.1,
                "reject_reason": "not_converged",
            },
        ],
    }

    candidates = candidates_from_runtime_multistart(payload)

    assert len(candidates) == 2
    assert candidates[0].pose.stamp_sec == 12.5
    assert candidates[0].pose.x == 1.0
    assert candidates[0].pose.y == 2.0
    assert abs(candidates[0].pose.yaw - math.pi / 2.0) < 1e-9
    assert candidates[0].score == 3.5
    assert candidates[0].initial_to_result_m == 0.4
    assert abs(candidates[0].initial_to_result_yaw_rad - math.radians(1.5)) < 1e-9
    assert candidates[1].score == 2.0
    assert candidates[1].rejection_reason == "not_converged"


def test_route_filter_candidates_uses_local_route_progress_on_hairpin():
    route = RoutePath(
        [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 4.0),
            (0.0, 4.0),
        ]
    )
    correct_branch = NdtCandidate(Pose2D(1.0, 9.0, 0.2, 0.0), score=1.0)
    wrong_branch = NdtCandidate(Pose2D(1.0, 9.0, 3.8, math.pi), score=2.0)

    filtered = route_filter_candidates(
        [correct_branch, wrong_branch],
        route,
        cross_gate_m=1.0,
        yaw_gate_rad=math.radians(20.0),
        predicted_progress_m=9.0,
        search_radius_m=4.0,
    )

    assert filtered == [correct_branch]


def test_startup_hypotheses_keep_later_route_better_fallback_branch():
    tracker = FixedLagMultiHypothesisTracker(
        Pose2D(1.0, 0.0, 0.0, math.radians(5.0)),
        initial_score=-1.0,
    )

    tracker.add_startup_hypothesis(
        Pose2D(1.1, 1.0, 0.0, math.radians(1.0)),
        route_progress_m=1.0,
        score=0.5,
    )

    assert len(tracker.hypotheses) == 2
    assert tracker.best().pose.x == 1.0
