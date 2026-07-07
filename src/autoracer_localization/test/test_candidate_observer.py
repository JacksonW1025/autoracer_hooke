import math

from autoracer_localization.candidate_observer import (
    GnssWeakPrior,
    NdtDebugMetrics,
    Pose2D,
    build_candidate_payload,
    offset_pose,
)
from autoracer_localization.runtime_candidate_selector import (
    CausalFixedLagSelector,
    SelectorConfig,
)


def test_offset_pose_uses_body_frame_and_yaw_offset():
    seed = Pose2D(stamp_sec=1.0, x=10.0, y=20.0, z=0.0, yaw=math.radians(90.0))

    pose = offset_pose(seed, along_m=2.0, cross_m=1.0, yaw_deg=3.0)

    assert math.isclose(pose.x, 9.0, abs_tol=1e-9)
    assert math.isclose(pose.y, 22.0, abs_tol=1e-9)
    assert math.isclose(math.degrees(pose.yaw), 93.0, abs_tol=1e-9)


def test_observer_payload_is_non_invasive_and_marks_unaligned_hypotheses_rejected():
    base = Pose2D(
        stamp_sec=12.5,
        x=100.0,
        y=200.0,
        z=1.0,
        yaw=0.25,
        covariance=(0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.09),
    )
    gnss = GnssWeakPrior(
        pose=Pose2D(stamp_sec=12.4, x=103.0, y=204.0, z=1.0, yaw=0.0),
        age_sec=0.1,
        sigma_m=5.0,
        max_penalty=8.0,
    )

    payload = build_candidate_payload(
        base_pose=base,
        metrics=NdtDebugMetrics(
            transform_probability=3.2,
            nearest_voxel_transformation_likelihood=2.4,
            iteration_count=12,
            initial_to_result_distance_m=0.2,
            metric_age_sec=0.01,
        ),
        weak_prior=gnss,
        offset_along_m=(0.0, 1.0),
        offset_cross_m=(0.0,),
        offset_yaw_deg=(0.0,),
        gnss_max_age_sec=0.5,
        max_iterations=80,
        default_transform_probability=3.0,
        default_nearest_voxel_transformation_likelihood=2.3,
    )

    assert payload["reason"] == "runtime_candidate_observer"
    assert payload["source"] == "independent_candidate_observer"
    assert payload["controls_output"] is False
    assert payload["controls_final_localization"] is False
    assert payload["uses_gt"] is False
    assert payload["uses_future_frames"] is False
    assert payload["gnss_usage"] == "weak_penalty_only"
    assert payload["selected_candidate_index"] == 0

    base_candidate = payload["candidates"][0]
    assert base_candidate["source"] == "base_ndt_pose_snapshot"
    assert base_candidate["selected_by_observer"] is True
    assert base_candidate["nearest_voxel_transformation_likelihood"] == 2.4
    assert base_candidate["gnss_weak_prior_distance_m"] == 5.0

    unaligned = payload["candidates"][1]
    assert unaligned["converged"] is False
    assert unaligned["selected_by_observer"] is False
    assert unaligned["rejection_reason"] == "not_lidar_aligned_independent_observer_hypothesis"


def test_unaligned_observer_hypotheses_cannot_drive_selector_takeover():
    base = Pose2D(stamp_sec=1.0, x=0.0, y=0.0, z=0.0, yaw=0.0)
    payload = build_candidate_payload(
        base_pose=base,
        metrics=NdtDebugMetrics(
            transform_probability=1.0,
            nearest_voxel_transformation_likelihood=0.5,
            iteration_count=10,
            initial_to_result_distance_m=1.2,
            metric_age_sec=0.01,
        ),
        weak_prior=None,
        offset_along_m=(0.0, 1.0),
        offset_cross_m=(0.0,),
        offset_yaw_deg=(0.0,),
        gnss_max_age_sec=0.5,
        max_iterations=80,
        default_transform_probability=3.0,
        default_nearest_voxel_transformation_likelihood=2.3,
    )

    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=1, min_nvtl=0.1))
    decision = selector.update(payload)

    assert decision.selected is not None
    assert decision.selected.index == 0
    assert not decision.allow_takeover
    assert decision.rejected_takeover_reason == "base_candidate_selected"
