import math

from autoracer_localization.runtime_candidate_selector import (
    CausalFixedLagSelector,
    SelectorConfig,
    candidate_is_plausible,
    parse_observer_payload,
)


def _candidate(index, x, *, nvtl=3.0, offset_along=0.0, converged=True, gnss_dist=4.0):
    return {
        "index": index,
        "initial_x": 0.0,
        "initial_y": 0.0,
        "initial_z": 0.0,
        "initial_yaw_deg": 0.0,
        "result_x": x,
        "result_y": 0.0,
        "result_z": 0.0,
        "result_yaw_deg": 0.0,
        "offset_along_m": offset_along,
        "offset_cross_m": 0.0,
        "offset_yaw_deg": 0.0,
        "converged": converged,
        "iteration_count": 5,
        "max_iterations": 40,
        "transform_probability": 6.0,
        "nearest_voxel_transformation_likelihood": nvtl,
        "score": nvtl,
        "total_score": nvtl,
        "initial_to_result_distance_m": 0.2,
        "initial_to_result_yaw_deg": 0.0,
        "innovation_along_m": x,
        "innovation_cross_m": 0.0,
        "innovation_yaw_deg": 0.0,
        "localizability_along_variance_m2": 0.1,
        "localizability_cross_variance_m2": 0.1,
        "covariance_condition_number": 1.0,
        "gnss_weak_prior_distance_m": gnss_dist,
        "gnss_weak_prior_penalty": gnss_dist * gnss_dist / 50.0,
        "reject_reason": "",
    }


def _payload(stamp, candidates):
    return {
        "stamp_sec": stamp,
        "reason": "runtime_candidate_observer",
        "uses_gnss_or_gt": False,
        "controls_output": False,
        "candidates": candidates,
    }


def test_observer_payload_serialization_contract_is_parsed():
    parsed = parse_observer_payload(_payload(10.0, [_candidate(1, 2.0, offset_along=1.0)]))
    assert len(parsed) == 1
    assert parsed[0].pose.stamp_sec == 10.0
    assert parsed[0].pose.x == 2.0
    assert parsed[0].offset_along_m == 1.0
    assert parsed[0].gnss_weak_prior_distance_m == 4.0


def test_gnss5m_is_only_weak_gate_not_nearest_selector():
    selector = CausalFixedLagSelector(
        SelectorConfig(stable_required_frames=1, max_gnss_weak_prior_distance_m=15.0)
    )
    far_from_gnss_but_lidar_best = _candidate(1, 1.0, nvtl=4.0, offset_along=1.0, gnss_dist=8.0)
    near_gnss_low_lidar_score = _candidate(2, 10.0, nvtl=1.2, offset_along=2.0, gnss_dist=0.5)
    decision = selector.update(
        _payload(
            1.0,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                far_from_gnss_but_lidar_best,
                near_gnss_low_lidar_score,
            ],
        )
    )
    assert decision.selected is not None
    assert decision.selected.index == 1


def test_offset_penalty_breaks_near_tie_toward_small_seed_offset():
    selector = CausalFixedLagSelector(
        SelectorConfig(
            stable_required_frames=1,
            offset_xy_weight=0.02,
            max_initial_to_result_m=2.0,
        )
    )
    near_center = _candidate(1, 0.0, nvtl=1.698, offset_along=0.0, gnss_dist=4.0)
    far_offset_wrong_basin = _candidate(2, 5.0, nvtl=1.712, offset_along=5.0, gnss_dist=4.0)

    decision = selector.update(
        _payload(1.0, [_candidate(0, 0.0, nvtl=0.8, converged=False), near_center, far_offset_wrong_basin])
    )

    assert decision.selected is not None
    assert decision.selected.index == 1


def test_selector_requires_consecutive_stability_before_takeover():
    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=3))
    decisions = []
    for stamp in (1.0, 2.0, 3.0):
        decisions.append(
            selector.update(
                _payload(
                    stamp,
                    [
                        _candidate(0, 0.0, nvtl=0.8, converged=False),
                        _candidate(1, stamp, nvtl=3.0, offset_along=1.0),
                    ],
                )
            )
        )
    assert not decisions[0].allow_takeover
    assert not decisions[1].allow_takeover
    assert decisions[2].allow_takeover


def test_selector_stability_tracks_consecutive_measurements_not_offset_bucket():
    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=2))
    first = selector.update(
        _payload(
            1.0,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                _candidate(1, 1.0, nvtl=3.0, offset_along=1.0),
            ],
        )
    )
    second = selector.update(
        _payload(
            2.0,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                _candidate(2, 2.0, nvtl=3.0, offset_along=-1.0),
            ],
        )
    )
    assert first.selected is not None
    assert second.selected is not None
    assert second.stable
    assert second.allow_takeover


def test_takeover_gate_rejects_healthy_base():
    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=1))
    decision = selector.update(
        _payload(
            1.0,
            [
                _candidate(0, 0.0, nvtl=3.2, converged=True),
                _candidate(1, 1.0, nvtl=3.4, offset_along=1.0),
            ],
        )
    )
    assert decision.selected is not None
    assert not decision.allow_takeover
    assert decision.rejected_takeover_reason == "base_not_degraded"


def test_independent_observer_can_allow_index0_takeover_when_configured():
    selector = CausalFixedLagSelector(
        SelectorConfig(stable_required_frames=1, allow_index0_takeover=True)
    )
    payload = _payload(1.0, [_candidate(0, 0.0, nvtl=3.0, converged=True)])
    payload["main_ndt_health"] = {
        "available": True,
        "metric_age_sec": 0.01,
        "pose_age_sec": 2.0,
        "nearest_voxel_transformation_likelihood": 0.0,
        "initial_to_result_distance_m": 99.0,
    }

    decision = selector.update(payload)

    assert decision.selected is not None
    assert decision.selected.index == 0
    assert decision.allow_takeover
    assert decision.rejected_takeover_reason == ""


def test_main_ndt_health_overrides_observer_base_degraded_for_takeover():
    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=3))
    decisions = []
    for stamp in (1.0, 2.0, 3.0):
        payload = _payload(
            stamp,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                _candidate(1, stamp, nvtl=3.0, offset_along=1.0),
            ],
        )
        payload["main_ndt_health"] = {
            "available": True,
            "metric_age_sec": 0.01,
            "pose_age_sec": 0.01,
            "nearest_voxel_transformation_likelihood": 1.6,
            "initial_to_result_distance_m": 0.2,
        }
        decisions.append(selector.update(payload))

    assert decisions[-1].stable
    assert decisions[-1].base_degraded is False
    assert decisions[-1].main_ndt_degraded is False
    assert not decisions[-1].allow_takeover
    assert decisions[-1].rejected_takeover_reason == "main_ndt_not_degraded"


def test_main_ndt_degraded_allows_stable_nonbase_takeover():
    selector = CausalFixedLagSelector(SelectorConfig(stable_required_frames=2))
    decisions = []
    for stamp in (1.0, 2.0):
        payload = _payload(
            stamp,
            [
                _candidate(0, 0.0, nvtl=3.0, converged=True),
                _candidate(1, stamp, nvtl=3.2, offset_along=1.0),
            ],
        )
        payload["main_ndt_health"] = {
            "available": True,
            "metric_age_sec": 0.01,
            "pose_age_sec": 0.01,
            "nearest_voxel_transformation_likelihood": 1.5,
            "initial_to_result_distance_m": 1.2,
        }
        decisions.append(selector.update(payload))

    assert decisions[-1].stable
    assert decisions[-1].base_degraded is True
    assert decisions[-1].main_ndt_degraded is True
    assert decisions[-1].main_ndt_health_reason == "main_ndt_i2r_large"
    assert decisions[-1].allow_takeover


def test_candidate_rejects_max_iteration_and_large_gnss_distance():
    max_iter = parse_observer_payload(_payload(1.0, [_candidate(1, 1.0)]))[0]
    max_iter = max_iter.__class__(**{**max_iter.__dict__, "iteration_count": 40, "max_iterations": 40})
    assert not candidate_is_plausible(max_iter, SelectorConfig())[0]

    gnss_far = parse_observer_payload(
        _payload(1.0, [_candidate(1, 1.0, gnss_dist=30.0)])
    )[0]
    ok, reason = candidate_is_plausible(gnss_far, SelectorConfig(max_gnss_weak_prior_distance_m=15.0))
    assert not ok
    assert reason == "gnss_weak_prior_distance_too_large"


def test_selector_uses_only_current_and_past_motion():
    selector = CausalFixedLagSelector(
        SelectorConfig(stable_required_frames=1, max_twist_residual_m=2.0)
    )
    first = selector.update(
        _payload(
            1.0,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                _candidate(1, 0.0, nvtl=3.0, offset_along=1.0),
            ],
        ),
        vehicle_speed_mps=1.0,
    )
    assert first.selected is not None
    second = selector.update(
        _payload(
            2.0,
            [
                _candidate(0, 0.0, nvtl=0.8, converged=False),
                _candidate(1, 1.0, nvtl=3.0, offset_along=1.0),
            ],
        ),
        vehicle_speed_mps=1.0,
    )
    assert second.selected is not None
    assert math.isclose(second.selected.pose.x, 1.0)
