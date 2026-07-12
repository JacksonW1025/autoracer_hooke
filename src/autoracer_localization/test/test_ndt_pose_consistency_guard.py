import math
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from autoracer_localization.ndt_pose_consistency_guard import (
    Pose2D,
    PoseConsistencyGuardCore,
    NdtPoseConsistencyGuard,
    _correction_between,
    _yaw_from_quaternion,
    predicted_pose_message,
)


def _core() -> PoseConsistencyGuardCore:
    return PoseConsistencyGuardCore(
        anchor_interval_sec=5.0,
        anchor_history_sec=20.0,
        max_xy_innovation_m=0.2,
        max_yaw_innovation_rad=math.radians(0.15),
        gross_xy_innovation_m=0.6,
        gross_yaw_innovation_rad=math.radians(2.0),
        yaw_rate_allowance_sec=0.06,
        violation_hold_sec=0.2,
        recovery_xy_innovation_m=0.12,
        recovery_yaw_innovation_rad=math.radians(0.2),
        recovery_hold_sec=5.0,
        recovery_consistency_window_sec=5.0,
        recovery_consistency_min_samples=30,
        recovery_consistency_max_xy_spread_m=0.15,
        recovery_consistency_max_yaw_spread_rad=math.radians(0.5),
        max_twist_age_sec=0.25,
    )


def test_persistent_joint_ndt_drift_enters_degraded_state() -> None:
    core = _core()
    assert not core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0)).degraded

    decision = None
    for index in range(1, 66):
        stamp = index * 0.1
        core.update_twist(stamp, 1.0, 0.0)
        drift = max(0.0, stamp - 5.0) * 0.25
        yaw_drift = math.radians(max(0.0, stamp - 5.0) * 0.3)
        decision = core.evaluate_pose(stamp, Pose2D(stamp, drift, yaw_drift))

    assert decision is not None
    assert decision.degraded
    assert decision.xy_innovation_m > 0.2


def test_guard_recovers_only_after_sustained_consistency() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    recovery_reason = None
    for index in range(8, 61):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.0, 0.0))
        if decision.state_changed:
            recovery_reason = decision.reason
    assert not decision.degraded
    assert recovery_reason == "motion_consistency_recovered"


def test_stale_twist_fails_open_instead_of_rejecting_ndt() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    decision = core.evaluate_pose(1.0, Pose2D(10.0, 10.0, 1.0))
    assert not decision.degraded
    assert decision.reason == "twist_unavailable"


def test_small_translation_only_innovation_does_not_reject_ndt() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 10):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.3, 0.0, 0.0))

    assert not decision.degraded
    assert decision.reason == "trusted_anchor_refresh_withheld"
    assert core.anchor_stamp_sec == 0.0


def test_withheld_translation_anchor_allows_later_joint_drift_detection() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))

    for index in range(1, 61):
        stamp = index * 0.1
        core.update_twist(stamp, 1.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(stamp, 0.3, 0.0))

    assert not decision.degraded
    assert core.anchor_stamp_sec == 0.0

    transition_reason = None
    for index in range(61, 66):
        stamp = index * 0.1
        core.update_twist(stamp, 1.0, 0.0)
        decision = core.evaluate_pose(
            stamp,
            Pose2D(stamp, 0.3, math.radians(0.3)),
        )
        if decision.state_changed:
            transition_reason = decision.reason

    assert decision.degraded
    assert transition_reason == "persistent_motion_inconsistency"


def test_prediction_uses_robust_median_of_recent_trusted_anchors() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))

    for stamp, x in ((5.0, 0.02), (10.0, -0.02), (15.0, 0.15)):
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(x, 0.0, 0.0))
        assert not decision.degraded

    assert len(core.prediction_tracks) == 4
    assert core.predicted_pose is not None
    assert math.isclose(core.predicted_pose.x, 0.01, abs_tol=1e-9)


def test_gross_translation_innovation_rejects_without_yaw_error() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 10):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(1.0, 0.0, 0.0))

    assert decision.degraded


def test_turn_rate_allowance_avoids_model_error_false_positive() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    yaw_error = math.radians(0.2)
    for index in range(1, 10):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.05)
        decision = core.evaluate_pose(
            stamp,
            Pose2D(0.3, 0.0, 0.05 * stamp + yaw_error),
        )

    assert not decision.degraded


def test_fallback_pose_replaces_only_planar_prediction() -> None:
    template = PoseWithCovarianceStamped()
    template.pose.pose.position.x = 10.0
    template.pose.pose.position.y = 20.0
    template.pose.pose.position.z = 3.0
    template.pose.pose.orientation.w = 1.0

    output = predicted_pose_message(template, Pose2D(1.0, 2.0, 0.3))

    assert output.pose.pose.position.x == 1.0
    assert output.pose.pose.position.y == 2.0
    assert output.pose.pose.position.z == 3.0
    assert math.isclose(_yaw_from_quaternion(output.pose.pose.orientation), 0.3)
    assert template.pose.pose.position.x == 10.0


def test_pose_correction_maps_the_request_pose_and_preserves_later_motion() -> None:
    source = Pose2D(10.0, -3.0, 0.2)
    target = Pose2D(10.3, -3.1, 0.21)
    correction = _correction_between(source, target)

    corrected_source = correction.apply(source)
    assert math.isclose(corrected_source.x, target.x, abs_tol=1e-9)
    assert math.isclose(corrected_source.y, target.y, abs_tol=1e-9)
    assert math.isclose(corrected_source.yaw, target.yaw, abs_tol=1e-9)

    later_source = Pose2D(15.0, -2.0, 0.25)
    later_target = correction.apply(later_source)
    assert math.hypot(later_target.x - later_source.x, later_target.y - later_source.y) < 0.5


def test_explicit_relocalization_resets_degraded_prediction() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    relocalized = Pose2D(0.25, 0.05, math.radians(0.1))
    assert core.accept_relocalization(1.0, relocalized)
    assert not core.degraded
    assert core.predicted_pose == relocalized
    assert len(core.prediction_tracks) == 1


def test_bounded_window_recovers_without_allowing_a_large_pose_jump() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    recovery_reason = None
    for index in range(8, 61):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        offset = 0.08 if index % 2 else 0.16
        decision = core.evaluate_pose(stamp, Pose2D(offset, 0.0, 0.0))
        if decision.state_changed and not decision.degraded:
            recovery_reason = decision.reason
            break

    assert not decision.degraded
    assert recovery_reason == "bounded_window_recovered"
    assert decision.xy_innovation_m <= core.recovery_xy_innovation_m
    assert core.predicted_pose is not None
    assert core.predicted_pose.x == 0.08


def test_unstable_recovery_candidate_does_not_rebase() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    for index in range(8, 70):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        offset = 0.2 if index % 2 else 0.6
        decision = core.evaluate_pose(stamp, Pose2D(offset, 0.0, 0.0))

    assert decision.degraded


def test_large_stable_offset_cannot_rebase_the_prediction() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    for index in range(8, 80):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.4, 0.0, 0.0))

    assert decision.degraded


def test_short_false_consistency_does_not_recover() -> None:
    core = _core()
    core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert decision.degraded

    for index in range(8, 53):
        stamp = index * 0.1
        core.update_twist(stamp, 0.0, 0.0)
        decision = core.evaluate_pose(stamp, Pose2D(0.05, 0.0, 0.0))
    assert decision.degraded

    stamp = 5.3
    core.update_twist(stamp, 0.0, 0.0)
    decision = core.evaluate_pose(stamp, Pose2D(0.4, 0.0, 0.0))
    assert decision.degraded


class _CompletedFuture:
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response


def _degrade_node_core(node: NdtPoseConsistencyGuard) -> None:
    node._core.evaluate_pose(0.0, Pose2D(0.0, 0.0, 0.0))
    for index in range(1, 8):
        stamp = index * 0.1
        node._core.update_twist(stamp, 0.0, 0.0)
        node._core.evaluate_pose(stamp, Pose2D(0.0, 0.3, math.radians(0.3)))
    assert node._core.degraded


def test_ndt_align_result_is_propagated_then_raw_confirmed() -> None:
    rclpy.init()
    node = NdtPoseConsistencyGuard()
    try:
        _degrade_node_core(node)
        request_prediction = node._core.predicted_pose
        assert request_prediction is not None

        result = PoseWithCovarianceStamped()
        result.pose.pose.position.x = request_prediction.x + 0.3
        result.pose.pose.position.y = request_prediction.y - 0.1
        result.pose.pose.orientation.w = 1.0
        response = SimpleNamespace(
            success=True,
            reliable=True,
            pose_with_covariance=result,
        )
        node._relocalization_generation = 1
        node._relocalization_future = object()
        node._relocalization_request_prediction = request_prediction
        node._relocalization_request_stamp_sec = 0.7
        node._core.latest_twist_stamp_sec = 0.7
        node._on_relocalization_response(_CompletedFuture(response), 1)
        assert node._relocalization_correction is not None

        node._core.update_twist(1.0, 1.0, 0.0)
        candidate = node._relocalization_correction.apply(node._core.predicted_pose)
        assert not node._try_confirm_relocalization(1.0, candidate)
        node._core.update_twist(1.6, 1.0, 0.0)
        candidate = node._relocalization_correction.apply(node._core.predicted_pose)
        assert node._try_confirm_relocalization(1.6, candidate)
        assert not node._core.degraded
        assert node._core.predicted_pose == candidate
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ndt_align_rejects_correction_outside_the_safety_bound() -> None:
    rclpy.init()
    node = NdtPoseConsistencyGuard()
    try:
        _degrade_node_core(node)
        request_prediction = node._core.predicted_pose
        assert request_prediction is not None

        result = PoseWithCovarianceStamped()
        result.pose.pose.position.x = request_prediction.x + 1.0
        result.pose.pose.position.y = request_prediction.y
        result.pose.pose.orientation.w = 1.0
        response = SimpleNamespace(
            success=True,
            reliable=True,
            pose_with_covariance=result,
        )
        node._relocalization_generation = 1
        node._relocalization_future = object()
        node._relocalization_request_prediction = request_prediction
        node._relocalization_request_stamp_sec = 0.7
        node._on_relocalization_response(_CompletedFuture(response), 1)
        assert node._relocalization_correction is None
        assert node._core.degraded
    finally:
        node.destroy_node()
        rclpy.shutdown()
