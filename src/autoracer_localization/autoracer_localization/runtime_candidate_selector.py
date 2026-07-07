"""Causal observer-candidate selector for shadow and gated NDT takeover.

The selector consumes NDT runtime observer JSON and never uses GT, future
frames, GNSS yaw, or nearest-to-GNSS selection.  GNSS5m appears only as a weak
penalty / plausibility gate over LiDAR/NDT candidates.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


@dataclass(frozen=True)
class Pose2D:
    stamp_sec: float
    x: float
    y: float
    yaw: float
    z: float = 0.0


@dataclass(frozen=True)
class ObserverCandidate:
    index: int
    pose: Pose2D
    initial_pose: Pose2D
    offset_along_m: float
    offset_cross_m: float
    offset_yaw_deg: float
    converged: bool
    iteration_count: int
    max_iterations: int
    transform_probability: float
    nearest_voxel_transformation_likelihood: float
    score: float
    total_score: float | None
    initial_to_result_distance_m: float
    initial_to_result_yaw_rad: float
    innovation_along_m: float
    innovation_cross_m: float
    innovation_yaw_rad: float
    localizability_along_variance_m2: float
    localizability_cross_variance_m2: float
    covariance_condition_number: float
    gnss_weak_prior_distance_m: float | None = None
    gnss_weak_prior_penalty: float | None = None
    rejection_reason: str = ""

    @property
    def hit_max_iteration(self) -> bool:
        return self.max_iterations > 0 and self.iteration_count >= self.max_iterations

    @property
    def basin_key(self) -> tuple[int, int, int]:
        return (
            int(round(self.offset_along_m * 2.0)),
            int(round(self.offset_cross_m * 2.0)),
            int(round(self.offset_yaw_deg)),
        )


@dataclass(frozen=True)
class SelectorConfig:
    min_nvtl: float = 1.0
    max_initial_to_result_m: float = 5.0
    max_initial_to_result_yaw_deg: float = 8.0
    max_gnss_weak_prior_distance_m: float = 15.0
    gnss_weak_prior_weight: float = 1.0
    twist_residual_weight: float = 0.6
    max_twist_residual_m: float = 4.0
    yaw_jump_weight: float = 0.2
    max_yaw_jump_deg: float = 8.0
    initial_to_result_weight: float = 0.25
    offset_xy_weight: float = 0.0
    offset_yaw_weight: float = 0.0
    covariance_condition_weight: float = 0.01
    route_nonmonotonic_penalty: float = 5.0
    stable_required_frames: int = 3
    base_degraded_nvtl: float = 1.3
    base_degraded_i2r_m: float = 0.9
    max_main_ndt_health_age_sec: float = 0.30
    candidate_margin_over_base: float = 0.0
    takeover_only_when_degraded: bool = True
    allow_index0_takeover: bool = False


@dataclass(frozen=True)
class SelectorDecision:
    stamp_sec: float
    selected: ObserverCandidate | None
    selected_score: float | None
    stable_count: int
    stable: bool
    base_degraded: bool
    allow_takeover: bool
    reason: str
    rejected_takeover_reason: str = ""
    main_ndt_degraded: bool | None = None
    main_ndt_health_reason: str = ""


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _float(value: Any, default: float = 0.0) -> float:
    parsed = _float_or_none(value)
    return default if parsed is None else parsed


def parse_observer_payload(payload: dict[str, Any]) -> list[ObserverCandidate]:
    stamp_sec = _float(payload.get("stamp_sec"), 0.0)
    parsed: list[ObserverCandidate] = []
    for item in payload.get("candidates") or []:
        try:
            result_yaw = math.radians(_float(item.get("result_yaw_deg"), 0.0))
            initial_yaw = math.radians(_float(item.get("initial_yaw_deg"), 0.0))
            nvtl = _float(item.get("nearest_voxel_transformation_likelihood"), math.nan)
            raw_score = _float(item.get("score"), nvtl)
            total_score = _float_or_none(item.get("total_score"))
            initial_to_result_yaw = math.radians(
                _float(
                    item.get("initial_to_result_yaw_deg", item.get("innovation_yaw_deg")),
                    0.0,
                )
            )
            parsed.append(
                ObserverCandidate(
                    index=int(item.get("index", len(parsed))),
                    pose=Pose2D(
                        stamp_sec=stamp_sec,
                        x=_float(item["result_x"]),
                        y=_float(item["result_y"]),
                        z=_float(item.get("result_z"), 0.0),
                        yaw=result_yaw,
                    ),
                    initial_pose=Pose2D(
                        stamp_sec=stamp_sec,
                        x=_float(item.get("initial_x", item["result_x"])),
                        y=_float(item.get("initial_y", item["result_y"])),
                        z=_float(item.get("initial_z"), 0.0),
                        yaw=initial_yaw,
                    ),
                    offset_along_m=_float(item.get("offset_along_m"), 0.0),
                    offset_cross_m=_float(item.get("offset_cross_m"), 0.0),
                    offset_yaw_deg=_float(item.get("offset_yaw_deg"), 0.0),
                    converged=bool(item.get("converged", False)),
                    iteration_count=int(item.get("iteration_count", item.get("iteration_num", 0))),
                    max_iterations=int(item.get("max_iterations", 0)),
                    transform_probability=_float(item.get("transform_probability"), 0.0),
                    nearest_voxel_transformation_likelihood=nvtl,
                    score=raw_score,
                    total_score=total_score,
                    initial_to_result_distance_m=_float(
                        item.get("initial_to_result_distance_m"), 0.0
                    ),
                    initial_to_result_yaw_rad=initial_to_result_yaw,
                    innovation_along_m=_float(item.get("innovation_along_m"), 0.0),
                    innovation_cross_m=_float(item.get("innovation_cross_m"), 0.0),
                    innovation_yaw_rad=math.radians(_float(item.get("innovation_yaw_deg"), 0.0)),
                    localizability_along_variance_m2=_float(
                        item.get("localizability_along_variance_m2"), 0.0
                    ),
                    localizability_cross_variance_m2=_float(
                        item.get("localizability_cross_variance_m2"), 0.0
                    ),
                    covariance_condition_number=_float(
                        item.get("covariance_condition_number"), 1.0
                    ),
                    gnss_weak_prior_distance_m=_float_or_none(
                        item.get("gnss_weak_prior_distance_m")
                    ),
                    gnss_weak_prior_penalty=_float_or_none(
                        item.get("gnss_weak_prior_penalty")
                    ),
                    rejection_reason=str(
                        item.get("rejection_reason", item.get("reject_reason", "")) or ""
                    ),
                )
            )
        except (KeyError, TypeError, ValueError):
            continue
    return parsed


def candidate_is_plausible(candidate: ObserverCandidate, config: SelectorConfig) -> tuple[bool, str]:
    if not candidate.converged:
        return False, "not_converged"
    if candidate.hit_max_iteration:
        return False, "hit_max_iteration"
    if candidate.rejection_reason:
        return False, candidate.rejection_reason
    if candidate.nearest_voxel_transformation_likelihood < config.min_nvtl:
        return False, "nvtl_below_threshold"
    if candidate.initial_to_result_distance_m > config.max_initial_to_result_m:
        return False, "initial_to_result_too_large"
    if abs(math.degrees(candidate.initial_to_result_yaw_rad)) > config.max_initial_to_result_yaw_deg:
        return False, "initial_to_result_yaw_too_large"
    if (
        candidate.gnss_weak_prior_distance_m is not None
        and config.max_gnss_weak_prior_distance_m > 0.0
        and candidate.gnss_weak_prior_distance_m > config.max_gnss_weak_prior_distance_m
    ):
        return False, "gnss_weak_prior_distance_too_large"
    return True, ""


def base_candidate_is_degraded(
    base: ObserverCandidate | None,
    config: SelectorConfig,
) -> bool:
    if base is None:
        return True
    if not base.converged or base.rejection_reason or base.hit_max_iteration:
        return True
    if base.nearest_voxel_transformation_likelihood < config.base_degraded_nvtl:
        return True
    return base.initial_to_result_distance_m > config.base_degraded_i2r_m


def main_ndt_health_is_degraded(
    payload: dict[str, Any],
    config: SelectorConfig,
) -> tuple[bool | None, str]:
    health = payload.get("main_ndt_health")
    if not isinstance(health, dict):
        return None, ""
    if not bool(health.get("available", True)):
        return True, str(health.get("reason") or "main_ndt_health_unavailable")

    metric_age = _float_or_none(health.get("metric_age_sec"))
    pose_age = _float_or_none(health.get("pose_age_sec"))
    max_age = max(0.0, config.max_main_ndt_health_age_sec)
    if metric_age is None or metric_age > max_age:
        return True, "main_ndt_metric_stale"
    if pose_age is None or pose_age > max_age:
        return True, "main_ndt_pose_stale"

    nvtl = _float_or_none(health.get("nearest_voxel_transformation_likelihood"))
    if nvtl is None or nvtl < config.base_degraded_nvtl:
        return True, "main_ndt_nvtl_low"
    i2r = _float_or_none(health.get("initial_to_result_distance_m"))
    if i2r is None or i2r > config.base_degraded_i2r_m:
        return True, "main_ndt_i2r_large"
    return False, "main_ndt_healthy"


class CausalFixedLagSelector:
    def __init__(self, config: SelectorConfig | None = None) -> None:
        self.config = config or SelectorConfig()
        self._stable_key: tuple[int, int, int] | None = None
        self._stable_count = 0
        self._last_pose: Pose2D | None = None
        self._last_progress_m: float | None = None

    def update(
        self,
        payload: dict[str, Any],
        *,
        vehicle_speed_mps: float | None = None,
    ) -> SelectorDecision:
        candidates = parse_observer_payload(payload)
        stamp_sec = _float(payload.get("stamp_sec"), 0.0)
        base = next((item for item in candidates if item.index == 0), None)
        observer_base_degraded = base_candidate_is_degraded(base, self.config)
        main_ndt_degraded, main_ndt_health_reason = main_ndt_health_is_degraded(
            payload, self.config
        )
        base_degraded = (
            observer_base_degraded if main_ndt_degraded is None else main_ndt_degraded
        )
        scored: list[tuple[float, ObserverCandidate]] = []
        rejection_reasons: list[str] = []
        for candidate in candidates:
            plausible, reason = candidate_is_plausible(candidate, self.config)
            if not plausible:
                rejection_reasons.append(reason)
                continue
            twist_residual = self._twist_residual(candidate, vehicle_speed_mps)
            if twist_residual is not None and twist_residual > self.config.max_twist_residual_m:
                rejection_reasons.append("twist_residual_too_large")
                continue
            scored.append((self._score_candidate(candidate, twist_residual), candidate))
        if not scored:
            self._stable_key = None
            self._stable_count = 0
            reason = rejection_reasons[0] if rejection_reasons else "no_candidates"
            return SelectorDecision(
                stamp_sec,
                None,
                None,
                0,
                False,
                base_degraded,
                False,
                reason,
                reason,
                main_ndt_degraded,
                main_ndt_health_reason,
            )

        scored.sort(key=lambda item: item[0], reverse=True)
        best_score, selected = scored[0]
        base_score = next((score for score, candidate in scored if candidate.index == 0), None)
        if (
            selected.index != 0
            and base_score is not None
            and best_score < base_score + self.config.candidate_margin_over_base
        ):
            selected = next(candidate for score, candidate in scored if candidate.index == 0)
            best_score = base_score

        # Stability must be based on consecutive plausible measurements, not
        # on the seed-offset bucket. The same physical basin can be reached
        # from different multistart offsets as the prior moves, especially in
        # hairpins. Requiring a constant offset bucket made the selector unable
        # to take over exactly when the base NDT stream had gaps.
        self._stable_key = selected.basin_key
        self._stable_count += 1
        stable = self._stable_count >= max(1, int(self.config.stable_required_frames))
        rejected_takeover_reason = ""
        allow_takeover = stable and (selected.index != 0 or self.config.allow_index0_takeover)
        if self.config.takeover_only_when_degraded and not base_degraded:
            allow_takeover = False
            rejected_takeover_reason = (
                "main_ndt_not_degraded"
                if main_ndt_degraded is False
                else "base_not_degraded"
            )
        elif not stable:
            rejected_takeover_reason = "selector_not_stable"
        elif selected.index == 0 and not self.config.allow_index0_takeover:
            rejected_takeover_reason = "base_candidate_selected"

        self._last_pose = selected.pose
        progress = payload.get("route_progress_m")
        if progress is not None:
            parsed_progress = _float_or_none(progress)
            if parsed_progress is not None:
                self._last_progress_m = parsed_progress
        return SelectorDecision(
            stamp_sec=stamp_sec,
            selected=selected,
            selected_score=best_score,
            stable_count=self._stable_count,
            stable=stable,
            base_degraded=base_degraded,
            allow_takeover=allow_takeover,
            reason="selected",
            rejected_takeover_reason=rejected_takeover_reason,
            main_ndt_degraded=main_ndt_degraded,
            main_ndt_health_reason=main_ndt_health_reason,
        )

    def _twist_residual(
        self,
        candidate: ObserverCandidate,
        vehicle_speed_mps: float | None,
    ) -> float | None:
        if self._last_pose is None or vehicle_speed_mps is None:
            return None
        dt_sec = candidate.pose.stamp_sec - self._last_pose.stamp_sec
        if not math.isfinite(dt_sec) or dt_sec <= 0.0 or dt_sec > 2.0:
            return None
        predicted_x = self._last_pose.x + math.cos(self._last_pose.yaw) * vehicle_speed_mps * dt_sec
        predicted_y = self._last_pose.y + math.sin(self._last_pose.yaw) * vehicle_speed_mps * dt_sec
        return math.hypot(candidate.pose.x - predicted_x, candidate.pose.y - predicted_y)

    def _score_candidate(
        self,
        candidate: ObserverCandidate,
        twist_residual: float | None,
    ) -> float:
        score = (
            candidate.nearest_voxel_transformation_likelihood
            + 0.05 * candidate.transform_probability
            - self.config.initial_to_result_weight * candidate.initial_to_result_distance_m
            - self.config.offset_xy_weight
            * math.hypot(candidate.offset_along_m, candidate.offset_cross_m)
            - self.config.offset_yaw_weight * abs(candidate.offset_yaw_deg)
            - self.config.yaw_jump_weight * abs(math.degrees(candidate.innovation_yaw_rad))
            - self.config.covariance_condition_weight
            * math.log1p(max(0.0, candidate.covariance_condition_number))
        )
        if candidate.gnss_weak_prior_penalty is not None:
            score -= self.config.gnss_weak_prior_weight * max(
                0.0,
                candidate.gnss_weak_prior_penalty,
            )
        if twist_residual is not None:
            score -= self.config.twist_residual_weight * twist_residual
        return score


def selector_decision_to_diagnostics(decision: SelectorDecision) -> dict[str, Any]:
    selected = decision.selected
    return {
        "stamp_sec": decision.stamp_sec,
        "has_selected_candidate": selected is not None,
        "selected_candidate_index": None if selected is None else selected.index,
        "stable_count": decision.stable_count,
        "stable": decision.stable,
        "base_degraded": decision.base_degraded,
        "main_ndt_degraded": decision.main_ndt_degraded,
        "main_ndt_health_reason": decision.main_ndt_health_reason,
        "allow_takeover": decision.allow_takeover,
        "reason": decision.reason,
        "rejected_takeover_reason": decision.rejected_takeover_reason,
        "selected_score": decision.selected_score,
        "gnss_weak_prior_distance_m": None
        if selected is None
        else selected.gnss_weak_prior_distance_m,
    }


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from autoware_vehicle_msgs.msg import VelocityReport
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    class RuntimeCandidateSelectorNode(Node):
        def __init__(self) -> None:
            super().__init__("runtime_candidate_selector")
            self._frame_id = str(self.declare_parameter("frame_id", "map").value)
            self._enable_shadow = bool(self.declare_parameter("enable_shadow", True).value)
            self._enable_gated_takeover = bool(
                self.declare_parameter("enable_gated_takeover", False).value
            )
            self._max_selector_age_sec = float(
                self.declare_parameter("max_selector_age_sec", 0.25).value
            )
            self._selector = CausalFixedLagSelector(
                SelectorConfig(
                    stable_required_frames=int(
                        self.declare_parameter("stable_required_frames", 3).value
                    ),
                    max_gnss_weak_prior_distance_m=float(
                        self.declare_parameter("max_gnss_weak_prior_distance_m", 15.0).value
                    ),
                    max_twist_residual_m=float(
                        self.declare_parameter("max_twist_residual_m", 4.0).value
                    ),
                    allow_index0_takeover=bool(
                        self.declare_parameter("allow_index0_takeover", False).value
                    ),
                    offset_xy_weight=float(
                        self.declare_parameter("offset_xy_weight", 0.0).value
                    ),
                    offset_yaw_weight=float(
                        self.declare_parameter("offset_yaw_weight", 0.0).value
                    ),
                )
            )
            observer_topic = str(
                self.declare_parameter(
                    "observer_topic", "/localization/ndt/runtime_multistart/observer"
                ).value
            )
            baseline_pose_topic = str(
                self.declare_parameter(
                    "baseline_pose_topic", "/localization/pose_estimator/pose_with_covariance"
                ).value
            )
            velocity_topic = str(
                self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status").value
            )
            shadow_pose_topic = str(
                self.declare_parameter(
                    "shadow_pose_topic", "/localization/selector_shadow/pose_with_covariance"
                ).value
            )
            shadow_diag_topic = str(
                self.declare_parameter(
                    "shadow_diagnostics_topic", "/localization/selector_shadow/diagnostics"
                ).value
            )
            gated_output_topic = str(
                self.declare_parameter(
                    "gated_output_topic", "/localization/selector_gated/pose_with_covariance"
                ).value
            )
            gated_diag_topic = str(
                self.declare_parameter(
                    "gated_diagnostics_topic", "/localization/selector_gated/diagnostics"
                ).value
            )
            self._shadow_pub = self.create_publisher(
                PoseWithCovarianceStamped, shadow_pose_topic, 10
            )
            self._shadow_diag_pub = self.create_publisher(String, shadow_diag_topic, 10)
            self._gated_pub = self.create_publisher(
                PoseWithCovarianceStamped, gated_output_topic, 10
            )
            self._gated_diag_pub = self.create_publisher(String, gated_diag_topic, 10)
            self._last_decision: SelectorDecision | None = None
            self._vehicle_speed_mps: float | None = None
            self._takeover_count = 0
            self.create_subscription(String, observer_topic, self._on_observer, 10)
            self.create_subscription(
                PoseWithCovarianceStamped, baseline_pose_topic, self._on_baseline_pose, 10
            )
            self.create_subscription(VelocityReport, velocity_topic, self._on_velocity, 10)

        def _on_velocity(self, msg: VelocityReport) -> None:
            self._vehicle_speed_mps = float(msg.longitudinal_velocity)

        def _on_observer(self, msg: String) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            decision = self._selector.update(
                payload,
                vehicle_speed_mps=self._vehicle_speed_mps,
            )
            self._last_decision = decision
            diagnostics = selector_decision_to_diagnostics(decision)
            diagnostics["takeover_count"] = self._takeover_count
            diag_msg = String()
            diag_msg.data = json.dumps(diagnostics, sort_keys=True)
            self._shadow_diag_pub.publish(diag_msg)
            if self._enable_shadow and decision.selected is not None:
                self._shadow_pub.publish(self._pose_msg_from_candidate(decision.selected))
            if self._enable_gated_takeover and decision.allow_takeover and decision.selected is not None:
                self._gated_pub.publish(self._pose_msg_from_candidate(decision.selected))
                self._takeover_count += 1
                diagnostics["takeover"] = True
                diagnostics["takeover_count"] = self._takeover_count
                diag_msg = String()
                diag_msg.data = json.dumps(diagnostics, sort_keys=True)
                self._gated_diag_pub.publish(diag_msg)

        def _on_baseline_pose(self, msg: PoseWithCovarianceStamped) -> None:
            if not self._enable_gated_takeover:
                return
            output = msg
            decision = self._last_decision
            diagnostics = {"stamp_sec": self._stamp_to_sec(msg.header.stamp), "takeover": False}
            if decision is not None:
                age = abs(self._stamp_to_sec(msg.header.stamp) - decision.stamp_sec)
                diagnostics.update(selector_decision_to_diagnostics(decision))
                diagnostics["selector_age_sec"] = age
                if age <= self._max_selector_age_sec and decision.allow_takeover and decision.selected:
                    output = self._pose_msg_from_candidate(decision.selected)
                    output.header.stamp = msg.header.stamp
                    self._takeover_count += 1
                    diagnostics["takeover"] = True
                else:
                    diagnostics["takeover"] = False
            diagnostics["takeover_count"] = self._takeover_count
            self._gated_pub.publish(output)
            diag_msg = String()
            diag_msg.data = json.dumps(diagnostics, sort_keys=True)
            self._gated_diag_pub.publish(diag_msg)

        def _pose_msg_from_candidate(
            self,
            candidate: ObserverCandidate,
        ) -> PoseWithCovarianceStamped:
            msg = PoseWithCovarianceStamped()
            stamp = self.get_clock().now().to_msg()
            if candidate.pose.stamp_sec > 0.0:
                sec = int(candidate.pose.stamp_sec)
                nanosec = int(round((candidate.pose.stamp_sec - sec) * 1e9))
                stamp.sec = sec
                stamp.nanosec = nanosec
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.pose.pose.position.x = candidate.pose.x
            msg.pose.pose.position.y = candidate.pose.y
            msg.pose.pose.position.z = candidate.pose.z
            qz = math.sin(candidate.pose.yaw * 0.5)
            qw = math.cos(candidate.pose.yaw * 0.5)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            xy_var = max(0.0225, candidate.localizability_along_variance_m2, 0.05)
            msg.pose.covariance[0] = xy_var
            msg.pose.covariance[7] = max(0.0225, candidate.localizability_cross_variance_m2, 0.05)
            msg.pose.covariance[35] = 0.01
            return msg

        @staticmethod
        def _stamp_to_sec(stamp: Any) -> float:
            return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    rclpy.init()
    node = RuntimeCandidateSelectorNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
