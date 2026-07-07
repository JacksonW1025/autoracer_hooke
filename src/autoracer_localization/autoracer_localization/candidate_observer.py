"""Independent, non-invasive runtime candidate observer.

This node publishes causal candidate JSON for selector/shadow experiments
without calling into the live NDT matcher.  The base candidate is a snapshot of
the already-published NDT pose plus NDT debug metrics when available.  Additional
bounded-search hypotheses are explicitly marked unaligned and rejected, so they
cannot be used for takeover until an independent LiDAR alignment backend is
attached.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Iterable


@dataclass(frozen=True)
class Pose2D:
    stamp_sec: float
    x: float
    y: float
    z: float
    yaw: float
    covariance: tuple[float, ...] = ()


@dataclass(frozen=True)
class GnssWeakPrior:
    pose: Pose2D | None
    age_sec: float | None
    sigma_m: float
    max_penalty: float


@dataclass(frozen=True)
class NdtDebugMetrics:
    transform_probability: float | None = None
    nearest_voxel_transformation_likelihood: float | None = None
    iteration_count: int | None = None
    initial_to_result_distance_m: float | None = None
    metric_age_sec: float | None = None


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def offset_pose(seed: Pose2D, *, along_m: float, cross_m: float, yaw_deg: float) -> Pose2D:
    yaw = normalize_angle(seed.yaw + math.radians(yaw_deg))
    cos_yaw = math.cos(seed.yaw)
    sin_yaw = math.sin(seed.yaw)
    return Pose2D(
        stamp_sec=seed.stamp_sec,
        x=seed.x + cos_yaw * along_m - sin_yaw * cross_m,
        y=seed.y + sin_yaw * along_m + cos_yaw * cross_m,
        z=seed.z,
        yaw=yaw,
        covariance=seed.covariance,
    )


def covariance_summary(covariance: Iterable[float]) -> tuple[float, float, float]:
    values = list(covariance)
    along = float(values[0]) if len(values) > 0 and math.isfinite(float(values[0])) else 0.0
    cross = float(values[7]) if len(values) > 7 and math.isfinite(float(values[7])) else 0.0
    positive = [value for value in (along, cross) if value > 1.0e-9]
    if len(positive) < 2:
        return along, cross, 1.0
    return along, cross, max(positive) / max(min(positive), 1.0e-9)


def gnss_distance_and_penalty(
    pose: Pose2D,
    weak_prior: GnssWeakPrior | None,
    *,
    max_age_sec: float,
) -> tuple[float | None, float | None, str, float | None]:
    if weak_prior is None or weak_prior.pose is None:
        return None, None, "no_gnss_weak_prior", None
    age_sec = weak_prior.age_sec
    if age_sec is None or not math.isfinite(age_sec) or age_sec > max_age_sec:
        return None, None, "gnss_weak_prior_stale", age_sec
    distance = math.hypot(pose.x - weak_prior.pose.x, pose.y - weak_prior.pose.y)
    sigma = max(0.1, weak_prior.sigma_m)
    penalty = min(max(0.0, weak_prior.max_penalty), (distance * distance) / (2.0 * sigma * sigma))
    return distance, penalty, "weak_penalty_only", age_sec


def parse_offset_list(value: str, default: tuple[float, ...]) -> tuple[float, ...]:
    parsed: list[float] = []
    for token in str(value or "").split(","):
        token = token.strip()
        if not token:
            continue
        try:
            parsed.append(float(token))
        except ValueError:
            continue
    return tuple(parsed) if parsed else default


def build_candidate_payload(
    *,
    base_pose: Pose2D,
    metrics: NdtDebugMetrics,
    weak_prior: GnssWeakPrior | None,
    offset_along_m: tuple[float, ...],
    offset_cross_m: tuple[float, ...],
    offset_yaw_deg: tuple[float, ...],
    gnss_max_age_sec: float,
    max_iterations: int,
    default_transform_probability: float,
    default_nearest_voxel_transformation_likelihood: float,
    include_unaligned_hypotheses: bool = True,
) -> dict[str, Any]:
    tp = (
        float(metrics.transform_probability)
        if metrics.transform_probability is not None
        else float(default_transform_probability)
    )
    nvtl = (
        float(metrics.nearest_voxel_transformation_likelihood)
        if metrics.nearest_voxel_transformation_likelihood is not None
        else float(default_nearest_voxel_transformation_likelihood)
    )
    iteration_count = int(metrics.iteration_count) if metrics.iteration_count is not None else 0
    initial_to_result = (
        float(metrics.initial_to_result_distance_m)
        if metrics.initial_to_result_distance_m is not None
        else 0.0
    )
    along_var, cross_var, covariance_condition = covariance_summary(base_pose.covariance)
    base_gnss_dist, base_gnss_penalty, base_gnss_reason, base_gnss_age = gnss_distance_and_penalty(
        base_pose,
        weak_prior,
        max_age_sec=gnss_max_age_sec,
    )
    metric_source = "ndt_debug_topics" if metrics.metric_age_sec is not None else "configured_defaults"
    candidates: list[dict[str, Any]] = [
        {
            "index": 0,
            "source": "base_ndt_pose_snapshot",
            "initial_x": base_pose.x,
            "initial_y": base_pose.y,
            "initial_z": base_pose.z,
            "initial_yaw_deg": math.degrees(base_pose.yaw),
            "result_x": base_pose.x,
            "result_y": base_pose.y,
            "result_z": base_pose.z,
            "result_yaw_deg": math.degrees(base_pose.yaw),
            "offset_along_m": 0.0,
            "offset_cross_m": 0.0,
            "offset_yaw_deg": 0.0,
            "converged": True,
            "iteration_count": iteration_count,
            "iteration_num": iteration_count,
            "max_iterations": int(max_iterations),
            "hit_max_iteration": bool(max_iterations > 0 and iteration_count >= int(max_iterations)),
            "transform_probability": tp,
            "nearest_voxel_transformation_likelihood": nvtl,
            "score": nvtl,
            "total_score": nvtl,
            "initial_to_result_distance_m": initial_to_result,
            "initial_to_result_yaw_deg": 0.0,
            "innovation_along_m": 0.0,
            "innovation_cross_m": 0.0,
            "innovation_yaw_deg": 0.0,
            "localizability_along_variance_m2": along_var,
            "localizability_cross_variance_m2": cross_var,
            "covariance_condition_number": covariance_condition,
            "gnss_weak_prior_distance_m": base_gnss_dist,
            "gnss_weak_prior_penalty": base_gnss_penalty,
            "gnss_weak_prior_age_sec": base_gnss_age,
            "gnss_weak_prior_gate_reason": base_gnss_reason,
            "rejection_reason": "",
            "reject_reason": "",
            "selected_by_observer": True,
            "metric_source": metric_source,
            "metric_age_sec": metrics.metric_age_sec,
        }
    ]

    if include_unaligned_hypotheses:
        index = 1
        for along in offset_along_m:
            for cross in offset_cross_m:
                for yaw_deg in offset_yaw_deg:
                    if along == 0.0 and cross == 0.0 and yaw_deg == 0.0:
                        continue
                    pose = offset_pose(base_pose, along_m=along, cross_m=cross, yaw_deg=yaw_deg)
                    gnss_dist, gnss_penalty, gnss_reason, gnss_age = gnss_distance_and_penalty(
                        pose,
                        weak_prior,
                        max_age_sec=gnss_max_age_sec,
                    )
                    candidates.append(
                        {
                            "index": index,
                            "source": "bounded_search_seed_unaligned",
                            "initial_x": pose.x,
                            "initial_y": pose.y,
                            "initial_z": pose.z,
                            "initial_yaw_deg": math.degrees(pose.yaw),
                            "result_x": pose.x,
                            "result_y": pose.y,
                            "result_z": pose.z,
                            "result_yaw_deg": math.degrees(pose.yaw),
                            "offset_along_m": along,
                            "offset_cross_m": cross,
                            "offset_yaw_deg": yaw_deg,
                            "converged": False,
                            "iteration_count": 0,
                            "iteration_num": 0,
                            "max_iterations": int(max_iterations),
                            "hit_max_iteration": False,
                            "transform_probability": 0.0,
                            "nearest_voxel_transformation_likelihood": 0.0,
                            "score": 0.0,
                            "total_score": 0.0,
                            "initial_to_result_distance_m": 0.0,
                            "initial_to_result_yaw_deg": 0.0,
                            "innovation_along_m": along,
                            "innovation_cross_m": cross,
                            "innovation_yaw_deg": yaw_deg,
                            "localizability_along_variance_m2": along_var,
                            "localizability_cross_variance_m2": cross_var,
                            "covariance_condition_number": covariance_condition,
                            "gnss_weak_prior_distance_m": gnss_dist,
                            "gnss_weak_prior_penalty": gnss_penalty,
                            "gnss_weak_prior_age_sec": gnss_age,
                            "gnss_weak_prior_gate_reason": gnss_reason,
                            "rejection_reason": "not_lidar_aligned_independent_observer_hypothesis",
                            "reject_reason": "not_lidar_aligned_independent_observer_hypothesis",
                            "selected_by_observer": False,
                            "metric_source": "not_aligned",
                            "metric_age_sec": None,
                        }
                    )
                    index += 1

    has_gnss = base_gnss_dist is not None
    return {
        "schema_version": 1,
        "stamp_sec": base_pose.stamp_sec,
        "reason": "runtime_candidate_observer",
        "source": "independent_candidate_observer",
        "controls_output": False,
        "controls_final_localization": False,
        "uses_gt": False,
        "uses_future_frames": False,
        "uses_gnss_or_gt": False,
        "uses_gnss_weak_prior": has_gnss,
        "gnss_usage": "weak_penalty_only" if has_gnss else "not_used",
        "candidate_count": len(candidates),
        "has_selected_candidate": True,
        "selected_candidate_index": 0,
        "route_progress_m": None,
        "rejection_reason": "",
        "candidates": candidates,
    }


def main() -> None:
    import rclpy
    from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    from .pose_stream_qos import latest_pose_qos

    class IndependentCandidateObserverNode(Node):
        def __init__(self) -> None:
            super().__init__("independent_candidate_observer")
            self._base_pose_topic = str(
                self.declare_parameter(
                    "base_pose_topic",
                    "/localization/pose_estimator/pose_with_covariance",
                ).value
            )
            self._gnss_topic = str(
                self.declare_parameter(
                    "gnss_weak_prior_topic",
                    "/sensing/gnss/pose_with_covariance",
                ).value
            )
            self._enable_gnss_weak_prior = bool(
                self.declare_parameter("enable_gnss_weak_prior", False).value
            )
            self._enable_ndt_debug_metrics = bool(
                self.declare_parameter("enable_ndt_debug_metrics", False).value
            )
            output_topic = str(
                self.declare_parameter(
                    "output_topic",
                    "/localization/candidate_observer/candidates",
                ).value
            )
            debug_topic = str(
                self.declare_parameter(
                    "debug_topic",
                    "/localization/candidate_observer/diagnostics",
                ).value
            )
            self._max_metric_age_sec = float(
                self.declare_parameter("max_metric_age_sec", 0.25).value
            )
            self._gnss_max_age_sec = float(
                self.declare_parameter("gnss_weak_prior_max_age_sec", 0.5).value
            )
            self._gnss_sigma_m = float(self.declare_parameter("gnss_weak_prior_sigma_m", 5.0).value)
            self._gnss_max_penalty = float(
                self.declare_parameter("gnss_weak_prior_max_penalty", 8.0).value
            )
            self._max_iterations = int(self.declare_parameter("max_iterations", 80).value)
            self._default_tp = float(
                self.declare_parameter("default_transform_probability", 3.0).value
            )
            self._default_nvtl = float(
                self.declare_parameter(
                    "default_nearest_voxel_transformation_likelihood",
                    2.3,
                ).value
            )
            self._include_unaligned = bool(
                self.declare_parameter("include_unaligned_hypotheses", True).value
            )
            self._publish_min_period_sec = max(
                0.0,
                float(self.declare_parameter("publish_min_period_sec", 0.0).value),
            )
            self._offset_along_m = parse_offset_list(
                str(self.declare_parameter("offset_along_m", "-2.0,-1.0,0.0,1.0,2.0").value),
                (-2.0, -1.0, 0.0, 1.0, 2.0),
            )
            self._offset_cross_m = parse_offset_list(
                str(self.declare_parameter("offset_cross_m", "-0.75,0.0,0.75").value),
                (-0.75, 0.0, 0.75),
            )
            self._offset_yaw_deg = parse_offset_list(
                str(self.declare_parameter("offset_yaw_deg", "-2.0,0.0,2.0").value),
                (-2.0, 0.0, 2.0),
            )

            self._latest_gnss: Pose2D | None = None
            self._latest_gnss_stamp_sec: float | None = None
            self._metrics: dict[str, tuple[float, float]] = {}
            self._int_metrics: dict[str, tuple[float, int]] = {}
            self._publish_count = 0
            self._last_publish_stamp_sec: float | None = None

            self._pub = self.create_publisher(String, output_topic, 10)
            self._debug_pub = self.create_publisher(String, debug_topic, 10)
            debug_qos = QoSProfile(depth=1)
            debug_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._base_pose_topic,
                self._on_base_pose,
                latest_pose_qos(),
            )
            if self._enable_gnss_weak_prior:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    self._gnss_topic,
                    self._on_gnss_pose,
                    latest_pose_qos(),
                )
            if self._enable_ndt_debug_metrics:
                self.create_subscription(
                    Float32Stamped,
                    "/transform_probability",
                    lambda msg: self._on_float_metric("transform_probability", msg),
                    debug_qos,
                )
                self.create_subscription(
                    Float32Stamped,
                    "/nearest_voxel_transformation_likelihood",
                    lambda msg: self._on_float_metric(
                        "nearest_voxel_transformation_likelihood",
                        msg,
                    ),
                    debug_qos,
                )
                self.create_subscription(
                    Float32Stamped,
                    "/initial_to_result_distance",
                    lambda msg: self._on_float_metric("initial_to_result_distance", msg),
                    debug_qos,
                )
                self.create_subscription(
                    Int32Stamped,
                    "/iteration_num",
                    lambda msg: self._on_int_metric("iteration_num", msg),
                    debug_qos,
                )

        @staticmethod
        def _stamp_sec(stamp: Any) -> float:
            return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        @staticmethod
        def _pose_from_msg(msg: PoseWithCovarianceStamped) -> Pose2D:
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            return Pose2D(
                stamp_sec=IndependentCandidateObserverNode._stamp_sec(msg.header.stamp),
                x=float(p.x),
                y=float(p.y),
                z=float(p.z),
                yaw=yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
                covariance=tuple(float(value) for value in msg.pose.covariance),
            )

        def _on_gnss_pose(self, msg: PoseWithCovarianceStamped) -> None:
            pose = self._pose_from_msg(msg)
            self._latest_gnss = pose
            self._latest_gnss_stamp_sec = pose.stamp_sec

        def _on_float_metric(self, name: str, msg: Any) -> None:
            self._metrics[name] = (self._stamp_sec(msg.stamp), float(msg.data))

        def _on_int_metric(self, name: str, msg: Any) -> None:
            self._int_metrics[name] = (self._stamp_sec(msg.stamp), int(msg.data))

        def _float_metric(self, name: str, stamp_sec: float) -> tuple[float | None, float | None]:
            sample = self._metrics.get(name)
            if sample is None:
                return None, None
            sample_stamp, value = sample
            age = abs(stamp_sec - sample_stamp)
            if age > self._max_metric_age_sec:
                return None, None
            return value, age

        def _int_metric(self, name: str, stamp_sec: float) -> tuple[int | None, float | None]:
            sample = self._int_metrics.get(name)
            if sample is None:
                return None, None
            sample_stamp, value = sample
            age = abs(stamp_sec - sample_stamp)
            if age > self._max_metric_age_sec:
                return None, None
            return value, age

        def _debug_metrics(self, stamp_sec: float) -> NdtDebugMetrics:
            tp, tp_age = self._float_metric("transform_probability", stamp_sec)
            nvtl, nvtl_age = self._float_metric(
                "nearest_voxel_transformation_likelihood",
                stamp_sec,
            )
            i2r, i2r_age = self._float_metric("initial_to_result_distance", stamp_sec)
            iteration_count, iter_age = self._int_metric("iteration_num", stamp_sec)
            ages = [age for age in (tp_age, nvtl_age, i2r_age, iter_age) if age is not None]
            metric_age = max(ages) if ages else None
            return NdtDebugMetrics(
                transform_probability=tp,
                nearest_voxel_transformation_likelihood=nvtl,
                iteration_count=iteration_count,
                initial_to_result_distance_m=i2r,
                metric_age_sec=metric_age,
            )

        def _weak_prior(self, stamp_sec: float) -> GnssWeakPrior | None:
            if self._latest_gnss is None or self._latest_gnss_stamp_sec is None:
                return None
            return GnssWeakPrior(
                pose=self._latest_gnss,
                age_sec=abs(stamp_sec - self._latest_gnss_stamp_sec),
                sigma_m=self._gnss_sigma_m,
                max_penalty=self._gnss_max_penalty,
            )

        def _on_base_pose(self, msg: PoseWithCovarianceStamped) -> None:
            stamp_sec = self._stamp_sec(msg.header.stamp)
            if (
                self._last_publish_stamp_sec is not None
                and stamp_sec - self._last_publish_stamp_sec
                < self._publish_min_period_sec
            ):
                return
            self._last_publish_stamp_sec = stamp_sec
            base_pose = self._pose_from_msg(msg)
            payload = build_candidate_payload(
                base_pose=base_pose,
                metrics=self._debug_metrics(base_pose.stamp_sec),
                weak_prior=self._weak_prior(base_pose.stamp_sec),
                offset_along_m=self._offset_along_m,
                offset_cross_m=self._offset_cross_m,
                offset_yaw_deg=self._offset_yaw_deg,
                gnss_max_age_sec=self._gnss_max_age_sec,
                max_iterations=self._max_iterations,
                default_transform_probability=self._default_tp,
                default_nearest_voxel_transformation_likelihood=self._default_nvtl,
                include_unaligned_hypotheses=self._include_unaligned,
            )
            out = String()
            out.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self._pub.publish(out)
            self._publish_count += 1

            debug = {
                "stamp_sec": base_pose.stamp_sec,
                "publish_count": self._publish_count,
                "candidate_count": payload["candidate_count"],
                "controls_output": False,
                "controls_final_localization": False,
                "gnss_usage": payload["gnss_usage"],
                "base_metric_source": payload["candidates"][0]["metric_source"],
                "base_metric_age_sec": payload["candidates"][0]["metric_age_sec"],
                "base_gnss_weak_prior_distance_m": payload["candidates"][0][
                    "gnss_weak_prior_distance_m"
                ],
            }
            debug_msg = String()
            debug_msg.data = json.dumps(debug, separators=(",", ":"), sort_keys=True)
            self._debug_pub.publish(debug_msg)

    rclpy.init()
    node = IndependentCandidateObserverNode()
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
