import copy
import json
import math
import random

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

STATE_STARTUP = "STARTUP"
STATE_TRACKING = "TRACKING"
STATE_LOST_RECOVERY = "LOST_RECOVERY"


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rpy_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch, _yaw_from_quaternion(q)


def _yaw_to_quaternion(yaw):
    return _rpy_to_quaternion(0.0, 0.0, yaw)


def _rpy_to_quaternion(roll, pitch, yaw):
    q = PoseWithCovarianceStamped().pose.pose.orientation
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def _xy_variance(covariance):
    var_x = float(covariance[0])
    var_y = float(covariance[7])
    if not math.isfinite(var_x) or not math.isfinite(var_y) or var_x < 0.0 or var_y < 0.0:
        return math.inf
    return max(var_x, var_y)


def _yaw_variance(covariance):
    var = float(covariance[35])
    if not math.isfinite(var) or var < 0.0:
        return math.inf
    return var


def _variance_gain(state_var, measurement_var):
    if not math.isfinite(measurement_var):
        return 0.0
    if not math.isfinite(state_var):
        state_var = measurement_var
    state_var = max(float(state_var), 1e-6)
    measurement_var = max(float(measurement_var), 1e-6)
    return min(1.0, max(0.0, state_var / (state_var + measurement_var)))


def _message_time(msg, fallback):
    if hasattr(msg, "header"):
        return rclpy.time.Time.from_msg(msg.header.stamp)
    if hasattr(msg, "stamp"):
        return rclpy.time.Time.from_msg(msg.stamp)
    return fallback


def _propagate(x, y, yaw, velocity, yaw_rate, dt, lateral_velocity=0.0):
    if abs(yaw_rate) < 1e-5:
        return (
            x + (velocity * math.cos(yaw) - lateral_velocity * math.sin(yaw)) * dt,
            y + (velocity * math.sin(yaw) + lateral_velocity * math.cos(yaw)) * dt,
            _normalize_angle(yaw),
        )

    next_yaw = _normalize_angle(yaw + yaw_rate * dt)
    return (
        x
        + velocity / yaw_rate * (math.sin(next_yaw) - math.sin(yaw))
        + lateral_velocity / yaw_rate * (math.cos(next_yaw) - math.cos(yaw)),
        y
        + velocity / yaw_rate * (math.cos(yaw) - math.cos(next_yaw))
        + lateral_velocity / yaw_rate * (math.sin(next_yaw) - math.sin(yaw)),
        next_yaw,
    )


class NdtInitialPosePredictor(Node):
    def __init__(self):
        super().__init__("ndt_initial_pose_predictor")

        self.declare_parameter("seed_pose_topic", "/localization/fixposition/seed_pose")
        self.declare_parameter("ndt_pose_topic", "/localization/pose_with_covariance")
        self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("steering_topic", "/vehicle/status/steering_status")
        self.declare_parameter("output_topic", "/localization/ndt_initial_pose")
        self.declare_parameter("prediction_output_topic", "")
        self.declare_parameter("regularization_seed_topic", "")
        self.declare_parameter("corrected_seed_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wheel_base_m", 1.9)
        self.declare_parameter("vehicle_status_timeout_sec", 0.5)
        self.declare_parameter("ndt_lost_timeout_sec", 1.0)
        self.declare_parameter("max_prediction_step_sec", 0.2)
        self.declare_parameter("process_xy_noise_per_m", 0.02)
        self.declare_parameter("process_yaw_noise_per_s", 0.0025)
        self.declare_parameter("ndt_seed_deviation_guard_m", 0.0)
        self.declare_parameter("ndt_seed_deviation_guard_max_age_sec", 1.0)
        self.declare_parameter("seed_reset_cooldown_sec", 1.0)
        self.declare_parameter("max_lost_recovery_along_residual_m", 1.0)
        self.declare_parameter("enable_seed_bias_correction", False)
        self.declare_parameter("seed_bias_correction_alpha", 0.25)
        self.declare_parameter("seed_bias_correction_max_age_sec", 0.5)
        self.declare_parameter("enable_tracking_seed_fusion", True)
        self.declare_parameter("enable_tracking_seed_along_fusion", False)
        self.declare_parameter("tracking_seed_along_gain", 0.03)
        self.declare_parameter("tracking_seed_along_min_interval_sec", 0.1)
        self.declare_parameter("max_tracking_seed_stddev_m", 0.75)
        self.declare_parameter("max_tracking_seed_age_sec", 0.5)
        self.declare_parameter("log_seed_decisions", False)
        self.declare_parameter("enable_lost_recovery_hypotheses", False)
        self.declare_parameter("recovery_hypothesis_period_sec", 0.2)
        self.declare_parameter(
            "recovery_hypothesis_along_offsets_m",
            [0.0, 2.0, -2.0, 5.0, -5.0, 10.0, -10.0],
        )
        self.declare_parameter("recovery_hypothesis_cross_offsets_m", [0.0, 0.75, -0.75])
        self.declare_parameter("recovery_hypothesis_yaw_offsets_deg", [0.0, 3.0, -3.0])
        self.declare_parameter("relocalization_decision_topic", "/localization/relocalization/decision")
        self.declare_parameter("motion_noise_seed", 424242)
        self.declare_parameter("motion_velocity_scale_error", 0.0)
        self.declare_parameter("motion_longitudinal_velocity_bias_mps", 0.0)
        self.declare_parameter("motion_velocity_white_noise_stddev_mps", 0.0)
        self.declare_parameter("motion_yaw_rate_bias_rad_s", 0.0)
        self.declare_parameter("motion_yaw_rate_random_walk_stddev_rad_sqrt_s", 0.0)
        self.declare_parameter("enable_motion_scale_correction", False)
        self.declare_parameter("preserve_tracking_ndt_along", False)
        self.declare_parameter("tracking_ndt_max_along_correction_m", 0.0)
        self.declare_parameter("motion_scale_correction_alpha", 0.05)
        self.declare_parameter("motion_scale_correction_max_abs", 0.03)
        self.declare_parameter("motion_scale_correction_min_distance_m", 1.0)
        self.declare_parameter("motion_scale_correction_max_cross_residual_m", 0.5)
        self.declare_parameter("motion_scale_correction_observation_limit", 0.03)
        self.declare_parameter("motion_scale_correction_max_step_abs", 0.0)
        self.declare_parameter("motion_scale_correction_bootstrap_min_abs", 0.0)
        self.declare_parameter("motion_scale_correction_bootstrap_min_updates", 0)
        self.declare_parameter("motion_scale_correction_bootstrap_initial_observation_count", 1)
        self.declare_parameter("motion_scale_correction_min_stamp_sec", 0.0)
        self.declare_parameter(
            "motion_scale_correction_opposite_observation_required_count",
            3,
        )
        self.declare_parameter(
            "runtime_multistart_decision_topic",
            "/localization/ndt/runtime_multistart/decision",
        )
        self.declare_parameter("motion_scale_correction_runtime_decision_max_age_sec", 0.2)
        self.declare_parameter("motion_scale_correction_skip_ambiguous_runtime", True)
        self.declare_parameter(
            "motion_scale_correction_robust_decision_topic",
            "/localization/robust_ndt/decision",
        )
        self.declare_parameter("motion_scale_correction_require_robust_decision", False)
        self.declare_parameter("motion_scale_correction_robust_decision_max_age_sec", 0.2)
        self.declare_parameter("motion_scale_correction_max_mahalanobis", 2.0)
        self.declare_parameter("motion_scale_correction_max_innovation_along_m", 0.8)
        self.declare_parameter("motion_scale_correction_max_innovation_cross_m", 0.25)
        self.declare_parameter("motion_scale_correction_max_innovation_yaw_deg", 2.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._wheel_base = float(self.get_parameter("wheel_base_m").value)
        self._vehicle_status_timeout = float(
            self.get_parameter("vehicle_status_timeout_sec").value
        )
        self._ndt_lost_timeout = float(self.get_parameter("ndt_lost_timeout_sec").value)
        self._max_prediction_step = float(self.get_parameter("max_prediction_step_sec").value)
        self._process_xy_noise_per_m = float(
            self.get_parameter("process_xy_noise_per_m").value
        )
        self._process_yaw_noise_per_s = float(
            self.get_parameter("process_yaw_noise_per_s").value
        )
        self._ndt_seed_deviation_guard = float(
            self.get_parameter("ndt_seed_deviation_guard_m").value
        )
        self._ndt_seed_deviation_guard_max_age = float(
            self.get_parameter("ndt_seed_deviation_guard_max_age_sec").value
        )
        self._seed_reset_cooldown = float(
            self.get_parameter("seed_reset_cooldown_sec").value
        )
        self._max_lost_recovery_along_residual = max(
            0.0, float(self.get_parameter("max_lost_recovery_along_residual_m").value)
        )
        self._log_seed_decisions = bool(self.get_parameter("log_seed_decisions").value)
        self._regularization_seed_topic = str(
            self.get_parameter("regularization_seed_topic").value
        )
        self._corrected_seed_topic = str(self.get_parameter("corrected_seed_topic").value)
        self._enable_seed_bias_correction = bool(
            self.get_parameter("enable_seed_bias_correction").value
        )
        self._seed_bias_correction_alpha = min(
            1.0, max(0.0, float(self.get_parameter("seed_bias_correction_alpha").value))
        )
        self._seed_bias_correction_max_age = float(
            self.get_parameter("seed_bias_correction_max_age_sec").value
        )
        self._enable_tracking_seed_fusion = bool(
            self.get_parameter("enable_tracking_seed_fusion").value
        )
        self._enable_tracking_seed_along_fusion = bool(
            self.get_parameter("enable_tracking_seed_along_fusion").value
        )
        self._tracking_seed_along_gain = min(
            1.0, max(0.0, float(self.get_parameter("tracking_seed_along_gain").value))
        )
        self._tracking_seed_along_min_interval = max(
            0.0, float(self.get_parameter("tracking_seed_along_min_interval_sec").value)
        )
        self._max_tracking_seed_stddev = float(
            self.get_parameter("max_tracking_seed_stddev_m").value
        )
        self._max_tracking_seed_age = float(
            self.get_parameter("max_tracking_seed_age_sec").value
        )
        self._enable_lost_recovery_hypotheses = bool(
            self.get_parameter("enable_lost_recovery_hypotheses").value
        )
        self._recovery_hypothesis_period = max(
            1e-3, float(self.get_parameter("recovery_hypothesis_period_sec").value)
        )
        self._recovery_hypotheses = self._build_recovery_hypotheses(
            self.get_parameter("recovery_hypothesis_along_offsets_m").value,
            self.get_parameter("recovery_hypothesis_cross_offsets_m").value,
            self.get_parameter("recovery_hypothesis_yaw_offsets_deg").value,
        )
        self._relocalization_decision_topic = str(
            self.get_parameter("relocalization_decision_topic").value
        )
        self._motion_rng = random.Random(int(self.get_parameter("motion_noise_seed").value))
        self._motion_velocity_scale_error = float(
            self.get_parameter("motion_velocity_scale_error").value
        )
        self._motion_longitudinal_velocity_bias = float(
            self.get_parameter("motion_longitudinal_velocity_bias_mps").value
        )
        self._motion_velocity_white_noise_stddev = max(
            0.0, float(self.get_parameter("motion_velocity_white_noise_stddev_mps").value)
        )
        self._motion_yaw_rate_bias = float(
            self.get_parameter("motion_yaw_rate_bias_rad_s").value
        )
        self._motion_yaw_rate_random_walk_stddev = max(
            0.0,
            float(self.get_parameter("motion_yaw_rate_random_walk_stddev_rad_sqrt_s").value),
        )
        self._motion_yaw_rate_random_walk_bias = 0.0
        self._last_motion_noise_stamp = None
        self._enable_motion_scale_correction = bool(
            self.get_parameter("enable_motion_scale_correction").value
        )
        self._preserve_tracking_ndt_along = bool(
            self.get_parameter("preserve_tracking_ndt_along").value
        )
        self._tracking_ndt_max_along_correction = max(
            0.0, float(self.get_parameter("tracking_ndt_max_along_correction_m").value)
        )
        self._motion_scale_correction_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("motion_scale_correction_alpha").value)),
        )
        self._motion_scale_correction_max_abs = max(
            0.0, float(self.get_parameter("motion_scale_correction_max_abs").value)
        )
        self._motion_scale_correction_min_distance = max(
            0.0, float(self.get_parameter("motion_scale_correction_min_distance_m").value)
        )
        self._motion_scale_correction_max_cross_residual = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_max_cross_residual_m").value),
        )
        self._motion_scale_correction_observation_limit = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_observation_limit").value),
        )
        self._motion_scale_correction_max_step_abs = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_max_step_abs").value),
        )
        self._motion_scale_correction_bootstrap_min_abs = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_bootstrap_min_abs").value),
        )
        self._motion_scale_correction_bootstrap_min_updates = max(
            0,
            int(self.get_parameter("motion_scale_correction_bootstrap_min_updates").value),
        )
        self._motion_scale_correction_bootstrap_initial_observation_count = max(
            1,
            int(
                self.get_parameter(
                    "motion_scale_correction_bootstrap_initial_observation_count"
                ).value
            ),
        )
        self._motion_scale_correction_min_stamp_sec = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_min_stamp_sec").value),
        )
        self._motion_scale_correction_opposite_observation_required_count = max(
            1,
            int(
                self.get_parameter(
                    "motion_scale_correction_opposite_observation_required_count"
                ).value
            ),
        )
        self._runtime_multistart_decision_topic = str(
            self.get_parameter("runtime_multistart_decision_topic").value
        )
        self._motion_scale_correction_runtime_decision_max_age = max(
            0.0,
            float(
                self.get_parameter(
                    "motion_scale_correction_runtime_decision_max_age_sec"
                ).value
            ),
        )
        self._motion_scale_correction_skip_ambiguous_runtime = bool(
            self.get_parameter("motion_scale_correction_skip_ambiguous_runtime").value
        )
        self._motion_scale_correction_robust_decision_topic = str(
            self.get_parameter("motion_scale_correction_robust_decision_topic").value
        )
        self._motion_scale_correction_require_robust_decision = bool(
            self.get_parameter("motion_scale_correction_require_robust_decision").value
        )
        self._motion_scale_correction_robust_decision_max_age = max(
            0.0,
            float(
                self.get_parameter(
                    "motion_scale_correction_robust_decision_max_age_sec"
                ).value
            ),
        )
        self._motion_scale_correction_max_mahalanobis = max(
            0.0,
            float(self.get_parameter("motion_scale_correction_max_mahalanobis").value),
        )
        self._motion_scale_correction_max_innovation_along = max(
            0.0,
            float(
                self.get_parameter("motion_scale_correction_max_innovation_along_m").value
            ),
        )
        self._motion_scale_correction_max_innovation_cross = max(
            0.0,
            float(
                self.get_parameter("motion_scale_correction_max_innovation_cross_m").value
            ),
        )
        self._motion_scale_correction_max_innovation_yaw_deg = max(
            0.0,
            float(
                self.get_parameter("motion_scale_correction_max_innovation_yaw_deg").value
            ),
        )
        self._motion_velocity_scale_correction = 0.0
        self._last_motion_scale_pose_sample = None
        self._motion_scale_correction_motion_accum = 0.0
        self._motion_scale_correction_accepted_accum = 0.0
        self._motion_scale_correction_innovation_along_accum = 0.0
        self._motion_scale_correction_innovation_sample_count = 0
        self._motion_scale_correction_update_count = 0
        self._motion_scale_correction_opposite_observation_streak = 0
        self._motion_scale_correction_initial_observation_streak = 0
        self._motion_scale_correction_initial_observation_sign = 0
        self._last_runtime_multistart_decision = None
        self._last_robust_ndt_decision = None

        self._state = None
        self._predictor_state = STATE_STARTUP
        self._last_ndt_receipt = None
        self._last_seed_pose = None
        self._last_seed_stamp = None
        self._last_velocity = None
        self._last_velocity_receipt = None
        self._last_steering = None
        self._last_steering_receipt = None
        self._motion_history = []
        self._max_motion_history_samples = 20000
        self._seed_reset_active = False
        self._last_seed_reset_stamp = None
        self._last_startup_seed_refresh_stamp = None
        self._last_tracking_seed_along_fusion_stamp = None
        self._startup_seed_refresh_count = 0
        self._tracking_seed_ignored_count = 0
        self._tracking_seed_fusion_count = 0
        self._tracking_seed_along_fusion_count = 0
        self._lost_recovery_seed_reset_count = 0
        self._startup_seed_cooldown_ignored_count = 0
        self._lost_recovery_seed_ignored_count = 0
        self._relocalization_attempt_count = 0
        self._seed_bias_correction = (0.0, 0.0, 0.0, 0.0)
        self._seed_bias_correction_sample_count = 0

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("output_topic").value, 10
        )
        prediction_output_topic = str(self.get_parameter("prediction_output_topic").value)
        self._prediction_publisher = (
            self.create_publisher(PoseWithCovarianceStamped, prediction_output_topic, 10)
            if prediction_output_topic
            else None
        )
        self._regularization_seed_publisher = (
            self.create_publisher(PoseWithCovarianceStamped, self._regularization_seed_topic, 10)
            if self._regularization_seed_topic
            else None
        )
        self._corrected_seed_publisher = (
            self.create_publisher(PoseWithCovarianceStamped, self._corrected_seed_topic, 10)
            if self._corrected_seed_topic
            else None
        )
        self._relocalization_decision_publisher = (
            self.create_publisher(String, self._relocalization_decision_topic, 10)
            if self._relocalization_decision_topic
            else None
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("seed_pose_topic").value,
            self._on_seed_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("ndt_pose_topic").value,
            self._on_ndt_pose,
            10,
        )
        if self._runtime_multistart_decision_topic:
            self.create_subscription(
                String,
                self._runtime_multistart_decision_topic,
                self._on_runtime_multistart_decision,
                10,
            )
        if self._motion_scale_correction_robust_decision_topic:
            self.create_subscription(
                String,
                self._motion_scale_correction_robust_decision_topic,
                self._on_robust_ndt_decision,
                10,
            )
        self.create_subscription(
            VelocityReport,
            self.get_parameter("velocity_topic").value,
            self._on_velocity,
            10,
        )
        self.create_subscription(
            SteeringReport,
            self.get_parameter("steering_topic").value,
            self._on_steering,
            10,
        )

        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"Publishing NDT initial pose on {self.get_parameter('output_topic').value}"
        )

    def _on_seed_pose(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        if not self._pose_is_usable(msg):
            self.get_logger().warn(
                "Ignoring unusable Fixposition seed pose", throttle_duration_sec=1.0
            )
            return
        pose = msg.pose.pose
        self._last_seed_pose = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            _yaw_from_quaternion(pose.orientation),
        )
        self._last_seed_stamp = stamp
        corrected_msg = self._corrected_seed_msg(msg)
        self._publish_recovery_regularization_seed(msg, stamp)
        self._publish_corrected_seed(corrected_msg)
        if self._state is None:
            if self._set_state_from_pose(corrected_msg, stamp, source="Fixposition seed startup"):
                self._seed_reset_active = False
                self._last_startup_seed_refresh_stamp = stamp
                self._predictor_state = STATE_STARTUP
        elif self._last_ndt_receipt is None:
            if self._can_refresh_startup_seed(stamp):
                if self._set_state_from_pose(
                    corrected_msg, stamp, source="Fixposition seed startup refresh"
                ):
                    self._seed_reset_active = False
                    self._last_startup_seed_refresh_stamp = stamp
                    self._startup_seed_refresh_count += 1
                    self._predictor_state = STATE_STARTUP
            else:
                self._startup_seed_cooldown_ignored_count += 1
                self._log_seed_decision("ignored", "startup cooldown")
        elif self._ndt_is_lost(stamp) and self._can_reset_from_seed(stamp):
            if self._recover_from_seed_bounded_along(
                corrected_msg, stamp, source="Fixposition seed bounded recovery"
            ):
                self._seed_reset_active = True
                self._last_seed_reset_stamp = stamp
                self._lost_recovery_seed_reset_count += 1
                self._predictor_state = STATE_LOST_RECOVERY
        elif self._ndt_is_lost(stamp):
            self._lost_recovery_seed_ignored_count += 1
            self._log_seed_decision("ignored", "lost recovery already active")
        else:
            fused, reason = self._fuse_tracking_seed(corrected_msg, stamp)
            if fused:
                self._tracking_seed_fusion_count += 1
                self._log_seed_decision("fused", reason)
            else:
                along_fused, along_reason = self._fuse_tracking_seed_along(corrected_msg, stamp)
                if along_fused:
                    self._tracking_seed_along_fusion_count += 1
                    self._log_seed_decision("fused", along_reason)
                else:
                    self._tracking_seed_ignored_count += 1
                    self._log_seed_decision("ignored", along_reason if reason.endswith("disabled") else reason)

    def _on_ndt_pose(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        if self._ndt_contradicts_fresh_seed(msg, stamp):
            return
        was_tracking = self._predictor_state == STATE_TRACKING
        if was_tracking and self._preserve_tracking_ndt_along:
            state_updated = self._set_state_from_tracking_ndt_pose(msg, stamp, source="NDT")
        else:
            state_updated = self._set_state_from_pose(msg, stamp, source="NDT")
        if state_updated:
            self._update_motion_scale_correction_from_pose(
                msg,
                stamp,
                was_tracking=was_tracking,
            )
            self._update_seed_bias_correction(msg, stamp)
            self._last_ndt_receipt = stamp
            self._seed_reset_active = False
            self._predictor_state = STATE_TRACKING

    def _on_runtime_multistart_decision(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if isinstance(payload, dict):
            self._last_runtime_multistart_decision = payload

    def _on_robust_ndt_decision(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if isinstance(payload, dict):
            self._last_robust_ndt_decision = payload

    def _on_velocity(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        self._advance_state_to_status_stamp(stamp)
        self._last_velocity = msg
        self._last_velocity_receipt = stamp
        self._record_motion_sample(stamp, msg)

    def _on_steering(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        self._advance_state_to_status_stamp(stamp)
        self._last_steering = msg
        self._last_steering_receipt = stamp

    def _on_timer(self):
        if self._state is None:
            return

        now = self.get_clock().now()
        self._advance_state(now)
        self._publisher.publish(self._state_to_msg(now))
        if self._prediction_publisher is not None:
            self._prediction_publisher.publish(
                self._state_to_msg(now, include_recovery_hypothesis=False)
            )

    def _set_state_from_pose(self, msg, stamp, source):
        pose = msg.pose.pose
        if not self._pose_is_usable(msg):
            self.get_logger().warn(f"Ignoring unusable {source} pose", throttle_duration_sec=1.0)
            return False

        roll, pitch, yaw = _rpy_from_quaternion(pose.orientation)
        self._state = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "covariance": list(msg.pose.covariance),
            "stamp": stamp,
        }
        self.get_logger().info(f"Initial-pose predictor corrected from {source}")
        return True

    def _set_state_from_tracking_ndt_pose(self, msg, stamp, source):
        if self._state is None:
            return self._set_state_from_pose(msg, stamp, source=source)
        if not self._pose_is_usable(msg):
            self.get_logger().warn(f"Ignoring unusable {source} pose", throttle_duration_sec=1.0)
            return False

        self._advance_state_to(stamp)
        pose = msg.pose.pose
        roll, pitch, yaw = _rpy_from_quaternion(pose.orientation)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        lateral_x = -math.sin(yaw)
        lateral_y = math.cos(yaw)
        dx = float(pose.position.x) - float(self._state["x"])
        dy = float(pose.position.y) - float(self._state["y"])
        along = dx * forward_x + dy * forward_y
        cross = dx * lateral_x + dy * lateral_y
        along_correction = 0.0
        if self._tracking_ndt_max_along_correction > 0.0:
            along_correction = max(
                -self._tracking_ndt_max_along_correction,
                min(self._tracking_ndt_max_along_correction, along),
            )

        self._state["x"] = (
            float(self._state["x"]) + forward_x * along_correction + lateral_x * cross
        )
        self._state["y"] = (
            float(self._state["y"]) + forward_y * along_correction + lateral_y * cross
        )
        self._state["z"] = float(pose.position.z)
        self._state["roll"] = roll
        self._state["pitch"] = pitch
        self._state["yaw"] = yaw
        self._state["covariance"] = list(msg.pose.covariance)
        self._state["stamp"] = stamp
        self.get_logger().info(f"Initial-pose predictor tracking-corrected from {source}")
        return True

    def _recover_from_seed_bounded_along(self, msg, stamp, source):
        if self._state is None:
            return self._set_state_from_pose(msg, stamp, source=source)
        if not self._pose_is_usable(msg):
            self.get_logger().warn(f"Ignoring unusable {source} pose", throttle_duration_sec=1.0)
            return False

        self._advance_state_to(stamp)
        seed_pose = msg.pose.pose
        seed_yaw = _yaw_from_quaternion(seed_pose.orientation)
        forward_x = math.cos(seed_yaw)
        forward_y = math.sin(seed_yaw)
        dx = float(self._state["x"]) - float(seed_pose.position.x)
        dy = float(self._state["y"]) - float(seed_pose.position.y)
        along = dx * forward_x + dy * forward_y
        if abs(along) > self._max_lost_recovery_along_residual:
            along = 0.0

        self._state["x"] = float(seed_pose.position.x) + forward_x * along
        self._state["y"] = float(seed_pose.position.y) + forward_y * along
        self._state["z"] = float(seed_pose.position.z)
        self._state["roll"], self._state["pitch"], _ = _rpy_from_quaternion(seed_pose.orientation)
        self._state["yaw"] = seed_yaw
        self._state["stamp"] = stamp

        seed_cov = msg.pose.covariance
        cov = self._state["covariance"]
        cov[14] = seed_cov[14]
        cov[35] = seed_cov[35]
        self.get_logger().info(f"Initial-pose predictor corrected from {source}")
        return True

    def _log_seed_decision(self, action, reason):
        if self._log_seed_decisions:
            self.get_logger().info(
                f"Fixposition seed {action}: state={self._predictor_state} reason={reason}"
            )

    def _publish_recovery_regularization_seed(self, msg, stamp):
        if self._regularization_seed_publisher is None:
            return
        if self._last_ndt_receipt is not None and not self._ndt_is_lost(stamp):
            return
        self._regularization_seed_publisher.publish(msg)

    def _publish_corrected_seed(self, msg):
        if self._corrected_seed_publisher is not None:
            self._corrected_seed_publisher.publish(msg)

    def _corrected_seed_msg(self, msg):
        if not self._enable_seed_bias_correction:
            return msg
        corrected = copy.deepcopy(msg)
        dx, dy, dz, dyaw = self._seed_bias_correction
        pose = corrected.pose.pose
        pose.position.x = float(pose.position.x) + dx
        pose.position.y = float(pose.position.y) + dy
        pose.position.z = float(pose.position.z) + dz
        pose.orientation = _yaw_to_quaternion(_yaw_from_quaternion(pose.orientation) + dyaw)
        return corrected

    def _update_seed_bias_correction(self, ndt_msg, stamp):
        if (
            not self._enable_seed_bias_correction
            or self._last_seed_pose is None
            or self._last_seed_stamp is None
            or not self._pose_is_usable(ndt_msg)
        ):
            return

        seed_age = abs((stamp - self._last_seed_stamp).nanoseconds / 1e9)
        if seed_age > self._seed_bias_correction_max_age:
            return

        seed_x, seed_y, seed_z, seed_yaw = self._last_seed_pose
        pose = ndt_msg.pose.pose
        sample = (
            float(pose.position.x) - seed_x,
            float(pose.position.y) - seed_y,
            float(pose.position.z) - seed_z,
            _normalize_angle(_yaw_from_quaternion(pose.orientation) - seed_yaw),
        )
        if self._seed_bias_correction_sample_count == 0:
            self._seed_bias_correction = sample
        else:
            alpha = self._seed_bias_correction_alpha
            old_x, old_y, old_z, old_yaw = self._seed_bias_correction
            self._seed_bias_correction = (
                (1.0 - alpha) * old_x + alpha * sample[0],
                (1.0 - alpha) * old_y + alpha * sample[1],
                (1.0 - alpha) * old_z + alpha * sample[2],
                _normalize_angle((1.0 - alpha) * old_yaw + alpha * sample[3]),
            )
        self._seed_bias_correction_sample_count += 1

    def _fuse_tracking_seed(self, msg, stamp):
        if not self._enable_tracking_seed_fusion:
            return False, "tracking seed fusion disabled"
        if self._state is None or not self._pose_is_usable(msg):
            return False, "tracking seed fusion unavailable"

        if (stamp - self._state["stamp"]).nanoseconds > 0:
            self._advance_state_to(stamp)
        seed_age = abs((stamp - self._state["stamp"]).nanoseconds / 1e9)
        if self._max_tracking_seed_age > 0.0 and seed_age > self._max_tracking_seed_age:
            return False, f"tracking seed stale age={seed_age:.3f}s"

        seed_var = _xy_variance(msg.pose.covariance)
        seed_stddev = math.sqrt(seed_var) if math.isfinite(seed_var) else math.inf
        if self._max_tracking_seed_stddev > 0.0 and seed_stddev > self._max_tracking_seed_stddev:
            return False, f"tracking seed covariance stddev {seed_stddev:.3f}m too high"

        gain = _variance_gain(_xy_variance(self._state["covariance"]), seed_var)

        pose = msg.pose.pose
        dx = float(pose.position.x) - self._state["x"]
        dy = float(pose.position.y) - self._state["y"]
        yaw = self._state["yaw"]
        cross = -dx * math.sin(yaw) + dy * math.cos(yaw)
        applied = False
        if abs(cross) >= 1e-4 and gain > 0.0:
            self._state["x"] += -math.sin(yaw) * cross * gain
            self._state["y"] += math.cos(yaw) * cross * gain
            applied = True

        seed_yaw_var = _yaw_variance(msg.pose.covariance)
        seed_yaw = _yaw_from_quaternion(pose.orientation)
        yaw_error = _normalize_angle(seed_yaw - self._state["yaw"])
        seed_yaw_stddev = math.sqrt(seed_yaw_var) if math.isfinite(seed_yaw_var) else math.inf
        yaw_deadband = max(3.0 * seed_yaw_stddev, 1e-4)
        yaw_gain = _variance_gain(
            _yaw_variance(self._state["covariance"]),
            seed_yaw_var,
        )
        if abs(yaw_error) >= yaw_deadband and yaw_gain > 0.0:
            self._state["yaw"] = _normalize_angle(self._state["yaw"] + yaw_error * yaw_gain)
            applied = True

        if not applied:
            return False, "tracking seed has no lateral/yaw correction"
        return True, f"tracking seed soft correction cross_gain={gain:.3f} yaw_gain={yaw_gain:.3f}"

    def _fuse_tracking_seed_along(self, msg, stamp):
        if not self._enable_tracking_seed_along_fusion:
            return False, "tracking seed along fusion disabled"
        if self._state is None or not self._pose_is_usable(msg):
            return False, "tracking seed along fusion unavailable"
        if (
            self._last_tracking_seed_along_fusion_stamp is not None
            and self._tracking_seed_along_min_interval > 0.0
        ):
            elapsed = (
                stamp - self._last_tracking_seed_along_fusion_stamp
            ).nanoseconds / 1e9
            if elapsed < self._tracking_seed_along_min_interval:
                return False, f"tracking seed along fusion rate limited elapsed={elapsed:.3f}s"

        if (stamp - self._state["stamp"]).nanoseconds > 0:
            self._advance_state_to(stamp)
        seed_age = abs((stamp - self._state["stamp"]).nanoseconds / 1e9)
        if self._max_tracking_seed_age > 0.0 and seed_age > self._max_tracking_seed_age:
            return False, f"tracking seed stale age={seed_age:.3f}s"

        seed_var = _xy_variance(msg.pose.covariance)
        seed_stddev = math.sqrt(seed_var) if math.isfinite(seed_var) else math.inf
        if self._max_tracking_seed_stddev > 0.0 and seed_stddev > self._max_tracking_seed_stddev:
            return False, f"tracking seed covariance stddev {seed_stddev:.3f}m too high"

        gain = min(1.0, max(0.0, self._tracking_seed_along_gain))
        if gain <= 0.0:
            return False, "tracking seed along gain is zero"

        pose = msg.pose.pose
        dx = float(pose.position.x) - self._state["x"]
        dy = float(pose.position.y) - self._state["y"]
        yaw = self._state["yaw"]
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        if abs(along) < 1e-4:
            return False, "tracking seed has no along correction"

        self._state["x"] += math.cos(yaw) * along * gain
        self._state["y"] += math.sin(yaw) * along * gain
        self._last_tracking_seed_along_fusion_stamp = stamp
        return True, f"tracking seed along correction gain={gain:.3f}"

    def _ndt_contradicts_fresh_seed(self, msg, stamp):
        if (
            self._ndt_seed_deviation_guard <= 0.0
            or self._last_seed_pose is None
            or self._last_seed_stamp is None
            or not self._pose_is_usable(msg)
        ):
            return False

        seed_age = abs((stamp - self._last_seed_stamp).nanoseconds / 1e9)
        if seed_age > self._ndt_seed_deviation_guard_max_age:
            return False

        pose = msg.pose.pose
        seed_x, seed_y, _seed_z, _seed_yaw = self._last_seed_pose
        distance = math.hypot(float(pose.position.x) - seed_x, float(pose.position.y) - seed_y)
        if distance <= self._ndt_seed_deviation_guard:
            return False

        self.get_logger().warn(
            f"Ignoring NDT correction {distance:.2f}m from fresh Fixposition seed",
            throttle_duration_sec=1.0,
        )
        return True

    def _advance_state(self, now):
        dt = (now - self._state["stamp"]).nanoseconds / 1e9
        if dt <= 0.0:
            # Asynchronous clock/status callbacks can arrive slightly out of order.
            # Rewinding the stamp would make later callbacks integrate the same
            # interval again and drift the NDT initial pose out of convergence.
            return

        target_ns = now.nanoseconds
        current_ns = self._state["stamp"].nanoseconds
        while current_ns < target_ns:
            velocity, lateral_velocity, yaw_rate = self._motion_at_ns(current_ns)
            next_ns = min(
                target_ns,
                current_ns + int(max(self._max_prediction_step, 1e-6) * 1e9),
            )
            next_motion_ns = self._next_motion_sample_ns_after(current_ns)
            if next_motion_ns is not None and next_motion_ns < next_ns:
                next_ns = next_motion_ns
            step = (next_ns - current_ns) / 1e9
            if step <= 0.0:
                current_ns = next_ns + 1
                continue
            x, y, yaw = _propagate(
                self._state["x"],
                self._state["y"],
                self._state["yaw"],
                velocity,
                yaw_rate,
                step,
                lateral_velocity=lateral_velocity,
            )
            self._state["x"] = x
            self._state["y"] = y
            self._state["yaw"] = yaw

            distance = abs(velocity) * step
            cov = self._state["covariance"]
            cov[0] = float(cov[0]) + self._process_xy_noise_per_m * distance
            cov[7] = float(cov[7]) + self._process_xy_noise_per_m * distance
            cov[35] = float(cov[35]) + self._process_yaw_noise_per_s * step
            current_ns = next_ns

        self._state["stamp"] = now

    def _advance_state_to(self, stamp):
        if self._state is None:
            return
        if (stamp - self._state["stamp"]).nanoseconds <= 0:
            return
        self._advance_state(stamp)

    def _advance_state_to_status_stamp(self, stamp):
        # Vehicle status messages can arrive ahead of the node's ROS clock when
        # /clock is sourced from lower-rate LiDAR stamps. Advancing the internal
        # state into that future and then publishing it with the older clock time
        # creates time-inconsistent NDT initial poses.
        if (stamp - self.get_clock().now()).nanoseconds > 0:
            return
        self._advance_state_to(stamp)

    def _motion(self, now):
        return self._motion_at_ns(now.nanoseconds)

    def _record_motion_sample(self, stamp, msg):
        velocity = float(msg.longitudinal_velocity)
        if not math.isfinite(velocity):
            velocity = 0.0

        lateral_velocity = float(msg.lateral_velocity)
        if not math.isfinite(lateral_velocity):
            lateral_velocity = 0.0

        yaw_rate = float(msg.heading_rate)
        if not math.isfinite(yaw_rate):
            yaw_rate = self._yaw_rate_from_steering(stamp, velocity)
        velocity, lateral_velocity, yaw_rate = self._apply_motion_noise(
            stamp,
            velocity,
            lateral_velocity,
            yaw_rate,
        )

        sample = {
            "stamp_ns": stamp.nanoseconds,
            "velocity": velocity,
            "lateral_velocity": lateral_velocity,
            "yaw_rate": yaw_rate,
        }
        if not self._motion_history or sample["stamp_ns"] >= self._motion_history[-1]["stamp_ns"]:
            self._motion_history.append(sample)
        else:
            index = 0
            while index < len(self._motion_history) and (
                self._motion_history[index]["stamp_ns"] <= sample["stamp_ns"]
            ):
                index += 1
            self._motion_history.insert(index, sample)

        if len(self._motion_history) > self._max_motion_history_samples:
            self._motion_history = self._motion_history[-self._max_motion_history_samples :]

    def _update_motion_scale_correction_from_pose(self, msg, stamp, *, was_tracking=True):
        if not self._enable_motion_scale_correction:
            return

        pose = msg.pose.pose
        _roll, _pitch, yaw = _rpy_from_quaternion(pose.orientation)
        current_sample = {
            "stamp_ns": stamp.nanoseconds,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
        }
        previous_sample = self._last_motion_scale_pose_sample

        if stamp.nanoseconds * 1e-9 < self._motion_scale_correction_min_stamp_sec:
            self._reset_motion_scale_learning_segment()
            return
        if not was_tracking:
            self._reset_motion_scale_learning_segment()
            return
        if (
            previous_sample is not None
            and current_sample["stamp_ns"] <= previous_sample["stamp_ns"]
        ):
            return
        runtime_freeze_reason = self._motion_scale_runtime_decision_freeze_reason(stamp)
        if runtime_freeze_reason is not None:
            if runtime_freeze_reason in {"runtime_recovery", "runtime_tier2"}:
                self._reset_motion_scale_learning_segment()
            else:
                self._clear_motion_scale_learning_anchor()
            return
        robust_freeze_reason = self._motion_scale_robust_decision_freeze_reason(stamp)
        if robust_freeze_reason is not None:
            if robust_freeze_reason in {
                "robust_reject",
                "robust_reason",
                "robust_mahalanobis",
            }:
                self._reset_motion_scale_learning_segment()
            else:
                self._clear_motion_scale_learning_anchor()
            return
        if previous_sample is None:
            self._last_motion_scale_pose_sample = current_sample
            return

        motion_along = self._motion_distance_between_ns(
            previous_sample["stamp_ns"],
            current_sample["stamp_ns"],
        )
        if abs(motion_along) <= 1e-9:
            self._last_motion_scale_pose_sample = current_sample
            return

        previous_yaw = float(previous_sample["yaw"])
        forward_x = math.cos(previous_yaw)
        forward_y = math.sin(previous_yaw)
        dx = current_sample["x"] - float(previous_sample["x"])
        dy = current_sample["y"] - float(previous_sample["y"])
        accepted_along = dx * forward_x + dy * forward_y
        accepted_cross = -dx * forward_y + dy * forward_x

        if (
            self._motion_scale_correction_max_cross_residual > 0.0
            and abs(accepted_cross) > self._motion_scale_correction_max_cross_residual
        ):
            self._last_motion_scale_pose_sample = current_sample
            self._reset_motion_scale_correction_accumulators()
            return

        self._motion_scale_correction_motion_accum += motion_along
        self._motion_scale_correction_accepted_accum += accepted_along
        robust_innovation_along = self._motion_scale_current_robust_innovation_along()
        if robust_innovation_along is not None:
            self._motion_scale_correction_innovation_along_accum += robust_innovation_along
            self._motion_scale_correction_innovation_sample_count += 1
        self._last_motion_scale_pose_sample = current_sample

        if (
            abs(self._motion_scale_correction_motion_accum)
            < self._motion_scale_correction_min_distance
        ):
            return

        observation = (
            self._motion_scale_correction_accepted_accum
            / self._motion_scale_correction_motion_accum
            - 1.0
        )
        if self._motion_scale_correction_require_robust_decision:
            if self._motion_scale_correction_innovation_sample_count <= 0:
                self._reset_motion_scale_correction_accumulators()
                self._reset_motion_scale_opposite_observation_streak()
                return
            robust_observation = (
                self._motion_scale_correction_innovation_along_accum
                / self._motion_scale_correction_motion_accum
            )
            if not math.isfinite(robust_observation):
                self._reset_motion_scale_correction_accumulators()
                self._reset_motion_scale_opposite_observation_streak()
                return
            if self._preserve_tracking_ndt_along:
                observation = robust_observation
            elif self._motion_scale_observations_disagree(observation, robust_observation):
                catchup_observation = (
                    self._motion_scale_robust_disagreement_catchup_observation(
                        robust_observation
                    )
                )
                if catchup_observation is None:
                    self._reset_motion_scale_correction_accumulators()
                    self._reset_motion_scale_opposite_observation_streak()
                    return
                observation = catchup_observation
        if not math.isfinite(observation):
            self._reset_motion_scale_correction_accumulators()
            self._reset_motion_scale_opposite_observation_streak()
            return
        if abs(observation) <= 1e-9:
            self._reset_motion_scale_correction_accumulators()
            self._reset_motion_scale_opposite_observation_streak()
            return

        if self._motion_scale_correction_observation_limit > 0.0:
            observation = max(
                -self._motion_scale_correction_observation_limit,
                min(self._motion_scale_correction_observation_limit, observation),
            )
        if not self._motion_scale_initial_bootstrap_allows_observation(observation):
            self._reset_motion_scale_correction_accumulators()
            self._reset_motion_scale_opposite_observation_streak()
            return
        if self._motion_scale_bootstrap_blocks_observation(observation):
            self._reset_motion_scale_correction_accumulators()
            self._reset_motion_scale_opposite_observation_streak()
            return

        correction = (
            (1.0 - self._motion_scale_correction_alpha)
            * self._motion_velocity_scale_correction
            + self._motion_scale_correction_alpha * observation
        )
        if self._motion_scale_correction_max_step_abs > 0.0:
            delta = correction - self._motion_velocity_scale_correction
            delta = max(
                -self._motion_scale_correction_max_step_abs,
                min(self._motion_scale_correction_max_step_abs, delta),
            )
            correction = self._motion_velocity_scale_correction + delta
        if self._motion_scale_correction_max_abs > 0.0:
            correction = max(
                -self._motion_scale_correction_max_abs,
                min(self._motion_scale_correction_max_abs, correction),
            )

        if math.isfinite(correction):
            self._motion_velocity_scale_correction = correction
            self._motion_scale_correction_update_count += 1
            self._reset_motion_scale_opposite_observation_streak()
            self._reset_motion_scale_initial_observation_streak()
            self._reset_motion_scale_correction_accumulators()

    def _reset_motion_scale_learning_segment(self):
        self._clear_motion_scale_learning_anchor()
        self._reset_motion_scale_correction_accumulators()

    def _reset_motion_scale_correction_accumulators(self):
        self._motion_scale_correction_motion_accum = 0.0
        self._motion_scale_correction_accepted_accum = 0.0
        self._motion_scale_correction_innovation_along_accum = 0.0
        self._motion_scale_correction_innovation_sample_count = 0

    def _clear_motion_scale_learning_anchor(self):
        self._last_motion_scale_pose_sample = None
        self._reset_motion_scale_opposite_observation_streak()
        self._reset_motion_scale_initial_observation_streak()

    def _reset_motion_scale_opposite_observation_streak(self):
        self._motion_scale_correction_opposite_observation_streak = 0

    def _reset_motion_scale_initial_observation_streak(self):
        self._motion_scale_correction_initial_observation_streak = 0
        self._motion_scale_correction_initial_observation_sign = 0

    def _motion_scale_initial_bootstrap_allows_observation(self, observation):
        required = self._motion_scale_correction_bootstrap_initial_observation_count
        if required <= 1:
            return True
        if self._motion_scale_correction_update_count > 0:
            return True
        if abs(self._motion_velocity_scale_correction) > 1e-9:
            return True
        if not math.isfinite(observation) or abs(observation) <= 1e-9:
            self._reset_motion_scale_initial_observation_streak()
            return False
        sign = 1 if observation > 0.0 else -1
        if sign != self._motion_scale_correction_initial_observation_sign:
            self._motion_scale_correction_initial_observation_sign = sign
            self._motion_scale_correction_initial_observation_streak = 1
        else:
            self._motion_scale_correction_initial_observation_streak += 1
        return self._motion_scale_correction_initial_observation_streak >= required

    def _motion_scale_observation_opposes_correction(self, observation):
        current_correction = self._motion_velocity_scale_correction
        if not math.isfinite(observation) or not math.isfinite(current_correction):
            return False
        if abs(observation) <= 1e-9 or abs(current_correction) <= 1e-9:
            return False
        return math.copysign(1.0, observation) != math.copysign(1.0, current_correction)

    def _motion_scale_bootstrap_blocks_observation(self, observation):
        trusted_min = self._motion_scale_correction_bootstrap_min_abs
        trusted_updates = self._motion_scale_correction_bootstrap_min_updates
        if trusted_min <= 0.0 and trusted_updates <= 0:
            return False
        current_correction = self._motion_velocity_scale_correction
        if not math.isfinite(current_correction):
            return False
        if abs(current_correction) <= 1e-9:
            return False
        if trusted_min > 0.0 and abs(current_correction) >= trusted_min:
            return False
        if self._motion_scale_correction_bootstrap_has_trusted_updates():
            return False
        return self._motion_scale_observation_opposes_correction(observation)

    def _motion_scale_robust_disagreement_catchup_observation(self, robust_observation):
        if not self._motion_scale_correction_bootstrap_is_established():
            return None
        if not math.isfinite(robust_observation) or abs(robust_observation) <= 1e-9:
            return None
        if self._motion_scale_observation_opposes_correction(robust_observation):
            return None
        return robust_observation

    def _motion_scale_correction_bootstrap_is_established(self):
        trusted_min = self._motion_scale_correction_bootstrap_min_abs
        if (
            trusted_min > 0.0
            and abs(self._motion_velocity_scale_correction) >= trusted_min
        ):
            return True
        return self._motion_scale_correction_bootstrap_has_trusted_updates()

    def _motion_scale_correction_bootstrap_has_trusted_updates(self):
        trusted_updates = self._motion_scale_correction_bootstrap_min_updates
        return (
            trusted_updates > 0
            and self._motion_scale_correction_update_count >= trusted_updates
        )

    @staticmethod
    def _motion_scale_observations_disagree(first_observation, second_observation):
        if not math.isfinite(first_observation) or not math.isfinite(second_observation):
            return False
        if abs(first_observation) <= 1e-9 or abs(second_observation) <= 1e-9:
            return False
        return math.copysign(1.0, first_observation) != math.copysign(
            1.0, second_observation
        )

    def _motion_scale_current_robust_innovation_along(self):
        decision = self._last_robust_ndt_decision
        if not isinstance(decision, dict):
            return None
        try:
            value = float(decision.get("innovation_along_m"))
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    def _motion_scale_runtime_decision_allows_update(self, stamp):
        return self._motion_scale_runtime_decision_freeze_reason(stamp) is None

    def _motion_scale_runtime_decision_freeze_reason(self, stamp):
        if not self._motion_scale_correction_skip_ambiguous_runtime:
            return None
        decision = self._last_runtime_multistart_decision
        if not isinstance(decision, dict):
            return None
        decision_stamp = decision.get("stamp_sec")
        try:
            decision_age = abs(float(decision_stamp) - stamp.nanoseconds * 1e-9)
        except (TypeError, ValueError):
            return None
        if (
            self._motion_scale_correction_runtime_decision_max_age > 0.0
            and decision_age > self._motion_scale_correction_runtime_decision_max_age
        ):
            return None
        if bool(decision.get("recovery_active", False)):
            return "runtime_recovery"
        if bool(decision.get("tier2_evaluated", False)):
            return "runtime_tier2"
        try:
            candidate_count = int(decision.get("candidate_count", 0))
        except (TypeError, ValueError):
            candidate_count = 0
        selected_candidate_index = decision.get("selected_candidate_index")
        if candidate_count != 1:
            if selected_candidate_index is None:
                return "runtime_candidates"
            if bool(decision.get("small_tier_ambiguous", False)):
                return "runtime_candidates"
            return None
        if selected_candidate_index is not None:
            try:
                if int(selected_candidate_index) != 0:
                    return "runtime_nonbase"
            except (TypeError, ValueError):
                return "runtime_bad_selected"
        if bool(decision.get("small_tier_ambiguous", False)):
            return "runtime_ambiguous"
        return None

    def _motion_scale_robust_decision_allows_update(self, stamp):
        return self._motion_scale_robust_decision_freeze_reason(stamp) is None

    def _motion_scale_robust_decision_freeze_reason(self, stamp):
        if not self._motion_scale_correction_require_robust_decision:
            return None
        decision = self._last_robust_ndt_decision
        if not isinstance(decision, dict):
            return "robust_missing"
        decision_stamp = decision.get("stamp_sec")
        try:
            decision_age = abs(float(decision_stamp) - stamp.nanoseconds * 1e-9)
        except (TypeError, ValueError):
            return "robust_bad_stamp"
        if (
            self._motion_scale_correction_robust_decision_max_age > 0.0
            and decision_age > self._motion_scale_correction_robust_decision_max_age
        ):
            return "robust_old"
        if not bool(decision.get("accepted", False)):
            return "robust_reject"
        if str(decision.get("reason", "")) not in {
            "ekf_measurement_update",
            "bounded_innovation",
        }:
            return "robust_reason"
        if (
            self._motion_scale_correction_max_mahalanobis > 0.0
            and self._decision_abs_float(decision, "mahalanobis")
            > self._motion_scale_correction_max_mahalanobis
        ):
            return "robust_mahalanobis"
        if (
            self._motion_scale_correction_max_innovation_along > 0.0
            and self._decision_abs_float(decision, "innovation_along_m")
            > self._motion_scale_correction_max_innovation_along
        ):
            return "robust_along"
        if (
            self._motion_scale_correction_max_innovation_cross > 0.0
            and self._decision_abs_float(decision, "innovation_cross_m")
            > self._motion_scale_correction_max_innovation_cross
        ):
            return "robust_cross"
        if (
            self._motion_scale_correction_max_innovation_yaw_deg > 0.0
            and self._decision_abs_float(decision, "innovation_yaw_deg")
            > self._motion_scale_correction_max_innovation_yaw_deg
        ):
            return "robust_yaw"
        return None

    @staticmethod
    def _decision_abs_float(decision, key):
        try:
            value = abs(float(decision.get(key, 0.0)))
        except (TypeError, ValueError):
            return math.inf
        if not math.isfinite(value):
            return math.inf
        return value

    def _motion_distance_between_ns(self, start_ns, end_ns):
        if end_ns <= start_ns:
            return 0.0

        current_ns = start_ns
        distance = 0.0
        while current_ns < end_ns:
            velocity, _lateral_velocity, _yaw_rate = self._motion_at_ns(current_ns)
            next_ns = min(
                end_ns,
                current_ns + int(max(self._max_prediction_step, 1e-6) * 1e9),
            )
            next_motion_ns = self._next_motion_sample_ns_after(current_ns)
            if next_motion_ns is not None and next_motion_ns < next_ns:
                next_ns = next_motion_ns
            step = (next_ns - current_ns) / 1e9
            if step <= 0.0:
                current_ns = next_ns + 1
                continue
            distance += velocity * step
            current_ns = next_ns

        return distance

    def _apply_motion_noise(self, stamp, velocity, lateral_velocity, yaw_rate):
        if self._last_motion_noise_stamp is not None:
            dt = max(0.0, (stamp - self._last_motion_noise_stamp).nanoseconds / 1e9)
        else:
            dt = 0.0
        self._last_motion_noise_stamp = stamp
        if self._motion_yaw_rate_random_walk_stddev > 0.0 and dt > 0.0:
            self._motion_yaw_rate_random_walk_bias += (
                self._motion_rng.gauss(0.0, 1.0)
                * self._motion_yaw_rate_random_walk_stddev
                * math.sqrt(dt)
            )
        velocity = (
            float(velocity) * (1.0 + self._motion_velocity_scale_error)
            + self._motion_longitudinal_velocity_bias
        )
        velocity *= 1.0 + self._motion_velocity_scale_correction
        if self._motion_velocity_white_noise_stddev > 0.0:
            velocity += self._motion_rng.gauss(0.0, self._motion_velocity_white_noise_stddev)
        yaw_rate = (
            float(yaw_rate)
            + self._motion_yaw_rate_bias
            + self._motion_yaw_rate_random_walk_bias
        )
        return velocity, float(lateral_velocity), yaw_rate

    def _motion_at_ns(self, stamp_ns):
        sample = self._motion_sample_at_or_before(stamp_ns)
        if sample is not None:
            age = (stamp_ns - sample["stamp_ns"]) / 1e9
            if age <= self._vehicle_status_timeout:
                return sample["velocity"], sample.get("lateral_velocity", 0.0), sample["yaw_rate"]

        if self._last_velocity is None or self._last_velocity_receipt is None:
            return 0.0, 0.0, 0.0

        velocity_age = (stamp_ns - self._last_velocity_receipt.nanoseconds) / 1e9
        if velocity_age > self._vehicle_status_timeout:
            return 0.0, 0.0, 0.0

        velocity = float(self._last_velocity.longitudinal_velocity)
        if not math.isfinite(velocity):
            velocity = 0.0

        lateral_velocity = float(self._last_velocity.lateral_velocity)
        if not math.isfinite(lateral_velocity):
            lateral_velocity = 0.0

        yaw_rate = float(self._last_velocity.heading_rate)
        if math.isfinite(yaw_rate):
            return velocity, lateral_velocity, yaw_rate

        return velocity, lateral_velocity, self._yaw_rate_from_steering(
            rclpy.time.Time(nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type),
            velocity,
        )

    def _motion_sample_at_or_before(self, stamp_ns):
        for sample in reversed(self._motion_history):
            if sample["stamp_ns"] <= stamp_ns:
                return sample
        return None

    def _next_motion_sample_ns_after(self, stamp_ns):
        for sample in self._motion_history:
            if sample["stamp_ns"] > stamp_ns:
                return sample["stamp_ns"]
        return None

    def _yaw_rate_from_steering(self, now, velocity):
        if self._last_steering is None or self._last_steering_receipt is None:
            return 0.0

        steering_age = (now - self._last_steering_receipt).nanoseconds / 1e9
        if steering_age > self._vehicle_status_timeout:
            return 0.0

        steering = float(self._last_steering.steering_tire_angle)
        if not math.isfinite(steering) or self._wheel_base <= 0.0:
            return 0.0
        return velocity * math.tan(steering) / self._wheel_base

    def _state_to_msg(self, stamp, *, include_recovery_hypothesis=True):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._map_frame
        x = self._state["x"]
        y = self._state["y"]
        yaw = self._state["yaw"]
        if include_recovery_hypothesis and self._should_publish_recovery_hypothesis(stamp):
            along, cross, yaw_offset = self._recovery_hypothesis_for_stamp(stamp)
            x += math.cos(yaw) * along - math.sin(yaw) * cross
            y += math.sin(yaw) * along + math.cos(yaw) * cross
            yaw = _normalize_angle(yaw + yaw_offset)
            self._publish_relocalization_decision(stamp, along, cross, yaw_offset)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = self._state["z"]
        msg.pose.pose.orientation = _rpy_to_quaternion(
            self._state.get("roll", 0.0),
            self._state.get("pitch", 0.0),
            yaw,
        )
        msg.pose.covariance = copy.copy(self._state["covariance"])
        return msg

    def _publish_relocalization_decision(self, stamp, along, cross, yaw_offset):
        if self._relocalization_decision_publisher is None:
            return
        self._relocalization_attempt_count += 1
        msg = String()
        msg.data = json.dumps(
            {
                "stamp_sec": stamp.nanoseconds / 1e9,
                "reason": "lost_recovery_hypothesis",
                "attempt_count": self._relocalization_attempt_count,
                "along_offset_m": float(along),
                "cross_offset_m": float(cross),
                "yaw_offset_deg": math.degrees(float(yaw_offset)),
                "uses_gnss_or_gt": False,
            },
            sort_keys=True,
        )
        self._relocalization_decision_publisher.publish(msg)

    @staticmethod
    def _build_recovery_hypotheses(along_offsets, cross_offsets, yaw_offsets_deg):
        hypotheses = []
        for along in [float(value) for value in along_offsets]:
            for cross in [float(value) for value in cross_offsets]:
                for yaw_deg in [float(value) for value in yaw_offsets_deg]:
                    hypotheses.append((along, cross, math.radians(yaw_deg)))
        return hypotheses or [(0.0, 0.0, 0.0)]

    def _should_publish_recovery_hypothesis(self, stamp):
        return (
            self._enable_lost_recovery_hypotheses
            and self._last_ndt_receipt is not None
            and self._ndt_is_lost(stamp)
            and bool(self._recovery_hypotheses)
        )

    def _recovery_hypothesis_for_stamp(self, stamp):
        if not self._should_publish_recovery_hypothesis(stamp):
            return 0.0, 0.0, 0.0
        elapsed = (stamp - self._last_ndt_receipt).nanoseconds / 1e9
        index = int(max(0.0, elapsed) / self._recovery_hypothesis_period)
        return self._recovery_hypotheses[index % len(self._recovery_hypotheses)]

    def _ndt_is_lost(self, now):
        if self._last_ndt_receipt is None:
            return False
        return (now - self._last_ndt_receipt).nanoseconds / 1e9 > self._ndt_lost_timeout

    def _can_reset_from_seed(self, stamp):
        if not self._seed_reset_active:
            return True
        return False

    def _can_refresh_startup_seed(self, stamp):
        if self._last_startup_seed_refresh_stamp is None:
            return True
        if self._seed_reset_cooldown <= 0.0:
            return True
        elapsed = (stamp - self._last_startup_seed_refresh_stamp).nanoseconds / 1e9
        return elapsed >= self._seed_reset_cooldown

    @staticmethod
    def _pose_is_usable(msg):
        pose = msg.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not all(math.isfinite(float(value)) for value in values):
            return False
        norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        return math.isfinite(norm) and norm > 0.1


def main(args=None):
    rclpy.init(args=args)
    node = NdtInitialPosePredictor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
