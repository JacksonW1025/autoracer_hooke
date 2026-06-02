import copy
import math

import rclpy
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

STATE_STARTUP = "STARTUP"
STATE_TRACKING = "TRACKING"
STATE_LOST_RECOVERY = "LOST_RECOVERY"


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw):
    q = PoseWithCovarianceStamped().pose.pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
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


def _propagate(x, y, yaw, velocity, yaw_rate, dt):
    if abs(yaw_rate) < 1e-5:
        return (
            x + velocity * math.cos(yaw) * dt,
            y + velocity * math.sin(yaw) * dt,
            _normalize_angle(yaw),
        )

    next_yaw = _normalize_angle(yaw + yaw_rate * dt)
    radius = velocity / yaw_rate
    return (
        x + radius * (math.sin(next_yaw) - math.sin(yaw)),
        y - radius * (math.cos(next_yaw) - math.cos(yaw)),
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
        self.declare_parameter("enable_seed_bias_correction", False)
        self.declare_parameter("seed_bias_correction_alpha", 0.25)
        self.declare_parameter("seed_bias_correction_max_age_sec", 0.5)
        self.declare_parameter("enable_tracking_seed_fusion", True)
        self.declare_parameter("max_tracking_seed_stddev_m", 0.75)
        self.declare_parameter("max_tracking_seed_age_sec", 0.5)
        self.declare_parameter("log_seed_decisions", False)

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
        self._max_tracking_seed_stddev = float(
            self.get_parameter("max_tracking_seed_stddev_m").value
        )
        self._max_tracking_seed_age = float(
            self.get_parameter("max_tracking_seed_age_sec").value
        )

        self._state = None
        self._predictor_state = STATE_STARTUP
        self._last_ndt_receipt = None
        self._last_seed_pose = None
        self._last_seed_stamp = None
        self._last_velocity = None
        self._last_velocity_receipt = None
        self._last_steering = None
        self._last_steering_receipt = None
        self._seed_reset_active = False
        self._last_seed_reset_stamp = None
        self._last_startup_seed_refresh_stamp = None
        self._startup_seed_refresh_count = 0
        self._tracking_seed_ignored_count = 0
        self._tracking_seed_fusion_count = 0
        self._lost_recovery_seed_reset_count = 0
        self._startup_seed_cooldown_ignored_count = 0
        self._lost_recovery_seed_ignored_count = 0
        self._seed_bias_correction = (0.0, 0.0, 0.0, 0.0)
        self._seed_bias_correction_sample_count = 0

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("output_topic").value, 10
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
            if self._set_state_from_pose(corrected_msg, stamp, source="Fixposition seed reset"):
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
                self._tracking_seed_ignored_count += 1
                self._log_seed_decision("ignored", reason)

    def _on_ndt_pose(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        if self._ndt_contradicts_fresh_seed(msg, stamp):
            return
        if self._set_state_from_pose(msg, stamp, source="NDT"):
            self._update_seed_bias_correction(msg, stamp)
            self._last_ndt_receipt = stamp
            self._seed_reset_active = False
            self._predictor_state = STATE_TRACKING

    def _on_velocity(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        self._advance_state_to(stamp)
        self._last_velocity = msg
        self._last_velocity_receipt = stamp

    def _on_steering(self, msg):
        stamp = _message_time(msg, self.get_clock().now())
        self._advance_state_to(stamp)
        self._last_steering = msg
        self._last_steering_receipt = stamp

    def _on_timer(self):
        if self._state is None:
            return

        now = self.get_clock().now()
        self._advance_state(now)
        self._publisher.publish(self._state_to_msg(now))

    def _set_state_from_pose(self, msg, stamp, source):
        pose = msg.pose.pose
        if not self._pose_is_usable(msg):
            self.get_logger().warn(f"Ignoring unusable {source} pose", throttle_duration_sec=1.0)
            return False

        self._state = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": _yaw_from_quaternion(pose.orientation),
            "covariance": list(msg.pose.covariance),
            "stamp": stamp,
        }
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
            self._state["stamp"] = now
            return

        velocity, yaw_rate = self._motion(now)
        remaining = dt
        while remaining > 1e-9:
            step = min(remaining, self._max_prediction_step)
            x, y, yaw = _propagate(
                self._state["x"],
                self._state["y"],
                self._state["yaw"],
                velocity,
                yaw_rate,
                step,
            )
            self._state["x"] = x
            self._state["y"] = y
            self._state["yaw"] = yaw

            distance = abs(velocity) * step
            cov = self._state["covariance"]
            cov[0] = float(cov[0]) + self._process_xy_noise_per_m * distance
            cov[7] = float(cov[7]) + self._process_xy_noise_per_m * distance
            cov[35] = float(cov[35]) + self._process_yaw_noise_per_s * step
            remaining -= step

        self._state["stamp"] = now

    def _advance_state_to(self, stamp):
        if self._state is None:
            return
        if (stamp - self._state["stamp"]).nanoseconds <= 0:
            return
        self._advance_state(stamp)

    def _motion(self, now):
        if self._last_velocity is None or self._last_velocity_receipt is None:
            return 0.0, 0.0

        velocity_age = (now - self._last_velocity_receipt).nanoseconds / 1e9
        if velocity_age > self._vehicle_status_timeout:
            return 0.0, 0.0

        velocity = float(self._last_velocity.longitudinal_velocity)
        if not math.isfinite(velocity):
            velocity = 0.0

        yaw_rate = float(self._last_velocity.heading_rate)
        if math.isfinite(yaw_rate):
            return velocity, yaw_rate

        return velocity, self._yaw_rate_from_steering(now, velocity)

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

    def _state_to_msg(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = self._state["x"]
        msg.pose.pose.position.y = self._state["y"]
        msg.pose.pose.position.z = self._state["z"]
        msg.pose.pose.orientation = _yaw_to_quaternion(self._state["yaw"])
        msg.pose.covariance = copy.copy(self._state["covariance"])
        return msg

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
