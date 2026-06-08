import copy
import math
import random
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .fixposition_odom_to_seed_pose import normalize_angle, yaw_from_quaternion, yaw_to_quaternion
from .ndt_initial_pose_predictor import _message_time


@dataclass(frozen=True)
class NoiseSample:
    stamp_sec: float
    xy: tuple[float, float]
    z: float
    yaw: float
    bias_xy: tuple[float, float]
    white_xy: tuple[float, float]


class CorrelatedNoiseModel:
    def __init__(
        self,
        *,
        seed: int,
        tau_sec: float,
        planar_stddev_m: float,
        white_stddev_m: float,
        z_stddev_m: float = 0.0,
        yaw_stddev_deg: float = 0.0,
    ):
        self._rng = random.Random(int(seed))
        self._tau_sec = max(float(tau_sec), 1e-6)
        self._planar_stddev_m = max(float(planar_stddev_m), 0.0)
        self._white_stddev_m = max(float(white_stddev_m), 0.0)
        self._z_stddev_m = max(float(z_stddev_m), 0.0)
        self._yaw_stddev_rad = math.radians(max(float(yaw_stddev_deg), 0.0))
        self._last_stamp_sec: float | None = None
        self._bias_xy = (0.0, 0.0)

    def sample(self, stamp_sec: float) -> NoiseSample:
        stamp_sec = float(stamp_sec)
        if self._last_stamp_sec is None:
            alpha = 0.0
        else:
            dt = max(0.0, stamp_sec - self._last_stamp_sec)
            alpha = math.exp(-dt / self._tau_sec)
        innovation_std = self._planar_stddev_m * math.sqrt(max(0.0, 1.0 - alpha * alpha))
        bx = alpha * self._bias_xy[0] + self._rng.gauss(0.0, innovation_std)
        by = alpha * self._bias_xy[1] + self._rng.gauss(0.0, innovation_std)
        wx = self._rng.gauss(0.0, self._white_stddev_m)
        wy = self._rng.gauss(0.0, self._white_stddev_m)
        z = self._rng.gauss(0.0, self._z_stddev_m)
        yaw = self._rng.gauss(0.0, self._yaw_stddev_rad)
        self._bias_xy = (bx, by)
        self._last_stamp_sec = stamp_sec
        return NoiseSample(
            stamp_sec=stamp_sec,
            xy=(bx + wx, by + wy),
            z=z,
            yaw=yaw,
            bias_xy=(bx, by),
            white_xy=(wx, wy),
        )


def apply_planar_noise_to_odometry(
    msg: Odometry,
    sample: NoiseSample,
    *,
    reported_xy_sigma_m: float,
    reported_z_sigma_m: float,
    reported_yaw_sigma_deg: float,
) -> Odometry:
    noisy = copy.deepcopy(msg)
    noisy.pose.pose.position.x = float(noisy.pose.pose.position.x) + sample.xy[0]
    noisy.pose.pose.position.y = float(noisy.pose.pose.position.y) + sample.xy[1]
    noisy.pose.pose.position.z = float(noisy.pose.pose.position.z) + sample.z
    yaw = yaw_from_quaternion(noisy.pose.pose.orientation)
    noisy.pose.pose.orientation = yaw_to_quaternion(normalize_angle(yaw + sample.yaw))

    xy_var = max(float(reported_xy_sigma_m), 0.0) ** 2
    z_var = max(float(reported_z_sigma_m), 0.0) ** 2
    yaw_var = math.radians(max(float(reported_yaw_sigma_deg), 0.0)) ** 2
    noisy.pose.covariance[0] = xy_var
    noisy.pose.covariance[7] = xy_var
    noisy.pose.covariance[14] = z_var
    noisy.pose.covariance[35] = yaw_var
    return noisy


class CorrelatedFixpositionNoise(Node):
    def __init__(self, **kwargs):
        super().__init__("correlated_fixposition_noise", **kwargs)
        self.declare_parameter("input_odom_topic", "/fixposition/odometry_enu")
        self.declare_parameter("output_odom_topic", "/fixposition/noisy_odometry_enu")
        self.declare_parameter("random_seed", 424242)
        self.declare_parameter("tau_sec", 45.0)
        self.declare_parameter("planar_stddev_m", 3.0)
        self.declare_parameter("white_stddev_m", 0.3)
        self.declare_parameter("z_stddev_m", 0.5)
        self.declare_parameter("yaw_stddev_deg", 1.0)
        self.declare_parameter("reported_xy_sigma_m", 3.0)
        self.declare_parameter("reported_z_sigma_m", 1.5)
        self.declare_parameter("reported_yaw_sigma_deg", 1.0)

        self._model = CorrelatedNoiseModel(
            seed=int(self.get_parameter("random_seed").value),
            tau_sec=float(self.get_parameter("tau_sec").value),
            planar_stddev_m=float(self.get_parameter("planar_stddev_m").value),
            white_stddev_m=float(self.get_parameter("white_stddev_m").value),
            z_stddev_m=float(self.get_parameter("z_stddev_m").value),
            yaw_stddev_deg=float(self.get_parameter("yaw_stddev_deg").value),
        )
        self._reported_xy_sigma_m = float(self.get_parameter("reported_xy_sigma_m").value)
        self._reported_z_sigma_m = float(self.get_parameter("reported_z_sigma_m").value)
        self._reported_yaw_sigma_deg = float(self.get_parameter("reported_yaw_sigma_deg").value)

        self._publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_odom_topic").value,
            10,
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self._on_odom,
            qos,
        )
        self.get_logger().info(
            "Publishing correlated no-RTK Fixposition odometry on "
            f"{self.get_parameter('output_odom_topic').value}"
        )

    def _on_odom(self, msg: Odometry):
        stamp = _message_time(msg, self.get_clock().now())
        sample = self._model.sample(stamp.nanoseconds / 1e9)
        self._publisher.publish(
            apply_planar_noise_to_odometry(
                msg,
                sample,
                reported_xy_sigma_m=self._reported_xy_sigma_m,
                reported_z_sigma_m=self._reported_z_sigma_m,
                reported_yaw_sigma_deg=self._reported_yaw_sigma_deg,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = CorrelatedFixpositionNoise()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
