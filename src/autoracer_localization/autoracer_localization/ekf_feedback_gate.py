import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .ndt_initial_pose_predictor import _message_time


def feedback_pose_is_measurement_backed(ekf_stamp, measurement_stamp, max_age_sec):
    if measurement_stamp is None:
        return False
    if max_age_sec <= 0.0:
        return True
    age_sec = abs((ekf_stamp - measurement_stamp).nanoseconds / 1e9)
    return age_sec <= float(max_age_sec)


class EkfFeedbackGate(Node):
    def __init__(self):
        super().__init__("ekf_feedback_gate")
        self.declare_parameter("ekf_pose_topic", "/localization/ekf/pose_with_covariance")
        self.declare_parameter(
            "measurement_pose_topic", "/localization/pose_with_covariance"
        )
        self.declare_parameter(
            "output_topic", "/localization/ndt/accepted_pose_with_covariance"
        )
        self.declare_parameter("max_measurement_age_sec", 0.2)

        self._max_measurement_age_sec = float(
            self.get_parameter("max_measurement_age_sec").value
        )
        self._last_measurement_stamp = None
        self._last_published_measurement_stamp_ns = None
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("measurement_pose_topic").value,
            self._on_measurement_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("ekf_pose_topic").value,
            self._on_ekf_pose,
            10,
        )

    def _on_measurement_pose(self, msg):
        self._last_measurement_stamp = _message_time(msg, self.get_clock().now())

    def _on_ekf_pose(self, msg):
        ekf_stamp = _message_time(msg, self.get_clock().now())
        measurement_stamp_ns = (
            None
            if self._last_measurement_stamp is None
            else self._last_measurement_stamp.nanoseconds
        )
        if measurement_stamp_ns == self._last_published_measurement_stamp_ns:
            return
        if feedback_pose_is_measurement_backed(
            ekf_stamp,
            self._last_measurement_stamp,
            self._max_measurement_age_sec,
        ):
            self._publisher.publish(msg)
            self._last_published_measurement_stamp_ns = measurement_stamp_ns


def main(args=None):
    rclpy.init(args=args)
    node = EkfFeedbackGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
