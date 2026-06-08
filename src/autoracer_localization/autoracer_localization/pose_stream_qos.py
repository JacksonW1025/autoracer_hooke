from rclpy.qos import QoSProfile, ReliabilityPolicy


def latest_pose_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return qos
