#!/usr/bin/env python3
"""Bridge NDT pose and honest 5 m GNSS into one EKF pose-measurement stream.

This node is intentionally conservative:
- NDT is always preferred when it is recent.
- GNSS is used only during NDT gaps.
- GNSS covariance is never made better than the configured weak covariance.
It does not publish final localization and must not be treated as GNSS fallback.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from .pose_stream_qos import latest_pose_qos


@dataclass(frozen=True)
class BridgeDecision:
    source: str
    reason: str


def clamp_gnss_covariance(
    msg: PoseWithCovarianceStamped, *, min_xy_variance: float, yaw_variance: float
) -> PoseWithCovarianceStamped:
    out = PoseWithCovarianceStamped()
    out.header = msg.header
    out.pose.pose = msg.pose.pose
    out.pose.covariance = list(msg.pose.covariance)
    out.pose.covariance[0] = max(float(out.pose.covariance[0]), float(min_xy_variance))
    out.pose.covariance[7] = max(float(out.pose.covariance[7]), float(min_xy_variance))
    out.pose.covariance[35] = max(float(out.pose.covariance[35]), float(yaw_variance))
    return out


def choose_bridge_source(
    *, now_sec: float, last_ndt_sec: float | None, ndt_gap_threshold_sec: float
) -> BridgeDecision:
    if last_ndt_sec is None:
        return BridgeDecision("gnss", "no_ndt_yet")
    gap = now_sec - last_ndt_sec
    if math.isfinite(gap) and gap <= ndt_gap_threshold_sec:
        return BridgeDecision("ndt", "ndt_recent")
    return BridgeDecision("gnss", "ndt_gap")


class Gnss5mWeakPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("gnss5m_weak_pose_bridge")
        self.declare_parameter("ndt_pose_topic", "/localization/pose_estimator/pose_with_covariance")
        self.declare_parameter("gnss_pose_topic", "/sensing/gnss/pose_with_covariance")
        self.declare_parameter(
            "output_pose_topic", "/localization/gnss5m_weak_pose_bridge/pose_with_covariance"
        )
        self.declare_parameter("ndt_gap_threshold_sec", 0.7)
        self.declare_parameter("gnss_max_age_sec", 0.5)
        self.declare_parameter("gnss_min_xy_variance", 25.0)
        self.declare_parameter("gnss_yaw_variance", 999.0)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._ndt_gap_threshold_sec = float(self.get_parameter("ndt_gap_threshold_sec").value)
        self._gnss_max_age_sec = float(self.get_parameter("gnss_max_age_sec").value)
        self._gnss_min_xy_variance = float(self.get_parameter("gnss_min_xy_variance").value)
        self._gnss_yaw_variance = float(self.get_parameter("gnss_yaw_variance").value)

        self._latest_ndt: PoseWithCovarianceStamped | None = None
        self._latest_ndt_sec: float | None = None
        self._latest_gnss: PoseWithCovarianceStamped | None = None
        self._latest_gnss_sec: float | None = None

        output_topic = str(self.get_parameter("output_pose_topic").value)
        self._pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self._diag_pub = self.create_publisher(
            PoseWithCovarianceStamped, f"{output_topic}/selected_input", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("ndt_pose_topic").value),
            self._on_ndt,
            latest_pose_qos(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("gnss_pose_topic").value),
            self._on_gnss,
            latest_pose_qos(),
        )
        period = 1.0 / max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self._on_timer)

    @staticmethod
    def _stamp_sec(msg: PoseWithCovarianceStamped) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9

    def _now_sec(self) -> float:
        now = self.get_clock().now()
        return float(now.nanoseconds) * 1.0e-9

    def _on_ndt(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_ndt = msg
        self._latest_ndt_sec = self._stamp_sec(msg)
        self._pub.publish(msg)

    def _on_gnss(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_gnss = clamp_gnss_covariance(
            msg,
            min_xy_variance=self._gnss_min_xy_variance,
            yaw_variance=self._gnss_yaw_variance,
        )
        self._latest_gnss_sec = self._stamp_sec(msg)

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        decision = choose_bridge_source(
            now_sec=now_sec,
            last_ndt_sec=self._latest_ndt_sec,
            ndt_gap_threshold_sec=self._ndt_gap_threshold_sec,
        )
        if decision.source == "ndt":
            return
        if self._latest_gnss is None or self._latest_gnss_sec is None:
            return
        if abs(now_sec - self._latest_gnss_sec) > self._gnss_max_age_sec:
            return
        self._pub.publish(self._latest_gnss)
        self._diag_pub.publish(self._latest_gnss)


def main() -> None:
    rclpy.init()
    node = Gnss5mWeakPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
