"""Normalize trustworthy G90 NMEA observations at the RC platform boundary."""

from dataclasses import replace
import math
from typing import Optional

from autoware_map_msgs.msg import MapProjectorInfo
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nmea_msgs.msg import Sentence
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformListener

from .nmea_gnss import (
    FixDecision,
    Gga,
    Gst,
    Hdt,
    NmeaGnssGate,
    NmeaParseError,
    OrientationSample,
    Ths,
    parse_nmea_sentence,
)


class G90NmeaAdapter(Node):
    """Expose only G90 data that satisfies the existing Core GNSS contract."""

    _SUPPORTED_PROJECTORS = {
        MapProjectorInfo.LOCAL_CARTESIAN,
        MapProjectorInfo.LOCAL_CARTESIAN_UTM,
        MapProjectorInfo.MGRS,
        MapProjectorInfo.TRANSVERSE_MERCATOR,
    }

    def __init__(self, **kwargs) -> None:
        super().__init__("g90_nmea_adapter", **kwargs)

        self._frame_id = str(self.declare_parameter("frame_id", "gnss_link").value)
        self._base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self._enable_localization_output = bool(
            self.declare_parameter("enable_localization_output", False).value
        )
        self._allow_statusless_hdt = bool(
            self.declare_parameter("allow_statusless_hdt", False).value
        )
        heading_mount_offset_deg = float(
            self.declare_parameter("heading_mount_offset_deg", 0.0).value
        )
        heading_max_age_sec = float(
            self.declare_parameter("heading_max_age_sec", 0.3).value
        )
        gst_max_age_sec = float(self.declare_parameter("gst_max_age_sec", 0.5).value)
        fix_timeout_sec = float(
            self.declare_parameter("fix_timeout_sec", 0.5).value
        )
        self._yaw_stddev_rad = math.radians(
            float(self.declare_parameter("yaw_stddev_deg", 0.0).value)
        )
        self._roll_pitch_stddev_rad = math.radians(
            float(self.declare_parameter("roll_pitch_stddev_deg", 180.0).value)
        )

        if not self._frame_id or not self._base_frame:
            raise ValueError("frame_id and base_frame must not be empty")
        if (
            not math.isfinite(self._roll_pitch_stddev_rad)
            or self._roll_pitch_stddev_rad <= 0.0
        ):
            raise ValueError("roll_pitch_stddev_deg must be finite and positive")

        self._gate = NmeaGnssGate(
            heading_mount_offset_deg=heading_mount_offset_deg,
            heading_max_age_sec=heading_max_age_sec,
            gst_max_age_sec=gst_max_age_sec,
            fix_timeout_sec=fix_timeout_sec,
        )
        self._last_receiver_decision: Optional[FixDecision] = None
        self._last_effective_decision: Optional[FixDecision] = None
        self._last_gga_stamp: Optional[float] = None
        self._last_heading_stamp: Optional[float] = None
        self._last_heading_source = "none"
        self._last_heading_mode = "unknown"
        self._last_gst_stamp: Optional[float] = None
        self._map_projector_type: Optional[str] = None
        self._invalid_sentence_count = 0
        self._ignored_sentence_count = 0
        self._last_parse_error = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._orientation_publisher = self.create_publisher(
            GnssInsOrientationStamped, "autoware_orientation", 10
        )
        self._fix_publisher = self.create_publisher(NavSatFix, "fix", 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._sentence_subscription = self.create_subscription(
            Sentence, "nmea_sentence", self._on_sentence, 10
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            MapProjectorInfo,
            "/map/map_projector_info",
            self._on_map_projector_info,
            map_qos,
        )
        watchdog_period_sec = min(0.1, fix_timeout_sec / 2.0)
        self._watchdog_timer = self.create_timer(
            watchdog_period_sec, self._on_watchdog
        )
        self._diagnostic_timer = self.create_timer(
            1.0, self._on_diagnostics_timer
        )

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _set_stamp(message_stamp, stamp: float) -> None:
        stamp_nanoseconds = round(stamp * 1e9)
        message_stamp.sec = stamp_nanoseconds // 1_000_000_000
        message_stamp.nanosec = stamp_nanoseconds % 1_000_000_000

    @staticmethod
    def _position(value: Optional[float]) -> float:
        return math.nan if value is None else value

    @staticmethod
    def _is_gga_sentence(sentence: str) -> bool:
        value = sentence.lstrip()
        return len(value) >= 6 and value.startswith("$") and value[3:6] == "GGA"

    def _on_map_projector_info(self, message: MapProjectorInfo) -> None:
        self._map_projector_type = message.projector_type

    def _tf_ready(self) -> bool:
        if self._frame_id == self._base_frame:
            return True
        try:
            return self._tf_buffer.can_transform(
                self._frame_id, self._base_frame, Time()
            )
        except Exception:  # noqa: BLE001 - TF readiness must fail closed.
            return False

    def _runtime_prerequisite_reason(self) -> Optional[str]:
        if not self._enable_localization_output:
            return "localization_output_disabled"
        if not math.isfinite(self._yaw_stddev_rad) or self._yaw_stddev_rad <= 0.0:
            return "yaw_covariance_unconfigured"
        if self._map_projector_type is None:
            return "map_projector_missing"
        if self._map_projector_type == "Local":
            return "map_projector_local"
        if self._map_projector_type not in self._SUPPORTED_PROJECTORS:
            return "map_projector_unsupported"
        if not self._tf_ready():
            return "base_transform_missing"
        return None

    def _make_orientation(
        self, sample: OrientationSample
    ) -> GnssInsOrientationStamped:
        message = GnssInsOrientationStamped()
        self._set_stamp(message.header.stamp, sample.stamp)
        message.header.frame_id = self._frame_id
        message.orientation.orientation.x = sample.quaternion.x
        message.orientation.orientation.y = sample.quaternion.y
        message.orientation.orientation.z = sample.quaternion.z
        message.orientation.orientation.w = sample.quaternion.w
        message.orientation.rmse_rotation_x = self._roll_pitch_stddev_rad
        message.orientation.rmse_rotation_y = self._roll_pitch_stddev_rad
        message.orientation.rmse_rotation_z = self._yaw_stddev_rad
        return message

    def _make_fix(self, decision: FixDecision) -> NavSatFix:
        message = NavSatFix()
        self._set_stamp(message.header.stamp, decision.stamp)
        message.header.frame_id = self._frame_id
        message.status.status = (
            NavSatStatus.STATUS_GBAS_FIX
            if decision.accepted
            else NavSatStatus.STATUS_NO_FIX
        )
        message.status.service = 0
        message.latitude = self._position(decision.latitude_deg)
        message.longitude = self._position(decision.longitude_deg)
        message.altitude = self._position(decision.altitude_ellipsoid_m)
        if decision.accepted:
            message.position_covariance[0] = float(decision.east_variance_m2)
            message.position_covariance[4] = float(decision.north_variance_m2)
            message.position_covariance[8] = float(decision.up_variance_m2)
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return message

    @staticmethod
    def _runtime_rejection(decision: FixDecision, reason: str) -> FixDecision:
        return replace(
            decision,
            accepted=False,
            reason=reason,
            orientation=None,
            east_variance_m2=None,
            north_variance_m2=None,
            up_variance_m2=None,
        )

    def _publish_receiver_decision(self, decision: FixDecision) -> None:
        self._last_receiver_decision = decision
        effective = decision
        if decision.accepted:
            runtime_reason = self._runtime_prerequisite_reason()
            if runtime_reason is not None:
                effective = self._runtime_rejection(decision, runtime_reason)

        if effective.accepted:
            if effective.orientation is None:
                raise RuntimeError("accepted G90 decision has no orientation")
            self._orientation_publisher.publish(
                self._make_orientation(effective.orientation)
            )
        self._fix_publisher.publish(self._make_fix(effective))
        self._last_effective_decision = effective

    def _on_sentence(self, message: Sentence) -> None:
        stamp = self._stamp_to_seconds(message.header.stamp)
        try:
            parsed = parse_nmea_sentence(message.sentence)
        except NmeaParseError as error:
            self._invalid_sentence_count += 1
            self._last_parse_error = str(error)
            self.get_logger().warning(f"Rejected invalid G90 NMEA sentence: {error}")
            if self._is_gga_sentence(message.sentence):
                self._last_gga_stamp = stamp
                self._publish_receiver_decision(
                    self._gate.reject_gga(stamp=stamp, reason="invalid_gga")
                )
            self._publish_diagnostics(stamp)
            return

        if isinstance(parsed, Hdt):
            self._last_heading_stamp = stamp
            self._last_heading_source = "HDT"
            self._last_heading_mode = "statusless"
            if self._allow_statusless_hdt:
                self._gate.accept_heading(parsed.heading_true_deg, stamp)
            else:
                self._gate.reject_heading("heading_status_unavailable")
        elif isinstance(parsed, Ths):
            self._last_heading_stamp = stamp
            self._last_heading_source = "THS"
            self._last_heading_mode = parsed.mode
            if parsed.mode == "A" and parsed.heading_true_deg is not None:
                self._gate.accept_heading(parsed.heading_true_deg, stamp)
            else:
                self._gate.reject_heading(f"heading_mode_{parsed.mode.lower()}")
        elif isinstance(parsed, Gst):
            self._gate.accept_gst(parsed, stamp)
            self._last_gst_stamp = stamp
        elif isinstance(parsed, Gga):
            self._last_gga_stamp = stamp
            self._last_parse_error = ""
            self._publish_receiver_decision(self._gate.accept_gga(parsed, stamp))
            self._publish_diagnostics(stamp)
        else:
            self._ignored_sentence_count += 1

    def _on_watchdog(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        expired = self._gate.expire(now_sec)
        if expired is not None:
            self._publish_receiver_decision(expired)
            self._publish_diagnostics(now_sec)
            return

        if self._last_effective_decision and self._last_effective_decision.accepted:
            runtime_reason = self._runtime_prerequisite_reason()
            if runtime_reason is not None:
                rejected = self._runtime_rejection(
                    replace(self._last_effective_decision, stamp=now_sec), runtime_reason
                )
                self._fix_publisher.publish(self._make_fix(rejected))
                self._last_effective_decision = rejected
                self._publish_diagnostics(now_sec)

    def _on_diagnostics_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._publish_diagnostics(now_sec)

    @staticmethod
    def _age_text(sample_stamp: Optional[float], reference_stamp: float) -> str:
        if sample_stamp is None:
            return "unknown"
        age = reference_stamp - sample_stamp
        return "clock_reset" if age < 0.0 else f"{age:.3f}"

    def _publish_diagnostics(self, stamp: float) -> None:
        message = DiagnosticArray()
        self._set_stamp(message.header.stamp, stamp)
        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}: G90 GNSS boundary"
        status.hardware_id = "G90"

        effective = self._last_effective_decision
        receiver = self._last_receiver_decision
        if effective is not None and effective.accepted:
            status.level = DiagnosticStatus.OK
            status.message = "RTK Fixed accepted by the Core GNSS boundary"
        elif self._last_parse_error:
            status.level = DiagnosticStatus.WARN
            status.message = f"Invalid NMEA: {self._last_parse_error}"
        elif effective is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Waiting for GGA"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = effective.reason

        runtime_reason = self._runtime_prerequisite_reason()
        values = {
            "receiver_rtk_fixed": str(bool(receiver and receiver.accepted)).lower(),
            "core_boundary_fixed": str(bool(effective and effective.accepted)).lower(),
            "reason": "waiting" if effective is None else effective.reason,
            "runtime_prerequisite": runtime_reason or "ready",
            "localization_output_enabled": str(self._enable_localization_output).lower(),
            "allow_statusless_hdt": str(self._allow_statusless_hdt).lower(),
            "map_projector_type": self._map_projector_type or "unknown",
            "tf_ready": str(self._tf_ready()).lower(),
            "gga_quality": "unknown" if receiver is None else str(receiver.quality),
            "satellites": "unknown" if receiver is None else str(receiver.satellites),
            "hdop": (
                "unknown"
                if receiver is None or receiver.hdop is None
                else str(receiver.hdop)
            ),
            "last_gga_age_sec": self._age_text(self._last_gga_stamp, stamp),
            "heading_source": self._last_heading_source,
            "heading_mode": self._last_heading_mode,
            "heading_age_sec": self._age_text(self._last_heading_stamp, stamp),
            "gst_age_sec": self._age_text(self._last_gst_stamp, stamp),
            "invalid_sentence_count": str(self._invalid_sentence_count),
            "ignored_sentence_count": str(self._ignored_sentence_count),
            "last_parse_error": self._last_parse_error or "none",
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message.status = [status]
        self._diagnostic_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = G90NmeaAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
