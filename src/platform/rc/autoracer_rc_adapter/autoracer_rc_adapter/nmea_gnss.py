"""ROS-independent parsing and safety policy for the RC G90 NMEA path."""

from dataclasses import dataclass
import math
from typing import NamedTuple, Optional, Union


class NmeaParseError(ValueError):
    """Raised when a supported NMEA sentence is malformed or corrupt."""


@dataclass(frozen=True)
class Gga:
    quality: int
    satellites: int
    hdop: Optional[float]
    latitude_deg: Optional[float]
    longitude_deg: Optional[float]
    altitude_ellipsoid_m: Optional[float]


@dataclass(frozen=True)
class Gst:
    latitude_stddev_m: float
    longitude_stddev_m: float
    altitude_stddev_m: float


@dataclass(frozen=True)
class GstUnavailable:
    """A checksum-valid GST epoch without a usable covariance estimate."""


@dataclass(frozen=True)
class Hdt:
    heading_true_deg: float


@dataclass(frozen=True)
class Ths:
    heading_true_deg: Optional[float]
    mode: str


ParsedSentence = Union[Gga, Gst, GstUnavailable, Hdt, Ths]


class Quaternion(NamedTuple):
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class OrientationSample:
    stamp: float
    yaw_enu_rad: float
    quaternion: Quaternion


@dataclass(frozen=True)
class FixDecision:
    stamp: float
    quality: int
    satellites: int
    hdop: Optional[float]
    latitude_deg: Optional[float]
    longitude_deg: Optional[float]
    altitude_ellipsoid_m: Optional[float]
    accepted: bool
    reason: str
    orientation: Optional[OrientationSample]
    east_variance_m2: Optional[float]
    north_variance_m2: Optional[float]
    up_variance_m2: Optional[float]


def wrap_angle(angle_rad: float) -> float:
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def heading_to_enu_yaw(heading_true_deg: float, mount_offset_deg: float) -> float:
    """Convert clockwise-from-true-north heading to counterclockwise-from-east yaw."""
    if not math.isfinite(heading_true_deg) or not math.isfinite(mount_offset_deg):
        raise ValueError("heading and mount offset must be finite")
    return wrap_angle(
        math.pi / 2.0
        - math.radians(heading_true_deg)
        + math.radians(mount_offset_deg)
    )


def yaw_quaternion(yaw_rad: float) -> Quaternion:
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw must be finite")
    half_yaw = yaw_rad / 2.0
    return Quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def _finite_timestamp(stamp: float, name: str) -> float:
    value = float(stamp)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _nonnegative(value: float, name: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return numeric


def _positive(value: float, name: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return numeric


class NmeaGnssGate:
    """Accept a complete usable RTK position, heading and covariance epoch."""

    _TIME_EPSILON = 1e-9
    _USABLE_RTK_QUALITIES = {4: "rtk_fixed", 5: "rtk_float"}

    def __init__(
        self,
        *,
        heading_mount_offset_deg: float = 0.0,
        heading_max_age_sec: float = 0.3,
        gst_max_age_sec: float = 0.5,
        fix_timeout_sec: float = 0.5,
    ) -> None:
        if not math.isfinite(float(heading_mount_offset_deg)):
            raise ValueError("heading_mount_offset_deg must be finite")
        self.heading_mount_offset_deg = float(heading_mount_offset_deg)
        self.heading_max_age_sec = _nonnegative(
            heading_max_age_sec, "heading_max_age_sec"
        )
        self.gst_max_age_sec = _nonnegative(gst_max_age_sec, "gst_max_age_sec")
        self.fix_timeout_sec = _positive(fix_timeout_sec, "fix_timeout_sec")

        self._orientation: Optional[OrientationSample] = None
        self._heading_rejection_reason = "heading_missing"
        self._gst: Optional[Gst] = None
        self._gst_stamp: Optional[float] = None
        self._last_gga_stamp: Optional[float] = None
        self._last_decision_accepted = False

    @property
    def orientation(self) -> Optional[OrientationSample]:
        return self._orientation

    @classmethod
    def _is_fresh(cls, sample_stamp: float, reference_stamp: float, max_age: float) -> bool:
        age = reference_stamp - sample_stamp
        return -cls._TIME_EPSILON <= age <= max_age + cls._TIME_EPSILON

    def accept_heading(
        self, heading_true_deg: float, stamp: float
    ) -> OrientationSample:
        sample_stamp = _finite_timestamp(stamp, "heading stamp")
        yaw = heading_to_enu_yaw(
            heading_true_deg, self.heading_mount_offset_deg
        )
        self._orientation = OrientationSample(
            stamp=sample_stamp,
            yaw_enu_rad=yaw,
            quaternion=yaw_quaternion(yaw),
        )
        self._heading_rejection_reason = ""
        return self._orientation

    def reject_heading(self, reason: str) -> None:
        if not reason:
            raise ValueError("heading rejection reason must not be empty")
        self._orientation = None
        self._heading_rejection_reason = reason

    def accept_gst(self, gst: Gst, stamp: float) -> None:
        self._gst = gst
        self._gst_stamp = _finite_timestamp(stamp, "GST stamp")

    def reject_gst(self) -> None:
        """Immediately revoke a previous covariance when GST is unavailable."""
        self._gst = None
        self._gst_stamp = None

    def _rejected(self, gga: Gga, stamp: float, reason: str) -> FixDecision:
        self._last_decision_accepted = False
        return FixDecision(
            stamp=stamp,
            quality=gga.quality,
            satellites=gga.satellites,
            hdop=gga.hdop,
            latitude_deg=gga.latitude_deg,
            longitude_deg=gga.longitude_deg,
            altitude_ellipsoid_m=gga.altitude_ellipsoid_m,
            accepted=False,
            reason=reason,
            orientation=None,
            east_variance_m2=None,
            north_variance_m2=None,
            up_variance_m2=None,
        )

    def accept_gga(self, gga: Gga, stamp: float) -> FixDecision:
        sample_stamp = _finite_timestamp(stamp, "GGA stamp")
        self._last_gga_stamp = sample_stamp

        rtk_state = self._USABLE_RTK_QUALITIES.get(gga.quality)
        if rtk_state is None:
            return self._rejected(gga, sample_stamp, f"quality_{gga.quality}")
        if any(
            value is None
            for value in (
                gga.latitude_deg,
                gga.longitude_deg,
                gga.altitude_ellipsoid_m,
            )
        ):
            return self._rejected(gga, sample_stamp, "position_missing")
        if self._orientation is None:
            return self._rejected(gga, sample_stamp, self._heading_rejection_reason)
        if not self._is_fresh(
            self._orientation.stamp, sample_stamp, self.heading_max_age_sec
        ):
            return self._rejected(gga, sample_stamp, "heading_stale")
        if self._gst is None or self._gst_stamp is None:
            return self._rejected(gga, sample_stamp, "covariance_missing")
        if not self._is_fresh(self._gst_stamp, sample_stamp, self.gst_max_age_sec):
            return self._rejected(gga, sample_stamp, "covariance_stale")

        self._last_decision_accepted = True
        return FixDecision(
            stamp=sample_stamp,
            quality=gga.quality,
            satellites=gga.satellites,
            hdop=gga.hdop,
            latitude_deg=gga.latitude_deg,
            longitude_deg=gga.longitude_deg,
            altitude_ellipsoid_m=gga.altitude_ellipsoid_m,
            accepted=True,
            reason=rtk_state,
            orientation=self._orientation,
            east_variance_m2=self._gst.longitude_stddev_m**2,
            north_variance_m2=self._gst.latitude_stddev_m**2,
            up_variance_m2=self._gst.altitude_stddev_m**2,
        )

    def reject_gga(self, stamp: float, reason: str) -> FixDecision:
        sample_stamp = _finite_timestamp(stamp, "invalid GGA stamp")
        if not reason:
            raise ValueError("invalid GGA reason must not be empty")
        self._last_gga_stamp = sample_stamp
        self._last_decision_accepted = False
        return FixDecision(
            stamp=sample_stamp,
            quality=-1,
            satellites=0,
            hdop=None,
            latitude_deg=None,
            longitude_deg=None,
            altitude_ellipsoid_m=None,
            accepted=False,
            reason=reason,
            orientation=None,
            east_variance_m2=None,
            north_variance_m2=None,
            up_variance_m2=None,
        )

    def expire(self, now: float) -> Optional[FixDecision]:
        current_stamp = _finite_timestamp(now, "current stamp")
        if not self._last_decision_accepted or self._last_gga_stamp is None:
            return None
        age = current_stamp - self._last_gga_stamp
        if age < -self._TIME_EPSILON:
            reason = "clock_reset"
        elif age > self.fix_timeout_sec + self._TIME_EPSILON:
            reason = "timeout"
        else:
            return None
        return self.reject_gga(current_stamp, reason)


def _finite_float(field: str, name: str) -> float:
    try:
        value = float(field)
    except ValueError as error:
        raise NmeaParseError(f"invalid {name}") from error
    if not math.isfinite(value):
        raise NmeaParseError(f"non-finite {name}")
    return value


def _optional_float(field: str, name: str) -> Optional[float]:
    return None if field == "" else _finite_float(field, name)


def _integer(field: str, name: str, default: Optional[int] = None) -> int:
    if field == "" and default is not None:
        return default
    try:
        return int(field)
    except ValueError as error:
        raise NmeaParseError(f"invalid {name}") from error


def _coordinate(field: str, direction: str, degree_digits: int, name: str) -> float:
    if len(field) <= degree_digits:
        raise NmeaParseError(f"missing {name}")
    degrees = _finite_float(field[:degree_digits], f"{name} degrees")
    minutes = _finite_float(field[degree_digits:], f"{name} minutes")
    limit = 90.0 if degree_digits == 2 else 180.0
    if minutes < 0.0 or minutes >= 60.0:
        raise NmeaParseError(f"invalid {name} minutes")
    if degrees < 0.0 or degrees > limit or (degrees == limit and minutes != 0.0):
        raise NmeaParseError(f"invalid {name} degrees")

    positive = "N" if degree_digits == 2 else "E"
    negative = "S" if degree_digits == 2 else "W"
    if direction not in (positive, negative):
        raise NmeaParseError(f"invalid {name} direction")
    value = degrees + minutes / 60.0
    return -value if direction == negative else value


def _validate_checksum(sentence: str) -> str:
    value = sentence.strip()
    if not value.startswith("$") or "*" not in value:
        raise NmeaParseError("missing checksum framing")
    payload, checksum_text = value[1:].rsplit("*", 1)
    if len(checksum_text) != 2:
        raise NmeaParseError("invalid checksum field")
    try:
        expected = int(checksum_text, 16)
        payload_bytes = payload.encode("ascii")
    except (UnicodeEncodeError, ValueError) as error:
        raise NmeaParseError("invalid checksum field") from error

    actual = 0
    for byte in payload_bytes:
        actual ^= byte
    if actual != expected:
        raise NmeaParseError("checksum mismatch")
    return payload


def _parse_gga(fields: list[str]) -> Gga:
    if len(fields) < 13:
        raise NmeaParseError("GGA has too few fields")
    quality = _integer(fields[6], "GGA quality")
    satellites = _integer(fields[7], "satellite count", default=0)
    hdop = _optional_float(fields[8], "HDOP")
    if quality < 0 or satellites < 0 or (hdop is not None and hdop < 0.0):
        raise NmeaParseError("negative GGA status field")

    has_position = all(fields[index] != "" for index in (2, 3, 4, 5, 9, 11))
    if not has_position:
        if quality != 0:
            raise NmeaParseError("position fields missing for valid GGA quality")
        return Gga(quality, satellites, hdop, None, None, None)
    if fields[10] != "M" or fields[12] != "M":
        raise NmeaParseError("unsupported GGA altitude units")

    latitude = _coordinate(fields[2], fields[3], 2, "latitude")
    longitude = _coordinate(fields[4], fields[5], 3, "longitude")
    altitude_msl = _finite_float(fields[9], "mean-sea-level altitude")
    geoid_separation = _finite_float(fields[11], "geoid separation")
    return Gga(
        quality=quality,
        satellites=satellites,
        hdop=hdop,
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_ellipsoid_m=altitude_msl + geoid_separation,
    )


def _parse_gst(fields: list[str]) -> Union[Gst, GstUnavailable]:
    if len(fields) < 9:
        raise NmeaParseError("GST has too few fields")
    uncertainty_fields = fields[6:9]
    if all(field == "" for field in uncertainty_fields):
        return GstUnavailable()
    if any(field == "" for field in uncertainty_fields):
        raise NmeaParseError("incomplete GST standard deviation")
    latitude = _finite_float(fields[6], "GST latitude standard deviation")
    longitude = _finite_float(fields[7], "GST longitude standard deviation")
    altitude = _finite_float(fields[8], "GST altitude standard deviation")
    if min(latitude, longitude, altitude) <= 0.0:
        raise NmeaParseError("non-positive GST standard deviation")
    return Gst(latitude, longitude, altitude)


def _parse_hdt(fields: list[str]) -> Hdt:
    if len(fields) < 3:
        raise NmeaParseError("HDT has too few fields")
    heading = _finite_float(fields[1], "HDT heading")
    if heading < 0.0 or heading > 360.0:
        raise NmeaParseError("HDT heading outside [0, 360]")
    if fields[2] != "T":
        raise NmeaParseError("HDT is not referenced to true north")
    return Hdt(0.0 if heading == 360.0 else heading)


def _parse_ths(fields: list[str]) -> Ths:
    if len(fields) < 3:
        raise NmeaParseError("THS has too few fields")
    mode = fields[2]
    if mode not in {"A", "E", "M", "S", "V"}:
        raise NmeaParseError("unsupported THS mode")
    if fields[1] == "":
        if mode != "V":
            raise NmeaParseError("THS heading missing for available mode")
        return Ths(None, mode)
    heading = _finite_float(fields[1], "THS heading")
    if heading < 0.0 or heading > 360.0:
        raise NmeaParseError("THS heading outside [0, 360]")
    return Ths(0.0 if heading == 360.0 else heading, mode)


def parse_nmea_sentence(sentence: str) -> Optional[ParsedSentence]:
    payload = _validate_checksum(sentence)
    fields = payload.split(",")
    identifier = fields[0]
    allowed_talkers = {"GP", "GN", "GB", "BD", "GL", "GA"}
    if len(identifier) != 5 or identifier[:2] not in allowed_talkers:
        raise NmeaParseError("unsupported NMEA talker or identifier")
    sentence_type = identifier[2:]
    if sentence_type == "GGA":
        return _parse_gga(fields)
    if sentence_type == "GST":
        return _parse_gst(fields)
    if sentence_type == "HDT":
        return _parse_hdt(fields)
    if sentence_type == "THS":
        return _parse_ths(fields)
    return None
