#!/usr/bin/env python3
"""Fail-closed outdoor preflight for an RC mapping rosbag session."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import tty
import unicodedata
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage


WINDOW_SECONDS = 5.0
READINESS_STABILITY_SECONDS = 0.0
PROGRESS_HEARTBEAT_SECONDS = 5.0
LIDAR_PROBE_STOP_TIMEOUT_SECONDS = 30.0
LIDAR_PROBE_CACHE_BYTES = 256 * 1024 * 1024

LIDAR_TOPIC = "/sensing/lidar/raw/pointcloud"
IMU_TOPIC = "/sensing/imu/raw/imu_data"
NMEA_TOPIC = "/g90/raw/nmea_sentence"
TF_STATIC_TOPIC = "/tf_static"
DIAGNOSTICS_TOPIC = "/diagnostics"

LIDAR_FRAME = "lidar_top"
IMU_FRAME = "imu_link"
GNSS_FRAME = "gnss_link"
REQUIRED_LIDAR_FIELDS = {"x", "y", "z", "intensity", "ring"}
REQUIRED_STATIC_TRANSFORMS = {
    ("base_link", "lidar_top"),
    ("base_link", "imu_link"),
    ("base_link", "gnss_link"),
}

MINIMUM_RATE_HZ = {"lidar": 18.0, "imu": 90.0, "GGA": 9.5, "GST": 9.5, "THS": 9.5}
MAXIMUM_GAP_SECONDS = {
    "lidar": 0.20,
    "imu": 0.10,
    "GGA": 0.30,
    "GST": 0.30,
    "THS": 0.30,
}
MAXIMUM_DIFFERENTIAL_AGE_SECONDS = 2.0
ACCEPTED_GGA_QUALITIES = frozenset({4, 5})
ACQUISITION_FRESHNESS_SECONDS = {
    "lidar": 0.5,
    "imu": 0.2,
    "nmea": 0.5,
    "GGA": 0.5,
    "GST": 0.5,
    "THS": 0.5,
    "COM2": 2.5,
}


def g90_com2_ready(values: dict[str, str]) -> bool:
    """Return whether the project-owned correction path is usable now."""

    return (
        values.get("worker_alive") == "true"
        and values.get("serial_open") == "true"
        and values.get("caster_connected") == "true"
        and values.get("rtcm_fresh") == "true"
        and values.get("credential_expired") == "false"
    )


def diagnostic_level(value: Any) -> int:
    """Normalize ROS ``byte`` fields across generated Python representations."""

    if isinstance(value, (bytes, bytearray, memoryview)):
        encoded = bytes(value)
        if len(encoded) != 1:
            raise ValueError("diagnostic level byte must contain exactly one octet")
        return encoded[0]
    return int(value)


def g90_com2_summary(values: dict[str, str]) -> tuple[str, str]:
    """Return a concise state/detail pair without exposing private NTRIP data."""

    if not values:
        return "等待", "尚未收到差分节点状态"
    if values.get("credential_expired") == "true":
        return "故障", "NTRIP 凭据已过期"
    if values.get("worker_alive") != "true":
        return "故障", "差分 relay 未运行"
    if values.get("serial_open") != "true":
        return "故障", "COM2 串口未打开"
    if values.get("caster_connected") != "true":
        return "等待", "COM2 已打开，CORS 尚未连接"
    if values.get("rtcm_fresh") != "true":
        return "等待", "COM2/CORS 已连接，尚无新鲜 RTCM"
    return "正常", "新鲜 RTCM 正在写入 COM2"


StatusItem = tuple[str, str, str]
DEFAULT_TERMINAL_COLUMNS = 80
MINIMUM_TERMINAL_COLUMNS = 20


def display_width(text: str) -> int:
    """Return terminal cell width, including double-width Chinese characters."""

    width = 0
    for character in text:
        if unicodedata.combining(character):
            continue
        width += 2 if unicodedata.east_asian_width(character) in {"F", "W"} else 1
    return width


def terminal_columns() -> int:
    """Read the controlling terminal width even though stdout is piped to tee."""

    if sys.stdin.isatty():
        try:
            columns = os.get_terminal_size(sys.stdin.fileno()).columns
        except OSError:
            columns = DEFAULT_TERMINAL_COLUMNS
    else:
        columns = DEFAULT_TERMINAL_COLUMNS
    return max(MINIMUM_TERMINAL_COLUMNS, columns)


def split_display_lines(text: str, columns: int) -> tuple[str, ...]:
    """Split text without producing a line wider than the terminal."""

    columns = max(1, columns)
    remaining = text.strip()
    if not remaining:
        return ("",)
    lines: list[str] = []
    while display_width(remaining) > columns:
        prefix_width = 0
        cutoff = 0
        for index, character in enumerate(remaining):
            character_width = display_width(character)
            if cutoff and prefix_width + character_width > columns:
                break
            prefix_width += character_width
            cutoff = index + 1
        cutoff = max(1, cutoff)
        prefix = remaining[:cutoff]
        break_at = prefix.rfind(" ")
        if break_at > 0:
            lines.append(prefix[:break_at].rstrip())
            remaining = remaining[break_at + 1 :].lstrip()
        else:
            lines.append(prefix.rstrip())
            remaining = remaining[cutoff:].lstrip()
    lines.append(remaining.rstrip())
    return tuple(lines)


def pad_display(text: str, width: int) -> str:
    return text + " " * max(0, width - display_width(text))


def render_status_rows(items: tuple[StatusItem, ...], columns: int) -> tuple[str, ...]:
    """Render one device per row, wrapping details to the current terminal width."""

    if not items:
        return ()
    columns = max(MINIMUM_TERMINAL_COLUMNS, columns)
    label_width = max(display_width(label) for label, _, _ in items)
    rendered: list[str] = []
    for label, state, detail in items:
        prefix = f"{pad_display(label, label_width)}  {state}  "
        prefix_width = display_width(prefix)
        if columns - prefix_width < 8:
            rendered.extend(split_display_lines(f"{label}  {state}", columns))
            detail_prefix = "  "
            detail_width = columns - display_width(detail_prefix)
            rendered.extend(
                detail_prefix + line
                for line in split_display_lines(detail, detail_width)
            )
            continue

        detail_lines = split_display_lines(detail, columns - prefix_width)
        rendered.append(prefix + detail_lines[0])
        continuation = " " * prefix_width
        rendered.extend(continuation + line for line in detail_lines[1:])
    return tuple(rendered)


def changed_status_items(
    previous: tuple[StatusItem, ...], current: tuple[StatusItem, ...]
) -> tuple[StatusItem, ...]:
    previous_by_label = {label: (state, detail) for label, state, detail in previous}
    return tuple(
        item
        for item in current
        if previous_by_label.get(item[0]) != (item[1], item[2])
    )


def readiness_summary(items: tuple[StatusItem, ...]) -> str:
    faults = [label for label, state, _ in items if state == "故障"]
    waiting = [label for label, state, _ in items if state not in {"正常", "故障"}]
    parts: list[str] = []
    if faults:
        parts.append("故障：" + "、".join(faults))
    if waiting:
        parts.append("等待：" + "、".join(waiting))
    return "；".join(parts) if parts else "全部输入在线"


def print_wrapped(text: str) -> None:
    for line in split_display_lines(text, terminal_columns()):
        print(line, flush=True)


class OperatorCancelled(Exception):
    """Raised when the operator presses Q before mapping reaches READY."""


class OperatorCancelInput:
    """Read a single Q from the controlling terminal without blocking ROS."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._settings: list[Any] | None = None

    def enable(self) -> None:
        if not sys.stdin.isatty():
            return
        self._fd = sys.stdin.fileno()
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def requested(self) -> bool:
        if self._fd is None:
            return False
        readable, _, _ = select.select([self._fd], [], [], 0.0)
        if not readable:
            return False
        return os.read(self._fd, 1).lower() == b"q"

    def restore(self) -> None:
        if self._fd is not None and self._settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
        self._fd = None
        self._settings = None


def ensure_process_alive(process_id: int) -> None:
    try:
        os.kill(process_id, 0)
    except (ProcessLookupError, PermissionError) as error:
        raise RuntimeError(
            f"watched sensing process {process_id} exited or is inaccessible"
        ) from error


def now_iso8601() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def normalize_frame(frame: str) -> str:
    return frame.lstrip("/")


def sample_rate(times: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    elapsed = times[-1] - times[0]
    return 0.0 if elapsed <= 0.0 else (len(times) - 1) / elapsed


def maximum_gap(times: list[float]) -> float | None:
    if len(times) < 2:
        return None
    return max(second - first for first, second in zip(times, times[1:]))


class CdrReader:
    """Read the small PointCloud2 metadata prefix without materializing point data."""

    def __init__(self, serialized: bytes | bytearray | memoryview) -> None:
        self.data = memoryview(serialized).cast("B")
        if len(self.data) < 4:
            raise ValueError("serialized PointCloud2 is shorter than the CDR header")
        representation = bytes(self.data[:2])
        if representation == b"\x00\x01":
            self.endian = "<"
        elif representation == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError(
                f"unsupported PointCloud2 CDR representation 0x{representation.hex()}"
            )
        self.offset = 4
        self.alignment_origin = 4

    def align(self, alignment: int) -> None:
        relative = self.offset - self.alignment_origin
        self.offset += (-relative) % alignment
        if self.offset > len(self.data):
            raise ValueError("serialized PointCloud2 ends in CDR alignment padding")

    def unpack(self, format_code: str, size: int, alignment: int) -> int:
        self.align(alignment)
        end = self.offset + size
        if end > len(self.data):
            raise ValueError("serialized PointCloud2 metadata is truncated")
        value = struct.unpack_from(self.endian + format_code, self.data, self.offset)[0]
        self.offset = end
        return int(value)

    def uint8(self) -> int:
        return self.unpack("B", 1, 1)

    def int32(self) -> int:
        return self.unpack("i", 4, 4)

    def uint32(self) -> int:
        return self.unpack("I", 4, 4)

    def string(self) -> str:
        length = self.uint32()
        if length == 0:
            raise ValueError("serialized PointCloud2 contains a zero-length CDR string")
        end = self.offset + length
        if end > len(self.data):
            raise ValueError("serialized PointCloud2 contains a truncated CDR string")
        encoded = self.data[self.offset:end]
        self.offset = end
        if encoded[-1] != 0:
            raise ValueError("serialized PointCloud2 CDR string is not NUL terminated")
        try:
            return bytes(encoded[:-1]).decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("serialized PointCloud2 contains invalid UTF-8 metadata") from error

    def skip(self, size: int) -> None:
        if size < 0 or self.offset + size > len(self.data):
            raise ValueError("serialized PointCloud2 data payload is truncated")
        self.offset += size


def pointcloud2_metadata_from_cdr(
    serialized: bytes | bytearray | memoryview,
) -> tuple[int, str, set[str]]:
    """Return stamp, frame and field names while skipping the large point payload."""

    reader = CdrReader(serialized)
    seconds = reader.int32()
    nanoseconds = reader.uint32()
    if nanoseconds >= 1_000_000_000:
        raise ValueError("serialized PointCloud2 has an invalid nanosecond stamp")
    frame = reader.string()
    reader.uint32()  # height
    reader.uint32()  # width
    field_count = reader.uint32()
    if field_count > 1024:
        raise ValueError("serialized PointCloud2 has an unreasonable field count")
    fields: set[str] = set()
    for _ in range(field_count):
        fields.add(reader.string())
        reader.uint32()  # offset
        reader.uint8()  # datatype
        reader.uint32()  # count
    reader.uint8()  # is_bigendian
    reader.uint32()  # point_step
    reader.uint32()  # row_step
    payload_size = reader.uint32()
    reader.skip(payload_size)
    reader.uint8()  # is_dense
    return seconds * 1_000_000_000 + nanoseconds, frame, fields


class LidarProbeRecorder:
    """Run the same C++ rosbag2 path used by the formal recorder."""

    def __init__(self, bag_directory: Path, qos_file: Path, log_path: Path) -> None:
        self.bag_directory = bag_directory
        self.qos_file = qos_file
        self.log_path = log_path
        self.process: subprocess.Popen[bytes] | None = None
        self.log_stream: Any = None
        self.stop_escalation = "none"
        self.command: list[str] = []

    def start(self) -> None:
        if self.bag_directory.exists():
            raise RuntimeError(f"LiDAR probe directory already exists: {self.bag_directory}")
        self.bag_directory.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_stream = self.log_path.open("xb")
        self.command = [
            "ros2",
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--output",
            str(self.bag_directory),
            "--max-cache-size",
            str(LIDAR_PROBE_CACHE_BYTES),
            "--compression-mode",
            "none",
            "--qos-profile-overrides-path",
            str(self.qos_file),
            LIDAR_TOPIC,
        ]
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=self.log_stream,
                stderr=subprocess.STDOUT,
            )
        except Exception:
            self.log_stream.close()
            self.log_stream = None
            raise

    def ensure_running(self) -> None:
        if self.process is None:
            raise RuntimeError("LiDAR probe recorder was not started")
        return_code = self.process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"LiDAR probe recorder exited before the fixed window (code {return_code})"
            )

    def subscription_ready(self) -> bool:
        """Return true only after rosbag2 confirms the requested topic subscription."""
        try:
            log_text = self.log_path.read_text(encoding="utf-8", errors="replace")
        except FileNotFoundError:
            return False
        return f"Subscribed to topic '{LIDAR_TOPIC}'" in log_text

    def stop(self) -> int | None:
        if self.process is None:
            return None
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=LIDAR_PROBE_STOP_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                self.stop_escalation = "sigterm"
                self.process.terminate()
                try:
                    self.process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    self.stop_escalation = "sigkill"
                    self.process.kill()
                    self.process.wait(timeout=5.0)
        return_code = self.process.returncode
        if self.log_stream is not None:
            self.log_stream.close()
            self.log_stream = None
        return return_code


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def analyze_lidar_probe(
    bag_directory: Path,
    *,
    window_start_epoch_ns: int,
    window_end_epoch_ns: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Analyze every C++-recorded LiDAR sample inside the fixed wall-clock window."""

    import rosbag2_py

    if window_end_epoch_ns <= window_start_epoch_ns:
        raise RuntimeError("LiDAR probe fixed window has a non-positive duration")
    metadata_path = bag_directory / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError("LiDAR probe did not seal metadata.yaml")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    reception_times: list[float] = []
    source_stamps: list[int] = []
    frames: set[str] = set()
    field_sets: set[tuple[str, ...]] = set()
    metadata_parse_errors: list[str] = []
    metadata_parse_error_count = 0
    field_failures = 0
    total_messages = 0
    while reader.has_next():
        topic, serialized, receipt_ns = reader.read_next()
        if topic != LIDAR_TOPIC:
            continue
        total_messages += 1
        if not window_start_epoch_ns <= receipt_ns <= window_end_epoch_ns:
            continue
        reception_times.append(receipt_ns / 1_000_000_000.0)
        try:
            stamp, raw_frame, fields = pointcloud2_metadata_from_cdr(serialized)
        except ValueError as error:
            metadata_parse_error_count += 1
            if len(metadata_parse_errors) < 20:
                metadata_parse_errors.append(str(error))
            continue
        source_stamps.append(stamp)
        frames.add(normalize_frame(raw_frame))
        field_sets.add(tuple(sorted(fields)))
        if not REQUIRED_LIDAR_FIELDS <= fields:
            field_failures += 1

    artifacts = []
    for path in sorted(item for item in bag_directory.rglob("*") if item.is_file()):
        artifacts.append(
            {
                "path": str(path.relative_to(bag_directory.parent)),
                "size_bytes": path.stat().st_size,
                "sha256": file_sha256(path),
            }
        )

    state = {
        "reception_times": reception_times,
        "source_stamps": source_stamps,
        "frames": frames,
        "field_sets": field_sets,
        "metadata_parse_errors": metadata_parse_errors,
        "metadata_parse_error_count": metadata_parse_error_count,
        "field_failures": field_failures,
    }
    source_times = [value / 1_000_000_000.0 for value in source_stamps]
    evidence = {
        "bag_directory": str(bag_directory),
        "window_start_epoch_ns": window_start_epoch_ns,
        "window_end_epoch_ns": window_end_epoch_ns,
        "window_duration_seconds": (
            window_end_epoch_ns - window_start_epoch_ns
        )
        / 1_000_000_000.0,
        "total_recorded_messages": total_messages,
        "window_messages": len(reception_times),
        "bag_receipt_rate_hz": sample_rate(reception_times),
        "bag_receipt_maximum_gap_seconds": maximum_gap(reception_times),
        "source_stamp_rate_hz": sample_rate(source_times),
        "source_stamp_maximum_gap_seconds": maximum_gap(source_times),
        "metadata_parse_error_count": metadata_parse_error_count,
        "artifacts": artifacts,
    }
    return state, evidence


def validate_checksum(sentence: str) -> list[str]:
    value = sentence.strip()
    if not value.startswith("$") or "*" not in value:
        raise ValueError("missing checksum framing")
    payload, checksum_text = value[1:].rsplit("*", 1)
    if len(checksum_text) != 2:
        raise ValueError("invalid checksum field")
    try:
        expected = int(checksum_text, 16)
        payload_bytes = payload.encode("ascii")
    except (UnicodeEncodeError, ValueError) as error:
        raise ValueError("invalid checksum field") from error
    actual = 0
    for byte in payload_bytes:
        actual ^= byte
    if actual != expected:
        raise ValueError("checksum mismatch")
    fields = payload.split(",")
    identifier = fields[0]
    if len(identifier) != 5 or identifier[:2] not in {
        "GP",
        "GN",
        "GB",
        "BD",
        "GL",
        "GA",
    }:
        raise ValueError("unsupported NMEA talker or identifier")
    return fields


def finite_float(field: str, name: str) -> float:
    try:
        value = float(field)
    except ValueError as error:
        raise ValueError(f"invalid {name}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def coordinate(field: str, direction: str, degree_digits: int, name: str) -> float:
    if len(field) <= degree_digits:
        raise ValueError(f"missing {name}")
    degrees = finite_float(field[:degree_digits], f"{name} degrees")
    minutes = finite_float(field[degree_digits:], f"{name} minutes")
    limit = 90.0 if degree_digits == 2 else 180.0
    if minutes < 0.0 or minutes >= 60.0:
        raise ValueError(f"invalid {name} minutes")
    if degrees < 0.0 or degrees > limit or (degrees == limit and minutes != 0.0):
        raise ValueError(f"invalid {name} degrees")
    positive, negative = (("N", "S") if degree_digits == 2 else ("E", "W"))
    if direction not in {positive, negative}:
        raise ValueError(f"invalid {name} direction")
    result = degrees + minutes / 60.0
    return -result if direction == negative else result


def parse_mapping_sentence(sentence: str) -> tuple[str, dict[str, Any]]:
    fields = validate_checksum(sentence)
    formatter = fields[0][2:]
    if formatter == "GGA":
        if len(fields) < 15:
            raise ValueError("GGA has too few fields")
        quality = int(fields[6])
        if quality not in ACCEPTED_GGA_QUALITIES:
            return formatter, {
                "quality": quality,
                "accepted": False,
                "reason": "GGA quality outside the explicit recording policy",
            }
        latitude = coordinate(fields[2], fields[3], 2, "latitude")
        longitude = coordinate(fields[4], fields[5], 3, "longitude")
        if fields[10] != "M" or fields[12] != "M":
            raise ValueError("unsupported GGA altitude units")
        altitude_ellipsoid = finite_float(fields[9], "MSL altitude") + finite_float(
            fields[11], "geoid separation"
        )
        if fields[13] == "" or fields[14] == "":
            return formatter, {
                "quality": quality,
                "accepted": False,
                "reason": "differential age or reference station missing",
            }
        differential_age = finite_float(fields[13], "differential age")
        if not 0.0 <= differential_age <= MAXIMUM_DIFFERENTIAL_AGE_SECONDS:
            return formatter, {
                "quality": quality,
                "accepted": False,
                "reason": "differential age outside limit",
                "differential_age_sec": differential_age,
            }
        return formatter, {
            "quality": quality,
            "accepted": True,
            "latitude_deg": latitude,
            "longitude_deg": longitude,
            "altitude_ellipsoid_m": altitude_ellipsoid,
            "differential_age_sec": differential_age,
            "reference_station_id": fields[14],
        }
    if formatter == "GST":
        if len(fields) < 9:
            raise ValueError("GST has too few fields")
        uncertainty_fields = fields[6:9]
        if all(field == "" for field in uncertainty_fields):
            return formatter, {
                "accepted": False,
                "reason": "covariance unavailable",
            }
        if any(field == "" for field in uncertainty_fields):
            raise ValueError("incomplete GST standard deviation")
        stddev = [finite_float(field, "GST standard deviation") for field in uncertainty_fields]
        if min(stddev) <= 0.0:
            raise ValueError("non-positive GST standard deviation")
        return formatter, {"accepted": True, "stddev_m": stddev}
    if formatter == "THS":
        if len(fields) < 3:
            raise ValueError("THS has too few fields")
        mode = fields[2]
        heading = None if fields[1] == "" else finite_float(fields[1], "THS heading")
        if heading is not None and not 0.0 <= heading <= 360.0:
            raise ValueError("THS heading outside [0, 360]")
        return formatter, {
            "accepted": mode == "A" and heading is not None,
            "mode": mode,
            "heading_true_deg": heading,
        }
    return formatter, {"accepted": False, "unsupported": True}


class MappingPreflight(Node):
    def __init__(self) -> None:
        super().__init__("rc_mapping_recording_preflight")
        self.accepted_gga_qualities = ACCEPTED_GGA_QUALITIES
        self.gnss_solution_policy = "RTK_FIXED_OR_FLOAT"
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self.on_lidar_serialized,
            sensor_qos,
            raw=True,
        )
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, sensor_qos)
        self.create_subscription(Sentence, NMEA_TOPIC, self.on_nmea, sensor_qos)
        self.create_subscription(TFMessage, TF_STATIC_TOPIC, self.on_tf_static, tf_static_qos)
        self.create_subscription(
            DiagnosticArray,
            DIAGNOSTICS_TOPIC,
            self.on_diagnostics,
            10,
        )

        self.window_active = False
        self.static_transforms: set[tuple[str, str]] = set()
        self.latest_seen: dict[str, float] = {}
        self.latest_lidar_valid = False
        self.latest_imu_valid = False
        self.latest_gga_valid = False
        self.latest_gst_valid = False
        self.latest_ths_valid = False
        self.latest_lidar_detail = "尚未收到点云"
        self.latest_imu_detail = "尚未收到 IMU"
        self.latest_gga_detail: dict[str, Any] = {}
        self.latest_gst_detail: dict[str, Any] = {}
        self.latest_ths_detail: dict[str, Any] = {}
        self.latest_com2_values: dict[str, str] = {}
        self.acquisition_counts = {
            "lidar": 0,
            "lidar_metadata_parse_errors": 0,
            "imu": 0,
            "nmea": 0,
            "nmea_parse_errors": 0,
        }
        self.acquisition_counts_at_window_start: dict[str, int] = {}
        self.lidar_probe_evidence: dict[str, Any] = {}
        self.reset_window()

    def reset_window(self) -> None:
        self.reception_times: dict[str, list[float]] = {
            "lidar": [],
            "imu": [],
            "GGA": [],
            "GST": [],
            "THS": [],
        }
        self.frames: dict[str, set[str]] = {"lidar": set(), "imu": set(), "nmea": set()}
        self.last_stamp: dict[str, int | None] = {"lidar": None, "imu": None, "nmea": None}
        self.stamp_violations: dict[str, int] = {"lidar": 0, "imu": 0, "nmea": 0}
        self.zero_stamps: dict[str, int] = {"lidar": 0, "imu": 0, "nmea": 0}
        self.lidar_field_failures = 0
        self.lidar_field_sets: set[tuple[str, ...]] = set()
        self.lidar_metadata_parse_errors: list[str] = []
        self.nmea_parse_errors: list[str] = []
        self.unsupported_nmea: dict[str, int] = {}
        self.gga_qualities: list[int] = []
        self.gga_accepted_count = 0
        self.gga_differential_ages_sec: list[float] = []
        self.gga_reference_station_ids: list[str] = []
        self.gst_stddev_m: list[list[float]] = []
        self.ths_modes: list[str] = []
        self.ths_headings_deg: list[float] = []
        self.com2_window_samples: list[dict[str, str]] = []

    def begin_window(self) -> None:
        self.acquisition_counts_at_window_start = dict(self.acquisition_counts)
        self.reset_window()
        self.window_active = True

    def stop_python_lidar_observer(self) -> None:
        if self.lidar_subscription is not None:
            self.destroy_subscription(self.lidar_subscription)
            self.lidar_subscription = None

    def load_lidar_probe(self, state: dict[str, Any]) -> None:
        self.reception_times["lidar"] = list(state["reception_times"])
        self.frames["lidar"] = set(state["frames"])
        self.lidar_field_sets = set(state["field_sets"])
        self.lidar_field_failures = int(state["field_failures"])
        self.lidar_metadata_parse_errors = list(state["metadata_parse_errors"])
        self.last_stamp["lidar"] = None
        for stamp in state["source_stamps"]:
            self.update_stamp_value("lidar", int(stamp))

    def update_stamp_value(self, key: str, value: int) -> None:
        if value <= 0:
            self.zero_stamps[key] += 1
        previous = self.last_stamp[key]
        if previous is not None and value <= previous:
            self.stamp_violations[key] += 1
        self.last_stamp[key] = value

    def on_lidar_serialized(self, serialized: bytes) -> None:
        received = time.monotonic()
        self.acquisition_counts["lidar"] += 1
        if self.window_active:
            self.reception_times["lidar"].append(received)
        try:
            stamp, raw_frame, fields = pointcloud2_metadata_from_cdr(serialized)
        except ValueError as error:
            self.acquisition_counts["lidar_metadata_parse_errors"] += 1
            self.latest_lidar_valid = False
            self.latest_lidar_detail = f"点云 metadata 无法解析：{error}"
            self.latest_seen["lidar"] = received
            if self.window_active and len(self.lidar_metadata_parse_errors) < 20:
                self.lidar_metadata_parse_errors.append(str(error))
            return
        frame = normalize_frame(raw_frame)
        self.latest_lidar_valid = frame == LIDAR_FRAME and REQUIRED_LIDAR_FIELDS <= fields
        missing_fields = sorted(REQUIRED_LIDAR_FIELDS - fields)
        if frame != LIDAR_FRAME:
            self.latest_lidar_detail = f"frame={frame or '<empty>'}，应为 {LIDAR_FRAME}"
        elif missing_fields:
            self.latest_lidar_detail = f"缺少字段 {missing_fields}"
        else:
            self.latest_lidar_detail = f"frame={LIDAR_FRAME}，字段正常"
        self.latest_seen["lidar"] = received
        if not self.window_active:
            return
        self.frames["lidar"].add(frame)
        self.lidar_field_sets.add(tuple(sorted(fields)))
        if not REQUIRED_LIDAR_FIELDS <= fields:
            self.lidar_field_failures += 1
        self.update_stamp_value("lidar", stamp)

    def on_imu(self, message: Imu) -> None:
        received = time.monotonic()
        self.acquisition_counts["imu"] += 1
        frame = normalize_frame(message.header.frame_id)
        self.latest_imu_valid = frame == IMU_FRAME
        self.latest_imu_detail = (
            f"frame={IMU_FRAME}"
            if self.latest_imu_valid
            else f"frame={frame or '<empty>'}，应为 {IMU_FRAME}"
        )
        self.latest_seen["imu"] = received
        if not self.window_active:
            return
        self.reception_times["imu"].append(received)
        self.frames["imu"].add(frame)
        self.update_stamp_value("imu", stamp_ns(message))

    def on_nmea(self, message: Sentence) -> None:
        received = time.monotonic()
        self.acquisition_counts["nmea"] += 1
        self.latest_seen["nmea"] = received
        formatter = "INVALID"
        parsed: dict[str, Any] = {"accepted": False}
        error_text = ""
        try:
            formatter, parsed = parse_mapping_sentence(message.sentence)
        except (ValueError, OverflowError) as error:
            error_text = str(error)
            self.acquisition_counts["nmea_parse_errors"] += 1

        if formatter == "GGA":
            self.latest_gga_valid = bool(parsed.get("accepted"))
            self.latest_gga_detail = dict(parsed)
            self.latest_seen["GGA"] = received
        elif formatter == "GST":
            self.latest_gst_valid = bool(parsed.get("accepted"))
            self.latest_gst_detail = dict(parsed)
            self.latest_seen["GST"] = received
        elif formatter == "THS":
            self.latest_ths_valid = bool(parsed.get("accepted"))
            self.latest_ths_detail = dict(parsed)
            self.latest_seen["THS"] = received

        if not self.window_active:
            return
        self.frames["nmea"].add(normalize_frame(message.header.frame_id))
        self.update_stamp_value("nmea", stamp_ns(message))
        if error_text:
            if len(self.nmea_parse_errors) < 20:
                self.nmea_parse_errors.append(error_text)
            return
        if formatter not in {"GGA", "GST", "THS"}:
            self.unsupported_nmea[formatter] = self.unsupported_nmea.get(formatter, 0) + 1
            return
        self.reception_times[formatter].append(received)
        if formatter == "GGA":
            self.gga_qualities.append(int(parsed["quality"]))
            if parsed.get("accepted"):
                self.gga_accepted_count += 1
                self.gga_differential_ages_sec.append(float(parsed["differential_age_sec"]))
                self.gga_reference_station_ids.append(str(parsed["reference_station_id"]))
        elif formatter == "GST":
            if parsed.get("accepted"):
                self.gst_stddev_m.append(list(parsed["stddev_m"]))
        elif formatter == "THS":
            self.ths_modes.append(str(parsed.get("mode", "")))
            heading = parsed.get("heading_true_deg")
            if heading is not None:
                self.ths_headings_deg.append(float(heading))

    def on_tf_static(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self.static_transforms.add(
                (
                    normalize_frame(transform.header.frame_id),
                    normalize_frame(transform.child_frame_id),
                )
            )

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        received = time.monotonic()
        for status in message.status:
            if status.hardware_id != "G90-COM2":
                continue
            values = {str(item.key): str(item.value) for item in status.values}
            values["level"] = str(diagnostic_level(status.level))
            values["message"] = str(status.message)
            self.latest_com2_values = values
            self.latest_seen["COM2"] = received
            if self.window_active:
                self.com2_window_samples.append(
                    {
                        key: values.get(key, "unknown")
                        for key in (
                            "worker_alive",
                            "serial_open",
                            "caster_connected",
                            "rtcm_fresh",
                            "credential_expired",
                        )
                    }
                )

    def readiness_blockers(self) -> list[str]:
        current = time.monotonic()
        blockers: list[str] = []
        conditions = (
            ("lidar", "LiDAR", self.latest_lidar_valid),
            ("imu", "IMU", self.latest_imu_valid),
            ("GGA", "GGA RTK", self.latest_gga_valid),
            ("GST", "GST 协方差", self.latest_gst_valid),
            ("THS", "THS 航向", self.latest_ths_valid),
        )
        for key, label, valid in conditions:
            seen = self.latest_seen.get(key)
            if seen is None:
                blockers.append(f"{label} 尚无数据")
            elif current - seen > ACQUISITION_FRESHNESS_SECONDS[key]:
                blockers.append(f"{label} 数据已中断")
            elif not valid:
                if key == "GGA" and "quality" in self.latest_gga_detail:
                    blockers.append(
                        f"GGA quality={self.latest_gga_detail['quality']}，需要 4/5"
                    )
                elif key == "THS" and "mode" in self.latest_ths_detail:
                    blockers.append(
                        f"THS mode={self.latest_ths_detail['mode'] or '<empty>'}，需要 A"
                    )
                else:
                    blockers.append(f"{label} 当前无效")

        nmea_seen = self.latest_seen.get("nmea")
        if nmea_seen is None:
            blockers.append("G90 COM1 尚无 NMEA 数据")
        elif current - nmea_seen > ACQUISITION_FRESHNESS_SECONDS["nmea"]:
            blockers.append("G90 COM1 NMEA 数据已中断")

        com2_seen = self.latest_seen.get("COM2")
        if com2_seen is None:
            blockers.append("G90 COM2 尚未上报差分状态")
        elif current - com2_seen > ACQUISITION_FRESHNESS_SECONDS["COM2"]:
            blockers.append("G90 COM2 差分状态已中断")
        elif not g90_com2_ready(self.latest_com2_values):
            _, detail = g90_com2_summary(self.latest_com2_values)
            blockers.append(f"G90 COM2：{detail}")

        missing_tf = sorted(REQUIRED_STATIC_TRANSFORMS - self.static_transforms)
        if missing_tf:
            blockers.append(f"静态 TF 缺失：{missing_tf}")
        return blockers

    def status_items(self) -> tuple[StatusItem, ...]:
        """Return stable per-device state without coupling it to terminal layout."""

        current = time.monotonic()

        def is_fresh(key: str) -> bool:
            seen = self.latest_seen.get(key)
            return seen is not None and (
                current - seen <= ACQUISITION_FRESHNESS_SECONDS[key]
            )

        if not is_fresh("lidar"):
            lidar = ("等待", "无数据或数据已中断")
        elif self.latest_lidar_valid:
            lidar = ("正常", self.latest_lidar_detail)
        else:
            lidar = ("故障", self.latest_lidar_detail)

        if not is_fresh("imu"):
            imu = ("等待", "无数据或数据已中断")
        elif self.latest_imu_valid:
            imu = ("正常", self.latest_imu_detail)
        else:
            imu = ("故障", self.latest_imu_detail)

        com1 = (
            ("正常", "NMEA 持续输出")
            if is_fresh("nmea")
            else ("等待", "无 NMEA 或数据已中断")
        )

        com2_state, com2_detail = g90_com2_summary(self.latest_com2_values)
        if self.latest_seen.get("COM2") is not None and not is_fresh("COM2"):
            com2_state, com2_detail = "故障", "差分状态已中断"

        if not is_fresh("GGA"):
            gga = ("等待", "无数据或数据已中断")
        elif self.latest_gga_valid:
            quality = int(self.latest_gga_detail.get("quality", -1))
            solution = "RTK Fixed" if quality == 4 else "RTK Float"
            gga = ("正常", f"{solution}，quality={quality}")
        elif "quality" in self.latest_gga_detail:
            gga = (
                "等待",
                f"quality={self.latest_gga_detail['quality']}，需要 4/5",
            )
        else:
            gga = ("等待", "当前数据无效")

        if not is_fresh("GST"):
            gst = ("等待", "无数据或数据已中断")
        elif self.latest_gst_valid:
            gst = ("正常", "完整且新鲜")
        else:
            gst = ("等待", "协方差不可用或不完整")

        if not is_fresh("THS"):
            ths = ("等待", "无数据或数据已中断")
        elif self.latest_ths_valid:
            ths = ("正常", "mode=A")
        else:
            mode = self.latest_ths_detail.get("mode", "<empty>") or "<empty>"
            ths = ("等待", f"mode={mode}，需要 A")

        missing_tf = sorted(REQUIRED_STATIC_TRANSFORMS - self.static_transforms)
        tf_state = (
            ("正常", "3/3")
            if not missing_tf
            else ("等待", f"缺少 {missing_tf}")
        )
        return (
            ("LiDAR", *lidar),
            ("IMU", *imu),
            ("G90 COM1", *com1),
            ("G90 COM2", com2_state, com2_detail),
            ("RTK", *gga),
            ("GST", *gst),
            ("THS", *ths),
            ("静态 TF", *tf_state),
        )

    def status_snapshot(self) -> tuple[str, ...]:
        """Return one stable, human-readable evidence string per device."""

        return tuple(
            f"{label} {state}：{detail}"
            for label, state, detail in self.status_items()
        )

    def result(self, *, acquisition_wait_seconds: float) -> dict[str, Any]:
        checks: list[dict[str, Any]] = []

        def add_check(name: str, passed: bool, observed: Any, required: Any) -> None:
            checks.append(
                {"name": name, "passed": bool(passed), "observed": observed, "required": required}
            )

        rates = {key: sample_rate(values) for key, values in self.reception_times.items()}
        gaps = {key: maximum_gap(values) for key, values in self.reception_times.items()}
        for key in ("lidar", "imu", "GGA", "GST", "THS"):
            add_check(
                f"{key}_rate",
                rates[key] >= MINIMUM_RATE_HZ[key],
                rates[key],
                {"minimum_hz": MINIMUM_RATE_HZ[key]},
            )
            add_check(
                f"{key}_maximum_gap",
                gaps[key] is not None and gaps[key] <= MAXIMUM_GAP_SECONDS[key],
                gaps[key],
                {"maximum_seconds": MAXIMUM_GAP_SECONDS[key]},
            )

        add_check("lidar_frame", self.frames["lidar"] == {LIDAR_FRAME}, sorted(self.frames["lidar"]), [LIDAR_FRAME])
        add_check("imu_frame", self.frames["imu"] == {IMU_FRAME}, sorted(self.frames["imu"]), [IMU_FRAME])
        add_check("nmea_frame", self.frames["nmea"] == {GNSS_FRAME}, sorted(self.frames["nmea"]), [GNSS_FRAME])
        add_check(
            "lidar_serialized_metadata_parse",
            not self.lidar_metadata_parse_errors
            and int(self.lidar_probe_evidence.get("metadata_parse_error_count", 0)) == 0,
            {
                "count": self.lidar_probe_evidence.get("metadata_parse_error_count", 0),
                "examples": self.lidar_metadata_parse_errors,
            },
            {"count": 0, "examples": []},
        )
        probe_exit_code = self.lidar_probe_evidence.get("recorder_exit_code")
        probe_stop_escalation = self.lidar_probe_evidence.get("stop_escalation")
        add_check(
            "lidar_probe_recorder",
            probe_exit_code in {0, 130, -int(signal.SIGINT)}
            and probe_stop_escalation == "none"
            and bool(self.lidar_probe_evidence.get("artifacts")),
            {
                "exit_code": probe_exit_code,
                "stop_escalation": probe_stop_escalation,
                "total_recorded_messages": self.lidar_probe_evidence.get(
                    "total_recorded_messages", 0
                ),
                "window_messages": self.lidar_probe_evidence.get("window_messages", 0),
                "artifacts": self.lidar_probe_evidence.get("artifacts", []),
            },
            "sealed C++ rosbag2 probe with expected SIGINT shutdown and no escalation",
        )
        add_check("lidar_fields", self.lidar_field_failures == 0, {"failures": self.lidar_field_failures, "observed_sets": [list(values) for values in sorted(self.lidar_field_sets)]}, {"required_subset": sorted(REQUIRED_LIDAR_FIELDS)})
        for key in ("lidar", "imu", "nmea"):
            add_check(f"{key}_nonzero_stamps", self.zero_stamps[key] == 0, self.zero_stamps[key], 0)
            add_check(f"{key}_strictly_increasing_stamps", self.stamp_violations[key] == 0, self.stamp_violations[key], 0)
        add_check("nmea_checksum_and_parse", not self.nmea_parse_errors, self.nmea_parse_errors, [])
        add_check("nmea_supported_formatters", not self.unsupported_nmea, self.unsupported_nmea, {})
        observed_gga_qualities = set(self.gga_qualities)
        add_check(
            "gga_solution_quality",
            bool(observed_gga_qualities)
            and observed_gga_qualities <= self.accepted_gga_qualities,
            sorted(observed_gga_qualities),
            {
                "accepted_gga_qualities": sorted(self.accepted_gga_qualities),
                "policy": self.gnss_solution_policy,
            },
        )
        add_check(
            "gga_fresh_differential",
            self.gga_accepted_count == len(self.reception_times["GGA"])
            and bool(self.gga_differential_ages_sec)
            and max(self.gga_differential_ages_sec) <= MAXIMUM_DIFFERENTIAL_AGE_SECONDS
            and len(set(self.gga_reference_station_ids)) == 1,
            {
                "accepted": self.gga_accepted_count,
                "received": len(self.reception_times["GGA"]),
                "maximum_differential_age_sec": max(self.gga_differential_ages_sec, default=None),
                "reference_station_ids": sorted(set(self.gga_reference_station_ids)),
            },
            {
                "all_epochs": True,
                "maximum_differential_age_sec": MAXIMUM_DIFFERENTIAL_AGE_SECONDS,
                "single_nonempty_reference_station": True,
            },
        )
        add_check(
            "g90_com2_correction_stream",
            bool(self.com2_window_samples)
            and all(g90_com2_ready(sample) for sample in self.com2_window_samples),
            {
                "sample_count": len(self.com2_window_samples),
                "all_ready": bool(self.com2_window_samples)
                and all(
                    g90_com2_ready(sample) for sample in self.com2_window_samples
                ),
            },
            "every fixed-window G90-COM2 diagnostic sample ready",
        )
        add_check("gst_positive_complete", len(self.gst_stddev_m) == len(self.reception_times["GST"]), {"valid": len(self.gst_stddev_m), "received": len(self.reception_times["GST"])}, "all GST epochs")
        add_check("ths_mode_a", bool(self.ths_modes) and set(self.ths_modes) == {"A"} and len(self.ths_headings_deg) == len(self.reception_times["THS"]), {"modes": sorted(set(self.ths_modes)), "valid_headings": len(self.ths_headings_deg), "received": len(self.reception_times["THS"])}, "all THS epochs mode A with heading")
        missing_tf = sorted(REQUIRED_STATIC_TRANSFORMS - self.static_transforms)
        add_check("required_static_transforms", not missing_tf, {"missing": [list(pair) for pair in missing_tf], "observed": [list(pair) for pair in sorted(self.static_transforms)]}, {"required": [list(pair) for pair in sorted(REQUIRED_STATIC_TRANSFORMS)]})

        failures = [check["name"] for check in checks if not check["passed"]]
        return {
            "schema_version": 1,
            "kind": "rc_mapping_recording_preflight",
            "status": "PASS" if not failures else "FAIL",
            "completed_at": now_iso8601(),
            "clock_contract": "ROS wall clock; use_sim_time=false",
            "lidar_observation_contract": {
                "acquisition": "Python raw CDR metadata until all inputs are simultaneously valid",
                "fixed_window_transport": "C++ ros2 bag record using the formal QoS file",
                "rate_and_gap": "rosbag2 receipt timestamp for every fixed-window sample",
                "metadata": "CDR stamp/frame/fields parsed from every recorded sample",
            },
            "gnss_solution_policy": {
                "name": self.gnss_solution_policy,
                "accepted_gga_qualities": sorted(self.accepted_gga_qualities),
                "quality_labels_preserved": True,
                "rtk_float_is_not_relabelled_as_fixed": True,
            },
            "acquisition_wait_seconds": acquisition_wait_seconds,
            "readiness_stability_seconds": READINESS_STABILITY_SECONDS,
            "window_seconds": WINDOW_SECONDS,
            "thresholds": {
                "minimum_rate_hz": MINIMUM_RATE_HZ,
                "maximum_gap_seconds": MAXIMUM_GAP_SECONDS,
                "maximum_differential_age_seconds": MAXIMUM_DIFFERENTIAL_AGE_SECONDS,
            },
            "counts": {key: len(values) for key, values in self.reception_times.items()},
            "rates_hz": rates,
            "maximum_gap_seconds": gaps,
            "checks": checks,
            "failures": failures,
            "acquisition_counts_before_window": self.acquisition_counts_at_window_start,
            "lidar_probe": self.lidar_probe_evidence,
        }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def nmea_sentence(payload: str) -> str:
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    return f"${payload}*{checksum:02X}"


def self_test() -> int:
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointField

    for raw_level, expected_level in ((b"\x00", 0), (b"\x01", 1), (2, 2)):
        if diagnostic_level(raw_level) != expected_level:
            raise RuntimeError("self-test failed to normalize a diagnostic level")
    try:
        diagnostic_level(b"\x00\x01")
    except ValueError:
        pass
    else:
        raise RuntimeError("self-test accepted a multi-byte diagnostic level")

    examples = [
        ("GNGGA,092750.00,2232.1234,N,11401.5678,E,4,18,0.7,12.3,M,-2.4,M,0.8,1234", "GGA"),
        ("GPGST,092750.00,0.4,0.3,0.2,45.0,0.10,0.20,0.30", "GST"),
        ("GPTHS,90.0,A", "THS"),
    ]
    for payload, expected in examples:
        formatter, parsed = parse_mapping_sentence(nmea_sentence(payload))
        if formatter != expected or not parsed.get("accepted"):
            raise RuntimeError(f"self-test failed for {expected}")
    formatter, parsed = parse_mapping_sentence(nmea_sentence("GPGST,,,,,,,,"))
    if formatter != "GST" or parsed.get("accepted") or parsed.get("reason") != "covariance unavailable":
        raise RuntimeError("self-test rejected a checksum-valid unavailable GST epoch")
    float_gga = nmea_sentence(
        "GNGGA,092750.00,2232.1234,N,11401.5678,E,5,18,0.7,12.3,M,-2.4,M,0.8,1234"
    )
    formatter, parsed = parse_mapping_sentence(float_gga)
    if formatter != "GGA" or not parsed.get("accepted") or parsed.get("quality") != 5:
        raise RuntimeError("self-test rejected RTK Float under the formal recording policy")
    standalone_gga = nmea_sentence(
        "GNGGA,092750.00,2232.1234,N,11401.5678,E,1,18,0.7,12.3,M,-2.4,M,0.8,1234"
    )
    formatter, parsed = parse_mapping_sentence(standalone_gga)
    if formatter != "GGA" or parsed.get("accepted"):
        raise RuntimeError("self-test accepted non-RTK GGA quality")
    try:
        parse_mapping_sentence("$GPTHS,90.0,A*00")
    except ValueError:
        pass
    else:
        raise RuntimeError("self-test accepted a bad checksum")

    cloud = PointCloud2()
    cloud.header.stamp.sec = 123
    cloud.header.stamp.nanosec = 456
    cloud.header.frame_id = LIDAR_FRAME
    cloud.height = 1
    cloud.width = 2
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    ]
    cloud.point_step = 18
    cloud.row_step = 36
    cloud.data = bytes(range(36))
    cloud.is_dense = True
    serialized_cloud = serialize_message(cloud)
    stamp, frame, fields = pointcloud2_metadata_from_cdr(serialized_cloud)
    if stamp != 123_000_000_456 or frame != LIDAR_FRAME or fields != REQUIRED_LIDAR_FIELDS:
        raise RuntimeError("self-test failed to parse serialized PointCloud2 metadata")
    try:
        pointcloud2_metadata_from_cdr(b"\x00\x07\x00\x00")
    except ValueError:
        pass
    else:
        raise RuntimeError("self-test accepted an unsupported PointCloud2 CDR representation")
    ensure_process_alive(os.getpid())
    if g90_com2_ready({}):
        raise RuntimeError("self-test accepted an empty G90 COM2 state")
    ready_correction = {
        "worker_alive": "true",
        "serial_open": "true",
        "caster_connected": "true",
        "rtcm_fresh": "true",
        "credential_expired": "false",
    }
    if not g90_com2_ready(ready_correction):
        raise RuntimeError("self-test rejected a ready G90 COM2 state")
    if g90_com2_summary(ready_correction)[0] != "正常":
        raise RuntimeError("self-test did not report a ready G90 COM2 state")

    render_items: tuple[StatusItem, ...] = (
        ("LiDAR", "正常", "frame=lidar_top，字段正常"),
        ("IMU", "正常", "frame=imu_link"),
        ("G90 COM1", "正常", "NMEA 持续输出"),
        ("G90 COM2", "正常", "新鲜 RTCM 正在写入 COM2"),
        ("RTK", "等待", "quality=0，需要 4/5"),
        ("GST", "等待", "协方差不可用或不完整"),
        ("THS", "等待", "mode=V，需要 A"),
        (
            "静态 TF",
            "等待",
            "缺少 base_link 到 lidar_top、imu_link、gnss_link 的静态变换",
        ),
    )
    for columns in (60, 80, 120):
        rendered = render_status_rows(render_items, columns)
        if not rendered or any(display_width(line) > columns for line in rendered):
            raise RuntimeError(
                f"self-test rendered a status line wider than {columns} columns"
            )
        if any("｜" in line for line in rendered):
            raise RuntimeError("self-test rendered a multi-device separator")
    updated_items = render_items[:4] + (
        ("RTK", "正常", "RTK Float，quality=5"),
    ) + render_items[5:]
    changed = changed_status_items(render_items, updated_items)
    if changed != (("RTK", "正常", "RTK Float，quality=5"),):
        raise RuntimeError("self-test did not isolate the changed status row")
    if readiness_summary(render_items) != "等待：RTK、GST、THS、静态 TF":
        raise RuntimeError("self-test produced an unexpected readiness summary")
    print("preflight parser self-test: PASS")
    return 0


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="JSON result path")
    parser.add_argument(
        "--watch-pid",
        type=int,
        help="sensing process to fail on if it exits before preflight completes",
    )
    parser.add_argument(
        "--lidar-probe-dir",
        type=Path,
        help="quality-evidence directory for the temporary C++ LiDAR probe bag",
    )
    parser.add_argument(
        "--lidar-probe-log",
        type=Path,
        help="log path for the temporary C++ LiDAR probe recorder",
    )
    parser.add_argument(
        "--lidar-qos-file",
        type=Path,
        help="formal rosbag2 QoS override file used by the LiDAR probe",
    )
    parser.add_argument("--self-test", action="store_true", help="run parser checks without joining a ROS graph")
    arguments = parser.parse_args()
    if not arguments.self_test and arguments.output is None:
        parser.error("--output is required unless --self-test is used")
    if not arguments.self_test:
        if arguments.lidar_probe_dir is None:
            parser.error("--lidar-probe-dir is required unless --self-test is used")
        if arguments.lidar_probe_log is None:
            parser.error("--lidar-probe-log is required unless --self-test is used")
        if arguments.lidar_qos_file is None or not arguments.lidar_qos_file.is_file():
            parser.error("--lidar-qos-file must name an existing file")
        if arguments.lidar_probe_dir.exists():
            parser.error("--lidar-probe-dir must not already exist")
        if arguments.watch_pid is None or arguments.watch_pid <= 0:
            parser.error("--watch-pid must name the active sensing process")
    return arguments


def main() -> int:
    arguments = parse_arguments()
    if arguments.self_test:
        return self_test()

    started_at = now_iso8601()
    start_monotonic = time.monotonic()
    rclpy.init(args=None)
    node = MappingPreflight()
    lidar_probe: LidarProbeRecorder | None = None
    cancel_input = OperatorCancelInput()
    blockers: list[str] = []
    try:
        cancel_input.enable()
        next_progress_report = start_monotonic
        previous_items: tuple[StatusItem, ...] | None = None
        while True:
            if cancel_input.requested():
                raise OperatorCancelled
            ensure_process_alive(arguments.watch_pid)
            rclpy.spin_once(node, timeout_sec=0.1)
            blockers = node.readiness_blockers()
            current = time.monotonic()
            elapsed = current - start_monotonic
            items = node.status_items()
            changed_items = (
                items
                if previous_items is None
                else changed_status_items(previous_items, items)
            )
            if changed_items:
                heading = "当前状态" if previous_items is None else "状态变化"
                print()
                print_wrapped(
                    f"[设备预检 {elapsed:.0f}s] {heading}（按 Q 取消）"
                )
                for line in render_status_rows(changed_items, terminal_columns()):
                    print(line, flush=True)
                print_wrapped(readiness_summary(items))
                previous_items = items
                next_progress_report = current + PROGRESS_HEARTBEAT_SECONDS
            elif current >= next_progress_report:
                print_wrapped(
                    f"[设备预检 {elapsed:.0f}s] {readiness_summary(items)}"
                    "（按 Q 取消）"
                )
                next_progress_report = current + PROGRESS_HEARTBEAT_SECONDS
            if not blockers:
                print()
                print_wrapped(
                    "[设备预检] 所有必需输入已同时在线；不再增加稳定等待，"
                    f"立即进入 {WINDOW_SECONDS:.0f} 秒质量窗口。"
                )
                break

        acquisition_wait = time.monotonic() - start_monotonic
        node.stop_python_lidar_observer()
        lidar_probe = LidarProbeRecorder(
            arguments.lidar_probe_dir,
            arguments.lidar_qos_file,
            arguments.lidar_probe_log,
        )
        print_wrapped("[质量准备] 正在确认 rosbag2 已订阅 LiDAR 原始点云...")
        lidar_probe.start()
        probe_warmup_start = time.monotonic()
        next_probe_report = probe_warmup_start + PROGRESS_HEARTBEAT_SECONDS
        while not lidar_probe.subscription_ready():
            if cancel_input.requested():
                raise OperatorCancelled
            ensure_process_alive(arguments.watch_pid)
            lidar_probe.ensure_running()
            rclpy.spin_once(node, timeout_sec=0.05)
            current = time.monotonic()
            if current >= next_probe_report:
                print_wrapped("[质量准备] 仍在等待 rosbag2 LiDAR 订阅确认...")
                next_probe_report = current + PROGRESS_HEARTBEAT_SECONDS
        probe_subscription_wait_seconds = time.monotonic() - probe_warmup_start
        print_wrapped("[质量准备] LiDAR 订阅已确认，开始固定质量采样。")

        node.begin_window()
        window_start_monotonic = time.monotonic()
        window_start_epoch_ns = time.time_ns()
        previous_window_summary = "全部输入在线"
        print_wrapped(
            f"[质量窗口] 开始 {WINDOW_SECONDS:.0f} 秒固定采样，输入必须持续在线。"
        )
        while time.monotonic() - window_start_monotonic < WINDOW_SECONDS:
            if cancel_input.requested():
                raise OperatorCancelled
            ensure_process_alive(arguments.watch_pid)
            lidar_probe.ensure_running()
            rclpy.spin_once(node, timeout_sec=0.05)
            blockers = node.readiness_blockers()
            window_summary = readiness_summary(node.status_items())
            if window_summary != previous_window_summary:
                print_wrapped(f"[质量窗口] {window_summary}")
                previous_window_summary = window_summary
        window_end_epoch_ns = time.time_ns()
        print_wrapped(f"[质量窗口] {WINDOW_SECONDS:.0f} 秒采样完成。")
        node.window_active = False
        probe_exit_code = lidar_probe.stop()
        probe_state, probe_evidence = analyze_lidar_probe(
            arguments.lidar_probe_dir,
            window_start_epoch_ns=window_start_epoch_ns,
            window_end_epoch_ns=window_end_epoch_ns,
        )
        probe_evidence["recorder_exit_code"] = probe_exit_code
        probe_evidence["stop_escalation"] = lidar_probe.stop_escalation
        probe_evidence["command"] = lidar_probe.command
        probe_evidence["subscription_marker_seen"] = True
        probe_evidence["subscription_wait_seconds"] = probe_subscription_wait_seconds
        node.lidar_probe_evidence = probe_evidence
        node.load_lidar_probe(probe_state)
        payload = node.result(acquisition_wait_seconds=acquisition_wait)
        payload["started_at"] = started_at
        write_json(arguments.output, payload)
        if payload["status"] == "PASS":
            print_wrapped("[预检通过] 所有固定窗口质量检查通过。")
        else:
            print_wrapped(
                "[预检失败] 未通过项：" + "，".join(payload["failures"])
            )
        return 0 if payload["status"] == "PASS" else 3
    except OperatorCancelled:
        payload = {
            "schema_version": 1,
            "kind": "rc_mapping_recording_preflight",
            "status": "CANCELLED",
            "started_at": started_at,
            "completed_at": now_iso8601(),
            "failure": "operator_cancelled",
            "readiness_blockers": blockers,
            "acquisition_counts": node.acquisition_counts,
            "last_status_snapshot": list(node.status_snapshot()),
        }
        write_json(arguments.output, payload)
        print("mapping preflight: CANCELLED")
        return 2
    except KeyboardInterrupt:
        payload = {
            "schema_version": 1,
            "kind": "rc_mapping_recording_preflight",
            "status": "FAIL",
            "started_at": started_at,
            "completed_at": now_iso8601(),
            "failure": "interrupted",
        }
        write_json(arguments.output, payload)
        return 130
    except Exception as error:
        payload = {
            "schema_version": 1,
            "kind": "rc_mapping_recording_preflight",
            "status": "FAIL",
            "started_at": started_at,
            "completed_at": now_iso8601(),
            "failure": "preflight_internal_error",
            "details": str(error),
        }
        write_json(arguments.output, payload)
        print(f"mapping preflight: FAIL ({error})", file=sys.stderr)
        return 4
    finally:
        cancel_input.restore()
        if lidar_probe is not None:
            lidar_probe.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
