"""Keep G90 COM2 NTRIP corrections inside the owned RC runtime graph."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import os
from pathlib import Path
import re
import shlex
import stat
import threading
import time
from typing import Callable, Optional
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


_REQUIRED_KEYS = frozenset(
    {
        "NTRIP_USERNAME",
        "NTRIP_PASSWORD",
        "NTRIP_HOST",
        "NTRIP_PORT",
        "NTRIP_MOUNTPOINT",
        "NTRIP_EXPIRES_AT_LOCAL",
    }
)
_OPTIONAL_KEYS = frozenset(
    {
        "NTRIP_VERSION",
        "NTRIP_VALIDITY_DAYS",
        "G90_COM2_DEVICE",
        "G90_COM2_RTKLIB_PORT",
        "G90_COM2_BAUD",
    }
)
_KEY_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]*$")
_MAX_CONFIG_BYTES = 64 * 1024
_MAX_NMEA_BUFFER_BYTES = 8192
_MAX_NMEA_SENTENCE_BYTES = 256


class NtripConfigError(ValueError):
    """A sanitized configuration error that never contains a secret value."""


@dataclass(frozen=True, repr=False)
class NtripConfig:
    """Validated secrets and endpoint metadata kept only in process memory."""

    host: str = field(repr=False)
    port: int
    mountpoint: str = field(repr=False)
    username: str = field(repr=False)
    password: str = field(repr=False)
    expires_at: datetime = field(repr=False)
    ntrip_version: Optional[str] = field(default=None, repr=False)

    def is_expired(self, now: Optional[datetime] = None) -> bool:
        reference = now or datetime.now().astimezone()
        return reference >= self.expires_at


@dataclass(frozen=True)
class RelaySnapshot:
    worker_alive: bool
    serial_open: bool
    caster_connected: bool
    credential_expired: bool
    gga_received_count: int
    gga_forwarded_count: int
    rtcm_packet_count: int
    rtcm_byte_count: int
    serial_error_count: int
    ntrip_error_count: int
    ntrip_connect_attempt_count: int
    last_gga_age_sec: Optional[float]
    last_rtcm_age_sec: Optional[float]
    last_error: str


def _decode_env_value(raw_value: str, line_number: int) -> str:
    if "\x00" in raw_value:
        raise NtripConfigError(
            f"NTRIP config line {line_number} contains a NUL byte"
        )
    lexer = shlex.shlex(raw_value, posix=True)
    lexer.whitespace_split = True
    lexer.commenters = ""
    try:
        tokens = list(lexer)
    except ValueError as error:
        raise NtripConfigError(
            f"NTRIP config line {line_number} has invalid quoting"
        ) from error
    if len(tokens) != 1:
        raise NtripConfigError(
            f"NTRIP config line {line_number} must contain one value"
        )
    return tokens[0]


def _read_private_env(path: Path) -> dict[str, str]:
    if not path.is_absolute():
        raise NtripConfigError("NTRIP config path must be absolute")

    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise NtripConfigError("NTRIP config cannot be opened") from error

    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise NtripConfigError("NTRIP config must be a regular file")
        if metadata.st_uid != os.geteuid():
            raise NtripConfigError(
                "NTRIP config must be owned by the runtime user"
            )
        if stat.S_IMODE(metadata.st_mode) != 0o600:
            raise NtripConfigError("NTRIP config mode must be exactly 0600")
        if metadata.st_size > _MAX_CONFIG_BYTES:
            raise NtripConfigError("NTRIP config is unexpectedly large")
        with os.fdopen(descriptor, "r", encoding="utf-8") as stream:
            descriptor = -1
            lines = stream.read().splitlines()
    except UnicodeDecodeError as error:
        raise NtripConfigError("NTRIP config must be UTF-8 text") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)

    values: dict[str, str] = {}
    for line_number, original in enumerate(lines, start=1):
        line = original.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            raise NtripConfigError(
                f"NTRIP config line {line_number} is not KEY=VALUE"
            )
        key, raw_value = line.split("=", 1)
        key = key.strip()
        if not _KEY_PATTERN.fullmatch(key):
            raise NtripConfigError(
                f"NTRIP config line {line_number} has an invalid key"
            )
        if key not in _REQUIRED_KEYS | _OPTIONAL_KEYS:
            raise NtripConfigError(
                f"NTRIP config contains unsupported key {key}"
            )
        if key in values:
            raise NtripConfigError(
                f"NTRIP config contains duplicate key {key}"
            )
        values[key] = _decode_env_value(raw_value.strip(), line_number)

    missing = sorted(key for key in _REQUIRED_KEYS if not values.get(key))
    if missing:
        raise NtripConfigError(
            "NTRIP config is missing required keys: " + ", ".join(missing)
        )
    return values


def _parse_expiry(value: str) -> datetime:
    match = re.fullmatch(
        r"(\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2})\s+([A-Za-z0-9_+\-/]+)",
        value,
    )
    if match is None:
        raise NtripConfigError(
            "NTRIP_EXPIRES_AT_LOCAL must include seconds and an IANA timezone"
        )
    try:
        timezone = ZoneInfo(match.group(2))
        local_time = datetime.strptime(
            match.group(1).replace("T", " "), "%Y-%m-%d %H:%M:%S"
        )
    except (ValueError, ZoneInfoNotFoundError) as error:
        raise NtripConfigError("NTRIP_EXPIRES_AT_LOCAL is invalid") from error
    return local_time.replace(tzinfo=timezone)


def load_ntrip_config(
    path: Path,
    *,
    serial_device: str,
    serial_baud: int,
    now: Optional[datetime] = None,
) -> NtripConfig:
    values = _read_private_env(path)
    try:
        port = int(values["NTRIP_PORT"])
    except ValueError as error:
        raise NtripConfigError("NTRIP_PORT must be an integer") from error
    if not 1 <= port <= 65535:
        raise NtripConfigError("NTRIP_PORT is outside the valid range")

    host = values["NTRIP_HOST"].strip()
    mountpoint = values["NTRIP_MOUNTPOINT"].strip().lstrip("/")
    if not host or any(character.isspace() for character in host):
        raise NtripConfigError("NTRIP_HOST is invalid")
    if not mountpoint or any(character.isspace() for character in mountpoint):
        raise NtripConfigError("NTRIP_MOUNTPOINT is invalid")

    configured_device = values.get("G90_COM2_DEVICE")
    if configured_device and configured_device != serial_device:
        raise NtripConfigError(
            "G90_COM2_DEVICE does not match the launch contract"
        )
    configured_baud = values.get("G90_COM2_BAUD")
    if configured_baud:
        try:
            parsed_baud = int(configured_baud)
        except ValueError as error:
            raise NtripConfigError(
                "G90_COM2_BAUD must be an integer"
            ) from error
        if parsed_baud != serial_baud:
            raise NtripConfigError(
                "G90_COM2_BAUD does not match the launch contract"
            )

    expires_at = _parse_expiry(values["NTRIP_EXPIRES_AT_LOCAL"])
    config = NtripConfig(
        host=host,
        port=port,
        mountpoint=mountpoint,
        username=values["NTRIP_USERNAME"],
        password=values["NTRIP_PASSWORD"],
        expires_at=expires_at,
        ntrip_version=values.get("NTRIP_VERSION") or None,
    )
    if config.is_expired(now):
        raise NtripConfigError("NTRIP credentials are expired")
    return config


def extract_gga_sentences(buffer: bytes) -> tuple[list[str], bytes]:
    """Extract complete GGA lines and retain one bounded partial line."""

    if len(buffer) > _MAX_NMEA_BUFFER_BYTES:
        buffer = buffer[-_MAX_NMEA_BUFFER_BYTES:]
    complete, separator, remainder = buffer.rpartition(b"\n")
    if not separator:
        return [], buffer

    sentences: list[str] = []
    for raw_line in complete.splitlines():
        line = raw_line.strip(b"\r\x00")
        if not 6 <= len(line) <= _MAX_NMEA_SENTENCE_BYTES:
            continue
        if line.startswith(b"$") and line[3:6] == b"GGA":
            try:
                sentences.append(line.decode("ascii"))
            except UnicodeDecodeError:
                continue
    return sentences, remainder[-_MAX_NMEA_SENTENCE_BYTES:]


class G90NtripRelayWorker:
    """Own the COM2 and caster connections in one stoppable worker thread."""

    def __init__(
        self,
        config: NtripConfig,
        serial_device: str,
        serial_baud: int,
        *,
        serial_factory: Optional[Callable[[str, int], object]] = None,
        ntrip_factory: Optional[Callable[[NtripConfig], object]] = None,
        clock: Callable[[], float] = time.monotonic,
        reconnect_wait_sec: float = 2.0,
    ) -> None:
        self._config = config
        self._serial_device = serial_device
        self._serial_baud = serial_baud
        self._serial_factory = serial_factory or self._default_serial_factory
        self._ntrip_factory = ntrip_factory or self._default_ntrip_factory
        self._clock = clock
        self._reconnect_wait_sec = reconnect_wait_sec
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._serial = None
        self._client = None
        self._worker_alive = False
        self._serial_open = False
        self._caster_connected = False
        self._gga_received_count = 0
        self._gga_forwarded_count = 0
        self._rtcm_packet_count = 0
        self._rtcm_byte_count = 0
        self._serial_error_count = 0
        self._ntrip_error_count = 0
        self._ntrip_connect_attempt_count = 0
        self._last_gga_time: Optional[float] = None
        self._last_rtcm_time: Optional[float] = None
        self._last_error = "none"

    @staticmethod
    def _default_serial_factory(device: str, baud: int):
        import serial

        return serial.Serial(
            port=device,
            baudrate=baud,
            timeout=0.1,
            write_timeout=1.0,
            exclusive=True,
        )

    @staticmethod
    def _default_ntrip_factory(config: NtripConfig):
        from ntrip_client.ntrip_client import NTRIPClient

        def quiet(_message):
            return None
        client = NTRIPClient(
            host=config.host,
            port=config.port,
            mountpoint=config.mountpoint,
            ntrip_version=config.ntrip_version,
            username=config.username,
            password=config.password,
            logerr=quiet,
            logwarn=quiet,
            loginfo=quiet,
            logdebug=quiet,
        )
        # The G90 emits high-precision GGA sentences that can exceed the
        # legacy NMEA 82-byte limit enforced by ntrip_client.  The relay has
        # already bounded complete receiver lines to _MAX_NMEA_SENTENCE_BYTES;
        # include CRLF here because send_nmea() appends it before validating.
        client.nmea_parser.nmea_max_length = _MAX_NMEA_SENTENCE_BYTES + 2
        # Reconnection is owned by this interruptible worker, not by blocking
        # sleeps inside the upstream helper.
        client.rtcm_timeout_seconds = 1_000_000_000
        client.reconnect_attempt_max = 1
        client.reconnect_attempt_wait_seconds = 0
        return client

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("G90 NTRIP relay worker was already started")
        self._thread = threading.Thread(
            target=self._run,
            name="g90-ntrip-relay",
            daemon=False,
        )
        self._thread.start()

    def stop(self, timeout_sec: float = 7.0) -> bool:
        self._stop_event.set()
        client = self._client
        serial_port = self._serial
        if client is not None:
            try:
                client.disconnect()
            except Exception:  # noqa: BLE001 - shutdown is best effort.
                pass
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:  # noqa: BLE001 - shutdown is best effort.
                pass
        if self._thread is not None:
            self._thread.join(timeout=timeout_sec)
            return not self._thread.is_alive()
        return True

    def snapshot(self) -> RelaySnapshot:
        now = self._clock()
        with self._lock:
            return RelaySnapshot(
                worker_alive=self._worker_alive,
                serial_open=self._serial_open,
                caster_connected=self._caster_connected,
                credential_expired=self._config.is_expired(),
                gga_received_count=self._gga_received_count,
                gga_forwarded_count=self._gga_forwarded_count,
                rtcm_packet_count=self._rtcm_packet_count,
                rtcm_byte_count=self._rtcm_byte_count,
                serial_error_count=self._serial_error_count,
                ntrip_error_count=self._ntrip_error_count,
                ntrip_connect_attempt_count=self._ntrip_connect_attempt_count,
                last_gga_age_sec=(
                    None
                    if self._last_gga_time is None
                    else max(0.0, now - self._last_gga_time)
                ),
                last_rtcm_age_sec=(
                    None
                    if self._last_rtcm_time is None
                    else max(0.0, now - self._last_rtcm_time)
                ),
                last_error=self._last_error,
            )

    def _set_state(self, **updates) -> None:
        with self._lock:
            for name, value in updates.items():
                setattr(self, f"_{name}", value)

    def _increment(self, name: str, amount: int = 1) -> None:
        with self._lock:
            attribute = f"_{name}"
            setattr(self, attribute, getattr(self, attribute) + amount)

    def _disconnect_client(self) -> None:
        client, self._client = self._client, None
        self._set_state(caster_connected=False)
        if client is not None:
            try:
                client.disconnect()
            except Exception:  # noqa: BLE001 - connection is already unusable.
                pass

    def _disconnect_serial(self) -> None:
        serial_port, self._serial = self._serial, None
        self._set_state(serial_open=False)
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:  # noqa: BLE001 - connection is already unusable.
                pass

    def _connect_serial(self) -> bool:
        try:
            self._serial = self._serial_factory(
                self._serial_device, self._serial_baud
            )
        except Exception:  # noqa: BLE001 - retry with a sanitized diagnostic.
            self._increment("serial_error_count")
            self._set_state(serial_open=False, last_error="serial_open_failed")
            return False
        self._set_state(serial_open=True, last_error="none")
        return True

    def _connect_client(self) -> bool:
        self._increment("ntrip_connect_attempt_count")
        client = None
        try:
            client = self._ntrip_factory(self._config)
            connected = bool(client.connect())
        except Exception:  # noqa: BLE001 - retry with a sanitized diagnostic.
            connected = False
        if not connected:
            if client is not None:
                try:
                    client.disconnect()
                except Exception:  # noqa: BLE001 - failed connection cleanup.
                    pass
            self._increment("ntrip_error_count")
            self._set_state(
                caster_connected=False, last_error="caster_connect_failed"
            )
            return False
        self._client = client
        self._set_state(caster_connected=True, last_error="none")
        return True

    def _read_serial(self, pending: bytes) -> tuple[bytes, Optional[str]]:
        if self._serial is None:
            return pending, None
        waiting = int(getattr(self._serial, "in_waiting", 0))
        chunk = self._serial.read(max(1, min(waiting, 4096)))
        if not chunk:
            return pending, None
        sentences, remainder = extract_gga_sentences(pending + bytes(chunk))
        latest = sentences[-1] if sentences else None
        if sentences:
            now = self._clock()
            self._increment("gga_received_count", len(sentences))
            self._set_state(last_gga_time=now)
        return remainder, latest

    def _forward_gga(self, sentence: str) -> bool:
        if self._client is None:
            return False
        try:
            self._client.send_nmea(sentence)
        except Exception:  # noqa: BLE001 - reconnect without exposing secrets.
            self._increment("ntrip_error_count")
            self._set_state(last_error="gga_forward_failed")
            self._disconnect_client()
            return False
        self._increment("gga_forwarded_count")
        return True

    def _receive_rtcm(self) -> bool:
        if self._client is None or self._serial is None:
            return False
        try:
            packets = self._client.recv_rtcm()
            for packet in packets:
                raw_packet = bytes(packet)
                written = self._serial.write(raw_packet)
                if written != len(raw_packet):
                    raise OSError("short serial write")
                self._increment("rtcm_packet_count")
                self._increment("rtcm_byte_count", len(raw_packet))
                self._set_state(
                    last_rtcm_time=self._clock(), last_error="none"
                )
        except Exception:  # noqa: BLE001 - owned boundary failure.
            self._increment("ntrip_error_count")
            self._set_state(last_error="rtcm_transfer_failed")
            self._disconnect_client()
            return False
        return True

    def _run(self) -> None:
        self._set_state(worker_alive=True)
        serial_buffer = b""
        pending_gga: Optional[str] = None
        next_serial_attempt = 0.0
        next_ntrip_attempt = 0.0
        try:
            while not self._stop_event.is_set():
                now = self._clock()
                if self._serial is None:
                    if now < next_serial_attempt:
                        self._stop_event.wait(
                            min(0.1, next_serial_attempt - now)
                        )
                        continue
                    if not self._connect_serial():
                        next_serial_attempt = now + self._reconnect_wait_sec
                        continue

                try:
                    serial_buffer, latest_gga = self._read_serial(
                        serial_buffer
                    )
                    if latest_gga is not None:
                        pending_gga = latest_gga
                except Exception:  # noqa: BLE001 - reconnect physical port.
                    self._increment("serial_error_count")
                    self._set_state(last_error="serial_read_failed")
                    self._disconnect_client()
                    self._disconnect_serial()
                    next_serial_attempt = (
                        self._clock() + self._reconnect_wait_sec
                    )
                    continue

                now = self._clock()
                if self._client is None and now >= next_ntrip_attempt:
                    if not self._connect_client():
                        next_ntrip_attempt = (
                            self._clock() + self._reconnect_wait_sec
                        )

                if self._client is not None and pending_gga is not None:
                    if self._forward_gga(pending_gga):
                        pending_gga = None
                    else:
                        next_ntrip_attempt = (
                            self._clock() + self._reconnect_wait_sec
                        )

                if self._client is not None and not self._receive_rtcm():
                    next_ntrip_attempt = (
                        self._clock() + self._reconnect_wait_sec
                    )

                self._stop_event.wait(0.02)
        except Exception:  # noqa: BLE001 - sanitized terminal state.
            self._set_state(last_error="worker_failed")
        finally:
            self._disconnect_client()
            self._disconnect_serial()
            self._set_state(worker_alive=False)


def diagnostic_status(
    node_name: str,
    serial_device: str,
    snapshot: RelaySnapshot,
    *,
    rtcm_freshness_sec: float,
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = f"{node_name}: G90 NTRIP corrections"
    status.hardware_id = "G90-COM2"
    rtcm_fresh = (
        snapshot.last_rtcm_age_sec is not None
        and snapshot.last_rtcm_age_sec <= rtcm_freshness_sec
    )

    if snapshot.credential_expired:
        status.level = DiagnosticStatus.ERROR
        status.message = "NTRIP credentials expired"
    elif not snapshot.worker_alive:
        status.level = DiagnosticStatus.ERROR
        status.message = "Correction worker is not running"
    elif not snapshot.serial_open:
        status.level = DiagnosticStatus.ERROR
        status.message = "G90 COM2 is not open"
    elif not snapshot.caster_connected:
        status.level = DiagnosticStatus.WARN
        status.message = "G90 COM2 open; correction caster not connected"
    elif not rtcm_fresh:
        status.level = DiagnosticStatus.WARN
        status.message = (
            "G90 COM2 and caster connected; waiting for fresh RTCM"
        )
    else:
        status.level = DiagnosticStatus.OK
        status.message = "Fresh RTCM is being written to G90 COM2"

    def age_text(value: Optional[float]) -> str:
        return "unknown" if value is None else f"{value:.3f}"

    values = {
        "serial_device": serial_device,
        "worker_alive": str(snapshot.worker_alive).lower(),
        "serial_open": str(snapshot.serial_open).lower(),
        "caster_connected": str(snapshot.caster_connected).lower(),
        "credential_expired": str(snapshot.credential_expired).lower(),
        "rtcm_fresh": str(rtcm_fresh).lower(),
        "gga_received_count": str(snapshot.gga_received_count),
        "gga_forwarded_count": str(snapshot.gga_forwarded_count),
        "rtcm_packet_count": str(snapshot.rtcm_packet_count),
        "rtcm_byte_count": str(snapshot.rtcm_byte_count),
        "last_gga_age_sec": age_text(snapshot.last_gga_age_sec),
        "last_rtcm_age_sec": age_text(snapshot.last_rtcm_age_sec),
        "serial_error_count": str(snapshot.serial_error_count),
        "ntrip_error_count": str(snapshot.ntrip_error_count),
        "ntrip_connect_attempt_count": str(
            snapshot.ntrip_connect_attempt_count
        ),
        "last_error": snapshot.last_error,
    }
    status.values = [
        KeyValue(key=key, value=value) for key, value in values.items()
    ]
    return status


class G90NtripRelayNode(Node):
    """ROS lifecycle owner and sanitized diagnostic boundary for the relay."""

    def __init__(self, **kwargs) -> None:
        super().__init__("g90_ntrip_relay", **kwargs)
        config_file = str(self.declare_parameter("config_file", "").value)
        self._serial_device = str(
            self.declare_parameter(
                "serial_device", "/dev/autoracer_g90_com2"
            ).value
        )
        serial_baud = int(self.declare_parameter("serial_baud", 115200).value)
        self._rtcm_freshness_sec = float(
            self.declare_parameter("rtcm_freshness_sec", 5.0).value
        )
        diagnostic_period_sec = float(
            self.declare_parameter("diagnostic_period_sec", 1.0).value
        )
        if not config_file:
            raise NtripConfigError("config_file must not be empty")
        if not self._serial_device.startswith("/dev/"):
            raise NtripConfigError(
                "serial_device must be an absolute /dev path"
            )
        if serial_baud <= 0:
            raise NtripConfigError("serial_baud must be positive")
        if self._rtcm_freshness_sec <= 0.0 or diagnostic_period_sec <= 0.0:
            raise NtripConfigError("diagnostic timing must be positive")

        config = load_ntrip_config(
            Path(config_file),
            serial_device=self._serial_device,
            serial_baud=serial_baud,
        )
        self._worker = G90NtripRelayWorker(
            config=config,
            serial_device=self._serial_device,
            serial_baud=serial_baud,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._diagnostic_timer = self.create_timer(
            diagnostic_period_sec, self._publish_diagnostics
        )
        self._stopped = False
        self._worker.start()
        self.get_logger().info("G90 project-owned correction relay started")

    def _publish_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            diagnostic_status(
                self.get_fully_qualified_name(),
                self._serial_device,
                self._worker.snapshot(),
                rtcm_freshness_sec=self._rtcm_freshness_sec,
            )
        ]
        self._diagnostic_publisher.publish(message)

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        if not self._worker.stop():
            self.get_logger().error(
                "G90 correction worker did not stop cleanly"
            )
        else:
            self.get_logger().info(
                "G90 project-owned correction relay stopped"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[G90NtripRelayNode] = None
    try:
        node = G90NtripRelayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
