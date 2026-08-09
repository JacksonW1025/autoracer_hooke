from collections import deque
from datetime import datetime
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticStatus
import pytest

from autoracer_rc_adapter.g90_ntrip_relay_node import (
    G90NtripRelayWorker,
    NtripConfigError,
    RelaySnapshot,
    diagnostic_status,
    extract_gga_sentences,
    load_ntrip_config,
)


SERIAL_DEVICE = "/dev/autoracer_g90_com2"


def write_config(path: Path, *, overrides=None, extra_lines=()) -> Path:
    values = {
        "NTRIP_USERNAME": "unit-test-user",
        "NTRIP_PASSWORD": "unit-test-password-$-#",
        "NTRIP_HOST": "caster.invalid",
        "NTRIP_PORT": "2101",
        "NTRIP_MOUNTPOINT": "TEST",
        "NTRIP_EXPIRES_AT_LOCAL": "2099-08-26 00:00:00 UTC",
        "G90_COM2_DEVICE": SERIAL_DEVICE,
        "G90_COM2_RTKLIB_PORT": "autoracer_g90_com2",
        "G90_COM2_BAUD": "115200",
    }
    values.update(overrides or {})
    lines = [f"{key}={value!r}" for key, value in values.items()]
    lines.extend(extra_lines)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    path.chmod(0o600)
    return path


def load(path: Path, now=None):
    return load_ntrip_config(
        path,
        serial_device=SERIAL_DEVICE,
        serial_baud=115200,
        now=now,
    )


def test_private_config_is_parsed_without_secret_repr(tmp_path):
    path = write_config(tmp_path / "g90-ntrip.env")
    config = load(path)

    assert config.port == 2101
    assert config.mountpoint == "TEST"
    assert config.password == "unit-test-password-$-#"
    rendered = repr(config)
    for secret in (
        config.host,
        config.mountpoint,
        config.username,
        config.password,
    ):
        assert secret not in rendered


@pytest.mark.parametrize("mode", [0o644, 0o640, 0o400])
def test_private_config_requires_exact_0600(tmp_path, mode):
    path = write_config(tmp_path / "g90-ntrip.env")
    path.chmod(mode)

    with pytest.raises(NtripConfigError, match="mode must be exactly 0600"):
        load(path)


def test_private_config_rejects_symlink_duplicate_and_unknown_keys(tmp_path):
    target = write_config(tmp_path / "target.env")
    symlink = tmp_path / "symlink.env"
    symlink.symlink_to(target)
    with pytest.raises(NtripConfigError, match="cannot be opened"):
        load(symlink)

    duplicate = write_config(
        tmp_path / "duplicate.env", extra_lines=("NTRIP_PORT=2102",)
    )
    with pytest.raises(NtripConfigError, match="duplicate key NTRIP_PORT"):
        load(duplicate)

    unknown = write_config(tmp_path / "unknown.env", extra_lines=("OTHER=x",))
    with pytest.raises(NtripConfigError, match="unsupported key OTHER"):
        load(unknown)


def test_private_config_fails_closed_on_expiry_or_serial_mismatch(tmp_path):
    expired = write_config(
        tmp_path / "expired.env",
        overrides={"NTRIP_EXPIRES_AT_LOCAL": "2026-08-26 00:00:00 UTC"},
    )
    with pytest.raises(NtripConfigError, match="credentials are expired"):
        load(expired, now=datetime.fromisoformat("2026-08-27T00:00:00+00:00"))

    mismatch = write_config(
        tmp_path / "mismatch.env",
        overrides={"G90_COM2_DEVICE": "/dev/ttyUSB0"},
    )
    with pytest.raises(NtripConfigError, match="launch contract"):
        load(mismatch)


def test_gga_extraction_keeps_partial_data_and_ignores_other_sentences():
    data = (
        b"noise\r\n"
        b"$GNTHS,12.3,A*00\r\n"
        b"$GNGGA,123519,2232.0,N,11401.0,E,4,18,0.7,1.0,M,0.0,M,,*00\r\n"
        b"$GPGGA,partial"
    )
    sentences, remainder = extract_gga_sentences(data)

    assert sentences == [
        "$GNGGA,123519,2232.0,N,11401.0,E,4,18,0.7,1.0,M,0.0,M,,*00"
    ]
    assert remainder == b"$GPGGA,partial"


class FakeSerial:
    def __init__(self, chunks=()):
        self.chunks = deque(chunks)
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, _size):
        if not self.chunks:
            time.sleep(0.001)
            return b""
        return self.chunks.popleft()

    def write(self, value):
        self.writes.append(bytes(value))
        return len(value)

    def close(self):
        self.closed = True


class FakeNtripClient:
    def __init__(self):
        self.sentences = []
        self.disconnected = False
        self._packets = deque([[b"\xd3\x00\x00\x47\xea\x4b"], []])

    def connect(self):
        return True

    def disconnect(self):
        self.disconnected = True

    def send_nmea(self, sentence):
        self.sentences.append(sentence)

    def recv_rtcm(self):
        return self._packets.popleft() if self._packets else []


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true")


def test_worker_relays_gga_and_rtcm_and_closes_owned_connections(tmp_path):
    config = load(write_config(tmp_path / "g90-ntrip.env"))
    serial_port = FakeSerial(
        [b"$GNGGA,123519,2232.0,N,11401.0,E,4,18,0.7,1.0,M,0.0,M,,*00\r\n"]
    )
    client = FakeNtripClient()
    worker = G90NtripRelayWorker(
        config,
        SERIAL_DEVICE,
        115200,
        serial_factory=lambda _device, _baud: serial_port,
        ntrip_factory=lambda _config: client,
        reconnect_wait_sec=0.01,
    )

    worker.start()
    wait_until(lambda: worker.snapshot().rtcm_packet_count == 1)
    wait_until(lambda: worker.snapshot().gga_forwarded_count == 1)
    snapshot = worker.snapshot()
    assert snapshot.worker_alive
    assert snapshot.serial_open
    assert snapshot.caster_connected
    assert snapshot.rtcm_byte_count == 6
    assert serial_port.writes == [b"\xd3\x00\x00\x47\xea\x4b"]
    assert client.sentences[0].startswith("$GNGGA,")

    assert worker.stop()
    assert serial_port.closed
    assert client.disconnected
    assert not worker.snapshot().worker_alive


def test_worker_retries_a_transient_serial_open_failure(tmp_path):
    config = load(write_config(tmp_path / "g90-ntrip.env"))
    serial_port = FakeSerial()
    attempts = []

    def serial_factory(_device, _baud):
        attempts.append(1)
        if len(attempts) == 1:
            raise OSError("test failure")
        return serial_port

    worker = G90NtripRelayWorker(
        config,
        SERIAL_DEVICE,
        115200,
        serial_factory=serial_factory,
        ntrip_factory=lambda _config: FakeNtripClient(),
        reconnect_wait_sec=0.01,
    )
    worker.start()
    wait_until(lambda: worker.snapshot().serial_open)
    snapshot = worker.snapshot()
    assert snapshot.serial_error_count == 1
    assert len(attempts) == 2
    assert worker.stop()


def test_default_client_accepts_g90_high_precision_gga(tmp_path):
    config = load(write_config(tmp_path / "g90-ntrip.env"))
    client = G90NtripRelayWorker._default_ntrip_factory(config)
    sentence = (
        "$GNGGA,014958.40,3120.38818036,N,12129.69966861,E,1,19,1.1,"
        "-1.5424,M,11.6785,M,,*57\r\n"
    )

    assert len(sentence) > 82
    assert client.nmea_parser.nmea_max_length == 258
    assert client.nmea_parser.is_valid_sentence(sentence)


def test_diagnostics_are_sanitized_and_distinguish_indoor_waiting():
    snapshot = RelaySnapshot(
        worker_alive=True,
        serial_open=True,
        caster_connected=True,
        credential_expired=False,
        gga_received_count=5,
        gga_forwarded_count=5,
        rtcm_packet_count=0,
        rtcm_byte_count=0,
        serial_error_count=0,
        ntrip_error_count=0,
        ntrip_connect_attempt_count=1,
        last_gga_age_sec=0.1,
        last_rtcm_age_sec=None,
        last_error="none",
    )
    status = diagnostic_status(
        "/g90/g90_ntrip_relay",
        SERIAL_DEVICE,
        snapshot,
        rtcm_freshness_sec=5.0,
    )

    assert status.level == DiagnosticStatus.WARN
    assert "waiting for fresh RTCM" in status.message
    rendered = status.message + " ".join(
        f"{item.key}={item.value}" for item in status.values
    )
    for secret in (
        "unit-test-user",
        "unit-test-password",
        "caster.invalid",
        "TEST",
    ):
        assert secret not in rendered
