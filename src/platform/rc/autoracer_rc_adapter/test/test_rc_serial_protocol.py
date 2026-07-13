import math
import struct

import pytest

from autoracer_rc_adapter.rc_serial_protocol import (
    TelemetryParser,
    bcc,
    decode_telemetry_frame,
    encode_control_frame,
)


def telemetry_frame(vx=0.4, vy=0.0, wz=-0.2, battery_v=12.3):
    frame = bytearray(24)
    frame[0] = 0x7B
    frame[1] = 1
    frame[2:4] = struct.pack(">h", round(vx * 1000))
    frame[4:6] = struct.pack(">h", round(vy * 1000))
    frame[6:8] = struct.pack(">h", round(wz * 1000))
    frame[20:22] = struct.pack(">h", round(battery_v * 1000))
    frame[22] = bcc(frame[:22])
    frame[23] = 0x7D
    return bytes(frame)


def test_control_frame_contract():
    frame = encode_control_frame(1.0, 0.0, -0.25, stop=False)
    assert len(frame) == 11
    assert frame[0] == 0x7B
    assert frame[-1] == 0x7D
    assert frame[3:5] == b"\x03\xe8"
    assert frame[5:7] == b"\x00\x00"
    assert frame[7:9] == b"\xff\x06"
    assert frame[9] == bcc(frame[:9])


def test_stop_bit_is_explicit():
    assert encode_control_frame(0.0, 0.0, 0.0, stop=True)[2] == 0x80


def test_control_fields_saturate_to_signed_int16():
    frame = encode_control_frame(100.0, -100.0, math.inf, stop=False)
    assert frame[3:5] == b"\x7f\xff"
    assert frame[5:7] == b"\x80\x00"
    assert frame[7:9] == b"\x00\x00"


def test_decode_valid_telemetry():
    telemetry = decode_telemetry_frame(telemetry_frame())
    assert telemetry.enabled is True
    assert telemetry.vx_mps == pytest.approx(0.4)
    assert telemetry.vy_mps == pytest.approx(0.0)
    assert telemetry.wz_rad_s == pytest.approx(-0.2)
    assert telemetry.battery_v == pytest.approx(12.3)


@pytest.mark.parametrize("mutation", ["length", "head", "tail", "checksum"])
def test_decode_rejects_malformed_telemetry(mutation):
    frame = bytearray(telemetry_frame())
    if mutation == "length":
        frame.pop()
    elif mutation == "head":
        frame[0] = 0
    elif mutation == "tail":
        frame[-1] = 0
    else:
        frame[3] ^= 1
    with pytest.raises(ValueError):
        decode_telemetry_frame(bytes(frame))


def test_parser_handles_fragmented_frame():
    frame = telemetry_frame()
    parser = TelemetryParser()
    assert parser.feed(frame[:7]) == []
    assert parser.feed(frame[7:]) == [decode_telemetry_frame(frame)]


def test_parser_resynchronizes_and_returns_concatenated_frames():
    first = telemetry_frame(vx=0.1)
    second = telemetry_frame(vx=-0.2)
    parser = TelemetryParser()
    decoded = parser.feed(b"garbage" + first + second)
    assert [item.vx_mps for item in decoded] == pytest.approx([0.1, -0.2])


def test_parser_rejects_bad_checksum_and_recovers():
    bad = bytearray(telemetry_frame(vx=0.1))
    bad[22] ^= 1
    good = telemetry_frame(vx=0.3)
    decoded = TelemetryParser().feed(bytes(bad) + good)
    assert [item.vx_mps for item in decoded] == pytest.approx([0.3])
