import pytest

from autoracer_vehicle_interface.rc_serial_protocol import (
    FRAME_HEAD,
    FRAME_TAIL,
    bcc,
    decode_telemetry_frame,
    encode_control_frame,
    pop_telemetry_frame,
)


def test_encode_control_frame_scales_signed_motion_values():
    frame = encode_control_frame(1.234, 0.0, -0.456)

    assert len(frame) == 11
    assert frame[0] == FRAME_HEAD
    assert frame[1] == 0x00
    assert frame[2] == 0x00
    assert frame[3:5] == b"\x04\xd2"
    assert frame[5:7] == b"\x00\x00"
    assert frame[7:9] == b"\xfe8"
    assert frame[9] == bcc(frame[:9])
    assert frame[10] == FRAME_TAIL


def test_encode_control_frame_sets_stop_bit():
    frame = encode_control_frame(0.0, 0.0, 0.0, stop=True)

    assert frame[2] == 0x80
    assert frame[9] == bcc(frame[:9])


def test_decode_telemetry_frame_reads_current_firmware_layout():
    frame = bytearray(24)
    frame[0] = FRAME_HEAD
    frame[1] = 1
    frame[2:4] = b"\x00d"
    frame[4:6] = b"\x00\x00"
    frame[6:8] = b"\xff\xce"
    frame[20:22] = b"\x2e\xe0"
    frame[22] = bcc(frame[:22])
    frame[23] = FRAME_TAIL

    telemetry = decode_telemetry_frame(bytes(frame))

    assert telemetry.flag == 1
    assert telemetry.vx_mps == pytest.approx(0.1)
    assert telemetry.vy_mps == pytest.approx(0.0)
    assert telemetry.wz_rad_s == pytest.approx(-0.05)
    assert telemetry.battery_mv == 12000


def test_pop_telemetry_frame_resynchronizes_noise():
    frame = bytearray(24)
    frame[0] = FRAME_HEAD
    frame[1] = 1
    frame[22] = bcc(frame[:22])
    frame[23] = FRAME_TAIL
    buffer = bytearray(b"noise")
    buffer.extend(frame)

    telemetry = pop_telemetry_frame(buffer)

    assert telemetry.flag == 1
    assert buffer == bytearray()
