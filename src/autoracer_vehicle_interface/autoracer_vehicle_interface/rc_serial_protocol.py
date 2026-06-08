from dataclasses import dataclass
import struct


CONTROL_FRAME_LEN = 11
TELEMETRY_FRAME_LEN = 24
FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
CMD_SPEED_CONTROL = 0x00
CMD_STOP_BIT = 0x80
MILLI_SCALE = 1000.0


@dataclass(frozen=True)
class TelemetryFrame:
    flag: int
    vx_mps: float
    vy_mps: float
    wz_rad_s: float
    battery_mv: int


def bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def _to_int16_milli(value):
    scaled = int(round(float(value) * MILLI_SCALE))
    return max(-32768, min(32767, scaled))


def encode_control_frame(vx_mps, vy_mps, wz_rad_s, stop=False):
    payload = bytearray(
        [
            FRAME_HEAD,
            CMD_SPEED_CONTROL,
            CMD_STOP_BIT if stop else 0x00,
        ]
    )
    payload.extend(struct.pack(">h", _to_int16_milli(vx_mps)))
    payload.extend(struct.pack(">h", _to_int16_milli(vy_mps)))
    payload.extend(struct.pack(">h", _to_int16_milli(wz_rad_s)))
    payload.append(bcc(payload))
    payload.append(FRAME_TAIL)
    return bytes(payload)


def _read_int16(frame, offset):
    return struct.unpack(">h", bytes(frame[offset : offset + 2]))[0]


def decode_telemetry_frame(frame):
    if len(frame) != TELEMETRY_FRAME_LEN:
        raise ValueError(f"telemetry frame must be {TELEMETRY_FRAME_LEN} bytes")
    if frame[0] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("invalid telemetry frame delimiters")
    if bcc(frame[:22]) != frame[22]:
        raise ValueError("invalid telemetry frame checksum")

    return TelemetryFrame(
        flag=int(frame[1]),
        vx_mps=_read_int16(frame, 2) / MILLI_SCALE,
        vy_mps=_read_int16(frame, 4) / MILLI_SCALE,
        wz_rad_s=_read_int16(frame, 6) / MILLI_SCALE,
        battery_mv=_read_int16(frame, 20),
    )


def pop_telemetry_frame(buffer):
    while buffer:
        if buffer[0] != FRAME_HEAD:
            del buffer[0]
            continue
        if len(buffer) < TELEMETRY_FRAME_LEN:
            return None
        candidate = bytes(buffer[:TELEMETRY_FRAME_LEN])
        if candidate[-1] != FRAME_TAIL:
            del buffer[0]
            continue
        del buffer[:TELEMETRY_FRAME_LEN]
        return decode_telemetry_frame(candidate)
    return None
