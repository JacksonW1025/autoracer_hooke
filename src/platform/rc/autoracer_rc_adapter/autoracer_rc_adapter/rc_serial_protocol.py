# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Codec for RCCar-Firmware's fixed ROS UART frames."""

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List


FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
CONTROL_FRAME_LEN = 11
TELEMETRY_FRAME_LEN = 24
_SCALE = 1000.0


@dataclass(frozen=True)
class Telemetry:
    """Motion and battery values carried by one firmware telemetry frame."""

    enabled: bool
    vx_mps: float
    vy_mps: float
    wz_rad_s: float
    battery_v: float


def bcc(data: Iterable[int]) -> int:
    """Return the firmware's byte-wise XOR checksum."""

    checksum = 0
    for value in data:
        checksum ^= int(value)
    return checksum


def _scaled_int16(value: float) -> int:
    if not math.isfinite(value):
        return 0
    return max(-32768, min(32767, int(value * _SCALE)))


def encode_control_frame(vx_mps: float, vy_mps: float, wz_rad_s: float, stop: bool) -> bytes:
    """Encode the firmware's 11-byte Ackermann motion command."""

    frame = bytearray(CONTROL_FRAME_LEN)
    frame[0] = FRAME_HEAD
    frame[1] = 0
    frame[2] = 0x80 if stop else 0
    frame[3:5] = struct.pack(">h", _scaled_int16(vx_mps))
    frame[5:7] = struct.pack(">h", _scaled_int16(vy_mps))
    frame[7:9] = struct.pack(">h", _scaled_int16(wz_rad_s))
    frame[9] = bcc(frame[:9])
    frame[10] = FRAME_TAIL
    return bytes(frame)


def decode_telemetry_frame(frame: bytes) -> Telemetry:
    """Validate and decode one 24-byte firmware telemetry frame."""

    if len(frame) != TELEMETRY_FRAME_LEN:
        raise ValueError("telemetry frame must be 24 bytes")
    if frame[0] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("telemetry frame delimiters are invalid")
    if frame[22] != bcc(frame[:22]):
        raise ValueError("telemetry checksum is invalid")
    vx_raw, vy_raw, wz_raw = struct.unpack(">hhh", frame[2:8])
    battery_raw = struct.unpack(">h", frame[20:22])[0]
    return Telemetry(
        enabled=bool(frame[1]),
        vx_mps=vx_raw / _SCALE,
        vy_mps=vy_raw / _SCALE,
        wz_rad_s=wz_raw / _SCALE,
        battery_v=battery_raw / _SCALE,
    )


class TelemetryParser:
    """Incrementally recover valid telemetry frames from a UART byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[Telemetry]:
        self._buffer.extend(data)
        decoded = []
        while self._buffer:
            try:
                head = self._buffer.index(FRAME_HEAD)
            except ValueError:
                self._buffer.clear()
                break
            if head:
                del self._buffer[:head]
            if len(self._buffer) < TELEMETRY_FRAME_LEN:
                break
            candidate = bytes(self._buffer[:TELEMETRY_FRAME_LEN])
            try:
                decoded.append(decode_telemetry_frame(candidate))
            except ValueError:
                del self._buffer[0]
            else:
                del self._buffer[:TELEMETRY_FRAME_LEN]
        return decoded
