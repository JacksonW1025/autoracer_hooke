from __future__ import annotations

from dataclasses import dataclass
import math

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def finite_stamp(stamp) -> bool:
    value = stamp_to_sec(stamp)
    return math.isfinite(value) and value >= 0.0


@dataclass
class TimedInput:
    message: object | None = None
    receipt_sec: float = -1.0
    source_stamp_sec: float = -1.0
    sequence: int = 0
    rejected_out_of_order: int = 0
    rejected_future: int = 0

    def update(self, message, receipt_sec: float, source_stamp=None) -> bool:
        stamp_sec = receipt_sec if source_stamp is None else stamp_to_sec(source_stamp)
        if not math.isfinite(stamp_sec) or stamp_sec < 0.0:
            return False
        if stamp_sec + 1.0e-9 < self.source_stamp_sec:
            self.rejected_out_of_order += 1
            return False
        if stamp_sec > receipt_sec + 0.05:
            self.rejected_future += 1
            return False
        self.message = message
        self.receipt_sec = receipt_sec
        self.source_stamp_sec = stamp_sec
        self.sequence += 1
        return True

    def fresh(self, now_sec: float, timeout_sec: float) -> bool:
        return (
            self.message is not None
            and now_sec + 0.05 >= self.receipt_sec
            and now_sec + 0.05 >= self.source_stamp_sec
            and now_sec - self.receipt_sec <= timeout_sec
            and now_sec - self.source_stamp_sec <= timeout_sec
        )

    def clear(self) -> None:
        self.message = None
        self.receipt_sec = -1.0
        self.source_stamp_sec = -1.0
        self.sequence = 0
