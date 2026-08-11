from __future__ import annotations

from dataclasses import dataclass, field as dataclass_field
import math
import threading

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
    last_future_offset_sec: float = 0.0
    max_future_offset_sec: float = 0.0
    _lock: threading.RLock = dataclass_field(
        default_factory=threading.RLock, init=False, repr=False, compare=False
    )

    def update(self, message, receipt_sec: float, source_stamp=None) -> bool:
        with self._lock:
            stamp_sec = receipt_sec if source_stamp is None else stamp_to_sec(source_stamp)
            if not math.isfinite(stamp_sec) or stamp_sec < 0.0:
                return False
            if stamp_sec + 1.0e-9 < self.source_stamp_sec:
                self.rejected_out_of_order += 1
                return False
            if stamp_sec > receipt_sec + 0.05:
                self.rejected_future += 1
                self.last_future_offset_sec = stamp_sec - receipt_sec
                self.max_future_offset_sec = max(
                    self.max_future_offset_sec, self.last_future_offset_sec
                )
                return False
            # Publish the message reference last.  A concurrent freshness reader
            # can therefore see either the complete previous sample or the
            # complete new sample, never a new message with old timing metadata.
            self.receipt_sec = receipt_sec
            self.source_stamp_sec = stamp_sec
            self.sequence += 1
            self.message = message
            return True

    def fresh(
        self,
        now_sec: float,
        receipt_timeout_sec: float,
        source_timeout_sec: float | None = None,
    ) -> bool:
        """Return whether delivery and measurement time are both healthy.

        ``receipt_timeout_sec`` supervises the transport/processing heartbeat.
        ``source_timeout_sec`` supervises measurement age and defaults to the
        receipt budget for backwards compatibility.  Keeping these budgets
        separate avoids counting deterministic sensor-processing latency as a
        transport outage while still rejecting newly delivered stale samples.
        """
        with self._lock:
            source_timeout = (
                receipt_timeout_sec
                if source_timeout_sec is None
                else source_timeout_sec
            )
            return (
                self.message is not None
                and now_sec + 0.05 >= self.receipt_sec
                and now_sec + 0.05 >= self.source_stamp_sec
                and now_sec - self.receipt_sec <= receipt_timeout_sec
                and now_sec - self.source_stamp_sec <= source_timeout
            )

    def clear(self) -> None:
        with self._lock:
            self.message = None
            self.receipt_sec = -1.0
            self.source_stamp_sec = -1.0
            self.sequence = 0
