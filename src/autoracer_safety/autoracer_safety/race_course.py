from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math

from autoracer_planning.fixed_course import CourseSample


@dataclass(frozen=True)
class CourseMatch:
    index: int
    cross_track_m: float
    heading_error_rad: float
    boundary_margin_m: float


class CourseIndex:
    def __init__(
        self,
        samples: list[CourseSample],
        vehicle_width_m: float,
        vehicle_length_m: float,
    ) -> None:
        if len(samples) < 2:
            raise ValueError("fixed course requires at least two samples")
        self._samples = samples
        self._half_width = vehicle_width_m * 0.5
        self._half_length = vehicle_length_m * 0.5
        self._cells: dict[tuple[int, int], list[int]] = defaultdict(list)
        for index, point in enumerate(samples):
            self._cells[self._cell(point.x, point.y)].append(index)
        self._last_index: int | None = None

    @staticmethod
    def _cell(x: float, y: float) -> tuple[int, int]:
        return (math.floor(x / 10.0), math.floor(y / 10.0))

    def reset(self) -> None:
        self._last_index = None

    def _candidate_indices(self, x: float, y: float) -> list[int]:
        if self._last_index is not None:
            return list(
                range(
                    max(0, self._last_index - 20),
                    min(len(self._samples), self._last_index + 401),
                )
            )
        cell_x, cell_y = self._cell(x, y)
        indices: list[int] = []
        for radius in range(4):
            for delta_x in range(-radius, radius + 1):
                for delta_y in range(-radius, radius + 1):
                    indices.extend(
                        self._cells.get((cell_x + delta_x, cell_y + delta_y), ())
                    )
            if indices:
                break
        return indices

    def match(self, x: float, y: float, yaw: float) -> CourseMatch | None:
        candidates = self._candidate_indices(x, y)
        if not candidates:
            return None
        index = min(
            candidates,
            key=lambda candidate: (self._samples[candidate].x - x) ** 2
            + (self._samples[candidate].y - y) ** 2,
        )
        if self._last_index is not None:
            index = max(index, self._last_index)
        self._last_index = index
        point = self._samples[index]
        delta_x = x - point.x
        delta_y = y - point.y
        cross_track = -math.sin(point.yaw) * delta_x + math.cos(point.yaw) * delta_y
        heading_error = math.atan2(
            math.sin(yaw - point.yaw), math.cos(yaw - point.yaw)
        )
        heading_sweep = self._half_length * abs(math.sin(heading_error))
        left_margin = point.left_offset - self._half_width - heading_sweep - cross_track
        right_margin = point.right_offset - self._half_width - heading_sweep + cross_track
        return CourseMatch(
            index,
            cross_track,
            heading_error,
            min(left_margin, right_margin),
        )
