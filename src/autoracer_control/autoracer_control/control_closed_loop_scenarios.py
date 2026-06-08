"""Lightweight in-Python scenario contracts for the closed-loop bench."""

from __future__ import annotations

from dataclasses import dataclass
import math


SMOKE = "smoke"
FULL_VALIDATION = "full_validation"


@dataclass(frozen=True)
class SegmentSpec:
    name: str
    start_s_m: float
    end_s_m: float
    reference_velocity_mps: float

    @property
    def s_range_m(self) -> tuple[float, float]:
        return (self.start_s_m, self.end_s_m)


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    scenario_type: str
    path_kind: str
    path_length_m: float
    initial_y: float
    initial_yaw: float
    initial_v: float
    max_duration_sec: float
    max_speed: float
    segments: tuple[SegmentSpec, ...]
    completion_threshold: float = 0.98
    radius_m: float = 20.0
    arc_angle_rad: float = math.pi * 0.5
    s_curve_amplitude_m: float = 1.0
    s_curve_wavelength_m: float = 40.0
    point_spacing_m: float = 0.5


def _single_segment(name: str, length_m: float, velocity_mps: float) -> tuple[SegmentSpec, ...]:
    return (SegmentSpec(name=name, start_s_m=0.0, end_s_m=length_m, reference_velocity_mps=velocity_mps),)


SMOKE_SCENARIOS = (
    "straight_lateral_offset",
    "straight_heading_offset",
    "constant_radius_left",
    "s_curve",
    "longitudinal_speed_step",
    "speed_regime_sweep",
)
FULL_VALIDATION_SCENARIOS = (
    "straight_120m_v1",
    "arc_r20_90deg_v1",
    "s_curve_100m_v1",
    "speed_step_120m_v1",
)


SCENARIO_SPECS: dict[str, ScenarioSpec] = {
    "straight_lateral_offset": ScenarioSpec(
        name="straight_lateral_offset",
        scenario_type=SMOKE,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=-0.5,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=12.0,
        max_speed=2.0,
        segments=_single_segment("straight_1mps", 120.0, 1.0),
    ),
    "straight_heading_offset": ScenarioSpec(
        name="straight_heading_offset",
        scenario_type=SMOKE,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=0.0,
        initial_yaw=math.radians(5.0),
        initial_v=1.0,
        max_duration_sec=12.0,
        max_speed=2.0,
        segments=_single_segment("straight_1mps", 120.0, 1.0),
    ),
    "constant_radius_left": ScenarioSpec(
        name="constant_radius_left",
        scenario_type=SMOKE,
        path_kind="arc_left",
        path_length_m=20.0 * 1.6,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=12.0,
        max_speed=2.0,
        segments=_single_segment("arc_r20_1mps", 20.0 * 1.6, 1.0),
        arc_angle_rad=1.6,
    ),
    "s_curve": ScenarioSpec(
        name="s_curve",
        scenario_type=SMOKE,
        path_kind="s_curve",
        path_length_m=100.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=14.0,
        max_speed=2.0,
        segments=_single_segment("s_curve_1mps", 100.0, 1.0),
    ),
    "longitudinal_speed_step": ScenarioSpec(
        name="longitudinal_speed_step",
        scenario_type=SMOKE,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=0.5,
        max_duration_sec=18.0,
        max_speed=2.0,
        segments=(
            SegmentSpec("0p5mps", 0.0, 8.0, 0.5),
            SegmentSpec("1mps", 8.0, 20.0, 1.0),
            SegmentSpec("2mps", 20.0, 120.0, 2.0),
        ),
    ),
    "speed_regime_sweep": ScenarioSpec(
        name="speed_regime_sweep",
        scenario_type=SMOKE,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=0.5,
        max_duration_sec=24.0,
        max_speed=3.0,
        segments=(
            SegmentSpec("0p5mps", 0.0, 5.0, 0.5),
            SegmentSpec("1mps", 5.0, 15.0, 1.0),
            SegmentSpec("2mps", 15.0, 35.0, 2.0),
            SegmentSpec("3mps", 35.0, 120.0, 3.0),
        ),
    ),
    "straight_120m_v1": ScenarioSpec(
        name="straight_120m_v1",
        scenario_type=FULL_VALIDATION,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=150.0,
        max_speed=2.0,
        segments=_single_segment("straight_1mps", 120.0, 1.0),
    ),
    "arc_r20_90deg_v1": ScenarioSpec(
        name="arc_r20_90deg_v1",
        scenario_type=FULL_VALIDATION,
        path_kind="arc_left",
        path_length_m=20.0 * math.pi * 0.5,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=45.0,
        max_speed=2.0,
        segments=_single_segment("arc_r20_1mps", 20.0 * math.pi * 0.5, 1.0),
        arc_angle_rad=math.pi * 0.5,
    ),
    "s_curve_100m_v1": ScenarioSpec(
        name="s_curve_100m_v1",
        scenario_type=FULL_VALIDATION,
        path_kind="s_curve",
        path_length_m=100.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=1.0,
        max_duration_sec=130.0,
        max_speed=2.0,
        segments=_single_segment("s_curve_1mps", 100.0, 1.0),
    ),
    "speed_step_120m_v1": ScenarioSpec(
        name="speed_step_120m_v1",
        scenario_type=FULL_VALIDATION,
        path_kind="straight",
        path_length_m=120.0,
        initial_y=0.0,
        initial_yaw=0.0,
        initial_v=0.5,
        max_duration_sec=155.0,
        max_speed=2.0,
        segments=(
            SegmentSpec("0p5mps", 0.0, 30.0, 0.5),
            SegmentSpec("1mps", 30.0, 70.0, 1.0),
            SegmentSpec("2mps", 70.0, 120.0, 2.0),
        ),
    ),
}


def get_scenario_spec(name: str) -> ScenarioSpec:
    try:
        return SCENARIO_SPECS[name]
    except KeyError as exc:
        raise ValueError(f"unsupported closed-loop scenario: {name}") from exc


def velocity_at_s(spec: ScenarioSpec, station_m: float) -> float:
    for segment in spec.segments:
        if segment.start_s_m <= station_m < segment.end_s_m:
            return segment.reference_velocity_mps
    if spec.segments and station_m >= spec.segments[-1].end_s_m:
        return spec.segments[-1].reference_velocity_mps
    return spec.initial_v
