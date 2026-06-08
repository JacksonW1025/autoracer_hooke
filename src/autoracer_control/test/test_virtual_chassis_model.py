import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from autoracer_control.virtual_chassis_model import (
    RawControlCommand,
    VirtualChassisConfig,
    VirtualChassisModel,
    VirtualChassisState,
)


def make_model(**overrides):
    initial_state = overrides.pop("initial_state", None)
    params = {
        "wheel_base": 2.0,
        "max_steer": 0.4,
        "max_steer_rate": 100.0,
        "steer_tau": 0.1,
        "actuator_input_delay": 0.0,
        "max_speed": 5.0,
        "max_acc": 5.0,
        "min_acc": -5.0,
        "max_jerk": 100.0,
        "min_jerk": -100.0,
        "acc_tau": 0.1,
        "dt": 0.1,
    }
    params.update(overrides)
    return VirtualChassisModel(VirtualChassisConfig(**params), initial_state)


def command(delta=0.0, velocity=0.0, acceleration=0.0):
    return RawControlCommand(
        steering_tire_angle=float(delta),
        velocity=float(velocity),
        acceleration=float(acceleration),
    )


def test_steering_is_limited_to_configured_max_angle():
    model = make_model(max_steer=0.25)

    state = model.step(command(delta=2.0))

    assert state.delta_actual == pytest.approx(0.25)


def test_steering_rate_limit_bounds_single_step_change():
    model = make_model(max_steer=1.0, max_steer_rate=0.2, steer_tau=0.1, dt=0.1)

    state = model.step(command(delta=1.0))

    assert state.delta_actual == pytest.approx(0.02)


def test_steering_first_order_response_moves_toward_target():
    model = make_model(max_steer=1.0, steer_tau=1.0, dt=0.1)

    state = model.step(command(delta=0.5))

    assert state.delta_actual == pytest.approx(0.05)


def test_actuator_input_delay_holds_command_until_delay_elapsed():
    model = make_model(
        max_steer=1.0,
        steer_tau=0.1,
        actuator_input_delay=0.2,
        dt=0.1,
    )

    first = model.step(command(delta=0.3))
    second = model.step(command(delta=0.3))
    third = model.step(command(delta=0.3))

    assert first.delta_actual == pytest.approx(0.0)
    assert second.delta_actual == pytest.approx(0.0)
    assert third.delta_actual == pytest.approx(0.3)


def test_acceleration_is_limited_to_configured_bounds():
    model = make_model(max_acc=1.0, min_acc=-1.0, acc_tau=0.1)

    state = model.step(command(acceleration=10.0))

    assert state.a_actual == pytest.approx(1.0)


def test_jerk_limit_bounds_acceleration_change():
    model = make_model(max_acc=10.0, max_jerk=2.0, acc_tau=0.1, dt=0.1)

    state = model.step(command(acceleration=10.0))

    assert state.a_actual == pytest.approx(0.2)


def test_velocity_is_integrated_from_acceleration_not_set_from_velocity_command():
    model = make_model(max_acc=10.0, acc_tau=0.1, dt=0.1)

    state = model.step(command(velocity=10.0, acceleration=1.0))

    assert state.v == pytest.approx(0.1)
    assert state.v != pytest.approx(10.0)


def test_zero_steering_integrates_straight_motion():
    model = make_model(initial_state=VirtualChassisState(v=1.0), dt=0.1)

    state = model.step(command())

    assert state.x == pytest.approx(0.1)
    assert state.y == pytest.approx(0.0)
    assert state.yaw == pytest.approx(0.0)


def test_positive_steering_increases_yaw():
    model = make_model(
        initial_state=VirtualChassisState(v=1.0),
        max_steer=1.0,
        steer_tau=0.1,
        dt=0.1,
    )

    state = model.step(command(delta=0.2))

    assert state.yaw > 0.0
    assert math.isfinite(state.yaw)
