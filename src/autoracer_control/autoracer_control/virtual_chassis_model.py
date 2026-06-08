from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class RawControlCommand:
    steering_tire_angle: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0


@dataclass(frozen=True)
class VirtualChassisState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    delta_actual: float = 0.0
    a_actual: float = 0.0


@dataclass(frozen=True)
class VirtualChassisConfig:
    wheel_base: float = 1.9
    max_steer: float = 0.488
    max_steer_rate: float = 1.0
    steer_tau: float = 0.27
    actuator_input_delay: float = 0.15
    max_speed: float = 2.0
    max_acc: float = 1.0
    min_acc: float = -2.0
    max_jerk: float = 2.0
    min_jerk: float = -4.0
    acc_tau: float = 0.20
    dt: float = 0.05
    fixed_speed_mode: bool = False
    fixed_speed: float = 1.0


class VirtualChassisModel:
    def __init__(
        self,
        config: VirtualChassisConfig,
        initial_state: VirtualChassisState | None = None,
    ) -> None:
        self.config = config
        self.state = initial_state or VirtualChassisState()
        delay_steps = max(0, int(math.ceil(config.actuator_input_delay / config.dt)))
        zeros = [RawControlCommand() for _ in range(delay_steps)]
        self._delay_buffer = deque(zeros)

    def step(self, command: RawControlCommand) -> VirtualChassisState:
        delayed = self._delay(command)
        cfg = self.config
        prev = self.state

        delta_target = _clamp(delayed.steering_tire_angle, -cfg.max_steer, cfg.max_steer)
        steer_alpha = cfg.dt / cfg.steer_tau if cfg.steer_tau > 0.0 else 1.0
        delta_desired = prev.delta_actual + steer_alpha * (delta_target - prev.delta_actual)
        delta_step = _clamp(
            delta_desired - prev.delta_actual,
            -cfg.max_steer_rate * cfg.dt,
            cfg.max_steer_rate * cfg.dt,
        )
        delta_actual = prev.delta_actual + delta_step

        if cfg.fixed_speed_mode:
            a_actual = 0.0
            v_next = _clamp(cfg.fixed_speed, 0.0, cfg.max_speed)
        else:
            acc_target = _clamp(delayed.acceleration, cfg.min_acc, cfg.max_acc)
            acc_alpha = cfg.dt / cfg.acc_tau if cfg.acc_tau > 0.0 else 1.0
            acc_desired = prev.a_actual + acc_alpha * (acc_target - prev.a_actual)
            acc_step = _clamp(
                acc_desired - prev.a_actual,
                cfg.min_jerk * cfg.dt,
                cfg.max_jerk * cfg.dt,
            )
            a_actual = prev.a_actual + acc_step
            v_next = _clamp(prev.v + a_actual * cfg.dt, 0.0, cfg.max_speed)

        x_next = prev.x + v_next * math.cos(prev.yaw) * cfg.dt
        y_next = prev.y + v_next * math.sin(prev.yaw) * cfg.dt
        yaw_next = prev.yaw + v_next / cfg.wheel_base * math.tan(delta_actual) * cfg.dt

        self.state = replace(
            prev,
            x=x_next,
            y=y_next,
            yaw=yaw_next,
            v=v_next,
            delta_actual=delta_actual,
            a_actual=a_actual,
        )
        return self.state

    def _delay(self, command: RawControlCommand) -> RawControlCommand:
        if not self._delay_buffer:
            return command
        self._delay_buffer.append(command)
        return self._delay_buffer.popleft()
