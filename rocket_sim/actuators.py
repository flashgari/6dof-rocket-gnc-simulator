"""Thrust-vector-control actuator dynamics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control import TVCCommand, TVCController
from .math3d import Vector, cross, norm, unit, v_scale
from .models import RocketParams


@dataclass(frozen=True)
class GimbalActuatorConfig:
    """Finite-bandwidth two-axis TVC actuator model."""

    max_gimbal_rad: float
    max_rate_radps: float
    time_constant_s: float


@dataclass(frozen=True)
class GimbalActuatorState:
    gimbal_x_rad: float = 0.0
    gimbal_y_rad: float = 0.0

    @property
    def total_gimbal_rad(self) -> float:
        return math.sqrt(self.gimbal_x_rad**2 + self.gimbal_y_rad**2)


@dataclass(frozen=True)
class GimbalActuatorOutput:
    state: GimbalActuatorState
    achieved_command: TVCCommand
    command_error_rad: float
    rate_limited: bool
    position_limited: bool


def _clamp_vector(x: float, y: float, max_norm: float) -> tuple[float, float, bool]:
    magnitude = math.sqrt(x * x + y * y)
    if magnitude <= max_norm or magnitude == 0.0:
        return x, y, False
    scale = max_norm / magnitude
    return x * scale, y * scale, True


def tvc_command_from_gimbal(
    controller: TVCController,
    rocket: RocketParams,
    requested: TVCCommand,
    gimbal_x_rad: float,
    gimbal_y_rad: float,
    actuator_limited: bool,
) -> TVCCommand:
    """Build the plant command from the achieved nozzle direction."""

    direction = unit((math.tan(gimbal_x_rad), math.tan(gimbal_y_rad), 1.0))
    achievable_force = v_scale(rocket.thrust_n, direction)
    achievable_torque = cross(controller.engine_position_body_m, achievable_force)
    return TVCCommand(
        thrust_direction_body=direction,
        gimbal_x_rad=gimbal_x_rad,
        gimbal_y_rad=gimbal_y_rad,
        requested_torque_body=requested.requested_torque_body,
        achievable_torque_body=achievable_torque,
        saturated=requested.saturated or actuator_limited,
    )


class GimbalActuator:
    """Rate-limited first-order gimbal servo.

    The controller supplies a desired gimbal angle. The actuator responds with
    finite bandwidth, finite slew rate, and a hard angular envelope. The plant
    receives the achieved angle, not the requested angle.
    """

    def __init__(self, config: GimbalActuatorConfig, initial_state: GimbalActuatorState | None = None):
        if config.max_gimbal_rad <= 0.0:
            raise ValueError("max_gimbal_rad must be positive.")
        if config.max_rate_radps <= 0.0:
            raise ValueError("max_rate_radps must be positive.")
        if config.time_constant_s <= 0.0:
            raise ValueError("time_constant_s must be positive.")
        self.config = config
        self.state = initial_state or GimbalActuatorState()

    def update(self, requested: TVCCommand, rocket: RocketParams, controller: TVCController, dt_s: float) -> GimbalActuatorOutput:
        if dt_s < 0.0:
            raise ValueError("dt_s cannot be negative.")

        target_x, target_y, target_limited = _clamp_vector(
            requested.gimbal_x_rad,
            requested.gimbal_y_rad,
            self.config.max_gimbal_rad,
        )

        if dt_s == 0.0:
            command_error = math.sqrt((target_x - self.state.gimbal_x_rad) ** 2 + (target_y - self.state.gimbal_y_rad) ** 2)
            achieved = tvc_command_from_gimbal(controller, rocket, requested, self.state.gimbal_x_rad, self.state.gimbal_y_rad, target_limited)
            return GimbalActuatorOutput(self.state, achieved, command_error, False, target_limited)

        commanded_rate_x = (target_x - self.state.gimbal_x_rad) / self.config.time_constant_s
        commanded_rate_y = (target_y - self.state.gimbal_y_rad) / self.config.time_constant_s
        rate_x, rate_y, rate_limited = _clamp_vector(commanded_rate_x, commanded_rate_y, self.config.max_rate_radps)

        next_x = self.state.gimbal_x_rad + rate_x * dt_s
        next_y = self.state.gimbal_y_rad + rate_y * dt_s
        next_x, next_y, position_limited = _clamp_vector(next_x, next_y, self.config.max_gimbal_rad)
        self.state = GimbalActuatorState(next_x, next_y)

        command_error = math.sqrt((target_x - next_x) ** 2 + (target_y - next_y) ** 2)
        actuator_limited = target_limited or rate_limited or position_limited
        achieved = tvc_command_from_gimbal(controller, rocket, requested, next_x, next_y, actuator_limited)
        return GimbalActuatorOutput(self.state, achieved, command_error, rate_limited, target_limited or position_limited)


def max_tvc_torque_nm(rocket: RocketParams, controller: TVCController, max_gimbal_rad: float) -> float:
    lever_arm = norm(controller.engine_position_body_m)
    return lever_arm * rocket.thrust_n * math.sin(max_gimbal_rad)
