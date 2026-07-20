"""Closed-loop simulation utilities."""

from __future__ import annotations

from collections.abc import Iterator

from .control import IdealTorqueController
from .dynamics import derivatives_with_external_torque
from .math3d import weighted_sum
from .models import Environment, RocketParams, State


def _state_plus_scaled_derivative(state: State, scale: float, derivative: tuple[float, ...]) -> State:
    return State.from_tuple(tuple(s + scale * ds for s, ds in zip(state.as_tuple(), derivative)))


def _controlled_derivatives(
    time_s: float,
    state: State,
    rocket: RocketParams,
    env: Environment,
    controller: IdealTorqueController,
) -> tuple[float, ...]:
    return derivatives_with_external_torque(time_s, state, rocket, env, controller.torque_body(state))


def rk4_controlled_step(
    time_s: float,
    state: State,
    dt_s: float,
    rocket: RocketParams,
    env: Environment,
    controller: IdealTorqueController,
) -> State:
    k1 = _controlled_derivatives(time_s, state, rocket, env, controller)
    k2 = _controlled_derivatives(time_s + 0.5 * dt_s, _state_plus_scaled_derivative(state, 0.5 * dt_s, k1), rocket, env, controller)
    k3 = _controlled_derivatives(time_s + 0.5 * dt_s, _state_plus_scaled_derivative(state, 0.5 * dt_s, k2), rocket, env, controller)
    k4 = _controlled_derivatives(time_s + dt_s, _state_plus_scaled_derivative(state, dt_s, k3), rocket, env, controller)
    delta = weighted_sum((dt_s / 6.0, dt_s / 3.0, dt_s / 3.0, dt_s / 6.0), (k1, k2, k3, k4))
    return State.from_tuple(tuple(s + ds for s, ds in zip(state.as_tuple(), delta)))


def simulate_controlled(
    initial_state: State,
    rocket: RocketParams,
    env: Environment,
    controller: IdealTorqueController,
    duration_s: float,
    dt_s: float,
) -> Iterator[tuple[float, State]]:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive.")
    if duration_s < 0.0:
        raise ValueError("duration_s cannot be negative.")

    time_s = 0.0
    state = initial_state.normalized()
    yield time_s, state
    for _ in range(int(round(duration_s / dt_s))):
        state = rk4_controlled_step(time_s, state, dt_s, rocket, env, controller)
        time_s += dt_s
        yield time_s, state
