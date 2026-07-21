"""Closed-loop TVC simulation using estimated attitude state."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

from .control import TVCCommand, TVCController
from .dynamics import derivatives
from .math3d import weighted_sum
from .models import Environment, RocketParams, State
from .sensors import AttitudeEstimate, AttitudeEstimator, SensorMeasurement, SensorSampler, attitude_error_deg
from .tvc_sim import rocket_with_tvc_command


@dataclass(frozen=True)
class EstimatedClosedLoopSample:
    time_s: float
    true_state: State
    estimated_state: State
    measurement: SensorMeasurement
    estimate: AttitudeEstimate
    command: TVCCommand
    attitude_error_deg: float
    rate_error_radps: float


def _state_plus_scaled_derivative(state: State, scale: float, derivative: tuple[float, ...]) -> State:
    return State.from_tuple(tuple(s + scale * ds for s, ds in zip(state.as_tuple(), derivative)))


def _constant_command_step(
    time_s: float,
    state: State,
    dt_s: float,
    rocket: RocketParams,
    env: Environment,
    command: TVCCommand,
    controller: TVCController,
) -> State:
    actuated_rocket = rocket_with_tvc_command(rocket, command, controller)
    k1 = derivatives(time_s, state, actuated_rocket, env)
    k2 = derivatives(time_s + 0.5 * dt_s, _state_plus_scaled_derivative(state, 0.5 * dt_s, k1), actuated_rocket, env)
    k3 = derivatives(time_s + 0.5 * dt_s, _state_plus_scaled_derivative(state, 0.5 * dt_s, k2), actuated_rocket, env)
    k4 = derivatives(time_s + dt_s, _state_plus_scaled_derivative(state, dt_s, k3), actuated_rocket, env)
    delta = weighted_sum((dt_s / 6.0, dt_s / 3.0, dt_s / 3.0, dt_s / 6.0), (k1, k2, k3, k4))
    return State.from_tuple(tuple(s + ds for s, ds in zip(state.as_tuple(), delta)))


def simulate_estimated_tvc(
    initial_state: State,
    rocket: RocketParams,
    env: Environment,
    controller: TVCController,
    sensor_sampler: SensorSampler,
    estimator: AttitudeEstimator,
    duration_s: float,
    dt_s: float,
) -> Iterator[EstimatedClosedLoopSample]:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive.")
    if duration_s < 0.0:
        raise ValueError("duration_s cannot be negative.")

    time_s = 0.0
    true_state = initial_state.normalized()
    measurement = sensor_sampler.measure(time_s, true_state, rocket, env)
    estimate = estimator.update(measurement, 0.0)
    estimated_state = State(true_state.position_m, true_state.velocity_mps, estimate.attitude, estimate.angular_velocity_radps)
    command = controller.command(estimated_state, rocket)
    yield EstimatedClosedLoopSample(
        time_s,
        true_state,
        estimated_state,
        measurement,
        estimate,
        command,
        attitude_error_deg(true_state.attitude, estimate.attitude),
        0.0,
    )

    for _ in range(int(round(duration_s / dt_s))):
        true_state = _constant_command_step(time_s, true_state, dt_s, rocket, env, command, controller)
        time_s += dt_s
        measurement = sensor_sampler.measure(time_s, true_state, rocket, env)
        estimate = estimator.update(measurement, dt_s)
        estimated_state = State(true_state.position_m, true_state.velocity_mps, estimate.attitude, estimate.angular_velocity_radps)
        command = controller.command(estimated_state, rocket)
        rate_error = sum((a - b) ** 2 for a, b in zip(true_state.angular_velocity_radps, estimate.angular_velocity_radps)) ** 0.5
        yield EstimatedClosedLoopSample(
            time_s,
            true_state,
            estimated_state,
            measurement,
            estimate,
            command,
            attitude_error_deg(true_state.attitude, estimate.attitude),
            rate_error,
        )
