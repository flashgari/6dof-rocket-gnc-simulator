"""Closed-loop simulation with time-varying mass properties."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

from .actuators import GimbalActuator, GimbalActuatorOutput
from .control import TVCCommand, TVCController
from .dynamics import derivatives
from .math3d import weighted_sum
from .models import Environment, State
from .propulsion import TimeVaryingRocket
from .tvc_sim import rocket_with_tvc_command


@dataclass(frozen=True)
class VariableMassActuatorSample:
    time_s: float
    state: State
    feedback_state: State
    requested_command: TVCCommand
    achieved_command: TVCCommand
    actuator_output: GimbalActuatorOutput
    mass_kg: float
    thrust_n: float
    inertia_kg_m2: tuple[float, float, float]
    center_of_mass_body_m: tuple[float, float, float]
    propellant_fraction: float


def _state_plus_scaled_derivative(state: State, scale: float, derivative: tuple[float, ...]) -> State:
    return State.from_tuple(tuple(s + scale * ds for s, ds in zip(state.as_tuple(), derivative)))


def _derivative_with_command(
    time_s: float,
    state: State,
    vehicle: TimeVaryingRocket,
    env: Environment,
    command: TVCCommand,
    controller: TVCController,
) -> tuple[float, ...]:
    rocket = rocket_with_tvc_command(vehicle.at(time_s), command, controller)
    return derivatives(time_s, state, rocket, env)


def rk4_variable_mass_actuator_step(
    time_s: float,
    state: State,
    dt_s: float,
    vehicle: TimeVaryingRocket,
    env: Environment,
    command: TVCCommand,
    controller: TVCController,
) -> State:
    k1 = _derivative_with_command(time_s, state, vehicle, env, command, controller)
    k2 = _derivative_with_command(
        time_s + 0.5 * dt_s,
        _state_plus_scaled_derivative(state, 0.5 * dt_s, k1),
        vehicle,
        env,
        command,
        controller,
    )
    k3 = _derivative_with_command(
        time_s + 0.5 * dt_s,
        _state_plus_scaled_derivative(state, 0.5 * dt_s, k2),
        vehicle,
        env,
        command,
        controller,
    )
    k4 = _derivative_with_command(
        time_s + dt_s,
        _state_plus_scaled_derivative(state, dt_s, k3),
        vehicle,
        env,
        command,
        controller,
    )
    delta = weighted_sum((dt_s / 6.0, dt_s / 3.0, dt_s / 3.0, dt_s / 6.0), (k1, k2, k3, k4))
    return State.from_tuple(tuple(s + ds for s, ds in zip(state.as_tuple(), delta)))


def _sample(
    time_s: float,
    state: State,
    feedback_state: State,
    requested: TVCCommand,
    achieved: TVCCommand,
    actuator_output: GimbalActuatorOutput,
    vehicle: TimeVaryingRocket,
) -> VariableMassActuatorSample:
    rocket = vehicle.at(time_s)
    mass_props = vehicle.mass_properties
    return VariableMassActuatorSample(
        time_s=time_s,
        state=state,
        feedback_state=feedback_state,
        requested_command=requested,
        achieved_command=achieved,
        actuator_output=actuator_output,
        mass_kg=rocket.mass_kg,
        thrust_n=rocket.thrust_n,
        inertia_kg_m2=rocket.inertia_kg_m2,
        center_of_mass_body_m=rocket.center_of_mass_body_m,
        propellant_fraction=mass_props.remaining_propellant_fraction(time_s),
    )


def simulate_variable_mass_actuator_limited_tvc(
    initial_state: State,
    vehicle: TimeVaryingRocket,
    env: Environment,
    controller: TVCController,
    actuator: GimbalActuator,
    duration_s: float,
    dt_s: float,
) -> Iterator[VariableMassActuatorSample]:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive.")
    if duration_s < 0.0:
        raise ValueError("duration_s cannot be negative.")

    time_s = 0.0
    state = initial_state.normalized()
    rocket = vehicle.at(time_s)
    requested = controller.command(state, rocket)
    actuator_output = actuator.update(requested, rocket, controller, 0.0)
    achieved = actuator_output.achieved_command
    yield _sample(time_s, state, state, requested, achieved, actuator_output, vehicle)

    for _ in range(int(round(duration_s / dt_s))):
        state = rk4_variable_mass_actuator_step(time_s, state, dt_s, vehicle, env, achieved, controller)
        time_s += dt_s
        rocket = vehicle.at(time_s)
        requested = controller.command(state, rocket)
        actuator_output = actuator.update(requested, rocket, controller, dt_s)
        achieved = actuator_output.achieved_command
        yield _sample(time_s, state, state, requested, achieved, actuator_output, vehicle)
