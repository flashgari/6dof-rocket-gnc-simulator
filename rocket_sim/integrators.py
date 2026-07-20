"""Numerical integrators."""

from __future__ import annotations
from collections.abc import Callable
from .math3d import weighted_sum
from .models import Environment,RocketParams,State
DerivativeFunction=Callable[[float,State,RocketParams,Environment],tuple[float,...]]
def _state_plus_scaled_derivative(state: State, scale: float, derivative: tuple[float,...]) -> State:
    return State.from_tuple(tuple(s+scale*ds for s,ds in zip(state.as_tuple(),derivative)))
def rk4_step(f: DerivativeFunction, time_s: float, state: State, dt_s: float, rocket: RocketParams, env: Environment) -> State:
    k1=f(time_s,state,rocket,env); k2=f(time_s+0.5*dt_s,_state_plus_scaled_derivative(state,0.5*dt_s,k1),rocket,env)
    k3=f(time_s+0.5*dt_s,_state_plus_scaled_derivative(state,0.5*dt_s,k2),rocket,env); k4=f(time_s+dt_s,_state_plus_scaled_derivative(state,dt_s,k3),rocket,env)
    delta=weighted_sum((dt_s/6.0,dt_s/3.0,dt_s/3.0,dt_s/6.0),(k1,k2,k3,k4))
    return State.from_tuple(tuple(s+ds for s,ds in zip(state.as_tuple(),delta)))
