"""Simulation utilities."""

from __future__ import annotations
from collections.abc import Iterator
from .dynamics import derivatives
from .integrators import rk4_step
from .models import Environment,RocketParams,State
def simulate(initial_state: State, rocket: RocketParams, env: Environment, duration_s: float, dt_s: float) -> Iterator[tuple[float,State]]:
    if dt_s<=0.0: raise ValueError('dt_s must be positive.')
    if duration_s<0.0: raise ValueError('duration_s cannot be negative.')
    t=0.0; state=initial_state.normalized(); yield t,state
    for _ in range(int(round(duration_s/dt_s))):
        state=rk4_step(derivatives,t,state,dt_s,rocket,env); t+=dt_s; yield t,state
