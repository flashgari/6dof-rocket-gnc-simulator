"""Data models for the rocket simulator."""

from __future__ import annotations
from dataclasses import dataclass
from .math3d import Quaternion, Vector, q_normalize

@dataclass(frozen=True)
class State:
    position_m: Vector
    velocity_mps: Vector
    attitude: Quaternion
    angular_velocity_radps: Vector
    def as_tuple(self) -> tuple[float,...]: return (*self.position_m,*self.velocity_mps,*self.attitude,*self.angular_velocity_radps)
    @classmethod
    def from_tuple(cls, values: tuple[float,...]) -> 'State':
        if len(values)!=13: raise ValueError(f'Expected 13 state values, got {len(values)}.')
        return cls((values[0],values[1],values[2]),(values[3],values[4],values[5]),q_normalize((values[6],values[7],values[8],values[9])),(values[10],values[11],values[12]))
    def normalized(self) -> 'State': return State(self.position_m,self.velocity_mps,q_normalize(self.attitude),self.angular_velocity_radps)

@dataclass(frozen=True)
class RocketParams:
    mass_kg: float = 50.0
    inertia_kg_m2: Vector = (3.0,3.0,0.45)
    thrust_n: float = 850.0
    reference_area_m2: float = 0.03
    drag_coefficient: float = 0.0
    normal_force_coefficient_per_rad: float = 0.0
    center_of_mass_body_m: Vector = (0.0,0.0,0.0)
    center_of_pressure_body_m: Vector = (0.0,0.0,-0.5)
    thrust_offset_body_m: Vector = (0.0,0.0,0.0)
    thrust_direction_body: Vector = (0.0,0.0,1.0)

@dataclass(frozen=True)
class Environment:
    gravity_mps2: float = 9.80665
    air_density_kgpm3: float = 1.225
    wind_mps: Vector = (0.0,0.0,0.0)
