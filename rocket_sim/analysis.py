"""Analysis helpers for simulation outputs."""

from __future__ import annotations
import math
from .math3d import Vector,dot,norm,q_norm,rotate_body_to_inertial
from .models import Environment,RocketParams,State
def linear_momentum_kg_mps(state: State, rocket: RocketParams) -> tuple[float,float,float]: return tuple(rocket.mass_kg*v for v in state.velocity_mps)  # type: ignore[return-value]
def angular_momentum_body_kg_m2_radps(state: State, rocket: RocketParams) -> tuple[float,float,float]:
    ix,iy,iz=rocket.inertia_kg_m2; wx,wy,wz=state.angular_velocity_radps; return (ix*wx,iy*wy,iz*wz)
def angular_momentum_inertial_kg_m2_radps(state: State, rocket: RocketParams) -> tuple[float,float,float]: return rotate_body_to_inertial(state.attitude,angular_momentum_body_kg_m2_radps(state,rocket))
def speed_mps(state: State) -> float: return norm(state.velocity_mps)
def body_z_axis_inertial(state: State) -> Vector: return rotate_body_to_inertial(state.attitude,(0.0,0.0,1.0))
def tilt_angle_deg(state: State) -> float:
    axis=body_z_axis_inertial(state); c=max(-1.0,min(1.0,dot(axis,(0.0,0.0,1.0))/norm(axis))); return math.degrees(math.acos(c))
def signed_pitch_deg(state: State) -> float:
    axis=body_z_axis_inertial(state); return math.degrees(math.atan2(axis[0], axis[2]))
def signed_yaw_deg(state: State) -> float:
    axis=body_z_axis_inertial(state); return math.degrees(math.atan2(axis[1], axis[2]))
def lateral_displacement_m(state: State) -> float: return math.sqrt(state.position_m[0]**2+state.position_m[1]**2)
def translational_kinetic_energy_j(state: State, rocket: RocketParams) -> float: return 0.5*rocket.mass_kg*speed_mps(state)**2
def rotational_kinetic_energy_j(state: State, rocket: RocketParams) -> float:
    ix,iy,iz=rocket.inertia_kg_m2; wx,wy,wz=state.angular_velocity_radps; return 0.5*(ix*wx**2+iy*wy**2+iz*wz**2)
def potential_energy_j(state: State, rocket: RocketParams, env: Environment) -> float: return rocket.mass_kg*env.gravity_mps2*state.position_m[2]
def mechanical_energy_j(state: State, rocket: RocketParams, env: Environment) -> float: return translational_kinetic_energy_j(state,rocket)+rotational_kinetic_energy_j(state,rocket)+potential_energy_j(state,rocket,env)
def summary_metrics(samples: list[tuple[float,State]], rocket: RocketParams, env: Environment) -> dict[str,float]:
    if not samples: raise ValueError('Cannot summarize an empty trajectory.')
    tf,final=samples[-1]; tilts=[tilt_angle_deg(s) for _,s in samples]; speeds=[speed_mps(s) for _,s in samples]; lateral=[lateral_displacement_m(s) for _,s in samples]; rates=[norm(s.angular_velocity_radps) for _,s in samples]; vertical=[body_z_axis_inertial(s)[2] for _,s in samples]
    return {'duration_s':tf,'samples':float(len(samples)),'final_altitude_m':final.position_m[2],'max_altitude_m':max(s.position_m[2] for _,s in samples),'final_speed_mps':speed_mps(final),'max_speed_mps':max(speeds),'final_vertical_velocity_mps':final.velocity_mps[2],'final_lateral_displacement_m':lateral_displacement_m(final),'max_lateral_displacement_m':max(lateral),'final_tilt_deg':tilt_angle_deg(final),'max_tilt_deg':max(tilts),'min_body_z_vertical_component':min(vertical),'max_angular_rate_radps':max(rates),'max_quaternion_norm_error':max(abs(q_norm(s.attitude)-1.0) for _,s in samples),'final_mechanical_energy_j':mechanical_energy_j(final,rocket,env)}
