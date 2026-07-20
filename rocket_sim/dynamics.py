"""Rigid-body dynamics for the 6-DOF rocket model."""

from __future__ import annotations
from .math3d import Vector,cross,dot,norm,q_derivative,rotate_body_to_inertial,rotate_inertial_to_body,unit,v_add,v_neg,v_scale,v_sub
from .models import Environment, RocketParams, State

def relative_velocity_body(state: State, env: Environment) -> Vector:
    return rotate_inertial_to_body(state.attitude, v_sub(state.velocity_mps, env.wind_mps))

def aerodynamic_drag_body(state: State, rocket: RocketParams, env: Environment) -> Vector:
    if rocket.drag_coefficient==0.0 or rocket.reference_area_m2==0.0: return (0.0,0.0,0.0)
    v=relative_velocity_body(state,env); s=norm(v)
    if s==0.0: return (0.0,0.0,0.0)
    # Drag opposes relative wind and scales with dynamic pressure: qbar = 0.5 rho V^2.
    return v_scale(-0.5*env.air_density_kgpm3*s*s*rocket.drag_coefficient*rocket.reference_area_m2, unit(v))

def aerodynamic_normal_force_body(state: State, rocket: RocketParams, env: Environment) -> Vector:
    if rocket.normal_force_coefficient_per_rad==0.0 or rocket.reference_area_m2==0.0: return (0.0,0.0,0.0)
    v=relative_velocity_body(state,env); s=norm(v)
    if s==0.0: return (0.0,0.0,0.0)
    axis=(0.0,0.0,1.0); lateral=v_sub(v, v_scale(dot(v,axis),axis)); ls=norm(lateral)
    if ls==0.0: return (0.0,0.0,0.0)
    # The lateral velocity ratio is a small-angle proxy for angle of attack.
    alpha=min(1.2, ls/max(s,1e-9)); qbar=0.5*env.air_density_kgpm3*s*s
    return v_scale(qbar*rocket.reference_area_m2*rocket.normal_force_coefficient_per_rad*alpha, unit(v_neg(lateral)))

def body_forces_and_torques(state: State, rocket: RocketParams, env: Environment) -> tuple[Vector,Vector]:
    thrust=v_scale(rocket.thrust_n, unit(rocket.thrust_direction_body))
    drag=aerodynamic_drag_body(state,rocket,env); normal=aerodynamic_normal_force_body(state,rocket,env)
    total=v_add(v_add(thrust,drag),normal)
    # Any force applied away from CM creates a rigid-body moment: tau = r x F.
    thrust_tau=cross(rocket.thrust_offset_body_m, thrust)
    aero_tau=cross(v_sub(rocket.center_of_pressure_body_m, rocket.center_of_mass_body_m), normal)
    return total, v_add(thrust_tau,aero_tau)

def derivatives_with_external_torque(_time_s: float, state: State, rocket: RocketParams, env: Environment, external_torque_body: Vector) -> tuple[float,...]:
    force_body,tau=body_forces_and_torques(state,rocket,env)
    tau=v_add(tau, external_torque_body)
    force_i=rotate_body_to_inertial(state.attitude, force_body)
    accel=v_scale(1.0/rocket.mass_kg, v_add(force_i,(0.0,0.0,-rocket.mass_kg*env.gravity_mps2)))
    qdot=q_derivative(state.attitude,state.angular_velocity_radps)
    ix,iy,iz=rocket.inertia_kg_m2; w=state.angular_velocity_radps
    # Euler's rigid-body equation in body axes includes gyroscopic coupling.
    h=(ix*w[0],iy*w[1],iz*w[2]); rhs=v_sub(tau,cross(w,h))
    wdot=(rhs[0]/ix,rhs[1]/iy,rhs[2]/iz)
    return (*state.velocity_mps,*accel,*qdot,*wdot)

def derivatives(_time_s: float, state: State, rocket: RocketParams, env: Environment) -> tuple[float,...]:
    return derivatives_with_external_torque(_time_s,state,rocket,env,(0.0,0.0,0.0))
