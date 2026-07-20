"""Attitude controllers for the rocket simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .analysis import body_z_axis_inertial
from .math3d import Vector, cross, norm, rotate_inertial_to_body, unit, v_add, v_clamp_norm, v_scale, v_sub
from .models import RocketParams, State


@dataclass(frozen=True)
class IdealTorqueController:
    """PD attitude controller that commands ideal body torque.

    Week 3A uses ideal torque to validate the control law before modeling a
    thrust-vector-control actuator. The controller aligns the rocket body +z
    axis with a commanded inertial direction and damps body angular velocity.
    """

    kp_nmpu: float
    kd_nms: float
    max_torque_nm: float
    commanded_axis_inertial: Vector = (0.0, 0.0, 1.0)

    def torque_body(self, state: State) -> Vector:
        body_axis_inertial = body_z_axis_inertial(state)
        error_inertial = cross(body_axis_inertial, self.commanded_axis_inertial)
        error_body = rotate_inertial_to_body(state.attitude, error_inertial)
        proportional = v_scale(self.kp_nmpu, error_body)
        damping = v_scale(self.kd_nms, state.angular_velocity_radps)
        raw_torque = v_sub(proportional, damping)
        return v_clamp_norm(raw_torque, self.max_torque_nm)


@dataclass(frozen=True)
class LQRAttitudeController:
    """Small-angle infinite-horizon LQR attitude controller.

    The linearized pitch/yaw channels are modeled as double integrators:

        theta_dot = omega
        omega_dot = tau / I

    The controller uses the analytic continuous-time LQR solution for each
    decoupled transverse axis, then applies the resulting gains to the same
    body-axis attitude error used by the nonlinear PD controller.
    """

    q_angle: float
    q_rate: float
    r_control: float
    inertia_kg_m2: Vector
    max_torque_nm: float
    commanded_axis_inertial: Vector = (0.0, 0.0, 1.0)

    def gains_for_axis(self, inertia_axis_kg_m2: float) -> tuple[float, float]:
        if self.q_angle <= 0.0:
            raise ValueError("q_angle must be positive.")
        if self.q_rate < 0.0:
            raise ValueError("q_rate cannot be negative.")
        if self.r_control <= 0.0:
            raise ValueError("r_control must be positive.")
        if inertia_axis_kg_m2 <= 0.0:
            raise ValueError("inertia_axis_kg_m2 must be positive.")

        k_angle = math.sqrt(self.q_angle / self.r_control)
        k_rate = math.sqrt((2.0 * inertia_axis_kg_m2 * math.sqrt(self.q_angle * self.r_control) + self.q_rate) / self.r_control)
        return k_angle, k_rate

    def torque_body(self, state: State) -> Vector:
        body_axis_inertial = body_z_axis_inertial(state)
        error_inertial = cross(body_axis_inertial, self.commanded_axis_inertial)
        error_body = rotate_inertial_to_body(state.attitude, error_inertial)

        kx_angle, kx_rate = self.gains_for_axis(self.inertia_kg_m2[0])
        ky_angle, ky_rate = self.gains_for_axis(self.inertia_kg_m2[1])

        raw_torque = (
            kx_angle * error_body[0] - kx_rate * state.angular_velocity_radps[0],
            ky_angle * error_body[1] - ky_rate * state.angular_velocity_radps[1],
            0.0,
        )
        return v_clamp_norm(raw_torque, self.max_torque_nm)


@dataclass(frozen=True)
class TVCCommand:
    thrust_direction_body: Vector
    gimbal_x_rad: float
    gimbal_y_rad: float
    requested_torque_body: Vector
    achievable_torque_body: Vector
    saturated: bool


@dataclass(frozen=True)
class TVCController:
    """PD attitude controller allocated through a gimbaled thrust vector.

    The controller requests body torque, then converts the requested roll-free
    pitch/yaw torque into lateral thrust components. This makes Week 3B more
    realistic than Week 3A: control torque is limited by thrust, lever arm, and
    maximum gimbal angle.
    """

    ideal_controller: IdealTorqueController
    engine_position_body_m: Vector = (0.0, 0.0, -1.2)
    max_gimbal_rad: float = math.radians(5.0)
    thrust_misalignment_body: Vector = (0.0, 0.0, 1.0)

    def command(self, state: State, rocket: RocketParams) -> TVCCommand:
        requested = self.ideal_controller.torque_body(state)
        lever_arm = self.engine_position_body_m[2]
        thrust = rocket.thrust_n
        if thrust <= 0.0 or lever_arm == 0.0:
            return TVCCommand((0.0, 0.0, 1.0), 0.0, 0.0, requested, (0.0, 0.0, 0.0), True)

        # For r = [0, 0, -L], tau = r x F = [L Fy, -L Fx, 0].
        lateral_fx = -requested[1] / abs(lever_arm)
        lateral_fy = requested[0] / abs(lever_arm)
        max_lateral = thrust * math.sin(self.max_gimbal_rad)
        lateral_norm = math.sqrt(lateral_fx**2 + lateral_fy**2)
        saturated = lateral_norm > max_lateral
        if saturated and lateral_norm > 0.0:
            scale = max_lateral / lateral_norm
            lateral_fx *= scale
            lateral_fy *= scale

        axial_force = math.sqrt(max(0.0, thrust**2 - lateral_fx**2 - lateral_fy**2))
        commanded_force = (lateral_fx, lateral_fy, axial_force)
        direction = unit(v_add(commanded_force, v_scale(thrust, self.thrust_misalignment_body)))
        achievable_force = v_scale(thrust, direction)
        achievable = cross(self.engine_position_body_m, achievable_force)
        return TVCCommand(
            thrust_direction_body=direction,
            gimbal_x_rad=math.atan2(direction[0], direction[2]),
            gimbal_y_rad=math.atan2(direction[1], direction[2]),
            requested_torque_body=requested,
            achievable_torque_body=achievable,
            saturated=saturated,
        )
