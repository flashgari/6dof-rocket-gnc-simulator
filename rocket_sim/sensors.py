"""Sensor and estimator models for avionics-style closed-loop simulation."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .dynamics import derivatives
from .math3d import (
    Quaternion,
    Vector,
    cross,
    norm,
    q_conj,
    q_derivative,
    q_mul,
    q_normalize,
    rotate_body_to_inertial,
    rotate_inertial_to_body,
    v_add,
    v_scale,
    v_sub,
)
from .models import Environment, RocketParams, State


@dataclass(frozen=True)
class SensorMeasurement:
    """One sampled avionics measurement packet.

    Accelerometer output is modeled as specific force in the body frame:

        f_B = R_IB (a_I - g_I)

    During powered ascent this is dominated by thrust and aerodynamic force, so
    it is logged for avionics realism but not treated as a clean gravity-vector
    attitude reference.
    """

    time_s: float
    gyro_radps: Vector
    accelerometer_mps2: Vector
    reference_attitude: Quaternion | None


@dataclass(frozen=True)
class SensorModel:
    gyro_bias_radps: Vector = (0.010, -0.006, 0.002)
    gyro_noise_std_radps: float = 0.0015
    accel_bias_mps2: Vector = (0.08, -0.05, 0.12)
    accel_noise_std_mps2: float = 0.08
    attitude_reference_noise_rad: float = math.radians(0.35)
    attitude_reference_period_s: float = 0.05
    seed: int = 20260721

    def sampler(self) -> "SensorSampler":
        return SensorSampler(self)


class SensorSampler:
    def __init__(self, model: SensorModel):
        self.model = model
        self._rng = random.Random(model.seed)
        self._next_reference_time_s = 0.0

    def _noise_vec(self, std: float) -> Vector:
        return (
            self._rng.gauss(0.0, std),
            self._rng.gauss(0.0, std),
            self._rng.gauss(0.0, std),
        )

    def _noisy_attitude_reference(self, true_attitude: Quaternion) -> Quaternion:
        noise = self._noise_vec(self.model.attitude_reference_noise_rad)
        small_angle = (1.0, 0.5 * noise[0], 0.5 * noise[1], 0.5 * noise[2])
        return q_normalize(q_mul(true_attitude, q_normalize(small_angle)))

    def measure(self, time_s: float, state: State, rocket: RocketParams, env: Environment) -> SensorMeasurement:
        deriv = derivatives(time_s, state, rocket, env)
        accel_inertial = (deriv[3], deriv[4], deriv[5])
        gravity_inertial = (0.0, 0.0, -env.gravity_mps2)
        specific_force_body = rotate_inertial_to_body(state.attitude, v_sub(accel_inertial, gravity_inertial))

        gyro_noise = self._noise_vec(self.model.gyro_noise_std_radps)
        accel_noise = self._noise_vec(self.model.accel_noise_std_mps2)
        gyro = v_add(v_add(state.angular_velocity_radps, self.model.gyro_bias_radps), gyro_noise)
        accelerometer = v_add(v_add(specific_force_body, self.model.accel_bias_mps2), accel_noise)

        reference = None
        if time_s + 1.0e-12 >= self._next_reference_time_s:
            reference = self._noisy_attitude_reference(state.attitude)
            self._next_reference_time_s += self.model.attitude_reference_period_s

        return SensorMeasurement(time_s, gyro, accelerometer, reference)


@dataclass(frozen=True)
class AttitudeEstimate:
    attitude: Quaternion
    angular_velocity_radps: Vector
    gyro_bias_radps: Vector
    correction_norm_rad: float


@dataclass(frozen=True)
class AttitudeEstimatorConfig:
    gyro_bias_estimate_radps: Vector = (0.010, -0.006, 0.002)
    attitude_reference_gain: float = 0.18


class AttitudeEstimator:
    """Quaternion gyro propagation with low-rate attitude-reference correction."""

    def __init__(self, initial_attitude: Quaternion, config: AttitudeEstimatorConfig | None = None):
        self.config = config or AttitudeEstimatorConfig()
        self.attitude = q_normalize(initial_attitude)
        self.angular_velocity_radps = (0.0, 0.0, 0.0)
        self.last_correction_norm_rad = 0.0

    def update(self, measurement: SensorMeasurement, dt_s: float) -> AttitudeEstimate:
        self.angular_velocity_radps = v_sub(measurement.gyro_radps, self.config.gyro_bias_estimate_radps)

        q_dot = q_derivative(self.attitude, self.angular_velocity_radps)
        predicted = q_normalize(tuple(q + dt_s * dq for q, dq in zip(self.attitude, q_dot)))  # type: ignore[arg-type]

        correction_norm = 0.0
        if measurement.reference_attitude is not None:
            predicted_axis = rotate_body_to_inertial(predicted, (0.0, 0.0, 1.0))
            reference_axis = rotate_body_to_inertial(measurement.reference_attitude, (0.0, 0.0, 1.0))
            error_inertial = cross(predicted_axis, reference_axis)
            error_body = rotate_inertial_to_body(predicted, error_inertial)
            correction_body = v_scale(self.config.attitude_reference_gain, error_body)
            correction_norm = norm(correction_body)
            delta_q = q_normalize((1.0, 0.5 * correction_body[0], 0.5 * correction_body[1], 0.5 * correction_body[2]))
            predicted = q_normalize(q_mul(predicted, delta_q))

        self.attitude = predicted
        self.last_correction_norm_rad = correction_norm
        return AttitudeEstimate(
            attitude=self.attitude,
            angular_velocity_radps=self.angular_velocity_radps,
            gyro_bias_radps=self.config.gyro_bias_estimate_radps,
            correction_norm_rad=correction_norm,
        )


def attitude_error_deg(true_attitude: Quaternion, estimated_attitude: Quaternion) -> float:
    q_err = q_mul(q_conj(true_attitude), estimated_attitude)
    q_err = q_normalize(q_err if q_err[0] >= 0.0 else (-q_err[0], -q_err[1], -q_err[2], -q_err[3]))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, q_err[0]))))

