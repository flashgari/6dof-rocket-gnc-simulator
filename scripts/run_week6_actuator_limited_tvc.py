#!/usr/bin/env python3
"""Run Week 6 finite-bandwidth TVC actuator simulations."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import AttitudeEstimator, AttitudeEstimatorConfig, SensorModel
from rocket_sim.actuator_sim import ActuatorLimitedSample, EstimatedActuatorLimitedSample
from rocket_sim.actuator_sim import simulate_actuator_limited_tvc, simulate_estimated_actuator_limited_tvc
from rocket_sim.actuators import GimbalActuator, GimbalActuatorConfig, max_tvc_torque_nm
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg, summary_metrics, tilt_angle_deg
from rocket_sim.math3d import norm, q_norm
from scripts.run_week4a_lqr_tvc_ascent import week4a_setup


def week6_actuator_config() -> GimbalActuatorConfig:
    return GimbalActuatorConfig(
        max_gimbal_rad=math.radians(5.0),
        max_rate_radps=math.radians(45.0),
        time_constant_s=0.075,
    )


def week6_sensor_model() -> SensorModel:
    return SensorModel(
        gyro_bias_radps=(0.010, -0.006, 0.002),
        gyro_noise_std_radps=0.0015,
        accel_bias_mps2=(0.08, -0.05, 0.12),
        accel_noise_std_mps2=0.08,
        attitude_reference_noise_rad=math.radians(0.35),
        attitude_reference_period_s=0.05,
        seed=20260722,
    )


def week6_estimator(initial_attitude):
    return AttitudeEstimator(
        initial_attitude=initial_attitude,
        config=AttitudeEstimatorConfig(
            gyro_bias_estimate_radps=(0.010, -0.006, 0.002),
            attitude_reference_gain=0.18,
        ),
    )


def write_truth_csv(path: Path, samples: list[ActuatorLimitedSample], max_torque_nm: float) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "vx_mps",
                "vy_mps",
                "vz_mps",
                "qw",
                "qx",
                "qy",
                "qz",
                "wx_radps",
                "wy_radps",
                "wz_radps",
                "q_norm",
                "tilt_deg",
                "signed_pitch_deg",
                "signed_yaw_deg",
                "body_z_x",
                "body_z_y",
                "body_z_z",
                "lateral_m",
                "cmd_gimbal_x_deg",
                "cmd_gimbal_y_deg",
                "cmd_gimbal_total_deg",
                "ach_gimbal_x_deg",
                "ach_gimbal_y_deg",
                "ach_gimbal_total_deg",
                "gimbal_lag_error_deg",
                "requested_torque_norm_nm",
                "achievable_torque_norm_nm",
                "available_tvc_torque_nm",
                "torque_authority_margin_nm",
                "rate_limited",
                "position_limited",
                "saturated",
            ]
        )
        for sample in samples:
            state = sample.true_state
            body_z = body_z_axis_inertial(state)
            requested = sample.requested_command
            achieved = sample.achieved_command
            cmd_total = math.degrees(math.sqrt(requested.gimbal_x_rad**2 + requested.gimbal_y_rad**2))
            ach_total = math.degrees(math.sqrt(achieved.gimbal_x_rad**2 + achieved.gimbal_y_rad**2))
            requested_torque = norm(requested.requested_torque_body)
            achievable_torque = norm(achieved.achievable_torque_body)
            writer.writerow(
                [
                    sample.time_s,
                    *state.as_tuple(),
                    q_norm(state.attitude),
                    tilt_angle_deg(state),
                    signed_pitch_deg(state),
                    signed_yaw_deg(state),
                    *body_z,
                    lateral_displacement_m(state),
                    math.degrees(requested.gimbal_x_rad),
                    math.degrees(requested.gimbal_y_rad),
                    cmd_total,
                    math.degrees(achieved.gimbal_x_rad),
                    math.degrees(achieved.gimbal_y_rad),
                    ach_total,
                    math.degrees(sample.actuator_output.command_error_rad),
                    requested_torque,
                    achievable_torque,
                    max_torque_nm,
                    max_torque_nm - requested_torque,
                    int(sample.actuator_output.rate_limited),
                    int(sample.actuator_output.position_limited),
                    int(achieved.saturated),
                ]
            )


def write_estimated_csv(path: Path, samples: list[EstimatedActuatorLimitedSample], max_torque_nm: float) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "vx_mps",
                "vy_mps",
                "vz_mps",
                "qw",
                "qx",
                "qy",
                "qz",
                "wx_radps",
                "wy_radps",
                "wz_radps",
                "est_qw",
                "est_qx",
                "est_qy",
                "est_qz",
                "est_wx_radps",
                "est_wy_radps",
                "est_wz_radps",
                "q_norm",
                "est_q_norm",
                "tilt_deg",
                "est_tilt_deg",
                "attitude_error_deg",
                "rate_error_radps",
                "body_z_x",
                "body_z_y",
                "body_z_z",
                "lateral_m",
                "cmd_gimbal_total_deg",
                "ach_gimbal_total_deg",
                "gimbal_lag_error_deg",
                "requested_torque_norm_nm",
                "achievable_torque_norm_nm",
                "available_tvc_torque_nm",
                "torque_authority_margin_nm",
                "rate_limited",
                "position_limited",
                "saturated",
            ]
        )
        for sample in samples:
            state = sample.true_state
            est = sample.estimated_state
            body_z = body_z_axis_inertial(state)
            requested = sample.requested_command
            achieved = sample.achieved_command
            requested_torque = norm(requested.requested_torque_body)
            achievable_torque = norm(achieved.achievable_torque_body)
            writer.writerow(
                [
                    sample.time_s,
                    *state.as_tuple(),
                    *est.attitude,
                    *est.angular_velocity_radps,
                    q_norm(state.attitude),
                    q_norm(est.attitude),
                    tilt_angle_deg(state),
                    tilt_angle_deg(est),
                    sample.attitude_error_deg,
                    sample.rate_error_radps,
                    *body_z,
                    lateral_displacement_m(state),
                    math.degrees(math.sqrt(requested.gimbal_x_rad**2 + requested.gimbal_y_rad**2)),
                    math.degrees(math.sqrt(achieved.gimbal_x_rad**2 + achieved.gimbal_y_rad**2)),
                    math.degrees(sample.actuator_output.command_error_rad),
                    requested_torque,
                    achievable_torque,
                    max_torque_nm,
                    max_torque_nm - requested_torque,
                    int(sample.actuator_output.rate_limited),
                    int(sample.actuator_output.position_limited),
                    int(achieved.saturated),
                ]
            )


def main() -> None:
    rocket, env, initial, tvc = week4a_setup()
    config = week6_actuator_config()
    max_torque = max_tvc_torque_nm(rocket, tvc, config.max_gimbal_rad)
    truth_samples = list(
        simulate_actuator_limited_tvc(
            initial,
            rocket,
            env,
            tvc,
            GimbalActuator(config),
            duration_s=3.0,
            dt_s=0.005,
        )
    )
    estimated_samples = list(
        simulate_estimated_actuator_limited_tvc(
            initial,
            rocket,
            env,
            tvc,
            GimbalActuator(config),
            week6_sensor_model().sampler(),
            week6_estimator(initial.attitude),
            duration_s=3.0,
            dt_s=0.005,
        )
    )
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    truth_path = output_dir / "week6_lqr_tvc_actuator_limited.csv"
    estimated_path = output_dir / "week6_estimated_lqr_tvc_actuator_limited.csv"
    write_truth_csv(truth_path, truth_samples, max_torque)
    write_estimated_csv(estimated_path, estimated_samples, max_torque)

    truth_metrics = summary_metrics([(sample.time_s, sample.true_state) for sample in truth_samples], rocket, env)
    estimated_metrics = summary_metrics([(sample.time_s, sample.true_state) for sample in estimated_samples], rocket, env)
    truth_rate_fraction = sum(sample.actuator_output.rate_limited for sample in truth_samples) / len(truth_samples)
    estimated_rate_fraction = sum(sample.actuator_output.rate_limited for sample in estimated_samples) / len(estimated_samples)
    truth_lag = max(math.degrees(sample.actuator_output.command_error_rad) for sample in truth_samples)
    estimated_lag = max(math.degrees(sample.actuator_output.command_error_rad) for sample in estimated_samples)

    print(f"Wrote {len(truth_samples)} samples to {truth_path}")
    print(f"Wrote {len(estimated_samples)} samples to {estimated_path}")
    print(f"Truth-state final altitude: {truth_metrics['final_altitude_m']:.2f} m")
    print(f"Truth-state max tilt angle: {truth_metrics['max_tilt_deg']:.2f} deg")
    print(f"Truth-state max lateral displacement: {truth_metrics['max_lateral_displacement_m']:.2f} m")
    print(f"Truth-state max gimbal lag: {truth_lag:.2f} deg")
    print(f"Truth-state rate-limit fraction: {100.0 * truth_rate_fraction:.1f}%")
    print(f"Estimated-state final altitude: {estimated_metrics['final_altitude_m']:.2f} m")
    print(f"Estimated-state max tilt angle: {estimated_metrics['max_tilt_deg']:.2f} deg")
    print(f"Estimated-state max lateral displacement: {estimated_metrics['max_lateral_displacement_m']:.2f} m")
    print(f"Estimated-state max gimbal lag: {estimated_lag:.2f} deg")
    print(f"Estimated-state rate-limit fraction: {100.0 * estimated_rate_fraction:.1f}%")


if __name__ == "__main__":
    main()
