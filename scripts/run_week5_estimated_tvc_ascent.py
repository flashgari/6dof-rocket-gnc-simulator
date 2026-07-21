#!/usr/bin/env python3
"""Run Week 5 ascent with noisy sensors and estimated-state TVC feedback."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import AttitudeEstimator, AttitudeEstimatorConfig, SensorModel
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg, summary_metrics, tilt_angle_deg
from rocket_sim.estimated_sim import EstimatedClosedLoopSample, simulate_estimated_tvc
from rocket_sim.math3d import norm, q_norm
from scripts.run_week4a_lqr_tvc_ascent import week4a_setup


def week5_setup():
    rocket, env, initial, tvc = week4a_setup()
    sensors = SensorModel(
        gyro_bias_radps=(0.010, -0.006, 0.002),
        gyro_noise_std_radps=0.0015,
        accel_bias_mps2=(0.08, -0.05, 0.12),
        accel_noise_std_mps2=0.08,
        attitude_reference_noise_rad=math.radians(0.35),
        attitude_reference_period_s=0.05,
        seed=20260721,
    )
    estimator = AttitudeEstimator(
        initial_attitude=initial.attitude,
        config=AttitudeEstimatorConfig(
            gyro_bias_estimate_radps=(0.010, -0.006, 0.002),
            attitude_reference_gain=0.18,
        ),
    )
    return rocket, env, initial, tvc, sensors, estimator


def write_csv(path: Path, samples: list[EstimatedClosedLoopSample]) -> None:
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
                "gyro_x_radps",
                "gyro_y_radps",
                "gyro_z_radps",
                "accel_x_mps2",
                "accel_y_mps2",
                "accel_z_mps2",
                "attitude_error_deg",
                "rate_error_radps",
                "estimator_correction_rad",
                "q_norm",
                "est_q_norm",
                "tilt_deg",
                "est_tilt_deg",
                "signed_pitch_deg",
                "est_signed_pitch_deg",
                "signed_yaw_deg",
                "est_signed_yaw_deg",
                "body_z_x",
                "body_z_y",
                "body_z_z",
                "est_body_z_x",
                "est_body_z_y",
                "est_body_z_z",
                "lateral_m",
                "gimbal_x_deg",
                "gimbal_y_deg",
                "gimbal_total_deg",
                "requested_torque_norm_nm",
                "achievable_torque_norm_nm",
                "saturated",
            ]
        )
        for sample in samples:
            state = sample.true_state
            est = sample.estimated_state
            command = sample.command
            body_z = body_z_axis_inertial(state)
            est_body_z = body_z_axis_inertial(est)
            gimbal_total = math.degrees(math.sqrt(command.gimbal_x_rad**2 + command.gimbal_y_rad**2))
            writer.writerow(
                [
                    sample.time_s,
                    *state.as_tuple(),
                    *est.attitude,
                    *est.angular_velocity_radps,
                    *sample.measurement.gyro_radps,
                    *sample.measurement.accelerometer_mps2,
                    sample.attitude_error_deg,
                    sample.rate_error_radps,
                    sample.estimate.correction_norm_rad,
                    q_norm(state.attitude),
                    q_norm(est.attitude),
                    tilt_angle_deg(state),
                    tilt_angle_deg(est),
                    signed_pitch_deg(state),
                    signed_pitch_deg(est),
                    signed_yaw_deg(state),
                    signed_yaw_deg(est),
                    *body_z,
                    *est_body_z,
                    lateral_displacement_m(state),
                    math.degrees(command.gimbal_x_rad),
                    math.degrees(command.gimbal_y_rad),
                    gimbal_total,
                    norm(command.requested_torque_body),
                    norm(command.achievable_torque_body),
                    int(command.saturated),
                ]
            )


def main() -> None:
    rocket, env, initial, tvc, sensors, estimator = week5_setup()
    samples = list(
        simulate_estimated_tvc(
            initial,
            rocket,
            env,
            tvc,
            sensors.sampler(),
            estimator,
            duration_s=3.0,
            dt_s=0.005,
        )
    )
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    path = output_dir / "week5_estimated_tvc_controlled.csv"
    write_csv(path, samples)

    state_samples = [(sample.time_s, sample.true_state) for sample in samples]
    metrics = summary_metrics(state_samples, rocket, env)
    saturation_fraction = sum(sample.command.saturated for sample in samples) / len(samples)
    max_attitude_error = max(sample.attitude_error_deg for sample in samples)
    rms_attitude_error = math.sqrt(sum(sample.attitude_error_deg**2 for sample in samples) / len(samples))

    print(f"Wrote {len(samples)} samples to {path}")
    print(f"Final altitude: {metrics['final_altitude_m']:.2f} m")
    print(f"Max tilt angle: {metrics['max_tilt_deg']:.2f} deg")
    print(f"Max lateral displacement: {metrics['max_lateral_displacement_m']:.2f} m")
    print(f"Max attitude estimation error: {max_attitude_error:.2f} deg")
    print(f"RMS attitude estimation error: {rms_attitude_error:.2f} deg")
    print(f"Gimbal saturation fraction: {100.0 * saturation_fraction:.1f}%")


if __name__ == "__main__":
    main()
