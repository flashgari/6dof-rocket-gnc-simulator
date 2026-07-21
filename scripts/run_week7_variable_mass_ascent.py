#!/usr/bin/env python3
"""Run Week 7 ascent with thrust curve, mass depletion, and changing inertia."""

from __future__ import annotations

import csv
import math
import sys
from dataclasses import replace
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim.actuators import GimbalActuator, max_tvc_torque_nm
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg, summary_metrics, tilt_angle_deg
from rocket_sim.math3d import norm, q_norm
from rocket_sim.propulsion import MassPropertySchedule, ThrustCurve, TimeVaryingRocket
from rocket_sim.variable_mass_sim import VariableMassActuatorSample, simulate_variable_mass_actuator_limited_tvc
from scripts.run_week4a_lqr_tvc_ascent import week4a_setup
from scripts.run_week6_actuator_limited_tvc import week6_actuator_config


def week7_vehicle_setup() -> tuple[TimeVaryingRocket, object, object, object]:
    rocket, env, initial, tvc = week4a_setup()
    thrust_curve = ThrustCurve(
        (
            (0.0, 790.0),
            (0.25, 860.0),
            (1.5, 900.0),
            (2.4, 835.0),
            (3.0, 760.0),
        )
    )
    mass_properties = MassPropertySchedule(
        initial_mass_kg=50.0,
        dry_mass_kg=42.0,
        initial_inertia_kg_m2=(3.15, 3.15, 0.48),
        dry_inertia_kg_m2=(2.55, 2.55, 0.38),
        initial_center_of_mass_body_m=(0.0, 0.0, -0.08),
        dry_center_of_mass_body_m=(0.0, 0.0, 0.10),
        isp_s=245.0,
        thrust_curve=thrust_curve,
        gravity_mps2=env.gravity_mps2,
    )
    base = replace(
        rocket,
        mass_kg=mass_properties.initial_mass_kg,
        inertia_kg_m2=mass_properties.initial_inertia_kg_m2,
        thrust_n=thrust_curve.thrust_n(0.0),
        center_of_mass_body_m=mass_properties.initial_center_of_mass_body_m,
    )
    vehicle = TimeVaryingRocket(base, mass_properties)
    return vehicle, env, initial, tvc


def write_csv(path: Path, samples: list[VariableMassActuatorSample], tvc) -> None:
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
                "mass_kg",
                "propellant_fraction",
                "thrust_n",
                "thrust_to_mass_mps2",
                "inertia_x_kg_m2",
                "inertia_y_kg_m2",
                "inertia_z_kg_m2",
                "center_of_mass_z_m",
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
            state = sample.state
            body_z = body_z_axis_inertial(state)
            requested = sample.requested_command
            achieved = sample.achieved_command
            requested_torque = norm(requested.requested_torque_body)
            achievable_torque = norm(achieved.achievable_torque_body)
            instantaneous_rocket = sample_to_rocket_like(sample)
            available_torque = max_tvc_torque_nm(instantaneous_rocket, tvc, tvc.max_gimbal_rad)
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
                    sample.mass_kg,
                    sample.propellant_fraction,
                    sample.thrust_n,
                    sample.thrust_n / sample.mass_kg,
                    *sample.inertia_kg_m2,
                    sample.center_of_mass_body_m[2],
                    math.degrees(math.sqrt(requested.gimbal_x_rad**2 + requested.gimbal_y_rad**2)),
                    math.degrees(math.sqrt(achieved.gimbal_x_rad**2 + achieved.gimbal_y_rad**2)),
                    math.degrees(sample.actuator_output.command_error_rad),
                    requested_torque,
                    achievable_torque,
                    available_torque,
                    available_torque - requested_torque,
                    int(sample.actuator_output.rate_limited),
                    int(sample.actuator_output.position_limited),
                    int(achieved.saturated),
                ]
            )


def sample_to_rocket_like(sample: VariableMassActuatorSample):
    class RocketLike:
        pass

    rocket = RocketLike()
    rocket.thrust_n = sample.thrust_n
    return rocket


def main() -> None:
    vehicle, env, initial, tvc = week7_vehicle_setup()
    samples = list(
        simulate_variable_mass_actuator_limited_tvc(
            initial,
            vehicle,
            env,
            tvc,
            GimbalActuator(week6_actuator_config()),
            duration_s=3.0,
            dt_s=0.005,
        )
    )
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    path = output_dir / "week7_variable_mass_lqr_tvc.csv"
    write_csv(path, samples, tvc)

    metrics = summary_metrics([(sample.time_s, sample.state) for sample in samples], vehicle.at(0.0), env)
    max_lag = max(math.degrees(sample.actuator_output.command_error_rad) for sample in samples)
    rate_fraction = sum(sample.actuator_output.rate_limited for sample in samples) / len(samples)
    initial_mass = samples[0].mass_kg
    final_mass = samples[-1].mass_kg
    initial_t_over_m = samples[0].thrust_n / samples[0].mass_kg
    final_t_over_m = samples[-1].thrust_n / samples[-1].mass_kg

    print(f"Wrote {len(samples)} samples to {path}")
    print(f"Final altitude: {metrics['final_altitude_m']:.2f} m")
    print(f"Max tilt angle: {metrics['max_tilt_deg']:.2f} deg")
    print(f"Max lateral displacement: {metrics['max_lateral_displacement_m']:.2f} m")
    print(f"Mass change: {initial_mass:.2f} kg -> {final_mass:.2f} kg")
    print(f"T/m change: {initial_t_over_m:.2f} m/s^2 -> {final_t_over_m:.2f} m/s^2")
    print(f"Max gimbal lag: {max_lag:.2f} deg")
    print(f"Rate-limit fraction: {100.0 * rate_fraction:.1f}%")


if __name__ == "__main__":
    main()
