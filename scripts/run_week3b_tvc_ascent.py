#!/usr/bin/env python3
"""Run Week 3B ascent controlled by a thrust-vector-control actuator."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import Environment, IdealTorqueController, RocketParams, State, TVCController
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg, summary_metrics, tilt_angle_deg
from rocket_sim.math3d import norm, q_norm
from rocket_sim.tvc_sim import simulate_tvc


def week3b_setup() -> tuple[RocketParams, Environment, State, TVCController]:
    misalignment = math.radians(1.5)
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=2.5,
        center_of_pressure_body_m=(0.0, 0.0, 0.35),
        thrust_direction_body=(0.0, 0.0, 1.0),
    )
    env = Environment(wind_mps=(4.0, 1.0, 0.0))
    initial = State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    ideal = IdealTorqueController(kp_nmpu=120.0, kd_nms=28.0, max_torque_nm=80.0)
    tvc = TVCController(
        ideal_controller=ideal,
        engine_position_body_m=(0.0, 0.0, -1.2),
        max_gimbal_rad=math.radians(5.0),
        thrust_misalignment_body=(math.sin(misalignment), 0.0, 0.0),
    )
    return rocket, env, initial, tvc


def write_csv(path: Path, samples) -> None:
    states = [(time_s, state) for time_s, state, _ in samples]
    raw_pitch = [signed_pitch_deg(s) for _, s in states]
    unwrapped_pitch = []
    offset = 0.0
    previous = raw_pitch[0]
    for pitch in raw_pitch:
        delta = pitch - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        unwrapped_pitch.append(pitch + offset)
        previous = pitch

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
                "unwrapped_pitch_deg",
                "signed_yaw_deg",
                "body_z_x",
                "body_z_y",
                "body_z_z",
                "lateral_m",
                "gimbal_x_deg",
                "gimbal_y_deg",
                "gimbal_total_deg",
                "requested_torque_norm_nm",
                "achievable_torque_norm_nm",
                "saturated",
            ]
        )
        for pitch_unwrapped, (time_s, state, command) in zip(unwrapped_pitch, samples):
            body_z = body_z_axis_inertial(state)
            gimbal_total = math.degrees(math.sqrt(command.gimbal_x_rad**2 + command.gimbal_y_rad**2))
            writer.writerow(
                [
                    time_s,
                    *state.as_tuple(),
                    q_norm(state.attitude),
                    tilt_angle_deg(state),
                    signed_pitch_deg(state),
                    pitch_unwrapped,
                    signed_yaw_deg(state),
                    *body_z,
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
    rocket, env, initial, tvc = week3b_setup()
    samples = list(simulate_tvc(initial, rocket, env, tvc, duration_s=3.0, dt_s=0.005))
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    path = output_dir / "week3b_tvc_controlled.csv"
    write_csv(path, samples)

    state_samples = [(time_s, state) for time_s, state, _ in samples]
    metrics = summary_metrics(state_samples, rocket, env)
    saturation_fraction = sum(command.saturated for _, _, command in samples) / len(samples)
    print(f"Wrote {len(samples)} samples to {path}")
    print(f"Final altitude: {metrics['final_altitude_m']:.2f} m")
    print(f"Max tilt angle: {metrics['max_tilt_deg']:.2f} deg")
    print(f"Max angular rate: {metrics['max_angular_rate_radps']:.2f} rad/s")
    print(f"Max lateral displacement: {metrics['max_lateral_displacement_m']:.2f} m")
    print(f"Gimbal saturation fraction: {100.0 * saturation_fraction:.1f}%")


if __name__ == "__main__":
    main()
