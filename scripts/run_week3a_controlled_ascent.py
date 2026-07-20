#!/usr/bin/env python3
"""Run Week 3A closed-loop ascent with an ideal body-torque controller."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import Environment, IdealTorqueController, RocketParams, State
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg, summary_metrics, tilt_angle_deg
from rocket_sim.math3d import norm, q_norm
from rocket_sim.controlled_sim import simulate_controlled


def week3a_setup() -> tuple[RocketParams, Environment, State, IdealTorqueController]:
    misalign = math.radians(1.5)
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=2.5,
        center_of_pressure_body_m=(0.0, 0.0, 0.35),
        thrust_offset_body_m=(0.004, 0.0, 0.0),
        thrust_direction_body=(math.sin(misalign), 0.0, math.cos(misalign)),
    )
    env = Environment(wind_mps=(4.0, 1.0, 0.0))
    initial = State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    controller = IdealTorqueController(kp_nmpu=18.0, kd_nms=8.0, max_torque_nm=40.0)
    return rocket, env, initial, controller


def write_csv(path: Path, samples: list[tuple[float, State]], controller: IdealTorqueController) -> None:
    raw_pitch = [signed_pitch_deg(s) for _, s in samples]
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
                "control_tx_nm",
                "control_ty_nm",
                "control_tz_nm",
                "control_torque_norm_nm",
            ]
        )
        for pitch_unwrapped, (time_s, state) in zip(unwrapped_pitch, samples):
            body_z = body_z_axis_inertial(state)
            torque = controller.torque_body(state)
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
                    *torque,
                    norm(torque),
                ]
            )


def main() -> None:
    rocket, env, initial, controller = week3a_setup()
    samples = list(simulate_controlled(initial, rocket, env, controller, duration_s=3.0, dt_s=0.005))
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    path = output_dir / "week3a_controlled_ideal_torque.csv"
    write_csv(path, samples, controller)

    metrics = summary_metrics(samples, rocket, env)
    print(f"Wrote {len(samples)} samples to {path}")
    print(f"Final altitude: {metrics['final_altitude_m']:.2f} m")
    print(f"Max tilt angle: {metrics['max_tilt_deg']:.2f} deg")
    print(f"Max angular rate: {metrics['max_angular_rate_radps']:.2f} rad/s")
    print(f"Max lateral displacement: {metrics['max_lateral_displacement_m']:.2f} m")


if __name__ == "__main__":
    main()
