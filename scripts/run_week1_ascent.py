#!/usr/bin/env python3
"""Run the Week 1 straight-ascent case and write a CSV trajectory."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import summary_metrics
from rocket_sim.math3d import q_norm
from rocket_sim.sim import simulate


def main() -> None:
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        drag_coefficient=0.0,
    )
    env = Environment()
    initial_state = State(
        position_m=(0.0, 0.0, 0.0),
        velocity_mps=(0.0, 0.0, 0.0),
        attitude=(1.0, 0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0),
    )

    samples = list(simulate(initial_state, rocket, env, duration_s=10.0, dt_s=0.01))

    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "week1_ascent.csv"
    with output_path.open("w", newline="") as f:
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
            ]
        )
        for time_s, state in samples:
            writer.writerow([time_s, *state.as_tuple(), q_norm(state.attitude)])

    final_time, final_state = samples[-1]
    metrics = summary_metrics(samples, rocket, env)
    print(f"Wrote {len(samples)} samples to {output_path}")
    print(f"Final time: {final_time:.2f} s")
    print(f"Final altitude: {final_state.position_m[2]:.2f} m")
    print(f"Final vertical velocity: {final_state.velocity_mps[2]:.2f} m/s")
    print(f"Final quaternion norm: {q_norm(final_state.attitude):.12f}")
    print(f"Max tilt angle: {metrics['max_tilt_deg']:.6f} deg")
    print(f"Max quaternion norm error: {metrics['max_quaternion_norm_error']:.3e}")


if __name__ == "__main__":
    main()
