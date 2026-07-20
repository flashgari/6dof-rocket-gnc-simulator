#!/usr/bin/env python3
"""Run Week 4A ascent controlled by an LQR attitude law through TVC."""

from __future__ import annotations

import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import LQRAttitudeController, TVCController
from rocket_sim.analysis import summary_metrics
from rocket_sim.tvc_sim import simulate_tvc
from scripts.run_week3b_tvc_ascent import week3b_setup, write_csv


def week4a_setup():
    rocket, env, initial, _ = week3b_setup()
    misalignment = math.radians(1.5)
    lqr = LQRAttitudeController(
        q_angle=22500.0,
        q_rate=120.0,
        r_control=1.0,
        inertia_kg_m2=rocket.inertia_kg_m2,
        max_torque_nm=80.0,
    )
    tvc = TVCController(
        ideal_controller=lqr,
        engine_position_body_m=(0.0, 0.0, -1.2),
        max_gimbal_rad=math.radians(5.0),
        thrust_misalignment_body=(math.sin(misalignment), 0.0, 0.0),
    )
    return rocket, env, initial, tvc


def main() -> None:
    rocket, env, initial, tvc = week4a_setup()
    samples = list(simulate_tvc(initial, rocket, env, tvc, duration_s=3.0, dt_s=0.005))
    output_dir = PROJECT_ROOT / "outputs"
    output_dir.mkdir(exist_ok=True)
    path = output_dir / "week4a_lqr_tvc_controlled.csv"
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
