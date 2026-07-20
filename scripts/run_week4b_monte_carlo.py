#!/usr/bin/env python3
"""Run Week 4B Monte Carlo robustness campaign."""

from __future__ import annotations

import csv
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import Environment, IdealTorqueController, LQRAttitudeController, RocketParams, State, TVCController
from rocket_sim.analysis import summary_metrics
from rocket_sim.sim import simulate
from rocket_sim.tvc_sim import simulate_tvc

OUT = PROJECT_ROOT / "outputs"
SEED = 4242
TRIALS = 100
DURATION_S = 3.0
DT_S = 0.01


@dataclass(frozen=True)
class TrialParameters:
    trial: int
    mass_kg: float
    inertia_scale: float
    thrust_n: float
    wind_x_mps: float
    wind_y_mps: float
    thrust_misalign_deg: float
    thrust_offset_x_m: float
    cp_z_m: float
    cn_alpha: float
    max_gimbal_deg: float


def sample_trial(rng: random.Random, trial: int) -> TrialParameters:
    wind_speed = rng.uniform(0.0, 8.0)
    wind_dir = rng.uniform(0.0, 2.0 * math.pi)
    return TrialParameters(
        trial=trial,
        mass_kg=rng.uniform(47.5, 52.5),
        inertia_scale=rng.uniform(0.85, 1.15),
        thrust_n=rng.uniform(820.0, 880.0),
        wind_x_mps=wind_speed * math.cos(wind_dir),
        wind_y_mps=wind_speed * math.sin(wind_dir),
        thrust_misalign_deg=rng.uniform(0.0, 2.5),
        thrust_offset_x_m=rng.uniform(0.0, 0.007),
        cp_z_m=rng.uniform(0.15, 0.55),
        cn_alpha=rng.uniform(1.8, 3.2),
        max_gimbal_deg=rng.uniform(4.0, 6.0),
    )


def rocket_and_env(params: TrialParameters, closed_loop: bool) -> tuple[RocketParams, Environment]:
    misalign = math.radians(params.thrust_misalign_deg)
    inertia = (3.0 * params.inertia_scale, 3.0 * params.inertia_scale, 0.45 * params.inertia_scale)
    thrust_direction = (0.0, 0.0, 1.0) if closed_loop else (math.sin(misalign), 0.0, math.cos(misalign))
    thrust_offset = (0.0, 0.0, 0.0) if closed_loop else (params.thrust_offset_x_m, 0.0, 0.0)
    rocket = RocketParams(
        mass_kg=params.mass_kg,
        inertia_kg_m2=inertia,
        thrust_n=params.thrust_n,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=params.cn_alpha,
        center_of_pressure_body_m=(0.0, 0.0, params.cp_z_m),
        thrust_offset_body_m=thrust_offset,
        thrust_direction_body=thrust_direction,
    )
    env = Environment(wind_mps=(params.wind_x_mps, params.wind_y_mps, 0.0))
    return rocket, env


def initial_state() -> State:
    return State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def pd_tvc_controller(params: TrialParameters) -> TVCController:
    misalign = math.radians(params.thrust_misalign_deg)
    ideal = IdealTorqueController(kp_nmpu=120.0, kd_nms=28.0, max_torque_nm=80.0)
    return TVCController(
        ideal_controller=ideal,
        engine_position_body_m=(0.0, 0.0, -1.2),
        max_gimbal_rad=math.radians(params.max_gimbal_deg),
        thrust_misalignment_body=(math.sin(misalign), 0.0, 0.0),
    )


def lqr_tvc_controller(params: TrialParameters, rocket: RocketParams) -> TVCController:
    misalign = math.radians(params.thrust_misalign_deg)
    lqr = LQRAttitudeController(
        q_angle=22500.0,
        q_rate=120.0,
        r_control=1.0,
        inertia_kg_m2=rocket.inertia_kg_m2,
        max_torque_nm=80.0,
    )
    return TVCController(
        ideal_controller=lqr,
        engine_position_body_m=(0.0, 0.0, -1.2),
        max_gimbal_rad=math.radians(params.max_gimbal_deg),
        thrust_misalignment_body=(math.sin(misalign), 0.0, 0.0),
    )


def saturation_fraction(samples) -> float:
    if not samples or len(samples[0]) < 3:
        return 0.0
    return sum(command.saturated for _, _, command in samples) / len(samples)


def pass_case(metrics: dict[str, float], sat_fraction: float) -> bool:
    return (
        metrics["max_tilt_deg"] < 25.0
        and metrics["final_altitude_m"] > 20.0
        and metrics["max_lateral_displacement_m"] < 25.0
        and sat_fraction < 0.10
    )


def run_case(controller_name: str, params: TrialParameters) -> dict[str, float | str | int]:
    closed_loop = controller_name != "open_loop"
    rocket, env = rocket_and_env(params, closed_loop=closed_loop)
    if controller_name == "open_loop":
        samples = list(simulate(initial_state(), rocket, env, DURATION_S, DT_S))
        state_samples = samples
        sat = 0.0
    elif controller_name == "pd_tvc":
        tvc_samples = list(simulate_tvc(initial_state(), rocket, env, pd_tvc_controller(params), DURATION_S, DT_S))
        state_samples = [(time_s, state) for time_s, state, _ in tvc_samples]
        sat = saturation_fraction(tvc_samples)
    elif controller_name == "lqr_tvc":
        tvc_samples = list(simulate_tvc(initial_state(), rocket, env, lqr_tvc_controller(params, rocket), DURATION_S, DT_S))
        state_samples = [(time_s, state) for time_s, state, _ in tvc_samples]
        sat = saturation_fraction(tvc_samples)
    else:
        raise ValueError(f"Unknown controller: {controller_name}")

    metrics = summary_metrics(state_samples, rocket, env)
    passed = pass_case(metrics, sat)
    return {
        "trial": params.trial,
        "controller": controller_name,
        "passed": int(passed),
        "final_altitude_m": metrics["final_altitude_m"],
        "max_altitude_m": metrics["max_altitude_m"],
        "max_tilt_deg": metrics["max_tilt_deg"],
        "final_tilt_deg": metrics["final_tilt_deg"],
        "max_lateral_m": metrics["max_lateral_displacement_m"],
        "final_lateral_m": metrics["final_lateral_displacement_m"],
        "max_angular_rate_radps": metrics["max_angular_rate_radps"],
        "min_body_z": metrics["min_body_z_vertical_component"],
        "saturation_fraction": sat,
        "mass_kg": params.mass_kg,
        "inertia_scale": params.inertia_scale,
        "thrust_n": params.thrust_n,
        "wind_x_mps": params.wind_x_mps,
        "wind_y_mps": params.wind_y_mps,
        "wind_speed_mps": math.hypot(params.wind_x_mps, params.wind_y_mps),
        "thrust_misalign_deg": params.thrust_misalign_deg,
        "thrust_offset_x_m": params.thrust_offset_x_m,
        "cp_z_m": params.cp_z_m,
        "cn_alpha": params.cn_alpha,
        "max_gimbal_deg": params.max_gimbal_deg,
    }


def fmt(value: object) -> object:
    if isinstance(value, float):
        return f"{value:.6g}"
    return value


def write_results(path: Path, rows: list[dict[str, float | str | int]]) -> None:
    fields = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(value) for key, value in row.items()})


def summary(rows: list[dict[str, float | str | int]]) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for controller in ("open_loop", "pd_tvc", "lqr_tvc"):
        subset = [row for row in rows if row["controller"] == controller]
        result[controller] = {
            "trials": float(len(subset)),
            "success_rate": sum(float(row["passed"]) for row in subset) / len(subset),
            "median_max_tilt_deg": sorted(float(row["max_tilt_deg"]) for row in subset)[len(subset) // 2],
            "median_max_lateral_m": sorted(float(row["max_lateral_m"]) for row in subset)[len(subset) // 2],
            "worst_max_tilt_deg": max(float(row["max_tilt_deg"]) for row in subset),
            "worst_max_lateral_m": max(float(row["max_lateral_m"]) for row in subset),
            "mean_saturation_fraction": sum(float(row["saturation_fraction"]) for row in subset) / len(subset),
        }
    return result


def make_bar_svg(summary_data: dict[str, dict[str, float]], path: Path) -> None:
    labels = ["open_loop", "pd_tvc", "lqr_tvc"]
    names = {"open_loop": "open loop", "pd_tvc": "PD TVC", "lqr_tvc": "LQR TVC"}
    colors = {"open_loop": "#dc2626", "pd_tvc": "#059669", "lqr_tvc": "#0891b2"}
    metrics = [
        ("success_rate", "Success rate", 0.0, 1.0, "%"),
        ("median_max_tilt_deg", "Median max tilt", 0.0, 180.0, "deg"),
        ("median_max_lateral_m", "Median max lateral drift", 0.0, 35.0, "m"),
    ]
    panels = []
    for idx, (key, title, ymin, ymax, unit) in enumerate(metrics):
        top = 100 + idx * 180
        left = 92
        width = 650
        height = 112
        bars = []
        for j, label in enumerate(labels):
            value = summary_data[label][key]
            shown = value * 100.0 if unit == "%" else value
            scale_value = value if unit == "%" else value
            bar_h = (scale_value - ymin) / (ymax - ymin) * height
            x = left + 70 + j * 170
            y = top + height - bar_h
            bars.append(
                f'<rect x="{x}" y="{y:.2f}" width="82" height="{bar_h:.2f}" fill="{colors[label]}"/>'
                f'<text x="{x+41}" y="{y-8:.2f}" class="value" text-anchor="middle">{shown:.1f}{unit}</text>'
                f'<text x="{x+41}" y="{top+height+24}" class="legend" text-anchor="middle">{names[label]}</text>'
            )
        panels.append(
            f'<text x="{left}" y="{top-20}" class="panel-title">{title}</text>'
            f'<line x1="{left}" y1="{top+height}" x2="{left+width}" y2="{top+height}" class="axis"/>'
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+height}" class="axis"/>'
            + "".join(bars)
        )
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="820" height="660" viewBox="0 0 820 660">'
        '<style>.bg{fill:#f8fafc}.title{font:700 16px system-ui;fill:#111827}.subtitle{font:500 13px system-ui;fill:#475569}.panel-title{font:700 15px system-ui;fill:#111827}.axis{stroke:#334155;stroke-width:1.2}.legend{font:600 12px system-ui;fill:#475569}.value{font:700 12px system-ui;fill:#111827}</style>'
        '<rect class="bg" width="820" height="660"/>'
        '<text x="34" y="42" class="title">Week 4B Monte Carlo Robustness Summary</text>'
        f'<text x="34" y="64" class="subtitle">{TRIALS} randomized dispersions per controller, fixed seed {SEED}.</text>'
        + "".join(panels)
        + "</svg>"
    )
    path.write_text(svg)


def main() -> None:
    rng = random.Random(SEED)
    rows: list[dict[str, float | str | int]] = []
    for trial in range(TRIALS):
        params = sample_trial(rng, trial)
        for controller in ("open_loop", "pd_tvc", "lqr_tvc"):
            rows.append(run_case(controller, params))

    OUT.mkdir(exist_ok=True)
    results_path = OUT / "week4b_monte_carlo_results.csv"
    write_results(results_path, rows)
    summary_data = summary(rows)
    make_bar_svg(summary_data, OUT / "week4b_monte_carlo_summary.svg")

    print(f"Wrote {len(rows)} case results to {results_path}")
    for controller, values in summary_data.items():
        print(
            f"{controller}: success {100.0 * values['success_rate']:.1f}%, "
            f"median tilt {values['median_max_tilt_deg']:.1f} deg, "
            f"median lateral {values['median_max_lateral_m']:.1f} m"
        )


if __name__ == "__main__":
    main()
