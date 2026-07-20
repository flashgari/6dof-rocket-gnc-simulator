#!/usr/bin/env python3
"""Write a concise Week 1 milestone report from the generated ascent CSV."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import summary_metrics

OUTPUT_DIR = PROJECT_ROOT / "outputs"
CSV_PATH = OUTPUT_DIR / "week1_ascent.csv"
REPORT_PATH = OUTPUT_DIR / "week1_milestone_report.md"


def load_samples() -> list[tuple[float, State]]:
    with CSV_PATH.open() as f:
        rows = csv.DictReader(f)
        samples = []
        for row in rows:
            time_s = float(row["time_s"])
            state = State(
                position_m=(float(row["x_m"]), float(row["y_m"]), float(row["z_m"])),
                velocity_mps=(float(row["vx_mps"]), float(row["vy_mps"]), float(row["vz_mps"])),
                attitude=(float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])),
                angular_velocity_radps=(
                    float(row["wx_radps"]),
                    float(row["wy_radps"]),
                    float(row["wz_radps"]),
                ),
            )
            samples.append((time_s, state))
    return samples


def main() -> None:
    rocket = RocketParams(mass_kg=50.0, inertia_kg_m2=(3.0, 3.0, 0.45), thrust_n=850.0, drag_coefficient=0.0)
    env = Environment()
    metrics = summary_metrics(load_samples(), rocket, env)

    report = f"""# Week 1 Milestone Report

## Objective

Build and verify the open-loop dynamics core for a 6-DOF rocket ascent simulator. The Week 1 configuration is intentionally simple: aligned thrust, no aerodynamic drag, no wind, and no control. This gives a clean baseline before adding disturbances and attitude control.

## Implemented

- 13-state rigid-body model: inertial position, inertial velocity, body-to-inertial quaternion, and body angular velocity
- Quaternion kinematics with scalar-first `[w, x, y, z]` convention
- Translational dynamics with thrust and gravity
- Rotational dynamics using Euler's rigid-body equation
- Fixed-step RK4 integration
- CSV trajectory export and dependency-free SVG plots
- Unit tests for straight ascent, quaternion normalization, ballistic energy conservation, force-free linear momentum conservation, torque-free angular momentum conservation, and torque response

## Baseline Case

| Metric | Value |
| --- | ---: |
| Duration | {metrics["duration_s"]:.2f} s |
| Samples | {metrics["samples"]:.0f} |
| Final altitude | {metrics["final_altitude_m"]:.2f} m |
| Final vertical velocity | {metrics["final_vertical_velocity_mps"]:.2f} m/s |
| Maximum speed | {metrics["max_speed_mps"]:.2f} m/s |
| Maximum tilt angle | {metrics["max_tilt_deg"]:.6f} deg |
| Maximum quaternion norm error | {metrics["max_quaternion_norm_error"]:.3e} |

## Sanity Checks

The straight-ascent case matches the closed-form constant-acceleration solution:

```text
a_z = T / m - g
z(t) = 0.5 a_z t^2
v_z(t) = a_z t
```

With zero thrust and zero drag, the ballistic test conserves specific mechanical energy. With gravity disabled and no external forces, linear momentum remains constant. With no external torques, inertial angular momentum remains constant. With an initial angular velocity, the quaternion is renormalized after each RK4 state construction and remains unit length to numerical precision.

## Completion Checklist

- [x] 13-state vector: position, velocity, quaternion, angular velocity
- [x] Quaternion-first attitude representation
- [x] Translational equations of motion
- [x] Rotational equations of motion
- [x] RK4 integration
- [x] Straight-up open-loop ascent case
- [x] Quaternion normalization check
- [x] Energy conservation check
- [x] Linear momentum conservation check
- [x] Torque-free angular momentum conservation check
- [x] CSV output, SVG plots, and milestone report

## Week 2 Entry Point

The next milestone should deliberately break this clean baseline by adding thrust misalignment, wind, and aerodynamic center-of-pressure moments. The uncontrolled vehicle should tumble, which will create a clear before/after comparison once attitude control is added in Week 3.
"""
    OUTPUT_DIR.mkdir(exist_ok=True)
    REPORT_PATH.write_text(report)
    print(f"Wrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
