# 6-DOF Rocket Flight Simulator with TVC, LQR, and Monte Carlo Verification

Python launch-vehicle ascent simulation built as a GNC/controls portfolio project. The project models nonlinear rigid-body motion with quaternion attitude, adds aerodynamic and propulsion disturbances, implements attitude control, allocates torque through thrust vector control, compares PD and LQR feedback, and verifies robustness with a Monte Carlo dispersion campaign.

The engineering story is:

```text
rigid-body dynamics -> disturbance-driven open-loop failure -> feedback control -> TVC actuator allocation -> LQR comparison -> Monte Carlo robustness
```

## Start Here

For a fast technical review, open these in order:

1. `FIGURE_INDEX.md` - recruiter-facing guide to the key visuals and what each result means physically.
2. `figures/week4b-monte-carlo-robustness.svg` - strongest robustness evidence across randomized dispersions.
3. `figures/week4a-lqr-control-comparison.svg` - nominal comparison from open-loop failure through LQR TVC.
4. `outputs/rocket_flight_animation.html` - synchronized visual comparison of open loop, ideal torque, PD TVC, and LQR TVC.
5. `PORTFOLIO_WRITEUP.md` - polished engineering narrative for the full project.
6. `docs/figure_results_interpretations.md` - upper-division plot-by-plot physics explanations.

## Key Results

Nominal disturbed ascent, `3 s` simulation window:

| Case | Final altitude | Max tilt | Max lateral drift | Gimbal saturation |
| --- | ---: | ---: | ---: | ---: |
| Open loop | 3.99 m | 177.63 deg | 25.10 m | n/a |
| Ideal torque PD | 31.96 m | 9.88 deg | 5.65 m | n/a |
| PD TVC | 30.97 m | 12.94 deg | 13.24 m | 0.0% |
| LQR TVC | 31.43 m | 10.30 deg | 10.73 m | 0.0% |

Monte Carlo robustness campaign, 100 randomized dispersions per controller:

| Controller | Success rate | Median max tilt | Median max lateral drift | Worst max tilt | Worst lateral drift |
| --- | ---: | ---: | ---: | ---: | ---: |
| Open loop | 1.0% | 177.04 deg | 24.29 m | 179.82 deg | 27.74 m |
| PD TVC | 100.0% | 12.05 deg | 12.69 m | 22.88 deg | 23.56 m |
| LQR TVC | 100.0% | 9.67 deg | 10.35 m | 17.92 deg | 19.10 m |

## Why This Matters

The project is written to demonstrate aerospace reasoning, not just software output. Every generated figure has an upper-division physical interpretation tied to forces, moments, state variables, control laws, actuator limits, or verification criteria.

Key physical relationships used throughout:

```text
m r_ddot_I = R_BI(q) F_B + [0, 0, -mg]
I omega_dot_B + omega_B x (I omega_B) = tau_B
tau = r x F
qbar = 0.5 rho |v_rel|^2
F_N ~ qbar S C_N_alpha alpha
T_vertical = T cos(theta)
T_lateral = T sin(theta)
tau_TVC = r_engine x F_thrust
tau_max,TVC ~= L T sin(delta_max)
```

The plot-by-plot results guide is:

```text
docs/figure_results_interpretations.md
```

The polished portfolio writeup is:

```text
PORTFOLIO_WRITEUP.md
```

## How To Run

The project uses only the Python standard library.

```bash
python3 scripts/run_all.py
```

This regenerates simulations, plots, milestone reports, the animation HTML, the Monte Carlo campaign, and the test suite.

Run tests only:

```bash
python3 -m unittest discover -s tests
```

Current verification:

```text
Ran 23 tests
OK
```

## Primary Artifacts

| Artifact | Purpose |
| --- | --- |
| `outputs/week3b_control_comparison_plots.svg` | Open loop vs ideal torque vs TVC comparison |
| `outputs/week4a_lqr_control_comparison_plots.svg` | Open loop vs ideal torque vs PD TVC vs LQR TVC |
| `outputs/week4b_monte_carlo_summary.svg` | Robustness summary over randomized dispersions |
| `outputs/rocket_flight_animation.html` | Synchronized animation of open loop, ideal torque, PD TVC, and LQR TVC |
| `outputs/week4b_monte_carlo_results.csv` | Trial-by-trial robustness data |
| `figures/` | Stable recruiter-facing copies of the main generated plots |
| `FIGURE_INDEX.md` | Fast visual guide with numerical takeaways and physical interpretations |
| `docs/figure_results_interpretations.md` | Upper-division explanation of every generated graph |

## Technical Scope

### Week 1: Dynamics Core

- 13-state rigid-body model: inertial position, inertial velocity, body-to-inertial quaternion, body angular velocity
- Quaternion attitude propagation
- Translational dynamics with thrust and gravity
- Rotational dynamics using Euler's rigid-body equation
- Fixed-step RK4 integration
- Conservation and sanity tests

### Week 2: Disturbances And Open-Loop Failure

- Crosswind and relative-wind calculation
- Thrust misalignment and thrust offset
- Aerodynamic drag
- Angle-of-attack normal force
- CP/CM aerodynamic moment
- Unwrapped attitude plotting to avoid false recovery after inverted flight

### Week 3A: Ideal-Torque Attitude Control

- Body-axis attitude error:

```text
e_I = z_body,I x z_cmd,I
tau_cmd,B = Kp e_B - Kd omega_B
```

- Bounded ideal body torque
- Controlled-vs-uncontrolled comparison
- Verification that the feedback law stabilizes the nonlinear rigid body before actuator allocation

### Week 3B: Thrust Vector Control

- TVC allocation through:

```text
tau_TVC = r_engine x F_thrust
```

- Engine lever arm
- Maximum gimbal angle
- Requested vs achievable torque telemetry
- Saturation tracking

### Week 4A: LQR Controller

- Small-angle linearized attitude model:

```text
theta_dot = omega
omega_dot = tau / I
```

- Infinite-horizon LQR feedback with `Q/R` state-error/control-effort trade
- Nonlinear verification through the same TVC actuator model

### Week 4B: Monte Carlo Robustness

- 100 deterministic randomized dispersions with fixed seed `4242`
- Open loop, PD TVC, and LQR TVC evaluated for each dispersion
- Randomized wind, mass, inertia, thrust, thrust alignment, CP location, normal-force slope, and gimbal authority
- Pass/fail gates for max tilt, final altitude, max lateral drift, and gimbal saturation

## Repository Layout

```text
rocket_sim/
  analysis.py        derived metrics and physical summary quantities
  control.py         ideal torque, TVC, and LQR controllers
  controlled_sim.py  ideal-torque closed-loop integration
  dynamics.py        force, moment, and 13-state derivative model
  integrators.py     RK4 integration
  math3d.py          vector and quaternion utilities
  models.py          State, RocketParams, Environment dataclasses
  sim.py             open-loop simulation loop
  tvc_sim.py         TVC closed-loop simulation loop
scripts/
  run_all.py
  run_week1_ascent.py
  run_week2_disturbed_ascent.py
  run_week3a_controlled_ascent.py
  run_week3b_tvc_ascent.py
  run_week4a_lqr_tvc_ascent.py
  run_week4b_monte_carlo.py
  plot_outputs.py
  write_reports.py
  build_animation.py
tests/
  test_week1_dynamics.py
  test_week2_disturbances.py
  test_week3a_control.py
  test_week3b_tvc.py
  test_week4a_lqr.py
  test_week4b_monte_carlo.py
docs/
  technical_physics_notes.md
  figure_results_interpretations.md
  week1_equations.md
  week2_disturbance_model.md
  week3a_ideal_torque_control.md
  week3b_tvc_control.md
  week4a_lqr_control.md
  week4b_monte_carlo.md
  animation_viewer.md
outputs/
  generated CSV, SVG, HTML, and milestone reports
figures/
  recruiter-facing copies of the main SVG plots
FIGURE_INDEX.md
  fast visual review guide for the main plots
```

## Limitations And Next Work

- Aerodynamics use a simplified normal-force model rather than full coefficient tables.
- Mass, inertia, and thrust are constant during the burn.
- TVC dynamics are instantaneous; actuator rate limits and servo dynamics are not yet modeled.
- LQR is designed around the upright operating point and is not a global tumble-recovery controller.
- Future extensions: sensor simulation, state estimation, actuator rate limits, gain scheduling, and higher-fidelity atmosphere/aerodynamics.

## Interview Talking Points

- Why quaternions are used for large attitude excursions and tumble cases.
- How CP/CM offset produces aerodynamic moment through `(r_CP - r_CM) x F_N`.
- Why open loop loses altitude when `T cos(theta)` collapses.
- Why TVC couples attitude correction to lateral acceleration.
- Why ideal torque is useful for control-law verification but not actuator-realistic.
- Why LQR is local and must be verified in the nonlinear plant.
- Why Monte Carlo robustness is stronger evidence than one nominal trajectory.
