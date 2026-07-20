# 6-DOF Rocket Flight Simulator with TVC, LQR, and Monte Carlo Verification

This repository is a controls/GNC portfolio project for launch-vehicle ascent dynamics. It implements a nonlinear 6-DOF rigid-body rocket simulation, introduces aerodynamic and propulsion disturbances, demonstrates open-loop instability, stabilizes the vehicle with attitude feedback, allocates control through thrust vector control, compares PD and LQR control laws, and verifies robustness with a Monte Carlo dispersion campaign.

The project is intentionally written as an engineering artifact: the code, plots, animation, tests, and writeups are organized so a reviewer can trace the work from first-principles dynamics to closed-loop verification.

## Engineering Summary

| Area | Implementation |
| --- | --- |
| Dynamics | 13-state nonlinear rigid-body model: inertial position, velocity, quaternion attitude, and body angular velocity |
| Integration | Fixed-step RK4 with quaternion normalization and sanity tests |
| Disturbances | Crosswind, thrust misalignment, thrust offset, drag, angle-of-attack normal force, and CP/CM moment arm |
| Control | Ideal body-torque PD, actuator-realistic PD TVC, and LQR TVC |
| Verification | Nominal controlled/uncontrolled comparisons plus 300-case Monte Carlo campaign |
| Presentation | SVG plots, CSV outputs, milestone reports, synchronized HTML animation, and upper-division physics explanations |

## Results At A Glance

Nominal disturbed ascent over a `3 s` simulation window:

| Case | Final altitude | Max tilt | Max lateral drift | Gimbal saturation |
| --- | ---: | ---: | ---: | ---: |
| Open loop | 3.99 m | 177.63 deg | 25.10 m | n/a |
| Ideal torque PD | 31.96 m | 9.88 deg | 5.65 m | n/a |
| PD TVC | 30.97 m | 12.94 deg | 13.24 m | 0.0% |
| LQR TVC | 31.43 m | 10.30 deg | 10.73 m | 0.0% |

Monte Carlo robustness campaign with `100` randomized dispersions per controller:

| Controller | Success rate | Median max tilt | Median max lateral drift | Worst max tilt | Worst lateral drift |
| --- | ---: | ---: | ---: | ---: | ---: |
| Open loop | 1.0% | 177.04 deg | 24.29 m | 179.82 deg | 27.74 m |
| PD TVC | 100.0% | 12.05 deg | 12.69 m | 22.88 deg | 23.56 m |
| LQR TVC | 100.0% | 9.67 deg | 10.35 m | 17.92 deg | 19.10 m |

## Visual Evidence

The main result is that uncontrolled ascent fails under realistic disturbance moments, while TVC feedback keeps the vehicle inside the attitude corridor across the sampled uncertainty envelope.

## Interactive Flight Animation

The project includes a standalone HTML animation generated from the simulator CSV outputs:

[Open the animation artifact](outputs/rocket_flight_animation.html)

The animation compares open-loop failure, ideal body-torque control, PD TVC, and LQR TVC on a synchronized timeline. It shows the rocket attitude, trajectory, body-axis vertical alignment, lateral drift, and gimbal usage in one viewer.

![Rocket flight animation preview](figures/rocket-animation-preview.svg)

![Monte Carlo robustness summary](figures/week4b-monte-carlo-robustness.svg)

![LQR control comparison](figures/week4a-lqr-control-comparison.svg)

## Review Path

| Start here | Purpose |
| --- | --- |
| [FIGURE_INDEX.md](FIGURE_INDEX.md) | Quick visual guide with numerical takeaways and physical interpretation |
| [PORTFOLIO_WRITEUP.md](PORTFOLIO_WRITEUP.md) | Polished project narrative suitable for a portfolio page |
| [docs/figure_results_interpretations.md](docs/figure_results_interpretations.md) | Upper-division explanation of every generated plot |
| [outputs/rocket_flight_animation.html](outputs/rocket_flight_animation.html) | Synchronized animation of open loop, ideal torque, PD TVC, and LQR TVC |
| [outputs/week4b_monte_carlo_results.csv](outputs/week4b_monte_carlo_results.csv) | Trial-by-trial robustness data |

## Flight Physics

The project is written to demonstrate aerospace reasoning, not just software output. Every generated figure has an upper-division physical interpretation tied to forces, moments, state variables, control laws, actuator limits, or verification criteria.

Core flight-physics relationships used throughout the simulator:

**Translational Dynamics**

`m r_ddot_I = R_BI(q) F_B + [0, 0, -mg]`

Body-frame thrust and aerodynamic forces are rotated into the inertial frame before gravity is applied. This is why attitude error immediately becomes trajectory error during ascent.

**Rigid-Body Rotation**

`I omega_dot_B + omega_B x (I omega_B) = tau_B`

The vehicle attitude response depends on inertia and gyroscopic coupling, not just net torque. This is the nonlinear rotational plant stabilized by the PD and LQR controllers.

**Aerodynamic Loading**

`qbar = 0.5 rho |v_rel|^2`

`F_N ~= qbar S C_N_alpha alpha`

Dynamic pressure and angle of attack drive the normal-force model. With CP/CM separation, that side force becomes an aerodynamic moment that can either restore or destabilize the vehicle.

**Thrust Projection**

`T_vertical = T cos(theta)`

`T_lateral = T sin(theta)`

A tumbling rocket can still generate thrust, but the useful vertical component collapses while lateral acceleration grows. This explains the open-loop altitude loss and crossrange drift.

**Moment And TVC Authority**

`tau = r x F`

`tau_TVC = r_engine x F_thrust`

`tau_max,TVC ~= L T sin(delta_max)`

Thrust offsets, CP/CM separation, and engine gimbal commands all produce moments through lever arms. TVC control authority is therefore limited by engine location, thrust magnitude, and maximum gimbal angle.

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
| [figures/week4b-monte-carlo-robustness.svg](figures/week4b-monte-carlo-robustness.svg) | Recruiter-facing Monte Carlo robustness summary |
| [figures/rocket-animation-preview.svg](figures/rocket-animation-preview.svg) | README preview for the interactive rocket-flight animation |
| [figures/week4a-lqr-control-comparison.svg](figures/week4a-lqr-control-comparison.svg) | Open loop vs ideal torque vs PD TVC vs LQR TVC comparison |
| [outputs/rocket_flight_animation.html](outputs/rocket_flight_animation.html) | Synchronized animation of open loop, ideal torque, PD TVC, and LQR TVC |
| [outputs/week4b_monte_carlo_results.csv](outputs/week4b_monte_carlo_results.csv) | Trial-by-trial robustness data |
| [FIGURE_INDEX.md](FIGURE_INDEX.md) | Fast visual guide with numerical takeaways and physical interpretations |
| [docs/figure_results_interpretations.md](docs/figure_results_interpretations.md) | Upper-division explanation of every generated graph |

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
