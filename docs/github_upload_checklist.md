# GitHub Upload Checklist

Use this before publishing the repository.

## Required Files

- [x] `README.md`
- [x] `PORTFOLIO_WRITEUP.md`
- [x] `requirements.txt`
- [x] `rocket_sim/`
- [x] `scripts/`
- [x] `tests/`
- [x] `docs/`
- [x] `outputs/`
- [x] `figures/`
- [x] `FIGURE_INDEX.md`

## Must-Show Artifacts

- [x] `outputs/week3b_control_comparison_plots.svg`
- [x] `outputs/week4a_lqr_control_comparison_plots.svg`
- [x] `outputs/week4b_monte_carlo_summary.svg`
- [x] `outputs/week6_actuator_limited_tvc_plots.svg`
- [x] `outputs/week7_variable_mass_plots.svg`
- [x] `outputs/rocket_flight_animation.html`
- [x] `figures/week2-open-loop-failure.svg`
- [x] `figures/week3b-control-comparison.svg`
- [x] `figures/week4a-lqr-control-comparison.svg`
- [x] `figures/week4b-monte-carlo-robustness.svg`
- [x] `figures/week6-actuator-limited-tvc.svg`
- [x] `figures/week7-variable-mass-ascent.svg`
- [x] `docs/figure_results_interpretations.md`

## Verification Command

Run:

```bash
python3 scripts/run_all.py
```

Expected result:

```text
Ran 37 tests
OK
```

## Reviewer Path

For a recruiter or engineer reviewing quickly:

1. Read `README.md`.
2. Read `FIGURE_INDEX.md`.
3. Open `figures/week4b-monte-carlo-robustness.svg`.
4. Open `figures/week4a-lqr-control-comparison.svg`.
5. Open `figures/week6-actuator-limited-tvc.svg`.
6. Open `figures/week7-variable-mass-ascent.svg`.
7. Open `outputs/rocket_flight_animation.html`.
8. Read `PORTFOLIO_WRITEUP.md`.
9. Read `docs/figure_results_interpretations.md` for plot-by-plot physics.

## Interview Prep

Be ready to explain:

- quaternion attitude propagation
- `I omega_dot + omega x I omega = tau`
- CP/CM aerodynamic moment
- thrust projection through `T cos(theta)` and `T sin(theta)`
- TVC moment authority through `L T sin(delta_max)`
- why ideal torque is not actuator-realistic
- why LQR is local
- why Monte Carlo robustness matters
- why finite TVC bandwidth changes phase margin and actuator authority
- why mass depletion, changing inertia, and CM shift motivate gain scheduling
