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
- [x] `VERIFICATION_MATRIX.md`

## Must-Show Artifacts

- [x] `outputs/rocket_flight_animation.html`
- [x] `figures/rocket-animation-preview.svg`
- [x] `figures/control-system-evidence.svg`
- [x] `figures/monte-carlo-control-envelope.svg`
- [x] `figures/estimated-state-control-evidence.svg`
- [x] `figures/actuator-bandwidth-evidence.svg`
- [x] `figures/variable-mass-evidence.svg`
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
2. Open the rendered animation preview:
   `https://htmlpreview.github.io/?https://github.com/flashgari/6dof-rocket-gnc-simulator/blob/main/outputs/rocket_flight_animation.html`
3. Read `FIGURE_INDEX.md`.
4. Open `figures/control-system-evidence.svg`.
5. Open `figures/monte-carlo-control-envelope.svg`.
6. Open `figures/estimated-state-control-evidence.svg`.
7. Open `figures/actuator-bandwidth-evidence.svg`.
8. Open `figures/variable-mass-evidence.svg`.
9. Read `VERIFICATION_MATRIX.md` and `PORTFOLIO_WRITEUP.md`.

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
