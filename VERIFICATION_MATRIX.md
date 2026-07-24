# Verification Matrix

This matrix connects each engineering claim to an observable quantity, an acceptance criterion, and reproducible evidence. The thresholds are project-level comparison gates for this demonstration; they are not launch-vehicle mission requirements.

| ID | Requirement | Acceptance criterion | Primary evidence | Automated check |
| --- | --- | --- | --- | --- |
| DYN-01 | Straight, aligned thrust produces one-dimensional ascent | Numerical position and velocity agree with constant-acceleration kinematics | `outputs/week1_ascent.csv` | `test_straight_ascent_matches_constant_acceleration` |
| DYN-02 | Quaternion propagation remains on the unit sphere | `abs(||q|| - 1) < 1e-12` at sampled states | Baseline state history | `test_quaternion_stays_normalized` |
| DYN-03 | Conservative cases preserve the expected invariants | Ballistic energy and force-free linear/angular momentum remain within test tolerance | Unit-test output | Energy and momentum tests in `test_week1_dynamics.py` |
| AERO-01 | Wind-relative flow generates normal force and CP/CM moment | Nonzero crosswind produces angle of attack, normal force, and the expected moment sign | Disturbance-model tests | Tests in `test_week2_disturbances.py` |
| FAIL-01 | The prescribed open-loop disturbance causes loss of ascent attitude | Maximum tilt exceeds `90 deg` and thrust-axis alignment crosses zero | `figures/control-system-evidence.svg` | `test_disturbed_uncontrolled_vehicle_tilts_open_loop` |
| CTRL-01 | Ideal-torque feedback stabilizes the nonlinear rotational plant | Controlled tilt and lateral drift are below open-loop values | Control-system evidence figure | `test_controlled_case_reduces_open_loop_failure` |
| TVC-01 | TVC allocation produces the requested restoring-moment sign | `r_engine x F_thrust` opposes attitude error and respects the gimbal envelope | TVC unit-test output | Tests in `test_week3b_tvc.py` |
| LQR-01 | LQR improves the nominal TVC trajectory relative to PD | LQR maximum lateral drift is no greater than PD in the reference case | Control-system evidence figure | `test_lqr_tvc_improves_pd_tvc_lateral_drift_in_reference_case` |
| ROB-01 | Closed-loop control meets all dispersion gates | Tilt `< 25 deg`, final altitude `> 20 m`, drift `< 25 m`, saturation `< 10%` | `figures/monte-carlo-control-envelope.svg` and trial CSV | Fixed-seed sampling and small-campaign regression tests |
| NAV-01 | Estimated-state feedback remains control-feasible | Attitude error remains sub-degree in the nominal case and saturation is zero | `figures/estimated-state-control-evidence.svg` | Tests in `test_week5_estimation.py` |
| ACT-01 | Finite-bandwidth TVC remains within physical limits | Position and rate limiting are zero in the nominal case; torque margin remains positive | `figures/actuator-bandwidth-evidence.svg` | Tests in `test_week6_actuators.py` |
| PROP-01 | Propellant depletion changes mass properties consistently | Mass, transverse inertia, and CM move monotonically toward dry values | `figures/variable-mass-evidence.svg` | Tests in `test_week7_variable_mass.py` |
| SW-01 | The complete analysis is reproducible | One command regenerates data, reports, visuals, animation, and tests | `python3 scripts/run_all.py` | GitHub Actions test workflow |

## Robustness Gates

A Monte Carlo trial passes only when every gate is satisfied:

```text
max tilt < 25 deg
final altitude > 20 m
max lateral drift < 25 m
gimbal saturation fraction < 10%
```

The gates answer whether a simulated trajectory remains inside a deliberately stated comparison envelope. They do not represent structural load, range-safety, human-rating, or mission certification requirements.

## Evidence Hierarchy

1. Unit tests verify signs, invariants, transformations, interpolation, allocation, and limiter behavior.
2. Nominal integration cases verify coupled behavior in the nonlinear plant.
3. Estimated-state and actuator-limited cases remove truth-state and instantaneous-actuator assumptions.
4. Matched Monte Carlo trials test robustness to the declared uncertainty set.
5. CSV histories retain the numerical evidence behind every recruiter-facing figure.

This hierarchy prevents a polished plot from becoming the sole basis of a claim. Each visual result is backed by data and a lower-level check of the mechanism that produces it.
