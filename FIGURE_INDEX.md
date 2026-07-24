# Engineering Evidence Guide

This is the shortest technical review path through the project. Each figure is generated directly from committed simulator CSV histories and is organized around an engineering question, not a development milestone.

## 1. Does Attitude Instability Actually Cause Trajectory Failure?

![Open-loop failure and controlled-ascent comparison](figures/control-system-evidence.svg)

**Artifact:** [`figures/control-system-evidence.svg`](figures/control-system-evidence.svg)

**What is plotted**

- altitude for the matched open-loop, ideal-torque, PD TVC, and LQR TVC cases
- `body_z_z`, the inertial-up component of the body thrust axis
- integrated lateral displacement
- controlled-case tilt from inertial vertical

**Physical interpretation**

The crucial state is not pitch by itself but thrust-axis alignment. With the nominal body thrust direction along `+z_B`,

`body_z_z = e_z,I^T R_BI(q) e_z,B = cos(theta)`.

At `body_z_z = 1`, the engine is aligned with inertial up. At zero, thrust is horizontal and produces no vertical support. Negative values indicate that a component of thrust points downward. The open-loop trace crosses both zero and `-1`, so its late return toward a positive value is a continued tumble, not recovery.

This rotation enters translation through

`m r_ddot_I = R_BI(q) F_B + m g_I`.

The open-loop vehicle therefore loses altitude performance while accumulating lateral velocity. Drift is an integral consequence: even a temporary horizontal thrust component can leave persistent crossrange velocity after the attitude has changed again.

Ideal body torque separates control-law behavior from force-vector geometry. It stabilizes rotation without changing the net thrust direction, so it has the smallest drift. TVC creates moment through `r_engine x F_thrust`; the same canted force that corrects attitude also adds lateral impulse. The PD and LQR results should therefore be compared against both open loop and ideal torque.

**Numerical result**

Open loop reaches `177.63 deg` maximum tilt, only `3.99 m` final altitude, and `25.10 m` lateral drift. LQR TVC limits maximum tilt to `10.30 deg`, reaches `31.43 m`, and limits drift to `10.73 m` without gimbal saturation. PD TVC is also stable, while LQR reduces nominal tilt and drift relative to PD.

**Claim boundary**

This is a short low-altitude nonlinear demonstration. It verifies the modeled failure mechanism and control architecture; it is not a max-Q, flexible-body, or global tumble-recovery result.

## 2. Does the Controller Retain Margin Under Dispersion?

![Monte Carlo robustness and margin](figures/monte-carlo-control-envelope.svg)

**Artifact:** [`figures/monte-carlo-control-envelope.svg`](figures/monte-carlo-control-envelope.svg)

**What is plotted**

- simultaneous pass rate against all four engineering gates
- every controlled trial's peak tilt versus thrust misalignment
- every controlled trial's peak drift versus thrust misalignment
- explicit `25 deg` and `25 m` boundaries
- sampled worst-case margin

**Physical interpretation**

The three architectures receive matched trials drawn from the same fixed-seed dispersion set. This pairing reduces comparison noise because controller differences are evaluated against identical mass, inertia, wind, thrust, aerodynamic, and gimbal samples.

Thrust misalignment dominates the controlled response in this envelope. A persistent angular bias in the engine force creates two coupled effects:

1. A disturbance moment that increases compensating TVC demand.
2. A transverse force that integrates into lateral velocity and displacement.

The nearly linear scatter trend is consistent with the small-angle relation

`F_perp approximately T delta_mis`

and with the local closed-loop system remaining in a bounded upright region. Correlation is used here as sensitivity evidence within this generated sample; it does not prove that all omitted nonlinear interactions are negligible.

A `100%` pass bar alone would be weak evidence because it hides proximity to failure. The scatter panels retain every sample and expose the distance to each requirement. LQR's worst cases preserve `7.08 deg` of tilt margin and `5.90 m` of drift margin. PD passes the same trials but approaches the gates more closely, with `2.12 deg` and `1.44 m` of corresponding margin.

**Numerical result**

| Architecture | Pass rate | Worst tilt | Worst drift |
| --- | ---: | ---: | ---: |
| Open loop | 1% | 179.82 deg | 27.74 m |
| PD TVC | 100% | 22.88 deg | 23.56 m |
| LQR TVC | 100% | 17.92 deg | 19.10 m |

**Claim boundary**

The result is conditional on `100` samples per controller, seed `4242`, the stated uniform distributions, and the project-level gates. It is a reproducible robustness regression, not a validated probability of mission success.

## 3. Can the Controller Operate on Estimated Attitude?

![Estimated-state attitude-control evidence](figures/estimated-state-control-evidence.svg)

**Artifact:** [`figures/estimated-state-control-evidence.svg`](figures/estimated-state-control-evidence.svg)

**What is plotted**

- true and estimated thrust-axis tilt
- quaternion attitude-error magnitude
- angular-rate estimation error
- estimated-state TVC command

**Estimator interpretation**

Gyro measurements provide high-rate propagation:

`omega_meas = omega + b_g + eta_g`

`q_hat_dot = 0.5 q_hat tensor [0, omega_meas - b_hat]`.

Because gyro bias integrates into attitude drift, a lower-rate noisy attitude reference bounds the error. The correction is performed on the quaternion manifold, avoiding subtraction of wrapped Euler angles.

The accelerometer is modeled as specific force,

`f_B = R_IB(q) (a_I - g_I) + b_a + eta_a`.

During powered ascent, thrust dominates this measurement. Treating the accelerometer as a pure gravity direction would therefore inject a physically incorrect reference into the attitude estimate. The implemented estimator does not make that stationary-platform assumption.

**Control interpretation**

Estimation error enters the feedback loop as false attitude and rate error. If its spectrum or magnitude were large, the controller would spend gimbal authority reacting to measurement error and could excite the finite-bandwidth actuator. Instead, the maximum quaternion attitude error is `0.32 deg`, RMS error is `0.17 deg`, and maximum true tilt is `10.43 deg`. Error remains small relative to the controlled excursion, and gimbal saturation remains zero.

**Claim boundary**

This is a focused quaternion attitude filter. It does not estimate inertial position, velocity, IMU alignment, or a full navigation covariance, and it is not represented as a flight navigation EKF.

## 4. Does Finite Gimbal Bandwidth Destabilize the Loop?

![Finite-bandwidth TVC evidence](figures/actuator-bandwidth-evidence.svg)

**Artifact:** [`figures/actuator-bandwidth-evidence.svg`](figures/actuator-bandwidth-evidence.svg)

**What is plotted**

- instantaneous-TVC, actuator-limited, and estimated-state actuator-limited tilt
- commanded and achieved gimbal magnitude
- servo tracking error
- available TVC moment minus requested moment

**Physical interpretation**

An instantaneous allocator hides actuator poles and nonlinear limits. The actuator model instead propagates

`delta_dot = sat_rate((delta_cmd - delta_act) / tau_servo)`

and then clips `delta_act` to the position envelope. The nonlinear plant receives force and moment computed from `delta_act`, never from the unattained command.

The lag matters because rotational damping requires torque with the correct phase relative to angular rate. A servo pole contributes phase delay; near the control-loop crossover, enough delay can erode phase margin and turn nominally dissipative feedback into energy injection. The time-domain comparison is a first check that the added pole has not created a large overshoot or divergent oscillation.

Moment feasibility is checked separately:

`margin_tau = L T sin(delta_max) - ||tau_cmd,transverse||`.

The minimum margin is `48.58 N m`, and neither slew nor position limiting occurs in the nominal run. The peak command is `1.50 deg`, while the peak achieved deflection is `0.86 deg`; the visible lag is real, but the attitude response remains bounded and close to the instantaneous-TVC result.

**Claim boundary**

Positive time-domain margin does not replace frequency-response identification. A stronger hardware-correlated result would identify servo bandwidth and uncertainty, include transport delay and compliance, and report gain/phase or disk margins.

## 5. Does Propellant Depletion Change the Control Problem?

![Variable-mass propulsion and GNC evidence](figures/variable-mass-evidence.svg)

**Artifact:** [`figures/variable-mass-evidence.svg`](figures/variable-mass-evidence.svg)

**What is plotted**

- constant-mass and variable-mass altitude
- controlled tilt
- propellant depletion
- thrust-to-mass schedule
- transverse inertia
- center-of-mass migration

**Physical interpretation**

Mass flow is tied to thrust and specific impulse:

`m_dot = -T / (I_sp g_0)`.

The translational plant changes through `T(t)/m(t)`. The rotational plant changes simultaneously:

`I(t) omega_dot + omega x (I(t) omega) = tau`.

Lower transverse inertia increases angular acceleration for the same aerodynamic or TVC moment. Center-of-mass migration changes the aerodynamic lever arm `r_CP - r_CM(t)`, while thrust variation changes the available TVC moment `L T(t) sin(delta_max)`. Propellant depletion therefore modifies disturbance sensitivity, control effectiveness, and trajectory acceleration at the same time.

Mass decreases from `50.00 kg` to `48.93 kg`, transverse inertia from `3.15` to `3.07 kg m^2`, and CM moves from `-8.0 cm` to `-5.6 cm`. The fixed LQR remains stable with `11.21 deg` maximum tilt and `50.27 N m` minimum TVC moment margin. The higher altitude is caused by the combined thrust schedule and declining mass, not by mass loss alone.

**Claim boundary**

The short run demonstrates a moving plant, but its modest mass fraction does not establish full-ascent robustness. The next control step is trajectory linearization and gain scheduling against mass state, thrust, and dynamic pressure.

## 6. Synchronized Animation

![Animation preview with architecture color key](figures/rocket-animation-preview.svg)

**Interactive artifact:** [open the browser-rendered animation](https://htmlpreview.github.io/?https://github.com/flashgari/6dof-rocket-gnc-simulator/blob/main/outputs/rocket_flight_animation.html)

The animation is useful because the attitude and translation histories are synchronized. The open-loop vehicle can momentarily look upright after a full rotation, but the unwrapped history and thrust-axis-alignment trace reveal that it has passed through horizontal and inverted flight. Its accumulated lateral velocity and altitude loss cannot be undone by simply pointing upward again.

Color key:

| Color | Architecture | Engineering purpose |
| --- | --- | --- |
| Red | Open loop | Establish the disturbed-plant failure mechanism |
| Blue | Ideal body torque | Isolate feedback-law performance |
| Green | PD TVC | Realize restoring moment through gimbaled thrust |
| Cyan | LQR TVC | Compare optimized local feedback in the same actuator geometry |

## Supporting Material

- [Full technical report](PORTFOLIO_WRITEUP.md)
- [Requirement-to-evidence traceability](VERIFICATION_MATRIX.md)
- [Detailed historical plot interpretations](docs/figure_results_interpretations.md)
- [Raw Monte Carlo trial data](outputs/week4b_monte_carlo_results.csv)
- [Model assumptions and derivations](docs/technical_physics_notes.md)
