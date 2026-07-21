# Week 5 Milestone Report

## Objective

Add a sensor and attitude-estimation layer, then close the TVC control loop using estimated attitude and angular rate instead of truth-state feedback.

## Estimated-State Controlled Case

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 31.42 m |
| Final vertical velocity | 20.90 m/s |
| Maximum tilt angle | 10.43 deg |
| Minimum body-axis vertical component | 0.983 |
| Maximum angular rate | 0.55 rad/s |
| Maximum lateral displacement | 10.83 m |
| Maximum quaternion norm error | 2.220e-16 |
| Maximum attitude estimation error | 0.32 deg |
| RMS attitude estimation error | 0.17 deg |
| Maximum angular-rate estimation error | 0.007 rad/s |
| Gimbal saturation fraction | 0.0% |

## Upper-Division Avionics And Estimation Physics

Week 5 changes the closed-loop architecture from truth-state control to estimated-state control. This is a major GNC step because real flight software does not receive the true quaternion and true body angular velocity from the plant. It receives measurements corrupted by sensor bias, white noise, sampling, and reference-update limitations.

The gyro model is:

```text
omega_meas = omega_true + b_g + eta_g
```

The accelerometer model is specific force:

```text
f_B = R_IB(q)(a_I - g_I)
f_meas = f_B + b_a + eta_a
```

The specific-force definition is important. During powered ascent, an accelerometer does not measure gravity direction by itself. It measures non-gravitational acceleration, which is dominated by thrust and aerodynamic force. A naive "accelerometer points down" attitude correction would be physically invalid during high-thrust ascent because the sensed acceleration vector is largely aligned with thrust, not with gravity. The project therefore logs accelerometer channels as avionics measurements but uses gyro propagation plus a low-rate noisy attitude reference for attitude correction.

## Estimator Physics

The estimator propagates quaternion attitude from bias-corrected gyro rate:

```text
omega_hat = omega_meas - b_hat
q_hat_dot = 0.5 q_hat [0, omega_hat]
```

Gyro integration is high-bandwidth but drifts when bias is imperfect. The low-rate attitude reference bounds that drift by applying a small correction based on thrust-axis pointing error:

```text
e_I = z_hat,I x z_ref,I
e_B = R_IB(q_hat)e_I
```

This correction is intentionally applied to the thrust-axis direction because pitch/yaw alignment is the propulsion-relevant attitude quantity for vertical ascent. Roll about the thrust axis is less important for this simplified axisymmetric vehicle model.

## Estimated-State Control Physics

The LQR TVC controller is driven by:

```text
q_control = q_hat
omega_control = omega_hat
```

while the plant still evolves with the true nonlinear 6-DOF state. Estimation error therefore enters the feedback loop as a false attitude/rate command. If the estimate lags or drifts, the TVC controller can command the wrong moment, inject extra lateral thrust, or use up gimbal authority.

The Week 5 result remains close to the truth-state LQR case: final altitude is `31.42 m`, maximum tilt is `10.43 deg`, maximum lateral drift is `10.83 m`, maximum attitude estimation error is `0.32 deg`, RMS attitude estimation error is `0.17 deg`, and gimbal saturation is `0.0%`.

## Plot-Level Interpretation

### True vs Estimated Tilt

The true and estimated tilt traces stay nearly coincident. This means the estimator tracks the transverse thrust-axis attitude well enough for the controller to remain inside the same small-angle operating region assumed by the LQR design.

### Attitude Estimation Error

The attitude error remains sub-degree. This is small relative to the roughly `10 deg` controlled tilt envelope, so estimator error is not the dominant driver of the closed-loop response. The plot demonstrates that gyro propagation plus reference correction prevents bias-driven attitude drift over the simulated ascent window.

### Gyro Measurement

The gyro channels show the measured angular-rate signal that the controller indirectly depends on. Rate measurement matters because derivative damping and LQR rate feedback are both sensitive to angular-rate error. Bias-corrected gyro propagation keeps the estimator from interpreting a constant sensor bias as real vehicle rotation.

### Accelerometer Specific Force

The accelerometer channels are dominated by powered-flight specific force. This supports the modeling decision not to use accelerometer-only gravity leveling during ascent. In a rocket under thrust, accelerometer magnitude and direction reflect thrust and aerodynamic loading, not just vehicle attitude relative to gravity.

### Estimated-State TVC Usage

The gimbal trace verifies actuator feasibility under estimated-state feedback. If sensor noise caused aggressive false corrections, the gimbal angle or saturation fraction would increase. The result stays within the same actuator envelope as truth-state LQR, showing that the estimator does not destabilize or overdrive the TVC loop in the nominal disturbed case.

## Engineering Takeaway

Week 5 demonstrates the transition from controls-only simulation to avionics-aware GNC simulation. The controller is no longer granted perfect attitude knowledge; it must operate through a physically motivated measurement and estimation layer. The resulting closed-loop performance shows that the estimator error is small enough to preserve TVC stability and ascent performance for the modeled sensor noise/bias case.
