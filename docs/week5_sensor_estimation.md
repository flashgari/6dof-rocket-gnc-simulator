# Week 5: Sensor Simulation And Estimated-State Feedback

Week 5 adds an avionics layer between the truth-state simulation and the TVC controller. Earlier weeks close the loop using true attitude and true angular rate. That is useful for control-law development, but real flight software acts on estimated state derived from imperfect sensors.

## Sensor Model

The simulated IMU produces gyro and accelerometer measurements:

```text
gyro_meas = omega_true + bias_g + noise_g
accel_meas = f_B + bias_a + noise_a
```

where accelerometer output is modeled as body-frame specific force:

```text
f_B = R_IB(q)(a_I - g_I)
```

This distinction matters. During powered ascent, an accelerometer is not a clean gravity-vector sensor because it is dominated by thrust and aerodynamic specific force. Treating it as a simple "which way is down" measurement would be physically wrong for this phase of flight.

The estimator therefore uses gyro propagation with a noisy low-rate attitude reference. That reference represents a generic external attitude update, such as a star-tracker-like, vision, or navigation-aided attitude correction. The purpose is not to claim a specific flight sensor suite; it is to model the avionics problem that the controller receives estimated attitude rather than truth attitude.

## Quaternion Attitude Estimator

The estimator propagates attitude using gyro measurements corrected by a known preflight bias estimate:

```text
omega_hat = gyro_meas - bias_hat
q_hat_dot = 0.5 q_hat [0, omega_hat]
```

When a reference attitude measurement is available, the filter computes thrust-axis error between the propagated attitude and the reference attitude:

```text
e_I = z_hat,I x z_ref,I
e_B = R_BI(q_hat)^T e_I
```

and applies a small quaternion correction. This keeps the estimator focused on the attitude components that matter most for ascent guidance: pitch/yaw thrust-axis alignment.

## Closed-Loop Estimated-State Control

The LQR TVC controller is then driven by the estimated state:

```text
state_for_control = [r_true, v_true, q_hat, omega_hat]
```

The translational state is left as truth-state in this week so the scope stays focused on attitude estimation. A later extension can add GPS/barometer and a translational Kalman filter.

## Results

The Week 5 nominal disturbed ascent stays close to truth-state LQR:

| Metric | Value |
| --- | ---: |
| Final altitude | 31.42 m |
| Maximum tilt | 10.43 deg |
| Maximum lateral drift | 10.83 m |
| Maximum attitude estimation error | 0.32 deg |
| RMS attitude estimation error | 0.17 deg |
| Gimbal saturation | 0.0% |

The important result is not that the estimate is mathematically perfect. The important result is that bounded sensor noise and bias produce small enough attitude/rate errors that the LQR TVC loop remains stable, actuator-feasible, and close to the truth-state control baseline.

## Physical Interpretation

The estimator closes the gap between controls and avionics. In the truth-feedback case, the controller sees the exact quaternion and angular velocity used by the plant. In the estimated-state case, the controller sees a propagated and corrected estimate. Any estimator phase lag, gyro bias residual, or reference noise becomes a control input error.

Because TVC moment is limited by

```text
tau_max,TVC ~= L T sin(delta_max)
```

estimation error cannot simply be overcome by arbitrary gain. If estimated attitude error caused over-commanding, the gimbal trace would show saturation or excess lateral thrust. The Week 5 result keeps saturation at `0.0%`, which means the estimator/controller combination remains inside the modeled actuator envelope.

## Next Extensions

- Add gyro-bias estimation instead of assuming preflight bias calibration.
- Add barometer/GPS measurements and a translational Kalman filter.
- Add delayed measurements and estimator latency.
- Couple the estimator to higher-fidelity actuator hardware data and guidance commands beyond vertical ascent.
