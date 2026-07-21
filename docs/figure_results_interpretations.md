# Figure Results Interpretations

This file is the project-wide guide for interpreting every generated graph at an aerospace upper-division level. Each plot is tied to the governing equations, the numerical result, and the GNC relevance.

## Week 1 Baseline Ascent Plot

Artifact:

```text
outputs/week1_ascent_plots.svg
```

### Altitude

The final altitude is `359.67 m` after `10 s`. This matches the closed-form constant-acceleration result:

```text
a_z = T/m - g = 7.19335 m/s^2
z(t) = 0.5 a_z t^2
```

The parabolic altitude curve confirms that the translational force balance, gravity sign convention, and RK4 integration are consistent for the zero-drag, vertical-thrust limiting case.

### Vertical Velocity

The final vertical velocity is `71.93 m/s`, matching:

```text
v_z(t) = a_z t
```

This verifies that the simulator is not introducing spurious drag, lateral force, or attitude-induced thrust loss in the baseline configuration.

### Quaternion Norm

The quaternion norm remains at numerical unity. This matters because the quaternion is the rotation operator used to map body-frame forces into inertial coordinates. If the norm drifted, later aerodynamic and TVC force projections would be physically invalid.

## Week 2 Disturbed Open-Loop Plot

Artifact:

```text
outputs/week2_disturbed_uncontrolled_plots.svg
```

### Altitude

The vehicle reaches a low-altitude failed state with final altitude `3.99 m` and downward vertical velocity `-10.33 m/s`. The result is caused by attitude divergence reducing vertical thrust projection:

```text
T_vertical = T cos(theta)
```

As the vehicle tumbles, thrust is no longer primarily upward. The altitude peak and descent are therefore trajectory consequences of rotational instability.

### Unwrapped Pitch

The maximum tilt reaches `177.63 deg`, and the unwrapped pitch shows continuous rotation through inverted flight. This is the correct attitude-failure diagnostic because folded tilt can appear to recover after `180 deg`.

The rotational cause is:

```text
I omega_dot + omega x I omega = tau_dist
tau_dist = r_T x F_T + (r_CP - r_CM) x F_N
```

with no feedback torque opposing the disturbance.

### Body-Axis Vertical Component

The minimum `body_z_z` is `-0.999`, meaning the thrust axis becomes almost exactly inverted. Since `body_z_z = cos(theta)`, this indicates near-complete loss of vertical thrust authority.

### Lateral Drift

The maximum lateral drift is `25.10 m`. This is not only a wind-displacement result. It is the integrated result of horizontal thrust projection and aerodynamic side force:

```text
m a_lateral ~= T sin(theta) + F_N,lateral - D_lateral
```

The result demonstrates why attitude control is necessary for trajectory control.

## Week 3A Ideal-Torque Controlled Plot

Artifact:

```text
outputs/week3a_controlled_ideal_torque_plots.svg
```

### Altitude

The final altitude is `31.96 m`, much higher than the Week 2 open-loop final altitude over the same disturbed `3 s` window. The controller preserves vertical thrust projection by keeping the thrust axis near inertial up.

### Unwrapped Pitch And Body-Axis Alignment

Maximum tilt is limited to `9.88 deg`, and minimum `body_z_z` stays at `0.985`. This indicates that the controller keeps the vehicle inside a small-attitude envelope where:

```text
T_vertical = T cos(theta)
```

remains close to full thrust.

### Lateral Drift

Maximum lateral drift falls to `5.65 m`. Since ideal torque does not redirect the thrust force, this case separates feedback-law effectiveness from actuator-induced lateral acceleration.

### Control Torque

The peak ideal control torque is about `3.63 N m`. This torque arrests disturbance-driven angular-rate growth through:

```text
tau_cmd = Kp e - Kd omega
```

The plot verifies that bounded feedback torque is sufficient to stabilize the nominal disturbed case before adding actuator geometry.

## Week 3B TVC Controlled Plot

Artifact:

```text
outputs/week3b_tvc_controlled_plots.svg
```

### Altitude

The final altitude is `30.97 m`, slightly below ideal torque. That difference is physically meaningful: TVC generates moment by tilting thrust, so some thrust is spent laterally instead of purely vertically.

### Attitude Metrics

Maximum tilt is `12.94 deg`, with minimum `body_z_z = 0.975`. The vehicle remains within a controlled ascent envelope, but the attitude excursion is larger than ideal torque because moment generation is constrained by:

```text
tau_TVC = r_engine x F_thrust
```

### Lateral Drift

Maximum lateral drift is `13.24 m`, larger than ideal torque but much smaller than open loop. This is the TVC coupling penalty: lateral thrust produces stabilizing moment and appears in the translational equation.

### Gimbal Angle

Peak gimbal angle is about `1.50 deg`, and saturation is `0.0%`. The controller stabilizes the case well inside the `5 deg` gimbal limit, so the result is actuator-feasible.

## Week 3B Control Comparison Plot

Artifact:

```text
outputs/week3b_control_comparison_plots.svg
```

### What The Comparison Means

The open-loop curve establishes the failure mode: attitude divergence rotates thrust away from inertial up, altitude collapses, and lateral drift grows. The ideal-torque curve establishes the best-case feedback response without actuator coupling. The TVC curve demonstrates that the same feedback objective remains feasible with finite engine-gimbal authority.

The important engineering conclusion is that actuator realism changes the trajectory. TVC improves stability dramatically relative to open loop, but it cannot match ideal torque exactly because attitude correction and lateral acceleration are coupled through the thrust vector.

## Week 4A LQR TVC Plot

Artifact:

```text
outputs/week4a_lqr_tvc_controlled_plots.svg
```

### Altitude

The LQR TVC final altitude is `31.43 m`, higher than PD TVC and close to ideal torque. This means the LQR controller reduced attitude error enough to preserve more vertical thrust projection over the burn.

### Attitude Metrics

Maximum tilt is `10.30 deg`, and minimum `body_z_z = 0.984`. This verifies that the nonlinear trajectory stays near the upright operating point assumed by the LQR linearization:

```text
theta_dot = omega
omega_dot = tau/I
```

### Lateral Drift

Maximum drift is `10.73 m`, lower than PD TVC. Since:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

the lower drift indicates reduced integrated attitude error and reduced aerodynamic side-force exposure.

### Gimbal Angle

Peak gimbal angle remains about `1.50 deg`, with `0.0%` saturation. The LQR improvement is therefore not caused by exceeding actuator authority; it occurs inside the same TVC envelope.

## Week 4A LQR Control Comparison Plot

Artifact:

```text
outputs/week4a_lqr_control_comparison_plots.svg
```

### What The Comparison Means

This plot compares four levels of GNC realism:

- open loop: no disturbance rejection
- ideal torque: feedback without actuator coupling
- PD TVC: nonlinear feedback through finite gimbal authority
- LQR TVC: local optimal feedback through finite gimbal authority

The result shows that LQR TVC improves median attitude and drift relative to PD TVC in the nominal disturbed case while remaining actuator-feasible. The upper-division interpretation is that LQR changes the closed-loop transverse-mode damping and stiffness through the `Q/R` cost, but the TVC allocator still imposes the same physical moment limit.

## Week 4B Monte Carlo Summary Plot

Artifact:

```text
outputs/week4b_monte_carlo_summary.svg
```

### Success Rate

Open loop succeeds in only `1.0%` of the sampled dispersions. PD TVC and LQR TVC both succeed in `100.0%` of the sampled dispersions. This is a robustness result, not just a nominal result. It means feedback keeps the vehicle inside the selected tilt, altitude, lateral drift, and saturation gates despite randomized wind, thrust alignment, mass properties, aerodynamic slope, CP location, and gimbal authority.

### Median Maximum Tilt

Open-loop median maximum tilt is `177.0 deg`, which means the representative uncontrolled trial tumbles to nearly inverted flight. PD TVC reduces the median to `12.0 deg`; LQR TVC reduces it to `9.7 deg`.

This metric matters because it measures peak departure from the ascent attitude corridor. Final attitude alone can be misleading after a tumble. Maximum tilt captures whether the controller prevented loss of thrust-axis authority during the burn.

### Median Maximum Lateral Drift

Open-loop median drift is `24.3 m`; PD TVC reduces it to `12.7 m`; LQR TVC reduces it to `10.3 m`.

This metric matters because lateral drift integrates the translational consequence of attitude error. A controller can have acceptable final attitude but still accumulate crossrange velocity during earlier excursions. LQR TVC reducing drift relative to PD TVC indicates lower integrated horizontal thrust projection and aerodynamic side-force exposure over the sampled envelope.

## Animation

Artifact:

```text
outputs/rocket_flight_animation.html
```

Rendered browser preview:

```text
https://htmlpreview.github.io/?https://github.com/flashgari/6dof-rocket-gnc-simulator/blob/main/outputs/rocket_flight_animation.html
```

The animation is the time-domain version of the same conclusions. Open loop may visually point upward again after tumbling, but the unwrapped pitch and trajectory show that it already lost vertical thrust authority. Ideal torque isolates the feedback law. PD TVC demonstrates actuator-realistic stabilization. LQR TVC shows that a local optimal controller can reduce attitude and drift while staying inside the same gimbal authority.

## Week 5 Estimated-State TVC Plot

Artifact:

```text
outputs/week5_estimated_tvc_plots.svg
```

### True vs Estimated Tilt

The true and estimated tilt traces stay nearly coincident, with maximum attitude estimation error `0.32 deg` and RMS attitude estimation error `0.17 deg`. This matters because TVC control is driven by thrust-axis pointing. If the estimator allowed large pitch/yaw error, the controller would command moments against the wrong attitude state and could inject lateral thrust unnecessarily.

### Attitude Estimation Error

The error remains sub-degree across the disturbed ascent. The estimator is not simply plotting truth; it propagates quaternion attitude from biased/noisy gyro measurements and corrects with a noisy low-rate attitude reference. The bounded error shows that gyro bias correction plus periodic reference updates prevents drift from becoming dynamically relevant over the burn.

### Gyro Measurement

The gyro measurement panel shows what the rate feedback channel actually sees:

```text
omega_meas = omega_true + b_g + eta_g
```

Rate feedback is central to both damping and LQR state feedback. A residual gyro bias would look like false angular rate and could cause steady TVC commands. The plotted response verifies that the estimator/controller can tolerate the modeled bias and noise without saturating the actuator.

### Accelerometer Specific Force

The accelerometer panel is interpreted as specific force:

```text
f_B = R_IB(q)(a_I - g_I)
```

During powered ascent this channel is dominated by thrust and aerodynamic acceleration, not gravity alone. That is why the estimator does not use accelerometer-only gravity leveling as the primary attitude correction. This is an important avionics distinction: launch-vehicle IMU acceleration is a force measurement, not a direct attitude measurement.

### Estimated-State TVC Usage

The gimbal trace remains inside the actuator envelope with `0.0%` saturation. This means estimation errors do not cause excessive false control demand. The result supports estimated-state feasibility: the controller remains close to the truth-state LQR response while using noisy sensor-derived attitude and angular rate.

## Week 5 Truth-State vs Estimated-State Control Plot

Artifact:

```text
outputs/week5_estimated_vs_truth_control_plots.svg
```

### What The Comparison Means

This plot compares the same LQR TVC controller using two feedback sources:

- truth-state feedback: exact simulation quaternion and angular velocity
- estimated-state feedback: noisy sensor measurements processed by the quaternion estimator

The altitude, tilt, body-axis vertical component, lateral drift, and gimbal traces remain close. That means the estimation layer introduces only small control-relevant error in the nominal disturbed case. The result is a bridge from controls to avionics: the controller is no longer relying on physically impossible perfect state knowledge.

### Physical Relevance

The comparison is important because estimator error enters the feedback loop as an apparent attitude/rate error. If the estimate lagged, drifted, or amplified noise, the TVC system would either under-correct the true disturbance or over-command gimbal motion. Instead, the estimated-state case reaches `31.42 m` final altitude, `10.43 deg` maximum tilt, `10.83 m` maximum lateral drift, and `0.0%` saturation, which is close to the truth-state LQR baseline. The estimator therefore preserves the control objective within the modeled sensor and actuator assumptions.

## Week 6 Actuator-Limited TVC Plot

Artifact:

```text
outputs/week6_actuator_limited_tvc_plots.svg
```

### Tilt Response

This panel compares instant LQR TVC, truth-state actuator-limited LQR, and estimated-state actuator-limited LQR. The actuator-limited cases remain close to the instant actuator baseline: the truth-state case reaches `11.09 deg` maximum tilt and the estimated-state case reaches `10.91 deg`, compared with `10.30 deg` for instant LQR TVC.

The physical interpretation is a bandwidth result. The actuator model inserts first-order lag between the requested nozzle angle and the achieved nozzle angle:

```text
delta_dot_cmd = (delta_cmd - delta_act) / tau_servo
```

Because the tilt response remains bounded, the gimbal bandwidth is still high enough relative to the dominant pitch/yaw rigid-body dynamics. If the actuator were too slow, the damping moment would arrive late, reducing phase margin and producing larger attitude overshoot.

### Lateral Drift

The lateral drift traces remain close to the instant-LQR case. Lateral displacement is an integrated response:

```text
v_lateral_dot ~= (T/m) sin(theta) + F_N,lateral/m
```

so even a brief attitude-control delay can leave a permanent crossrange velocity. The small difference between instant and actuator-limited LQR shows that the finite gimbal dynamics do not significantly increase the time-integrated horizontal impulse in this nominal case.

### Commanded vs Achieved Gimbal

The commanded gimbal is the controller/allocation request; the achieved gimbal is what the plant actually receives after actuator dynamics. Their separation is not a numerical artifact. It is the physical tracking error of the TVC servo. Since corrective moment is generated by:

```text
tau_TVC = r_engine x F(delta_act)
```

the achieved gimbal, not the commanded gimbal, determines angular acceleration.

### Gimbal Lag Error

The peak gimbal lag is about `1.50 deg`. This is the actuator-induced moment lag that an ideal TVC model hides. In control terms, it represents phase delay in the feedback path; in flight-mechanics terms, it means the engine moment is being applied to a slightly older attitude/rate state. The nominal case survives because the lag is small compared with both the `5 deg` gimbal envelope and the closed-loop attitude corridor.

### Torque Authority Margin

Torque authority margin is:

```text
tau_margin = L T sin(delta_max) - |tau_requested|
```

A positive margin means the controller is requesting a moment inside the feasible TVC envelope. This is an important verification quantity because high gains can make a simulated controller look good only by asking for impossible moments. In Week 6 the margin remains positive and the rate-limit fraction is `0.0%`, so the controller is not relying on unavailable actuator authority in the nominal actuator-limited case.

## Week 7 Variable-Mass Powered Ascent Plot

Artifact:

```text
outputs/week7_variable_mass_plots.svg
```

### Altitude

The variable-mass trajectory reaches `33.32 m`, above the constant-mass actuator-limited reference. The explanation is the integral of net vertical acceleration, not simply the fact that the rocket gets lighter. The thrust curve rises to `900 N` in the middle of the burn while mass is decreasing, so the vehicle experiences a stronger `T(t)/m(t)` segment before thrust tails down.

### Mass And Thrust-To-Mass

Mass decreases from `50.00 kg` to `48.93 kg` according to:

```text
m_dot = -T / (Isp g0)
```

The `T/m` trace starts at `15.80 m/s^2`, peaks at `18.20 m/s^2`, and ends at `15.53 m/s^2`. This panel is important because it shows that acceleration depends on both numerator and denominator. A falling thrust curve can overpower the acceleration benefit of lower mass near the end of burn.

### Inertia And CM Shift

The transverse inertia decreases from `3.15` to `3.07 kg m^2`, which changes the rotational plant:

```text
I(t) omega_dot + omega x (I(t) omega) = tau
```

The CM moves from `-0.080 m` to `-0.056 m`, changing the CP/CM aerodynamic moment arm. The plot displays this CM coordinate in centimeters so the shift is visible beside the inertia trend. This means the same angle of attack and dynamic pressure can produce a different rotational disturbance later in the burn.

### Tilt Response

The variable-mass case stays controlled with `11.21 deg` maximum tilt. The small difference relative to the constant-mass actuator-limited case is the expected closed-loop signature of changing inertia, CM, and thrust authority with fixed LQR gains. This is exactly why real ascent controllers are often scheduled over mass state, dynamic pressure, and thrust level.

### TVC Authority Margin

TVC authority margin remains positive, with minimum margin `50.27 N m`. Since:

```text
tau_max,TVC(t) ~= L T(t) sin(delta_max)
```

the thrust curve changes both acceleration and available control moment. Week 7 verifies that the controller remains inside the finite actuator authority envelope even while propulsion and mass properties vary.
