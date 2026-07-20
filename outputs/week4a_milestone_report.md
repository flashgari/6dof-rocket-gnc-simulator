# Week 4A Milestone Report

## Objective

Design a small-angle LQR attitude law and verify it in the nonlinear 6-DOF plant through the same TVC actuator model.

## LQR TVC Controlled Case

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 31.43 m |
| Final vertical velocity | 20.91 m/s |
| Maximum tilt angle | 10.30 deg |
| Minimum body-axis vertical component | 0.984 |
| Maximum angular rate | 0.55 rad/s |
| Maximum lateral displacement | 10.73 m |
| Maximum quaternion norm error | 2.220e-16 |
| Gimbal saturation fraction | 0.0% |

## Upper-Division LQR Physics

Near upright ascent, each transverse attitude channel can be locally approximated as:

```text
theta_dot = omega
omega_dot = tau / I
x = [theta, omega]^T
u = tau
```

LQR selects feedback gains by minimizing:

```text
J = integral(x^T Q x + u^T R u) dt
```

The attitude weight penalizes thrust-axis pointing error, the rate weight penalizes angular kinetic energy, and `R` penalizes torque demand. Since the torque request is allocated through TVC, `R` also indirectly shapes gimbal usage and lateral thrust demand.

This controller is local. It is designed about the upright operating point and is not a proof of global recovery from arbitrary tumble. The important verification step is that the LQR law is inserted back into the nonlinear plant with quaternion attitude, nonlinear thrust projection, aerodynamic CP/CM moments, thrust bias, finite gimbal authority, and saturation tracking.

## Comparison Physics

Open loop leaves `tau_dist` unopposed. Ideal torque performs best because moment and force direction are decoupled. PD TVC and LQR TVC are actuator-realistic because stabilizing moment requires lateral thrust. In the reference case, LQR TVC reduces tilt and lateral drift relative to PD TVC because its `Q/R` weighting damps the transverse rotational modes more efficiently while staying inside the same gimbal limit.

## Week 4A Comparison Plot Results

### Altitude

The altitude panel shows how controller quality maps into vertical impulse. Open loop loses vertical performance because `T_vertical = T cos(theta)` collapses as attitude error grows. Ideal torque ends highest because the model lets it apply moment without redirecting thrust. PD TVC and LQR TVC both pay a real actuator penalty: the lateral thrust used for moment slightly reduces axial thrust projection. LQR TVC ending closer to ideal torque means its lower attitude excursions reduce the time-integrated loss in `T cos(theta)`.

### Unwrapped Pitch

The unwrapped pitch result separates true rotational response from folded attitude display. Open loop is the uncontrolled integral of disturbance torque through `I omega_dot + omega x I omega = tau_dist`. The LQR trace remains bounded because the controller increases effective transverse stiffness and damping around the upright operating point. Its smaller excursion relative to PD TVC indicates that the selected `Q/R` weights damp the pitch mode with less attitude error accumulation.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` is the propulsion-relevant attitude result. Open loop approaching zero or negative values means thrust is no longer aligned with ascent. LQR TVC keeping the minimum `body_z_z` near unity indicates that the nonlinear trajectory remains inside the small-angle region assumed by the linearized design. This is the main validity check for using LQR in the nonlinear simulation.

### Lateral Drift

The lateral drift result is the integrated cost of attitude error:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

Ideal torque has the lowest drift because it does not require lateral thrust to generate moment. LQR TVC reducing drift relative to PD TVC means the LQR controller reduced both sideways thrust projection from attitude error and the aerodynamic side-force exposure associated with larger angle of attack.

### Gimbal Usage

The gimbal panel tests whether the LQR improvement is physically achievable. Since `tau_max,TVC ~= L T sin(delta_max)`, a controller that improves tracking only by saturating the gimbal would not be robust. In the Week 4A result, gimbal saturation remains zero, so the LQR improvement is attributable to feedback structure within the available actuator envelope rather than unrealistic command clipping.
