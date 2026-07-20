# Week 3A Milestone Report

## Objective

Validate nonlinear thrust-axis attitude feedback using an ideal bounded body-torque actuator.

## Controlled Case

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 31.96 m |
| Final vertical velocity | 21.27 m/s |
| Maximum tilt angle | 9.88 deg |
| Minimum body-axis vertical component | 0.985 |
| Maximum angular rate | 0.21 rad/s |
| Maximum lateral displacement | 5.65 m |
| Maximum quaternion norm error | 2.220e-16 |

## Upper-Division Control Physics

The Week 3A controller regulates the thrust-axis direction rather than an Euler angle:

```text
z_body,I = R_BI(q)[0, 0, 1]
e_I = z_body,I x z_cmd,I
e_B = R_IB(q)e_I
tau_cmd,B = Kp e_B - Kd omega_B
```

For small attitude error, `e_B` is approximately the transverse rotation vector needed to align the thrust axis with inertial up. The proportional term acts like rotational stiffness, while the derivative term dissipates angular kinetic energy in the pitch/yaw modes. This directly attacks the Week 2 failure chain:

```text
disturbance moment -> angular rate -> attitude error -> thrust-axis loss
```

The ideal actuator allows the feedback law to be validated without the confounding effects of gimbal geometry. Because the commanded torque does not redirect the thrust force, attitude correction and translational acceleration are artificially decoupled. That is why Week 3A is a control-law verification step rather than an actuator-realistic result.

### Plot-Level Interpretation

Maintaining `body_z_z` near `+1` preserves `T cos(theta)` and suppresses `T sin(theta)`. The altitude trace remains increasing because vertical thrust projection is retained. Lateral drift is reduced because the controller prevents large horizontal thrust components and limits aerodynamic angle-of-attack growth.
