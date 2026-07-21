# Technical Physics Notes

These notes define the physical interpretation standard for the project. Every figure should connect the plotted metric to a governing equation, a force or moment source, an actuator limit, a control law, or a verification criterion. The goal is aerospace upper-division reasoning, not qualitative captions.

## Figure Interpretation Standard

Every figure explanation should identify:

- the state or derived metric being plotted
- the relevant force, moment, or feedback law
- the coupling between attitude and translation
- the modeling or actuator limitation that constrains the result
- the engineering conclusion supported by the plot

A strong explanation of lateral drift, for example, should connect:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

to thrust-axis pointing error, aerodynamic side force, CP/CM moment arm, and controller authority. A strong robustness explanation should connect success rate to dispersions, pass/fail gates, and actuator saturation margin.

The dedicated results guide for every generated graph is:

```text
docs/figure_results_interpretations.md
```

## Frames And State

The simulator uses an inertial frame with `+z` upward and a body frame with body `+z` along the thrust axis:

```text
x = [r_I, v_I, q_BI, omega_B]
```

The quaternion `q_BI` maps body-frame vectors into inertial coordinates. This matters because thrust and aerodynamic forces are naturally defined in body coordinates, while trajectory and gravity are inertial.

## Translational Dynamics

The translational equation is:

```text
m r_ddot_I = R_BI(q) F_B + [0, 0, -mg]
```

The central ascent coupling is thrust projection:

```text
T_vertical = T cos(theta)
T_lateral  = T sin(theta)
```

Attitude error therefore has immediate trajectory consequences. A controller that keeps the body `+z` axis near inertial up preserves altitude authority and reduces lateral acceleration.

## Rotational Dynamics

The rotational equation is:

```text
I omega_dot_B + omega_B x (I omega_B) = tau_B
```

The `omega x I omega` term represents rigid-body gyroscopic coupling in the rotating body frame. Applied moments enter through force offsets:

```text
tau = r x F
```

This same expression governs engine-offset torque, aerodynamic CP/CM moment, and TVC moment generation.

## Aerodynamic Force And Moment Model

The relative wind is transformed into body coordinates:

```text
v_rel,B = R_IB(q) (v_I - wind_I)
```

The transverse component gives the small-angle angle-of-attack estimate:

```text
alpha ~= |v_rel,perp| / |v_rel|
```

The normal-force scaling is:

```text
qbar = 0.5 rho |v_rel|^2
F_N ~ qbar S C_N_alpha alpha
```

The aerodynamic moment is:

```text
tau_aero = (r_CP - r_CM) x F_N
```

The sign of this moment depends on CP location relative to CM. Week 2 intentionally uses a destabilizing CP/CM configuration so the uncontrolled vehicle exhibits divergent rotational response about the ascent attitude.

## Week 1 Verification Physics

Week 1 uses aligned thrust and zero external moment. The expected vertical acceleration is:

```text
a_z = T/m - g
```

The altitude and velocity traces should match the constant-acceleration solution. The quaternion norm should remain at unity because a non-unit quaternion no longer represents a pure rotation. Conservation tests verify limiting cases of the same equations: energy in ballistic flight, linear momentum without force, and inertial angular momentum without torque.

## Week 2 Open-Loop Failure Physics

The open-loop failure chain is:

```text
wind/thrust/aero dispersions -> tau_dist -> omega -> attitude error -> thrust-axis loss
```

As `theta` grows, `T cos(theta)` decreases and `T sin(theta)` increases. The altitude peak is therefore the translational result of attitude divergence. The lateral drift is the integrated result of horizontal thrust projection and aerodynamic side force.

`body_z_z = cos(theta)` is the most physically direct attitude plot because it tells how much of the thrust axis is aligned with inertial up. Unwrapped pitch is used because folded tilt hides full rotations after inverted flight.

## Week 3A Ideal-Torque Control Physics

The ideal controller regulates the thrust-axis vector:

```text
e_I = z_body,I x z_cmd,I
tau_cmd,B = Kp e_B - Kd omega_B
```

The proportional term supplies rotational stiffness; the derivative term damps angular kinetic energy. This prevents disturbance moments from integrating into tumble. Since the actuator is ideal, attitude correction is decoupled from thrust direction, making Week 3A a control-law verification case rather than a final actuator model.

## Week 3B TVC Physics

TVC produces moment by redirecting thrust:

```text
tau_TVC = r_engine x F_thrust
```

For an engine below CM:

```text
tau_x = L F_y
tau_y = -L F_x
tau_max,TVC ~= L T sin(delta_max)
```

This introduces actuator limits and couples attitude correction to lateral acceleration. TVC can stabilize the vehicle, but unlike ideal torque, it must spend some thrust laterally to create moment.

## Week 4A LQR Physics

Near upright ascent, the transverse attitude channels can be locally modeled as:

```text
theta_dot = omega
omega_dot = tau / I
```

LQR minimizes:

```text
J = integral(x^T Q x + u^T R u) dt
```

The `Q` weights penalize attitude error and angular rate; `R` penalizes control effort. Because the LQR torque command is allocated through TVC, the control-effort penalty also affects gimbal demand. The design is local and must be verified in the nonlinear plant.

## Week 4B Monte Carlo Robustness Physics

Monte Carlo analysis tests sensitivity to uncertainty in winds, mass properties, thrust alignment, aerodynamics, CP location, and gimbal authority. The pass/fail gates are tied to physical outcomes:

- maximum tilt checks thrust-axis stability
- final altitude checks preserved vertical acceleration
- lateral drift checks integrated crossrange acceleration
- saturation checks actuator authority margin

The Week 4B summary plot is therefore a compact robustness statement: open loop fails because disturbance moments are unopposed, PD TVC succeeds by closing the rotational loop through gimbaled thrust, and LQR TVC improves median tilt/drift in this envelope through a more structured state-error/control-effort trade.

## Week 5 Sensor And Estimation Physics

Week 5 changes the feedback assumption. Earlier controllers use truth-state attitude and angular rate; Week 5 uses measured and estimated quantities. The gyro model is:

```text
omega_meas = omega_true + b_g + eta_g
```

so uncompensated bias would integrate into attitude drift. The quaternion estimator subtracts a bias estimate and propagates:

```text
q_hat_dot = 0.5 q_hat [0, omega_meas - b_hat]
```

The accelerometer model is body-frame specific force:

```text
f_B = R_IB(q)(a_I - g_I)
```

This is not the same as a gravity direction measurement during powered ascent. Since thrust and aerodynamic loads dominate `f_B`, an accelerometer-only attitude correction would tend to align with the thrust environment rather than with inertial down. That is why Week 5 uses a low-rate attitude reference for correction and treats accelerometer outputs as logged avionics measurements.

Estimated-state feedback matters because estimator error enters the TVC loop as a false attitude/rate error. If `q_hat` lags or drifts, the controller may command moment in the wrong direction or waste gimbal authority. The Week 5 results show sub-degree attitude estimation error and zero gimbal saturation, so the estimator is accurate enough for the LQR TVC controller to preserve thrust-axis alignment in the nominal disturbed ascent.

## Week 6 Actuator Dynamics Physics

Week 6 changes the actuator assumption. Earlier TVC cases treat gimbal commands as instantaneously achieved. Week 6 separates controller request from actuator response:

```text
delta_dot_cmd = (delta_cmd - delta_act) / tau_servo
|delta_dot_act| <= delta_dot_max
|delta_act| <= delta_max
tau_TVC = r_engine x F(delta_act)
```

The distinction matters because closed-loop attitude control depends on when the corrective moment arrives, not only on whether the correct moment was requested. A finite-bandwidth gimbal inserts phase lag into the loop. In a transverse rigid-body channel,

```text
theta_dot = omega
I omega_dot = tau_TVC + tau_dist
```

late torque reduces damping effectiveness. With enough lag, the controller can apply moment after the state has moved, reducing phase margin or even adding rotational energy instead of dissipating it. This is why actuator dynamics are part of GNC stability analysis rather than a cosmetic mechanical detail.

The Week 6 nominal result shows bounded actuator lag instead of control failure. Truth-state actuator-limited LQR reaches `11.09 deg` maximum tilt, and estimated-state actuator-limited LQR reaches `10.91 deg`. The maximum gimbal lag is about `1.50 deg`, with `0.0%` rate-limit fraction. That means the first-order lag is visible in the actuator telemetry but does not push the nominal case into slew-rate saturation.

Torque authority is checked through:

```text
tau_max,TVC ~= L T sin(delta_max)
tau_margin = tau_max,TVC - |tau_requested|
```

Positive torque margin means the controller remains inside the available moment envelope. This is the key aerospace-controls interpretation: Week 6 verifies not just that the LQR law stabilizes the math model, but that it remains feasible after the moment command is filtered through nozzle bandwidth, gimbal limits, and estimated-state feedback.

## Week 7 Variable-Mass Flight Physics

Week 7 makes the vehicle plant time-varying. Propellant depletion is computed from impulse and specific impulse:

```text
m_dot = -T / (Isp g0)
m(t) = m0 - integral(T / (Isp g0)) dt
```

The translational acceleration scale is:

```text
a_thrust = T(t) / m(t)
```

This couples the thrust curve and mass depletion. Lower mass tends to increase acceleration, but thrust tailoff can reduce it. In the Week 7 run, `T/m` starts at `15.80 m/s^2`, peaks at `18.20 m/s^2`, and ends at `15.53 m/s^2`.

The rotational plant also changes:

```text
I(t) omega_dot + omega x (I(t) omega) = tau
```

As transverse inertia decreases, the same moment creates larger angular acceleration. The CM shift also changes aerodynamic moment arm:

```text
tau_aero = (r_CP - r_CM(t)) x F_N
```

and thrust-dependent TVC authority changes according to:

```text
tau_max,TVC(t) ~= L T(t) sin(delta_max)
```

The closed-loop consequence is that a fixed-gain controller is operating on a moving plant. Week 7 remains stable, reaching `33.32 m` final altitude and `11.21 deg` maximum tilt, but the result motivates gain scheduling because mass, inertia, CM, dynamic pressure, and thrust level all affect control response and authority.
