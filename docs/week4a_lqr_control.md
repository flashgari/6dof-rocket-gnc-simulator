# Week 4A LQR Attitude Control

Week 4A adds a small-angle LQR attitude controller and verifies it in the nonlinear 6-DOF simulation through the same finite TVC actuator used by the PD controller.

## Why Add LQR

PD control is physically interpretable, but its gains are manually selected. LQR introduces a standard aerospace controls workflow:

```text
linearize about an operating point -> choose Q/R weights -> solve optimal feedback -> verify on nonlinear plant
```

The value is not that LQR is automatically better. The value is that gain selection is tied to an explicit cost on state error and control effort.

## Linearized Attitude Model

Near upright ascent, pitch and yaw can be approximated as transverse double integrators. For one axis:

```text
theta_dot = omega
omega_dot = tau / I
```

or:

```text
x = [theta, omega]^T
u = tau
A = [[0, 1],
     [0, 0]]
B = [[0],
     [1/I]]
```

This model is local. It assumes small attitude error, moderate angular rate, and near-upright thrust-axis alignment. It is not a valid description of the open-loop tumble after the vehicle leaves the linearization neighborhood.

## LQR Cost Function

The controller minimizes:

```text
J = integral(x^T Q x + u^T R u) dt
```

where:

```text
Q = diag(q_angle, q_rate)
R = r_control
```

The attitude weight penalizes thrust-axis pointing error, which preserves `T cos(theta)`. The rate weight penalizes angular kinetic energy in the transverse modes. The control weight penalizes torque demand, which indirectly limits gimbal demand once the torque command is passed through the TVC allocator.

## Analytic Double-Integrator Feedback

For the double-integrator approximation, the resulting feedback has the same structural form as PD:

```text
tau = k_angle theta - k_rate omega
```

but the gains are determined by `Q`, `R`, and inertia rather than manual tuning. Since `omega_dot = tau/I`, inertia directly affects the rate gain needed for equivalent damping.

## Nonlinear Verification

The LQR law is not evaluated only on the linear model. Its torque request is allocated through TVC:

```text
tau_cmd -> F_lateral -> thrust direction
tau_TVC = r_engine x F_thrust
```

The verification run still includes quaternion attitude propagation, nonlinear thrust projection, wind-relative aerodynamics, CP/CM moment, thrust bias, finite gimbal authority, and actuator saturation tracking. This is the important GNC point: linear design is only credible after nonlinear verification.

## Comparison Physics

Open loop leaves the disturbance moment unopposed, so attitude error grows until thrust no longer projects upward. Ideal torque performs best because attitude moment and net thrust direction are decoupled. PD TVC and LQR TVC are actuator-realistic: any corrective moment requires lateral thrust and therefore some lateral acceleration.

The LQR TVC case reduces tilt and lateral drift relative to PD TVC in the nominal comparison because the chosen `Q/R` weights place more structured penalty on attitude error and angular rate while still avoiding gimbal saturation. This result should be interpreted as local robustness for the tested envelope, not as proof of global recovery from arbitrary tumbling attitude.

## Week 4A Comparison Plot Physics

### Altitude

The altitude panel is a direct readout of thrust-axis management. For any case with nonzero attitude error:

```text
T_vertical = T cos(theta)
```

Open loop loses altitude growth because `theta` approaches large values, so the engine can no longer maintain positive vertical acceleration. Ideal torque stays highest because it supplies corrective moment without redirecting the thrust vector. PD TVC and LQR TVC must generate moment by tilting thrust, so their vertical force is slightly reduced by the gimbal command. LQR TVC remains closer to ideal torque in this reference case because lower attitude error keeps `cos(theta)` closer to unity and reduces time spent with sideways thrust projection.

### Unwrapped Pitch

Unwrapped pitch exposes the rotational state without the `180 deg` ambiguity of unsigned tilt. Open loop is the response of:

```text
I omega_dot + omega x I omega = tau_dist
```

with no stabilizing feedback. The controlled traces show closed-loop damping of the transverse rotational mode. PD TVC and LQR TVC both act through the same engine lever arm, but LQR selects gains from the local state-space model, so its pitch response reflects a designed trade between attitude stiffness, angular-rate damping, and control effort rather than manual proportional/derivative tuning alone.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` is the most propulsion-relevant attitude metric in the plot. It maps directly to the fraction of thrust available for vertical acceleration. Open loop dropping toward zero or negative values means the vehicle has lost ascent authority. The controlled cases remain near `+1`, showing that feedback keeps the thrust axis inside the small-angle region where the LQR linearization is meaningful. This is important: LQR is not being credited for recovering a fully tumbling rocket; it is keeping the nonlinear plant from leaving the neighborhood where the linear design is valid.

### Lateral Drift

Lateral drift is the time integral of crossrange acceleration:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

Open loop drifts because attitude error creates horizontal thrust and wind-relative flow creates aerodynamic side force. Ideal torque has the least drift because it can generate moment without adding lateral thrust. PD TVC and LQR TVC retain the physical TVC coupling: lateral thrust is both the control mechanism and a source of crossrange acceleration. LQR TVC reduces drift relative to PD TVC in this nominal case because smaller attitude excursions reduce the `T sin(theta)` contribution over the ascent.

### TVC Actuator Usage

The gimbal panel is an actuator-authority plot, not just a control-effort plot. The available moment envelope is approximately:

```text
tau_max,TVC ~= L T sin(delta_max)
```

PD TVC and LQR TVC may request different time histories of lateral thrust because their gains weight attitude error and rate differently. If the gimbal trace approached the limit or the saturation flag increased, the comparison would shift from controller tuning to actuator insufficiency. In the Week 4A reference case, both controllers remain below the gimbal limit, so the LQR improvement can be interpreted as a feedback-law effect rather than a saturation artifact.
