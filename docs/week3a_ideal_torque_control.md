# Week 3A Ideal-Torque Attitude Control

Week 3A introduces closed-loop attitude control with an ideal body-torque actuator. This isolates the feedback law from actuator allocation so the control objective can be validated before adding TVC geometry.

## Why Ideal Torque First

The control problem is separated into two questions:

```text
1. Does the attitude feedback law stabilize the nonlinear rigid body?
2. Can a real actuator generate the requested moment?
```

Week 3A answers the first question. If direct body torque cannot stabilize the disturbed ascent, then a gimbaled engine will not solve the underlying feedback problem. If ideal torque works, Week 3B can focus on the actuator mapping:

```text
tau_cmd -> thrust direction -> r_engine x F_thrust
```

## Attitude Error Definition

The regulated vector is the body `+z` axis because it is also the thrust axis:

```text
z_body,I = R_BI(q) [0, 0, 1]
z_cmd,I  = [0, 0, 1]
```

The vector attitude error is:

```text
e_I = z_body,I x z_cmd,I
e_B = R_IB(q) e_I
```

For small angular errors, this cross product is approximately the rotation vector needed to align the thrust axis with inertial up. Its direction gives the corrective torque axis, and its magnitude scales with misalignment. Expressing the error in the body frame makes it compatible with body-frame Euler rotational dynamics.

## PD Control Law

The ideal torque command is:

```text
tau_cmd,B = Kp e_B - Kd omega_B
```

The proportional term is an attitude stiffness term. It creates a restoring moment that drives the thrust axis back toward the commanded vertical direction. The derivative term is rotational damping; it extracts angular kinetic energy from the transverse modes so the vehicle does not simply rotate through the target attitude.

This is a nonlinear vector-control implementation, even though the gain structure resembles small-angle PD. The error is computed from the quaternion-derived body axis rather than from a fragile Euler-angle sequence.

## Saturation

The ideal actuator is bounded:

```text
|tau_cmd| <= tau_max
```

This prevents the Week 3A result from relying on unbounded torque authority. It is still less realistic than TVC, but it establishes the concept of finite moment authority before the torque request is converted into lateral thrust and gimbal angle.

## Expected Closed-Loop Physics

Relative to the Week 2 open-loop case, ideal torque should reduce the rotational-energy growth caused by thrust misalignment, thrust offset, and CP/CM aerodynamic moment. Keeping `body_z_z` near `+1` preserves:

```text
T_vertical = T cos(theta)
```

and suppresses:

```text
T_lateral = T sin(theta)
```

Therefore the controlled trajectory should retain altitude, reduce lateral drift, and keep angular rates bounded. The comparison is a closed-loop GNC verification result: disturbance moments still exist, but feedback prevents them from integrating into tumble.
