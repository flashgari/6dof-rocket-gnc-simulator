# Week 3B Thrust Vector Control

Week 3B replaces ideal body torque with a thrust-vector-control actuator model. This is the first point where the controller must respect how moment is actually produced by a launch vehicle engine.

## Why TVC Matters

An ascent vehicle does not directly command arbitrary `tau_B`. It changes the thrust direction, and the thrust force acts at an engine location below the center of mass:

```text
tau_TVC = r_engine x F_thrust
```

The available pitch/yaw moment therefore depends on thrust magnitude, engine lever arm, gimbal angle, and actuator saturation. This couples attitude control to translational acceleration because the same thrust vector determines both moment and net force.

## Small-Angle Torque Allocation

For:

```text
r_engine = [0, 0, -L]
F_thrust = [F_x, F_y, F_z]
```

the generated moment is:

```text
tau = r x F = [L F_y, -L F_x, 0]
```

Thus pitch and yaw moments are produced by lateral thrust components. The sign convention matters: positive body `+x` thrust produces negative body-`y` torque when the engine is below the center of mass.

## Gimbal Authority And Saturation

The maximum lateral thrust component is limited by gimbal angle:

```text
F_lateral,max = T sin(delta_max)
```

and the approximate moment limit is:

```text
tau_max,TVC ~= L T sin(delta_max)
```

If the feedback law requests more moment than this, the allocator saturates. This distinction is central to GNC: a stable mathematical control law can still fail if the actuator cannot supply the demanded torque.

## Control Interpretation

Week 3B tests the same attitude objective as Week 3A but through finite thrust-vector authority. The controller must keep `z_body,I` aligned with inertial up while using an actuator that deliberately introduces lateral force to produce moment. The TVC response differs from direct body torque because attitude correction and lateral acceleration are no longer separable.

## Control Comparison Plot Physics

### Altitude

The open-loop altitude curve peaks because disturbance moments rotate the thrust vector away from inertial up:

```text
T_vertical = T cos(theta)
```

As `theta` grows, vertical acceleration decreases even though engine thrust magnitude remains constant. Ideal torque climbs highest because it can regulate attitude without redirecting thrust. TVC climbs slightly less because some thrust vector is spent laterally to generate corrective moment.

### Unwrapped Pitch

Open-loop pitch diverges because the net applied moment:

```text
tau_dist = tau_thrust_bias + tau_offset + tau_aero
```

is not opposed by feedback. The ideal controller supplies direct `Kp e - Kd omega` body torque. TVC follows the same feedback objective, but the requested moment must be realized through `r_engine x F_thrust`, so transient pitch error reflects both controller dynamics and actuator allocation limits.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` is an attitude metric with direct propulsion meaning. When it approaches zero, thrust is mostly horizontal; when it becomes negative, thrust has a downward component. The controlled cases staying near `+1` shows that feedback preserves vertical thrust authority and keeps the vehicle inside the local ascent regime.

### Lateral Drift

Open-loop lateral drift follows from the lateral component of the translational equation:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

The ideal-torque case has the lowest drift because attitude torque is applied without changing the thrust-force direction. TVC still reduces drift relative to open loop, but it cannot eliminate the actuator coupling: lateral thrust is the mechanism used to generate stabilizing moment.

### TVC Actuator Usage

The gimbal trace is a control-authority diagnostic. Large early deflections correspond to the controller arresting angular-rate growth from the disturbed initial ascent. The later lower gimbal demand indicates that the vehicle has entered a bounded attitude-error regime where the actuator primarily balances persistent thrust bias and aerodynamic loading. Saturation fraction indicates whether the demanded moment remained inside the finite TVC envelope.
