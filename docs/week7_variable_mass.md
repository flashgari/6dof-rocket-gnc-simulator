# Week 7: Variable-Mass Powered Ascent

## Objective

Week 7 adds a propulsion and mass-property schedule to the closed-loop ascent simulation. Earlier weeks use constant mass, constant inertia, fixed center of mass, and constant thrust. That is useful for building the dynamics and control stack, but a powered launch vehicle is a time-varying plant: propellant drains, thrust changes, center of mass shifts, inertia changes, and TVC moment authority varies with thrust.

The Week 7 model adds:

- piecewise-linear thrust curve
- propellant depletion from impulse and specific impulse
- time-varying vehicle mass
- shifting center of mass
- changing inertia tensor
- `T/m` acceleration diagnostics
- thrust-dependent TVC authority margin
- constant-mass vs variable-mass actuator-limited LQR comparison

## Propellant Depletion

Propellant usage is computed from thrust impulse:

```text
m_dot = -T / (Isp g0)
m(t) = m0 - integral(T / (Isp g0)) dt
```

This connects the propulsion model to the translational dynamics. The vehicle mass decreases according to the delivered impulse and engine specific impulse, not according to an arbitrary linear schedule.

## Translational Effect

The thrust acceleration scale is:

```text
a_thrust = T(t) / m(t)
```

This value does not have to increase monotonically. Decreasing mass tends to increase `T/m`, but a falling thrust curve can offset that effect. In the Week 7 run, thrust starts at `790 N`, peaks at `900 N`, then tails down to `760 N`. Mass decreases from `50.00 kg` to `48.93 kg`. As a result, `T/m` starts at `15.80 m/s^2`, peaks at `18.20 m/s^2`, and ends at `15.53 m/s^2`.

## Rotational Effect

The rotational dynamics become time-varying:

```text
I(t) omega_dot + omega x (I(t) omega) = tau
```

As propellant drains, the transverse inertia decreases from `3.15` to `3.07 kg m^2`. For the same TVC or aerodynamic moment, lower inertia produces larger angular acceleration. A fixed-gain controller therefore sees a different plant later in the burn than it saw at liftoff.

This is one of the main reasons launch-vehicle control laws are often scheduled. A gain set that gives good damping with wet inertia may become too aggressive near dry mass, while a gain set tuned near dry mass may be sluggish early in flight.

## CM Shift And Aerodynamic Moment

The center of mass shifts from `-0.080 m` to `-0.056 m` in body coordinates. The aerodynamic moment is:

```text
tau_aero = (r_CP - r_CM(t)) x F_N
```

Even if angle of attack and dynamic pressure were identical, the moment arm changes as CM moves. This means aerodynamic disturbance torque is time-varying. In the same way, any thrust-line offset or gimbaled-force geometry is affected by where the mass center is located.

## TVC Authority

Available TVC moment depends on thrust:

```text
tau_max,TVC(t) ~= L T(t) sin(delta_max)
```

The thrust curve therefore changes both vertical acceleration and control authority. During high-thrust segments, `T/m` increases and TVC authority increases. During thrust tailoff, both decrease. This coupling is important because a controller can lose authority late in burn even while the vehicle is lighter.

## Results

Variable-mass actuator-limited LQR TVC:

```text
Final altitude: 33.32 m
Maximum tilt: 11.21 deg
Maximum lateral drift: 11.12 m
Mass: 50.00 kg -> 48.93 kg
T/m: 15.80 -> 18.20 peak -> 15.53 m/s^2
Ixx: 3.15 -> 3.07 kg m^2
CM z: -0.080 -> -0.056 m
Minimum TVC torque margin: 50.27 N m
Rate-limit fraction: 0.0%
```

The variable-mass case climbs higher than the constant-mass actuator-limited baseline because the thrust curve has a high-thrust middle segment while mass is decreasing. Maximum tilt increases slightly but remains inside the controlled attitude corridor. That is the expected result for a fixed-gain controller operating on a changing plant.

## Plot Interpretation

### Altitude

The altitude plot shows variable mass outperforming the constant-mass actuator-limited reference. This is not just "lighter rocket goes higher." The result comes from the time integral of net acceleration, where `T(t)/m(t)`, gravity, drag, and thrust-axis pointing all matter.

### Mass And Thrust-To-Mass

Mass decreases smoothly because it is integrated from engine impulse. `T/m` rises during the high-thrust section and then falls near the end when thrust tailoff dominates the reduced mass. This is an important physical nuance: mass depletion alone does not determine acceleration.

### Inertia And CM Shift

The transverse inertia drop changes the rotational plant. The upward CM shift changes aerodynamic and propulsion moment arms. These are not secondary bookkeeping variables; they directly change `omega_dot` and the disturbance torques the controller must reject.

### Tilt Response

The LQR controller remains stable, but the variable-mass tilt trace differs slightly from the constant-mass case. That difference is the closed-loop signature of a time-varying plant. A more advanced version would schedule gains against mass state, dynamic pressure, or thrust level.

### TVC Authority Margin

Torque authority margin remains positive, so the controller's requested moment remains feasible. This matters because a thrust curve changes `tau_max,TVC`, not just vertical acceleration. Week 7 verifies that the controller still operates inside the available TVC moment envelope while mass properties vary.

## Engineering Takeaway

Week 7 connects propulsion, flight mechanics, and controls. The rocket is no longer a fixed plant with a constant engine; it is a time-varying system whose translational acceleration, rotational response, disturbance moments, and actuator authority evolve through the burn. The fixed LQR TVC controller remains stable for this reference schedule, and the result naturally motivates gain scheduling as the next controls upgrade.
