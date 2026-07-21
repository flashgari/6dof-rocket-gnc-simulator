# Week 7 Milestone Report

## Objective

Add a thrust curve, propellant depletion from impulse and specific impulse, time-varying mass, shifting center of mass, changing inertia tensor, and TVC authority diagnostics under variable vehicle properties.

## Variable-Mass LQR TVC Case

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 33.32 m |
| Final vertical velocity | 21.75 m/s |
| Maximum tilt angle | 11.21 deg |
| Minimum body-axis vertical component | 0.981 |
| Maximum angular rate | 0.70 rad/s |
| Maximum lateral displacement | 11.12 m |
| Maximum quaternion norm error | 2.220e-16 |
| Initial mass | 50.00 kg |
| Final mass | 48.93 kg |
| Initial thrust | 790.0 N |
| Maximum thrust | 900.0 N |
| Final thrust | 760.0 N |
| Initial thrust-to-mass | 15.80 m/s^2 |
| Maximum thrust-to-mass | 18.20 m/s^2 |
| Final thrust-to-mass | 15.53 m/s^2 |
| Initial transverse inertia | 3.15 kg m^2 |
| Final transverse inertia | 3.07 kg m^2 |
| Initial CM z | -0.080 m |
| Final CM z | -0.056 m |
| Minimum torque authority margin | 50.27 N m |
| Maximum gimbal lag error | 1.50 deg |
| Rate-limit fraction | 0.0% |

## Upper-Division Variable-Mass Flight Physics

Week 7 adds the coupling between propulsion, mass properties, and closed-loop control. The vehicle no longer flies with constant mass, constant inertia, or constant thrust. Propellant depletion is computed from engine impulse:

```text
m_dot = -T / (Isp g0)
m(t) = m0 - integral(T / (Isp g0)) dt
```

The thrust curve changes translational acceleration through:

```text
a_thrust = T(t) / m(t)
```

This quantity does not necessarily increase monotonically. Mass decreases as propellant drains, but thrust can ramp up or tail off depending on the engine curve. In the Week 7 reference case, `T/m` starts at `15.80 m/s^2`, reaches a higher value during the high-thrust segment, and ends at `15.53 m/s^2` after thrust tails down to `760 N`.

The rotational plant also changes:

```text
I(t) omega_dot + omega x (I(t) omega) = tau
```

As propellant mass decreases and the mass distribution moves toward the dry vehicle, transverse inertia decreases. For a fixed moment, lower inertia increases angular acceleration. That means the same controller gains can become more aggressive later in the burn. This is the physical motivation for gain scheduling in real ascent GNC: a controller tuned for wet mass may not have identical damping and bandwidth near dry mass.

The center of mass also shifts:

```text
tau_aero = (r_CP - r_CM(t)) x F_N
tau_TVC = r_engine x F_thrust
```

Moving CM changes both the aerodynamic moment arm and the effective geometry between engine force and vehicle mass center. In this simplified model, the CP stays fixed while CM moves upward as propellant drains, so the CP/CM lever arm changes during flight. This makes aerodynamic disturbance torque time-varying even for the same angle of attack and dynamic pressure.

## Result Interpretation

The variable-mass LQR TVC case reaches `33.32 m` final altitude, `11.21 deg` maximum tilt, and `11.12 m` maximum lateral drift. Mass decreases from `50.00 kg` to `48.93 kg`, transverse inertia decreases, and CM moves from `-0.080 m` to about `-0.056 m` in body coordinates.

Compared with constant-mass actuator-limited LQR, the variable-mass case climbs higher because the thrust curve has a high-thrust middle segment and the vehicle becomes lighter during the burn. The attitude corridor remains controlled, but maximum tilt is slightly larger because changing inertia and CM modify the rotational response and disturbance moment arms while the controller gains remain fixed.

The TVC authority margin remains positive. That matters because available TVC moment also varies with thrust:

```text
tau_max,TVC(t) ~= L T(t) sin(delta_max)
```

A thrust curve changes both acceleration and moment authority. Higher thrust increases `T/m` and increases available TVC torque; lower thrust near the end reduces both. Week 7 therefore makes the controller problem more coupled: propulsion performance, mass depletion, and actuator authority all move together.

## Plot-Level Interpretation

### Altitude

The variable-mass trajectory climbs above the constant-mass actuator-limited baseline. This is not only because mass decreases; it is because the integrated thrust curve supplies a stronger middle-burn acceleration while drag and gravity losses remain comparable over the short window.

### Mass And Thrust-To-Mass

Mass decreases according to impulse over `Isp g0`. `T/m` follows both the changing numerator and denominator. The final `T/m` is lower than the initial value because thrust tails down enough to offset the reduced mass.

### Inertia And CM Shift

The decreasing `Ixx` indicates that the transverse rotational plant is changing. Lower inertia means a given TVC or aerodynamic moment produces larger angular acceleration. The CM shift changes `r_CP - r_CM`, so aerodynamic torque is not constant even if the aerodynamic force model were unchanged.

### Tilt Response

The controlled tilt remains inside the attitude corridor, which means the fixed LQR gains still tolerate the modeled mass-property variation. The slight increase relative to constant-mass LQR is physically expected because the plant poles and disturbance moment arms are moving during the burn.

### TVC Authority Margin

Authority margin stays positive, so the controller is not asking for unavailable moment. Since `tau_max,TVC` scales with thrust, the same thrust curve that changes vertical acceleration also changes the available attitude-control moment.
