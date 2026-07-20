# Week 2 Milestone Report

## Objective

Add wind, thrust misalignment, thrust offset, aerodynamic drag, normal force, and CP/CM moment to produce an open-loop failure case.

## Disturbed Case

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 3.99 m |
| Final vertical velocity | -10.33 m/s |
| Maximum tilt angle | 177.63 deg |
| Minimum body-axis vertical component | -0.999 |
| Maximum angular rate | 5.83 rad/s |
| Maximum lateral displacement | 25.10 m |
| Maximum quaternion norm error | 2.220e-16 |

## Upper-Division Physical Interpretation

The Week 2 trajectory fails because rotational dynamics feed back into translational dynamics through thrust-vector projection. The disturbance moment is the sum of propulsion and aerodynamic contributions:

```text
tau_dist = r_T x F_T + (r_CP - r_CM) x F_N
I omega_dot + omega x I omega = tau_dist
```

The aerodynamic normal force is driven by wind-relative angle of attack:

```text
v_rel,B = R_IB(q)(v_I - wind_I)
qbar = 0.5 rho |v_rel|^2
F_N ~ qbar S C_N_alpha alpha
```

Because the demonstration configuration places CP above CM in the body-axis convention, the normal-force moment is destabilizing rather than restoring. Thrust misalignment and thrust offset add persistent propulsion-induced moments. With no feedback term, angular acceleration integrates into angular rate, and angular rate integrates into attitude error.

## Plot Physics

### Altitude

The altitude curve peaks because vertical thrust authority collapses as attitude error grows:

```text
T_vertical = T cos(theta)
```

Early in flight, `theta` is small and `T cos(theta)` exceeds weight plus vertical drag. As the vehicle tips, the vertical component decreases; near horizontal flight, thrust mainly accelerates the vehicle laterally; through inverted portions, thrust can project downward. The altitude peak is therefore an attitude-induced loss of vertical acceleration, not an engine-performance effect.

### Unwrapped Pitch

Unwrapped pitch shows the continuous rotational state instead of folding at `180 deg`. The monotonically growing pitch magnitude is the integral of uncontrolled angular rate. This is the correct diagnostic for tumble because a folded tilt angle can falsely look like recovery after the body rotates past inverted.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` directly measures thrust-axis alignment with inertial up. Values near `+1`, `0`, and `-1` correspond to upward, horizontal, and downward thrust projection. The curve crossing toward negative values indicates the vehicle has lost ascent authority even if the engine is still producing thrust.

### Lateral Drift

Lateral drift is the integrated footprint of horizontal acceleration:

```text
m a_lateral ~= T sin(theta) + F_N,lateral - D_lateral
```

Once attitude diverges, the thrust term dominates and engine impulse is spent crossrange instead of vertically. The plot therefore shows coupled aero-propulsive instability, not simply the wind pushing the rocket sideways.
