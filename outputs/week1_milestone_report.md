# Week 1 Milestone Report

## Objective

Verify the 13-state rigid-body dynamics core before adding aerodynamic disturbances or feedback control.

## Baseline Case

| Metric | Value |
| --- | ---: |
| Duration | 10.00 s |
| Samples | 1001 |
| Final altitude | 359.67 m |
| Final vertical velocity | 71.93 m/s |
| Maximum tilt angle | 0.00 deg |
| Minimum body-axis vertical component | 1.000 |
| Maximum angular rate | 0.00 rad/s |
| Maximum lateral displacement | 0.00 m |
| Maximum quaternion norm error | 0.000e+00 |

## Upper-Division Physical Interpretation

Week 1 is a verification case for the rigid-body equations, not a flight-performance claim. With aligned thrust, no aerodynamic force, and zero external moment, the translational dynamics reduce to:

```text
m v_dot_I = [0, 0, T - mg]
```

so the vertical acceleration is constant:

```text
a_z = T/m - g
```

The parabolic altitude trace and linear vertical-velocity trace are direct consequences of this constant net force. Any lateral acceleration or angular acceleration in this case would indicate a frame convention, force-projection, or torque-balance error.

The quaternion norm plot is also a physics check. The quaternion defines the rotation used to project body-frame thrust and aerodynamic forces into inertial coordinates. If `|q| != 1`, the implied rotation is no longer orthonormal and later force projections become nonphysical. Maintaining unit norm therefore verifies the attitude-state integrity needed before adding CP/CM moments and feedback control.

The conservation tests check limiting cases of the governing equations: ballistic energy conservation, force-free linear momentum conservation, torque-free inertial angular momentum conservation, and `tau = r x F` angular acceleration from an offset force.
