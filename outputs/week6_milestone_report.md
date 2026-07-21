# Week 6 Milestone Report

## Objective

Add finite TVC actuator dynamics, including first-order gimbal lag, slew-rate limit, hard position limit, commanded-vs-achieved gimbal tracking, and torque-authority margin diagnostics.

## Truth-State Actuator-Limited LQR

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 31.42 m |
| Final vertical velocity | 20.90 m/s |
| Maximum tilt angle | 11.09 deg |
| Minimum body-axis vertical component | 0.981 |
| Maximum angular rate | 0.72 rad/s |
| Maximum lateral displacement | 10.67 m |
| Maximum quaternion norm error | 2.220e-16 |
| Maximum gimbal lag error | 1.50 deg |
| Maximum commanded gimbal | 1.50 deg |
| Maximum achieved gimbal | 0.86 deg |
| Minimum torque authority margin | 48.58 N m |
| Rate-limit fraction | 0.0% |
| Position-limit fraction | 0.0% |

## Estimated-State Actuator-Limited LQR

| Metric | Value |
| --- | ---: |
| Duration | 3.00 s |
| Samples | 601 |
| Final altitude | 31.43 m |
| Final vertical velocity | 20.91 m/s |
| Maximum tilt angle | 10.91 deg |
| Minimum body-axis vertical component | 0.982 |
| Maximum angular rate | 0.71 rad/s |
| Maximum lateral displacement | 10.58 m |
| Maximum quaternion norm error | 2.220e-16 |
| Maximum gimbal lag error | 1.49 deg |
| Maximum commanded gimbal | 1.49 deg |
| Maximum achieved gimbal | 0.85 deg |
| Minimum torque authority margin | 48.67 N m |
| Rate-limit fraction | 0.0% |
| Position-limit fraction | 0.0% |

## Upper-Division Actuator Dynamics Physics

Week 6 removes the remaining idealization inside the TVC loop: instantaneous nozzle motion. The controller can request a gimbal angle, but the plant only receives the achieved angle after servo bandwidth, slew-rate, and position limits:

```text
delta_dot_cmd = (delta_cmd - delta_act) / tau_servo
|delta_dot_act| <= delta_dot_max
|delta_act| <= delta_max
tau_TVC,act = r_engine x F(delta_act)
```

This matters because actuator dynamics insert phase lag between attitude error and corrective moment. In the transverse attitude channel, the controller is trying to damp:

```text
theta_dot = omega
I omega_dot = tau_TVC + tau_dist
```

If `tau_TVC` arrives late, the moment is applied to an older attitude/rate condition. At low lag this slightly increases overshoot. At higher lag, the feedback can lose phase margin, inject energy into the rotational mode, or drive the nozzle into rate/position saturation. In launch-vehicle GNC, controller gains therefore cannot be judged only against the rigid-body plant; they must be checked against actuator bandwidth and control authority.

## Result Interpretation

The truth-state actuator-limited LQR case reaches `31.42 m` final altitude, reaches `11.09 deg` maximum tilt, and accumulates `10.67 m` maximum lateral drift. The estimated-state actuator-limited case reaches `31.43 m`, reaches `10.91 deg` maximum tilt, and accumulates `10.58 m` maximum lateral drift. Relative to instant LQR TVC, the finite actuator mainly appears as a bounded command-tracking error rather than a stability loss.

The maximum gimbal lag is about `1.50 deg`. That lag is physically meaningful: it is the angular separation between the moment the controller wants now and the moment the nozzle can actually produce now. The rate-limit fraction remains `0.0%`, so the selected nominal maneuver is bandwidth-limited by first-order response but not slew-rate saturated. The torque-authority margin stays positive, meaning the required control moment remains below:

```text
tau_max,TVC ~= L T sin(delta_max)
```

This is the key controls conclusion. The LQR design is not merely stable with an ideal actuator; it retains enough phase and authority margin when the commanded torque is filtered through a finite-bandwidth gimbal servo and, in the second case, through sensor-based state estimation.

## Plot-Level Interpretation

### Tilt Response

The actuator-limited tilt traces stay close to the instant-LQR baseline. This indicates that the servo time constant is small compared with the dominant transverse rigid-body response. If the actuator bandwidth were too low, tilt would show larger overshoot because damping torque would arrive after angular rate had already grown.

### Lateral Drift

Lateral drift remains close to the instant-LQR case because attitude error remains bounded. The lateral position is an integrated quantity, so even small attitude-control delays can matter: a short interval of excess `T sin(theta)` produces lateral velocity that persists. The small drift change shows that actuator lag does not significantly increase time-integrated horizontal impulse in this nominal case.

### Commanded vs Achieved Gimbal

This panel is the actuator tracking diagnostic. The commanded trace is the control allocation request from the LQR torque command, while the achieved trace is the nozzle angle actually used by the dynamics. Their separation is phase/amplitude error introduced by the actuator, not sensor error or a plotting issue.

### Gimbal Lag Error

The lag-error panel makes actuator bandwidth visible. A peak lag near `1.5 deg` means the servo does not instantly reach the requested lateral-thrust vector. Since `tau_TVC = r_engine x F_thrust`, this angular lag directly becomes moment lag.

### Torque Authority Margin

Torque margin is the difference between available TVC moment and requested controller moment. A positive margin means the control law is asking for moments inside the feasible gimbal envelope. This is why the Week 6 result is more credible than an ideal-controller-only result: the response is checked against the actual actuator authority available to the vehicle.
