# Week 4B Milestone Report

## Objective

Evaluate robustness statistically by running randomized off-nominal vehicle and environment dispersions through open-loop, PD TVC, and LQR TVC cases.

## Pass/Fail Criteria

```text
max tilt < 25 deg
final altitude > 20 m
max lateral drift < 25 m
gimbal saturation fraction < 10%
```

## Results

| Controller | Success rate | Median max tilt | Median max lateral drift | Worst max tilt | Worst lateral drift | Mean saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Open loop | 1.0% | 177.04 deg | 24.29 m | 179.82 deg | 27.74 m | 0.0% |
| PD TVC | 100.0% | 12.05 deg | 12.69 m | 22.88 deg | 23.56 m | 0.0% |
| LQR TVC | 100.0% | 9.67 deg | 10.35 m | 17.92 deg | 19.10 m | 0.0% |

## Summary Plot Physics

### Success Rate

The success-rate panel is a robustness metric over the specified uncertainty set. Each dispersion changes wind-relative velocity, dynamic pressure, mass properties, thrust alignment, CP/CM lever arm, aerodynamic normal-force slope, and gimbal authority. The open-loop vehicle has no feedback term in:

```text
I omega_dot + omega x I omega = tau_dist
```

so disturbance moments integrate into angular rate and attitude error. The closed-loop controllers succeed because they close that rotational-energy growth path and maintain enough thrust-axis alignment to satisfy the altitude, tilt, drift, and saturation gates.

The result means the open-loop system has almost no robustness margin over the selected dispersions: changing wind, CP location, thrust alignment, or inertia usually pushes the vehicle outside the acceptable ascent envelope. The 100% closed-loop success rates mean both TVC controllers maintain a bounded rotational response for this uncertainty set; it is a statement about the sampled region of operation, not a claim of global stability.

### Median Maximum Tilt

Median maximum tilt is a peak-excursion statistic, not a final-state statistic. Open loop approaching `177 deg` indicates typical near-inverted tumble. At those attitudes:

```text
T_vertical = T cos(theta)
```

is near zero or negative, so thrust magnitude no longer translates into ascent performance. PD TVC and LQR TVC reduce this metric by producing stabilizing pitch/yaw moments through the engine lever arm. LQR improves the median in this campaign because the chosen `Q/R` weights penalize both attitude error and angular-rate energy.

This panel is relevant because maximum tilt is a peak-envelope metric. A vehicle can end a run with a tolerable final attitude after rotating through an unacceptable excursion. Median maximum tilt measures whether the controller prevented departure from the ascent attitude corridor during the burn.

### Median Maximum Lateral Drift

Lateral drift is the integrated translational consequence of attitude error:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

Open loop converts engine impulse into crossrange motion. PD TVC reduces drift by keeping `theta` small. LQR TVC reduces it further in this reference campaign because lower median attitude error produces less horizontal thrust projection. Nonzero closed-loop drift is the actuator-coupling penalty of TVC because corrective moment is produced by deliberately introducing lateral thrust.

This panel is relevant because drift integrates errors over the trajectory. A brief attitude excursion can inject lateral velocity even if final attitude later looks acceptable. The LQR reduction from the PD TVC median indicates a lower accumulated crossrange impulse, not merely a prettier attitude trace.

### Worst-Case Metrics In The Table

Worst-case tilt and lateral drift are tail-risk indicators. Median values show typical behavior, but worst-case values reveal whether a small part of the dispersion set is near the edge of control authority. The lower LQR worst-case tilt and drift indicate more margin than PD TVC for this sampled set, while the conclusion remains bounded by the selected dispersions.

### Mean Saturation

Mean saturation checks whether robustness is being purchased with unavailable actuator authority. For TVC:

```text
F_lateral,max = T sin(delta_max)
tau_max,TVC ~= L T sin(delta_max)
```

Near-zero saturation means the successful closed-loop runs remain inside the finite gimbal envelope. That makes the result relevant to GNC verification: the controllers are not relying on impossible moments to pass the Monte Carlo gates.

## Engineering Takeaway

Nominal success is not robustness. Week 4B evaluates controller performance against dispersions in wind, mass properties, thrust alignment, aerodynamics, and actuator authority, then compares controllers using objective trajectory and actuator metrics.
