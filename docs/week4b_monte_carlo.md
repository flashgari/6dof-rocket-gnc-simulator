# Week 4B Monte Carlo Robustness Campaign

Week 4B evaluates the controllers across randomized off-nominal conditions. The goal is to measure robustness over a defined dispersion set rather than rely on a single nominal trajectory.

## Why Monte Carlo Matters

A nominal simulation answers one question: does the controller work for one selected vehicle and environment?

A Monte Carlo campaign asks a stronger GNC question: does the controller still work when the vehicle and environment are uncertain?

Launch vehicles see uncertainty in wind, mass properties, thrust alignment, aerodynamic coefficients, center-of-pressure location, and actuator authority. A credible controller should be tested against those dispersions.

## Dispersed Parameters

Each trial randomizes:

- mass
- pitch/yaw inertia scale
- thrust magnitude
- wind speed and direction
- thrust misalignment
- thrust offset for the open-loop case
- center-of-pressure location
- aerodynamic normal-force slope
- maximum gimbal angle

The random seed is fixed so the campaign is reproducible.

## Controllers Compared

The campaign runs each dispersion three ways:

- `open_loop`: disturbed vehicle with no feedback
- `pd_tvc`: nonlinear PD attitude feedback allocated through TVC
- `lqr_tvc`: small-angle LQR attitude feedback allocated through TVC

The two closed-loop cases use the same finite gimbal actuator model:

```text
tau = r_engine x F_thrust
```

This keeps the comparison actuator-realistic.

## Pass/Fail Criteria

A case passes if:

```text
max tilt < 25 deg
final altitude > 20 m
max lateral drift < 25 m
gimbal saturation fraction < 10%
```

These are not mission requirements. They are engineering gates chosen to compare controller robustness with objective thresholds.

## Physical Interpretation

Open loop usually fails because the dispersed parameters perturb both the forcing environment and the rotational plant. Wind changes `v_rel,B` and therefore dynamic pressure and angle of attack. CP location and `C_N_alpha` change the aerodynamic moment slope. Mass and inertia change translational acceleration and angular acceleration per unit force/moment. Thrust magnitude, thrust misalignment, and thrust offset change both the nominal acceleration level and the propulsion-induced moment.

In open loop, these effects enter:

```text
I omega_dot + omega x I omega = tau_thrust + tau_aero
tau_aero = (r_CP - r_CM) x F_N
F_N ~ qbar S C_N_alpha alpha
```

With no stabilizing term, even moderate disturbance moments can drive the vehicle outside the useful ascent attitude region. Once `theta` grows, the translational equations change character because `T cos(theta)` decreases and `T sin(theta)` increases.

PD TVC succeeds by applying restoring moment through gimbaled thrust. It keeps the body `+z` axis near inertial up, preserving vertical thrust authority over the dispersion set. Some lateral drift remains because TVC creates attitude torque by spending part of the thrust vector sideways.

LQR TVC uses the same actuator but selects gains from a state-error/control-effort cost. In the reference campaign, the LQR trade reduces median maximum tilt and median lateral drift relative to PD TVC while still avoiding actuator saturation. This is a local robustness result: the controller keeps the nonlinear trajectories close enough to the upright linearization point for the LQR design assumptions to remain reasonable.

## Summary Plot Physics

### Success Rate

The success-rate panel is the top-level robustness metric for the dispersion set. Each trial perturbs the forcing and moment environment, so the open-loop vehicle sees a different combination of wind-relative velocity, thrust-axis bias, mass properties, aerodynamic normal-force slope, and CP/CM moment arm. In body coordinates, the relative wind creates transverse velocity and angle of attack. The simplified normal-force model scales this into aerodynamic side force through dynamic pressure:

```text
qbar = 0.5 rho |v_rel|^2
F_N ~ qbar S C_N_alpha alpha
tau_aero = (r_CP - r_CM) x F_N
```

Because the demonstration vehicle intentionally places CP above CM, the aerodynamic moment is destabilizing for this ascent configuration. Thrust misalignment and thrust offset add additional propulsion-induced moment. In open loop, the rotational dynamics have no feedback term opposing the disturbance moment:

```text
I omega_dot + omega x I omega = tau_disturbance
```

The result is a low success rate: small initial angular accelerations integrate into angular rate, angular rate integrates into attitude error, and attitude error rotates the thrust vector away from inertial up.

PD TVC and LQR TVC reach 100% success in this sampled envelope because feedback closes that rotational-energy growth path. Both controllers command restoring moment, and the TVC allocator converts that moment into lateral thrust through the engine lever arm. This does not prove global stability for every possible vehicle state; it demonstrates robustness over the specified uncertainty set and pass/fail gates.

The important upper-division interpretation is that success rate is a statement about the closed-loop region of attraction under the selected dispersions. The open-loop vehicle leaves the acceptable attitude envelope for almost every sampled case. The controlled vehicles keep the sampled trajectories inside a bounded region where `max tilt`, `final altitude`, lateral drift, and actuator saturation all remain within the engineering gates.

### Median Maximum Tilt

The median maximum tilt panel measures the typical peak attitude excursion, not merely the final attitude. That distinction matters because a rocket can pass through inverted flight and later rotate toward upright again without ever recovering useful ascent performance. The open-loop median near 177 degrees means the representative trial tumbles to nearly inverted attitude. At that point, the thrust projection onto inertial up is approximately:

```text
T_vertical = T cos(theta)
```

so a large tilt destroys vertical acceleration even if thrust magnitude is unchanged. It also couples attitude error into translation because the horizontal component `T sin(theta)` accelerates the vehicle laterally.

PD TVC reduces median maximum tilt to about 12 degrees by creating a stabilizing torque with gimbaled thrust:

```text
tau_TVC = r_engine x F_thrust
```

For an engine below the center of mass, the lateral thrust component creates pitch/yaw moment. LQR TVC reduces the median tilt further in this campaign because its gains are selected from an explicit quadratic cost on attitude error, angular rate, and control effort. In practical GNC terms, the LQR law damps the transverse rotational modes more efficiently for this chosen dispersion set while staying inside the same gimbal limits.

Median maximum tilt is used instead of final tilt because final attitude can be misleading after a tumble. A vehicle can rotate through inverted and later point upward again while the trajectory has already lost altitude and gained crossrange velocity. The peak metric captures whether the controller prevented departure from the ascent attitude envelope at any point during the burn.

### Median Maximum Lateral Drift

The lateral-drift panel is the translational footprint of the attitude dynamics. Drift is not an independent failure mode; it is primarily caused by attitude error and aerodynamic side force. Open loop has the largest drift because tilt produces a horizontal thrust component and wind-relative flow produces side force. In first-order form:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

As attitude diverges, the thrust term grows and the vehicle converts engine energy into crossrange motion instead of altitude. This is why lateral drift is a useful integrated metric: it captures both aerodynamic disturbance rejection and thrust-axis management over time.

PD TVC cuts median lateral drift roughly in half by keeping `theta` small enough that most thrust remains axial/upward. LQR TVC reduces drift further in this reference campaign because lower median attitude error reduces sideways thrust projection. The remaining drift is the expected actuator-coupling penalty of TVC: a gimbaled engine creates control moment by intentionally introducing lateral thrust, so attitude correction and lateral acceleration are coupled. The engineering trade is to use enough lateral thrust to stabilize attitude without spending excessive impulse sideways.

This metric also integrates performance over time. A short attitude excursion may not violate final tilt, but it can still inject lateral velocity. That is why lateral drift is an important complement to maximum tilt: it captures the trajectory cost of attitude error and TVC control effort, not only instantaneous pointing quality.

### Worst-Case Tilt And Drift

The table also reports worst-case values because medians alone can hide tail risk. In launch-vehicle verification, the tail of the dispersion distribution is often where actuator margin and control robustness are exposed. The worst open-loop tilt approaching `180 deg` confirms near-complete loss of attitude authority in at least one trial. The lower worst-case tilt and drift for LQR TVC relative to PD TVC indicate improved margin over this sampled set, but the conclusion is bounded by the chosen dispersions and pass/fail thresholds.

### Mean Saturation Fraction

The saturation fraction checks whether the controller is succeeding by demanding unrealistic actuator authority. For TVC:

```text
F_lateral,max = T sin(delta_max)
tau_max,TVC ~= L T sin(delta_max)
```

A high success rate with high saturation would be less convincing because the controller would be operating at the edge of authority. The reported near-zero saturation means the successful closed-loop cases remain inside the modeled actuator envelope. That makes the robustness claim stronger: the controller is not merely clipping commands and hoping the plant survives; it is stabilizing the dispersed cases with available control authority.

## Interview Talking Point

The key phrase is: **nominal success is not robustness**.

Week 4B demonstrates a verification mindset: define dispersions, run many cases, apply pass/fail criteria, and compare controllers statistically.
