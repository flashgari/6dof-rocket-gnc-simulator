#!/usr/bin/env python3
"""Write milestone reports with aerospace-level physical interpretation."""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
OUT = PROJECT_ROOT / "outputs"

from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import summary_metrics


def load_samples(path: Path) -> list[tuple[float, State]]:
    with path.open() as f:
        samples = []
        for row in csv.DictReader(f):
            samples.append(
                (
                    float(row["time_s"]),
                    State(
                        (float(row["x_m"]), float(row["y_m"]), float(row["z_m"])),
                        (float(row["vx_mps"]), float(row["vy_mps"]), float(row["vz_mps"])),
                        (float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])),
                        (float(row["wx_radps"]), float(row["wy_radps"]), float(row["wz_radps"])),
                    ),
                )
            )
        return samples


def metrics_table(metrics: dict[str, float]) -> str:
    return f"""| Metric | Value |
| --- | ---: |
| Duration | {metrics["duration_s"]:.2f} s |
| Samples | {metrics["samples"]:.0f} |
| Final altitude | {metrics["final_altitude_m"]:.2f} m |
| Final vertical velocity | {metrics["final_vertical_velocity_mps"]:.2f} m/s |
| Maximum tilt angle | {metrics["max_tilt_deg"]:.2f} deg |
| Minimum body-axis vertical component | {metrics["min_body_z_vertical_component"]:.3f} |
| Maximum angular rate | {metrics["max_angular_rate_radps"]:.2f} rad/s |
| Maximum lateral displacement | {metrics["max_lateral_displacement_m"]:.2f} m |
| Maximum quaternion norm error | {metrics["max_quaternion_norm_error"]:.3e} |
"""


def saturation_fraction(path: Path) -> float:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows or "saturated" not in rows[0]:
        return 0.0
    return sum(int(row["saturated"]) for row in rows) / len(rows)


def load_monte_carlo(path: Path) -> list[dict[str, float | str]]:
    with path.open() as f:
        converted_rows = []
        for row in csv.DictReader(f):
            converted: dict[str, float | str] = {}
            for key, value in row.items():
                converted[key] = value if key == "controller" else float(value)
            converted_rows.append(converted)
        return converted_rows


def monte_carlo_summary(rows: list[dict[str, float | str]]) -> dict[str, dict[str, float]]:
    summary: dict[str, dict[str, float]] = {}
    for controller in ("open_loop", "pd_tvc", "lqr_tvc"):
        subset = [row for row in rows if row["controller"] == controller]
        summary[controller] = {
            "trials": float(len(subset)),
            "success_rate": sum(float(row["passed"]) for row in subset) / len(subset),
            "median_max_tilt_deg": sorted(float(row["max_tilt_deg"]) for row in subset)[len(subset) // 2],
            "median_max_lateral_m": sorted(float(row["max_lateral_m"]) for row in subset)[len(subset) // 2],
            "worst_max_tilt_deg": max(float(row["max_tilt_deg"]) for row in subset),
            "worst_max_lateral_m": max(float(row["max_lateral_m"]) for row in subset),
            "mean_saturation_fraction": sum(float(row["saturation_fraction"]) for row in subset) / len(subset),
        }
    return summary


def monte_carlo_table(summary: dict[str, dict[str, float]]) -> str:
    names = {"open_loop": "Open loop", "pd_tvc": "PD TVC", "lqr_tvc": "LQR TVC"}
    lines = [
        "| Controller | Success rate | Median max tilt | Median max lateral drift | Worst max tilt | Worst lateral drift | Mean saturation |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for key in ("open_loop", "pd_tvc", "lqr_tvc"):
        data = summary[key]
        lines.append(
            f"| {names[key]} | {100.0 * data['success_rate']:.1f}% | "
            f"{data['median_max_tilt_deg']:.2f} deg | {data['median_max_lateral_m']:.2f} m | "
            f"{data['worst_max_tilt_deg']:.2f} deg | {data['worst_max_lateral_m']:.2f} m | "
            f"{100.0 * data['mean_saturation_fraction']:.1f}% |"
        )
    return "\n".join(lines) + "\n"


def estimator_metrics(path: Path) -> dict[str, float]:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    errors = [float(row["attitude_error_deg"]) for row in rows]
    rates = [float(row["rate_error_radps"]) for row in rows]
    return {
        "max_attitude_error_deg": max(errors),
        "rms_attitude_error_deg": math.sqrt(sum(error**2 for error in errors) / len(errors)),
        "max_rate_error_radps": max(rates),
        "saturation_fraction": sum(int(row["saturated"]) for row in rows) / len(rows),
    }


def actuator_metrics(path: Path) -> dict[str, float]:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    return {
        "max_gimbal_lag_error_deg": max(float(row["gimbal_lag_error_deg"]) for row in rows),
        "max_commanded_gimbal_deg": max(float(row["cmd_gimbal_total_deg"]) for row in rows),
        "max_achieved_gimbal_deg": max(float(row["ach_gimbal_total_deg"]) for row in rows),
        "min_torque_authority_margin_nm": min(float(row["torque_authority_margin_nm"]) for row in rows),
        "rate_limit_fraction": sum(int(row["rate_limited"]) for row in rows) / len(rows),
        "position_limit_fraction": sum(int(row["position_limited"]) for row in rows) / len(rows),
        "saturation_fraction": sum(int(row["saturated"]) for row in rows) / len(rows),
    }


def week2_reference() -> tuple[RocketParams, Environment]:
    misalign = math.radians(1.5)
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=2.5,
        center_of_pressure_body_m=(0.0, 0.0, 0.35),
        thrust_offset_body_m=(0.004, 0.0, 0.0),
        thrust_direction_body=(math.sin(misalign), 0.0, math.cos(misalign)),
    )
    return rocket, Environment(wind_mps=(4.0, 1.0, 0.0))


WEEK1_PHYSICS = """
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
"""


WEEK2_PHYSICS = """
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
"""


WEEK3A_PHYSICS = """
## Upper-Division Control Physics

The Week 3A controller regulates the thrust-axis direction rather than an Euler angle:

```text
z_body,I = R_BI(q)[0, 0, 1]
e_I = z_body,I x z_cmd,I
e_B = R_IB(q)e_I
tau_cmd,B = Kp e_B - Kd omega_B
```

For small attitude error, `e_B` is approximately the transverse rotation vector needed to align the thrust axis with inertial up. The proportional term acts like rotational stiffness, while the derivative term dissipates angular kinetic energy in the pitch/yaw modes. This directly attacks the Week 2 failure chain:

```text
disturbance moment -> angular rate -> attitude error -> thrust-axis loss
```

The ideal actuator allows the feedback law to be validated without the confounding effects of gimbal geometry. Because the commanded torque does not redirect the thrust force, attitude correction and translational acceleration are artificially decoupled. That is why Week 3A is a control-law verification step rather than an actuator-realistic result.

### Plot-Level Interpretation

Maintaining `body_z_z` near `+1` preserves `T cos(theta)` and suppresses `T sin(theta)`. The altitude trace remains increasing because vertical thrust projection is retained. Lateral drift is reduced because the controller prevents large horizontal thrust components and limits aerodynamic angle-of-attack growth.
"""


WEEK3B_PHYSICS = """
## Upper-Division TVC Physics

Week 3B replaces ideal torque with a finite thrust-vector-control actuator. Moment is generated by lateral thrust acting through the engine lever arm:

```text
tau_TVC = r_engine x F_thrust
```

For `r_engine = [0, 0, -L]`:

```text
tau_x = L F_y
tau_y = -L F_x
tau_max,TVC ~= L T sin(delta_max)
```

This converts the controls problem into an actuator allocation problem. A feedback law may request a stabilizing moment, but the plant can only supply that moment if the required lateral thrust lies inside the gimbal envelope.

## Control Comparison Plot Physics

### Altitude

Open loop loses altitude because attitude divergence reduces `T cos(theta)`. Ideal torque climbs highest because it supplies corrective moment without changing the thrust direction. TVC remains stable but can trail ideal torque because some thrust must be vectored laterally to generate moment.

### Unwrapped Pitch

Open-loop pitch is the integral of uncompensated disturbance angular rate. Ideal torque directly applies `Kp e - Kd omega`. TVC must realize the same corrective objective through `r_engine x F_thrust`, so pitch response includes both controller dynamics and actuator allocation constraints.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` is a propulsion-relevant attitude metric. Keeping it near `+1` means the controller is preserving vertical thrust authority. Letting it approach zero or negative means engine thrust is being converted into lateral or downward acceleration.

### Lateral Drift

Open-loop lateral drift grows through `T sin(theta)` and aerodynamic side force. Ideal torque minimizes drift because moment generation is decoupled from net force direction. TVC reduces drift relative to open loop but cannot eliminate the coupling: the same lateral thrust that stabilizes attitude also contributes to lateral acceleration.

### Gimbal Usage

The gimbal trace is an actuator-authority diagnostic. Early deflection arrests angular-rate growth; later deflection balances persistent thrust bias and aerodynamic moment. Saturation fraction indicates whether the requested moment remained within the finite control envelope.
"""


WEEK4A_PHYSICS = """
## Upper-Division LQR Physics

Near upright ascent, each transverse attitude channel can be locally approximated as:

```text
theta_dot = omega
omega_dot = tau / I
x = [theta, omega]^T
u = tau
```

LQR selects feedback gains by minimizing:

```text
J = integral(x^T Q x + u^T R u) dt
```

The attitude weight penalizes thrust-axis pointing error, the rate weight penalizes angular kinetic energy, and `R` penalizes torque demand. Since the torque request is allocated through TVC, `R` also indirectly shapes gimbal usage and lateral thrust demand.

This controller is local. It is designed about the upright operating point and is not a proof of global recovery from arbitrary tumble. The important verification step is that the LQR law is inserted back into the nonlinear plant with quaternion attitude, nonlinear thrust projection, aerodynamic CP/CM moments, thrust bias, finite gimbal authority, and saturation tracking.

## Comparison Physics

Open loop leaves `tau_dist` unopposed. Ideal torque performs best because moment and force direction are decoupled. PD TVC and LQR TVC are actuator-realistic because stabilizing moment requires lateral thrust. In the reference case, LQR TVC reduces tilt and lateral drift relative to PD TVC because its `Q/R` weighting damps the transverse rotational modes more efficiently while staying inside the same gimbal limit.

## Week 4A Comparison Plot Results

### Altitude

The altitude panel shows how controller quality maps into vertical impulse. Open loop loses vertical performance because `T_vertical = T cos(theta)` collapses as attitude error grows. Ideal torque ends highest because the model lets it apply moment without redirecting thrust. PD TVC and LQR TVC both pay a real actuator penalty: the lateral thrust used for moment slightly reduces axial thrust projection. LQR TVC ending closer to ideal torque means its lower attitude excursions reduce the time-integrated loss in `T cos(theta)`.

### Unwrapped Pitch

The unwrapped pitch result separates true rotational response from folded attitude display. Open loop is the uncontrolled integral of disturbance torque through `I omega_dot + omega x I omega = tau_dist`. The LQR trace remains bounded because the controller increases effective transverse stiffness and damping around the upright operating point. Its smaller excursion relative to PD TVC indicates that the selected `Q/R` weights damp the pitch mode with less attitude error accumulation.

### Body-Axis Vertical Component

`body_z_z = cos(theta)` is the propulsion-relevant attitude result. Open loop approaching zero or negative values means thrust is no longer aligned with ascent. LQR TVC keeping the minimum `body_z_z` near unity indicates that the nonlinear trajectory remains inside the small-angle region assumed by the linearized design. This is the main validity check for using LQR in the nonlinear simulation.

### Lateral Drift

The lateral drift result is the integrated cost of attitude error:

```text
m a_lateral ~= T sin(theta) + F_N,lateral
```

Ideal torque has the lowest drift because it does not require lateral thrust to generate moment. LQR TVC reducing drift relative to PD TVC means the LQR controller reduced both sideways thrust projection from attitude error and the aerodynamic side-force exposure associated with larger angle of attack.

### Gimbal Usage

The gimbal panel tests whether the LQR improvement is physically achievable. Since `tau_max,TVC ~= L T sin(delta_max)`, a controller that improves tracking only by saturating the gimbal would not be robust. In the Week 4A result, gimbal saturation remains zero, so the LQR improvement is attributable to feedback structure within the available actuator envelope rather than unrealistic command clipping.
"""


WEEK4B_PLOT_PHYSICS = """
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
"""


WEEK5_PHYSICS = """
## Upper-Division Avionics And Estimation Physics

Week 5 changes the closed-loop architecture from truth-state control to estimated-state control. This is a major GNC step because real flight software does not receive the true quaternion and true body angular velocity from the plant. It receives measurements corrupted by sensor bias, white noise, sampling, and reference-update limitations.

The gyro model is:

```text
omega_meas = omega_true + b_g + eta_g
```

The accelerometer model is specific force:

```text
f_B = R_IB(q)(a_I - g_I)
f_meas = f_B + b_a + eta_a
```

The specific-force definition is important. During powered ascent, an accelerometer does not measure gravity direction by itself. It measures non-gravitational acceleration, which is dominated by thrust and aerodynamic force. A naive "accelerometer points down" attitude correction would be physically invalid during high-thrust ascent because the sensed acceleration vector is largely aligned with thrust, not with gravity. The project therefore logs accelerometer channels as avionics measurements but uses gyro propagation plus a low-rate noisy attitude reference for attitude correction.

## Estimator Physics

The estimator propagates quaternion attitude from bias-corrected gyro rate:

```text
omega_hat = omega_meas - b_hat
q_hat_dot = 0.5 q_hat [0, omega_hat]
```

Gyro integration is high-bandwidth but drifts when bias is imperfect. The low-rate attitude reference bounds that drift by applying a small correction based on thrust-axis pointing error:

```text
e_I = z_hat,I x z_ref,I
e_B = R_IB(q_hat)e_I
```

This correction is intentionally applied to the thrust-axis direction because pitch/yaw alignment is the propulsion-relevant attitude quantity for vertical ascent. Roll about the thrust axis is less important for this simplified axisymmetric vehicle model.

## Estimated-State Control Physics

The LQR TVC controller is driven by:

```text
q_control = q_hat
omega_control = omega_hat
```

while the plant still evolves with the true nonlinear 6-DOF state. Estimation error therefore enters the feedback loop as a false attitude/rate command. If the estimate lags or drifts, the TVC controller can command the wrong moment, inject extra lateral thrust, or use up gimbal authority.

The Week 5 result remains close to the truth-state LQR case: final altitude is `31.42 m`, maximum tilt is `10.43 deg`, maximum lateral drift is `10.83 m`, maximum attitude estimation error is `0.32 deg`, RMS attitude estimation error is `0.17 deg`, and gimbal saturation is `0.0%`.

## Plot-Level Interpretation

### True vs Estimated Tilt

The true and estimated tilt traces stay nearly coincident. This means the estimator tracks the transverse thrust-axis attitude well enough for the controller to remain inside the same small-angle operating region assumed by the LQR design.

### Attitude Estimation Error

The attitude error remains sub-degree. This is small relative to the roughly `10 deg` controlled tilt envelope, so estimator error is not the dominant driver of the closed-loop response. The plot demonstrates that gyro propagation plus reference correction prevents bias-driven attitude drift over the simulated ascent window.

### Gyro Measurement

The gyro channels show the measured angular-rate signal that the controller indirectly depends on. Rate measurement matters because derivative damping and LQR rate feedback are both sensitive to angular-rate error. Bias-corrected gyro propagation keeps the estimator from interpreting a constant sensor bias as real vehicle rotation.

### Accelerometer Specific Force

The accelerometer channels are dominated by powered-flight specific force. This supports the modeling decision not to use accelerometer-only gravity leveling during ascent. In a rocket under thrust, accelerometer magnitude and direction reflect thrust and aerodynamic loading, not just vehicle attitude relative to gravity.

### Estimated-State TVC Usage

The gimbal trace verifies actuator feasibility under estimated-state feedback. If sensor noise caused aggressive false corrections, the gimbal angle or saturation fraction would increase. The result stays within the same actuator envelope as truth-state LQR, showing that the estimator does not destabilize or overdrive the TVC loop in the nominal disturbed case.

## Engineering Takeaway

Week 5 demonstrates the transition from controls-only simulation to avionics-aware GNC simulation. The controller is no longer granted perfect attitude knowledge; it must operate through a physically motivated measurement and estimation layer. The resulting closed-loop performance shows that the estimator error is small enough to preserve TVC stability and ascent performance for the modeled sensor noise/bias case.
"""


WEEK6_PHYSICS = """
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
"""


def main() -> None:
    week1_metrics = summary_metrics(load_samples(OUT / "week1_ascent.csv"), RocketParams(mass_kg=50.0, thrust_n=850.0), Environment())
    week2_rocket, week2_env = week2_reference()
    week2_metrics = summary_metrics(load_samples(OUT / "week2_disturbed_uncontrolled.csv"), week2_rocket, week2_env)
    week3a_metrics = summary_metrics(load_samples(OUT / "week3a_controlled_ideal_torque.csv"), week2_rocket, week2_env)
    week3b_metrics = summary_metrics(load_samples(OUT / "week3b_tvc_controlled.csv"), week2_rocket, week2_env)
    week4a_metrics = summary_metrics(load_samples(OUT / "week4a_lqr_tvc_controlled.csv"), week2_rocket, week2_env)
    week5_metrics = summary_metrics(load_samples(OUT / "week5_estimated_tvc_controlled.csv"), week2_rocket, week2_env)
    week6_metrics = summary_metrics(load_samples(OUT / "week6_lqr_tvc_actuator_limited.csv"), week2_rocket, week2_env)
    week6_estimated_metrics = summary_metrics(load_samples(OUT / "week6_estimated_lqr_tvc_actuator_limited.csv"), week2_rocket, week2_env)

    (OUT / "week1_milestone_report.md").write_text(
        "# Week 1 Milestone Report\n\n"
        "## Objective\n\n"
        "Verify the 13-state rigid-body dynamics core before adding aerodynamic disturbances or feedback control.\n\n"
        "## Baseline Case\n\n"
        + metrics_table(week1_metrics)
        + WEEK1_PHYSICS
    )
    (OUT / "week2_milestone_report.md").write_text(
        "# Week 2 Milestone Report\n\n"
        "## Objective\n\n"
        "Add wind, thrust misalignment, thrust offset, aerodynamic drag, normal force, and CP/CM moment to produce an open-loop failure case.\n\n"
        "## Disturbed Case\n\n"
        + metrics_table(week2_metrics)
        + WEEK2_PHYSICS
    )
    (OUT / "week3a_milestone_report.md").write_text(
        "# Week 3A Milestone Report\n\n"
        "## Objective\n\n"
        "Validate nonlinear thrust-axis attitude feedback using an ideal bounded body-torque actuator.\n\n"
        "## Controlled Case\n\n"
        + metrics_table(week3a_metrics)
        + WEEK3A_PHYSICS
    )
    (OUT / "week3b_milestone_report.md").write_text(
        "# Week 3B Milestone Report\n\n"
        "## Objective\n\n"
        "Replace ideal body torque with finite thrust-vector-control allocation through a gimbaled engine lever arm.\n\n"
        "## TVC Controlled Case\n\n"
        + metrics_table(week3b_metrics)
        + f"| Gimbal saturation fraction | {100.0 * saturation_fraction(OUT / 'week3b_tvc_controlled.csv'):.1f}% |\n"
        + WEEK3B_PHYSICS
    )
    (OUT / "week4a_milestone_report.md").write_text(
        "# Week 4A Milestone Report\n\n"
        "## Objective\n\n"
        "Design a small-angle LQR attitude law and verify it in the nonlinear 6-DOF plant through the same TVC actuator model.\n\n"
        "## LQR TVC Controlled Case\n\n"
        + metrics_table(week4a_metrics)
        + f"| Gimbal saturation fraction | {100.0 * saturation_fraction(OUT / 'week4a_lqr_tvc_controlled.csv'):.1f}% |\n"
        + WEEK4A_PHYSICS
    )
    est_metrics = estimator_metrics(OUT / "week5_estimated_tvc_controlled.csv")
    (OUT / "week5_milestone_report.md").write_text(
        "# Week 5 Milestone Report\n\n"
        "## Objective\n\n"
        "Add a sensor and attitude-estimation layer, then close the TVC control loop using estimated attitude and angular rate instead of truth-state feedback.\n\n"
        "## Estimated-State Controlled Case\n\n"
        + metrics_table(week5_metrics)
        + f"| Maximum attitude estimation error | {est_metrics['max_attitude_error_deg']:.2f} deg |\n"
        + f"| RMS attitude estimation error | {est_metrics['rms_attitude_error_deg']:.2f} deg |\n"
        + f"| Maximum angular-rate estimation error | {est_metrics['max_rate_error_radps']:.3f} rad/s |\n"
        + f"| Gimbal saturation fraction | {100.0 * est_metrics['saturation_fraction']:.1f}% |\n"
        + WEEK5_PHYSICS
    )
    act_metrics = actuator_metrics(OUT / "week6_lqr_tvc_actuator_limited.csv")
    est_act_metrics = actuator_metrics(OUT / "week6_estimated_lqr_tvc_actuator_limited.csv")
    (OUT / "week6_milestone_report.md").write_text(
        "# Week 6 Milestone Report\n\n"
        "## Objective\n\n"
        "Add finite TVC actuator dynamics, including first-order gimbal lag, slew-rate limit, hard position limit, commanded-vs-achieved gimbal tracking, and torque-authority margin diagnostics.\n\n"
        "## Truth-State Actuator-Limited LQR\n\n"
        + metrics_table(week6_metrics)
        + f"| Maximum gimbal lag error | {act_metrics['max_gimbal_lag_error_deg']:.2f} deg |\n"
        + f"| Maximum commanded gimbal | {act_metrics['max_commanded_gimbal_deg']:.2f} deg |\n"
        + f"| Maximum achieved gimbal | {act_metrics['max_achieved_gimbal_deg']:.2f} deg |\n"
        + f"| Minimum torque authority margin | {act_metrics['min_torque_authority_margin_nm']:.2f} N m |\n"
        + f"| Rate-limit fraction | {100.0 * act_metrics['rate_limit_fraction']:.1f}% |\n"
        + f"| Position-limit fraction | {100.0 * act_metrics['position_limit_fraction']:.1f}% |\n\n"
        "## Estimated-State Actuator-Limited LQR\n\n"
        + metrics_table(week6_estimated_metrics)
        + f"| Maximum gimbal lag error | {est_act_metrics['max_gimbal_lag_error_deg']:.2f} deg |\n"
        + f"| Maximum commanded gimbal | {est_act_metrics['max_commanded_gimbal_deg']:.2f} deg |\n"
        + f"| Maximum achieved gimbal | {est_act_metrics['max_achieved_gimbal_deg']:.2f} deg |\n"
        + f"| Minimum torque authority margin | {est_act_metrics['min_torque_authority_margin_nm']:.2f} N m |\n"
        + f"| Rate-limit fraction | {100.0 * est_act_metrics['rate_limit_fraction']:.1f}% |\n"
        + f"| Position-limit fraction | {100.0 * est_act_metrics['position_limit_fraction']:.1f}% |\n"
        + WEEK6_PHYSICS
    )

    mc_rows = load_monte_carlo(OUT / "week4b_monte_carlo_results.csv")
    mc_summary = monte_carlo_summary(mc_rows)
    (OUT / "week4b_milestone_report.md").write_text(
        "# Week 4B Milestone Report\n\n"
        "## Objective\n\n"
        "Evaluate robustness statistically by running randomized off-nominal vehicle and environment dispersions through open-loop, PD TVC, and LQR TVC cases.\n\n"
        "## Pass/Fail Criteria\n\n"
        "```text\n"
        "max tilt < 25 deg\n"
        "final altitude > 20 m\n"
        "max lateral drift < 25 m\n"
        "gimbal saturation fraction < 10%\n"
        "```\n\n"
        "## Results\n\n"
        + monte_carlo_table(mc_summary)
        + WEEK4B_PLOT_PHYSICS
        + "\n## Engineering Takeaway\n\n"
        "Nominal success is not robustness. Week 4B evaluates controller performance against dispersions in wind, mass properties, thrust alignment, aerodynamics, and actuator authority, then compares controllers using objective trajectory and actuator metrics.\n"
    )
    print("Wrote milestone reports")


if __name__ == "__main__":
    main()
