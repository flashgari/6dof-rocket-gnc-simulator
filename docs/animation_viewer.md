# Animation Viewer

The animation viewer is a standalone portfolio artifact generated from the simulation CSV outputs:

```text
outputs/rocket_flight_animation.html
```

It compares four cases on a synchronized time base:

- Week 2 open-loop disturbed ascent
- Week 3A ideal body-torque control
- Week 3B PD thrust-vector-control actuator model
- Week 4A LQR thrust-vector-control actuator model

## What The Animation Shows

Each lane displays the same coupled states: trajectory, body-axis attitude, body-axis vertical component, lateral displacement, and TVC gimbal usage. The purpose is to make the attitude-translation coupling visible:

```text
attitude error -> thrust-vector projection -> vertical/lateral acceleration
```

The lower plots are not decorative. `body_z_z` measures vertical thrust-axis alignment, lateral drift integrates horizontal acceleration, and gimbal angle indicates actuator authority being used to create moment.

## Why The Animation Ends This Way

The open-loop vehicle can appear partly upright at the final frame, but that is not recovery. The unwrapped pitch history shows that the body has rotated through more than one full revolution. During the earlier tumble, `body_z_z` passed through zero and negative values, meaning thrust was horizontal or downward:

```text
T_vertical = T body_z_z
```

Once that happened, the vehicle lost altitude authority and converted thrust into lateral kinetic energy. A later positive `body_z_z` simply means the body axis rotated around again; it does not erase the lost trajectory energy or crossrange displacement.

The ideal body-torque controller ends highest because it applies corrective moment directly:

```text
tau_cmd = Kp e - Kd omega
```

without changing the thrust-force direction. In the model, this decouples attitude correction from lateral force, so it preserves the largest vertical thrust projection and minimizes crossrange acceleration.

The PD TVC case is actuator-realistic. It stabilizes attitude by tilting the engine:

```text
tau_TVC = r_engine x F_thrust
```

That lateral thrust component produces the needed moment but also appears in the translational equation as lateral acceleration. The difference from ideal torque is therefore an actuator-coupling effect, not controller failure.

The LQR TVC lane adds the local optimal-control comparison. It is designed around the upright operating point, so it is not a global recovery controller for arbitrary tumbling states. In the reference case, it keeps the vehicle inside the small-angle region and uses its `Q/R` trade to reduce lateral drift relative to PD TVC while respecting the same gimbal authority.

## Regeneration

Run:

```bash
python3 scripts/run_all.py
```

This regenerates the simulations, plots, reports, tests, and animation HTML.
