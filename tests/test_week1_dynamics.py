import math
import unittest

from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import angular_momentum_inertial_kg_m2_radps, linear_momentum_kg_mps
from rocket_sim.dynamics import derivatives
from rocket_sim.math3d import q_norm, rotate_body_to_inertial
from rocket_sim.sim import simulate


class Week1DynamicsTests(unittest.TestCase):
    def test_straight_ascent_matches_constant_acceleration(self):
        rocket = RocketParams(mass_kg=50.0, thrust_n=850.0, drag_coefficient=0.0)
        env = Environment()
        initial = State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        samples = list(simulate(initial, rocket, env, duration_s=5.0, dt_s=0.01))
        _, final = samples[-1]

        acceleration = rocket.thrust_n / rocket.mass_kg - env.gravity_mps2
        self.assertAlmostEqual(final.position_m[0], 0.0, places=9)
        self.assertAlmostEqual(final.position_m[1], 0.0, places=9)
        self.assertAlmostEqual(final.position_m[2], 0.5 * acceleration * 5.0**2, places=5)
        self.assertAlmostEqual(final.velocity_mps[2], acceleration * 5.0, places=5)
        self.assertAlmostEqual(final.angular_velocity_radps[0], 0.0, places=9)
        self.assertAlmostEqual(final.angular_velocity_radps[1], 0.0, places=9)
        self.assertAlmostEqual(final.angular_velocity_radps[2], 0.0, places=9)

    def test_quaternion_stays_normalized(self):
        rocket = RocketParams(mass_kg=50.0, thrust_n=850.0)
        env = Environment()
        initial = State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.2, 0.0))

        for _, state in simulate(initial, rocket, env, duration_s=8.0, dt_s=0.01):
            self.assertAlmostEqual(q_norm(state.attitude), 1.0, places=12)

    def test_ballistic_energy_is_conserved_without_thrust_or_drag(self):
        rocket = RocketParams(mass_kg=5.0, thrust_n=0.0, drag_coefficient=0.0)
        env = Environment()
        initial = State((0.0, 0.0, 100.0), (15.0, -2.0, 25.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        def specific_energy(state: State) -> float:
            v2 = sum(v * v for v in state.velocity_mps)
            return 0.5 * v2 + env.gravity_mps2 * state.position_m[2]

        e0 = specific_energy(initial)
        samples = list(simulate(initial, rocket, env, duration_s=3.0, dt_s=0.005))
        ef = specific_energy(samples[-1][1])

        self.assertTrue(math.isclose(e0, ef, rel_tol=0.0, abs_tol=1e-8))

    def test_linear_momentum_is_conserved_when_force_free(self):
        rocket = RocketParams(mass_kg=12.0, thrust_n=0.0, drag_coefficient=0.0)
        env = Environment(gravity_mps2=0.0)
        initial = State(
            (3.0, -4.0, 10.0),
            (11.0, -7.0, 2.5),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

        p0 = linear_momentum_kg_mps(initial, rocket)
        samples = list(simulate(initial, rocket, env, duration_s=4.0, dt_s=0.01))
        pf = linear_momentum_kg_mps(samples[-1][1], rocket)

        for initial_component, final_component in zip(p0, pf):
            self.assertAlmostEqual(initial_component, final_component, places=10)

    def test_axisymmetric_angular_momentum_is_conserved_when_torque_free(self):
        rocket = RocketParams(mass_kg=20.0, thrust_n=0.0, drag_coefficient=0.0, inertia_kg_m2=(3.0, 3.0, 0.45))
        env = Environment(gravity_mps2=0.0)
        initial = State(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.3, -0.2, 0.7),
        )

        h0 = angular_momentum_inertial_kg_m2_radps(initial, rocket)
        samples = list(simulate(initial, rocket, env, duration_s=4.0, dt_s=0.005))
        hf = angular_momentum_inertial_kg_m2_radps(samples[-1][1], rocket)

        for initial_component, final_component in zip(h0, hf):
            self.assertAlmostEqual(initial_component, final_component, places=9)

    def test_thrust_offset_produces_expected_angular_acceleration(self):
        rocket = RocketParams(
            mass_kg=50.0,
            inertia_kg_m2=(3.0, 4.0, 0.45),
            thrust_n=100.0,
            thrust_offset_body_m=(0.2, 0.0, 0.0),
            thrust_direction_body=(0.0, 0.0, 1.0),
        )
        env = Environment()
        state = State((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        state_dot = derivatives(0.0, state, rocket, env)

        # r x F = [0.2, 0, 0] x [0, 0, 100] = [0, -20, 0] N*m.
        self.assertAlmostEqual(state_dot[10], 0.0, places=12)
        self.assertAlmostEqual(state_dot[11], -20.0 / 4.0, places=12)
        self.assertAlmostEqual(state_dot[12], 0.0, places=12)

    def test_quaternion_rotates_body_axis_into_inertial_frame(self):
        half_angle = math.radians(45.0)
        q_pitch_90_about_y = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)

        rotated_axis = rotate_body_to_inertial(q_pitch_90_about_y, (0.0, 0.0, 1.0))

        self.assertAlmostEqual(rotated_axis[0], 1.0, places=12)
        self.assertAlmostEqual(rotated_axis[1], 0.0, places=12)
        self.assertAlmostEqual(rotated_axis[2], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
