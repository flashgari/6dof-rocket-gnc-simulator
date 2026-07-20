import math
import unittest

from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import body_z_axis_inertial, signed_pitch_deg, tilt_angle_deg
from rocket_sim.dynamics import aerodynamic_normal_force_body, body_forces_and_torques
from rocket_sim.sim import simulate


class Week2DisturbanceTests(unittest.TestCase):
    def test_wind_generates_angle_of_attack_normal_force(self):
        rocket = RocketParams(thrust_n=0.0, reference_area_m2=0.05, normal_force_coefficient_per_rad=10.0)
        env = Environment(gravity_mps2=0.0, wind_mps=(5.0, 0.0, 0.0))
        state = State((0, 0, 0), (0, 0, 20), (1, 0, 0, 0), (0, 0, 0))

        force = aerodynamic_normal_force_body(state, rocket, env)

        self.assertGreater(force[0], 0.0)
        self.assertAlmostEqual(force[1], 0.0, places=12)

    def test_cp_above_cm_creates_destabilizing_pitch_moment(self):
        rocket = RocketParams(
            thrust_n=0.0,
            reference_area_m2=0.05,
            normal_force_coefficient_per_rad=10.0,
            center_of_pressure_body_m=(0.0, 0.0, 0.35),
        )
        env = Environment(gravity_mps2=0.0, wind_mps=(5.0, 0.0, 0.0))
        state = State((0, 0, 0), (0, 0, 20), (1, 0, 0, 0), (0, 0, 0))

        _, torque = body_forces_and_torques(state, rocket, env)

        self.assertGreater(torque[1], 0.0)

    def test_disturbed_uncontrolled_vehicle_tilts_open_loop(self):
        misalign = math.radians(1.5)
        rocket = RocketParams(
            thrust_n=850.0,
            reference_area_m2=0.045,
            drag_coefficient=0.35,
            normal_force_coefficient_per_rad=2.5,
            center_of_pressure_body_m=(0.0, 0.0, 0.35),
            thrust_offset_body_m=(0.004, 0.0, 0.0),
            thrust_direction_body=(math.sin(misalign), 0.0, math.cos(misalign)),
        )
        env = Environment(wind_mps=(4.0, 1.0, 0.0))
        initial = State((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))

        final = list(simulate(initial, rocket, env, duration_s=4.0, dt_s=0.005))[-1][1]

        self.assertGreater(tilt_angle_deg(final), 5.0)

    def test_signed_pitch_exposes_direction_while_tilt_is_unsigned(self):
        q_pitch_120_about_y = (math.cos(math.radians(60.0)), 0.0, math.sin(math.radians(60.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_120_about_y, (0, 0, 0))

        self.assertAlmostEqual(tilt_angle_deg(state), 120.0, places=12)
        self.assertAlmostEqual(signed_pitch_deg(state), 120.0, places=12)
        self.assertLess(body_z_axis_inertial(state)[2], 0.0)


if __name__ == "__main__":
    unittest.main()
