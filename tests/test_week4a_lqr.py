import math
import unittest

from rocket_sim import LQRAttitudeController, RocketParams, State
from rocket_sim.analysis import lateral_displacement_m, tilt_angle_deg
from rocket_sim.tvc_sim import simulate_tvc
from scripts.run_week3b_tvc_ascent import week3b_setup
from scripts.run_week4a_lqr_tvc_ascent import week4a_setup


class Week4ALQRTests(unittest.TestCase):
    def test_lqr_double_integrator_gains_match_analytic_solution(self):
        controller = LQRAttitudeController(
            q_angle=22500.0,
            q_rate=120.0,
            r_control=1.0,
            inertia_kg_m2=(3.0, 3.0, 0.45),
            max_torque_nm=80.0,
        )

        k_angle, k_rate = controller.gains_for_axis(3.0)

        self.assertAlmostEqual(k_angle, 150.0)
        self.assertAlmostEqual(k_rate, math.sqrt(1020.0))

    def test_lqr_restoring_torque_sign(self):
        rocket = RocketParams(inertia_kg_m2=(3.0, 3.0, 0.45))
        controller = LQRAttitudeController(
            q_angle=22500.0,
            q_rate=120.0,
            r_control=1.0,
            inertia_kg_m2=rocket.inertia_kg_m2,
            max_torque_nm=80.0,
        )
        q_pitch_20_about_y = (math.cos(math.radians(10.0)), 0.0, math.sin(math.radians(10.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_20_about_y, (0, 0, 0))

        torque = controller.torque_body(state)

        self.assertLess(torque[1], 0.0)

    def test_lqr_tvc_improves_pd_tvc_lateral_drift_in_reference_case(self):
        pd_rocket, pd_env, pd_initial, pd_tvc = week3b_setup()
        lqr_rocket, lqr_env, lqr_initial, lqr_tvc = week4a_setup()

        pd_samples = list(simulate_tvc(pd_initial, pd_rocket, pd_env, pd_tvc, duration_s=3.0, dt_s=0.005))
        lqr_samples = list(simulate_tvc(lqr_initial, lqr_rocket, lqr_env, lqr_tvc, duration_s=3.0, dt_s=0.005))
        pd_final = pd_samples[-1][1]
        lqr_final = lqr_samples[-1][1]

        self.assertLess(tilt_angle_deg(lqr_final), 12.0)
        self.assertLess(lateral_displacement_m(lqr_final), lateral_displacement_m(pd_final))
        self.assertGreater(lqr_final.position_m[2], pd_final.position_m[2])


if __name__ == "__main__":
    unittest.main()
