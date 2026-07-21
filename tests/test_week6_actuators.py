import math
import unittest

from rocket_sim import GimbalActuator, GimbalActuatorConfig
from rocket_sim.actuator_sim import simulate_actuator_limited_tvc, simulate_estimated_actuator_limited_tvc
from rocket_sim.actuators import max_tvc_torque_nm
from rocket_sim.analysis import tilt_angle_deg
from scripts.run_week4a_lqr_tvc_ascent import week4a_setup
from scripts.run_week6_actuator_limited_tvc import week6_actuator_config, week6_estimator, week6_sensor_model


class Week6ActuatorTests(unittest.TestCase):
    def test_actuator_rate_limit_bounds_step_response(self):
        rocket, _, initial, tvc = week4a_setup()
        actuator = GimbalActuator(GimbalActuatorConfig(math.radians(5.0), math.radians(10.0), 0.01))
        requested = tvc.command(initial, rocket)
        output = actuator.update(requested, rocket, tvc, 0.1)
        self.assertLessEqual(output.state.total_gimbal_rad, math.radians(1.0) + 1.0e-12)
        self.assertTrue(output.rate_limited)

    def test_actuator_position_limit_bounds_large_command(self):
        rocket, _, initial, tvc = week4a_setup()
        actuator = GimbalActuator(GimbalActuatorConfig(math.radians(2.0), math.radians(1000.0), 0.001))
        requested = tvc.command(initial, rocket)
        output = actuator.update(requested, rocket, tvc, 0.1)
        self.assertLessEqual(output.state.total_gimbal_rad, math.radians(2.0) + 1.0e-12)

    def test_max_tvc_torque_matches_lever_arm_estimate(self):
        rocket, _, _, tvc = week4a_setup()
        max_torque = max_tvc_torque_nm(rocket, tvc, math.radians(5.0))
        self.assertAlmostEqual(max_torque, 1.2 * 850.0 * math.sin(math.radians(5.0)))

    def test_actuator_limited_lqr_stays_stable(self):
        rocket, env, initial, tvc = week4a_setup()
        samples = list(
            simulate_actuator_limited_tvc(
                initial,
                rocket,
                env,
                tvc,
                GimbalActuator(week6_actuator_config()),
                duration_s=3.0,
                dt_s=0.005,
            )
        )
        max_tilt = max(tilt_angle_deg(sample.true_state) for sample in samples)
        max_lag = max(math.degrees(sample.actuator_output.command_error_rad) for sample in samples)
        self.assertLess(max_tilt, 16.0)
        self.assertGreater(max_lag, 0.1)

    def test_estimated_actuator_limited_lqr_stays_stable(self):
        rocket, env, initial, tvc = week4a_setup()
        samples = list(
            simulate_estimated_actuator_limited_tvc(
                initial,
                rocket,
                env,
                tvc,
                GimbalActuator(week6_actuator_config()),
                week6_sensor_model().sampler(),
                week6_estimator(initial.attitude),
                duration_s=3.0,
                dt_s=0.005,
            )
        )
        max_tilt = max(tilt_angle_deg(sample.true_state) for sample in samples)
        max_estimation_error = max(sample.attitude_error_deg for sample in samples)
        self.assertLess(max_tilt, 16.0)
        self.assertLess(max_estimation_error, 1.0)


if __name__ == "__main__":
    unittest.main()
