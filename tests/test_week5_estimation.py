import math
import unittest

from rocket_sim import AttitudeEstimator, AttitudeEstimatorConfig, SensorModel
from rocket_sim.analysis import tilt_angle_deg
from rocket_sim.estimated_sim import simulate_estimated_tvc
from rocket_sim.sensors import attitude_error_deg
from scripts.run_week5_estimated_tvc_ascent import week5_setup


class Week5EstimationTests(unittest.TestCase):
    def test_sensor_sampler_is_deterministic(self):
        rocket, env, initial, _, sensors, _ = week5_setup()
        first = sensors.sampler().measure(0.0, initial, rocket, env)
        second = sensors.sampler().measure(0.0, initial, rocket, env)
        self.assertEqual(first.gyro_radps, second.gyro_radps)
        self.assertEqual(first.accelerometer_mps2, second.accelerometer_mps2)
        self.assertEqual(first.reference_attitude, second.reference_attitude)

    def test_attitude_error_metric_is_zero_for_identical_attitudes(self):
        self.assertAlmostEqual(attitude_error_deg((1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)), 0.0)

    def test_estimator_tracks_nominal_ascent_attitude(self):
        rocket, env, initial, tvc, sensors, estimator = week5_setup()
        samples = list(simulate_estimated_tvc(initial, rocket, env, tvc, sensors.sampler(), estimator, duration_s=3.0, dt_s=0.005))
        max_error = max(sample.attitude_error_deg for sample in samples)
        rms_error = math.sqrt(sum(sample.attitude_error_deg**2 for sample in samples) / len(samples))
        self.assertLess(max_error, 1.0)
        self.assertLess(rms_error, 0.5)

    def test_estimated_state_feedback_remains_actuator_feasible(self):
        rocket, env, initial, tvc, sensors, estimator = week5_setup()
        samples = list(simulate_estimated_tvc(initial, rocket, env, tvc, sensors.sampler(), estimator, duration_s=3.0, dt_s=0.005))
        saturation_fraction = sum(sample.command.saturated for sample in samples) / len(samples)
        max_tilt = max(tilt_angle_deg(sample.true_state) for sample in samples)
        self.assertEqual(saturation_fraction, 0.0)
        self.assertLess(max_tilt, 12.0)

    def test_estimator_rejects_bias_when_configured_with_bias_estimate(self):
        estimator = AttitudeEstimator(
            (1.0, 0.0, 0.0, 0.0),
            AttitudeEstimatorConfig(gyro_bias_estimate_radps=(0.01, -0.006, 0.002), attitude_reference_gain=0.0),
        )
        sensors = SensorModel(gyro_noise_std_radps=0.0, accel_noise_std_mps2=0.0, attitude_reference_period_s=10.0)
        rocket, env, initial, _, _, _ = week5_setup()
        measurement = sensors.sampler().measure(0.0, initial, rocket, env)
        estimate = estimator.update(measurement, 0.01)
        self.assertLess(abs(estimate.angular_velocity_radps[0]), 1.0e-12)
        self.assertLess(abs(estimate.angular_velocity_radps[1]), 1.0e-12)


if __name__ == "__main__":
    unittest.main()
