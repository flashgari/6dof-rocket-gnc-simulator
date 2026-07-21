import unittest

from rocket_sim.propulsion import MassPropertySchedule, ThrustCurve
from rocket_sim.variable_mass_sim import simulate_variable_mass_actuator_limited_tvc
from rocket_sim import GimbalActuator
from rocket_sim.analysis import tilt_angle_deg
from scripts.run_week6_actuator_limited_tvc import week6_actuator_config
from scripts.run_week7_variable_mass_ascent import week7_vehicle_setup


class Week7VariableMassTests(unittest.TestCase):
    def test_thrust_curve_interpolates_piecewise_linearly(self):
        curve = ThrustCurve(((0.0, 100.0), (1.0, 200.0), (2.0, 150.0)))
        self.assertAlmostEqual(curve.thrust_n(0.5), 150.0)
        self.assertAlmostEqual(curve.thrust_n(1.5), 175.0)
        self.assertAlmostEqual(curve.thrust_n(3.0), 150.0)

    def test_propellant_depletion_uses_impulse_over_isp_g0(self):
        curve = ThrustCurve(((0.0, 100.0), (10.0, 100.0)))
        schedule = MassPropertySchedule(
            initial_mass_kg=10.0,
            dry_mass_kg=8.0,
            initial_inertia_kg_m2=(2.0, 2.0, 1.0),
            dry_inertia_kg_m2=(1.0, 1.0, 0.5),
            initial_center_of_mass_body_m=(0.0, 0.0, -0.1),
            dry_center_of_mass_body_m=(0.0, 0.0, 0.1),
            isp_s=100.0,
            thrust_curve=curve,
            gravity_mps2=10.0,
        )
        self.assertAlmostEqual(schedule.propellant_used_kg(5.0), 0.5)
        self.assertAlmostEqual(schedule.mass_kg(5.0), 9.5)

    def test_mass_inertia_and_cm_move_toward_dry_values(self):
        vehicle, _, _, _ = week7_vehicle_setup()
        initial = vehicle.at(0.0)
        final = vehicle.at(3.0)
        self.assertLess(final.mass_kg, initial.mass_kg)
        self.assertLess(final.inertia_kg_m2[0], initial.inertia_kg_m2[0])
        self.assertGreater(final.center_of_mass_body_m[2], initial.center_of_mass_body_m[2])

    def test_variable_mass_actuator_limited_lqr_stays_stable(self):
        vehicle, env, initial, tvc = week7_vehicle_setup()
        samples = list(
            simulate_variable_mass_actuator_limited_tvc(
                initial,
                vehicle,
                env,
                tvc,
                GimbalActuator(week6_actuator_config()),
                duration_s=3.0,
                dt_s=0.005,
            )
        )
        max_tilt = max(tilt_angle_deg(sample.state) for sample in samples)
        self.assertLess(max_tilt, 16.0)
        self.assertLess(samples[-1].mass_kg, samples[0].mass_kg)
        self.assertGreater(samples[-1].state.position_m[2], 30.0)


if __name__ == "__main__":
    unittest.main()
