import math
import unittest

from rocket_sim import Environment, IdealTorqueController, RocketParams, State
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, tilt_angle_deg
from rocket_sim.controlled_sim import simulate_controlled
from rocket_sim.sim import simulate


def disturbed_setup():
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
    env = Environment(wind_mps=(4.0, 1.0, 0.0))
    initial = State((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    return rocket, env, initial


class Week3AIdealTorqueControlTests(unittest.TestCase):
    def test_controller_commands_restoring_pitch_torque(self):
        controller = IdealTorqueController(kp_nmpu=10.0, kd_nms=1.0, max_torque_nm=50.0)
        q_pitch_20_about_y = (math.cos(math.radians(10.0)), 0.0, math.sin(math.radians(10.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_20_about_y, (0, 0, 0))

        torque = controller.torque_body(state)

        self.assertLess(torque[1], 0.0)

    def test_controller_saturates_torque_norm(self):
        controller = IdealTorqueController(kp_nmpu=100.0, kd_nms=0.0, max_torque_nm=3.0)
        q_pitch_90_about_y = (math.cos(math.radians(45.0)), 0.0, math.sin(math.radians(45.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_90_about_y, (0, 0, 0))

        torque = controller.torque_body(state)
        torque_norm = math.sqrt(sum(component * component for component in torque))

        self.assertAlmostEqual(torque_norm, 3.0, places=12)

    def test_controlled_case_reduces_open_loop_failure(self):
        rocket, env, initial = disturbed_setup()
        controller = IdealTorqueController(kp_nmpu=18.0, kd_nms=8.0, max_torque_nm=40.0)

        open_loop = list(simulate(initial, rocket, env, duration_s=3.0, dt_s=0.005))
        controlled = list(simulate_controlled(initial, rocket, env, controller, duration_s=3.0, dt_s=0.005))
        open_final = open_loop[-1][1]
        controlled_final = controlled[-1][1]

        self.assertGreater(body_z_axis_inertial(controlled_final)[2], body_z_axis_inertial(open_final)[2])
        self.assertLess(tilt_angle_deg(controlled_final), tilt_angle_deg(open_final))
        self.assertLess(lateral_displacement_m(controlled_final), lateral_displacement_m(open_final))


if __name__ == "__main__":
    unittest.main()
