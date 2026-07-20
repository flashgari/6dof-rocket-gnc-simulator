import math
import unittest

from rocket_sim import Environment, IdealTorqueController, RocketParams, State, TVCController
from rocket_sim.analysis import body_z_axis_inertial, lateral_displacement_m, tilt_angle_deg
from rocket_sim.control import TVCCommand
from rocket_sim.dynamics import body_forces_and_torques
from rocket_sim.tvc_sim import rocket_with_tvc_command, simulate_tvc
from rocket_sim.sim import simulate


def tvc_setup():
    misalignment = math.radians(1.5)
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=2.5,
        center_of_pressure_body_m=(0.0, 0.0, 0.35),
    )
    env = Environment(wind_mps=(4.0, 1.0, 0.0))
    initial = State((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    controller = TVCController(
        ideal_controller=IdealTorqueController(kp_nmpu=120.0, kd_nms=28.0, max_torque_nm=80.0),
        engine_position_body_m=(0.0, 0.0, -1.2),
        max_gimbal_rad=math.radians(5.0),
        thrust_misalignment_body=(math.sin(misalignment), 0.0, 0.0),
    )
    return rocket, env, initial, controller


def week2_open_loop_setup():
    misalignment = math.radians(1.5)
    rocket = RocketParams(
        mass_kg=50.0,
        inertia_kg_m2=(3.0, 3.0, 0.45),
        thrust_n=850.0,
        reference_area_m2=0.045,
        drag_coefficient=0.35,
        normal_force_coefficient_per_rad=2.5,
        center_of_pressure_body_m=(0.0, 0.0, 0.35),
        thrust_offset_body_m=(0.004, 0.0, 0.0),
        thrust_direction_body=(math.sin(misalignment), 0.0, math.cos(misalignment)),
    )
    env = Environment(wind_mps=(4.0, 1.0, 0.0))
    initial = State((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    return rocket, env, initial


class Week3BTvcTests(unittest.TestCase):
    def test_tvc_allocator_restoring_torque_sign(self):
        rocket, _, _, controller = tvc_setup()
        q_pitch_20_about_y = (math.cos(math.radians(10.0)), 0.0, math.sin(math.radians(10.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_20_about_y, (0, 0, 0))

        command = controller.command(state, rocket)

        self.assertLess(command.requested_torque_body[1], 0.0)
        self.assertLess(command.achievable_torque_body[1], 0.0)

    def test_tvc_command_respects_gimbal_limit(self):
        rocket = RocketParams(thrust_n=850.0)
        controller = TVCController(
            ideal_controller=IdealTorqueController(kp_nmpu=500.0, kd_nms=0.0, max_torque_nm=500.0),
            max_gimbal_rad=math.radians(5.0),
        )
        q_pitch_90_about_y = (math.cos(math.radians(45.0)), 0.0, math.sin(math.radians(45.0)), 0.0)
        state = State((0, 0, 0), (0, 0, 0), q_pitch_90_about_y, (0, 0, 0))

        command = controller.command(state, rocket)
        total_gimbal = math.sqrt(command.gimbal_x_rad**2 + command.gimbal_y_rad**2)

        self.assertTrue(command.saturated)
        self.assertLessEqual(total_gimbal, math.radians(5.0) + 1e-12)

    def test_tvc_thrust_direction_creates_r_cross_f_torque(self):
        rocket = RocketParams(thrust_n=850.0)
        controller = TVCController(ideal_controller=IdealTorqueController(0.0, 0.0, 0.0))
        command = TVCCommand(
            thrust_direction_body=(math.sin(math.radians(3.0)), 0.0, math.cos(math.radians(3.0))),
            gimbal_x_rad=math.radians(3.0),
            gimbal_y_rad=0.0,
            requested_torque_body=(0.0, 0.0, 0.0),
            achievable_torque_body=(0.0, 0.0, 0.0),
            saturated=False,
        )
        actuated = rocket_with_tvc_command(rocket, command, controller)
        _, torque = body_forces_and_torques(State((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0)), actuated, Environment())

        self.assertLess(torque[1], 0.0)

    def test_tvc_control_improves_open_loop_failure(self):
        rocket, env, initial, controller = tvc_setup()
        open_rocket, open_env, open_initial = week2_open_loop_setup()

        open_loop = list(simulate(open_initial, open_rocket, open_env, duration_s=3.0, dt_s=0.005))
        tvc = list(simulate_tvc(initial, rocket, env, controller, duration_s=3.0, dt_s=0.005))
        open_final = open_loop[-1][1]
        tvc_final = tvc[-1][1]

        self.assertGreater(body_z_axis_inertial(tvc_final)[2], body_z_axis_inertial(open_final)[2])
        self.assertLess(tilt_angle_deg(tvc_final), tilt_angle_deg(open_final))
        self.assertLess(lateral_displacement_m(tvc_final), lateral_displacement_m(open_final))
        self.assertGreater(tvc_final.position_m[2], open_final.position_m[2])


if __name__ == "__main__":
    unittest.main()
