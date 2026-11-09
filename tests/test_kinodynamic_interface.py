"""Regression checks for kinodynamic references and whole-body targets (no solver build)."""
import unittest
from unittest.mock import patch

import mujoco
import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg
from quadruped_pympc.interfaces.wb_interface import WBInterface
from quadruped_pympc.helpers.quadruped_utils import GaitType

LEGS = ('FL', 'FR', 'RL', 'RR')


def legs(factory):
    return LegsAttr(**{leg: factory(i) for i, leg in enumerate(LEGS)})


class KinodynamicInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.config_patch = patch.dict(cfg.mpc_params, type='kinodynamic', optimize_step_freq=False)
        self.config_patch.start()
        self.addCleanup(self.config_patch.stop)
        self.wb = WBInterface(legs(lambda _: np.zeros(3)))
        self.env = self.wb.ik.env
        self.addCleanup(self.env.close)
        mujoco.mj_resetDataKeyframe(self.env.mjModel, self.env.mjData, 0)
        mujoco.mj_forward(self.env.mjModel, self.env.mjData)
        self.feet = self.env.feet_pos(frame='world')
        self.wb.frg.lift_off_positions = self.feet
        self.wb.frg.touch_down_positions = self.feet
        self.wb.esd.activated = False

    def update_references(self):
        env = self.env
        return self.wb.update_state_and_reference(
            env.com, env.base_pos, env.base_lin_vel(frame='world'),
            env.base_ori_euler_xyz, env.base_ang_vel(frame='base'), self.feet,
            env.hip_positions(frame='world'),
            legs(lambda i: env.mjData.qpos[env.legs_qpos_idx[LEGS[i]]]),
            None, LEGS, 0.002, np.zeros(3), np.zeros(3),
        )

    def test_reference_is_defined_for_stance_and_swing(self):
        for gait in (GaitType.FULL_STANCE.value, GaitType.TROT.value):
            with self.subTest(gait=gait):
                self.wb.pgg.gait_type = gait
                self.wb.pgg.reset()
                _, reference, contacts, _, _ = self.update_references()
                self.assertEqual(reference['ref_joints'].shape, (12,))
                for leg in LEGS:
                    trajectory = reference[f'ref_foot_swing_{leg}']
                    self.assertEqual(trajectory.shape, (cfg.mpc_params['horizon'], 3))
                    self.assertTrue(np.isfinite(trajectory).all())
                    self.assertEqual(reference[f'ref_foot_{leg}'].shape, (1, 3))
                    if gait == GaitType.FULL_STANCE.value:
                        assert_allclose(trajectory, np.tile(self.feet[leg], (len(trajectory), 1)))

    def test_swing_clock_resets_and_prediction_does_not_mutate_it(self):
        self.wb.stc.swing_time = [0.07] * 4
        self.wb.contact_sequence_dts = [0.01, 0.03]
        self.wb.contact_sequence_lenghts = [2, 6]
        contacts = np.tile([1, 0, 0, 1, 0, 0], (4, 1))
        targets = legs(lambda i: self.feet[LEGS[i]] + [0.1, 0, 0])
        with patch.dict(cfg.mpc_params, horizon=6):
            result = self.wb._kinodynamic_foot_references(self.feet, targets, contacts)
        self.assertEqual(self.wb.stc.swing_time, [0.07] * 4)
        for leg in LEGS:
            path = result[f'ref_foot_swing_{leg}']
            assert_allclose(path[0], self.feet[leg])
            assert_allclose(path[1], self.feet[leg])
            expected, _, _ = self.wb.stc.swing_generator.compute_trajectory_references(
                0.03, self.feet[leg], targets[leg]
            )
            assert_allclose(path[2], expected)
            assert_allclose(path[3], targets[leg])
            assert_allclose(path[4], targets[leg])

    def test_joint_targets_are_separate_flat_and_do_not_modify_mpc_output(self):
        env, wb = self.env, self.wb
        wb.current_contact = np.array([0, 1, 1, 0])
        wb.kinodynamic_joint_space_swing = True
        qpos, qvel = env.mjData.qpos, env.mjData.qvel
        positions = legs(lambda i: qpos[env.legs_qpos_idx[LEGS[i]]] + [0.1, 4., -4.])
        velocities = legs(lambda _: np.array([0.2, 12., -12.]))
        original_positions = legs(lambda i: positions[LEGS[i]].copy())
        original_velocities = legs(lambda i: velocities[LEGS[i]].copy())
        with patch.object(wb.stc, 'compute_swing_control_joint_space', wraps=wb.stc.compute_swing_control_joint_space) as joint_control:
            with patch.object(wb.stc, 'compute_swing_control_cartesian_space', side_effect=AssertionError('Cartesian control used')):
                _, desired_pos, desired_vel = wb.compute_stance_and_swing_torque(
                    0.002, qpos, qvel, env.feet_jacobians(), env.feet_jacobians_dot(),
                    self.feet, env.feet_vel(), env.legs_qfrc_passive, env.legs_qfrc_bias,
                    env.legs_mass_matrix, legs(lambda _: np.zeros(3)), self.feet,
                    env.legs_qpos_idx, env.legs_qvel_idx, legs(lambda _: np.zeros(3)),
                    0, wb.pgg.step_freq, positions, velocities, legs(lambda _: np.zeros(3)), np.zeros(24),
                )
        self.assertEqual(joint_control.call_count, 2)
        for leg in LEGS:
            self.assertEqual(desired_pos[leg].shape, (3,))
            self.assertEqual(desired_vel[leg].shape, (3,))
            assert_allclose(desired_pos[leg], qpos[env.legs_qpos_idx[leg]] + [0.1, 3., -3.])
            assert_allclose(desired_vel[leg], [0.2, 10., -10.])
            assert_array_equal(positions[leg], original_positions[leg])
            assert_array_equal(velocities[leg], original_velocities[leg])


if __name__ == '__main__':
    unittest.main()
