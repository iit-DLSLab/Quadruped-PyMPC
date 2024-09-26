from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.interfaces.wb_interface import WBInterface

from gym_quadruped.utils.quadruped_utils import LegsAttr
# from quadruped_pympc import config as cfg  # TODO: bad modularity

import numpy as np

_DEFAULT_OBS = ('ref_base_height', 'ref_base_angles', 'nmpc_GRFs', 'nmpc_footholds', 'swing_time')


class QuadrupedPyMPC_Wrapper:
    """A simple class wrapper of all the mpc submodules (swing, contact generator, mpc itself)."""

    def __init__(self,
                 mpc_cfg,
                 sim_cfg,
                 initial_feet_pos: LegsAttr,
                 legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR'),
                 mpc_observables_names: tuple[str, ...] = _DEFAULT_OBS):
        """ Constructor of the QuadrupedPyMPC_Wrapper class.

        Args:
            initial_feet_pos (LegsAttr): initial feet positions, otherwise they will be all zero.
            legs_order (tuple[str, str, str, str], optional): order of the leg. Defaults to ('FL', 'FR', 'RL', 'RR').
            mpc_observables_names (tuple[str, ...], optional): list of observable to save. Defaults to
            _DEFAULT_OBS.
        """
        self._mpc_cfg = mpc_cfg
        self._sim_cfg = sim_cfg

        self.mpc_frequency = self._sim_cfg['mpc_frequency']

        self.srbd_controller_interface = SRBDControllerInterface(mpc_cfg=self._mpc_cfg)

        if self._mpc_cfg['type'] != 'sampling' and self._mpc_cfg['optimize_step_freq']:
            self.srbd_batched_controller_interface = SRBDBatchedControllerInterface(mpc_cfg=self._mpc_cfg)

        self.wb_interface = WBInterface(mpc_cfg=self._mpc_cfg,
                                        sim_cfg=self._sim_cfg,
                                        initial_feet_pos=initial_feet_pos(frame='world'),
                                        legs_order=legs_order)

        self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.best_sample_freq = self.wb_interface.pgg.step_freq

        self.quadrupedpympc_observables_names = mpc_observables_names
        self.quadrupedpympc_observables = {}

    def compute_actions(self,
                        base_pos: np.ndarray,
                        base_lin_vel: np.ndarray,
                        base_ori_euler_xyz: np.ndarray,
                        base_ang_vel: np.ndarray,
                        feet_pos: LegsAttr,
                        hip_pos: LegsAttr,
                        heightmaps,
                        legs_order: tuple[str, str, str, str],
                        simulation_dt: float,
                        ref_base_lin_vel: np.ndarray,
                        ref_base_ang_vel: np.ndarray,
                        step_num: int,
                        qvel: np.ndarray,
                        feet_jac: LegsAttr,
                        jac_feet_dot: LegsAttr,
                        feet_vel: LegsAttr,
                        legs_qfrc_bias: LegsAttr,
                        legs_mass_matrix: LegsAttr,
                        legs_qvel_idx: LegsAttr,
                        tau: LegsAttr,
                        inertia: np.ndarray) -> LegsAttr:
        """ Given the current state of the robot (and the reference), 
            compute the torques to be applied to the motors.

        Args:
            base_pos (np.ndarray): base position in world frame
            base_lin_vel (np.ndarray): base velocity in world frame
            base_ori_euler_xyz (np.ndarray): base orientation in world frame
            base_ang_vel (np.ndarray): base angular velocity in base frame
            feet_pos (LegsAttr): locations of the feet in world frame
            hip_pos (LegsAttr): locations of the hip in world frame
            heightmaps (_type_): TODO
            legs_order (tuple[str, str, str, str]): order of the legs
            simulation_dt (float): simulation time step
            ref_base_lin_vel (np.ndarray): reference base linear velocity in world frame
            ref_base_ang_vel (np.ndarray): reference base angular velocity in world frame
            step_num (int): current step number of the environment
            qvel (np.ndarray): joint velocities
            feet_jac (LegsAttr): jacobian of the feet
            jac_feet_dot (LegsAttr): time derivative of the jacobian of the feet
            feet_vel (LegsAttr): velocity of the feet
            legs_qfrc_bias (LegsAttr): gravity compensation, coriolis and centrifugal forces
            legs_mass_matrix (LegsAttr): mass matrix of the legs
            legs_qvel_idx (LegsAttr): indices of the joint velocities
            tau (LegsAttr): joint torques
            inertia (np.ndarray): inertia matrix of the robot (CCRBI)

        Returns:
            LegsAttr: torques to be applied to the motors
        """

        # Update the state and reference -------------------------
        state_current, \
            ref_state, \
            contact_sequence, \
            ref_feet_pos, \
            ref_feet_constraints, \
            contact_sequence_dts, \
            contact_sequence_lenghts, \
            step_height, \
            optimize_swing = self.wb_interface.update_state_and_reference(base_pos,
                                                                          base_lin_vel,
                                                                          base_ori_euler_xyz,
                                                                          base_ang_vel,
                                                                          feet_pos,
                                                                          hip_pos,
                                                                          heightmaps,
                                                                          legs_order,
                                                                          simulation_dt,
                                                                          ref_base_lin_vel,
                                                                          ref_base_ang_vel)

        # Solve OCP ---------------------------------------------------------------------------------------
        if step_num % round(1 / (self.mpc_frequency * simulation_dt)) == 0:

            self.nmpc_GRFs, \
                self.nmpc_footholds, \
                self.best_sample_freq = self.srbd_controller_interface.compute_control(state_current,
                                                                                       ref_state,
                                                                                       contact_sequence,
                                                                                       inertia,
                                                                                       self.wb_interface.pgg,
                                                                                       ref_feet_pos,
                                                                                       contact_sequence_dts,
                                                                                       contact_sequence_lenghts,
                                                                                       step_height,
                                                                                       optimize_swing)

            # Update the gait
            if (self._mpc_cfg['type'] != 'sampling' and self._mpc_cfg['optimize_step_freq']):
                self.best_sample_freq = self.srbd_batched_controller_interface.optimize_gait(state_current,
                                                                                             ref_state,
                                                                                             contact_sequence,
                                                                                             inertia,
                                                                                             self.wb_interface.pgg,
                                                                                             ref_feet_pos,
                                                                                             contact_sequence_dts,
                                                                                             contact_sequence_lenghts,
                                                                                             step_height,
                                                                                             optimize_swing)

        # Compute Swing and Stance Torque ---------------------------------------------------------------------------
        tau = self.wb_interface.compute_stance_and_swing_torque(simulation_dt,
                                                                qvel,
                                                                feet_jac,
                                                                jac_feet_dot,
                                                                feet_pos,
                                                                feet_vel,
                                                                legs_qfrc_bias,
                                                                legs_mass_matrix,
                                                                self.nmpc_GRFs,
                                                                self.nmpc_footholds,
                                                                legs_qvel_idx,
                                                                tau,
                                                                optimize_swing,
                                                                self.best_sample_freq)

        # Save some observables -------------------------------------------------------------------------------------
        self.quadrupedpympc_observables = {}
        for obs_name in self.quadrupedpympc_observables_names:
            if obs_name == 'ref_base_height':
                data = {'ref_base_height': ref_state['ref_position'][2]}
            elif obs_name == 'ref_base_angles':
                data = {'ref_base_angles': ref_state['ref_orientation']}
            elif obs_name == 'ref_feet_pos':
                data = {'ref_feet_pos': ref_feet_pos}
            elif obs_name == 'ref_feet_constraints':
                data = {'ref_feet_constraints': ref_feet_constraints}
            elif obs_name == 'nmpc_GRFs':
                data = {'nmpc_GRFs': self.nmpc_GRFs}
            elif obs_name == 'nmpc_footholds':
                data = {'nmpc_footholds': self.nmpc_footholds}
            elif obs_name == 'swing_time':
                data = {'swing_time': self.wb_interface.stc.swing_time}
            elif obs_name == 'phase_signal':
                data = {'phase_signal': self.wb_interface.pgg._phase_signal}
            elif obs_name == 'lift_off_positions':
                data = {'lift_off_positions': self.wb_interface.frg.lift_off_positions}

            else:
                data = {}
                raise ValueError(f"Unknown observable name: {obs_name}")

            self.quadrupedpympc_observables.update(data)

        return tau

    def get_obs(self, ) -> dict:
        """ Get some user-defined observables from withing the control loop.

        Returns:
            Dict: dictionary of observables
        """
        return self.quadrupedpympc_observables

    def reset(self,
              initial_feet_pos: LegsAttr):
        """ Reset the controller."""

        self.wb_interface.reset(initial_feet_pos)
