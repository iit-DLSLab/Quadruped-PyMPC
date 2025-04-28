import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg
from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.interfaces.wb_interface import WBInterface

_DEFAULT_OBS = ("ref_base_height", "ref_base_angles", "nmpc_GRFs", "nmpc_footholds", "swing_time")


class QuadrupedPyMPC_Wrapper:
    """A simple class wrapper of all the mpc submodules (swing, contact generator, mpc itself)."""

    def __init__(
        self,
        initial_feet_pos: LegsAttr,
        legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR'),
        feet_geom_id: LegsAttr = None,
        quadrupedpympc_observables_names: tuple[str, ...] = _DEFAULT_OBS,
    ):
        """Constructor of the QuadrupedPyMPC_Wrapper class.

        Args:
            initial_feet_pos (LegsAttr): initial feet positions, otherwise they will be all zero.
            legs_order (tuple[str, str, str, str], optional): order of the leg. Defaults to ('FL', 'FR', 'RL', 'RR').
            quadrupedpympc_observables_names (tuple[str, ...], optional): list of observable to save. Defaults to _DEFAULT_OBS.
        """

        self.mpc_frequency = cfg.simulation_params["mpc_frequency"]

        self.srbd_controller_interface = SRBDControllerInterface()

        if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']:
            self.srbd_batched_controller_interface = SRBDBatchedControllerInterface()

        self.wb_interface = WBInterface(initial_feet_pos=initial_feet_pos(frame='world'), legs_order=legs_order, feet_geom_id =  feet_geom_id)

        self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_pos = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_vel = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_acc = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_predicted_state = np.zeros(12)
        self.best_sample_freq = self.wb_interface.pgg.step_freq

        self.quadrupedpympc_observables_names = quadrupedpympc_observables_names
        self.quadrupedpympc_observables = {}

    def compute_actions(
        self,
        com_pos: np.ndarray,
        base_pos: np.ndarray,
        base_lin_vel: np.ndarray,
        base_ori_euler_xyz: np.ndarray,
        base_ang_vel: np.ndarray,
        feet_pos: LegsAttr,
        hip_pos: LegsAttr,
        joints_pos: LegsAttr,
        heightmaps,
        legs_order: tuple[str, str, str, str],
        simulation_dt: float,
        ref_base_lin_vel: np.ndarray,
        ref_base_ang_vel: np.ndarray,
        step_num: int,
        qpos: np.ndarray,
        qvel: np.ndarray,
        feet_jac: LegsAttr,
        feet_jac_dot: LegsAttr,
        feet_vel: LegsAttr,
        legs_qfrc_passive: LegsAttr,
        legs_qfrc_bias: LegsAttr,
        legs_mass_matrix: LegsAttr,
        legs_qpos_idx: LegsAttr,
        legs_qvel_idx: LegsAttr,
        tau: LegsAttr,
        inertia: np.ndarray,
        mujoco_contact: np.ndarray,
    ) -> LegsAttr:
        """Given the current state of the robot (and the reference),
            compute the torques to be applied to the motors.

        Args:
            com_pos (np.ndarray): center of mass position in
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
            qpos (np.ndarray): joint positions
            qvel (np.ndarray): joint velocities
            feet_jac (LegsAttr): jacobian of the feet
            feet_jac_dot (LegsAttr): derivative of the jacobian of the feet
            feet_vel (LegsAttr): velocity of the feet
            legs_qfrc_passive (LegsAttr): passive forces acting on the joints
            legs_qfrc_bias (LegsAttr): gravity compensation, coriolis and centrifugal forces
            legs_mass_matrix (LegsAttr): mass matrix of the legs
            legs_qvel_idx (LegsAttr): indices of the joint velocities
            tau (LegsAttr): joint torques
            inertia (np.ndarray): inertia matrix of the robot (CCRBI)

        Returns:
            LegsAttr: torques to be applied to the motors
        """

        # Update the state and reference -------------------------
        state_current, ref_state, contact_sequence, step_height, optimize_swing = (
            self.wb_interface.update_state_and_reference(
                com_pos,
                base_pos,
                base_lin_vel,
                base_ori_euler_xyz,
                base_ang_vel,
                feet_pos,
                hip_pos,
                joints_pos,
                heightmaps,
                legs_order,
                simulation_dt,
                ref_base_lin_vel,
                ref_base_ang_vel,
                mujoco_contact,
            )
        )

        # Solve OCP ---------------------------------------------------------------------------------------
        if step_num % round(1 / (self.mpc_frequency * simulation_dt)) == 0:
            (
                self.nmpc_GRFs,
                self.nmpc_footholds,
                self.nmpc_joints_pos,
                self.nmpc_joints_vel,
                self.nmpc_joints_acc,
                self.best_sample_freq,
                self.nmpc_predicted_state,
            ) = self.srbd_controller_interface.compute_control(
                state_current,
                ref_state,
                contact_sequence,
                inertia,
                self.wb_interface.pgg.phase_signal,
                self.wb_interface.pgg.step_freq,
                optimize_swing,
            )

            if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']:
                # If the controller is gradient and is using RTI, we need to linearize the mpc after its computation
                # this helps to minize the delay between new state->control in a real case scenario.
                self.srbd_controller_interface.compute_RTI()

            # Update the gait
            if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']:
                self.best_sample_freq = self.srbd_batched_controller_interface.optimize_gait(
                    state_current,
                    ref_state,
                    inertia,
                    self.wb_interface.pgg.phase_signal,
                    self.wb_interface.pgg.step_freq,
                    self.wb_interface.pgg.duty_factor,
                    self.wb_interface.pgg.gait_type,
                    optimize_swing,
                )

        # Compute Swing and Stance Torque ---------------------------------------------------------------------------
        tau, des_joints_pos, des_joints_vel = self.wb_interface.compute_stance_and_swing_torque(
            simulation_dt,
            qpos,
            qvel,
            feet_jac,
            feet_jac_dot,
            feet_pos,
            feet_vel,
            legs_qfrc_passive,
            legs_qfrc_bias,
            legs_mass_matrix,
            self.nmpc_GRFs,
            self.nmpc_footholds,
            legs_qpos_idx,
            legs_qvel_idx,
            tau,
            optimize_swing,
            self.best_sample_freq,
            self.nmpc_joints_pos,
            self.nmpc_joints_vel,
            self.nmpc_joints_acc,
            self.nmpc_predicted_state,
            mujoco_contact,
        )

        # Do some PD control over the joints (these values are normally passed
        # to a low-level motor controller, here we can try to simulate it)
        kp_joint_motor = cfg.simulation_params['impedence_joint_position_gain']
        kd_joint_motor = cfg.simulation_params['impedence_joint_velocity_gain']
        # for leg in legs_order:
        #    tau[leg] += kp_joint_motor * (des_joints_pos[leg] - qpos[legs_qpos_idx[leg]]) + \
        #                kd_joint_motor * (des_joints_vel[leg] - qvel[legs_qvel_idx[leg]])

        # Save some observables -------------------------------------------------------------------------------------
        self.quadrupedpympc_observables = {}
        for obs_name in self.quadrupedpympc_observables_names:
            if obs_name == 'ref_base_height':
                data = {'ref_base_height': ref_state['ref_position'][2]}
            elif obs_name == 'ref_base_angles':
                data = {'ref_base_angles': ref_state['ref_orientation']}
            elif obs_name == 'ref_feet_pos':
                ref_feet_pos = LegsAttr(
                    FL=ref_state['ref_foot_FL'].reshape(3, 1),
                    FR=ref_state['ref_foot_FR'].reshape(3, 1),
                    RL=ref_state['ref_foot_RL'].reshape(3, 1),
                    RR=ref_state['ref_foot_RR'].reshape(3, 1),
                )
                data = {'ref_feet_pos': ref_feet_pos}
            elif obs_name == 'ref_feet_constraints':
                ref_feet_constraints = LegsAttr(
                    FL=ref_state['ref_foot_FL_constraints'],
                    FR=ref_state['ref_foot_FR_constraints'],
                    RL=ref_state['ref_foot_RL_constraints'],
                    RR=ref_state['ref_foot_RR_constraints'],
                )
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

    def get_obs(self) -> dict:
        """Get some user-defined observables from withing the control loop.

        Returns:
            Dict: dictionary of observables
        """
        return self.quadrupedpympc_observables

    def reset(self, initial_feet_pos: LegsAttr):
        """Reset the controller."""

        self.wb_interface.reset(initial_feet_pos)
        self.srbd_controller_interface.controller.reset()
