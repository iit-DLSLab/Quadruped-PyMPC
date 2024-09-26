import numpy as np
import time

from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.terrain_estimator import TerrainEstimator



class WBInterface:
    """
    WBInterface is responsible for interfacing with the whole body controller of a quadruped robot.
    It initializes the necessary components for motion planning and control, including gait generation,
    swing trajectory control, and terrain estimation.
    """

    def __init__(self,
                 mpc_cfg,
                 sim_cfg,
                 initial_feet_pos: LegsAttr,
                 legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR')):
        """ Constructor of the WBInterface class

        Args:
            initial_feet_pos (LegsAttr): initial feet positions, otherwise they will be all zero
            legs_order (tuple[str, str, str, str], optional): order of the leg. Defaults to ('FL', 'FR', 'RL', 'RR').
        """        
        
        self._mpc_cfg = mpc_cfg
        self._sim_cfg = sim_cfg
        
        mpc_dt = self._mpc_cfg['dt']
        horizon = self._mpc_cfg['horizon']
        self.legs_order = legs_order
        
        # Periodic gait generator --------------------------------------------------------------
        gait_name = self._sim_cfg['gait']
        gait_params = self._sim_cfg['gait_params'][gait_name]
        gait_type, duty_factor, step_frequency = gait_params['type'], gait_params['duty_factor'], gait_params['step_freq']
        # Given the possibility to use nonuniform discretization,
        # we generate a contact sequence two times longer and with a dt half of the one of the mpc
        self.pgg = PeriodicGaitGenerator(duty_factor=duty_factor,
                                    step_freq=step_frequency,
                                    gait_type=gait_type,
                                    horizon=horizon)
        # in the case of nonuniform discretization, we create a list of dts and horizons for each nonuniform discretization
        if (self._mpc_cfg['use_nonuniform_discretization']):
            self.contact_sequence_dts = [self._mpc_cfg['dt_fine_grained'], mpc_dt]
            self.contact_sequence_lenghts = [self._mpc_cfg['horizon_fine_grained'], horizon]
        else:
            self.contact_sequence_dts = [mpc_dt]
            self.contact_sequence_lenghts = [horizon]

        
        # Create the foothold reference generator ------------------------------------------------
        stance_time = (1 / self.pgg.step_freq) * self.pgg.duty_factor
        self.frg = FootholdReferenceGenerator(stance_time=stance_time, hip_height=self._sim_cfg['ref_z'], lift_off_positions=initial_feet_pos)


        # Create swing trajectory generator ------------------------------------------------------
        self.step_height = self._sim_cfg['step_height']
        swing_period = (1 - self.pgg.duty_factor) * (1 / self.pgg.step_freq) 
        position_gain_fb = self._sim_cfg['swing_position_gain_fb']
        velocity_gain_fb = self._sim_cfg['swing_velocity_gain_fb']
        swing_generator = self._sim_cfg['swing_generator']
        self.stc = SwingTrajectoryController(step_height=self.step_height, swing_period=swing_period,
                                        position_gain_fb=position_gain_fb, velocity_gain_fb=velocity_gain_fb,
                                        generator=swing_generator)


        # Terrain estimator -----------------------------------------------------------------------
        self.terrain_computation = TerrainEstimator()

        # Visual foothold adaptation -------------------------------------------------------------
        if(self._sim_cfg['visual_foothold_adaptation'] != 'blind'):
            from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
            self.vfa = VisualFootholdAdaptation(
                legs_order=self.legs_order, adaptation_strategy=self._sim_cfg['visual_foothold_adaptation']
                )


        self.current_contact = np.array([1, 1, 1, 1])


    
    def update_state_and_reference(self,
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
                                   ref_base_ang_vel: np.ndarray) -> [dict, dict, list, LegsAttr, list, list, float, bool]:
        """Update the state and reference for the whole body controller, including the contact sequence, footholds, and terrain estimation.

        Args:
            base_pos (np.ndarray): base position in world frame
            base_lin_vel (np.ndarray): base linear velocity in world frame
            base_ori_euler_xyz (np.ndarray): base orientation in euler angles in world frame
            base_ang_vel (np.ndarray): base angular velocity in base frame
            feet_pos (LegsAttr): feet positions in world frame
            hip_pos (LegsAttr): hip positions in world frame
            heightmaps (dict): heightmaps for each leg
            legs_order (tuple[str, str, str, str]): order of the legs
            simulation_dt (float): simulation time step
            ref_base_lin_vel (np.ndarray): reference base linear velocity in world frame
            ref_base_ang_vel (np.ndarray): reference base angular velocity in world frame

        Returns:
            state_current (dict): dictionary of the state of the robot that is used in the mpc
            ref_state (dict):  dictionary of the reference state of the robot that is used in the mpc
            contact_sequence (np.ndarray): this is an array, containing the contact sequence of the robot in the future
            ref_feet_pos (LegsAttr): where to step in world frame
            ref_feet_constraints (LegsAttr): constraints for the footholds in the world frame
            contact_sequence_dts (list): 
            contact_sequence_lenghts (list): 
            step_height (float): step height
            optimize_swing (bool), boolean to inform that the robot is in the apex, hence we can optimize step freq. 
        """
        
        state_current = dict(
            position=base_pos,
            linear_velocity=base_lin_vel,
            orientation=base_ori_euler_xyz,
            angular_velocity=base_ang_vel,
            foot_FL=feet_pos.FL,
            foot_FR=feet_pos.FR,
            foot_RL=feet_pos.RL,
            foot_RR=feet_pos.RR
            )


        # Update the desired contact sequence ---------------------------
        self.pgg.run(simulation_dt, self.pgg.step_freq)
        contact_sequence = self.pgg.compute_contact_sequence(contact_sequence_dts=self.contact_sequence_dts, 
                                                contact_sequence_lenghts=self.contact_sequence_lenghts)


        previous_contact = self.current_contact
        self.current_contact = np.array([contact_sequence[0][0],
                                    contact_sequence[1][0],
                                    contact_sequence[2][0],
                                    contact_sequence[3][0]])


        # Compute the reference for the footholds ---------------------------------------------------
        self.frg.update_lift_off_positions(previous_contact, self.current_contact, feet_pos, legs_order)
        ref_feet_pos = self.frg.compute_footholds_reference(
            com_position=base_pos,
            base_ori_euler_xyz=base_ori_euler_xyz,
            base_xy_lin_vel=base_lin_vel[0:2],
            ref_base_xy_lin_vel=ref_base_lin_vel[0:2],
            hips_position=hip_pos,
            com_height_nominal=self._sim_cfg['ref_z'])
        

        # Adjust the footholds given the terrain -----------------------------------------------------
        if(self._sim_cfg['visual_foothold_adaptation'] != 'blind'):
            
            time_adaptation = time.time()
            if(self.stc.check_apex_condition(self.current_contact, interval=0.01) and self.vfa.initialized == False):
                for leg_id, leg_name in enumerate(legs_order):
                    heightmaps[leg_name].update_height_map(ref_feet_pos[leg_name], yaw=base_ori_euler_xyz[2])
                self.vfa.compute_adaptation(legs_order, ref_feet_pos, hip_pos, heightmaps, base_lin_vel, base_ori_euler_xyz, base_ang_vel)
                #print("Adaptation time: ", time.time() - time_adaptation)
            
            if(self.stc.check_full_stance_condition(self.current_contact)):
                self.vfa.reset()
            
            ref_feet_pos, ref_feet_constraints = self.vfa.get_footholds_adapted(ref_feet_pos)


            """step_height_modification = False
            for _, leg_name in enumerate(legs_order):
                #TODO this should be in the terrain frame
                ref_feet_pos_height_diffence = ref_feet_pos[leg_name][2] - self.frg.lift_off_positions[leg_name][2]
                if(ref_feet_pos_height_diffence > (self._sim_cfg['step_height']) and ref_feet_pos_height_diffence > 0.0):
                    step_height = ref_feet_pos_height_diffence + 0.05
                    step_height_modification = True
                    self.stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=self.stc.swing_period)
            if(step_height_modification == False):
                step_height = self._sim_cfg['step_height']
                self.stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=self.stc.swing_period)"""
        
        else:
            ref_feet_constraints = LegsAttr(FL=None, FR=None, RL=None, RR=None)
        

        # Estimate the terrain slope and elevation -------------------------------------------------------
        terrain_roll, \
            terrain_pitch, \
            terrain_height = self.terrain_computation.compute_terrain_estimation(
            base_position=base_pos,
            yaw=base_ori_euler_xyz[2],
            feet_pos=self.frg.lift_off_positions,
            current_contact=self.current_contact)

        ref_pos = np.array([0, 0, self._sim_cfg['ref_z']])
        ref_pos[2] = self._sim_cfg['ref_z'] + terrain_height


        # Update state reference ------------------------------------------------------------------------
        ref_state = {}
        ref_state |= dict(ref_foot_FL=ref_feet_pos.FL.reshape((1, 3)),
                            ref_foot_FR=ref_feet_pos.FR.reshape((1, 3)),
                            ref_foot_RL=ref_feet_pos.RL.reshape((1, 3)),
                            ref_foot_RR=ref_feet_pos.RR.reshape((1, 3)),
                            # Also update the reference base linear velocity and
                            ref_linear_velocity=ref_base_lin_vel,
                            ref_angular_velocity=ref_base_ang_vel,
                            ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0]),
                            ref_position=ref_pos
                            )
        # -------------------------------------------------------------------------------------------------


        if(self._mpc_cfg['optimize_step_freq']):
            # we can always optimize the step freq, or just at the apex of the swing
            # to avoid possible jittering in the solution
            optimize_swing = self.stc.check_apex_condition(self.current_contact)
        else:
            optimize_swing = 0

        return state_current, ref_state, contact_sequence, ref_feet_pos, ref_feet_constraints, self.contact_sequence_dts, self.contact_sequence_lenghts, self.step_height, optimize_swing
    


    def compute_stance_and_swing_torque(self,
                                        simulation_dt: float,
                                        qvel: np.ndarray,
                                        feet_jac: LegsAttr,
                                        jac_feet_dot: LegsAttr,
                                        feet_pos: LegsAttr,
                                        feet_vel: LegsAttr,
                                        legs_qfrc_bias: LegsAttr,
                                        legs_mass_matrix: LegsAttr,
                                        nmpc_GRFs: LegsAttr,
                                        nmpc_footholds: LegsAttr,
                                        legs_qvel_idx: LegsAttr,
                                        tau: LegsAttr,
                                        optimize_swing: int,
                                        best_sample_freq: float) -> LegsAttr:
        """Compute the stance and swing torque.

        Args:
            simulation_dt (float): simulation time step
            qvel (np.ndarray): joint velocities
            feet_jac (LegsAttr): feet jacobian
            jac_feet_dot (LegsAttr): jacobian of the feet derivative
            feet_pos (LegsAttr): feet positions in world frame
            feet_vel (LegsAttr): feet velocities in world frame
            legs_qfrc_bias (LegsAttr): joint forces and torques
            legs_mass_matrix (LegsAttr): mass matrix of the legs
            nmpc_GRFs (LegsAttr): ground reaction forces from the MPC in world frame 
            nmpc_footholds (LegsAttr): footholds from the MPC in world frame 
            legs_qvel_idx (LegsAttr): joint velocities index
            tau (LegsAttr): joint torques
            optimize_swing (int): flag to signal that we need to update the swing trajectory time
            best_sample_freq (float): best sample frequency obtained from the 
                                      sampling optimization or the batched ocp

        Returns:
            LegsAttr: joint torques
        """
        

        # If we have optimized the gait, we set all the timing parameters
        if (optimize_swing == 1):
            self.pgg.step_freq = np.array([best_sample_freq])[0]
            nominal_sample_freq = self.pgg.step_freq
            self.frg.stance_time = (1 / self.pgg.step_freq) * self.pgg.duty_factor
            swing_period = (1 - self.pgg.duty_factor) * (1 / self.pgg.step_freq)
            self.stc.regenerate_swing_trajectory_generator(step_height=self.step_height, swing_period=swing_period)
        
        
        # Compute Stance Torque ---------------------------------------------------------------------------
        tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL)
        tau.FR = -np.matmul(feet_jac.FR[:, legs_qvel_idx.FR].T, nmpc_GRFs.FR)
        tau.RL = -np.matmul(feet_jac.RL[:, legs_qvel_idx.RL].T, nmpc_GRFs.RL)
        tau.RR = -np.matmul(feet_jac.RR[:, legs_qvel_idx.RR].T, nmpc_GRFs.RR)



        self.stc.update_swing_time(self.current_contact, self.legs_order, simulation_dt)


        # Compute Swing Torque ------------------------------------------------------------------------------
        # The swing controller is in the end-effector space. For its computation,
        # we save for simplicity joints position and velocities
        for leg_id, leg_name in enumerate(self.legs_order):
            if self.current_contact[leg_id] == 0:  # If in swing phase, compute the swing trajectory tracking control.
                tau[leg_name], _, _ = self.stc.compute_swing_control(
                    leg_id=leg_id,
                    q_dot=qvel[legs_qvel_idx[leg_name]],
                    J=feet_jac[leg_name][:, legs_qvel_idx[leg_name]],
                    J_dot=jac_feet_dot[leg_name][:, legs_qvel_idx[leg_name]],
                    lift_off=self.frg.lift_off_positions[leg_name],
                    touch_down=nmpc_footholds[leg_name],
                    foot_pos=feet_pos[leg_name],
                    foot_vel=feet_vel[leg_name],
                    h=legs_qfrc_bias[leg_name],
                    mass_matrix=legs_mass_matrix[leg_name]
                    )
                
        return tau
    


    def reset(self, 
              initial_feet_pos: LegsAttr):
        """Reset the whole body interface

        Args:
            initial_feet_pos (LegsAttr): initial feet positions
        """
        
        self.pgg.reset()
        #self.frg.reset()
        #self.stc.reset()
        #self.terrain_computation.reset()
        self.frg.lift_off_positions = initial_feet_pos
        if(self._sim_cfg['visual_foothold_adaptation'] != 'blind'):
            self.vfa.reset()
        self.current_contact = np.array([1, 1, 1, 1])
        return