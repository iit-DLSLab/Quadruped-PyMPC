import numpy as np
import time

from quadruped_pympc import config as cfg

from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.terrain_estimator import TerrainEstimator

if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation


class WBInterface:
    def __init__(self,
                 initial_feet_pos,
                 legs_order):
        
        
        mpc_dt = cfg.mpc_params['dt']
        horizon = cfg.mpc_params['horizon']
        self.legs_order = legs_order
        
        # Periodic gait generator --------------------------------------------------------------
        gait_name = cfg.simulation_params['gait']
        gait_params = cfg.simulation_params['gait_params'][gait_name]
        gait_type, duty_factor, step_frequency = gait_params['type'], gait_params['duty_factor'], gait_params['step_freq']
        # Given the possibility to use nonuniform discretization,
        # we generate a contact sequence two times longer and with a dt half of the one of the mpc
        self.pgg = PeriodicGaitGenerator(duty_factor=duty_factor,
                                    step_freq=step_frequency,
                                    gait_type=gait_type,
                                    horizon=horizon)
        # in the case of nonuniform discretization, we create a list of dts and horizons for each nonuniform discretization
        if (cfg.mpc_params['use_nonuniform_discretization']):
            self.contact_sequence_dts = [cfg.mpc_params['dt_fine_grained'], mpc_dt]
            self.contact_sequence_lenghts = [cfg.mpc_params['horizon_fine_grained'], horizon]
        else:
            self.contact_sequence_dts = [mpc_dt]
            self.contact_sequence_lenghts = [horizon]
        contact_sequence = self.pgg.compute_contact_sequence(contact_sequence_dts=self.contact_sequence_dts, 
                                                        contact_sequence_lenghts=self.contact_sequence_lenghts)
        nominal_sample_freq = self.pgg.step_freq
        

        # Create the foothold reference generator ------------------------------------------------
        stance_time = (1 / self.pgg.step_freq) * self.pgg.duty_factor
        self.frg = FootholdReferenceGenerator(stance_time=stance_time, hip_height=cfg.hip_height, lift_off_positions=initial_feet_pos)


        # Create swing trajectory generator ------------------------------------------------------
        self.step_height = cfg.simulation_params['step_height']
        swing_period = (1 - self.pgg.duty_factor) * (1 / self.pgg.step_freq) 
        position_gain_fb = cfg.simulation_params['swing_position_gain_fb']
        velocity_gain_fb = cfg.simulation_params['swing_velocity_gain_fb']
        swing_generator = cfg.simulation_params['swing_generator']
        self.stc = SwingTrajectoryController(step_height=self.step_height, swing_period=swing_period,
                                        position_gain_fb=position_gain_fb, velocity_gain_fb=velocity_gain_fb,
                                        generator=swing_generator)


        # Terrain estimator -----------------------------------------------------------------------
        self.terrain_computation = TerrainEstimator()

        if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
            # Visual foothold adaptation -------------------------------------------------------------
            self.vfa = VisualFootholdAdaptation(legs_order=self.legs_order, adaptation_strategy=cfg.simulation_params['visual_foothold_adaptation'])


        self.current_contact = np.array([1, 1, 1, 1])

    
    def update_state_and_reference(self,
                                   base_pos,
                                   base_lin_vel,
                                   base_ori_euler_xyz,
                                   base_ang_vel,
                                   feet_pos,
                                   hip_pos,
                                   heightmaps,
                                   legs_order,
                                   simulation_dt,
                                   ref_base_lin_vel,
                                   ref_base_ang_vel):
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
            com_height_nominal=cfg.simulation_params['ref_z'])
        

        # Adjust the footholds given the terrain -----------------------------------------------------
        if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
            
            time_adaptation = time.time()
            if(self.stc.check_apex_condition(self.current_contact, interval=0.01) and self.vfa.initialized == False):
                for leg_id, leg_name in enumerate(legs_order):
                    heightmaps[leg_name].update_height_map(ref_feet_pos[leg_name], yaw=base_ori_euler_xyz[2])
                self.vfa.compute_adaptation(legs_order, ref_feet_pos, hip_pos, heightmaps, base_lin_vel, base_ori_euler_xyz, base_ang_vel)
                #print("Adaptation time: ", time.time() - time_adaptation)
            
            if(self.stc.check_full_stance_condition(self.current_contact)):
                self.vfa.reset()
            
            ref_feet_pos = self.vfa.get_footholds_adapted(ref_feet_pos)
            
            """step_height_modification = False
            for leg_id, leg_name in enumerate(legs_order):
                if(ref_feet_pos[leg_name][2] - frg.lift_off_positions[leg_name][2] > (cfg.simulation_params['step_height'] - 0.05) 
                    and ref_feet_pos[leg_name][2] - frg.lift_off_positions[leg_name][2] > 0.0):
                    step_height = ref_feet_pos[leg_name][2] - frg.lift_off_positions[leg_name][2] + 0.05
                    step_height_modification = True
                    stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=stc.swing_period)
            if(step_height_modification == False):
                step_height = cfg.simulation_params['step_height']
                stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=stc.swing_period)"""
        

        # Estimate the terrain slope and elevation -------------------------------------------------------
        terrain_roll, \
            terrain_pitch, \
            terrain_height = self.terrain_computation.compute_terrain_estimation(
            base_position=base_pos,
            yaw=base_ori_euler_xyz[2],
            feet_pos=self.frg.lift_off_positions,
            current_contact=self.current_contact)

        ref_pos = np.array([0, 0, cfg.hip_height])
        ref_pos[2] = cfg.simulation_params['ref_z'] + terrain_height


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


        return state_current, ref_state, contact_sequence, self.current_contact, ref_feet_pos, self.contact_sequence_dts, self.contact_sequence_lenghts, self.step_height
    


    def compute_stance_and_swing_torque(self,
                                        current_contact,
                                        simulation_dt,
                                        qvel,
                                        feet_jac,
                                        jac_feet_dot,
                                        feet_pos,
                                        feet_vel,
                                        legs_qfrc_bias,
                                        legs_mass_matrix,
                                        nmpc_GRFs,
                                        nmpc_footholds,
                                        legs_qvel_idx,
                                        tau):
        
        # Compute Stance Torque ---------------------------------------------------------------------------
        tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL)
        tau.FR = -np.matmul(feet_jac.FR[:, legs_qvel_idx.FR].T, nmpc_GRFs.FR)
        tau.RL = -np.matmul(feet_jac.RL[:, legs_qvel_idx.RL].T, nmpc_GRFs.RL)
        tau.RR = -np.matmul(feet_jac.RR[:, legs_qvel_idx.RR].T, nmpc_GRFs.RR)



        self.stc.update_swing_time(current_contact, self.legs_order, simulation_dt)


        # Compute Swing Torque ------------------------------------------------------------------------------
        # The swing controller is in the end-effector space. For its computation,
        # we save for simplicity joints position and velocities
        for leg_id, leg_name in enumerate(self.legs_order):
            if current_contact[leg_id] == 0:  # If in swing phase, compute the swing trajectory tracking control.
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