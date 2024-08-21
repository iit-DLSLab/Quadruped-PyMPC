from quadruped_pympc.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.wb_interface import WBInterface

from gym_quadruped.utils.quadruped_utils import LegsAttr
from quadruped_pympc import config as cfg

import numpy as np


_DEFAULT_OBS = ('ref_base_height', 'ref_base_angles', 'nmpc_GRFs', 'nmpc_footholds', 'swing_time')

class QuadrupedPyMPC_Wrapper:
    def __init__(self, feet_pos, legs_order, quadrupedpympc_observables_names = _DEFAULT_OBS):

        self.mpc_frequency = cfg.simulation_params['mpc_frequency']

        self.srbd_controller_interface = SRBDControllerInterface()

        if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']):
            self.srbd_batched_controller_interface = SRBDBatchedControllerInterface()

        self.wb_interface = WBInterface(initial_feet_pos = feet_pos(frame='world'),
                                                            legs_order = legs_order)
        

        self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                  RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                        RL=np.zeros(3), RR=np.zeros(3))
        self.best_sample_freq = self.wb_interface.pgg.step_freq
        

        self.quadrupedpympc_observables_names = quadrupedpympc_observables_names
        self.quadrupedpympc_observables = {}
        


    def compute_actions(self, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel, 
                        feet_pos, hip_pos, heightmaps, 
                        legs_order, simulation_dt, ref_base_lin_vel, ref_base_ang_vel, 
                        step_num, qvel, feet_jac, jac_feet_dot, feet_vel, legs_qfrc_bias, 
                        legs_mass_matrix, legs_qvel_idx, tau, inertia):
                        
        # Update the state and reference -------------------------
        state_current, \
        ref_state, \
        contact_sequence, \
        ref_feet_pos, \
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

            self.nmpc_GRFs,  \
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
            if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']):
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
        
    

    def get_obs(self,):
        return self.quadrupedpympc_observables

    

    def reset(self, feet_pos):
        self.wb_interface.reset(feet_pos)
        

    