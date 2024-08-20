from quadruped_pympc.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.wb_interface import WBInterface

from quadruped_pympc import config as cfg

class QuadrupedPyMPC_Wrapper:
    def __init__(self, feet_pos, legs_order):

        self.mpc_frequency = cfg.simulation_params['mpc_frequency']

        self.srbd_controller_interface = SRBDControllerInterface()

        self.srbd_batched_controller_interface = SRBDBatchedControllerInterface()

        self.wb_interface = WBInterface(initial_feet_pos = feet_pos(frame='world'),
                                                            legs_order = legs_order)
        


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

            nmpc_GRFs,  \
            nmpc_footholds, \
            optimize_swing, \
            best_sample_freq = self.srbd_controller_interface.compute_control(state_current,
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
            best_sample_freq = self.srbd_batched_controller_interface.optimize_gait(state_current,
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
                                                    nmpc_GRFs,
                                                    nmpc_footholds,
                                                    legs_qvel_idx,
                                                    tau,
                                                    optimize_swing,
                                                    best_sample_freq)
        

        return tau
    

    def reset(self, feet_pos):
        self.wb_interface.reset(feet_pos)
        

    