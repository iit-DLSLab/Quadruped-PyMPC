import numpy as np

from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator

from gym_quadruped.utils.quadruped_utils import LegsAttr

# from quadruped_pympc import config as cfg 

class SRBDBatchedControllerInterface:
    """This is an interface for a batched controller that uses the SRBD method to optimize the gait"""


    def __init__(self, mpc_cfg):
        """ Constructor for the SRBD batched controller interface """
        self._mpc_cfg = mpc_cfg

        self.type = self._mpc_cfg['type']
        self.mpc_dt = self._mpc_cfg['dt']
        self.horizon = self._mpc_cfg['horizon']
        self.optimize_step_freq = self._mpc_cfg['optimize_step_freq']
        self.num_parallel_computations = self._mpc_cfg['num_parallel_computations']
        self.sampling_method = self._mpc_cfg['sampling_method']
        self.control_parametrization = self._mpc_cfg['control_parametrization']
        self.num_sampling_iterations = self._mpc_cfg['num_sampling_iterations']
        self.sigma_cem_mppi = self._mpc_cfg['sigma_cem_mppi']
        self.step_freq_available = self._mpc_cfg['step_freq_available']


        from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import \
                    Acados_NMPC_GaitAdaptive
        
        self.batched_controller = Acados_NMPC_GaitAdaptive()
        


    

    def optimize_gait(self, 
                        state_current: dict,
                        ref_state: dict,
                        contact_sequence: np.ndarray,
                        inertia: np.ndarray,
                        pgg,
                        ref_feet_pos: LegsAttr,
                        contact_sequence_dts: np.ndarray,
                        contact_sequence_lenghts: np.ndarray,
                        step_height: float,
                        optimize_swing: int) -> float:
        """Optimize the gait using the SRBD method
        TODO: remove the unused arguments, and not pass pgg but rather its values
        
        Args:
            state_current (dict): The current state of the robot
            ref_state (dict): The reference state of the robot
            contact_sequence (np.ndarray): The contact sequence of the robot
            inertia (np.ndarray): The inertia of the robot
            pgg (PeriodicGaitGenerator): The periodic gait generator
            ref_feet_pos (LegsAttr): The reference feet positions
            contact_sequence_dts (np.ndarray): The contact sequence dts
            contact_sequence_lenghts (np.ndarray): The contact sequence lengths
            step_height (float): The step height
            optimize_swing (int): The flag to optimize the swing

        Returns:
            float: The best sample frequency
        """
    

        

        best_sample_freq = pgg.step_freq
        if self.optimize_step_freq and optimize_swing == 1:
            contact_sequence_temp = np.zeros((len(self.step_freq_available), 4, self.horizon))
            for j in range(len(self.step_freq_available)):
                pgg_temp = PeriodicGaitGenerator(duty_factor=pgg.duty_factor,
                                                    step_freq=self.step_freq_available[j],
                                                    gait_type=pgg.gait_type,
                                                    horizon=self.horizon)
                pgg_temp.set_phase_signal(pgg.phase_signal)
                contact_sequence_temp[j] = pgg_temp.compute_contact_sequence(contact_sequence_dts=contact_sequence_dts, 
                                                                                contact_sequence_lenghts=contact_sequence_lenghts)


            costs, \
            best_sample_freq = self.batched_controller.compute_batch_control(state_current,
                                                                        ref_state,
                                                                        contact_sequence_temp)


        
        return best_sample_freq
        