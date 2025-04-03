import numpy as np

from quadruped_pympc import config as cfg
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator



class SRBDBatchedControllerInterface:
    """This is an interface for a batched controller that uses the SRBD method to optimize the gait"""

    def __init__(self):
        """Constructor for the SRBD batched controller interface"""

        self.type = cfg.mpc_params['type']
        self.mpc_dt = cfg.mpc_params['dt']
        self.horizon = cfg.mpc_params['horizon']
        self.optimize_step_freq = cfg.mpc_params['optimize_step_freq']
        self.step_freq_available = cfg.mpc_params['step_freq_available']

        from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive

        self.batched_controller = Acados_NMPC_GaitAdaptive()

        # in the case of nonuniform discretization, we create a list of dts and horizons for each nonuniform discretization
        if cfg.mpc_params['use_nonuniform_discretization']:
            self.contact_sequence_dts = [cfg.mpc_params['dt_fine_grained'], self.mpc_dt]
            self.contact_sequence_lenghts = [cfg.mpc_params['horizon_fine_grained'], self.horizon]
        else:
            self.contact_sequence_dts = [self.mpc_dt]
            self.contact_sequence_lenghts = [self.horizon]

    def optimize_gait(
        self,
        state_current: dict,
        ref_state: dict,
        inertia: np.ndarray,
        pgg_phase_signal: np.ndarray,
        pgg_step_freq: float,
        pgg_duty_factor: float,
        pgg_gait_type: int,
        optimize_swing: int,
    ) -> float:
        """Optimize the gait using the SRBD method
        TODO: remove the unused arguments, and not pass pgg but rather its values

        Args:
            state_current (dict): The current state of the robot
            ref_state (dict): The reference state of the robot
            inertia (np.ndarray): The inertia of the robot
            pgg_phase_signal (np.ndarray): The periodic gait generator phase signal of the legs (from 0 to 1)
            pgg_step_freq (float): The step frequency of the periodic gait generator
            pgg_duty_factor (float): The duty factor of the periodic gait generator
            pgg_gait_type (int): The gait type of the periodic gait generator
            contact_sequence_dts (np.ndarray): The contact sequence dts
            contact_sequence_lenghts (np.ndarray): The contact sequence lengths
            optimize_swing (int): The flag to optimize the swing

        Returns:
            float: The best sample frequency
        """

        best_sample_freq = pgg_step_freq
        if self.optimize_step_freq and optimize_swing == 1:
            contact_sequence_temp = np.zeros((len(self.step_freq_available), 4, self.horizon))
            for j in range(len(self.step_freq_available)):
                pgg_temp = PeriodicGaitGenerator(
                    duty_factor=pgg_duty_factor,
                    step_freq=self.step_freq_available[j],
                    gait_type=pgg_gait_type,
                    horizon=self.horizon,
                )
                pgg_temp.set_phase_signal(pgg_phase_signal)
                contact_sequence_temp[j] = pgg_temp.compute_contact_sequence(
                    contact_sequence_dts=self.contact_sequence_dts,
                    contact_sequence_lenghts=self.contact_sequence_lenghts,
                )

            costs, best_sample_freq = self.batched_controller.compute_batch_control(
                state_current, ref_state, contact_sequence_temp
            )

        return best_sample_freq
