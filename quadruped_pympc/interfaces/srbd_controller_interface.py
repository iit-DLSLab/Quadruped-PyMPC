import numpy as np

from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg

class SRBDControllerInterface:
    """This is an interface for a controller that uses the SRBD method to optimize the gait"""


    def __init__(self, ):
        """ Constructor for the SRBD controller interface """
        
        self.type = cfg.mpc_params['type']
        self.mpc_dt = cfg.mpc_params['dt']
        self.horizon = cfg.mpc_params['horizon']
        self.optimize_step_freq = cfg.mpc_params['optimize_step_freq']
        self.step_freq_available = cfg.mpc_params['step_freq_available']


        self.previous_contact_mpc = np.array([1, 1, 1, 1])
        

        # input_rates optimize the delta_GRF (smoooth!)
        # nominal optimize directly the GRF (not smooth)
        # sampling use GPU
        if self.type == 'nominal':
            from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal

            self.controller = Acados_NMPC_Nominal()

            if self.optimize_step_freq:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import \
                    Acados_NMPC_GaitAdaptive

                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif self.type == 'input_rates':
            from quadruped_pympc.controllers.gradient.input_rates.centroidal_nmpc_input_rates import Acados_NMPC_InputRates

            self.controller = Acados_NMPC_InputRates()

            if self.optimize_step_freq:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import \
                    Acados_NMPC_GaitAdaptive

                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif(cfg.mpc_params['type'] == 'lyapunov'):
            from quadruped_pympc.controllers.gradient.lyapunov.centroidal_nmpc_lyapunov import Acados_NMPC_Lyapunov

            self.controller = Acados_NMPC_Lyapunov()

            if cfg.mpc_params['optimize_step_freq']:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif self.type == 'sampling':
            if self.optimize_step_freq:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax_gait_adaptive import Sampling_MPC
            else:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC

            self.controller = Sampling_MPC()
            


    

    def compute_control(self, 
                        state_current: dict,
                        ref_state: dict,
                        contact_sequence: np.ndarray,
                        inertia: np.ndarray,
                        pgg: object,
                        ref_feet_pos: LegsAttr,
                        contact_sequence_dts: np.ndarray,
                        contact_sequence_lenghts: np.ndarray,
                        step_height: float,
                        optimize_swing: int) -> [LegsAttr, LegsAttr, float]:
        """Compute the control using the SRBD method
        TODO: remove the unused arguments, and not pass pgg but rather its values

        Args:
            state_current (dict): The current state of the robot
            ref_state (dict): The reference state of the robot
            contact_sequence (np.ndarray): The contact sequence of the robot
            inertia (np.ndarray): The inertia of the robot
            pgg (object): The periodic gait generator
            ref_feet_pos (LegsAttr): The reference feet positions
            contact_sequence_dts (np.ndarray): The contact sequence dts
            contact_sequence_lenghts (np.ndarray): The contact sequence lengths
            step_height (float): The step height
            optimize_swing (int): The flag to optimize the swing

        Returns:
            tuple: The GRFs and the feet positions in world frame, 
                   and the best sample frequency (only if the controller is sampling)
        """
    

        current_contact = np.array([contact_sequence[0][0],
                                    contact_sequence[1][0],
                                    contact_sequence[2][0],
                                    contact_sequence[3][0]])

        
        # If we use sampling
        if (self.type == 'sampling'):

            # Convert data to jax and shift previous solution
            state_current_jax, \
            reference_state_jax, = self.controller.prepare_state_and_reference(state_current,
                                                                            ref_state,
                                                                            current_contact,
                                                                            self.previous_contact_mpc)
            self.previous_contact_mpc = current_contact

            for iter_sampling in range(self.controller.num_sampling_iterations):
                self.controller = self.controller.with_newkey()
                if (self.controller.sampling_method == 'cem_mppi'):
                    if (iter_sampling == 0):
                        self.controller = self.controller.with_newsigma(cfg.mpc_params['sigma_cem_mppi'])

                    nmpc_GRFs, \
                    nmpc_footholds, \
                    self.controller.best_control_parameters, \
                    best_cost, \
                    best_sample_freq, \
                    costs, \
                    sigma_cem_mppi = self.controller.jitted_compute_control(state_current_jax, reference_state_jax,
                                                                contact_sequence, self.controller.best_control_parameters,
                                                                self.controller.master_key, self.controller.sigma_cem_mppi)
                    controller = self.controller.with_newsigma(sigma_cem_mppi)
                else:
                    nominal_sample_freq = pgg.step_freq
                    nmpc_GRFs, \
                    nmpc_footholds, \
                    self.controller.best_control_parameters, \
                    best_cost, \
                    best_sample_freq, \
                    costs = self.controller.jitted_compute_control(state_current_jax, reference_state_jax,
                                                        contact_sequence, self.controller.best_control_parameters,
                                                        self.controller.master_key, pgg.phase_signal,
                                                        nominal_sample_freq, optimize_swing)


            nmpc_footholds = ref_feet_pos
            nmpc_GRFs = np.array(nmpc_GRFs)


        # If we use Gradient-Based MPC
        else:

            nmpc_GRFs, \
            nmpc_footholds, \
            _, \
            _ = self.controller.compute_control(state_current,
                                                ref_state,
                                                contact_sequence,
                                                inertia=inertia)


            nmpc_footholds = LegsAttr(FL=nmpc_footholds[0],
                                        FR=nmpc_footholds[1],
                                        RL=nmpc_footholds[2],
                                        RR=nmpc_footholds[3])
            
            
            # If the controller is using RTI, we need to linearize the mpc after its computation
            # this helps to minize the delay between new state->control, but only in a real case.
            # Here we are in simulation and does not make any difference for now
            if (self.controller.use_RTI):
                # preparation phase
                self.controller.acados_ocp_solver.options_set('rti_phase', 1)
                self.controller.acados_ocp_solver.solve()
                # print("preparation phase time: ", controller.acados_ocp_solver.get_stats('time_tot'))



            best_sample_freq = pgg.step_freq





        # TODO: Indexing should not be hardcoded. Env should provide indexing of leg actuator dimensions.
        nmpc_GRFs = LegsAttr(FL=nmpc_GRFs[0:3] * current_contact[0],
                                FR=nmpc_GRFs[3:6] * current_contact[1],
                                RL=nmpc_GRFs[6:9] * current_contact[2],
                                RR=nmpc_GRFs[9:12] * current_contact[3])
            

        
        return nmpc_GRFs, nmpc_footholds, best_sample_freq
        