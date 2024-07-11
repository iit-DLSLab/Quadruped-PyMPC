
import numpy as np

# Parameters for both MPC and simulation
from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.terrain_estimator import TerrainEstimator

####
    
#------------
class SingleRigidBodyController:
    #constructor
    def __init__(self,
                 hip_height=None,
                 legs_order=None,        
                 inertia=None,
                 lift_off_positions=None,
                 simulation_parameters:dict=None,
                 mpc_parameters:dict=None,
                 jac_feet_dot=None,
                 jac_feet_prev=None,
                 tau=None,
                 ):
        #
        
        #Robot dependent parameters
        self.hip_height = hip_height
        self.inertia=inertia
        self.lift_off_positions=lift_off_positions
        self.legs_order = legs_order
        self.jac_feet_prev = jac_feet_prev
        self.jac_feet_dot = jac_feet_dot
        self.tau = tau
        ### All this parameter are for the WBC in simulation come from the dicotionary
        self.swing_generator = simulation_parameters['swing_generator']
        self.position_gain_fb = simulation_parameters['swing_position_gain_fb']
        self.velocity_gain_fb = simulation_parameters['swing_velocity_gain_fb']
        self.swing_integral_gain_fb=simulation_parameters['swing_integral_gain_fb']
        self.step_height =simulation_parameters['step_height']
        self.simulation_dt=simulation_parameters['dt'] #remove from here
        gait_name=simulation_parameters['gait']
        gait_params=simulation_parameters['gait_params'][gait_name]
        self.gait_type, self.duty_factor, self.step_frequency = gait_params['type'], gait_params['duty_factor'], gait_params['step_freq']
        self.use_inertia_recomputation=simulation_parameters['use_inertia_recomputation']
        self.mpc_frequency = simulation_parameters['mpc_frequency']
        self.ref_z=simulation_parameters['ref_z']
        
        ## Common MPC parameters
        self.controller_type=mpc_parameters['type']
        self.horizon = mpc_parameters['horizon']
        self.mpc_dt = mpc_parameters['dt']
        self.optimize_step_frequency=mpc_parameters['optimize_step_freq']
        self.step_freq_available=mpc_parameters['step_freq_available']
        self.grf_max=mpc_parameters["grf_max"]
        self.grf_min=mpc_parameters["grf_min"]
        self.mu=mpc_parameters["mu"]

        # compute stance and swing tme based on gate selection!
        self.stance_time = (1 / self.step_frequency) * self.duty_factor
        self.swing_period = (1 - self.duty_factor) * (1 / self.step_frequency)  # + 0.07
        ### initialized controller function
        self.set_controller(mpc_parameters=mpc_parameters)
        ## Initialize the foothold reference generator
        self.frg = FootholdReferenceGenerator(stance_time=self.stance_time, hip_height=self.hip_height,
                                              lift_off_positions= self.lift_off_positions)
        # Create swing trajectory generator

        self.stc = SwingTrajectoryController(step_height=self.step_height, swing_period=self.swing_period,
                                        position_gain_fb=self.position_gain_fb, velocity_gain_fb=self.velocity_gain_fb,
                                        generator=self.swing_generator)
        # periodic gate generator
        self.pgg = PeriodicGaitGenerator(duty_factor=self.duty_factor, step_freq=self.step_frequency, gait_type=self.gait_type,
                                         horizon=self.horizon * 2, contact_sequence_dt=self.mpc_dt/2.)
        self.contact_sequence = self.pgg.compute_contact_sequence()
        self.nominal_sample_freq = self.step_frequency

        # Online computation of the inertia parameter
        # Terrain estimator
        self.terrain_computation = TerrainEstimator()

        ####
        self.ref_orientation = np.array([0.0, 0.0, 0.0])
        # # SET REFERENCE AS DICTIONARY
        # TODO: I would suggest to create a DataClass for "BaseConfig" used in the PotatoModel controllers.
        self.ref_state = {}
        # Starting contact sequence
        self.previous_contact = np.array([1, 1, 1, 1])
        self.previous_contact_mpc = np.array([1, 1, 1, 1])
        self.current_contact = np.array([1, 1, 1, 1])

        self.nmpc_GRFs = np.zeros((12,))
        self.nmpc_wrenches = np.zeros((6,))
        self.nmpc_footholds = np.zeros((12,))
        # State
        self.state_current, self.state_prev = {}, {}
        self.feet_pos = None

    def set_controller(self,mpc_parameters:dict=None):
        ##############################################
        ############ SET THE CONTROLLER TYPE  ########
        ##############################################

        if(self.controller_type == 'nominal'):
            from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal
                    ##gradient based properties
        
            self.warm_start=mpc_parameters['use_warm_start']
            self.use_integrator=mpc_parameters['use_integrators']
            self.alpha_integrator=mpc_parameters['alpha_integrator']
            self.inttegrator_cap=mpc_parameters['integrator_cap']
            self.use_foothold_optimization=mpc_parameters['use_foothold_optimization']
            self.use_foothold_constraints=mpc_parameters['use_foothold_constraints']
            self.use_RTI=mpc_parameters['use_RTI']
            self.as_rti_type=mpc_parameters['as_rti_type']
            self.as_rti_iter=mpc_parameters['as_rti_iter']
            self.use_DDP=mpc_parameters['use_DDP']
            self.num_qp_iterations=mpc_parameters['num_qp_iterations']
            self.solver_mode=mpc_parameters['solver_mode']
            self.use_zmp_stability=mpc_parameters['use_zmp_stability']              
            self.trot_stability_margin=mpc_parameters['trot_stability_margin']              
            self.pace_stability_margin=mpc_parameters['pace_stability_margin']              
            self.crawl_stability_margin=mpc_parameters['crawl_stability_margin']

            self.external_wrenches_compensation=mpc_parameters['external_wrenches_compensation']
            self.external_wrenches_compensation_num_steps=mpc_parameters['external_wrenches_compensation_num_step']
            self.passive_arm_compensation=mpc_parameters['passive_arm_compensation']

            self.use_nonuniform_discretization=mpc_parameters['use_nonuniform_discretization']
            self.dt_fine_grained = mpc_parameters['dt_fine_grained']
            self.horizon_fine_grained = mpc_parameters['horizon_fine_grained']

            self.use_input_prediction=mpc_parameters['use_input_prediction']
            self.use_static_stability=mpc_parameters['use_static_stability']
            self.controller = Acados_NMPC_Nominal()

            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
                self.batched_controller = Acados_NMPC_GaitAdaptive()
        elif(self.controller_type == 'collaborative'):
            from quadruped_pympc.controllers.gradient.collaborative.centroidal_nmpc_collaborative import Acados_NMPC_Collaborative
            self.controller = Acados_NMPC_Nominal()
        elif(self.controller_type == 'input_rates'):
            from quadruped_pympc.controllers.gradient.input_rates.centroidal_nmpc_input_rates import Acados_NMPC_InputRates

            self.controller = Acados_NMPC_InputRates()

            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif(self.controller_type ==  'sampling'):
            self.sampling_method = mpc_parameters['sampling_method']
            self.control_parametrization = mpc_parameters['control_parametrization']
            self.num_parallel_computations = mpc_parameters['num_parallel_computations']
            self.num_sampling_iterations = mpc_parameters['num_sampling_iterations']
            self.sigma_mppi=mpc_parameters['sigma_mppi']
            self.sigma_cem_mppi=mpc_parameters['sigma_cem_mppi']
            self.sigma_random_sampling=mpc_parameters['sigma_random_sampling']
            self.shift_solution = mpc_parameters['shift_solution']
            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax_gait_adaptive import Sampling_MPC

            else:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC

            import jax
            import jax.numpy as jnp
            num_parallel_computations = self.num_parallel_computations
            self.controller = Sampling_MPC(horizon = self.horizon, 
                                      dt = self.mpc_dt, 
                                      num_parallel_computations = num_parallel_computations,
                                      sampling_method = self.sampling_method,
                                      control_parametrization = self.control_parametrization,
                                      device="gpu")
            self.best_control_parameters = jnp.zeros((self.controller.num_control_parameters, ))
            self.jitted_compute_control = jax.jit(self.controller.compute_control, device=self.controller.device)
            # jitted_get_key = jax.jit(controller.get_key, device=controller.device)
            self.jitted_prepare_state_and_reference = self.controller.prepare_state_and_reference

            self.index_shift = 0

        # Tracking mpc error (z, x_dot, y_dot, z_dot, roll, pitch, roll_dot, pitch_dot, yaw_dot)
        self.mean_tracking = np.zeros((9,))
        self.mean_num_sample = 1
        self.mean_optimizer_cost = 0
        self.optimizer_cost = 0

    def update_mpc_reference(self,nv:float,feet_pos:list,hip_pos:list,base_pos:list,base_ori_euler_xyz:list,base_lin_vel:list, 
                             base_ang_vel:list,ref_base_lin_vel:list, ref_base_ang_vel:list):
        """
        Update the MPC reference.
        
        Parameters:
        nv (int): mujoco number of variables
        feet_pos (list or np.ndarray): Feet positions;
        hip_pos (list or np.ndarray): Hip positions;
        base_pos (list or np.ndarray): Base position;
        base_ori_euler_xyz (list or np.ndarray): Base orientation in Euler angles (XYZ);
        base_lin_vel (list or np.ndarray): Base linear velocity;
        base_ang_vel (list or np.ndarray): Base angular velocity;
        ref_base_lin_vel (list or np.ndarray): Reference base linear velocity;
        ref_base_ang_vel (list or np.ndarray): Reference base angular velocity;
        """
        # Initialization of variables used in the main control loop
        # ____________________________________________________________
        # Set the reference for the state
        #if first robot
        # self.feet_pos=feet_pos

        # self.feet_pos = [remap_keys[leg] for leg in feet_pos]
        self.feet_pos=feet_pos
        self.hip_pos = hip_pos

        # Update the robot state --------------------------------
        #two robot change
        self.state_current = dict(
            position= base_pos,
            linear_velocity= base_lin_vel,
            orientation= base_ori_euler_xyz,
            angular_velocity= base_ang_vel,
            foot_FL=self.feet_pos.FL,
            foot_FR=self.feet_pos.FR,
            foot_RL=self.feet_pos.RL,
            foot_RR=self.feet_pos.RR
            )
        

        # Update target base velocity
        # -------------------------------------------------------

        # Update the desired contact sequence ---------------------------
        self.pgg.run(self.simulation_dt, self.pgg.step_freq)
        self.contact_sequence = self.pgg.compute_contact_sequence()

        # in the case of nonuniform discretization, we need to subsample the contact sequence
        if self.use_nonuniform_discretization==True:
            self.contact_sequence = self.pgg.sample_contact_sequence(self.contact_sequence, self.mpc_dt, self.dt_fine_grained, self.horizon_fine_grained)
        

        previous_contact = self.current_contact
        self.current_contact = np.array([self.contact_sequence[0][0],
                                    self.contact_sequence[1][0],
                                    self.contact_sequence[2][0],
                                    self.contact_sequence[3][0]])
        
        

        # Compute the reference for the footholds ---------------------------------------------------
        self.frg.update_lift_off_positions(previous_contact, self.current_contact, self.feet_pos, self.legs_order[0:4])
        ref_feet_pos = self.frg.compute_footholds_reference(
            com_position=base_pos, #this comes as input from env
            base_ori_euler_xyz=base_ori_euler_xyz, #this comes as input from env
            base_xy_lin_vel=base_lin_vel[0:2], #this comes as input from env
            ref_base_xy_lin_vel=ref_base_lin_vel[0:2],
            hips_position=self.hip_pos,
            com_height_nominal=self.ref_z,)

        # Estimate the terrain slope and elevation -------------------------------------------------------
        terrain_roll, \
            terrain_pitch, \
            terrain_height = self.terrain_computation.compute_terrain_estimation(
            base_position=base_pos, 
            yaw=base_ori_euler_xyz[2], 
            feet_pos=self.frg.lift_off_positions,
            current_contact=self.current_contact)

        self.ref_pos = np.array([0, 0, self.hip_height])
        self.ref_pos[2] = self.ref_z+ terrain_height

        # Update state reference ------------------------------------------------------------------------
        self.ref_state |= dict(ref_foot_FL=ref_feet_pos.FL.reshape((1, 3)),
                          ref_foot_FR=ref_feet_pos.FR.reshape((1, 3)),
                          ref_foot_RL=ref_feet_pos.RL.reshape((1, 3)),
                          ref_foot_RR=ref_feet_pos.RR.reshape((1, 3)),
                          # Also update the reference base linear velocity and
                          ref_linear_velocity=ref_base_lin_vel,
                          ref_angular_velocity=ref_base_ang_vel,
                          ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0]),
                          ref_position=self.ref_pos
                          )
        return ref_feet_pos
        
        # -------------------------------------------------------------------------------------------------

    def solve_MPC_main_loop(self,alpha : float,inertia_env: list,ref_feet_pos: list):
        """
        Main loop to solve the Model Predictive Control (MPC).
        
        Parameters:
        alpha (float): A numeric parameter representing simulation time for solving the MPC;
        inertia_env (list or np.ndarray): Inertia  values from QuadrupedEnv,
            
        """
        # Solve OCP ---------------------------------------------------------------------------------------
        # if alpha % round(1 / (self.mpc_frequency * self.simulation_dt)) == 0:

        # We can recompute the inertia of the single rigid body model
        # or use the fixed one in cfg.py
        if self.use_inertia_recomputation==True:
            # TODO: d.qpos is not defined
            #inertia = srb_inertia_computation.compute_inertia(d.qpos)
            inertia = inertia_env  # Reflected inertia of base at qpos, in world frame 
            #this comes as inout from env
        else:
            inertia = self.inertia.flatten()
        if (self.optimize_step_frequency == True):
            # we can always optimize the step freq, or just at the apex of the swing
            # to avoid possible jittering in the solution
            optimize_swing = 0  # 1 for always, 0 for apex
            for leg_id in range(8): #two robote edit
                # Swing time check
                if (self.current_contact[leg_id] == 0):
                    if ((self.stc.swing_time[leg_id] > (self.swing_period / 2.) - 0.02) and \
                            (self.stc.swing_time[leg_id] < (self.swing_period / 2.) + 0.02)):
                        optimize_swing = 1
                        nominal_sample_freq = self.step_frequency
        else:
            optimize_swing = 0
        # If we use sampling
        if (self.controller_type == 'sampling'):
            # Shift the previous solution ahead
            if (self.shift_solution == True):
                index_shift = 1./self.mpc_frequency
                best_control_parameters = self.controller.shift_solution(best_control_parameters, index_shift)
            
            # Convert data to jax
            state_current_jax, \
                reference_state_jax, \
                best_control_parameters = self.jitted_prepare_state_and_reference(self.state_current, self.ref_state,
                                                                             best_control_parameters,
                                                                             self.current_contact, self.previous_contact_mpc)
            for iter_sampling in range(self.num_sampling_iterations):
                if (self.sampling_method == 'mppi'):
                    if (iter_sampling == 0):
                        self.controller = self.controller.with_newsigma(self.sigma_cem_mppi)
                        nmpc_GRFs, \
                        nmpc_footholds, \
                        best_control_parameters, \
                        best_cost, \
                        best_sample_freq, \
                        costs, \
                        sigma_cem_mppi = self.jitted_compute_control(state_current_jax, reference_state_jax,
                                                                self.contact_sequence, best_control_parameters,
                                                                self.controller.master_key, self.controller.sigma_cem_mppi)
                    self.controller = self.controller.with_newsigma(sigma_cem_mppi)
                else:
                        nmpc_GRFs, \
                        nmpc_footholds, \
                        best_control_parameters, \
                        best_cost, \
                        best_sample_freq, \
                        costs = self.jitted_compute_control(state_current_jax, reference_state_jax, self.contact_sequence,
                                                       best_control_parameters, self.controller.master_key, self.pgg.get_t(),
                                                       nominal_sample_freq, optimize_swing)
                self.controller = self.controller.with_newkey()
            if ((self.optimize_step_frequency==True) and (optimize_swing == 1)):
                self.pgg.step_freq = np.array([best_sample_freq])[0]
                nominal_sample_freq = self.pgg.step_freq
                self.stance_time = (1 / self.pgg.step_freq) * self.duty_factor
                self.frg.stance_time = self.stance_time
                self.swing_period = (1 - self.duty_factor) * (1 / self.pgg.step_freq)  # + 0.07
                self.stc.regenerate_swing_trajectory_generator(step_height=self.step_height, swing_period=self.swing_period)
            nmpc_footholds = ref_feet_pos #todo return ref feet pos from the controller
            nmpc_GRFs = np.array(self.nmpc_GRFs)
            self.previous_contact_mpc = self.current_contact
            index_shift = 0
            
            # optimizer_cost = best_cost
        # If we use Gradient-Based MPC
        else:
            nmpc_GRFs, nmpc_footholds, _, status = self.controller.compute_control(
                self.state_current,
                self.ref_state,
                self.contact_sequence,
                inertia=inertia)
            nmpc_GRFs, nmpc_footholds, _, status = self.controller.compute_control(
                self.state_current,
                self.ref_state,
                self.contact_sequence,
                inertia=inertia)
            
            # optimizer_cost = controller.acados_ocp_solver.get_cost()
            if (self.optimize_step_frequency==True)  and optimize_swing == 1:
                contact_sequence_temp = np.zeros((len(self.step_freq_available), 4, self.horizon * 2))
                for j in range(len(self.step_freq_available)):
                    pgg_temp = PeriodicGaitGenerator(duty_factor=self.duty_factor,
                                                     step_freq=self.step_freq_available[j],
                                                     gait_type=self.gait_type,
                                                     horizon=self.horizon * 2)
                    pgg_temp.phase_signal = self.pgg.phase_signal
                    pgg_temp.init = self.pgg.init
                    contact_sequence_temp[j] = pgg_temp.compute_contact_sequence()
                    
                    # in the case of nonuniform discretization, we need to subsample the contact sequence
                    if (self.use_nonuniform_discretization==True):
                        contact_sequence_temp[j] = self.pgg.sample_contact_sequence(self.contact_sequence, self.mpc_dt, self.dt_fine_grained, self.horizon_fine_grained)
    
                costs, best_sample_freq = self.batched_controller.compute_batch_control(self.state_current, self.ref_state,
                                                                                   contact_sequence_temp)
                self.pgg.step_freq = best_sample_freq
                self.stance_time = (1 / self.pgg.step_freq) * self.duty_factor
                self.frg.stance_time = self.stance_time
                self.swing_period = (1 - self.duty_factor) * (1 / self.pgg.step_freq)  # + 0.07
                self.stc.regenerate_swing_trajectory_generator(step_height=self.step_height, swing_period=self.swing_period)
            # If the controller is using RTI, we need to linearize the mpc after its computation
            # this helps to minize the delay between new state->control, but only in a real case.
            # Here we are in simulation and does not make any difference for now
            if (self.controller.use_RTI):
                # preparation phase
                self.controller.acados_ocp_solver.options_set('rti_phase', 1)
                status = self.controller.acados_ocp_solver.solve()
                # print("preparation phase time: ", controller.acados_ocp_solver.get_stats('time_tot'))
    
        # current_contact=self.current_contact
        # print(nmpc_footholds,"\n", nmpc_GRFs,"\n",current_contact)
        assert self.nmpc_footholds is not None,f"nmpc is Nonve"
        assert nmpc_GRFs is not None,f"grf is Nonve"
        assert self.current_contact is not None,f"grf is Nonve"
        current_contact=self.current_contact
        self.nmpc_footholds = nmpc_footholds
        ##### Leggatrrdependencies to be remoed
        # self.nmpc_GRFs = LegsAttr(FL=self.nmpc_GRFs[0:3] * self.current_contact[0],
        #                           FR=self.nmpc_GRFs[3:6] * self.current_contact[1],
        #                           RL=self.nmpc_GRFs[6:9] * self.current_contact[2],
        #                           RR=self.nmpc_GRFs[9:12] * self.current_contact[3])        
        self.nmpc_GRFs=nmpc_GRFs
        ###
        return nmpc_footholds, nmpc_GRFs,current_contact

    def get_forward_action(self,simulation: bool,feet_jac: dict,feet_vel: dict,legs_qvel_idx: dict,qpos:list, qvel:list,legs_qfrc_bias:dict,legs_mass_matrix:dict,
                           nmpc_GRFs,nmpc_footholds,tau=None):
        """
        Compute the forward action for the robot based on the provided simulation state and parameters.

        Parameters:
        simulation (bool): Indicates if the simulation is active.
        feet_jac (dict): Jacobian matrices of the feet.
        feet_vel (dict): Feet velocities.
        legs_qvel_idx (dict): Index of the joint velocities.
        qpos (np.ndarray): Generalized positions vectors.
        qvel (np.ndarray): Generalized velocities vector.
        legs_qfrc_bias (dict): Bias forces for the legs.
        legs_mass_matrix (dict): Mass matrix for the legs.
        Returns:
        tau (dict): Torque vector or forward action for the robot.
        """


        
        if simulation == True:
            nmpc_GRFs = nmpc_GRFs
        else:
            ##suff
            breakpoint()
        
        # Compute Stance Torque ---------------------------------------------------------------------------
        # feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False) #come from outside
        # Compute feet velocities
        # Compute jacobian derivatives of the contact points
        self.jac_feet_dot = (feet_jac - self.jac_feet_prev) / self.simulation_dt  # Finite difference approximation
        self.jac_feet_prev = feet_jac  # Update previous Jacobianscomputed
        # Compute the torque with the contact jacobian (-J.T @ f)   J: R^nv -> R^3,   f: R^3
        tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL) #env
        tau.FR = -np.matmul(feet_jac.FR[:, legs_qvel_idx.FR].T, nmpc_GRFs.FR) #env
        tau.RL = -np.matmul(feet_jac.RL[:, legs_qvel_idx.RL].T, nmpc_GRFs.RL) #env
        tau.RR = -np.matmul(feet_jac.RR[:, legs_qvel_idx.RR].T, nmpc_GRFs.RR) #env

        # ---------------------------------------------------------------------------------------------------

        # Compute Swing Torque ------------------------------------------------------------------------------
        # TODO: Move contact sequence to labels FL1, FR1, RL1, RR1 instead of a fixed indexing.
        # The swing controller is in the end-effector space. For its computation,
        # we save for simplicity joints position and velocities
        
        # centrifugal, coriolis, gravity

        self.stc.update_swing_time(self.current_contact, self.legs_order, self.simulation_dt)


        for leg_id, leg_name in enumerate(self.legs_order):
            if self.current_contact[leg_id] == 0:  # If in swing phase, compute the swing trajectory tracking control.
                tau[leg_name], _, _ = self.stc.compute_swing_control(
                    leg_id=leg_id,
                    q_dot=qvel[legs_qvel_idx[leg_name]],
                    J=feet_jac[leg_name][:, legs_qvel_idx[leg_name]],
                    J_dot=self.jac_feet_dot[leg_name][:,legs_qvel_idx[leg_name]],
                    lift_off=self.frg.lift_off_positions[leg_name],
                    touch_down=nmpc_footholds[leg_name],
                    foot_pos=self.feet_pos[leg_name],
                    foot_vel=feet_vel[leg_name],
                    h=legs_qfrc_bias[leg_name],
                    mass_matrix=legs_mass_matrix[leg_name]
                    )
        lift_off_position=self.frg.lift_off_positions
        return tau, lift_off_position    

    def reset(self):
        self.pgg.reset()
        self.frg.lift_off_positions =  self.lift_off_positions
        self.current_contact = np.array([0, 0, 0, 0])
        self.previous_contact = np.asarray(self.current_contact)


        


