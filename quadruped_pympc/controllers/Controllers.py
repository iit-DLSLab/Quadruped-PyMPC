import os
import time

# TODO: Ugly hack so people dont have to run the python command specifying the working directory.
#  we should remove this before the final release.
import sys

# Add the parent directory of this script to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco
import numpy as np


# Parameters for both MPC and simulation
from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.terrain_estimator import TerrainEstimator
from quadruped_pympc.helpers.quadruped_utils import GaitType
from gym_quadruped.utils.quadruped_utils import LegsAttr





####
    
#------------
class Controller:
    #constructor
    def __init__(self,
                 env,
                 gait_name=None,
                 sim_dt=None,
                 hip_height=None,
                 legs_order=None,
                 step_height=None,
                 swing_position_gain_fb=None,
                 swing_velocity_gain_fb=None,
                 swing_generator=None,
                 mpc_frequency= None,
                 mpc_dt=None,
                 controller_type=None,
                 horizon=None,
                 optimize_step_freq=None,
                 step_freq_available=None,
                 sampling_method=None,
                 control_parametrization=None,
                 sigma_cem_mppi=None,
                 num_parallel_computations=None,
                 num_sampling_iterations=None,
                 use_nonuniform_discretization=False,
                 shift_solution=None,
                 dt_fine_grained=None,
                 horizon_fine_grained=None,
                 use_inertia_recomputation=None,
                 inertia=None):
        #
        self.env=env
        self.gait_name=gait_name
        self.simulation_dt = sim_dt
        self.hip_height = hip_height
        self.legs_order = legs_order
        self.ref_z=hip_height
        self.inertia=inertia
        self.use_inertia_recomputation=use_inertia_recomputation
        self.sigma_cem_mppi=sigma_cem_mppi
        self.use_nonuniform_discretization=use_nonuniform_discretization

        ### All this parameter are for the WBC 
        self.step_height = step_height
        self.position_gain_fb = swing_position_gain_fb
        self.velocity_gain_fb = swing_velocity_gain_fb
        self.swing_generator = swing_generator

        # MPC parameters
        self.mpc_frequency = mpc_frequency
        self.mpc_dt = mpc_dt
        self.horizon = horizon
        self.controller_type= controller_type
        self.optimize_step_frequency= optimize_step_freq
        self.step_freq_available = step_freq_available
        if self.controller_type == 'sampling':
            sampling_method = sampling_method
            control_parametrization = control_parametrization
            num_parallel_computations = num_parallel_computations
            self.iteration = num_sampling_iterations
            shift_solution = shift_solution
        
        if use_nonuniform_discretization==True :
            dt_fine_grained = dt_fine_grained
            horizon_fine_grained = horizon_fine_grained


        self.stance_time=0
        self.swing_period = 0

        self.nmpc_grf_old=None



        # Initialization of variables used in the main control loop
        # ____________________________________________________________
        # Set the reference for the state
        self.ref_pose = np.array([0, 0, hip_height])
        self.ref_base_lin_vel, self.ref_base_ang_vel = env.target_base_vel()
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

        # Jacobian matrices
        self.jac_feet_prev = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
        self.jac_feet_dot = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
        # Torque vector
        self.tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])
        # State
        self.state_current, self.state_prev = {}, {}
        self.feet_pos = None


        



    def get_gait_params(self,gait_type: str) -> [GaitType, float, float]:
        if gait_type == "trot":
            step_frequency = 2.5
            duty_factor = 0.65
            gait_type = GaitType.TROT
        elif gait_type == "crawl":
            step_frequency = 0.7
            duty_factor = 0.9
            gait_type = GaitType.BACKDIAGONALCRAWL
        elif gait_type == "pace":
            step_frequency = 2
            duty_factor = 0.7
            gait_type = GaitType.PACE
        elif gait_type == "bound":
            step_frequency = 4
            duty_factor = 0.65
            gait_type = GaitType.BOUNDING
        else:
            step_frequency = 2
            duty_factor = 0.65
            gait_type = GaitType.FULL_STANCE
        return gait_type, duty_factor, step_frequency
    

    def set_controller(self):
        ##############################################
        ############ SET THE CONTROLLER TYPE  ########
        ##############################################

        if(self.controller_type == 'nominal'):
            from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal
            self.controller = Acados_NMPC_Nominal()
            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif(self.controller_type == 'input_rates'):
            from quadruped_pympc.controllers.gradient.input_rates.centroidal_nmpc_input_rates import Acados_NMPC_InputRates

            self.controller = Acados_NMPC_InputRates()

            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif(self.controller_type ==  'sampling'):

            if self.optimize_step_frequency==True:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax_gait_adaptive import Sampling_MPC

            else:
                from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC

            import jax
            import jax.numpy as jnp
            num_parallel_computations = self.num_parallel_computations
            # self.iteration = self.iteration
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




        
    def periodic_gait_generator(self):

        # Periodic gait generator _________________________________________________________________________
        gait_name = self.gait_name
        gait_type, duty_factor, step_frequency = self.get_gait_params(gait_name)
        #
        self.duty_factor = duty_factor
        self.step_frequency = step_frequency
        self.gait_type = gait_type
        self.pgg = PeriodicGaitGenerator(duty_factor=duty_factor, step_freq=step_frequency, gait_type=self.gait_type,
                                        horizon=self.horizon * 2, contact_sequence_dt=self.mpc_dt/2.)
        
        self.contact_sequence = self.pgg.compute_contact_sequence()
        self.nominal_sample_freq = self.step_frequency
        # compute stance and swing tme based on gate selection!
        self.stance_time = (1 / self.step_frequency) * self.duty_factor
        self.swing_period = (1 - self.duty_factor) * (1 / self.step_frequency)  # + 0.07

    

    def initialize(self):
        self.set_controller()
        self.periodic_gait_generator()

        # Create the foothold reference generator
        self.frg = FootholdReferenceGenerator(stance_time=self.stance_time, hip_height=self.hip_height,
                                              lift_off_positions= self.env.feet_pos(frame='world'))

        # Create swing trajectory generator

        self.stc = SwingTrajectoryController(step_height=self.step_height, swing_period=self.swing_period,
                                        position_gain_fb=self.position_gain_fb, velocity_gain_fb=self.velocity_gain_fb,
                                        generator=self.swing_generator)
        
        # Online computation of the inertia parameter
        # Terrain estimator
        self.terrain_computation = TerrainEstimator()


    def update_mpc_reference(self):

        # Update the robot state --------------------------------
        self.feet_pos = self.env.feet_pos(frame='world')
        self.hip_pos = self.env.hip_positions(frame='world')
        self.base_lin_vel=self.env.base_lin_vel
        self.base_ang_vel = self.env.base_ang_vel

        self.state_current = dict(
            position=self.env.base_pos,
            linear_velocity=self.env.base_lin_vel,
            orientation=self.env.base_ori_euler_xyz,
            angular_velocity=self.env.base_ang_vel,
            foot_FL=self.feet_pos.FL,
            foot_FR=self.feet_pos.FR,
            foot_RL=self.feet_pos.RL,
            foot_RR=self.feet_pos.RR
            )

        # Update target base velocity
        self.ref_base_lin_vel, self.ref_base_ang_vel = self.env.target_base_vel()
        # -------------------------------------------------------

        # Update the desired contact sequence ---------------------------
        self.pgg.run(self.simulation_dt, self.pgg.step_freq)
        self.contact_sequence = self.pgg.compute_contact_sequence()

        # in the case of nonuniform discretization, we need to subsample the contact sequence
        if self.use_nonuniform_discretization==True:
            self.contact_sequence = self.pgg.sample_contact_sequence(self.contact_sequence, self.mpc_dt, dt_fine_grained, horizon_fine_grained)
        

        previous_contact = self.current_contact
        self.current_contact = np.array([self.contact_sequence[0][0],
                                    self.contact_sequence[1][0],
                                    self.contact_sequence[2][0],
                                    self.contact_sequence[3][0]])

        # Compute the reference for the footholds ---------------------------------------------------
        self.frg.update_lift_off_positions(previous_contact, self.current_contact, self.feet_pos, self.legs_order)
        self.ref_feet_pos = self.frg.compute_footholds_reference(
            com_position=self.env.base_pos,
            base_ori_euler_xyz=self.env.base_ori_euler_xyz,
            base_xy_lin_vel=self.env.base_lin_vel[0:2],
            ref_base_xy_lin_vel=self.ref_base_lin_vel[0:2],
            hips_position=self.hip_pos,
            com_height_nominal=self.ref_z)

        # Estimate the terrain slope and elevation -------------------------------------------------------
        terrain_roll, \
            terrain_pitch, \
            terrain_height = self.terrain_computation.compute_terrain_estimation(
            base_position=self.env.base_pos,
            yaw=self.env.base_ori_euler_xyz[2],
            feet_pos=self.frg.lift_off_positions,
            current_contact=self.current_contact)

        self.ref_pos = np.array([0, 0, self.hip_height])
        self.ref_pos[2] = self.ref_z+ terrain_height

        # Update state reference ------------------------------------------------------------------------
        self.ref_state |= dict(ref_foot_FL=self.ref_feet_pos.FL.reshape((1, 3)),
                          ref_foot_FR=self.ref_feet_pos.FR.reshape((1, 3)),
                          ref_foot_RL=self.ref_feet_pos.RL.reshape((1, 3)),
                          ref_foot_RR=self.ref_feet_pos.RR.reshape((1, 3)),
                          # Also update the reference base linear velocity and
                          ref_linear_velocity=self.ref_base_lin_vel,
                          ref_angular_velocity=self.ref_base_ang_vel,
                          ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0]),
                          ref_position=self.ref_pos
                          )
        # -------------------------------------------------------------------------------------------------

    def solve_MPC_main_loop(self,env=None):
        # Solve OCP ---------------------------------------------------------------------------------------
        if self.env.step_num % round(1 / (self.mpc_frequency * self.simulation_dt)) == 0:
            print("Solving MPC")

            # We can recompute the inertia of the single rigid body model

            # or use the fixed one in cfg.py
            if self.use_inertia_recomputation==True:
                # TODO: d.qpos is not defined
                #inertia = srb_inertia_computation.compute_inertia(d.qpos)
                inertia = self.env.get_base_inertia().flatten()  # Reflected inertia of base at qpos, in world frame
            else:
                inertia = self.inertia.flatten()

            if (self.optimize_step_frequency == True):
                # we can always optimize the step freq, or just at the apex of the swing
                # to avoid possible jittering in the solution
                optimize_swing = 0  # 1 for always, 0 for apex
                for leg_id in range(4):
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

                time_start = time.time()

                # Shift the previous solution ahead
                if (self.shift_solution == True):
                    index_shift = 1./self.mpc_frequency
                    best_control_parameters = self.controller.shift_solution(best_control_parameters, index_shift)
                
                # Convert data to jax
                state_current_jax, \
                    reference_state_jax, \
                    best_control_parameters = self.jitted_prepare_state_and_reference(self.state_current, self.ref_state,
                                                                                 best_control_parameters,
                                                                                 self.current_contact, previous_contact_mpc)

                for iter_sampling in range(self.iteration):
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

                nmpc_footholds = self.ref_feet_pos #todo return ref feet pos from the controller

                nmpc_GRFs = np.array(nmpc_GRFs)

                previous_contact_mpc = self.current_contact
                index_shift = 0
                # optimizer_cost = best_cost

            # If we use Gradient-Based MPC
            else:
                time_start = time.time()
                self.nmpc_GRFs, nmpc_footholds, _, status = self.controller.compute_control(
                    self.state_current,
                    self.ref_state,
                    self.contact_sequence,
                    inertia=inertia)
                nmpc_GRFs, nmpc_footholds, _, status = self.controller.compute_control(
                    self.state_current,
                    self.ref_state,
                    self.contact_sequence,
                    inertia=inertia)
                # TODO functions should output this class instance.
                nmpc_footholds = LegsAttr(FL=nmpc_footholds[0],
                                          FR=nmpc_footholds[1],
                                          RL=nmpc_footholds[2],
                                          RR=nmpc_footholds[3])
                self.nmpc_footholds = nmpc_footholds

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
       

                    costs, best_sample_freq = self.batched_controller.compute_batch_control(self.state_current, ref_state,
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
            
            # TODO: Indexing should not be hardcoded. self.env should provide indexing of leg actuator dimensions.
            nmpc_GRFs = LegsAttr(FL=nmpc_GRFs[0:3] * self.current_contact[0],
                                 FR=nmpc_GRFs[3:6] * self.current_contact[1],
                                 RL=nmpc_GRFs[6:9] * self.current_contact[2],
                                 RR=nmpc_GRFs[9:12] * self.current_contact[3])
            self.nmpc_GRFs = LegsAttr(FL=self.nmpc_GRFs[0:3] * self.current_contact[0],
                                      FR=self.nmpc_GRFs[3:6] * self.current_contact[1],
                                      RL=self.nmpc_GRFs[6:9] * self.current_contact[2],
                                      RR=self.nmpc_GRFs[9:12] * self.current_contact[3])
            # print("nmpc_GRFs: ", nmpc_GRFs.FL)
            # print("nmpc_GRFs: ", self.nmpc_GRFs)

    
    def get_forward_action(self,simulation=True,env=None):
        if simulation == True:
            nmpc_GRFs = self.nmpc_GRFs
        else:
            ##suff
            breakpoint()
        
        # Compute Stance Torque ---------------------------------------------------------------------------
        feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)
        # Compute feet velocities
        feet_vel = LegsAttr(**{leg_name: feet_jac[leg_name] @ self.env.mjData.qvel for leg_name in self.legs_order})
        # Compute jacobian derivatives of the contact points
        jac_feet_dot = (feet_jac - self.jac_feet_prev) / self.simulation_dt  # Finite difference approximation
        self.jac_feet_prev = feet_jac  # Update previous Jacobians
        # Compute the torque with the contact jacobian (-J.T @ f)   J: R^nv -> R^3,   f: R^3
        self.tau.FL = -np.matmul(feet_jac.FL[:, self.env.legs_qvel_idx.FL].T, nmpc_GRFs.FL)
        self.tau.FR = -np.matmul(feet_jac.FR[:, self.env.legs_qvel_idx.FR].T, nmpc_GRFs.FR)
        self.tau.RL = -np.matmul(feet_jac.RL[:, self.env.legs_qvel_idx.RL].T, nmpc_GRFs.RL)
        self.tau.RR = -np.matmul(feet_jac.RR[:, self.env.legs_qvel_idx.RR].T, nmpc_GRFs.RR)

        # ---------------------------------------------------------------------------------------------------

        # Compute Swing Torque ------------------------------------------------------------------------------
        # TODO: Move contact sequence to labels FL, FR, RL, RR instead of a fixed indexing.
        # The swing controller is in the end-effector space. For its computation,
        # we save for simplicity joints position and velocities
        qpos, qvel = self.env.mjData.qpos, self.env.mjData.qvel
        # centrifugal, coriolis, gravity
        legs_mass_matrix = self.env.legs_mass_matrix
        legs_qfrc_bias = self.env.legs_qfrc_bias
        self.stc.update_swing_time(self.current_contact, self.legs_order, self.simulation_dt)

        for leg_id, leg_name in enumerate(self.legs_order):
            if self.current_contact[leg_id] == 0:  # If in swing phase, compute the swing trajectory tracking control.
                self.tau[leg_name], _, _ = self.stc.compute_swing_control(
                    leg_id=leg_id,
                    q_dot=qvel[self.env.legs_qvel_idx[leg_name]],
                    J=feet_jac[leg_name][:, self.env.legs_qvel_idx[leg_name]],
                    J_dot=jac_feet_dot[leg_name][:, self.env.legs_qvel_idx[leg_name]],
                    lift_off=self.frg.lift_off_positions[leg_name],
                    touch_down=self.nmpc_footholds[leg_name],
                    foot_pos=self.feet_pos[leg_name],
                    foot_vel=feet_vel[leg_name],
                    h=legs_qfrc_bias[leg_name],
                    mass_matrix=legs_mass_matrix[leg_name]
                    )
        tau=self.tau
        return tau    

    def reset(self,env):
        self.pgg.reset()
        self.frg.lift_off_positions = env.feet_pos(frame='world')
        self.current_contact = np.array([0, 0, 0, 0])
        self.previous_contact = np.asarray(self.current_contact)


        


