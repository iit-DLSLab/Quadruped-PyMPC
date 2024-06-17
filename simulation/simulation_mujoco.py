# Description: This script is used to simulate the full model of the robot in mujoco
from pathlib import Path

# Authors: Giulio Turrisi -

import mujoco.viewer
import mujoco
import copy
import time
import pprint
import casadi as cs
import numpy as np

from quadruped_utils import LegsAttr
from simulation.quadruped_env import QuadrupedEnv

np.set_printoptions(precision=3, suppress=True)

import os

dir_path = os.path.dirname(os.path.realpath(__file__))
os.environ['XLA_FLAGS'] = ('--xla_gpu_triton_gemm_any=True')

import sys

sys.path.append(dir_path + '/../')
sys.path.append(dir_path + '/../helpers/')
sys.path.append(dir_path + '/../helpers/swing_generators/')
sys.path.append(dir_path + '/../gradient/')
sys.path.append(dir_path + '/../gradient/input_rates/')
sys.path.append(dir_path + '/../gradient/nominal/')
sys.path.append(dir_path + '/../gradient/collaborative/')
sys.path.append(dir_path + '/../sampling/')

import threading
import readchar

# General magic
from foothold_reference_generator import FootholdReferenceGenerator
from swing_trajectory_controller import SwingTrajectoryController
from periodic_gait_generator import PeriodicGaitGenerator, Gait
from srb_inertia_computation import SrbInertiaComputation
from terrain_estimator import TerrainEstimator
from other import euler_from_quaternion, plot_swing_mujoco, plot_state_matplotlib

# Parameters for both MPC and simulation
import config as cfg

robot_name = cfg.robot
scene_name = cfg.simulation_params['scene']

env = QuadrupedEnv(robot=robot_name, scene=scene_name)
env.reset()
env.render()
mass = np.sum(env.mjModel.body_mass)

# MPC Magic - i took the minimum value of the dt
# used along the horizon of the MPC
if (cfg.mpc_params['use_nonuniform_discretization']):
    mpc_dt = cfg.mpc_params['dt_fine_grained']
else:
    mpc_dt = cfg.mpc_params['dt']
horizon = cfg.mpc_params['horizon']

# input_rates optimize the delta_GRF (smoooth!)
# nominal optimize directly the GRF (not smooth)
# sampling use GPUUUU
# collaborative optimize directly the GRF and has a passive arm model inside
if (cfg.mpc_params['type'] == 'nominal'):
    from centroidal_nmpc_nominal import Acados_NMPC_Nominal

    controller = Acados_NMPC_Nominal()

    if (cfg.mpc_params['optimize_step_freq']):
        from centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive

        batched_controller = Acados_NMPC_GaitAdaptive()

elif (cfg.mpc_params['type'] == 'input_rates'):
    from centroidal_nmpc_input_rates import Acados_NMPC_InputRates

    controller = Acados_NMPC_InputRates()

    if (cfg.mpc_params['optimize_step_freq']):
        from centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive

        batched_controller = Acados_NMPC_GaitAdaptive()

elif (cfg.mpc_params['type'] == 'sampling'):
    if (cfg.mpc_params['optimize_step_freq']):
        from sampling.centroidal_nmpc_jax_gait_adaptive import Sampling_MPC
    else:
        from sampling.centroidal_nmpc_jax import Sampling_MPC

    import jax
    import jax.numpy as jnp

    num_parallel_computations = cfg.mpc_params['num_parallel_computations']
    iteration = cfg.mpc_params['num_sampling_iterations']
    controller = Sampling_MPC(horizon=horizon,
                              dt=mpc_dt,
                              num_parallel_computations=num_parallel_computations,
                              sampling_method=cfg.mpc_params['sampling_method'],
                              control_parametrization=cfg.mpc_params['control_parametrization'],
                              device="gpu")
    best_control_parameters = jnp.zeros((controller.num_control_parameters,))
    jitted_compute_control = jax.jit(controller.compute_control, device=controller.device)
    jitted_get_key = jax.jit(controller.get_key, device=controller.device)
    jitted_prepare_state_and_reference = controller.prepare_state_and_reference

    index_shift = 0

# Tracking mpc error (z, x_dot, y_dot, z_dot, roll, pitch, roll_dot, pitch_dot, yaw_dot)
mean_tracking = np.zeros((9,))
mean_num_sample = 1
mean_optimizer_cost = 0
optimizer_cost = 0

# Get the ids of the contact points, useful for computing
# contact jacobian later
FL_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
FR_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
RL_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
RR_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_GEOM, 'RR')
FL_geom = env.mjModel.geom("FL")
FR_geom = env.mjModel.geom("FR")
RL_geom = env.mjModel.geom("RL")
RR_geom = env.mjModel.geom("RR")
FL_calf_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FL_calf')
FR_calf_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FR_calf')
RL_calf_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RL_calf')
RR_calf_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RR_calf')

# Jacobian matrices
jac_com = np.zeros((3, env.mjModel.nv))
jac_feet = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
jac_feet_prev = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
jac_feet_dot = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])

# Previous foot positions
feet_pos, feet_pos_prev = env.feet_pos(), env.feet_pos()

# Torque vectors
tau_FL = np.zeros((env.mjModel.nv, 1))
tau_FR = np.zeros((env.mjModel.nv, 1))
tau_RL = np.zeros((env.mjModel.nv, 1))
tau_RR = np.zeros((env.mjModel.nv, 1))

# State
state_current = np.zeros(shape=(controller.states_dim,))
state_old = np.zeros(shape=(controller.states_dim,))

# Simulation dt
env.mjModel.opt.timestep = cfg.simulation_params['dt']
simulation_dt = cfg.simulation_params['dt']
mpc_frequency = cfg.simulation_params['mpc_frequency']

# Periodic gait generator
gait = cfg.simulation_params['gait']
if (gait == "trot"):
    step_frequency = 1.3
    duty_factor = 0.65
    p_gait = Gait.TROT
elif (gait == "crawl"):
    step_frequency = 0.7
    duty_factor = 0.9
    p_gait = Gait.BACKDIAGONALCRAWL
elif (gait == "pace"):
    step_frequency = 2
    duty_factor = 0.7
    p_gait = Gait.PACE
elif (gait == "bound"):
    step_frequency = 2
    duty_factor = 0.65
    p_gait = Gait.BOUNDING
else:
    step_frequency = 2
    duty_factor = 0.65
    p_gait = Gait.FULL_STANCE
    print("FULL STANCE")
# Given the possibility to use nonuniform discretization, 
# we generate a contact sequence two times longer
pgg = PeriodicGaitGenerator(duty_factor=duty_factor, step_freq=step_frequency, p_gait=p_gait, horizon=horizon * 2)
contact_sequence = pgg.compute_contact_sequence(mpc_dt=mpc_dt, simulation_dt=simulation_dt)
nominal_sample_freq = step_frequency

# Create the foothold reference generator
stance_time = (1 / step_frequency) * duty_factor
frg = FootholdReferenceGenerator(stance_time=stance_time)

# Hip position
FL_hip_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
FR_hip_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
RL_hip_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
RR_hip_id = mujoco.mj_name2id(env.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')
hip_pos = np.array(([env.mjData.body(FL_hip_id).xpos,
                     env.mjData.body(FR_hip_id).xpos,
                     env.mjData.body(RL_hip_id).xpos,
                     env.mjData.body(RR_hip_id).xpos]))

# Set the reference for the state
ref_pose = np.array([0, 0, cfg.simulation_params['ref_z']])
ref_linear_velocity = env.target_base_vel
ref_orientation = np.array([cfg.simulation_params['ref_roll'], cfg.simulation_params['ref_pitch'], 0])
ref_angular_velocity = np.array([0, 0, cfg.simulation_params['ref_yaw_dot']])

# # SET REFERENCE AS DICTIONARY
# TODO: I would suggest to create a DataClass for "BaseConfig"
ref_state = {'ref_position':         ref_pose,
             'ref_linear_velocity':  ref_linear_velocity,
             'ref_orientation':      ref_orientation,
             'ref_angular_velocity': ref_angular_velocity,
             }
# Create swing trajectory generator
step_height = cfg.simulation_params['step_height']
swing_period = (1 - duty_factor) * (1 / step_frequency)  # + 0.07
position_gain_fb = cfg.simulation_params['swing_position_gain_fb']
velocity_gain_fb = cfg.simulation_params['swing_velocity_gain_fb']
swing_generator = cfg.simulation_params['swing_generator']
stc = SwingTrajectoryController(step_height=step_height, swing_period=swing_period,
                                position_gain_fb=position_gain_fb, velocity_gain_fb=velocity_gain_fb,
                                generator=swing_generator)

# Swing controller variables
swing_time = [0, 0, 0, 0]
lift_off_positions = env.feet_pos(frame='world')

# Online computation of the inertia parameter
srb_inertia_computation = SrbInertiaComputation()
inertia = cfg.inertia

# Terrain estimator
z_foot_mean = 0.0
terrain_computation = TerrainEstimator()

# Starting contact sequence
previous_contact = np.array([1, 1, 1, 1])
previous_contact_mpc = np.array([1, 1, 1, 1])
current_contact = np.array([1, 1, 1, 1])

nmpc_GRFs = np.zeros((12,))
nmpc_wrenches = np.zeros((6,))
nmpc_footholds = np.zeros((12,))
nmpc_predicted_state = None

use_print_debug = cfg.simulation_params['use_print_debug']
use_visualization_debug = cfg.simulation_params['use_visualization_debug']

# Keyboard control
# def interactive_command_line():
#     global ref_linear_velocity, ref_angular_velocity, step_height
#     while True:
#         # command = input()
#         command = readchar.readkey()
#         if (command == "w"):
#             ref_linear_velocity[0] += 0.1
#         elif (command == "s"):
#             ref_linear_velocity[0] -= 0.1
#         elif (command == "a"):
#             ref_linear_velocity[1] += 0.1
#         elif (command == "d"):
#             ref_linear_velocity[1] -= 0.1
#         elif (command == "q"):
#             ref_angular_velocity[2] += 0.1
#         elif (command == "e"):
#             ref_angular_velocity[2] -= 0.1
#         elif (command == "0"):
#             ref_linear_velocity[0] = 0
#             ref_linear_velocity[1] = 0
#             ref_angular_velocity[2] = 0
#         elif (command == "+"):
#             step_height += 0.01
#             stc.step_height = step_height
#             stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)
#         elif (command == "-"):
#             step_height -= 0.01
#             stc.step_height = step_height
#             stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)
#
#
# t1 = threading.Thread(target=interactive_command_line)
# t1.daemon = True
# t1.start()

FL_idx, FR_idx, RL_idx, RR_idx = 0, 1, 2, 3
legs_order = ["FL", "FR", "RL", "RR"]
# Main simulation loop ------------------------------------------------------------------
while True:
    step_start = time.time()
    if (use_print_debug):
        print("###############")

    # Update the robot state --------------------------------
    feet_pos = env.feet_pos(frame='world')
    hip_pos = env.hip_positions(frame='world')

    state_current = dict(
        position=env.base_pos,
        linear_velocity=env.base_lin_vel,
        orientation=env.base_ori_euler_xyz,
        angular_velocity=env.base_ang_vel,
        foot_FL=feet_pos.FL,
        foot_FR=feet_pos.FR,
        foot_RL=feet_pos.RL,
        foot_RR=feet_pos.RR
        )
    # -------------------------------------------------------

    # Update the contact sequence ---------------------------
    if (gait == "full_stance"):
        contact_sequence = np.ones((4, horizon * 2))
    else:
        contact_sequence = pgg.compute_contact_sequence(mpc_dt=mpc_dt, simulation_dt=simulation_dt)

    # in the case of nonuniform discretization, we need to subsample the contact sequence
    if (cfg.mpc_params['use_nonuniform_discretization']):
        subsample_step_contact_sequence = int(cfg.mpc_params['dt_fine_grained'] / mpc_dt)
        if (subsample_step_contact_sequence > 1):
            contact_sequence_fine_grained = contact_sequence[:, ::subsample_step_contact_sequence][:,
                                            0:cfg.mpc_params['horizon_fine_grained']]
        else:
            contact_sequence_fine_grained = contact_sequence[:, 0:cfg.mpc_params['horizon_fine_grained']]

        subsample_step_contact_sequence = int(cfg.mpc_params['dt'] / mpc_dt)
        if (subsample_step_contact_sequence > 1):
            contact_sequence = contact_sequence[:, ::subsample_step_contact_sequence]
        contact_sequence = contact_sequence[:, 0:horizon]
        contact_sequence[:, 0:cfg.mpc_params['horizon_fine_grained']] = contact_sequence_fine_grained

    previous_contact = copy.deepcopy(current_contact)
    current_contact = np.array([contact_sequence[0][0],
                                contact_sequence[1][0],
                                contact_sequence[2][0],
                                contact_sequence[3][0]])

    # Compute the reference for the footholds ---------------------------------------------------
    ref_feet_pos = frg.compute_footholds_reference(
        com_position=env.base_pos,
        rpy_angles=env.base_ori_euler_xyz,
        linear_com_velocity=env.base_lin_vel[0:2],
        desired_linear_com_velocity=env.target_base_vel[0:2],
        hips_position=hip_pos,
        com_height=state_current["position"][2],
        lift_off_positions=lift_off_positions)

    # Update state reference
    ref_state |= dict(ref_foot_FL=ref_feet_pos.FL.reshape((1, 3)),
                      ref_foot_FR=ref_feet_pos.FR.reshape((1, 3)),
                      ref_foot_RL=ref_feet_pos.RL.reshape((1, 3)),
                      ref_foot_RR=ref_feet_pos.RR.reshape((1, 3)),
                      # Also update the reference base linear velocity and # TODO: orientation.
                      ref_linear_velocity=env.target_base_vel)
    # -------------------------------------------------------------------------------------------------
    # Compute the terrain estimation and reference height ---------------------------------------------
    roll, pitch = terrain_computation.compute_terrain_estimation(
        base_position=env.base_pos,
        yaw=env.base_ori_euler_xyz[2],
        lift_foot_position=lift_off_positions)
    ref_state["ref_orientation"] = np.array([roll, pitch, 0])

    # Update the reference height given the foot in contact
    num_feet_in_contact = np.sum(current_contact)
    feet_pos_z = np.asarray(feet_pos.to_list(order=legs_order))[:, 2]
    if num_feet_in_contact != 0:
        z_foot_mean_temp = np.sum(feet_pos_z * current_contact) / num_feet_in_contact
        z_foot_mean = z_foot_mean_temp * 0.4 + z_foot_mean * 0.6
    ref_state["ref_position"][2] = cfg.simulation_params['ref_z'] + z_foot_mean
    # -------------------------------------------------------------------------------------------------

    # TODO: WTF is this ? Need documentation
    if cfg.mpc_params['type'] == 'sampling':
        if cfg.mpc_params['shift_solution']:
            index_shift += 0.05
            best_control_parameters = controller.shift_solution(best_control_parameters, index_shift)

    # Solve OCP ---------------------------------------------------------------------------------------
    if env.step_num % round(1 / (mpc_frequency * simulation_dt)) == 0:

        # We can recompute the inertia of the single rigid body model
        # or use the fixed one in cfg.py
        if (cfg.simulation_params['use_inertia_recomputation']):
            # TODO: d.qpos is not defined
            inertia = srb_inertia_computation.compute_inertia(d.qpos)

        if ((cfg.mpc_params['optimize_step_freq'])):
            # we can always optimize the step freq, or just at the apex of the swing
            # to avoid possible jittering in the solution
            optimize_swing = 0  # 1 for always, 0 for apex
            for leg_id in range(4):
                # Swing time check
                if (current_contact[leg_id] == 0):
                    if ((swing_time[leg_id] > (swing_period / 2.) - 0.02) and \
                            (swing_time[leg_id] < (swing_period / 2.) + 0.02)):
                        optimize_swing = 1
                        nominal_sample_freq = step_frequency

        # If we use sampling
        if (cfg.mpc_params['type'] == 'sampling'):

            time_start = time.time()
            # Convert data to jax
            state_current_jax, \
                reference_state_jax, \
                best_control_parameters = jitted_prepare_state_and_reference(state_current, ref_state,
                                                                             best_control_parameters,
                                                                             current_contact, previous_contact_mpc)

            for iter_sampling in range(iteration):
                if (cfg.mpc_params['sampling_method'] == 'cem_mppi'):
                    if (iter_sampling == 0):
                        controller = controller.with_newsigma(cfg.mpc_params['sigma_cem_mppi'])
                    nmpc_GRFs, \
                        nmpc_footholds, \
                        best_control_parameters, \
                        best_cost, \
                        best_sample_freq, \
                        costs, \
                        sigma_cem_mppi = jitted_compute_control(state_current_jax, reference_state_jax,
                                                                contact_sequence, best_control_parameters,
                                                                controller.master_key, controller.sigma_cem_mppi)
                    controller = controller.with_newsigma(sigma_cem_mppi)
                else:
                    nmpc_GRFs, \
                        nmpc_footholds, \
                        best_control_parameters, \
                        best_cost, \
                        best_sample_freq, \
                        costs = jitted_compute_control(state_current_jax, reference_state_jax, contact_sequence,
                                                       best_control_parameters, controller.master_key, pgg.get_t(),
                                                       nominal_sample_freq, optimize_swing)

                controller = controller.with_newkey()

            if ((cfg.mpc_params['optimize_step_freq']) and (optimize_swing == 1)):
                pgg.step_freq = np.array([best_sample_freq])[0]
                nominal_sample_freq = pgg.step_freq
                stance_time = (1 / pgg.step_freq) * duty_factor
                frg.stance_time = stance_time

                swing_period = (1 - duty_factor) * (1 / pgg.step_freq)  # + 0.07
                stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)

            nmpc_footholds = np.zeros((4, 3))
            nmpc_footholds[0] = ref_state["ref_foot_FL"]
            nmpc_footholds[1] = ref_state["ref_foot_FR"]
            nmpc_footholds[2] = ref_state["ref_foot_RL"]
            nmpc_footholds[3] = ref_state["ref_foot_RR"]

            nmpc_GRFs = np.array(nmpc_GRFs)

            previous_contact_mpc = copy.deepcopy(current_contact)
            index_shift = 0
            optimizer_cost = best_cost


        # If we use Gradient-Based MPC
        else:

            time_start = time.time()
            nmpc_GRFs, \
                nmpc_footholds, \
                nmpc_predicted_state, \
                status = controller.compute_control(state_current, ref_state, contact_sequence,
                                                    inertia=inertia.flatten())

            optimizer_cost = controller.acados_ocp_solver.get_cost()

            if ((cfg.mpc_params['optimize_step_freq']) and (optimize_swing == 1)):
                contact_sequence_temp = np.zeros((len(cfg.mpc_params['step_freq_available']), 4, horizon * 2))

                for j in range(len(cfg.mpc_params['step_freq_available'])):
                    pgg_temp = PeriodicGaitGenerator(duty_factor=duty_factor,
                                                     step_freq=cfg.mpc_params['step_freq_available'][j], p_gait=p_gait,
                                                     horizon=horizon * 2)
                    pgg_temp.t = copy.deepcopy(pgg.t)
                    pgg_temp.init = copy.deepcopy(pgg.init)
                    contact_sequence_temp[j] = pgg_temp.compute_contact_sequence(mpc_dt=mpc_dt,
                                                                                 simulation_dt=simulation_dt)

                costs, best_sample_freq = batched_controller.compute_batch_control(state_current, ref_state,
                                                                                   contact_sequence_temp)

                pgg.step_freq = best_sample_freq
                stance_time = (1 / pgg.step_freq) * duty_factor
                frg.stance_time = stance_time
                swing_period = (1 - duty_factor) * (1 / pgg.step_freq)  # + 0.07
                stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)

            # If the controller is using RTI, we need to linearize the mpc after its computation
            # this helps to minize the delay between new state->control, but only in a real case.
            # Here we are in simulation and does not make any difference for now
            if (controller.use_RTI):
                # preparation phase
                controller.acados_ocp_solver.options_set('rti_phase', 1)
                status = controller.acados_ocp_solver.solve()
                print("preparation phase time: ", controller.acados_ocp_solver.get_stats('time_tot'))

        if (use_print_debug):
            mean_tracking[0] = mean_tracking[0] + np.abs(
                state_current["position"][2] - ref_state["ref_position"][2])
            mean_tracking[1] = mean_tracking[1] + np.abs(
                state_current["linear_velocity"][0] - ref_state["ref_linear_velocity"][0])
            mean_tracking[2] = mean_tracking[2] + np.abs(
                state_current["linear_velocity"][1] - ref_state["ref_linear_velocity"][1])
            mean_tracking[3] = mean_tracking[3] + np.abs(
                state_current["linear_velocity"][2] - ref_state["ref_linear_velocity"][2])
            mean_tracking[4] = mean_tracking[4] + np.abs(
                state_current["orientation"][0] - ref_state["ref_orientation"][0])
            mean_tracking[5] = mean_tracking[5] + np.abs(
                state_current["orientation"][1] - ref_state["ref_orientation"][1])
            mean_tracking[6] = mean_tracking[6] + np.abs(
                state_current["angular_velocity"][0] - ref_state["ref_angular_velocity"][0])
            mean_tracking[7] = mean_tracking[7] + np.abs(
                state_current["angular_velocity"][1] - ref_state["ref_angular_velocity"][1])
            mean_tracking[8] = mean_tracking[8] + np.abs(
                state_current["angular_velocity"][2] - ref_state["ref_angular_velocity"][2])
            print("mean_tracking: ", mean_tracking / (mean_num_sample))
            mean_optimizer_cost = mean_optimizer_cost + optimizer_cost
            print("mean_cost: ", mean_optimizer_cost / (mean_num_sample))
            mean_num_sample = mean_num_sample + 1

        # Put the GRFs to zero if the foot is not in contact
        nmpc_GRFs[0:3] = nmpc_GRFs[0:3] * current_contact[0]
        nmpc_GRFs[3:6] = nmpc_GRFs[3:6] * current_contact[1]
        nmpc_GRFs[6:9] = nmpc_GRFs[6:9] * current_contact[2]
        nmpc_GRFs[9:12] = nmpc_GRFs[9:12] * current_contact[3]

        # Compute wrench. This goes to the estimator!
        wrench_linear = nmpc_GRFs[0:3] + \
                        nmpc_GRFs[3:6] + \
                        nmpc_GRFs[6:9] + \
                        nmpc_GRFs[9:12]

        wrench_angular = cs.skew(state_current["foot_FL"] - state_current["position"]) @ nmpc_GRFs[0:3] + \
                         cs.skew(state_current["foot_FR"] - state_current["position"]) @ nmpc_GRFs[3:6] + \
                         cs.skew(state_current["foot_RL"] - state_current["position"]) @ nmpc_GRFs[6:9] + \
                         cs.skew(state_current["foot_RR"] - state_current["position"]) @ nmpc_GRFs[9:12]
        wrench_angular = np.array(wrench_angular)

        nmpc_wrenches = np.concatenate((wrench_linear, wrench_angular.flatten()), axis=0)

    if (use_print_debug):
        print("nmpc_GRFs: \n", nmpc_GRFs)
        print("nmpc_footholds: \n", nmpc_footholds)
    # -------------------------------------------------------------------------------------------------

    # Compute Stance Torque ---------------------------------------------------------------------------
    # Compute the jacobian of the contact points
    # TODO: Comment should explain this Jacobian, is this the Jacobian of the contact points?

    jac_feet = env.feet_jacobians(frame='world', return_rot_jac=False)
    # Compute jacobian derivatives of the contact points
    # TODO: Mujoco offers analytical derivatives of the Jacobian, why finite difference?
    jac_feet_dot = (jac_feet - jac_feet_prev) / simulation_dt   # Finite difference approximation
    jac_feet_prev = copy.deepcopy(jac_feet)  # Update previous Jacobians
    # Compute the torque with the contact jacobian (-J*f)
    # index 6:9 -> FL, 9:12 -> FR, 12:15 -> RL, 15:18 -> RR  # TODO: This should not be hardcoded.
    tau_FL = -np.matmul(jac_feet.FL[:, 6:9].T,   nmpc_GRFs[0:3]) * current_contact[0]
    tau_FR = -np.matmul(jac_feet.FR[:, 9:12].T,  nmpc_GRFs[3:6]) * current_contact[1]
    tau_RL = -np.matmul(jac_feet.RL[:, 12:15].T, nmpc_GRFs[6:9]) * current_contact[2]
    tau_RR = -np.matmul(jac_feet.RR[:, 15:18].T, nmpc_GRFs[9:12]) * current_contact[3]
    # ---------------------------------------------------------------------------------------------------

    # Compute Swing Torque ------------------------------------------------------------------------------
    # Compute foot position and velocities
    # TODO: The feet velocity can be computed from qvel and the jacobian, why finite difference?
    # feet_vel = (feet_pos - feet_pos_prev) / simulation_dt  # Finite difference approximation
    feet_vel = LegsAttr(**{leg_name: jac_feet[leg_name] @ env.mjData.qvel for leg_name in legs_order})
    # Update previous foot positions
    feet_pos_prev = copy.deepcopy(feet_pos)

    # Compute the reference for the swing trajectory
    if (use_print_debug):
        print("swing_period: ", swing_period)
        print("swing_time: ", swing_time)

    for leg_id, leg_name in enumerate(legs_order):
        # Swing time reset
        if (current_contact[leg_id] == 0):
            if swing_time[leg_id] < swing_period:
                swing_time[leg_id] = swing_time[leg_id] + simulation_dt
        else:
            swing_time[leg_id] = 0

        # Set lif-offs
        if previous_contact[leg_id] == 1 and current_contact[leg_id] == 0:
            lift_off_positions[leg_name] = copy.deepcopy(feet_pos[leg_name])

    # The swing controller is in the end-effector space. For its computation,
    # we save for simplicity joints position and velocities
    # TODO: Indices should not be hardcoded. Env should be provided with the joint names per legs, such that ids are
    #  automatically queried per robot URDF. We should assume quadruped structure, nothing else (leg names, leg order)
    joints_pos_FL = env.mjData.qpos[6:9]
    joints_pos_FR = env.mjData.qpos[9:12]
    joints_pos_RL = env.mjData.qpos[12:15]
    joints_pos_RR = env.mjData.qpos[15:18]

    joints_vel_FL = env.mjData.qvel[6:9]
    joints_vel_FR = env.mjData.qvel[9:12]
    joints_vel_RL = env.mjData.qvel[12:15]
    joints_vel_RR = env.mjData.qvel[15:18]

    # centrifugal, coriolis, gravity
    h_FL = env.mjData.qfrc_bias[6:9]
    h_FR = env.mjData.qfrc_bias[9:12]
    h_RL = env.mjData.qfrc_bias[12:15]
    h_RR = env.mjData.qfrc_bias[15:18]

    # and inertia matrix
    mass_matrix = np.zeros((env.mjModel.nv, env.mjModel.nv))
    mujoco.mj_fullM(env.mjModel, mass_matrix, env.mjData.qM)
    mass_matrix_FL = mass_matrix[6:9, 6:9]
    mass_matrix_FR = mass_matrix[9:12, 9:12]
    mass_matrix_RL = mass_matrix[12:15, 12:15]
    mass_matrix_RR = mass_matrix[15:18, 15:18]

    # If the foot is not in stance, we can calculate the swing controller
    if (current_contact[0] == 0):
        tau_FL, \
            desired_swing_foot_position_FL, \
            desired_swing_foot_velocity_FL = stc.compute_swing_control(env.mjModel,
                                                                       joints_pos_FL,
                                                                       joints_vel_FL,
                                                                       jac_feet.FL[0:3, 6:9],
                                                                       jac_feet_dot.FL[0:3, 6:9],
                                                                       lift_off_positions.FL,
                                                                       nmpc_footholds[0],
                                                                       swing_time[0],
                                                                       feet_pos.FL,
                                                                       feet_vel.FL,
                                                                       h_FL,
                                                                       mass_matrix_FL)

    if (current_contact[1] == 0):
        tau_FR, \
            desired_swing_foot_position_FR, \
            desired_swing_foot_velocity_FR = stc.compute_swing_control(env.mjModel,
                                                                       joints_pos_FR,
                                                                       joints_vel_FR,
                                                                       jac_feet.FR[0:3, 9:12],
                                                                       jac_feet_dot.FR[0:3, 9:12],
                                                                       lift_off_positions.FR,
                                                                       nmpc_footholds[1],
                                                                       swing_time[1],
                                                                       feet_pos.FR,
                                                                       feet_vel.FR,
                                                                       h_FR,
                                                                       mass_matrix_FR)

    if (current_contact[2] == 0):
        tau_RL, \
            desired_swing_foot_position_RL, \
            desired_swing_foot_velocity_RL = stc.compute_swing_control(env.mjModel,
                                                                       joints_pos_RL,
                                                                       joints_vel_RL,
                                                                       jac_feet.RL[0:3, 12:15],
                                                                       jac_feet_dot.RL[0:3, 12:15],
                                                                       lift_off_positions.RL,
                                                                       nmpc_footholds[2],
                                                                       swing_time[2],
                                                                       feet_pos.RL,
                                                                       feet_vel.RL,
                                                                       h_RL,
                                                                       mass_matrix_RL)

    if (current_contact[3] == 0):
        tau_RR, \
            desired_swing_foot_position_RR, \
            desired_swing_foot_velocity_RR = stc.compute_swing_control(env.mjModel,
                                                                       joints_pos_RR,
                                                                       joints_vel_RR,
                                                                       jac_feet.RR[0:3, 15:18],
                                                                       jac_feet_dot.RR[0:3, 15:18],
                                                                       lift_off_positions.RR,
                                                                       nmpc_footholds[3],
                                                                       swing_time[3],
                                                                       feet_pos.RR,
                                                                       feet_vel.RR,
                                                                       h_RR,
                                                                       mass_matrix_RR)

    # ---------------------------------------------------------------------------------------------------

    # Set control and mujoco step ----------------------------------------------------------------------
    tau = np.concatenate((tau_FR, tau_FL, tau_RR, tau_RL))
    if (use_print_debug):
        print("tau: \n", tau)

    env.step(action=tau.reshape(env.mjModel.nu))
    env.render()

    # Update the periodic gait generator
    pgg.run(simulation_dt, pgg.step_freq)

    print("loop time: ", time.time() - step_start)
    # ---------------------------------------------------------------------------------------------------
