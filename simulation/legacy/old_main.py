# Description: This script is used to simulate the full model of the robot in mujoco


import mujoco.viewer
import mujoco
import copy
import time
import pprint
import numpy as np
np.set_printoptions(precision=3, suppress = True)

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../../')
sys.path.append(dir_path + '/../../helpers/')
sys.path.append(dir_path + '/../../helpers/swing_generators/')
sys.path.append(dir_path + '/../../gradient/')
sys.path.append(dir_path + '/../../gradient/input_rates/')
sys.path.append(dir_path + '/../../gradient/nominal/')
sys.path.append(dir_path + '/../../gradient/collaborative/')
sys.path.append(dir_path + '/../../sampling/')

# General magic
#from foothold_reference_generator import FootholdReferenceGenerator
#from swing_trajectory_controller import SwingTrajectoryController
#from periodic_gait_generator import PeriodicGaitGenerator, Gait
#from srb_inertia_computation import SrbInertiaComputation
from terrain_estimator import TerrainEstimator
from other import euler_from_quaternion#, plot_swing_mujoco, plot_state_matplotlib


# Parameters for both MPC and simulation
from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.srb_inertia_computation import SrbInertiaComputation
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.quadruped_utils import GaitType, LegsAttr

# Parameters for both MPC and simulation
from quadruped_pympc import config


def get_gait_params(gait_type: str) -> [GaitType, float, float]:
    if gait_type == "trot":
        step_frequency = 1.4
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
        step_frequency = 2
        duty_factor = 0.65
        gait_type = GaitType.BOUNDING
    else:
        step_frequency = 2
        duty_factor = 0.65
        gait_type = GaitType.FULL_STANCE
        # print("FULL STANCE")
    return gait_type, duty_factor, step_frequency


# Mujoco model and data
scene = config.simulation_params['scene']
if(config.robot == 'go2'):
    m = mujoco.MjModel.from_xml_path(dir_path + '/../robot_model/go2/scene_' + scene  + '.xml')
elif(config.robot == 'aliengo'):
    m = mujoco.MjModel.from_xml_path(dir_path + '/../robot_model/aliengo/scene_' + scene  + '.xml')
elif(config.robot == 'hyqreal'):
    m = mujoco.MjModel.from_xml_path(dir_path + '/../robot_model/hyqreal/scene_' + scene  + '.xml')
elif(config.robot == 'mini_cheetah'):
    m = mujoco.MjModel.from_xml_path(dir_path + '/../robot_model/mini_cheetah/scene_' + scene  + '.xml')
else:
    print("robot not found")
    exit()
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
mass = np.sum(m.body_mass)



# MPC Magic - i took the minimum value of the dt 
# used along the horizon of the MPC
mpc_frequency = config.simulation_params['mpc_frequency']
mpc_dt = config.mpc_params['dt']
horizon = config.mpc_params['horizon']

# input_rates optimize the delta_GRF (smoooth!)
# nominal optimize directly the GRF (not smooth)
# sampling use GPUUUU
# collaborative optimize directly the GRF and has a passive arm model inside
if(config.mpc_params['type'] == 'nominal'):
    from gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal
    controller = Acados_NMPC_Nominal()

    if(config.mpc_params['optimize_step_freq']):
        from gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
        batched_controller = Acados_NMPC_GaitAdaptive()
    
elif(config.mpc_params['type'] == 'input_rates'):
    from quadruped_pympc.controllers.gradient.input_rates.centroidal_nmpc_input_rates import Acados_NMPC_InputRates
    controller = Acados_NMPC_InputRates()

    if(config.mpc_params['optimize_step_freq']):
        from gradient.nominal.centroidal_nmpc_gait_adaptive import Acados_NMPC_GaitAdaptive
        batched_controller = Acados_NMPC_GaitAdaptive()

elif(config.mpc_params['type'] == 'sampling'):
    if config.mpc_params['optimize_step_freq']:
        from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax_gait_adaptive import Sampling_MPC
    else:
        from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC
    
    import jax
    import jax.numpy as jnp
    num_parallel_computations = config.mpc_params['num_parallel_computations']
    iteration = config.mpc_params['num_sampling_iterations']
    controller = Sampling_MPC(horizon = horizon,
                              dt = mpc_dt,
                              num_parallel_computations = num_parallel_computations,
                              sampling_method = config.mpc_params['sampling_method'],
                              control_parametrization = config.mpc_params['control_parametrization'],
                              device="gpu")
    best_control_parameters = jnp.zeros((controller.num_control_parameters, ))
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
FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')
FL_geom = m.geom("FL")
FR_geom = m.geom("FR")
RL_geom = m.geom("RL")
RR_geom = m.geom("RR")
FL_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_calf')
FR_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_calf')
RL_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_calf')
RR_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_calf')

# Jacobian matrices
jac_com = np.zeros((3, m.nv))
jac_foot_FL = np.zeros((3, m.nv))
jac_foot_FR = np.zeros((3, m.nv))
jac_foot_RL = np.zeros((3, m.nv))
jac_foot_RR = np.zeros((3, m.nv))

# Previous jacobian matrices
jac_foot_FL_prev = np.zeros((3, m.nv))
jac_foot_FR_prev = np.zeros((3, m.nv))
jac_foot_RL_prev = np.zeros((3, m.nv))
jac_foot_RR_prev = np.zeros((3, m.nv))

# Derivative of the jacobian matrices
jac_foot_FL_dot = np.zeros((3, m.nv))
jac_foot_FR_dot = np.zeros((3, m.nv))
jac_foot_RL_dot = np.zeros((3, m.nv))
jac_foot_RR_dot = np.zeros((3, m.nv))

# Previous foot positions
position_foot_FL_prev = np.zeros((3, ))
position_foot_FR_prev = np.zeros((3, ))
position_foot_RL_prev = np.zeros((3, ))
position_foot_RR_prev = np.zeros((3, ))

# Torque vectors
tau_FL = np.zeros((m.nv, 1))
tau_FR = np.zeros((m.nv, 1))
tau_RL = np.zeros((m.nv, 1))
tau_RR = np.zeros((m.nv, 1))


# State
state_current = np.zeros(shape=(controller.states_dim,))
state_old = np.zeros(shape=(controller.states_dim,))


# Simulation dt
m.opt.timestep = config.simulation_params['dt']
simulation_dt = config.simulation_params['dt']



# Periodic gait generator
gait_name = config.simulation_params['gait']
gait_type, duty_factor, step_frequency = get_gait_params(gait_name)
# Given the possibility to use nonuniform discretization, 
# we generate a contact sequence two times longer
pgg = PeriodicGaitGenerator(duty_factor=duty_factor, step_freq=step_frequency, gait_type=gait_type, 
                            horizon=horizon*2, contact_sequence_dt=mpc_dt/2.)
contact_sequence = pgg.compute_contact_sequence()
nominal_sample_freq = step_frequency




# Create the foothold reference generator
stance_time = (1/step_frequency) * duty_factor
frg = FootholdReferenceGenerator(stance_time=stance_time, hip_height=config.hip_height)



# Hip position
FL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
FR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
RL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
RR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')


hip_pos_legattr = LegsAttr(
            FR=d.body(FR_hip_id).xpos,
            FL=d.body(FL_hip_id).xpos,
            RR=d.body(RR_hip_id).xpos,
            RL=d.body(RL_hip_id).xpos,
            )

lift_off_positions_legattr = LegsAttr(
    FR=d.geom_xpos[FR_id],
    FL=d.geom_xpos[FL_id],
    RR=d.geom_xpos[RR_id],
    RL=d.geom_xpos[RL_id],
    )


# Set the reference for the state
ref_x_dot = 0.3
ref_y_dot = 0.0
ref_yaw_dot = 0.0
ref_pose = np.array([0, 0, config.simulation_params['ref_z']])
ref_linear_velocity = np.array([ref_x_dot, ref_y_dot, 0])
ref_orientation = np.array([0.0, 0.0, 0])
ref_angular_velocity = np.array([0, 0, ref_yaw_dot])

ref_feet_pos = frg.compute_footholds_reference(ref_pose,
                                               ref_orientation,
                                                    ref_linear_velocity[0:2], 
                                                    ref_linear_velocity[0:2],
                                               hip_pos_legattr, config.simulation_params['ref_z'],
                                               lift_off_positions_legattr)

reference_foot_FL = ref_feet_pos.FL
reference_foot_FR = ref_feet_pos.FR
reference_foot_RL = ref_feet_pos.RL
reference_foot_RR = ref_feet_pos.RR


# SET REFERENCE AS DICTIONARY
reference_state = {'ref_position': ref_pose, 
                    'ref_linear_velocity': ref_linear_velocity, 
                    'ref_orientation': ref_orientation,
                    'ref_angular_velocity': ref_angular_velocity,
                    'ref_foot_FL': reference_foot_FL,
                    'ref_foot_FR': reference_foot_FR,
                    'ref_foot_RL': reference_foot_RL,
                    'ref_foot_RR': reference_foot_RR}



# Create swing trajectory generator
step_height = config.simulation_params['step_height'] 
swing_period = (1 - duty_factor) * (1 / step_frequency)# + 0.07
position_gain_fb = config.simulation_params['swing_position_gain_fb']
velocity_gain_fb = config.simulation_params['swing_velocity_gain_fb']
swing_generator = config.simulation_params['swing_generator']
stc = SwingTrajectoryController(step_height=step_height, swing_period=swing_period,
                                position_gain_fb = position_gain_fb, velocity_gain_fb = velocity_gain_fb,
                                generator = swing_generator)



# Swing controller variables
swing_time = [0, 0, 0, 0]
lift_off_positions = [np.zeros((3, 1)) for _ in range(4)]

position_foot_FL = d.geom_xpos[FL_id]
lift_off_positions[0] = copy.deepcopy(position_foot_FL)

position_foot_FR = d.geom_xpos[FR_id]
lift_off_positions[1] = copy.deepcopy(position_foot_FR)

position_foot_RL = d.geom_xpos[RL_id]
lift_off_positions[2] = copy.deepcopy(position_foot_RL)

position_foot_RR = d.geom_xpos[RR_id]
lift_off_positions[3] = copy.deepcopy(position_foot_RR)


# Online computation of the inertia parameter
srb_inertia_computation = SrbInertiaComputation()
inertia = config.inertia

# Terrain estimator
terrain_computation = TerrainEstimator()



# Starting contact sequence
previous_contact = np.array([1, 1, 1, 1])
previous_contact_mpc = np.array([1, 1, 1, 1])
current_contact = np.array([1, 1, 1, 1])

nmpc_GRFs = np.zeros((12,))
nmpc_wrenches = np.zeros((6,))
nmpc_footholds = np.zeros((12,))
nmpc_predicted_state = None



use_print_debug = config.simulation_params['use_print_debug']
use_visualization_debug = config.simulation_params['use_visualization_debug']




def key_callback(keycode):
    if(keycode == 262):   # arrow right
        ref_angular_velocity[2] -= 0.1
    elif(keycode == 263): # arrow left
        ref_angular_velocity[2] += 0.1
    elif(keycode == 265): # arrow up
        ref_linear_velocity[0] += 0.1
    elif(keycode == 264): # arrow down
        ref_linear_velocity[0] -= 0.1
    elif(keycode == 345): # ctrl
        ref_linear_velocity[0] = 0
        ref_linear_velocity[1] = 0
        ref_angular_velocity[2] = 0 



# Main simulation loop ------------------------------------------------------------------
with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False, key_callback=key_callback) as viewer:
    mujoco.mjv_defaultFreeCamera(m, viewer.cam)
    viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    i = 0
    while True:
        step_start = time.time()
        if(use_print_debug): 
            print("###############")


        # Update the robot state --------------------------------
        com_pos = d.qpos[0:3]
        rpy_angles = np.array(euler_from_quaternion(d.qpos[3:7]))
        linear_vel = d.qvel[0:3]
        angular_vel = copy.deepcopy(d.qvel[3:6])


        state_current = {
            'position': com_pos,
            'linear_velocity': linear_vel,
            'orientation': rpy_angles,
            'angular_velocity': angular_vel,
            'foot_FL': d.geom_xpos[FL_id],
            'foot_FR': d.geom_xpos[FR_id],
            'foot_RL': d.geom_xpos[RL_id],
            'foot_RR': d.geom_xpos[RR_id]
        }



        if(use_print_debug): 
            print("state_current: ")
            pprint.pprint(state_current)
        
        # -------------------------------------------------------

        # Update the contact sequence ---------------------------
        contact_sequence = pgg.compute_contact_sequence()

        # in the case of nonuniform discretization, we need to subsample the contact sequence
        if (config.mpc_params['use_nonuniform_discretization']):
            dt_fine_grained = config.mpc_params['dt_fine_grained']
            horizon_fine_grained = config.mpc_params['horizon_fine_grained']
            contact_sequence = pgg.sample_contact_sequence(contact_sequence, mpc_dt, dt_fine_grained, horizon_fine_grained)
               

        

        previous_contact = copy.deepcopy(current_contact)
        current_contact = np.array([contact_sequence[0][0], contact_sequence[1][0],
                                    contact_sequence[2][0], contact_sequence[3][0]])
        if(use_print_debug): 
            print("contact_sequence: \n", contact_sequence)
        # -------------------------------------------------------



        # Compute the reference for the footholds ---------------------------------------------------
        hip_pos_legattr = LegsAttr(
                    FR=d.body(FR_hip_id).xpos,
                    FL=d.body(FL_hip_id).xpos,
                    RR=d.body(RR_hip_id).xpos,
                    RL=d.body(RL_hip_id).xpos,
                    )
        
        


        ref_feet_pos = frg.compute_footholds_reference(com_pos, rpy_angles, state_current["linear_velocity"][0:2], reference_state["ref_linear_velocity"][0:2], hip_pos_legattr, 
                                                        state_current["position"][2], lift_off_positions_legattr)
        reference_foot_FL = ref_feet_pos.FL
        reference_foot_FR = ref_feet_pos.FR
        reference_foot_RL = ref_feet_pos.RL
        reference_foot_RR = ref_feet_pos.RR




        # Update state reference
        reference_state["ref_foot_FL"] = reference_foot_FL.reshape((1,3))
        reference_state["ref_foot_FR"] = reference_foot_FR.reshape((1,3))
        reference_state["ref_foot_RL"] = reference_foot_RL.reshape((1,3))
        reference_state["ref_foot_RR"] = reference_foot_RR.reshape((1,3))
        
        # and rotate the reference velocity in the world frame
        h_R_w = np.array([np.cos(rpy_angles[2]), np.sin(rpy_angles[2]), 0,
                          -np.sin(rpy_angles[2]), np.cos(rpy_angles[2]), 0,
                          0, 0, 1])
        h_R_w = h_R_w.reshape((3,3))
        reference_state["ref_linear_velocity"] = h_R_w.T@ref_linear_velocity.reshape((3,1)).flatten()
        
        if(use_print_debug): 
            print("reference_state: ")
            pprint.pprint(reference_state)
        # -------------------------------------------------------------------------------------------------


        # Compute the terrain estimation and reference height ---------------------------------------------
        terrain_roll, terrain_pitch, terrain_height = terrain_computation.compute_terrain_estimation(state_current["position"], state_current["orientation"][2], 
                                                                     lift_off_positions_legattr, current_contact)
        
        reference_state["ref_orientation"] = np.array([terrain_roll, terrain_pitch, 0])        
        reference_state["ref_position"][2] = config.simulation_params['ref_z'] + terrain_height
        # -------------------------------------------------------------------------------------------------


        # Solve OCP ---------------------------------------------------------------------------------------
        if(i % round(1/(mpc_frequency*simulation_dt)) == 0): 
            
            # We can recompute the inertia of the single rigid body model
            # or use the fixed one in config.py
            if(config.simulation_params['use_inertia_recomputation']):
                #inertia = srb_inertia_computation.compute_inertia(d.qpos)
                # Initialize the full mass matrix
                mass_matrix = np.zeros((m.nv, m.nv))
                mujoco.mj_fullM(m, mass_matrix, d.qM)
                inertia = mass_matrix[3:6, 3:6]
                

            optimize_swing = 0 #1 for always, 0 for apex
            if((config.mpc_params['optimize_step_freq'])):
                # we can always optimize the step freq, or just at the apex of the swing
                # to avoid possible jittering in the solution
                for leg in range(4):
                # Swing time check 
                    if(current_contact[leg] == 0):
                        if ((swing_time[leg] > (swing_period/2.) - 0.02) and \
                            (swing_time[leg] < (swing_period/2.) + 0.02)):
                            optimize_swing = 1
                            nominal_sample_freq = step_frequency
            
            # If we use sampling
            if(config.mpc_params['type'] == 'sampling'):

                time_start = time.time()

                # Shift the previous solution ahead
                if (config.mpc_params['shift_solution']):
                    index_shift = 1./mpc_frequency
                    best_control_parameters = controller.shift_solution(best_control_parameters, index_shift)
                
                # Convert data to jax
                state_current_jax, \
                reference_state_jax, \
                best_control_parameters = jitted_prepare_state_and_reference(state_current, reference_state, best_control_parameters, current_contact, previous_contact_mpc)


                            
                for iter_sampling in range(iteration):
                    if(config.mpc_params['sampling_method'] == 'cem_mppi'):
                        if(iter_sampling == 0):
                            controller = controller.with_newsigma(config.mpc_params['sigma_cem_mppi'])
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
                

                if((config.mpc_params['optimize_step_freq']) and (optimize_swing == 1)):
                    pgg.step_freq = np.array([best_sample_freq])[0]
                    nominal_sample_freq = pgg.step_freq
                    stance_time = (1/pgg.step_freq) * duty_factor
                    frg.stance_time = stance_time
                    
                    swing_period = (1 - duty_factor) * (1 / pgg.step_freq)# + 0.07
                    stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)


                nmpc_footholds = np.zeros((4,3))
                nmpc_footholds[0] = reference_state["ref_foot_FL"]
                nmpc_footholds[1] = reference_state["ref_foot_FR"]
                nmpc_footholds[2] = reference_state["ref_foot_RL"]
                nmpc_footholds[3] = reference_state["ref_foot_RR"]
        
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
                status = controller.compute_control(state_current, reference_state, contact_sequence, inertia=inertia.flatten())

                optimizer_cost = controller.acados_ocp_solver.get_cost()
                
                
                if((config.mpc_params['optimize_step_freq']) and (optimize_swing == 1)):
                    contact_sequence_temp = np.zeros((len(config.mpc_params['step_freq_available']), 4, horizon * 2))
                    
                    for j in range(len(config.mpc_params['step_freq_available'])):
                        pgg_temp = PeriodicGaitGenerator(duty_factor=duty_factor, step_freq=
                        config.mpc_params['step_freq_available'][j], p_gait=gait_type, horizon=horizon * 2)
                        pgg_temp.t = copy.deepcopy(pgg.t)
                        pgg_temp._init = copy.deepcopy(pgg._init)
                        contact_sequence_temp[j] = pgg_temp.compute_contact_sequence()

                        # in the case of nonuniform discretization, we need to subsample the contact sequence
                        if (config.mpc_params['use_nonuniform_discretization']):
                            dt_fine_grained = config.mpc_params['dt_fine_grained']
                            horizon_fine_grained = config.mpc_params['horizon_fine_grained']
                            contact_sequence_temp[j] = pgg.sample_contact_sequence(contact_sequence, mpc_dt, dt_fine_grained, horizon_fine_grained)

                    costs, best_sample_freq = batched_controller.compute_batch_control(state_current, reference_state, contact_sequence_temp)
                    
                    pgg.step_freq = best_sample_freq
                    stance_time = (1/pgg.step_freq) * duty_factor
                    frg.stance_time = stance_time
                    swing_period = (1 - duty_factor) * (1 / pgg.step_freq)# + 0.07
                    stc.regenerate_swing_trajectory_generator(step_height=step_height, swing_period=swing_period)


                # If the controller is using RTI, we need to linearize the mpc after its computation
                # this helps to minize the delay between new state->control, but only in a real case. 
                # Here we are in simulation and does not make any difference for now
                if(controller.use_RTI):
                    # preparation phase
                    controller.acados_ocp_solver.options_set('rti_phase', 1)
                    status = controller.acados_ocp_solver.solve()
                    print("preparation phase time: ", controller.acados_ocp_solver.get_stats('time_tot'))
            
            
            
            
            if(use_print_debug):
                mean_tracking[0] = mean_tracking[0] + np.abs(state_current["position"][2] - reference_state["ref_position"][2])
                mean_tracking[1] = mean_tracking[1] + np.abs(state_current["linear_velocity"][0] - reference_state["ref_linear_velocity"][0])
                mean_tracking[2] = mean_tracking[2] + np.abs(state_current["linear_velocity"][1] - reference_state["ref_linear_velocity"][1])
                mean_tracking[3] = mean_tracking[3] + np.abs(state_current["linear_velocity"][2] - reference_state["ref_linear_velocity"][2])
                mean_tracking[4] = mean_tracking[4] + np.abs(state_current["orientation"][0] - reference_state["ref_orientation"][0])
                mean_tracking[5] = mean_tracking[5] + np.abs(state_current["orientation"][1] - reference_state["ref_orientation"][1])
                mean_tracking[6] = mean_tracking[6] + np.abs(state_current["angular_velocity"][0] - reference_state["ref_angular_velocity"][0])
                mean_tracking[7] = mean_tracking[7] + np.abs(state_current["angular_velocity"][1] - reference_state["ref_angular_velocity"][1])
                mean_tracking[8] = mean_tracking[8] + np.abs(state_current["angular_velocity"][2] - reference_state["ref_angular_velocity"][2])
                print("mean_tracking: ", mean_tracking/(mean_num_sample))
                mean_optimizer_cost = mean_optimizer_cost + optimizer_cost
                print("mean_cost: ", mean_optimizer_cost/(mean_num_sample))
                mean_num_sample = mean_num_sample + 1



            # Put the GRFs to zero if the foot is not in contact
            nmpc_GRFs[0:3] = nmpc_GRFs[0:3]*current_contact[0]
            nmpc_GRFs[3:6] = nmpc_GRFs[3:6]*current_contact[1]
            nmpc_GRFs[6:9] = nmpc_GRFs[6:9]*current_contact[2]
            nmpc_GRFs[9:12] = nmpc_GRFs[9:12]*current_contact[3]
                    


        if(use_print_debug): 
            print("nmpc_GRFs: \n", nmpc_GRFs)
            print("nmpc_footholds: \n", nmpc_footholds)
        # -------------------------------------------------------------------------------------------------


        # Compute Stance Torque --------------------------------------------------------------------------- 
        # Compute the jacobian of the contact points
        mujoco.mj_jac(m, d, jac_foot_FL, None, d.geom_xpos[FL_id], FL_calf_id)
        mujoco.mj_jac(m, d, jac_foot_FR, None, d.geom_xpos[FR_id], FR_calf_id)
        mujoco.mj_jac(m, d, jac_foot_RL, None, d.geom_xpos[RL_id], RL_calf_id)
        mujoco.mj_jac(m, d, jac_foot_RR, None, d.geom_xpos[RR_id], RR_calf_id)


        # Compute jacobian derivatives of the contact points
        jac_foot_FL_dot = copy.deepcopy((jac_foot_FL - jac_foot_FL_prev) / simulation_dt)
        jac_foot_FR_dot = copy.deepcopy((jac_foot_FR - jac_foot_FR_prev) / simulation_dt)
        jac_foot_RL_dot = copy.deepcopy((jac_foot_RL - jac_foot_RL_prev) / simulation_dt)
        jac_foot_RR_dot = copy.deepcopy((jac_foot_RR - jac_foot_RR_prev) / simulation_dt)

        # Update previous jacobians
        jac_foot_FL_prev = copy.deepcopy(jac_foot_FL)
        jac_foot_FR_prev = copy.deepcopy(jac_foot_FR)
        jac_foot_RL_prev = copy.deepcopy(jac_foot_RL)
        jac_foot_RR_prev = copy.deepcopy(jac_foot_RR)




        # Compute the torque with the contact jacobian (-J*f)
        # index 6:9 -> FL, 9:12 -> FR, 12:15 -> RL, 15:18 -> RR
        tau_FL = -np.matmul(jac_foot_FL[0:3, 6:9].T, nmpc_GRFs[0:3])*current_contact[0]
        tau_FR = -np.matmul(jac_foot_FR[0:3, 9:12].T, nmpc_GRFs[3:6])*current_contact[1]
        tau_RL = -np.matmul(jac_foot_RL[0:3, 12:15].T, nmpc_GRFs[6:9])*current_contact[2]
        tau_RR = -np.matmul(jac_foot_RR[0:3, 15:18].T, nmpc_GRFs[9:12])*current_contact[3]
        # ---------------------------------------------------------------------------------------------------



        # Compute Swing Torque ------------------------------------------------------------------------------
        # Compute foot position and velocities
        position_foot_FL = copy.deepcopy(d.geom_xpos[FL_id])
        position_foot_FR = copy.deepcopy(d.geom_xpos[FR_id])
        position_foot_RL = copy.deepcopy(d.geom_xpos[RL_id])
        position_foot_RR = copy.deepcopy(d.geom_xpos[RR_id])
        
        velocity_foot_FL = copy.deepcopy((position_foot_FL - position_foot_FL_prev) / simulation_dt)
        velocity_foot_FR = copy.deepcopy((position_foot_FR - position_foot_FR_prev) / simulation_dt)
        velocity_foot_RL = copy.deepcopy((position_foot_RL - position_foot_RL_prev) / simulation_dt)
        velocity_foot_RR = copy.deepcopy((position_foot_RR - position_foot_RR_prev) / simulation_dt)
    
        # Update previous foot positions
        position_foot_FL_prev = copy.deepcopy(position_foot_FL)
        position_foot_FR_prev = copy.deepcopy(position_foot_FR)
        position_foot_RL_prev = copy.deepcopy(position_foot_RL)
        position_foot_RR_prev = copy.deepcopy(position_foot_RR)

            


        # Compute the reference for the swing trajectory 
        if(use_print_debug): 
            print("swing_period: ", swing_period)
            print("swing_time: ", swing_time)

        for leg in range(4):
            # Swing time reset
            if(current_contact[leg] == 0):
                if swing_time[leg] < swing_period:
                    swing_time[leg] = swing_time[leg] + simulation_dt
            else:
                swing_time[leg] = 0
            

            # Set lif-offs
            if previous_contact[leg] == 1 and current_contact[leg] == 0:
                if(leg == 0):
                    lift_off_positions[leg] = copy.deepcopy(position_foot_FL)
                    lift_off_positions_legattr["FL"] = copy.deepcopy(position_foot_FL)
                elif(leg == 1):
                    lift_off_positions[leg] = copy.deepcopy(position_foot_FR)
                    lift_off_positions_legattr["FR"] = copy.deepcopy(position_foot_FR)
                elif(leg == 2):
                    lift_off_positions[leg] = copy.deepcopy(position_foot_RL)
                    lift_off_positions_legattr["RL"] = copy.deepcopy(position_foot_RL)
                elif(leg == 3):
                    lift_off_positions[leg] = copy.deepcopy(position_foot_RR)
                    lift_off_positions_legattr["RR"] = copy.deepcopy(position_foot_RR)

        

        # The swing controller is in the end-effector space. For its computation,
        # we save for simplicity joints position and velocities
        joints_pos_FL = d.qpos[6:9]
        joints_pos_FR = d.qpos[9:12]
        joints_pos_RL = d.qpos[12:15]
        joints_pos_RR = d.qpos[15:18]

        joints_vel_FL = d.qvel[6:9]
        joints_vel_FR = d.qvel[9:12]
        joints_vel_RL = d.qvel[12:15]
        joints_vel_RR = d.qvel[15:18]

        # centrifugal, coriolis, gravity
        h_FL = d.qfrc_bias[6:9]
        h_FR = d.qfrc_bias[9:12]
        h_RL = d.qfrc_bias[12:15]
        h_RR = d.qfrc_bias[15:18]

        # and inertia matrix 
        mass_matrix = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, mass_matrix, d.qM)
        mass_matrix_FL = mass_matrix[6:9, 6:9]
        mass_matrix_FR = mass_matrix[9:12, 9:12]
        mass_matrix_RL = mass_matrix[12:15, 12:15]
        mass_matrix_RR = mass_matrix[15:18, 15:18]

        
        
        # If the foot is not in stance, we can calculate the swing controller
        if (current_contact[0] == 0):
            tau_FL, \
            desired_swing_foot_position_FL, \
            desired_swing_foot_velocity_FL = stc.compute_swing_control(m, 
                                               joints_pos_FL, 
                                               joints_vel_FL,
                                               jac_foot_FL[0:3,6:9], 
                                               jac_foot_FL_dot[0:3,6:9],
                                               lift_off_positions[0], 
                                               nmpc_footholds[0],
                                               swing_time[0],
                                               position_foot_FL, 
                                               velocity_foot_FL,
                                               h_FL,
                                               mass_matrix_FL)


        if (current_contact[1] == 0):
            tau_FR, \
            desired_swing_foot_position_FR, \
            desired_swing_foot_velocity_FR = stc.compute_swing_control(m,
                                               joints_pos_FR, 
                                               joints_vel_FR,
                                               jac_foot_FR[0:3,9:12],
                                               jac_foot_FR_dot[0:3,9:12],
                                               lift_off_positions[1],
                                               nmpc_footholds[1],
                                               swing_time[1],
                                               position_foot_FR,
                                               velocity_foot_FR,
                                               h_FR,
                                               mass_matrix_FR)


        if (current_contact[2] == 0):
            tau_RL, \
            desired_swing_foot_position_RL, \
            desired_swing_foot_velocity_RL = stc.compute_swing_control(m,
                                               joints_pos_RL, 
                                               joints_vel_RL,
                                               jac_foot_RL[0:3,12:15],
                                               jac_foot_RL_dot[0:3,12:15],
                                               lift_off_positions[2],
                                               nmpc_footholds[2],
                                               swing_time[2],
                                               position_foot_RL,
                                               velocity_foot_RL,
                                               h_RL,
                                               mass_matrix_RL)


        if (current_contact[3] == 0):
            tau_RR, \
            desired_swing_foot_position_RR, \
            desired_swing_foot_velocity_RR = stc.compute_swing_control(m,
                                               joints_pos_RR, 
                                               joints_vel_RR,
                                               jac_foot_RR[0:3,15:18],
                                               jac_foot_RR_dot[0:3,15:18],
                                               lift_off_positions[3],
                                               nmpc_footholds[3],
                                               swing_time[3],
                                               position_foot_RR,
                                               velocity_foot_RR,
                                               h_RR,
                                               mass_matrix_RR)

        # ---------------------------------------------------------------------------------------------------


        # Set control and mujoco step ----------------------------------------------------------------------
        tau = np.concatenate((tau_FR, tau_FL, tau_RR, tau_RL))
        if(use_print_debug): 
            print("tau: \n", tau)
        
        d.ctrl = tau.reshape(m.nu,)
        mujoco.mj_step(m, d)


        # Update the periodic gait generator
        pgg.run(simulation_dt, pgg.step_freq)
        
 

        # Plot
        #if(use_visualization_debug and i % 100 == 0):
        #    plot_swing_mujoco(mujoco, stc, swing_period, lift_off_positions, nmpc_footholds, 
        #                      reference_foot_FL, reference_foot_FR,
        #                      reference_foot_RL, reference_foot_RR, viewer)
            



        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        if(i%5 == 0):
            viewer.sync()
        
        if(config.simulation_params['use_kind_of_real_time']):
            time_until_next_step = simulation_dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        
         
        print("loop time: ", time.time() - step_start)
        # ---------------------------------------------------------------------------------------------------

        i = i + 1
