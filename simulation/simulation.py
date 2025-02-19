# Description: This script is used to simulate the full model of the robot in mujoco

# Authors:
# Giulio Turrisi, Daniel Ordonez

import time
import numpy as np
from tqdm import tqdm

# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.quadruped_utils import LegsAttr

# Config imports
from quadruped_pympc import config as cfg

# Helper functions for plotting
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from gym_quadruped.utils.mujoco.visual import render_vector
from gym_quadruped.utils.mujoco.visual import render_sphere

# PyMPC controller imports
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

# HeightMap import
if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
    from gym_quadruped.sensors.heightmap import HeightMap




def run_simulation(process=0, num_episodes=500, return_dict=None, seed_number=0, base_vel_command_type="human" ,render=True):
    
    np.set_printoptions(precision=3, suppress=True)
    np.random.seed(seed_number) 

    robot_name = cfg.robot
    hip_height = cfg.hip_height
    robot_leg_joints = cfg.robot_leg_joints
    robot_feet_geom_names = cfg.robot_feet_geom_names
    scene_name = cfg.simulation_params['scene']
    simulation_dt = cfg.simulation_params['dt']

    state_observables_names = ('base_pos', 'base_lin_vel', 'base_ori_euler_xyz', 'base_ori_quat_wxyz', 'base_ang_vel',
                               'qpos_js', 'qvel_js', 'tau_ctrl_setpoint',
                               'feet_pos_base', 'feet_vel_base', 'contact_state', 'contact_forces_base',)


    # Create the quadruped robot environment -----------------------------------------------------------
    env = QuadrupedEnv(robot=robot_name,
                       hip_height=hip_height,
                       legs_joint_names=robot_leg_joints,  # Joint names of the legs DoF
                       feet_geom_name=robot_feet_geom_names,  # Geom/Frame id of feet
                       scene=scene_name,
                       sim_dt=simulation_dt,
                       ref_base_lin_vel=(0, 0.6),  # pass a float for a fixed value
                       ref_base_ang_vel=(-0.2, 0.2),  # pass a float for a fixed value
                       ground_friction_coeff=1.5,  # pass a float for a fixed value
                       base_vel_command_type=base_vel_command_type,  # "forward", "random", "forward+rotate", "human"
                       state_obs_names=state_observables_names,  # Desired quantities in the 'state' vec
                       )



    # Some robots require a change in the zero joint-space configuration. If provided apply it
    if cfg.qpos0_js is not None:
        env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], cfg.qpos0_js))

    env.reset(random=False)
    if render:
        env.render()  # Pass in the first render call any mujoco.viewer.KeyCallbackType


    # Initialization of variables used in the main control loop --------------------------------

    # Jacobian matrices
    jac_feet_prev = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
    jac_feet_dot = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
    # Torque vector
    tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])
    # Torque limits
    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL]*tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR]*tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL]*tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR]*tau_soft_limits_scalar)

    
    
    # Feet positions and Legs order
    feet_pos = None
    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
    legs_order = ["FL", "FR", "RL", "RR"]


    # Create HeightMap -----------------------------------------------------------------------
    if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
        resolution_vfa = 0.04
        dimension_vfa = 7
        heightmaps = LegsAttr(FL=HeightMap(n=dimension_vfa, dist_x=resolution_vfa, dist_y=resolution_vfa, mj_model=env.mjModel, mj_data=env.mjData),
                        FR=HeightMap(n=dimension_vfa, dist_x=resolution_vfa, dist_y=resolution_vfa, mj_model=env.mjModel, mj_data=env.mjData),
                        RL=HeightMap(n=dimension_vfa, dist_x=resolution_vfa, dist_y=resolution_vfa, mj_model=env.mjModel, mj_data=env.mjData),
                        RR=HeightMap(n=dimension_vfa, dist_x=resolution_vfa, dist_y=resolution_vfa, mj_model=env.mjModel, mj_data=env.mjData))
    else:
        heightmaps = None


    # Quadruped PyMPC controller initialization -------------------------------------------------------------
    mpc_frequency = cfg.simulation_params['mpc_frequency']


    quadrupedpympc_observables_names = ('ref_base_height', 'ref_base_angles', 'ref_feet_pos',
                                        'nmpc_GRFs', 'nmpc_footholds', 
                                        'swing_time', 'phase_signal', 'lift_off_positions')
    
    quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(initial_feet_pos=env.feet_pos, legs_order=legs_order,
                                                    quadrupedpympc_observables_names = quadrupedpympc_observables_names)


    # --------------------------------------------------------------
    RENDER_FREQ = 30  # Hz
    N_EPISODES = num_episodes
    N_STEPS_PER_EPISODE = 2000 if env.base_vel_command_type != "human" else 20000
    last_render_time = time.time()

    state_obs_history, ctrl_state_history = [], []
    for episode_num in tqdm(range(N_EPISODES), desc="Episodes"):

        ep_state_obs_history, ep_ctrl_state_history = [], []
        for _ in range(N_STEPS_PER_EPISODE):
            step_start = time.time()


            # Update value from SE or Simulator ----------------------
            feet_pos = env.feet_pos(frame='world')
            hip_pos = env.hip_positions(frame='world')
            base_lin_vel = env.base_lin_vel(frame='world')
            base_ang_vel = env.base_ang_vel(frame='world')
            base_ori_euler_xyz = env.base_ori_euler_xyz
            base_pos = env.base_pos
            
            # Get the reference base velocity in the world frame
            ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()
            
            # Get the inertia matrix
            if(cfg.simulation_params['use_inertia_recomputation']):
                inertia = env.get_base_inertia().flatten()  # Reflected inertia of base at qpos, in world frame
            else:
                inertia = cfg.inertia.flatten()

            # Get the qpos and qvel
            qpos, qvel = env.mjData.qpos, env.mjData.qvel
            joints_pos = LegsAttr(FL=qpos[7:10], FR=qpos[10:13],
                                  RL=qpos[13:16], RR=qpos[16:19])
            
            # Get Centrifugal, Coriolis, Gravity for the swing controller
            legs_mass_matrix = env.legs_mass_matrix
            legs_qfrc_bias = env.legs_qfrc_bias

            # Compute feet jacobian
            feet_jac = env.feet_jacobians(frame='world', return_rot_jac=False)
            
            # Compute jacobian derivatives of the contact points
            jac_feet_dot = (feet_jac - jac_feet_prev) / simulation_dt  # Finite difference approximation
            jac_feet_prev = feet_jac  # Update previous Jacobians
            
            # Compute feet velocities
            feet_vel = LegsAttr(**{leg_name: feet_jac[leg_name] @ env.mjData.qvel for leg_name in legs_order})

            # Idx of the leg
            legs_qvel_idx = env.legs_qvel_idx
            legs_qpos_idx = env.legs_qpos_idx



            # Quadruped PyMPC controller --------------------------------------------------------------
            tau = quadrupedpympc_wrapper.compute_actions(base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
                                                    feet_pos, hip_pos, joints_pos,
                                                    heightmaps,
                                                    legs_order, simulation_dt, ref_base_lin_vel, ref_base_ang_vel,
                                                    env.step_num, qpos, qvel, feet_jac, jac_feet_dot, feet_vel, legs_qfrc_bias,
                                                    legs_mass_matrix, legs_qpos_idx, legs_qvel_idx, tau, inertia)
            # Limit tau between tau_limits
            for leg in ["FL", "FR", "RL", "RR"]:
                tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
                tau[leg] = np.clip(tau[leg], tau_min, tau_max)

            quadrupedpympc_observables = quadrupedpympc_wrapper.get_obs()

            

            
            # Set control and mujoco step ----------------------------------------------------------------------
            action = np.zeros(env.mjModel.nu)
            action[env.legs_tau_idx.FL] = tau.FL
            action[env.legs_tau_idx.FR] = tau.FR
            action[env.legs_tau_idx.RL] = tau.RL
            action[env.legs_tau_idx.RR] = tau.RR

            action_noise = np.random.normal(0, 2, size=env.mjModel.nu)
            action += action_noise

            state, reward, is_terminated, is_truncated, info = env.step(action=action)


            # Store the history of observations and control -------------------------------------------------------
            ep_state_obs_history.append(state)
            base_lin_vel_err = ref_base_lin_vel - base_lin_vel
            base_ang_vel_err = ref_base_ang_vel - base_ang_vel
            base_poz_z_err = quadrupedpympc_observables["ref_base_height"] - base_pos[2]
            ctrl_state = np.concatenate((base_lin_vel_err, base_ang_vel_err, [base_poz_z_err], quadrupedpympc_observables["phase_signal"]))
            ep_ctrl_state_history.append(ctrl_state)


            # Render only at a certain frequency -----------------------------------------------------------------
            if render and (time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1):
                _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)

                # Plot the swing trajectory
                feet_traj_geom_ids = plot_swing_mujoco(viewer=env.viewer,
                                                       swing_traj_controller=quadrupedpympc_wrapper.wb_interface.stc,
                                                       swing_period=quadrupedpympc_wrapper.wb_interface.stc.swing_period,
                                                       swing_time=LegsAttr(FL=quadrupedpympc_observables["swing_time"][0],
                                                                           FR=quadrupedpympc_observables["swing_time"][1],
                                                                           RL=quadrupedpympc_observables["swing_time"][2],
                                                                           RR=quadrupedpympc_observables["swing_time"][3]),
                                                       lift_off_positions=quadrupedpympc_observables["lift_off_positions"],
                                                       nmpc_footholds=quadrupedpympc_observables["nmpc_footholds"],
                                                       ref_feet_pos=quadrupedpympc_observables["ref_feet_pos"],
                                                       geom_ids=feet_traj_geom_ids)
                
                
                # Update and Plot the heightmap
                if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
                    #if(stc.check_apex_condition(current_contact, interval=0.01)):
                    for leg_id, leg_name in enumerate(legs_order):
                        data = heightmaps[leg_name].data#.update_height_map(ref_feet_pos[leg_name], yaw=env.base_ori_euler_xyz[2])
                        if(data is not None):
                            for i in range(data.shape[0]):
                                for j in range(data.shape[1]):
                                        heightmaps[leg_name].geom_ids[i, j] = render_sphere(viewer=env.viewer,
                                                                                            position=([data[i][j][0][0],data[i][j][0][1],data[i][j][0][2]]),
                                                                                            diameter=0.01,
                                                                                            color=[0, 1, 0, .5],
                                                                                            geom_id=heightmaps[leg_name].geom_ids[i, j]
                                                                                            )
                            
                # Plot the GRF
                for leg_id, leg_name in enumerate(legs_order):
                    feet_GRF_geom_ids[leg_name] = render_vector(env.viewer,
                                                                vector=feet_GRF[leg_name],
                                                                pos=feet_pos[leg_name],
                                                                scale=np.linalg.norm(feet_GRF[leg_name]) * 0.005,
                                                                color=np.array([0, 1, 0, .5]),
                                                                geom_id=feet_GRF_geom_ids[leg_name])

                env.render()
                last_render_time = time.time()


            # Reset the environment if the episode is terminated ------------------------------------------------
            if env.step_num >= N_STEPS_PER_EPISODE or is_terminated or is_truncated:
                if is_terminated:
                    print("Environment terminated")
                else:
                    state_obs_history.append(ep_state_obs_history)
                    ctrl_state_history.append(ep_ctrl_state_history)
                
                env.reset(random=True)     
                quadrupedpympc_wrapper.reset(initial_feet_pos = env.feet_pos(frame='world')) 

                if(return_dict is not None):
                    return_dict['process'+str(process)+'_ctrl_state_history_ep'+str(episode_num)] = np.array(ctrl_state_history).reshape(-1, len(ctrl_state))
                    if(is_terminated or is_truncated):
                        return_dict['process'+str(process)+'_success_rate_ep'+str(episode_num)] = 0
                    else:
                        return_dict['process'+str(process)+'_success_rate_ep'+str(episode_num)] = 1
                
                break


    env.close()
    return return_dict




if __name__ == '__main__':
    run_simulation()
    #run_simulation(num_episodes=1, render=False)