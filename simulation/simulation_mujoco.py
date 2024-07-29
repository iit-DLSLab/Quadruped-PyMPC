# Description: This script is used to simulate the full model of the robot in mujoco

# Authors:
# Giulio Turrisi, Daniel Ordonez

import time
import pathlib
import numpy as np
from tqdm import tqdm
# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.mujoco.visual import render_vector
from gym_quadruped.utils.quadruped_utils import LegsAttr
# Control imports
from quadruped_pympc import config as cfg

from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from quadruped_pympc.controllers.srb_controller import SingleRigidBodyController as Controller
##
import copy as cp
if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True)

    robot_name = cfg.robot
    hip_height = cfg.hip_height
    robot_leg_joints = cfg.robot_leg_joints
    robot_feet_geom_names = cfg.robot_feet_geom_names
    scene_name = cfg.simulation_params['scene']
    simulation_dt = cfg.simulation_params['dt']
    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1,FR=-1,RL=-1,RR=-1)
    legs_order = ["FL", "FR", "RL", "RR"]
    mpc_frequency = cfg.simulation_params['mpc_frequency']

    state_observables_names = ('base_pos', 'base_lin_vel', 'base_ori_euler_xyz', 'base_ori_quat_wxyz', 'base_ang_vel'
                               , 'qvel_js', 'tau_ctrl_setpoint',
                               'feet_pos_base', 'feet_vel_base', 'contact_state', 'contact_forces_base',)

    # Create the quadruped robot environment. _______________________________________________________________________
    env = QuadrupedEnv(robot=robot_name,
                       hip_height=hip_height,
                       legs_joint_names=robot_leg_joints,  # Joint names of the legs DoF
                       feet_geom_name=robot_feet_geom_names,  # Geom/Frame id of feet
                       scene=scene_name,
                       sim_dt=simulation_dt,
                       ref_base_lin_vel=0.0,  # pass a float for a fixed value
                       ground_friction_coeff=1.5,  # pass a float for a fixed value
                       base_vel_command_type="human",  # "forward", "random", "forward+rotate", "human"
                       state_obs_names=state_observables_names,  # Desired quantities in the 'state' vec
                       )
    # Some robots require a change in the zero joint-space configuration. If provided apply it
    if cfg.qpos0_js is not None:
        env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], cfg.qpos0_js))

    env.reset(random=False)
    env.render()  # Pass in the first render call any mujoco.viewer.KeyCallbackType
    # Jacobian matrices
    nv=env.mjModel.nv
    jac_feet_prev = LegsAttr(*[np.zeros((3, nv)) for _ in range(4)]) 
    jac_feet_prev_follower = LegsAttr(*[np.zeros((3, nv)) for _ in range(4)]) 
    jac_feet_dot = LegsAttr(*[np.zeros((3, nv)) for _ in range(4)]) 
    jac_feet_dot_follower = LegsAttr(*[np.zeros((3, nv)) for _ in range(4)]) 

    # Torque vector
    tau = LegsAttr(*[np.zeros((nv, 1)) for _ in range(4)]) 
    #
    lift_off_positions=env.feet_pos(frame='world')
    #
    # _______________________________________________________________________________________________________________
    # Controller initialization ______________________________________________________________________________________
    #firest create or select the controller type and the dictionary with the entries needed in this case we use the config file
    if cfg.mpc_params['type'] == 'nominal' or cfg.mpc_params['type'] == 'input_rates':
        mpc_params =cfg.mpc_params.update(cfg.mpc_nominal_params)
    if cfg.mpc_params['type'] == 'sampling':
        mpc_params =cfg.mpc_params.update(cfg.mpc_sampling_params)

    controller= Controller(hip_height=hip_height,
                           legs_order = ["FL", "FR", "RL", "RR"],
                           inertia=cfg.inertia,
                           mass=cfg.mass,
                           lift_off_positions=lift_off_positions,
                           simulation_parameters=cfg.simulation_params,
                           mpc_parameters=cfg.mpc_params,
                           jac_feet_dot=jac_feet_dot,
                           jac_feet_prev=jac_feet_prev,
                           tau=LegsAttr(*[np.zeros((nv, 1)) for _ in range(4)]),
                           )

    # _____________________________________________________________
    RENDER_FREQ = 20  # Hz
    N_EPISODES = 500
    N_STEPS_PER_EPISODE = 20000 if env.base_vel_command_type != "human" else 20000
    last_render_time = time.time()

    state_obs_history, ctrl_state_history = [], []
    for episode_num in tqdm(range(N_EPISODES), desc="Episodes"):

        ep_state_obs_history, ep_ctrl_state_history = [], []
        for _ in range(N_STEPS_PER_EPISODE):
            step_start = time.time()

            # update reference state
            ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()
            ####
            base_lin_vel=env.base_lin_vel(frame='world')
            base_ang_vel = env.base_ang_vel(frame='world')
            ###
            base_ori_euler_xyz_leader=env.base_ori_euler_xyz
            base_pos_leader=env.base_pos
            feet_pos = env.feet_pos(frame='world') 
            hip_pos = env.hip_positions(frame='world') 
            qpos, qvel = env.mjData.qpos, env.mjData.qvel
            feet_jac=env.feet_jacobians(frame='world', return_rot_jac=False) 
            feet_vel=LegsAttr(**{leg_name: feet_jac[leg_name] @ env.mjData.qvel for leg_name in legs_order}) 
            legs_qfrc_bias=env.legs_qfrc_bias 
            legs_mass_matrix=env.legs_mass_matrix 
            leg_qvel_idx= env.legs_qvel_idx 


            ###### Controller single aliengo model
            ref_state,state_current,ref_feet_pos,contact_sequence=controller.update_mpc_reference_and_state(nv=nv,
                                            feet_pos = feet_pos, #two robots edit,
                                            hip_pos = hip_pos, #two robots edit,
                                            base_pos=env.base_pos,
                                            base_ori_euler_xyz=env.base_ori_euler_xyz,
                                            base_lin_vel=base_lin_vel,
                                            base_ang_vel = base_ang_vel,
                                            ref_base_lin_vel=ref_base_lin_vel,
                                            ref_base_ang_vel=ref_base_ang_vel)
            if env.step_num % round(1 / (mpc_frequency * simulation_dt)) == 0:
                nmpc_footholds_mpc, nmpc_GRFs, current_contact = controller.solve_MPC_main_loop(
                                                                                                inertia_env=env.get_base_inertia().flatten(),
                                                                                                ref_feet_pos=ref_feet_pos,
                                                                                                spring_gain=None,
                                                                                                eef_position=None,
                                                                                                eef_jacobian=None,
                                                                                                ext_wrenches=None)    

            nmpc_footholds = LegsAttr(FL=nmpc_footholds_mpc[0], 
                                      FR=nmpc_footholds_mpc[1],
                                      RL=nmpc_footholds_mpc[2],
                                      RR=nmpc_footholds_mpc[3],)
            
            nmpc_GRFs_computed = LegsAttr(FL=nmpc_GRFs[0:3] * current_contact[0],
                                          FR=nmpc_GRFs[3:6] * current_contact[1],
                                          RL=nmpc_GRFs[6:9] * current_contact[2],
                                          RR=nmpc_GRFs[9:12]* current_contact[3])
                
            tau,lift_off_position_ctrl = controller.compute_stance_and_swing_torque(simulation=True,
                                    feet_jac=feet_jac,
                                    feet_vel =feet_vel,
                                    legs_qvel_idx=leg_qvel_idx,
                                    qpos=qpos,
                                    qvel=qvel,
                                    legs_qfrc_bias=legs_qfrc_bias,
                                    legs_mass_matrix=legs_mass_matrix,
                                    nmpc_GRFs=nmpc_GRFs_computed,
                                    nmpc_footholds=nmpc_footholds,
                                    tau=tau,
                                    current_contact=current_contact,)
            
            time_start = time.time()

            # Set control and mujoco step ----------------------------------------------------------------------

            action = np.zeros(env.mjModel.nu)
            action[env.legs_tau_idx.FL] = tau.FL
            action[env.legs_tau_idx.FR] = tau.FR
            action[env.legs_tau_idx.RL] = tau.RL
            action[env.legs_tau_idx.RR] = tau.RR

            state, reward, is_terminated, is_truncated, info = env.step(action=action)

            # Store the history of observations and control ___________________________________________________________
            ep_state_obs_history.append(state)
            base_lin_vel_err = ref_base_lin_vel - base_lin_vel
            base_ang_vel_err = ref_base_ang_vel - base_ang_vel
            base_poz_z_err = controller.ref_pos[2] - env.base_pos[2]
            ctrl_state = np.concatenate((base_lin_vel_err, base_ang_vel_err, [base_poz_z_err], controller.pgg.phase_signal))
            ep_ctrl_state_history.append(ctrl_state)
            # ___________________________________________________________________________________________________________
            # # Render only at a certain frequency


            if time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1:
                _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)
                feet_traj_geom_ids = plot_swing_mujoco(viewer=env.viewer,
                                                       swing_traj_controller=controller.stc,
                                                       swing_period=controller.stc.swing_period,
                                                       swing_time=LegsAttr(FL=controller.stc.swing_time[0],
                                                                           FR=controller.stc.swing_time[1],
                                                                           RL=controller.stc.swing_time[2],
                                                                           RR=controller.stc.swing_time[3],
                                                       ),
                                                       lift_off_positions=lift_off_positions,
                                                       nmpc_footholds=nmpc_footholds,
                                                       ref_feet_pos=ref_feet_pos,
                                                       geom_ids=feet_traj_geom_ids)
                for leg_id, leg_name in enumerate(controller.legs_order):
                    feet_GRF_geom_ids[leg_name] = render_vector(env.viewer,
                                                                vector=feet_GRF[leg_name],
                                                                pos=feet_pos[leg_name],
                                                                scale=np.linalg.norm(feet_GRF[leg_name]) * 0.005,
                                                                color=np.array([0, 1, 0, .5]),
                                                                geom_id=feet_GRF_geom_ids[leg_name])

                env.render()
                last_render_time = time.time()            
            if env.step_num > N_STEPS_PER_EPISODE or is_terminated or is_truncated:
                if is_terminated:
                    print("Environment terminated")
                else:
                    state_obs_history.append(ep_state_obs_history)
                    ctrl_state_history.append(ep_ctrl_state_history)
                env.reset(random=False)
                controller.reset()
                current_contact = np.array([0, 0, 0, 0])
                previous_contact = np.asarray(current_contact)
                z_foot_mean = 0.0

    env.close()


