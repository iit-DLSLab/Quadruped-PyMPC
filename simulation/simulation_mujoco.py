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


if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True)

    robot_name = cfg.robot
    hip_height = cfg.hip_height
    robot_leg_joints = cfg.robot_leg_joints
    robot_feet_geom_names = cfg.robot_feet_geom_names
    scene_name = cfg.simulation_params['scene']
    simulation_dt = cfg.simulation_params['dt']
    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
    gait_name="trot" # 'trot', 'pace', 'crawl', 'bound', 'full_stance'
    legs_order = ["FL", "FR", "RL", "RR"]
    

    state_observables_names = ('base_pos', 'base_lin_vel', 'base_ori_euler_xyz', 'base_ori_quat_wxyz', 'base_ang_vel',
                               'qpos_js', 'qvel_js', 'tau_ctrl_setpoint',
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

    #
    lift_off_positions=env.feet_pos(frame='world')

    # _______________________________________________________________________________________________________________
    # Controller initialization ______________________________________________________________________________________
    controller= Controller(hip_height=hip_height,
                           legs_order=legs_order,        
                           inertia=cfg.inertia,
                           lift_off_positions=lift_off_positions,
                           simulation_parameters=cfg.simulation_params,
                           mpc_parameters=cfg.mpc_params,
                           )


    # _____________________________________________________________
    RENDER_FREQ = 20  # Hz
    N_EPISODES = 500
    N_STEPS_PER_EPISODE = 2000 if env.base_vel_command_type != "human" else 20000
    last_render_time = time.time()

    state_obs_history, ctrl_state_history = [], []
    for episode_num in tqdm(range(N_EPISODES), desc="Episodes"):

        ep_state_obs_history, ep_ctrl_state_history = [], []
        for _ in range(N_STEPS_PER_EPISODE):
            step_start = time.time()

            # update reference state
            ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()
            base_lin_vel=env.base_lin_vel
            base_ang_vel = env.base_ang_vel

            controller.update_mpc_reference(nv=env.mjModel.nv,
                                            feet_pos = env.feet_pos(frame='world'),
                                            hip_pos = env.hip_positions(frame='world'),
                                            base_pos=env.base_pos,
                                            base_ori_euler_xyz=env.base_ori_euler_xyz,
                                            base_lin_vel=base_lin_vel,
                                            base_ang_vel = base_ang_vel,
                                            ref_base_lin_vel=ref_base_lin_vel, 
                                            ref_base_ang_vel=ref_base_ang_vel)

            # compute ground reaction forces out of mpc simulation only
            controller.solve_MPC_main_loop(alpha =env.step_num,inertia_env=env.get_base_inertia().flatten())

            qpos, qvel = env.mjData.qpos, env.mjData.qvel
            feet_jac=env.feet_jacobians(frame='world', return_rot_jac=False)


            time_start = time.time()
            tau = controller.get_forward_action(simulation=True,
                                                feet_jac=feet_jac,
                                                feet_vel = LegsAttr(**{leg_name: feet_jac[leg_name] @ env.mjData.qvel for leg_name in legs_order}),
                                                legs_qvel_idx=env.legs_qvel_idx,
                                                qpos=qpos,
                                                qvel=qvel,
                                                legs_qfrc_bias=env.legs_qfrc_bias,
                                                legs_mass_matrix=env.legs_mass_matrix)

            # Set control and mujoco step ----------------------------------------------------------------------
            action = np.zeros(env.mjModel.nu)
            action[env.legs_tau_idx.FL] = tau.FL
            action[env.legs_tau_idx.FR] = tau.FR
            action[env.legs_tau_idx.RL] = tau.RL
            action[env.legs_tau_idx.RR] = tau.RR

            action_noise = np.random.normal(0, 2, size=env.mjModel.nu)
            action += action_noise

            state, reward, is_terminated, is_truncated, info = env.step(action=action)

            # Store the history of observations and control ___________________________________________________________
            ep_state_obs_history.append(state)
            base_lin_vel_err = ref_base_lin_vel - base_lin_vel
            base_ang_vel_err = ref_base_ang_vel - base_ang_vel
            base_poz_z_err = controller.ref_pos[2] - env.base_pos[2]
            ctrl_state = np.concatenate((base_lin_vel_err, base_ang_vel_err, [base_poz_z_err], controller.pgg._phase_signal))
            ep_ctrl_state_history.append(ctrl_state)
            # ___________________________________________________________________________________________________________

            # Render only at a certain frequency
            if time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1:
                _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)
                feet_traj_geom_ids = plot_swing_mujoco(viewer=env.viewer,
                                                       swing_traj_controller=controller.stc,
                                                       swing_period=controller.stc.swing_period,
                                                       swing_time=LegsAttr(FL=controller.stc.swing_time[0],
                                                                           FR=controller.stc.swing_time[1],
                                                                           RL=controller.stc.swing_time[2],
                                                                           RR=controller.stc.swing_time[3]),
                                                       lift_off_positions=controller.frg.lift_off_positions,
                                                       nmpc_footholds=controller.nmpc_footholds,
                                                       ref_feet_pos=controller.ref_feet_pos,
                                                       geom_ids=feet_traj_geom_ids)
                for leg_id, leg_name in enumerate(controller.legs_order):
                    feet_GRF_geom_ids[leg_name] = render_vector(env.viewer,
                                                                vector=feet_GRF[leg_name],
                                                                pos=controller.feet_pos[leg_name],
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


