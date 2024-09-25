import os
import numpy as np

class SwingTrajectoryController:
    def __init__(self,
                 step_height: float,
                 swing_period: float,
                 position_gain_fb: np.ndarray,
                 velocity_gain_fb: np.ndarray,
                 generator: str) -> None:

        self.generator = generator

        if (self.generator == "ndcurves"):
            from .swing_generators.ndcurves_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        elif (self.generator == "scipy"):
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)

        self.position_gain_fb = position_gain_fb
        self.velocity_gain_fb = velocity_gain_fb
        self.swing_period = swing_period
        self.swing_time = [0, 0, 0, 0]

    def regenerate_swing_trajectory_generator(self, step_height: float, swing_period: float) -> None:
        if (self.generator == "ndcurves"):
            from .swing_generators.ndcurves_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        elif (self.generator == "scipy"):
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        self.swing_period = swing_period

    def compute_swing_control(self,
                              leg_id,
                              q_dot,
                              J,
                              J_dot,
                              lift_off,
                              touch_down,
                              foot_pos,
                              foot_vel,
                              h,
                              mass_matrix):
        """TODO: Docstring.

        Args:
        ----
            model:
            q:
            q_dot:
            J:
            J_dot:
            lift_off:
            touch_down:
            swing_time:
            foot_pos:
            foot_vel:
            h:
            mass_matrix:

        Returns:
        -------

        """
        # Compute trajectory references
        des_foot_pos, des_foot_vel, des_foot_acc = self.swing_generator.compute_trajectory_references(
            self.swing_time[leg_id],
            lift_off,
            touch_down
            )

        err_pos = des_foot_pos - foot_pos
        err_pos = err_pos.reshape((3,))

        err_vel = des_foot_vel - foot_vel
        err_vel = err_vel.reshape((3,))

        accelleration = des_foot_acc + \
                        self.position_gain_fb * (err_pos) + \
                        self.velocity_gain_fb * (err_vel)

        accelleration = accelleration.reshape((3,))

        # Compute inertia matrix in task space.
        # Mass Matrix and centrifugal missing
        tau_swing = J.T @ (self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel))
        tau_swing += mass_matrix @ np.linalg.pinv(J) @ (accelleration - J_dot @ q_dot) + h

        return tau_swing, des_foot_pos, des_foot_vel

    def update_swing_time(self, current_contact, legs_order, dt):
        for leg_id, leg_name in enumerate(legs_order):
            # Swing time reset
            if current_contact[leg_id] == 0:
                if self.swing_time[leg_id] < self.swing_period:
                    self.swing_time[leg_id] = self.swing_time[leg_id] + dt
            else:
                self.swing_time[leg_id] = 0

    def check_apex_condition(self, current_contact, interval=0.02):
        optimize_swing = 0
        for leg_id in range(4):
            # Swing time check
            if (current_contact[leg_id] == 0):
                if ((self.swing_time[leg_id] > (self.swing_period / 2.) - interval) and \
                        (self.swing_time[leg_id] < (self.swing_period / 2.) + interval)):
                    optimize_swing = 1
        return optimize_swing

    def check_full_stance_condition(self, current_contact):
        stance = 1
        # If one leg is not in stance, the robot is not in full stance
        for leg_id in range(4):
            if (current_contact[leg_id] == 0):
                stance = 0
        return stance


# Example:
if __name__ == "__main__":
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


    np.set_printoptions(precision=3, suppress=True)

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
                       ref_base_lin_vel=0.0,  # pass a float for a fixed value
                       ground_friction_coeff=1.5,  # pass a float for a fixed value
                       base_vel_command_type="human",  # "forward", "random", "forward+rotate", "human"
                       state_obs_names=state_observables_names,  # Desired quantities in the 'state' vec
                       )
    env.reset(random=False)
    env.render()  # Pass in the first render call any mujoco.viewer.KeyCallbackType
    

    feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None

    # Jacobian matrices
    jac_feet_prev = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
    jac_feet_dot = LegsAttr(*[np.zeros((3, env.mjModel.nv)) for _ in range(4)])
    # Torque vector
    tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])



    # Quadruped PyMPC controller initialization -------------------------------------------------------------
    from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
    from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
    from quadruped_pympc.interfaces.wb_interface import WBInterface


    wb_interface = WBInterface(initial_feet_pos = env.feet_pos(frame='world'), legs_order = legs_order)
    

    nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                RL=np.zeros(3), RR=np.zeros(3))
    nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                    RL=np.zeros(3), RR=np.zeros(3))
    best_sample_freq = wb_interface.pgg.step_freq

    import copy
    initial_pos=copy.deepcopy(np.array([0,0,0.6]))
    initial_angle=copy.deepcopy(env.mjData.qpos[3:7])

    while True:
        env.mjData.qpos[0:3] = initial_pos
        env.mjData.qpos[3:7] = initial_angle
        env.mjData.qvel[0:3] = 0
        env.mjData.qvel[3:7] = 0
        

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



        # Update the state and reference -------------------------
        state_current, \
        ref_state, \
        contact_sequence, \
        ref_feet_pos, \
        ref_feet_constraints, \
        contact_sequence_dts, \
        contact_sequence_lenghts, \
        step_height, \
        optimize_swing = wb_interface.update_state_and_reference(base_pos,
                                                base_lin_vel,
                                                base_ori_euler_xyz,
                                                base_ang_vel,
                                                feet_pos,
                                                hip_pos,
                                                heightmaps,
                                                legs_order,
                                                simulation_dt,
                                                ref_base_lin_vel,
                                                ref_base_ang_vel)
        
        

        ref_feet_pos.FL[2] = 0.3
        ref_feet_pos.FR[2] = 0.3
        ref_feet_pos.RL[2] = 0.3
        ref_feet_pos.RR[2] = 0.3
        wb_interface.frg.lift_off_positions.FL[2] = 0.3
        wb_interface.frg.lift_off_positions.FR[2] = 0.3
        wb_interface.frg.lift_off_positions.RL[2] = 0.3
        wb_interface.frg.lift_off_positions.RR[2] = 0.3
        nmpc_footholds = LegsAttr(FL=ref_feet_pos['FL'], FR=ref_feet_pos['FR'],
                                    RL=ref_feet_pos['RL'], RR=ref_feet_pos['RR'])




        
        

        # Compute Swing and Stance Torque ---------------------------------------------------------------------------
        tau = wb_interface.compute_stance_and_swing_torque(simulation_dt,
                                                    qvel,
                                                    feet_jac,
                                                    jac_feet_dot,
                                                    feet_pos,
                                                    feet_vel,
                                                    legs_qfrc_bias,
                                                    legs_mass_matrix,
                                                    nmpc_GRFs,
                                                    nmpc_footholds,
                                                    legs_qvel_idx,
                                                    tau,
                                                    optimize_swing,
                                                    best_sample_freq)
        

        action = np.zeros(env.mjModel.nu)
        action[env.legs_tau_idx.FL] = tau.FL
        action[env.legs_tau_idx.FR] = tau.FR
        action[env.legs_tau_idx.RL] = tau.RL
        action[env.legs_tau_idx.RR] = tau.RR
        state, reward, is_terminated, is_truncated, info = env.step(action=action)
        
        


        _, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)

        # Plot the swing trajectory
        feet_traj_geom_ids = plot_swing_mujoco(viewer=env.viewer,
                                                swing_traj_controller=wb_interface.stc,
                                                swing_period=wb_interface.stc.swing_period,
                                                swing_time=LegsAttr(FL=wb_interface.stc.swing_time[0],
                                                                    FR=wb_interface.stc.swing_time[1],
                                                                    RL=wb_interface.stc.swing_time[2],
                                                                    RR=wb_interface.stc.swing_time[3]),
                                                lift_off_positions=wb_interface.frg.lift_off_positions,
                                                nmpc_footholds=nmpc_footholds,
                                                ref_feet_pos=ref_feet_pos,
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