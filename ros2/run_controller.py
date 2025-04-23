import rclpy 
from rclpy.node import Node 
from dls2_msgs.msg import BaseStateMsg, BlindStateMsg, ControlSignalMsg, TrajectoryGeneratorMsg

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import threading
import multiprocessing


import copy

# Gym and Simulation related imports
import mujoco
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.quadruped_utils import LegsAttr

# Helper functions for plotting
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from gym_quadruped.utils.mujoco.visual import render_vector
from gym_quadruped.utils.mujoco.visual import render_sphere

# Config imports
from quadruped_pympc import config as cfg

import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

# Set the priority of the process
pid = os.getpid()
print("PID: ", pid)
os.system("sudo renice -n -21 -p " + str(pid))
os.system("sudo echo -20 > /proc/" + str(pid) + "/autogroup")
affinity_mask = {4, 5} 
os.sched_setaffinity(pid, affinity_mask)
#for real time, launch it with chrt -r 99 python3 run_controller.py
# to reserve the core 4, 5 for the process, ass
#GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=4-5" in etc/default/grub, then sudo update-grub


USE_MUJOCO_RENDER = True
USE_MUJOCO_SIMULATION = True

USE_THREADED_MPC = False
USE_PROCESS_MPC = False
MPC_FREQ = 100 

USE_SCHEDULER = False # This enable a call to the run function every tot seconds, instead of as fast as possible
SCHEDULER_FREQ = 250 # this is only valid if USE_SCHEDULER is True

USE_FIXED_LOOP_TIME = False # This is used to fix the clock time of periodic gait gen to 1/SCHEDULER_FREQ
USE_SATURATED_LOOP_TIME = True # This is used to cap the clock time of periodic gait gen to max 250Hz

# Shell for the controllers ----------------------------------------------
class Quadruped_PyMPC_Node(Node):
    def __init__(self):
        super().__init__('Quadruped_PyMPC_Node')

        # Subscribers and Publishers
        self.subscription_base_state = self.create_subscription(BaseStateMsg,"/dls2/base_state", self.get_base_state_callback, 1)
        self.subscription_blind_state = self.create_subscription(BlindStateMsg,"/dls2/blind_state", self.get_blind_state_callback, 1)
        self.publisher_control_signal = self.create_publisher(ControlSignalMsg,"dls2/quadruped_pympc_torques", 1)
        self.publisher_trajectory_generator = self.create_publisher(TrajectoryGeneratorMsg,"dls2/trajectory_generator", 1)
        if(USE_SCHEDULER):
            self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_whole_body_control)

        # Safety check to not do anything until a first base and blind state are received
        self.first_message_base_arrived = False
        self.first_message_joints_arrived = False 

        # Timing stuff
        self.loop_time = 0.002
        self.last_start_time = None
        self.last_mpc_loop_time = 0.0

        # Base State
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.stance_status = np.zeros(4)
        # Blind State
        self.joint_positions = np.zeros(12)
        self.joint_velocities = np.zeros(12)
        self.feet_contact = np.zeros(4)
        # Desired PD gain
        self.impedence_joint_position_gain = np.ones(12)*cfg.simulation_params['impedence_joint_position_gain']
        self.impedence_joint_velocity_gain = np.ones(12)*cfg.simulation_params['impedence_joint_velocity_gain']


        # Mujoco env
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=cfg.simulation_params['dt'],
            base_vel_command_type="human"
        )
        
        self.feet_traj_geom_ids, self.feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
        self.legs_order = ["FL", "FR", "RL", "RR"]
        self.env.reset(random=False)
        self.last_mpc_time = time.time()


        
        self.last_render_time = time.time()
        if USE_MUJOCO_RENDER:
            self.env.render()

        

        # Quadruped PyMPC controller initialization -------------------------------------------------------------
        from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
        from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
        from quadruped_pympc.interfaces.wb_interface import WBInterface

        self.wb_interface = WBInterface(initial_feet_pos = self.env.feet_pos(frame='world'), legs_order = self.legs_order)

        self.srbd_controller_interface = SRBDControllerInterface()

        if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']):
            self.srbd_batched_controller_interface = SRBDBatchedControllerInterface()

        # This variable are shared between the MPC and the whole body controller
        # in the case of the use of thread. In any case, i initialize them here
        self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                    RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                        RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_pos = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                        RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_vel = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                        RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_acc = LegsAttr(FL=np.zeros(3), FR=np.zeros(3),
                                        RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_predicted_state = np.zeros(12)
        
        self.best_sample_freq = self.wb_interface.pgg.step_freq
        self.state_current = None
        self.ref_state = None
        self.contact_sequence = None
        self.inertia = None
        self.optimize_swing = None

        # Torque vector
        self.tau = LegsAttr(*[np.zeros((self.env.mjModel.nv, 1)) for _ in range(4)])
        # Torque limits
        tau_soft_limits_scalar = 0.9
        self.tau_limits = LegsAttr(
            FL=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.FL]*tau_soft_limits_scalar,
            FR=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.FR]*tau_soft_limits_scalar,
            RL=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.RL]*tau_soft_limits_scalar,
            RR=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.RR]*tau_soft_limits_scalar)

        # Let's start in FULL STANCE in any case
        self.wb_interface.pgg.gait_type = 7 

        # Threaded MPC
        if(USE_THREADED_MPC):
            thread_mpc = threading.Thread(target=self.compute_mpc_thread_callback)
            thread_mpc.daemon = True
            thread_mpc.start()
        if(USE_PROCESS_MPC):
            self.input_data_process = multiprocessing.Queue(maxsize=1)
            self.output_data_process = multiprocessing.Queue(maxsize=1)
            process_mpc = multiprocessing.Process(target=self.compute_mpc_process_callback, args=(self.input_data_process, self.output_data_process))
            process_mpc.daemon = True
            process_mpc.start()
            


        # Interactive Command Line ----------------------------
        from console import Console
        self.console = Console(controller_node=self)
        thread_console = threading.Thread(target=self.console.interactive_command_line)
        thread_console.daemon = True
        thread_console.start()




    def compute_mpc_thread_callback(self):
        # This thread runs forever!
        last_mpc_thread_time = time.time()
        while True:
            #if time.time() - last_mpc_thread_time > 1.0 / MPC_FREQ:
                if(self.state_current is not None):
                    self.nmpc_GRFs,  \
                    self.nmpc_footholds, \
                    self.nmpc_joints_pos, \
                    self.nmpc_joints_vel, \
                    self.nmpc_joints_acc, \
                    self.best_sample_freq,\
                    self.nmpc_predicted_state = self.srbd_controller_interface.compute_control(self.state_current,
                                                                            self.ref_state,
                                                                            self.contact_sequence,
                                                                            self.inertia,
                                                                            self.wb_interface.pgg.phase_signal,
                                                                            self.wb_interface.pgg.step_freq,
                                                                            self.optimize_swing)
                    if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']):
                        # If the controller is gradient and is using RTI, we need to linearize the mpc after its computation
                        # this helps to minize the delay between new state->control in a real case scenario.
                        self.srbd_controller_interface.compute_RTI()
                    last_mpc_thread_time = time.time()


    def compute_mpc_process_callback(self, input_data_process, output_data_process):
        pid = os.getpid()
        os.system("sudo renice -n -21 -p " + str(pid))
        os.system("sudo echo -20 > /proc/" + str(pid) + "/autogroup")
        affinity_mask = {6, 7} 
        os.sched_setaffinity(pid, affinity_mask)
        # This process runs forever!
        last_mpc_process_time = time.time()
        while True:
            #if time.time() - last_mpc_process_time > 1.0 / MPC_FREQ:
                if(not input_data_process.empty()):
                    data = input_data_process.get()
                    state_current = data[0]
                    ref_state = data[1]
                    contact_sequence = data[2]
                    inertia = data[3]
                    optimize_swing = data[4]
                    phase_signal = data[5]
                    step_freq = data[6]
                    
                    nmpc_GRFs,  \
                    nmpc_footholds, \
                    nmpc_joints_pos, \
                    nmpc_joints_vel, \
                    nmpc_joints_acc, \
                    best_sample_freq,\
                    nmpc_predicted_state = self.srbd_controller_interface.compute_control(state_current,
                                                                            ref_state,
                                                                            contact_sequence,
                                                                            inertia,
                                                                            phase_signal,
                                                                            step_freq,
                                                                            optimize_swing)
                    
                    
                    last_mpc_loop_time = time.time() - last_mpc_process_time
                    output_data_process.put([nmpc_GRFs, nmpc_footholds, nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc, best_sample_freq, nmpc_predicted_state, last_mpc_loop_time])
                    last_mpc_process_time = time.time()
                    
                    #if(time.time() - last_mpc_process_time < 1.0 / MPC_FREQ):
                    #    time.sleep(1.0 / MPC_FREQ - (last_mpc_process_time - last_mpc_process_time))

                    if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']):
                        # If the controller is gradient and is using RTI, we need to linearize the mpc after its computation
                        # this helps to minize the delay between new state->control in a real case scenario.
                        self.srbd_controller_interface.compute_RTI()
                        
                




    def get_base_state_callback(self, msg):
        
        self.position = np.array(msg.position)
        # For the quaternion, the order is [w, x, y, z] on mujoco, and [x, y, z, w] on DLS2
        self.orientation = np.roll(np.array(msg.orientation), 1)
        self.linear_velocity = np.array(msg.linear_velocity)
        # For the angular velocity, mujoco is in the base frame, and DLS2 is in the world frame
        self.angular_velocity = np.array(msg.angular_velocity) 
        #R = self.env.base_configuration[0:3, 0:3]
        #self.angular_velocity = R.T @ self.angular_velocity # To comment if robot real
        self.stance_status = np.array(msg.stance_status)

        self.first_message_base_arrived = True



    def get_blind_state_callback(self, msg):
        
        self.joint_positions = np.array(msg.joints_position)
        self.joint_velocities = np.array(msg.joints_velocity)
        self.feet_contact = np.array(msg.feet_contact)

        # Fix convention DLS2
        self.joint_positions[0] = -self.joint_positions[0]
        self.joint_positions[6] = -self.joint_positions[6]
        self.joint_velocities[0] = -self.joint_velocities[0]
        self.joint_velocities[6] = -self.joint_velocities[6]

        self.first_message_joints_arrived = True
        

        if(not USE_SCHEDULER):
            self.compute_whole_body_control()




    def compute_whole_body_control(self):
        
        # Update the loop time
        if(USE_FIXED_LOOP_TIME):
            simulation_dt = 1./SCHEDULER_FREQ
        elif(USE_SATURATED_LOOP_TIME):
            start_time = time.perf_counter()
            if(self.last_start_time is not None):
                self.loop_time = (start_time - self.last_start_time)
            self.last_start_time = start_time
            simulation_dt = self.loop_time
            if(simulation_dt > 0.005):
                simulation_dt = 0.005
        else:
            start_time = time.perf_counter()
            if(self.last_start_time is not None):
                self.loop_time = (start_time - self.last_start_time)
            self.last_start_time = start_time
            simulation_dt = self.loop_time

        # Safety check to not do anything until a first base and blind state are received
        if(not USE_MUJOCO_SIMULATION and self.first_message_base_arrived==False and self.first_message_joints_arrived==False):
            return

        
        # Update the mujoco model
        if(not USE_MUJOCO_SIMULATION):
            self.env.mjData.qpos[0:3] = copy.deepcopy(self.position)
            self.env.mjData.qpos[3:7] = copy.deepcopy(self.orientation)
            self.env.mjData.qvel[0:3] = copy.deepcopy(self.linear_velocity)
            self.env.mjData.qvel[3:6] = copy.deepcopy(self.angular_velocity)
            self.env.mjData.qpos[7:] = copy.deepcopy(self.joint_positions)
            self.env.mjData.qvel[6:] = copy.deepcopy(self.joint_velocities)
            self.env.mjModel.opt.timestep = simulation_dt
            self.env.mjModel.opt.disableflags = 16 # Disable the collision detection
            mujoco.mj_forward(self.env.mjModel, self.env.mjData)     



        # Update value from SE or Simulator ----------------------
        legs_order = ["FL", "FR", "RL", "RR"]
        feet_pos = self.env.feet_pos(frame='world')
        feet_vel = self.env.feet_vel(frame='world')
        hip_pos = self.env.hip_positions(frame='world')
        base_lin_vel = self.env.base_lin_vel(frame='world')
        base_ang_vel = self.env.base_ang_vel(frame='world')
        base_ori_euler_xyz = self.env.base_ori_euler_xyz
        base_pos = self.env.base_pos
        com_pos = self.env.com




        # Get the reference base velocity in the world frame
        ref_base_lin_vel, ref_base_ang_vel = self.env.target_base_vel()
        
        # Get the inertia matrix
        if(cfg.simulation_params['use_inertia_recomputation']):
            inertia = self.env.get_base_inertia().flatten()  # Reflected inertia of base at qpos, in world frame
        else:
            inertia = cfg.inertia.flatten()

        # Get the qpos and qvel
        qpos, qvel = self.env.mjData.qpos, self.env.mjData.qvel
        joints_pos = LegsAttr(FL=qpos[7:10], FR=qpos[10:13],
                                RL=qpos[13:16], RR=qpos[16:19])
        
        # Get Centrifugal, Coriolis, Gravity, Friction for the swing controller
        legs_mass_matrix = self.env.legs_mass_matrix
        legs_qfrc_bias = self.env.legs_qfrc_bias
        legs_qfrc_passive = self.env.legs_qfrc_passive

        # Compute feet jacobian
        feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)
        feet_jac_dot = self.env.feet_jacobians_dot(frame='world', return_rot_jac=False)


        # Idx of the leg
        legs_qvel_idx = self.env.legs_qvel_idx
        legs_qpos_idx = self.env.legs_qpos_idx

        # Get the heightmaps
        heightmaps = None


        
        # Update the state and reference -------------------------
        state_current, \
        ref_state, \
        contact_sequence, \
        step_height, \
        optimize_swing = self.wb_interface.update_state_and_reference(com_pos,
                                                base_pos,
                                                base_lin_vel,
                                                base_ori_euler_xyz,
                                                base_ang_vel,
                                                feet_pos,
                                                hip_pos,
                                                joints_pos,
                                                heightmaps,
                                                legs_order,
                                                simulation_dt,
                                                ref_base_lin_vel,
                                                ref_base_ang_vel)


        
        # Console commands hacks
        ref_state["ref_position"][2] += self.console.height_delta
        ref_state["ref_orientation"][1] += self.console.pitch_delta


        
        # Publish to the MPC controller
        if(USE_THREADED_MPC):
            self.state_current = state_current
            self.ref_state = ref_state
            self.contact_sequence = contact_sequence
            self.inertia = inertia
            self.optimize_swing = optimize_swing

        elif(USE_PROCESS_MPC):
            if(not self.input_data_process.full()):
                self.input_data_process.put_nowait([state_current, ref_state, contact_sequence, inertia, optimize_swing, self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq])

            if(not self.output_data_process.empty()):
                data = self.output_data_process.get_nowait()
                self.nmpc_GRFs = data[0]
                self.nmpc_footholds = data[1]
                self.nmpc_joints_pos = data[2]
                self.nmpc_joints_vel = data[3]
                self.nmpc_joints_acc = data[4]
                self.best_sample_freq = data[5]
                self.nmpc_predicted_state = data[6]
                self.last_mpc_loop_time = data[7]

        else:
            if time.time() - self.last_mpc_time > 1.0 / MPC_FREQ:
                self.nmpc_GRFs,  \
                self.nmpc_footholds, \
                self.nmpc_joints_pos, \
                self.nmpc_joints_vel, \
                self.nmpc_joints_acc, \
                self.best_sample_freq, \
                self.nmpc_predicted_state = self.srbd_controller_interface.compute_control(state_current,
                                                                        ref_state,
                                                                        contact_sequence,
                                                                        inertia,
                                                                        self.wb_interface.pgg.phase_signal,
                                                                        self.wb_interface.pgg.step_freq,
                                                                        optimize_swing)
                
                if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']):
                    # If the controller is gradient and is using RTI, we need to linearize the mpc after its computation
                    # this helps to minize the delay between new state->control in a real case scenario.
                    self.srbd_controller_interface.compute_RTI()
        

                # Update the gait
                if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']):
                    self.best_sample_freq = self.srbd_batched_controller_interface.optimize_gait(state_current,
                                                                            ref_state,
                                                                            inertia,
                                                                            self.wb_interface.pgg.phase_signal,
                                                                            self.wb_interface.pgg.step_freq,
                                                                            self.wb_interface.pgg.duty_factor,
                                                                            self.wb_interface.pgg.gait_type,
                                                                            optimize_swing)
                    
                self.last_mpc_time = time.time()
                
        
        
        # Compute Swing and Stance Torque ---------------------------------------------------------------------------
        self.tau, \
        pd_target_joints_pos, \
        pd_target_joints_vel = self.wb_interface.compute_stance_and_swing_torque(simulation_dt,
                                                                                qpos,
                                                                                qvel,
                                                                                feet_jac,
                                                                                feet_jac_dot,
                                                                                feet_pos,
                                                                                feet_vel,
                                                                                legs_qfrc_passive,
                                                                                legs_qfrc_bias,
                                                                                legs_mass_matrix,
                                                                                self.nmpc_GRFs,
                                                                                self.nmpc_footholds,
                                                                                legs_qpos_idx,
                                                                                legs_qvel_idx,
                                                                                self.tau,
                                                                                optimize_swing,
                                                                                self.best_sample_freq,
                                                                                self.nmpc_joints_pos,
                                                                                self.nmpc_joints_vel,
                                                                                self.nmpc_joints_acc, 
                                                                                self.nmpc_predicted_state)
        

        # Limit tau between tau_limits
        for leg in ["FL", "FR", "RL", "RR"]:
            tau_min, tau_max = self.tau_limits[leg][:, 0], self.tau_limits[leg][:, 1]
            self.tau[leg] = np.clip(self.tau[leg], tau_min, tau_max)


        if USE_MUJOCO_SIMULATION:
            action = np.zeros(self.env.mjModel.nu)
            action[self.env.legs_tau_idx.FL] = self.tau.FL
            action[self.env.legs_tau_idx.FR] = self.tau.FR
            action[self.env.legs_tau_idx.RL] = self.tau.RL
            action[self.env.legs_tau_idx.RR] = self.tau.RR
            self.env.step(action=action)
        


        # Fix convention DLS2
        self.tau.FL[0] = -self.tau.FL[0]
        self.tau.RL[0] = -self.tau.RL[0]

        control_signal_msg = ControlSignalMsg()
        control_signal_msg.torques = np.concatenate([self.tau.FL, self.tau.FR, self.tau.RL, self.tau.RR], axis=0).flatten()
        control_signal_msg.timestamp = self.loop_time #self.loop_time#self.last_mpc_loop_time
        self.publisher_control_signal.publish(control_signal_msg) 

        # Fix convention DLS2 and send PD target
        pd_target_joints_pos.FL[0] = -pd_target_joints_pos.FL[0]
        pd_target_joints_pos.RL[0] = -pd_target_joints_pos.RL[0]
        pd_target_joints_vel.FL[0] = -pd_target_joints_vel.FL[0]
        pd_target_joints_vel.RL[0] = -pd_target_joints_vel.RL[0]  

        trajectory_generator_msg = TrajectoryGeneratorMsg()
        trajectory_generator_msg.joints_position = np.concatenate([pd_target_joints_pos.FL, pd_target_joints_pos.FR, pd_target_joints_pos.RL, pd_target_joints_pos.RR], axis=0).flatten()
        trajectory_generator_msg.joints_velocity = np.concatenate([pd_target_joints_vel.FL, pd_target_joints_vel.FR, pd_target_joints_vel.RL, pd_target_joints_vel.RR], axis=0).flatten()
        trajectory_generator_msg.stance_legs[0] = bool(contact_sequence[0, 0])
        trajectory_generator_msg.stance_legs[1] = bool(contact_sequence[0, 1])
        trajectory_generator_msg.stance_legs[2] = bool(contact_sequence[0, 2])
        trajectory_generator_msg.stance_legs[3] = bool(contact_sequence[0, 3])
        trajectory_generator_msg.kp = self.impedence_joint_position_gain
        trajectory_generator_msg.kd = self.impedence_joint_velocity_gain
        trajectory_generator_msg.timestamp = self.last_mpc_loop_time
        self.publisher_trajectory_generator.publish(trajectory_generator_msg)

        
        
        # Render the simulation -----------------------------------------------------------------------------------
        if USE_MUJOCO_RENDER:
            RENDER_FREQ = 30
            # Render only at a certain frequency -----------------------------------------------------------------
            if time.time() - self.last_render_time > 1.0 / RENDER_FREQ or self.env.step_num == 1:
                _, _, feet_GRF = self.env.feet_contact_state(ground_reaction_forces=True)

                # Plot the swing trajectory
                self.feet_traj_geom_ids = plot_swing_mujoco(viewer=self.env.viewer,
                                                        swing_traj_controller=self.wb_interface.stc,
                                                        swing_period=self.wb_interface.stc.swing_period,
                                                        swing_time=LegsAttr(FL=self.wb_interface.stc.swing_time[0],
                                                                            FR=self.wb_interface.stc.swing_time[1],
                                                                            RL=self.wb_interface.stc.swing_time[2],
                                                                            RR=self.wb_interface.stc.swing_time[3]),
                                                        lift_off_positions=self.wb_interface.frg.lift_off_positions,
                                                        nmpc_footholds=self.nmpc_footholds,
                                                        ref_feet_pos=LegsAttr(FL=ref_state['ref_foot_FL'].reshape(3,1),
                                                                              FR=ref_state['ref_foot_FR'].reshape(3,1),
                                                                              RL=ref_state['ref_foot_RL'].reshape(3,1),
                                                                              RR=ref_state['ref_foot_RR'].reshape(3,1)),
                                                        early_stance_detector=self.wb_interface.esd,
                                                        geom_ids=self.feet_traj_geom_ids)
                
                
                # Update and Plot the heightmap
                if(cfg.simulation_params['visual_foothold_adaptation'] != 'blind'):
                    #if(stc.check_apex_condition(current_contact, interval=0.01)):
                    for leg_id, leg_name in enumerate(legs_order):
                        data = heightmaps[leg_name].data#.update_height_map(ref_feet_pos[leg_name], yaw=env.base_ori_euler_xyz[2])
                        if(data is not None):
                            for i in range(data.shape[0]):
                                for j in range(data.shape[1]):
                                        heightmaps[leg_name].geom_ids[i, j] = render_sphere(viewer=self.env.viewer,
                                                                                            position=([data[i][j][0][0],data[i][j][0][1],data[i][j][0][2]]),
                                                                                            diameter=0.01,
                                                                                            color=[0, 1, 0, .5],
                                                                                            geom_id=heightmaps[leg_name].geom_ids[i, j]
                                                                                            )
                            
                # Plot the GRF
                for leg_id, leg_name in enumerate(legs_order):
                    self.feet_GRF_geom_ids[leg_name] = render_vector(self.env.viewer,
                                                                vector=feet_GRF[leg_name],
                                                                pos=feet_pos[leg_name],
                                                                scale=np.linalg.norm(feet_GRF[leg_name]) * 0.005,
                                                                color=np.array([0, 1, 0, .5]),
                                                                geom_id=self.feet_GRF_geom_ids[leg_name])

                self.env.render()
                self.last_render_time = time.time()


def main():
    print('Hello from quadruped_pympc_ros_interface.')
    rclpy.init()

    controller_node = Quadruped_PyMPC_Node()

    if(USE_MUJOCO_SIMULATION):
        while True:
            controller_node.compute_whole_body_control()
    else:
        rclpy.spin(controller_node)
        controller_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
