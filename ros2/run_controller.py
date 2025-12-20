import rclpy 
from rclpy.node import Node 
from dls2_interface.msg import BaseState, BlindState, ControlSignal, TrajectoryGenerator, TimeDebug
from sensor_msgs.msg import Joy

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import threading
import multiprocessing
from multiprocessing import shared_memory, Value

import copy

# Gym and Simulation related imports
import mujoco
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.quadruped_utils import LegsAttr


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

# to reserve the core 4, 5 for the process, add in etc/default/grub
# GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=4-5" in etc/default/grub
# and then sudo update-grub
# and uncomment the lines below
#affinity_mask = {4, 5} 
#os.sched_setaffinity(pid, affinity_mask)

#for real time, launch it with chrt -r 99 python3 run_controller.py


USE_DLS_CONVENTION = False

USE_THREADED_MPC = False
USE_PROCESS_QUEUE_MPC = False
USE_PROCESS_SHARED_MEMORY_MPC = False
if(USE_PROCESS_SHARED_MEMORY_MPC):
        # -------------------- Shared-memory layout for MPC → WBC --------------------------------------
    # Payload layout (float64):
    # 0..11   : GRF   (4 legs × 3)
    # 12..23  : Footholds (4×3)
    # 24..35  : Joints pos (4×3)
    # 36..47  : Joints vel (4×3)
    # 48..59  : Joints acc (4×3)
    # 60..71  : Predicted state (12)
    # 72      : best_sample_freq (1)
    # 73      : last_mpc_loop_time (1)
    # 74      : stamp_mono (1)
    N_DBL = 75
    IDX_GRF   = slice(0, 12)
    IDX_FH    = slice(12, 24)
    IDX_JP    = slice(24, 36)
    IDX_JV    = slice(36, 48)
    IDX_JA    = slice(48, 60)
    IDX_PRED  = slice(60, 72)
    IDX_BSF   = 72
    IDX_LAST  = 73
    IDX_STAMP = 74

    def legsattr_to12(legs: LegsAttr) -> np.ndarray:
        return np.concatenate([np.asarray(legs.FL).reshape(-1),
                            np.asarray(legs.FR).reshape(-1),
                            np.asarray(legs.RL).reshape(-1),
                            np.asarray(legs.RR).reshape(-1)], axis=0)


    def vec12_to_legsattr(vec12: np.ndarray) -> LegsAttr:
        v = np.asarray(vec12).reshape(4, 3)
        return LegsAttr(FL=v[0].copy(), FR=v[1].copy(), RL=v[2].copy(), RR=v[3].copy())
    

MPC_FREQ = 100 

USE_SCHEDULER = False # This enable a call to the run function every tot seconds, instead of as fast as possible
SCHEDULER_FREQ = 250 # this is only valid if USE_SCHEDULER is True

USE_FIXED_LOOP_TIME = False # This is used to fix the clock time of periodic gait gen to 1/SCHEDULER_FREQ
USE_SATURATED_LOOP_TIME = True # This is used to cap the clock time of periodic gait gen to max 250Hz

USE_SMOOTH_VELOCITY = False
USE_SMOOTH_HEIGHT = True

# Shell for the controllers ----------------------------------------------
class Quadruped_PyMPC_Node(Node):
    def __init__(self):
        super().__init__('Quadruped_PyMPC_Node')

        # Subscribers and Publishers
        self.subscription_base_state = self.create_subscription(BaseState,"/base_state", self.get_base_state_callback, 1)
        self.subscription_blind_state = self.create_subscription(BlindState,"/blind_state", self.get_blind_state_callback, 1)
        self.subscription_joy = self.create_subscription(Joy,"joy", self.get_joy_callback, 1)
        self.publisher_control_signal = self.create_publisher(ControlSignal,"/quadruped_pympc_torques", 1)
        self.publisher_trajectory_generator = self.create_publisher(TrajectoryGenerator,"/trajectory_generator", 1)
        self.publisher_time_debug = self.create_publisher(TimeDebug,"/time_debug", 1)
        if(USE_SCHEDULER):
            self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_control_callback)
        

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
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        
        self.feet_traj_geom_ids, self.feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
        self.legs_order = ["FL", "FR", "RL", "RR"]
        self.env.reset(random=False)
        self.last_mpc_time = time.time()


        # Quadruped PyMPC controller initialization -------------------------------------------------------------
        from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
        from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
        from quadruped_pympc.interfaces.wb_interface import WBInterface

        self.wb_interface = WBInterface(initial_feet_pos = self.env.feet_pos(frame='world'), legs_order = self.legs_order)
        self.srbd_controller_interface = SRBDControllerInterface()

        # This variable are shared between the MPC and the whole body controller
        # in the case of the use of thread. In any case, i initialize them here
        self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_pos = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_vel = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
        self.nmpc_joints_acc = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
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
        if(USE_PROCESS_QUEUE_MPC):
            self.input_data_process = multiprocessing.Queue(maxsize=1)
            self.output_data_process = multiprocessing.Queue(maxsize=1)
            process_mpc = multiprocessing.Process(target=self.compute_mpc_process_queue_callback, args=(self.input_data_process, self.output_data_process))
            process_mpc.daemon = True
            process_mpc.start()
        if(USE_PROCESS_SHARED_MEMORY_MPC):
            # WBC → MPC: keep a latest-only queue for complex inputs
            self.input_data_process = multiprocessing.Queue(maxsize=1)

            # MPC → WBC: shared-memory SPSC with seqlock
            self.shm_out = shared_memory.SharedMemory(create=True, size=N_DBL * 8)
            self.shm_out_name = self.shm_out.name
            np.ndarray((N_DBL,), dtype=np.float64, buffer=self.shm_out.buf)[:] = 0.0
            self.seq_out = Value('Q', 0, lock=False)  # 64-bit sequence, even=stable

            process_mpc = multiprocessing.Process(
                target=self.compute_mpc_process_shared_memory_callback,
                args=(self.input_data_process, self.shm_out_name, self.seq_out),
            )
            process_mpc.daemon = True
            process_mpc.start()
            

        # Interactive Command Line ----------------------------
        from console import Console
        self.console = Console(controller_node=self)
        thread_console = threading.Thread(target=self.console.interactive_command_line)
        thread_console.daemon = True
        thread_console.start()


        # Init for real robot and simulation gain, since real robot needs different values
        #    self.wb_interface.stc.position_gain_fb = 100
        #    self.wb_interface.stc.velocity_gain_fb = 10
        #    self.wb_interface.stc.use_feedback_linearization = False
        #    self.wb_interface.stc.use_friction_compensation = False


    def compute_mpc_thread_callback(self):
        # This thread runs forever!
        last_mpc_thread_time = time.time()
        while True:
            if time.time() - last_mpc_thread_time > 1.0 / MPC_FREQ:
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


    def compute_mpc_process_queue_callback(self, input_data_process, output_data_process):
        pid = os.getpid()
        os.system("sudo renice -n -21 -p " + str(pid))
        os.system("sudo echo -20 > /proc/" + str(pid) + "/autogroup")
        #affinity_mask = {6, 7} 
        #os.sched_setaffinity(pid, affinity_mask)
        
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
                
                
                if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']):
                    # If the controller is gradient and is using RTI, we need to linearize the mpc after its computation
                    # this helps to minize the delay between new state->control in a real case scenario.
                    self.srbd_controller_interface.compute_RTI()
                last_mpc_process_time = time.time()


    def compute_mpc_process_shared_memory_callback(self, input_data_process, shm_out_name: str, seq_out: Value):
        pid = os.getpid()
        os.system("sudo renice -n -21 -p " + str(pid))
        os.system("sudo echo -20 > /proc/" + str(pid) + "/autogroup")
        #affinity_mask = {6, 7} 
        #os.sched_setaffinity(pid, affinity_mask)

        shm = shared_memory.SharedMemory(name=shm_out_name)
        arr = np.ndarray((N_DBL,), dtype=np.float64, buffer=shm.buf)

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

                # Publish to SHM with seqlock: odd=writing, even=stable
                s = seq_out.value
                if s % 2 == 0:
                    seq_out.value = s + 1
                # pack payload
                arr[IDX_GRF]  = legsattr_to12(nmpc_GRFs)
                arr[IDX_FH]   = legsattr_to12(nmpc_footholds)
                arr[IDX_JP]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
                arr[IDX_JV]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
                arr[IDX_JA]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
                arr[IDX_PRED] = np.asarray(nmpc_predicted_state).reshape(-1)[:12]
                arr[IDX_BSF]  = float(best_sample_freq)
                arr[IDX_LAST] = float(last_mpc_loop_time)
                arr[IDX_STAMP]= float(time.monotonic())
                # mark stable
                seq_out.value = (s | 1) + 1

                if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']:
                    self.srbd_controller_interface.compute_RTI()


    def get_base_state_callback(self, msg):
        
        if(USE_SMOOTH_HEIGHT):
            # Smooth the height of the base
            self.position[2] = 0.5*self.position[2] + 0.5*np.array(msg.pose.position)[2]
        else:
            self.position[2] = np.array(msg.pose.position)[2]
        self.position[0:2] = np.array(msg.pose.position)[0:2]


        if(USE_SMOOTH_VELOCITY):
            self.linear_velocity = 0.5*self.linear_velocity + 0.5*np.array(msg.velocity.linear)
        else:
            self.linear_velocity = np.array(msg.velocity.linear)

        # For the quaternion, the order is [w, x, y, z] on mujoco, and [x, y, z, w] on DLS2
        self.orientation = np.roll(np.array(msg.pose.orientation), 1)
        # For the angular velocity, mujoco is in the base frame, and DLS2 is in the world frame
        self.angular_velocity = np.array(msg.velocity.angular) 


        self.first_message_base_arrived = True



    def get_blind_state_callback(self, msg):
        
        self.joint_positions = np.array(msg.joints_position)
        self.joint_velocities = np.array(msg.joints_velocity)
        self.feet_contact = np.array(msg.feet_contact)

        if(USE_DLS_CONVENTION):
            # Fix convention DLS2
            self.joint_positions[0] = -self.joint_positions[0]
            self.joint_positions[6] = -self.joint_positions[6]
            self.joint_velocities[0] = -self.joint_velocities[0]
            self.joint_velocities[6] = -self.joint_velocities[6]

        self.first_message_joints_arrived = True
        

        if(not USE_SCHEDULER):
            self.compute_control_callback()



    def get_joy_callback(self, msg):
        """
        Callback function to handle joystick input. Joystick used is a 
        8Bitdi Ultimate 2C Wireless Controller.
        """
        self.env._ref_base_lin_vel_H[0] = msg.axes[1]/3.5  # Forward/Backward
        self.env._ref_base_lin_vel_H[1] = msg.axes[0]/3.5  # Left/Right
        self.env._ref_base_ang_yaw_dot = msg.axes[3]/2.  # Yaw


        #kill the node if the button is pressed
        if msg.buttons[8] == 1:
            self.get_logger().info("Joystick button pressed, shutting down the node.") 
            # This will kill the robot hal
            os.system("kill -9 $(ps -u | grep -m 1 hal | grep -o \"^[^ ]* *[0-9]*\" | grep -o \"[0-9]*\")")
            # This will kill the process running this script
            os.system("pkill -f play_ros2.py") 
            exit(0)




    def compute_control_callback(self):
        
        # Update the loop time
        if(USE_FIXED_LOOP_TIME):
            simulation_dt = 1./SCHEDULER_FREQ
        else:
            start_time = time.perf_counter()
            if(self.last_start_time is not None):
                self.loop_time = (start_time - self.last_start_time)
            self.last_start_time = start_time
            simulation_dt = self.loop_time
            
            if(USE_SATURATED_LOOP_TIME):
                if(simulation_dt > 0.005):
                    simulation_dt = 0.005

        # Safety check to not do anything until a first base and blind state are received
        if(self.first_message_base_arrived==False and self.first_message_joints_arrived==False):
            return

        
        # Update the mujoco model
        #self.env.mjData.qpos[0:3] = copy.deepcopy(self.position) # s.e. height
        self.env.mjData.qpos[0:3] = np.zeros(3) # proprioceptive height
        self.env.mjData.qpos[3:7] = copy.deepcopy(self.orientation)
        self.env.mjData.qvel[0:3] = copy.deepcopy(self.linear_velocity)
        self.env.mjData.qvel[3:6] = copy.deepcopy(self.angular_velocity)
        self.env.mjData.qpos[7:] = copy.deepcopy(self.joint_positions)
        self.env.mjData.qvel[6:] = copy.deepcopy(self.joint_velocities)
        self.env.mjModel.opt.timestep = simulation_dt
        self.env.mjModel.opt.disableflags = 16 # Disable the collision detection
        mujoco.mj_forward(self.env.mjModel, self.env.mjData)   


        # And get the state of the robot
        legs_order = ["FL", "FR", "RL", "RR"]
        feet_pos = self.env.feet_pos(frame='world')
        feet_vel = self.env.feet_vel(frame='world')
        hip_pos = self.env.hip_positions(frame='world')
        base_lin_vel = self.env.base_lin_vel(frame='world')
        base_ang_vel = self.env.base_ang_vel(frame='base')
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

        elif(USE_PROCESS_QUEUE_MPC):
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
        
        elif(USE_PROCESS_SHARED_MEMORY_MPC):
            if(not self.input_data_process.full()):
                self.input_data_process.put_nowait([state_current, ref_state, contact_sequence, inertia, optimize_swing, self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq])
            
            # Read MPC output from shared memory with seqlock and stale-data guard
            if self.shm_out is not None and self.seq_out is not None:
                s1 = self.seq_out.value
                if s1 % 2 == 0:  # writer not in progress
                    buf = np.ndarray((N_DBL,), dtype=np.float64, buffer=self.shm_out.buf)
                    tmp = buf.copy()  # local copy
                    s2 = self.seq_out.value
                    if s1 == s2 and (s2 % 2 == 0):
                        self.nmpc_GRFs        = vec12_to_legsattr(tmp[IDX_GRF])
                        self.nmpc_footholds   = vec12_to_legsattr(tmp[IDX_FH])
                        self.nmpc_joints_pos  = vec12_to_legsattr(tmp[IDX_JP])
                        self.nmpc_joints_vel  = vec12_to_legsattr(tmp[IDX_JV])
                        self.nmpc_joints_acc  = vec12_to_legsattr(tmp[IDX_JA])
                        self.nmpc_predicted_state = tmp[IDX_PRED].copy()
                        self.best_sample_freq  = float(tmp[IDX_BSF])
                        self.last_mpc_loop_time = float(tmp[IDX_LAST])
                        self.last_mpc_update_mono = float(tmp[IDX_STAMP])
                        
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


        if(USE_DLS_CONVENTION):
            # Fix convention DLS2
            self.tau.FL[0] = -self.tau.FL[0]
            self.tau.RL[0] = -self.tau.RL[0]

        control_signal_msg = ControlSignal()
        control_signal_msg.torques = np.concatenate([self.tau.FL, self.tau.FR, self.tau.RL, self.tau.RR], axis=0).flatten().tolist()
        self.publisher_control_signal.publish(control_signal_msg) 

        if(USE_DLS_CONVENTION):
            # Fix convention DLS2 and send PD target
            pd_target_joints_pos.FL[0] = -pd_target_joints_pos.FL[0]
            pd_target_joints_pos.RL[0] = -pd_target_joints_pos.RL[0]
            pd_target_joints_vel.FL[0] = -pd_target_joints_vel.FL[0]
            pd_target_joints_vel.RL[0] = -pd_target_joints_vel.RL[0]  

        trajectory_generator_msg = TrajectoryGenerator()
        trajectory_generator_msg.timestamp = float(self.get_clock().now().nanoseconds)
        trajectory_generator_msg.joints_position = np.concatenate([pd_target_joints_pos.FL, pd_target_joints_pos.FR, pd_target_joints_pos.RL, pd_target_joints_pos.RR], axis=0).flatten().tolist()
        trajectory_generator_msg.joints_velocity = np.concatenate([pd_target_joints_vel.FL, pd_target_joints_vel.FR, pd_target_joints_vel.RL, pd_target_joints_vel.RR], axis=0).flatten().tolist()
        trajectory_generator_msg.kp = (self.impedence_joint_position_gain).tolist()
        trajectory_generator_msg.kd = (self.impedence_joint_velocity_gain).tolist()
        self.publisher_trajectory_generator.publish(trajectory_generator_msg)

        time_debug_msg = TimeDebug()
        time_debug_msg.time_wbc = self.loop_time
        time_debug_msg.time_mpc = self.last_mpc_loop_time
        self.publisher_time_debug.publish(time_debug_msg)



def main():
    print('Hello from Quadruped-PyMPC ros interface.')
    rclpy.init()

    controller_node = Quadruped_PyMPC_Node()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
