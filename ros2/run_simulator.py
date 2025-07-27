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

USE_SCHEDULER = True # Use the scheduler to compute the control signal
SCHEDULER_FREQ = 500 # Frequency of the scheduler
RENDER_FREQ = 30

# Shell for the controllers ----------------------------------------------
class Simulator_Node(Node):
    def __init__(self):
        super().__init__('Simulator_Node')

        # Subscribers and Publishers
        self.publisher_base_state = self.create_publisher(BaseStateMsg,"/dls2/base_state", 1)
        self.publisher_blind_state = self.create_publisher(BlindStateMsg,"/dls2/blind_state", 1)
        self.subscriber_control_signal = self.create_subscription(ControlSignalMsg,"dls2/quadruped_pympc_torques", self.get_torques_callback, 1)
        self.subscriber_trajectory_generator = self.create_subscription(TrajectoryGeneratorMsg,"dls2/trajectory_generator", self.get_trajectory_generator_callback, 1)

        self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_simulator_step_callback)

        # Timing stuff
        self.loop_time = 0.002
        self.last_start_time = None
        self.last_mpc_loop_time = 0.0


        # Mujoco env
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=1.0/SCHEDULER_FREQ,
            base_vel_command_type="human"
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
        

        self.last_render_time = time.time()
        self.env.render()  

        # Torque vector
        self.desired_tau = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])

        # Desired PD 
        self.desired_joints_position = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
        self.desired_joints_velocity = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])


    def get_torques_callback(self, msg):
        
        torques = np.array(msg.torques)

        self.desired_tau.FL = torques[0:3]
        self.desired_tau.FR = torques[3:6]
        self.desired_tau.RL = torques[6:9]
        self.desired_tau.RR = torques[9:12]




    def get_trajectory_generator_callback(self, msg):

        joints_position = np.array(msg.joints_position)

        self.desired_joints_position.FL = joints_position[0:3]
        self.desired_joints_position.FR = joints_position[3:6]
        self.desired_joints_position.RL = joints_position[6:9]
        self.desired_joints_position.RR = joints_position[9:12]
        


    def compute_simulator_step_callback(self):

        action = np.zeros(self.env.mjModel.nu)
        action[self.env.legs_tau_idx.FL] = self.desired_tau.FL.reshape(-1)
        action[self.env.legs_tau_idx.FR] = self.desired_tau.FR.reshape(-1)
        action[self.env.legs_tau_idx.RL] = self.desired_tau.RL.reshape(-1)
        action[self.env.legs_tau_idx.RR] = self.desired_tau.RR.reshape(-1)
        self.env.step(action=action)

        base_lin_vel = self.env.base_lin_vel(frame='world')
        base_ang_vel = self.env.base_ang_vel(frame='base')
        base_pos = self.env.base_pos

        base_state_msg = BaseStateMsg()
        base_state_msg.position = base_pos
        base_state_msg.orientation = np.roll(self.env.mjData.qpos[3:7],-1)
        base_state_msg.linear_velocity = base_lin_vel
        base_state_msg.angular_velocity = base_ang_vel
        self.publisher_base_state.publish(base_state_msg)

        blind_state_msg = BlindStateMsg()
        blind_state_msg.joints_position = self.env.mjData.qpos[7:]
        blind_state_msg.joints_velocity = self.env.mjData.qvel[6:]
        self.publisher_blind_state.publish(blind_state_msg)


        # Render only at a certain frequency -----------------------------------------------------------------
        if time.time() - self.last_render_time > 1.0 / RENDER_FREQ:
            self.env.render()
            self.last_render_time = time.time()


def main():
    print('Hello from the gym_quadruped simulator.')
    rclpy.init()

    simulator_node = Simulator_Node()

    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
