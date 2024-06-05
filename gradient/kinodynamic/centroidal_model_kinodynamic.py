# Description: This file contains the class Centroidal_Model that defines the
# prediction model used by the MPC

# Authors: Giulio Turrisi -

import time
import unittest
import casadi as cs

import numpy as np
from acados_template import AcadosModel

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path)
sys.path.append(dir_path + '/../../')


from adam.casadi import KinDynComputations
from adam import Representations
from liecasadi import SE3

import config 

# Class that defines the prediction model of the NMPC
class Centroidal_Model_KinoDynamic:
    def __init__(self,) -> None: 
        """
        This method initializes the foothold generator Centroidal_Model, which creates
        the prediction model of the NMPC.
        """

        if(config.robot == 'go2'):
            urdf_filename = dir_path + '/../../simulation/robot_model/go2/go2.urdf' 
        elif(config.robot == 'aliengo'):
            urdf_filename = dir_path + '/../../simulation/robot_model/aliengo/aliengo.urdf'
        elif(config.robot == 'hyqreal'):
            urdf_filename = dir_path + '/../../simulation/robot_model/hyqreal/hyqreal.urdf'
        elif(config.robot == 'mini_cheetah'):
            urdf_filename = dir_path + '/../../simulation/robot_model/mini_cheetah/mini_cheetah.urdf'
        

        joint_list = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                       'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
                       'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                       'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']


        self.kindyn = KinDynComputations(urdfstring=urdf_filename, joints_name_list=joint_list)
        self.kindyn.set_frame_velocity_representation(representation=Representations.BODY_FIXED_REPRESENTATION)
        self.mass_mass_fun = self.kindyn.mass_matrix_fun()
        self.com_position_fun = self.kindyn.CoM_position_fun()
        self.bias_force_fun = self.kindyn.bias_force_fun()
        self.gravity_fun = self.kindyn.gravity_term_fun()
        self.coriolis_fun = self.kindyn.coriolis_term_fun()
        

        self.forward_kinematics_FL_fun = self.kindyn.forward_kinematics_fun("FL_foot")
        self.forward_kinematics_FR_fun = self.kindyn.forward_kinematics_fun("FR_foot")
        self.forward_kinematics_RR_fun = self.kindyn.forward_kinematics_fun("RL_foot")
        self.forward_kinematics_RL_fun = self.kindyn.forward_kinematics_fun("RR_foot")

        self.jacobian_FL_fun = self.kindyn.jacobian_fun("FL_foot")
        self.jacobian_FR_fun = self.kindyn.jacobian_fun("FR_foot")
        self.jacobian_RL_fun = self.kindyn.jacobian_fun("RL_foot")
        self.jacobian_RR_fun = self.kindyn.jacobian_fun("RR_foot")



        # Define state and its casadi variables
        com_position_x = cs.SX.sym("com_position_x")
        com_position_y = cs.SX.sym("com_position_y")
        com_position_z = cs.SX.sym("com_position_z")

        com_velocity_x = cs.SX.sym("com_velocity_x")
        com_velocity_y = cs.SX.sym("com_velocity_y")
        com_velocity_z = cs.SX.sym("com_velocity_z")
        
        roll = cs.SX.sym("roll", 1, 1)
        pitch = cs.SX.sym("pitch", 1, 1)
        yaw = cs.SX.sym("yaw", 1, 1)
        omega_x = cs.SX.sym("omega_x", 1, 1)
        omega_y = cs.SX.sym("omega_y", 1, 1)
        omega_z = cs.SX.sym("omega_z", 1, 1)
        
        joint_position_fl = cs.SX.sym("joint_position_fl", 3, 1)
        joint_position_fr = cs.SX.sym("joint_position_fr", 3, 1)
        joint_position_rl = cs.SX.sym("joint_position_rl", 3, 1)
        joint_position_rr = cs.SX.sym("joint_position_rr", 3, 1)


        com_position_z_integral = cs.SX.sym("com_position_z_integral")
        com_velocity_x_integral = cs.SX.sym("com_velocity_x_integral")
        com_velocity_y_integral = cs.SX.sym("com_velocity_y_integral")
        com_velocity_z_integral = cs.SX.sym("com_velocity_z_integral")
        roll_integral = cs.SX.sym("roll_integral")
        pitch_integral = cs.SX.sym("pitch_integral")
        omega_x_integral = cs.SX.sym("omega_x_integral")
        omega_y_integral = cs.SX.sym("omega_y_integral")
        omega_z_integral = cs.SX.sym("omega_z_integral")


        self.states = cs.vertcat(com_position_x,
                            com_position_y,
                            com_position_z,
                            com_velocity_x,
                            com_velocity_y,
                            com_velocity_z,
                            roll,
                            pitch,
                            yaw,
                            omega_x,
                            omega_y,
                            omega_z,
                            joint_position_fl,
                            joint_position_fr,
                            joint_position_rl,
                            joint_position_rr,
                            com_position_z_integral,
                            com_velocity_x_integral,
                            com_velocity_y_integral,
                            com_velocity_z_integral,
                            roll_integral,
                            pitch_integral)
        


        # Define state dot 
        self.states_dot = cs.vertcat(cs.SX.sym("linear_com_vel", 3, 1), 
                                     cs.SX.sym("linear_com_acc", 3, 1), 
                                     cs.SX.sym("euler_rates_base", 3, 1), 
                                     cs.SX.sym("angular_acc_base", 3, 1),
                                     cs.SX.sym("joint_vel_FL", 3, 1),
                                     cs.SX.sym("joint_vel_FR", 3, 1),
                                     cs.SX.sym("joint_vel_RL", 3, 1),
                                     cs.SX.sym("joint_vel_RR", 3, 1),
                                     cs.SX.sym("linear_com_vel_z_integral", 1, 1),
                                     cs.SX.sym("linear_com_acc_integral", 3, 1),
                                     cs.SX.sym("euler_rates_roll_integral", 1, 1),
                                     cs.SX.sym("euler_rates_pitch_integral", 1, 1))
        


        # Define input and its casadi variables
        joint_velocity_fl = cs.SX.sym("joint_velocity_fl", 3, 1)
        joint_velocity_fr = cs.SX.sym("joint_velocity_fr", 3, 1)
        joint_velocity_rl = cs.SX.sym("joint_velocity_rl", 3, 1)
        joint_velocity_rr = cs.SX.sym("joint_velocity_rr", 3, 1)

        foot_force_fl = cs.SX.sym("foot_force_fl", 3, 1)
        foot_force_fr = cs.SX.sym("foot_force_fr", 3, 1)
        foot_force_rl = cs.SX.sym("foot_force_rl", 3, 1)
        foot_force_rr = cs.SX.sym("foot_force_rr", 3, 1)

        self.inputs = cs.vertcat(joint_velocity_fl, 
                            joint_velocity_fr, 
                            joint_velocity_rl, 
                            joint_velocity_rr, 
                            foot_force_fl, 
                            foot_force_fr, 
                            foot_force_rl, 
                            foot_force_rr)
        

        # Usefull for debug what things goes where in y_ref in the compute_control function,
        # because there are a lot of variables
        self.y_ref = cs.vertcat(self.states, self.inputs)
        

        # Define acados parameters that can be changed at runtine
        self.stanceFL = cs.SX.sym("stanceFL", 1, 1)
        self.stanceFR = cs.SX.sym("stanceFR", 1, 1)
        self.stanceRL = cs.SX.sym("stanceRL", 1, 1)
        self.stanceRR = cs.SX.sym("stanceRR", 1, 1)
        self.stance_param = cs.vertcat(self.stanceFL , self.stanceFR , self.stanceRL , self.stanceRR)


        self.mu_friction = cs.SX.sym("mu_friction", 1, 1)
        self.stance_proximity = cs.SX.sym("stanceProximity", 4, 1)
        self.base_position = cs.SX.sym("base_position", 3, 1)
        self.base_yaw = cs.SX.sym("base_yaw", 1, 1)

        self.external_wrench = cs.SX.sym("external_wrench", 6, 1)

        self.inertia = cs.SX.sym("inertia", 9, 1)
        self.mass = cs.SX.sym("mass", 1, 1)


        # Not so useful, i can instantiate a casadi function for the fd
        param = cs.vertcat(self.stance_param, self.mu_friction, self.stance_proximity,
                           self.base_position, self.base_yaw, self.external_wrench, self.inertia, self.mass)
        fd = self.forward_dynamics(self.states, self.inputs, param)
        self.fun_forward_dynamics = cs.Function('fun_forward_dynamics', [self.states, self.inputs, param], [fd])


    def compute_b_R_w(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        #Z Y X rotations!
        Rx = cs.SX.eye(3)
        Rx[0,0] = 1   
        Rx[0,1] = 0
        Rx[0,2] = 0
        Rx[1,0] = 0
        Rx[1,1] = cs.cos(roll)
        Rx[1,2] = cs.sin(roll)
        Rx[2,0] = 0
        Rx[2,1] = -cs.sin(roll)
        Rx[2,2] = cs.cos(roll)
                
        Ry = cs.SX.eye(3)
        Ry[0,0] = cs.cos(pitch)
        Ry[0,1] = 0
        Ry[0,2] = -cs.sin(pitch)
        Ry[1,0] = 0
        Ry[1,1] = 1
        Ry[1,2] = 0
        Ry[2,0] = cs.sin(pitch)
        Ry[2,1] = 0
        Ry[2,2] = cs.cos(pitch)

        Rz = cs.SX.eye(3)
        Rz[0,0] = cs.cos(yaw)
        Rz[0,1] = cs.sin(yaw)
        Rz[0,2] = 0
        Rz[1,0] = -cs.sin(yaw)
        Rz[1,1] = cs.cos(yaw)
        Rz[1,2] = 0
        Rz[2,0] = 0
        Rz[2,1] = 0
        Rz[2,2] = 1
        
        b_R_w = Rx@Ry@Rz
        return b_R_w


    def forward_dynamics(self, states: np.ndarray, inputs: np.ndarray, param: np.ndarray) -> cs.SX:
        """
        This method computes the symbolic forward dynamics of the robot. It is used inside
        Acados to compute the prediction model. It fill the same variables as the one in
        self.states_dot.

        Args:
            states: A numpy array of shape (29,) representing the current state of the robot.
            inputs: A numpy array of shape (29,) representing the inputs to the robot.
            param: A numpy array of shape (4,) representing the parameters (contact status) of the robot.

        Returns:
            A CasADi SX object of shape (29,) representing the predicted state of the robot.
        """
        
        # Saving for clarity a bunch of variables
        joint_velocity_fl = inputs[0:3]
        joint_velocity_fr = inputs[3:6]
        joint_velocity_rl = inputs[6:9]
        joint_velocity_rr = inputs[9:12]
        foot_force_fl = inputs[12:15]
        foot_force_fr = inputs[15:18]
        foot_force_rl = inputs[18:21]
        foot_force_rr = inputs[21:24]

        com_position = states[0:3]
        joint_position_fl = states[12:15]
        joint_position_fr = states[15:18]
        joint_position_rl = states[18:21]
        joint_position_rr = states[21:24]

        stanceFL = param[0]
        stanceFR = param[1]
        stanceRL = param[2]
        stanceRR = param[3]


        external_wrench_linear = param[13:16]
        external_wrench_angular = param[16:19]


        # we should fix this via adam
        inertia = param[19:28]
        inertia = inertia.reshape((3, 3))
        mass = param[28]

        

        # FINAL linear_com_vel STATE (1)
        linear_com_vel = states[3:6]
        

        # FINAL linear_com_acc STATE (2)
        temp =  foot_force_fl@stanceFL 
        temp += foot_force_fr@stanceFR
        temp += foot_force_rl@stanceRL
        temp += foot_force_rr@stanceRR
        temp += external_wrench_linear
        gravity = np.array([0, 0, -9.81])
        linear_com_acc = (1/mass)@temp + gravity
        
        
        # Start to write the component of euler_rates_base and angular_acc_base STATES
        w = states[9:12]
        roll = states[6]
        pitch = states[7]
        yaw = states[8]
    
        conj_euler_rates = cs.SX.eye(3)
        conj_euler_rates[1, 1] = cs.cos(roll)
        conj_euler_rates[2, 2] = cs.cos(pitch)*cs.cos(roll)
        conj_euler_rates[2, 1] = -cs.sin(roll)
        conj_euler_rates[0, 2] = -cs.sin(pitch)
        conj_euler_rates[1, 2] = cs.cos(pitch)*cs.sin(roll)



        
        # FINAL euler_rates_base STATE (3)
        euler_rates_base = cs.inv(conj_euler_rates)@w
        

        # FINAL angular_acc_base STATE (4)
        b_R_w = self.compute_b_R_w(roll, pitch, yaw)



        H = cs.SX.eye(4)
        H[0:3, 0:3] = b_R_w.T
        H[0:3, 3] = com_position


        # Compute com pos, feet pos and inertia via ADAM
        joint_position = cs.vertcat(joint_position_fl, joint_position_fr, joint_position_rl, joint_position_rr)
        joints_velocities = cs.vertcat(joint_velocity_fl, joint_velocity_fr, joint_velocity_rl, joint_velocity_rr)
        self.foot_position_fl = self.forward_kinematics_FL_fun(H, joint_position)[0:3, 3]
        self.foot_position_fr = self.forward_kinematics_FR_fun(H, joint_position)[0:3, 3]
        self.foot_position_rl = self.forward_kinematics_RL_fun(H, joint_position)[0:3, 3]
        self.foot_position_rr = self.forward_kinematics_RR_fun(H, joint_position)[0:3, 3]

        
        # Compute the angular wrench
        temp2 =  cs.skew(self.foot_position_fl - com_position)@foot_force_fl@stanceFL 
        temp2 += cs.skew(self.foot_position_fr - com_position)@foot_force_fr@stanceFR
        temp2 += cs.skew(self.foot_position_rl - com_position)@foot_force_rl@stanceRL
        temp2 += cs.skew(self.foot_position_rr - com_position)@foot_force_rr@stanceRR


        temp2 = temp2 + external_wrench_angular
        angular_acc_base = cs.inv(inertia)@(b_R_w@temp2 - cs.skew(w)@inertia@w)


        """inertia = self.mass_mass_fun(H, joint_position)[3:6, 3:6]
        eta = self.bias_force_fun(H, joint_position, linear_com_vel, joints_velocities)
        eta += self.gravity_fun(H, joint_position, linear_com_vel, joints_velocities)
        eta += self.coriolis_fun(H, joint_position, linear_com_vel, joints_velocities)
        acc = cs.inv(inertia)@(-eta + 
                               self.jacobian_FL_fun(H, joint_position).T@foot_force_fl@stanceFL + 
                               self.jacobian_FR_fun(H, joint_position).T@foot_force_fr@stanceFR + 
                               self.jacobian_RL_fun(H, joint_position).T@foot_force_rl@stanceRL + 
                               self.jacobian_RR_fun(H, joint_position).T@foot_force_rr@stanceRR)
        linear_com_acc = acc[0:3]
        angular_acc_base = acc[3:6]"""
        


        # FINAL linear_foot_vel STATES (5,6,7,8)
        linear_joint_vel_FL = joint_velocity_fl
        linear_joint_vel_FR = joint_velocity_fr
        linear_joint_vel_RL = joint_velocity_rl
        linear_joint_vel_RR = joint_velocity_rr

        # Integral states
        integral_states = states[24:]
        integral_states[0] += states[2]
        integral_states[1] += states[3]
        integral_states[2] += states[4]
        integral_states[3] += states[5]
        integral_states[4] += roll
        integral_states[5] += pitch

        
        # The order of the return should be equal to the order of the states_dot
        return cs.vertcat(linear_com_vel, linear_com_acc, euler_rates_base, angular_acc_base, 
                          linear_joint_vel_FL, linear_joint_vel_FR, linear_joint_vel_RL, linear_joint_vel_RR, integral_states)
    
    


    def export_robot_model(self,) -> AcadosModel:
        """
        This method set some general properties of the NMPC, such as the params,
        prediction mode, etc...! It will be called in centroidal_nmpc.py
        """
 
        # dynamics
        self.param = cs.vertcat(self.stance_param, self.mu_friction, self.stance_proximity, self.base_position, 
                           self.base_yaw, self.external_wrench, self.inertia, self.mass)
        f_expl = self.forward_dynamics(self.states, self.inputs, self.param)
        f_impl = self.states_dot - f_expl

        acados_model = AcadosModel()
        acados_model.f_impl_expr = f_impl
        acados_model.f_expl_expr = f_expl
        acados_model.x = self.states
        acados_model.xdot = self.states_dot
        acados_model.u = self.inputs
        acados_model.p = self.param
        acados_model.name = "centroidal_model"


        return acados_model
    
