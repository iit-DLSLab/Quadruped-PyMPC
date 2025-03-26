# Description: This file contains the class Centroidal_Model that defines the
# prediction model used by the MPC

# Authors: Giulio Turrisi -

import os

import casadi as cs
import numpy as np
from acados_template import AcadosModel

import os

dir_path = os.path.dirname(os.path.realpath(__file__))

import sys

sys.path.append(dir_path)
sys.path.append(dir_path + "/../../../")

import config


# Class that defines the prediction model of the NMPC
class Centroidal_Model_Lyapunov:
    def __init__(self) -> None:
        """
        This method initializes the Centroidal_Model, which creates
        the prediction model of the NMPC.
        """

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

        foot_position_fl = cs.SX.sym("foot_position_fl", 3, 1)
        foot_position_fr = cs.SX.sym("foot_position_fr", 3, 1)
        foot_position_rl = cs.SX.sym("foot_position_rl", 3, 1)
        foot_position_rr = cs.SX.sym("foot_position_rr", 3, 1)

        com_position_z_integral = cs.SX.sym("com_position_z_integral")
        com_velocity_x_integral = cs.SX.sym("com_velocity_x_integral")
        com_velocity_y_integral = cs.SX.sym("com_velocity_y_integral")
        com_velocity_z_integral = cs.SX.sym("com_velocity_z_integral")
        roll_integral = cs.SX.sym("roll_integral")
        pitch_integral = cs.SX.sym("pitch_integral")
        omega_x_integral = cs.SX.sym("omega_x_integral")
        omega_y_integral = cs.SX.sym("omega_y_integral")
        omega_z_integral = cs.SX.sym("omega_z_integral")

        z1 = cs.SX.sym("z1", 3, 1)
        z2 = cs.SX.sym("z2", 3, 1)
        eta = cs.SX.sym("eta", 6, 1)
        phi = cs.SX.sym("theta", 3, 1)

        self.states = cs.vertcat(
            com_position_x,
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
            foot_position_fl,
            foot_position_fr,
            foot_position_rl,
            foot_position_rr,
            com_position_z_integral,
            com_velocity_x_integral,
            com_velocity_y_integral,
            com_velocity_z_integral,
            roll_integral,
            pitch_integral,
            z1,
            z2,
            eta,
            phi,
        )

        # Define state dot
        self.states_dot = cs.vertcat(
            cs.SX.sym("linear_com_vel", 3, 1),
            cs.SX.sym("linear_com_acc", 3, 1),
            cs.SX.sym("euler_rates_base", 3, 1),
            cs.SX.sym("angular_acc_base", 3, 1),
            cs.SX.sym("linear_vel_foot_FL", 3, 1),
            cs.SX.sym("linear_vel_foot_FR", 3, 1),
            cs.SX.sym("linear_vel_foot_RL", 3, 1),
            cs.SX.sym("linear_vel_foot_RR", 3, 1),
            cs.SX.sym("linear_com_vel_z_integral", 1, 1),
            cs.SX.sym("linear_com_acc_integral", 3, 1),
            cs.SX.sym("euler_rates_roll_integral", 1, 1),
            cs.SX.sym("euler_rates_pitch_integral", 1, 1),
            cs.SX.sym("z1_dot", 3, 1),
            cs.SX.sym("z2_dot", 3, 1),
            cs.SX.sym("eta_dot", 6, 1),
            cs.SX.sym("phi_dot", 3, 1),
        )

        # Define input and its casadi variables
        foot_velocity_fl = cs.SX.sym("foot_velocity_fl", 3, 1)
        foot_velocity_fr = cs.SX.sym("foot_velocity_fr", 3, 1)
        foot_velocity_rl = cs.SX.sym("foot_velocity_rl", 3, 1)
        foot_velocity_rr = cs.SX.sym("foot_velocity_rr", 3, 1)

        foot_force_fl = cs.SX.sym("foot_force_fl", 3, 1)
        foot_force_fr = cs.SX.sym("foot_force_fr", 3, 1)
        foot_force_rl = cs.SX.sym("foot_force_rl", 3, 1)
        foot_force_rr = cs.SX.sym("foot_force_rr", 3, 1)

        self.inputs = cs.vertcat(
            foot_velocity_fl,
            foot_velocity_fr,
            foot_velocity_rl,
            foot_velocity_rr,
            foot_force_fl,
            foot_force_fr,
            foot_force_rl,
            foot_force_rr,
        )

        # Usefull for debug what things goes where in y_ref in the compute_control function,
        # because there are a lot of variables
        self.y_ref = cs.vertcat(self.states, self.inputs)

        # Define acados parameters that can be changed at runtine
        self.stanceFL = cs.SX.sym("stanceFL", 1, 1)
        self.stanceFR = cs.SX.sym("stanceFR", 1, 1)
        self.stanceRL = cs.SX.sym("stanceRL", 1, 1)
        self.stanceRR = cs.SX.sym("stanceRR", 1, 1)
        self.stance_param = cs.vertcat(self.stanceFL, self.stanceFR, self.stanceRL, self.stanceRR)

        self.mu_friction = cs.SX.sym("mu_friction", 1, 1)
        self.stance_proximity = cs.SX.sym("stanceProximity", 4, 1)
        self.base_position = cs.SX.sym("base_position", 3, 1)
        self.base_yaw = cs.SX.sym("base_yaw", 1, 1)

        self.external_wrench = cs.SX.sym("external_wrench", 6, 1)

        self.inertia = cs.SX.sym("inertia", 9, 1)
        self.mass = cs.SX.sym("mass", 1, 1)

        # Not so useful, i can instantiate a casadi function for the fd
        param = cs.vertcat(
            self.stance_param,
            self.mu_friction,
            self.stance_proximity,
            self.base_position,
            self.base_yaw,
            self.external_wrench,
            self.inertia,
            self.mass,
        )
        fd = self.forward_dynamics(self.states, self.inputs, param)
        self.fun_forward_dynamics = cs.Function("fun_forward_dynamics", [self.states, self.inputs, param], [fd])

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
        foot_velocity_fl = inputs[0:3]
        foot_velocity_fr = inputs[3:6]
        foot_velocity_rl = inputs[6:9]
        foot_velocity_rr = inputs[9:12]
        foot_force_fl = inputs[12:15]
        foot_force_fr = inputs[15:18]
        foot_force_rl = inputs[18:21]
        foot_force_rr = inputs[21:24]

        com_position = states[0:3]
        foot_position_fl = states[12:15]
        foot_position_fr = states[15:18]
        foot_position_rl = states[18:21]
        foot_position_rr = states[21:24]

        stanceFL = param[0]
        stanceFR = param[1]
        stanceRL = param[2]
        stanceRR = param[3]
        stance_proximity_FL = param[5]
        stance_proximity_FR = param[6]
        stance_proximity_RL = param[7]
        stance_proximity_RR = param[8]

        external_wrench_linear = param[13:16]
        external_wrench_angular = param[16:19]

        inertia = param[19:28]
        inertia = inertia.reshape((3, 3))
        mass = param[28]

        gravity = np.array([0, 0, -9.81])

        # Lyapunov states (ETA will be computed at the end of this function)
        K_z1 = config.mpc_params["K_z1"]
        K_z2 = config.mpc_params["K_z2"]
        p_ddot = np.array([0, 0, 0])

        z1 = states[30:33]
        z2 = states[33:36]
        phi = states[42:45]

        phi_dot = -z2

        # Calculate the adaptive law
        squared_K_z1 = np.array([K_z1[0] * K_z1[0], K_z1[1] * K_z1[1], K_z1[2] * K_z1[2]])
        F_star = mass * (-(K_z1 + K_z2) * z2 + squared_K_z1 * z1 - gravity + p_ddot)
        F_star -= phi

        z1_dot = -K_z1 * z1 + z2

        F_delta = foot_force_fl @ stanceFL
        F_delta += foot_force_fr @ stanceFR
        F_delta += foot_force_rl @ stanceRL
        F_delta += foot_force_rr @ stanceRR
        z2_dot = -K_z2 * z2 + 1 / mass @ (F_delta + F_star) + gravity - p_ddot + phi

        # Redistribute the lyapunov forces to the leg
        num_leg_contact = stanceFL + stanceFR + stanceRL + stanceRR
        foot_force_fl_final = foot_force_fl @ stanceFL + (F_star / num_leg_contact) @ stanceFL
        foot_force_fr_final = foot_force_fr @ stanceFR + (F_star / num_leg_contact) @ stanceFR
        foot_force_rl_final = foot_force_rl @ stanceRL + (F_star / num_leg_contact) @ stanceRL
        foot_force_rr_final = foot_force_rr @ stanceRR + (F_star / num_leg_contact) @ stanceRR

        # linear wrench exterted by the robot (considering mpc + adaptive law)
        temp = foot_force_fl_final @ stanceFL
        temp += foot_force_fr_final @ stanceFR
        temp += foot_force_rl_final @ stanceRL
        temp += foot_force_rr_final @ stanceRR
        temp += external_wrench_linear

        # angular wrench exerted by the robot (considering mpc + adaptive law)
        temp2 = cs.skew(foot_position_fl - com_position) @ foot_force_fl_final @ stanceFL
        temp2 += cs.skew(foot_position_fr - com_position) @ foot_force_fr_final @ stanceFR
        temp2 += cs.skew(foot_position_rl - com_position) @ foot_force_rl_final @ stanceRL
        temp2 += cs.skew(foot_position_rr - com_position) @ foot_force_rr_final @ stanceRR
        temp2 = temp2 + external_wrench_angular

        # FINAL linear_com_vel STATE (1)
        linear_com_vel = states[3:6]

        # FINAL linear_com_acc STATE (2)
        linear_com_acc = (1 / mass) @ temp + gravity - phi

        # Start to write the component of euler_rates_base and angular_acc_base STATES
        w = states[9:12]
        roll = states[6]
        pitch = states[7]
        yaw = states[8]

        conj_euler_rates = cs.SX.eye(3)
        conj_euler_rates[1, 1] = cs.cos(roll)
        conj_euler_rates[2, 2] = cs.cos(pitch) * cs.cos(roll)
        conj_euler_rates[2, 1] = -cs.sin(roll)
        conj_euler_rates[0, 2] = -cs.sin(pitch)
        conj_euler_rates[1, 2] = cs.cos(pitch) * cs.sin(roll)

        # FINAL euler_rates_base STATE (3)
        euler_rates_base = cs.inv(conj_euler_rates) @ w

        # FINAL angular_acc_base STATE (4)
        # Z Y X rotations!
        Rx = cs.SX.eye(3)
        Rx[0, 0] = 1
        Rx[0, 1] = 0
        Rx[0, 2] = 0
        Rx[1, 0] = 0
        Rx[1, 1] = cs.cos(roll)
        Rx[1, 2] = cs.sin(roll)
        Rx[2, 0] = 0
        Rx[2, 1] = -cs.sin(roll)
        Rx[2, 2] = cs.cos(roll)

        Ry = cs.SX.eye(3)
        Ry[0, 0] = cs.cos(pitch)
        Ry[0, 1] = 0
        Ry[0, 2] = -cs.sin(pitch)
        Ry[1, 0] = 0
        Ry[1, 1] = 1
        Ry[1, 2] = 0
        Ry[2, 0] = cs.sin(pitch)
        Ry[2, 1] = 0
        Ry[2, 2] = cs.cos(pitch)

        Rz = cs.SX.eye(3)
        Rz[0, 0] = cs.cos(yaw)
        Rz[0, 1] = cs.sin(yaw)
        Rz[0, 2] = 0
        Rz[1, 0] = -cs.sin(yaw)
        Rz[1, 1] = cs.cos(yaw)
        Rz[1, 2] = 0
        Rz[2, 0] = 0
        Rz[2, 1] = 0
        Rz[2, 2] = 1

        b_R_w = Rx @ Ry @ Rz

        pos_phi = np.array([0, 0, 0.2])
        pos_phi_base = b_R_w.T @ pos_phi
        temp2 -= cs.skew(pos_phi_base) @ phi
        angular_acc_base = cs.inv(inertia) @ (b_R_w @ temp2 - cs.skew(w) @ inertia @ w)

        # FINAL linear_foot_vel STATES (5,6,7,8)
        if not config.mpc_params["use_foothold_optimization"]:
            foot_velocity_fl = foot_velocity_fl @ 0.0
            foot_velocity_fr = foot_velocity_fr @ 0.0
            foot_velocity_rl = foot_velocity_rl @ 0.0
            foot_velocity_rr = foot_velocity_rr @ 0.0
        linear_foot_vel_FL = foot_velocity_fl @ (1 - stanceFL) @ (1 - stance_proximity_FL)
        linear_foot_vel_FR = foot_velocity_fr @ (1 - stanceFR) @ (1 - stance_proximity_FR)
        linear_foot_vel_RL = foot_velocity_rl @ (1 - stanceRL) @ (1 - stance_proximity_RL)
        linear_foot_vel_RR = foot_velocity_rr @ (1 - stanceRR) @ (1 - stance_proximity_RR)

        # Integral states
        integral_states = states[24:30]
        integral_states[0] += states[2]
        integral_states[1] += states[3]
        integral_states[2] += states[4]
        integral_states[3] += states[5]
        integral_states[4] += roll
        integral_states[5] += pitch

        # STILL LYAPUNOV FOR ETA
        eta_dot_temp = cs.inv(inertia) @ (b_R_w @ temp2 - cs.skew(w) @ inertia @ w)
        eta_dot = cs.vertcat(w, eta_dot_temp)

        # The order of the return should be equal to the order of the states_dot
        return cs.vertcat(
            linear_com_vel,
            linear_com_acc,
            euler_rates_base,
            angular_acc_base,
            linear_foot_vel_FL,
            linear_foot_vel_FR,
            linear_foot_vel_RL,
            linear_foot_vel_RR,
            integral_states,
            z1_dot,
            z2_dot,
            eta_dot,
            phi_dot,
        )

    def export_robot_model(self) -> AcadosModel:
        """
        This method set some general properties of the NMPC, such as the params,
        prediction mode, etc...! It will be called in centroidal_nmpc.py
        """

        # dynamics
        self.param = cs.vertcat(
            self.stance_param,
            self.mu_friction,
            self.stance_proximity,
            self.base_position,
            self.base_yaw,
            self.external_wrench,
            self.inertia,
            self.mass,
        )
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
