# Description: This file contains the class for the NMPC controller

# Authors: Giulio Turrisi -

import copy
import os
import time

import casadi as cs
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpBatchSolver

ACADOS_INFTY = 1000

from quadruped_pympc import config

from .centroidal_model_nominal import Centroidal_Model_Nominal


# Class for the Acados NMPC, the model is in another file!
class Acados_NMPC_GaitAdaptive:
    def __init__(self):
        self.horizon = config.mpc_params['horizon']  # Define the number of discretization steps
        self.dt = config.mpc_params['dt']
        self.T_horizon = self.horizon * self.dt
        self.use_RTI = config.mpc_params["use_RTI"]
        self.use_integrators = config.mpc_params["use_integrators"]
        self.use_warm_start = config.mpc_params["use_warm_start"]
        self.use_foothold_constraints = config.mpc_params["use_foothold_constraints"]

        self.use_static_stability = config.mpc_params["use_static_stability"]
        self.use_zmp_stability = config.mpc_params["use_zmp_stability"]
        self.use_stability_constraints = self.use_static_stability or self.use_zmp_stability

        self.use_DDP = config.mpc_params["use_DDP"]

        self.previous_status = -1
        self.previous_contact_sequence = np.zeros((4, self.horizon))
        self.optimal_next_state = np.zeros((24,))
        self.previous_optimal_GRF = np.zeros((12,))

        self.integral_errors = np.zeros((6,))

        # For centering the variable around 0, 0, 0 (World frame)
        self.initial_base_position = np.array([0, 0, 0])

        # Create the class of the centroidal model and instantiate the acados model
        self.centroidal_model = Centroidal_Model_Nominal()
        acados_model = self.centroidal_model.export_robot_model()
        self.states_dim = acados_model.x.size()[0]
        self.inputs_dim = acados_model.u.size()[0]

        # Create the acados ocp solver
        self.ocp = self.create_ocp_solver_description(
            acados_model, num_threads_in_batch_solve=len(config.mpc_params['step_freq_available'])
        )

        # Batch solver
        use_batch_solver = config.mpc_params["optimize_step_freq"]
        num_batch = len(config.mpc_params["step_freq_available"])
        self.batch = num_batch
        # batch_ocp = self.create_ocp_solver_description(acados_model, num_threads_in_batch_solve)
        batch_ocp = self.ocp
        dir_path = os.path.dirname(os.path.realpath(__file__))
        batch_ocp.code_export_directory = dir_path + '/c_generated_code_gait_adaptive'
        self.batch_solver = AcadosOcpBatchSolver(
            batch_ocp,
            self.batch,
            verbose=False,
            json_file=self.ocp.code_export_directory + "/centroidal_nmpc_batch" + ".json",
        )

        # Initialize solvers
        for stage in range(self.horizon + 1):
            for n in range(self.batch):
                self.batch_solver.ocp_solvers[n].set(stage, "x", np.zeros((self.states_dim,)))
        for stage in range(self.horizon):
            for n in range(self.batch):
                self.batch_solver.ocp_solvers[n].set(stage, "u", np.zeros((self.inputs_dim,)))

                # Set cost, constraints and options

    def create_ocp_solver_description(self, acados_model, num_threads_in_batch_solve=1) -> AcadosOcp:
        # Create ocp object to formulate the OCP
        ocp = AcadosOcp()
        ocp.model = acados_model
        nx = self.states_dim
        nu = self.inputs_dim
        ny = nx + nu

        # Set dimensions
        ocp.dims.N = self.horizon

        # Set cost
        Q_mat, R_mat = self.set_weight(nx, nu)
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        ny = nx + nu
        ny_e = nx

        ocp.cost.W_e = Q_mat
        ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)

        Vu = np.zeros((ny, nu))
        Vu[nx : nx + nu, 0:nu] = np.eye(nu)
        ocp.cost.Vu = Vu

        ocp.cost.Vx_e = np.eye(nx)

        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((ny_e,))

        # Set friction and foothold constraints
        expr_h_friction, self.constr_uh_friction, self.constr_lh_friction = self.create_friction_cone_constraints()

        ocp.model.con_h_expr = expr_h_friction
        ocp.constraints.uh = self.constr_uh_friction
        ocp.constraints.lh = self.constr_lh_friction
        ocp.model.con_h_expr_0 = expr_h_friction
        ocp.constraints.uh_0 = self.constr_uh_friction
        ocp.constraints.lh_0 = self.constr_lh_friction
        nsh = expr_h_friction.shape[0]
        nsh_state_constraint_start = copy.copy(nsh)

        if self.use_foothold_constraints:
            expr_h_foot, self.constr_uh_foot, self.constr_lh_foot = self.create_foothold_constraints()

            ocp.model.con_h_expr = cs.vertcat(ocp.model.con_h_expr, expr_h_foot)
            ocp.constraints.uh = np.concatenate((ocp.constraints.uh, self.constr_uh_foot))
            ocp.constraints.lh = np.concatenate((ocp.constraints.lh, self.constr_lh_foot))
            nsh += expr_h_foot.shape[0]

        # Set stability constraints
        if self.use_stability_constraints:
            self.nsh_stability_start = copy.copy(nsh)
            expr_h_support_polygon, self.constr_uh_support_polygon, self.constr_lh_support_polygon = (
                self.create_stability_constraints()
            )

            ocp.model.con_h_expr = cs.vertcat(ocp.model.con_h_expr, expr_h_support_polygon)
            ocp.constraints.uh = np.concatenate((ocp.constraints.uh, self.constr_uh_support_polygon))
            ocp.constraints.lh = np.concatenate((ocp.constraints.lh, self.constr_lh_support_polygon))
            nsh += expr_h_support_polygon.shape[0]
            self.nsh_stability_end = copy.copy(nsh)

        nsh_state_constraint_end = copy.copy(nsh)

        # Set slack variable configuration:
        num_state_cstr = nsh_state_constraint_end - nsh_state_constraint_start
        if num_state_cstr > 0:
            ocp.constraints.lsh = np.zeros(
                num_state_cstr
            )  # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            ocp.constraints.ush = np.zeros(
                num_state_cstr
            )  # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            ocp.constraints.idxsh = np.array(range(nsh_state_constraint_start, nsh_state_constraint_end))  # Jsh
            ns = num_state_cstr
            ocp.cost.zl = 1000 * np.ones((ns,))  # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
            ocp.cost.Zl = 1 * np.ones(
                (ns,)
            )  # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
            ocp.cost.zu = 1000 * np.ones((ns,))
            ocp.cost.Zu = 1 * np.ones((ns,))

            # Variables to save the upper and lower bound of the constraints applied
        list_upper_bound = []
        list_lower_bound = []
        for j in range(self.horizon):
            list_upper_bound.append(np.zeros((nsh,)))
            list_lower_bound.append(np.zeros((nsh,)))
        self.upper_bound = np.array(list_upper_bound, dtype=object)
        self.lower_bound = np.array(list_lower_bound, dtype=object)

        # Set initial state constraint
        X0 = np.zeros(shape=(nx,))
        ocp.constraints.x0 = X0

        # Set initialize parameters
        init_contact_status = np.array([1.0, 1.0, 1.0, 1.0])
        init_mu = np.array([0.5])
        init_stance_proximity = np.array([0, 0, 0, 0])
        init_base_position = np.array([0, 0, 0])
        init_base_yaw = np.array([0])
        init_external_wrench = np.array([0, 0, 0, 0, 0, 0])
        init_inertia = config.inertia.reshape((9,))
        init_mass = np.array([config.mass])

        ocp.parameter_values = np.concatenate(
            (
                init_contact_status,
                init_mu,
                init_stance_proximity,
                init_base_position,
                init_base_yaw,
                init_external_wrench,
                init_inertia,
                init_mass,
            )
        )

        # Set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES PARTIAL_CONDENSING_OSQP
        # PARTIAL_CONDENSING_HPIPM
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "ERK"  # ERK IRK GNSF DISCRETE
        if self.use_DDP:
            ocp.solver_options.nlp_solver_type = 'DDP'
            ocp.solver_options.nlp_solver_max_iter = config.mpc_params['num_qp_iterations']
            # ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
            ocp.solver_options.with_adaptive_levenberg_marquardt = True

            ocp.cost.cost_type = "NONLINEAR_LS"
            ocp.cost.cost_type_e = "NONLINEAR_LS"
            ocp.model.cost_y_expr = cs.vertcat(ocp.model.x, ocp.model.u)
            ocp.model.cost_y_expr_e = ocp.model.x

            ocp.translate_to_feasibility_problem(keep_x0=True, keep_cost=True)

        elif self.use_RTI:
            ocp.solver_options.nlp_solver_type = "SQP_RTI"
            ocp.solver_options.nlp_solver_max_iter = 1
            # Set the RTI type for the advanced RTI method
            # (see https://arxiv.org/pdf/2403.07101.pdf)
            if config.mpc_params['as_rti_type'] == "AS-RTI-A":
                ocp.solver_options.as_rti_iter = 1
                ocp.solver_options.as_rti_level = 0
            elif config.mpc_params['as_rti_type'] == "AS-RTI-B":
                ocp.solver_options.as_rti_iter = 1
                ocp.solver_options.as_rti_level = 1
            elif config.mpc_params['as_rti_type'] == "AS-RTI-C":
                ocp.solver_options.as_rti_iter = 1
                ocp.solver_options.as_rti_level = 2
            elif config.mpc_params['as_rti_type'] == "AS-RTI-D":
                ocp.solver_options.as_rti_iter = 1
                ocp.solver_options.as_rti_level = 3

        else:
            ocp.solver_options.nlp_solver_type = "SQP"
            ocp.solver_options.nlp_solver_max_iter = config.mpc_params["num_qp_iterations"]
        # ocp.solver_options.globalization = "MERIT_BACKTRACKING"  # FIXED_STEP, MERIT_BACKTRACKING

        if config.mpc_params['solver_mode'] == "balance":
            ocp.solver_options.hpipm_mode = "BALANCE"
        elif config.mpc_params['solver_mode'] == "robust":
            ocp.solver_options.hpipm_mode = "ROBUST"
        elif config.mpc_params['solver_mode'] == "fast":
            ocp.solver_options.qp_solver_iter_max = 10
            ocp.solver_options.hpipm_mode = "SPEED"
        elif config.mpc_params['solver_mode'] == "crazy_speed":
            ocp.solver_options.qp_solver_iter_max = 5
            ocp.solver_options.hpipm_mode = "SPEED_ABS"

        # ocp.solver_options.line_search_use_sufficient_descent = 1
        # ocp.solver_options.regularize_method = "PROJECT_REDUC_HESS"
        # ocp.solver_options.nlp_solver_ext_qp_res = 1
        ocp.solver_options.levenberg_marquardt = 1e-3
        ocp.solver_options.num_threads_in_batch_solve = num_threads_in_batch_solve

        # Set prediction horizon
        ocp.solver_options.tf = self.T_horizon

        # Nonuniform discretization
        if config.mpc_params['use_nonuniform_discretization']:
            time_steps_fine_grained = np.tile(
                config.mpc_params['dt_fine_grained'], config.mpc_params['horizon_fine_grained']
            )
            time_steps = np.concatenate(
                (time_steps_fine_grained, np.tile(self.dt, self.horizon - config.mpc_params['horizon_fine_grained']))
            )
            shooting_nodes = np.zeros((self.horizon + 1,))
            for i in range(len(time_steps)):
                shooting_nodes[i + 1] = shooting_nodes[i] + time_steps[i]
            ocp.solver_options.shooting_nodes = shooting_nodes

        return ocp

    # Create a constraint for  stability (COM, ZMP or CP inside support polygon)
    def create_stability_constraints(self) -> None:
        base_w = self.centroidal_model.states[0:3]
        base_vel_w = self.centroidal_model.states[3:6]

        FL = self.centroidal_model.states[12:15]
        FR = self.centroidal_model.states[15:18]
        RL = self.centroidal_model.states[18:21]
        RR = self.centroidal_model.states[21:24]

        # yaw = self.centroidal_model.base_yaw[0]
        yaw = self.centroidal_model.states[8]
        h_R_w = cs.SX.zeros(2, 2)
        h_R_w[0, 0] = cs.cos(yaw)
        h_R_w[0, 1] = cs.sin(yaw)
        h_R_w[1, 0] = -cs.sin(yaw)
        h_R_w[1, 1] = cs.cos(yaw)

        FL[0:2] = h_R_w @ (FL[0:2] - base_w[0:2])
        FR[0:2] = h_R_w @ (FR[0:2] - base_w[0:2])
        RL[0:2] = h_R_w @ (RL[0:2] - base_w[0:2])
        RR[0:2] = h_R_w @ (RR[0:2] - base_w[0:2])

        if self.use_static_stability:
            x = 0.0
            y = 0.0
        else:
            # Compute the ZMP
            robotHeight = base_w[2]
            foot_force_fl = self.centroidal_model.inputs[12:15]  # @self.centroidal_model.param[0]
            foot_force_fr = self.centroidal_model.inputs[15:18]  # @self.centroidal_model.param[1]
            foot_force_rl = self.centroidal_model.inputs[18:21]  # @self.centroidal_model.param[2]
            foot_force_rr = self.centroidal_model.inputs[21:24]  # @self.centroidal_model.param[3]
            temp = foot_force_fl + foot_force_fr + foot_force_rl + foot_force_rr
            gravity = np.array([0, 0, -9.81])
            linear_com_acc = (1 / self.centroidal_model.mass) @ temp + gravity
            zmp = base_w[0:2] - linear_com_acc[0:2] * (robotHeight / (-gravity[2]))
            # zmp = base_w[0:2] - base_vel_w[0:2]*(robotHeight/gravity[2])
            zmp = h_R_w @ (zmp - base_w[0:2])
            x = zmp[0]
            y = zmp[1]

        y_FL = FL[1]
        y_FR = FR[1]
        y_RL = RL[1]
        y_RR = RR[1]

        x_FL = FL[0]
        x_FR = FR[0]
        x_RL = RL[0]
        x_RR = RR[0]

        # LF - RF : x < (x2 - x1) (y - y1) / (y2 - y1) + x1
        # RF - RH: y > (y2 - y1) (x - x1) / (x2 - x1) + y1
        # RH - LH : x > (x2 - x1) (y - y1) / (y2 - y1) + x1
        # LH - LF: y < (y2 - y1) (x - x1) / (x2 - x1) + y1

        # FL and FR cannot stay at the same x! #constrint should be less than zero
        constraint_FL_FR = x - (x_FR - x_FL) * (y - y_FL) / (y_FR - y_FL + 0.001) - x_FL

        # FR and RR cannot stay at the same y! #constraint should be bigger than zero
        constraint_FR_RR = y - (y_RR - y_FR) * (x - x_FR) / (x_RR - x_FR + 0.001) - y_FR

        # RL and RR cannot stay at the same x! #constraint should be bigger than zero
        constraint_RR_RL = x - (x_RL - x_RR) * (y - y_RR) / (y_RL - y_RR + 0.001) - x_RR

        # FL and RL cannot stay at the same y! #constraint should be less than zero
        constraint_RL_FL = y - (y_FL - y_RL) * (x - x_RL) / (x_FL - x_RL + 0.001) - y_RL

        # the diagonal stuff can be at any point...
        constraint_FL_RR = y - (y_RR - y_FL) * (x - x_FL) / (x_RR - x_FL + 0.001) - y_FL  # bigger than zero
        constraint_FR_RL = y - (y_RL - y_FR) * (x - x_FR) / (x_RL - x_FR + 0.001) - y_FR  # bigger than zero

        # upper and lower bound
        ub = np.ones(6) * ACADOS_INFTY
        lb = -np.ones(6) * ACADOS_INFTY

        # constraint_FL_FR
        ub[0] = 0
        lb[0] = -ACADOS_INFTY

        # constraint_FR_RR
        ub[1] = ACADOS_INFTY
        lb[1] = 0

        # constraint_RR_RL
        ub[2] = ACADOS_INFTY
        lb[2] = 0

        # constraint_RL_FL
        ub[3] = 0
        lb[3] = -ACADOS_INFTY

        # constraint_FL_RR
        ub[4] = 0
        lb[4] = -ACADOS_INFTY

        # constraint_FR_RL
        ub[5] = 0
        lb[5] = -ACADOS_INFTY

        Jb = cs.vertcat(
            constraint_FL_FR, constraint_FR_RR, constraint_RR_RL, constraint_RL_FL, constraint_FL_RR, constraint_FR_RL
        )

        return Jb, ub, lb

    # Create a standard foothold box constraint
    def create_foothold_constraints(self):
        """
        This function calculates the symbolic foothold constraints for the centroidal NMPC problem.

        Returns:
            Jbu: A CasADi SX matrix of shape (8, 1) representing the foothold constraints.
            ubu: A numpy array of shape (8,) representing the upper bounds of the foothold constraints.
            lbu: A numpy array of shape (8,) representing the lower bounds of the foothold constraints.
        """

        ubu = np.ones(12) * ACADOS_INFTY
        lbu = -np.ones(12) * ACADOS_INFTY

        # The foothold constraint in acados are in the horizontal frame,
        # but they arrive to us in the world frame. We need to rotate them
        # using the robot yaw
        yaw = self.centroidal_model.base_yaw[0]
        h_R_w = cs.SX.zeros(2, 2)
        h_R_w[0, 0] = cs.cos(yaw)
        h_R_w[0, 1] = cs.sin(yaw)
        h_R_w[1, 0] = -cs.sin(yaw)
        h_R_w[1, 1] = cs.cos(yaw)

        # and translate them using the robot base position
        base = self.centroidal_model.base_position[0:3]

        foot_position_fl = cs.SX.zeros(3, 1)
        foot_position_fl[0:2] = h_R_w @ cs.vertcat(self.centroidal_model.states[12:14] - base[0:2])
        foot_position_fl[2] = self.centroidal_model.states[14]

        foot_position_fr = cs.SX.zeros(3, 1)
        foot_position_fr[0:2] = h_R_w @ cs.vertcat(self.centroidal_model.states[15:17] - base[0:2])
        foot_position_fr[2] = self.centroidal_model.states[17]

        foot_position_rl = cs.SX.zeros(3, 1)
        foot_position_rl[0:2] = h_R_w @ cs.vertcat(self.centroidal_model.states[18:20] - base[0:2])
        foot_position_rl[2] = self.centroidal_model.states[20]

        foot_position_rr = cs.SX.zeros(3, 1)
        foot_position_rr[0:2] = h_R_w @ cs.vertcat(self.centroidal_model.states[21:23] - base[0:2])
        foot_position_rr[2] = self.centroidal_model.states[23]

        Jbu = cs.vertcat(foot_position_fl, foot_position_fr, foot_position_rl, foot_position_rr)
        return Jbu, ubu, lbu

    # Create the friction cone constraint
    def create_friction_cone_constraints(self) -> None:
        """
        This function calculates the symbolic friction cone constraints for the centroidal NMPC problem.

        Returns:
            Jbu: A CasADi SX matrix of shape (20, 12) representing the friction cone constraints.
            ubu: A numpy array of shape (20,) representing the upper bounds of the friction cone constraints.
            lbu: A numpy array of shape (20,) representing the lower bounds of the friction cone constraints.
        """
        n = np.array([0, 0, 1])
        t = np.array([1, 0, 0])
        b = np.array([0, 1, 0])
        mu = self.centroidal_model.mu_friction
        f_max = config.mpc_params["grf_max"]
        f_min = config.mpc_params["grf_min"]

        # Derivation can be found in the paper
        # "High-slope terrain locomotion for torque-controlled quadruped robots",
        # M Focchi, A Del Prete, I Havoutis, R Featherstone, DG Caldwell, C Semini
        Jbu = cs.SX.zeros(20, 12)
        Jbu[0, :3] = -n * mu + t
        Jbu[1, :3] = -n * mu + b
        Jbu[2, :3] = n * mu + b
        Jbu[3, :3] = n * mu + t
        Jbu[4, :3] = n

        Jbu[5, 3:6] = -n * mu + t
        Jbu[6, 3:6] = -n * mu + b
        Jbu[7, 3:6] = n * mu + b
        Jbu[8, 3:6] = n * mu + t
        Jbu[9, 3:6] = n

        Jbu[10, 6:9] = -n * mu + t
        Jbu[11, 6:9] = -n * mu + b
        Jbu[12, 6:9] = n * mu + b
        Jbu[13, 6:9] = n * mu + t
        Jbu[14, 6:9] = n

        Jbu[15, 9:12] = -n * mu + t
        Jbu[16, 9:12] = -n * mu + b
        Jbu[17, 9:12] = n * mu + b
        Jbu[18, 9:12] = n * mu + t
        Jbu[19, 9:12] = n

        # C matrix
        Jbu = Jbu @ cs.vertcat(self.centroidal_model.inputs[12:24])

        # lower bound (-ACADOS_INFTY is "almost" -inf)
        lbu = np.zeros(20)
        lbu[0] = -ACADOS_INFTY
        lbu[1] = -ACADOS_INFTY
        lbu[2] = 0
        lbu[3] = 0
        lbu[4] = f_min
        lbu[5:10] = lbu[0:5]
        lbu[10:15] = lbu[0:5]
        lbu[15:20] = lbu[0:5]

        # upper bound (ACADOS_INFTY is "almost" inf)
        ubu = np.zeros(20)
        ubu[0] = 0
        ubu[1] = 0
        ubu[2] = ACADOS_INFTY
        ubu[3] = ACADOS_INFTY
        ubu[4] = f_max
        ubu[5:10] = ubu[0:5]
        ubu[10:15] = ubu[0:5]
        ubu[15:20] = ubu[0:5]

        return Jbu, ubu, lbu

    def set_weight(self, nx, nu):
        # Define the weight matrices for the cost function

        Q_position = np.array([0, 0, 1500])  # x, y, z
        Q_velocity = np.array([200, 200, 200])  # x_vel, y_vel, z_vel
        Q_base_angle = np.array([500, 500, 0])  # roll, pitch, yaw
        Q_base_angle_rates = np.array([20, 20, 50])  # roll_rate, pitch_rate, yaw_rate
        Q_foot_pos = np.array([300, 300, 300])  # f_x, f_y, f_z (should be 4 times this, once per foot)
        Q_com_position_z_integral = np.array([50])  # integral of z_com
        Q_com_velocity_x_integral = np.array([10])  # integral of x_com
        Q_com_velocity_y_integral = np.array([10])  # integral of y_com
        Q_com_velocity_z_integral = np.array([10])  # integral of z_com_vel
        Q_roll_integral_integral = np.array([10])  # integral of roll
        Q_pitch_integral_integral = np.array([10])  # integral of pitch

        R_foot_vel = np.array([0.0001, 0.0001, 0.00001])  # v_x, v_y, v_z (should be 4 times this, once per foot)
        if config.robot == "hyqreal":
            R_foot_force = np.array(
                [0.00001, 0.00001, 0.00001]
            )  # f_x, f_y, f_z (should be 4 times this, once per foot)
        else:
            R_foot_force = np.array([0.001, 0.001, 0.001])

        Q_mat = np.diag(
            np.concatenate(
                (
                    Q_position,
                    Q_velocity,
                    Q_base_angle,
                    Q_base_angle_rates,
                    Q_foot_pos,
                    Q_foot_pos,
                    Q_foot_pos,
                    Q_foot_pos,
                    Q_com_position_z_integral,
                    Q_com_velocity_x_integral,
                    Q_com_velocity_y_integral,
                    Q_com_velocity_z_integral,
                    Q_roll_integral_integral,
                    Q_pitch_integral_integral,
                )
            )
        )

        R_mat = np.diag(
            np.concatenate(
                (R_foot_vel, R_foot_vel, R_foot_vel, R_foot_vel, R_foot_force, R_foot_force, R_foot_force, R_foot_force)
            )
        )

        return Q_mat, R_mat

    def reset(self):
        self.acados_ocp_solver.reset()

    def set_stage_constraint(self, constraint, state, reference, contact_sequence, h_R_w, stance_proximity):
        """
        Set the stage constraint for the centroidal NMPC problem. We only consider the stance constraint, and the swing
        constraint up to 2 maximum references.

        Args:
            constraint (numpy.ndarray or None): Constraint passed from outside (e.g. vision). If None,
            nominal foothold is used.
            state (dict): Current state of the system.
            reference (dict): Reference state of the system.
            contact_sequence (numpy.ndarray): Array representing the contact sequence.
            h_R_w (numpy.ndarray): Rotation matrix from the horizontal frame to the world frame.
            stance_proximity (float): Proximity parameter for the stance constraint.

        Returns:
            None
        """
        try:
            # Take the array of the contact sequence and split
            # it in 4 arrays for clarity
            FL_contact_sequence = contact_sequence[0]
            FR_contact_sequence = contact_sequence[1]
            RL_contact_sequence = contact_sequence[2]
            RR_contact_sequence = contact_sequence[3]

            # Take the actual and reference foothold
            FL_actual_foot = state["foot_FL"]
            FR_actual_foot = state["foot_FR"]
            RL_actual_foot = state["foot_RL"]
            RR_actual_foot = state["foot_RR"]

            FL_reference_foot = reference["ref_foot_FL"]
            FR_reference_foot = reference["ref_foot_FR"]
            RL_reference_foot = reference["ref_foot_RL"]
            RR_reference_foot = reference["ref_foot_RR"]

            # Take the base position and the yaw rotation matrix. This is needed to
            # express the foothold constraint in the horizontal frame
            base = state["position"]

            h_R_w = h_R_w.reshape((2, 2))

            # Divide the constraint in upper and lower bound. The constraint are
            # represented by 4 vertex, but we only use box constraint hence
            # we need only 2 vertex for each constraint (first an last)

            # For the leg in stance now, we simply enlarge the actual position as a box constraint
            # maybe this is not needed, but we cannot disable constraint!!

            # FL Stance Constraint
            stance_up_constraint_FL = np.array([FL_actual_foot[0], FL_actual_foot[1], FL_actual_foot[2] + 0.002])
            stance_up_constraint_FL[0:2] = h_R_w @ (stance_up_constraint_FL[0:2] - base[0:2])
            stance_up_constraint_FL[0:2] = stance_up_constraint_FL[0:2] + 0.1
            stance_up_constraint_FL[2] = stance_up_constraint_FL[2] + 0.01

            stance_low_constraint_FL = np.array([FL_actual_foot[0], FL_actual_foot[1], FL_actual_foot[2] - 0.002])
            stance_low_constraint_FL[0:2] = h_R_w @ (stance_low_constraint_FL[0:2] - base[0:2])
            stance_low_constraint_FL[0:2] = stance_low_constraint_FL[0:2] - 0.1
            stance_low_constraint_FL[2] = stance_low_constraint_FL[2] - 0.01

            # FR Stance Constraint
            stance_up_constraint_FR = np.array([FR_actual_foot[0], FR_actual_foot[1], FR_actual_foot[2] + 0.002])
            stance_up_constraint_FR[0:2] = h_R_w @ (stance_up_constraint_FR[0:2] - base[0:2])
            stance_up_constraint_FR[0:2] = stance_up_constraint_FR[0:2] + 0.1
            stance_up_constraint_FR[2] = stance_up_constraint_FR[2] + 0.01

            stance_low_constraint_FR = np.array([FR_actual_foot[0], FR_actual_foot[1], FR_actual_foot[2] - 0.002])
            stance_low_constraint_FR[0:2] = h_R_w @ (stance_low_constraint_FR[0:2] - base[0:2])
            stance_low_constraint_FR[0:2] = stance_low_constraint_FR[0:2] - 0.1
            stance_low_constraint_FR[2] = stance_low_constraint_FR[2] - 0.01

            # RL Stance Constraint
            stance_up_constraint_RL = np.array([RL_actual_foot[0], RL_actual_foot[1], RL_actual_foot[2] + 0.002])
            stance_up_constraint_RL[0:2] = h_R_w @ (stance_up_constraint_RL[0:2] - base[0:2])
            stance_up_constraint_RL[0:2] = stance_up_constraint_RL[0:2] + 0.1
            stance_up_constraint_RL[2] = stance_up_constraint_RL[2] + 0.01

            stance_low_constraint_RL = np.array([RL_actual_foot[0], RL_actual_foot[1], RL_actual_foot[2] - 0.002])
            stance_low_constraint_RL[0:2] = h_R_w @ (stance_low_constraint_RL[0:2] - base[0:2])
            stance_low_constraint_RL[0:2] = stance_low_constraint_RL[0:2] - 0.1
            stance_low_constraint_RL[2] = stance_low_constraint_RL[2] - 0.01

            # RR Stance Constraint
            stance_up_constraint_RR = np.array([RR_actual_foot[0], RR_actual_foot[1], RR_actual_foot[2] + 0.002])
            stance_up_constraint_RR[0:2] = h_R_w @ (stance_up_constraint_RR[0:2] - base[0:2])
            stance_up_constraint_RR[0:2] = stance_up_constraint_RR[0:2] + 0.1
            stance_up_constraint_RR[2] = stance_up_constraint_RR[2] + 0.01

            stance_low_constraint_RR = np.array([RR_actual_foot[0], RR_actual_foot[1], RR_actual_foot[2] - 0.002])
            stance_low_constraint_RR[0:2] = h_R_w @ (stance_low_constraint_RR[0:2] - base[0:2])
            stance_low_constraint_RR[0:2] = stance_low_constraint_RR[0:2] - 0.1
            stance_low_constraint_RR[2] = stance_low_constraint_RR[2] - 0.01

            # Constraint for the first footholds at the next touchdown. If constraint == True
            # we have some constraint passed from outside (e.g. vision), otherwise we use the
            # nominal foothold enlarged as we do previously
            if constraint is not None:
                # From the VFA
                first_up_constraint_FL = np.array([constraint[0][0], constraint[1][0], constraint[2][0] + 0.002])
                first_up_constraint_FR = np.array([constraint[0][1], constraint[1][1], constraint[2][1] + 0.002])
                first_up_constraint_RL = np.array([constraint[0][2], constraint[1][2], constraint[2][2] + 0.002])
                first_up_constraint_RR = np.array([constraint[0][3], constraint[1][3], constraint[2][3] + 0.002])

                first_low_constraint_FL = np.array([constraint[9][0], constraint[10][0], constraint[11][0] - 0.002])
                first_low_constraint_FR = np.array([constraint[9][1], constraint[10][1], constraint[11][1] - 0.002])
                first_low_constraint_RL = np.array([constraint[9][2], constraint[10][2], constraint[11][2] - 0.002])
                first_low_constraint_RR = np.array([constraint[9][3], constraint[10][3], constraint[11][3] - 0.002])

                # Rotate the constraint in the horizontal frame
                first_up_constraint_FL[0:2] = h_R_w @ (first_up_constraint_FL[0:2] - base[0:2])
                first_up_constraint_FL = first_up_constraint_FL + 0.005
                # first_up_constraint_FL[2] = first_up_constraint_FL[2] + 0.5

                first_up_constraint_FR[0:2] = h_R_w @ (first_up_constraint_FR[0:2] - base[0:2])
                first_up_constraint_FR = first_up_constraint_FR + 0.005
                # first_up_constraint_FR[2] = first_up_constraint_FR[2] + 0.5

                first_up_constraint_RL[0:2] = h_R_w @ (first_up_constraint_RL[0:2] - base[0:2])
                first_up_constraint_RL = first_up_constraint_RL + 0.005
                # first_up_constraint_RL[2] = first_up_constraint_RL[2] + 0.5

                first_up_constraint_RR[0:2] = h_R_w @ (first_up_constraint_RR[0:2] - base[0:2])
                first_up_constraint_RR = first_up_constraint_RR + 0.005
                # first_up_constraint_RR[2] = first_up_constraint_RR[2] + 0.5

                first_low_constraint_FL[0:2] = h_R_w @ (first_low_constraint_FL[0:2] - base[0:2])
                first_low_constraint_FL = first_low_constraint_FL - 0.005
                # first_low_constraint_FL[2] = first_low_constraint_FL[2] - 0.5

                first_low_constraint_FR[0:2] = h_R_w @ (first_low_constraint_FR[0:2] - base[0:2])
                first_low_constraint_FR = first_low_constraint_FR - 0.005
                # first_low_constraint_FR[2] = first_low_constraint_FR[2] - 0.5

                first_low_constraint_RL[0:2] = h_R_w @ (first_low_constraint_RL[0:2] - base[0:2])
                first_low_constraint_RL = first_low_constraint_RL - 0.005
                # first_low_constraint_RL[2] = first_low_constraint_RL[2] - 0.5

                first_low_constraint_RR[0:2] = h_R_w @ (first_low_constraint_RR[0:2] - base[0:2])
                first_low_constraint_RR = first_low_constraint_RR - 0.005
                # first_low_constraint_RR[2] = first_low_constraint_RR[2] - 0.5
            else:
                # Constrain taken from the nominal foothold (NO VISION)

                # FL first touchdown constraint
                first_up_constraint_FL = np.array(
                    [FL_reference_foot[0][0], FL_reference_foot[0][1], FL_reference_foot[0][2] + 0.002]
                )
                first_up_constraint_FL[0:2] = h_R_w @ (first_up_constraint_FL[0:2] - base[0:2]) + 0.15

                first_low_constraint_FL = np.array(
                    [FL_reference_foot[0][0], FL_reference_foot[0][1], FL_reference_foot[0][2] - 0.002]
                )
                first_low_constraint_FL[0:2] = h_R_w @ (first_low_constraint_FL[0:2] - base[0:2]) - 0.15

                # FR first touchdown constraint
                first_up_constraint_FR = np.array(
                    [FR_reference_foot[0][0], FR_reference_foot[0][1], FR_reference_foot[0][2] + 0.002]
                )
                first_up_constraint_FR[0:2] = h_R_w @ (first_up_constraint_FR[0:2] - base[0:2]) + 0.15

                first_low_constraint_FR = np.array(
                    [FR_reference_foot[0][0], FR_reference_foot[0][1], FR_reference_foot[0][2] - 0.002]
                )
                first_low_constraint_FR[0:2] = h_R_w @ (first_low_constraint_FR[0:2] - base[0:2]) - 0.15

                # RL first touchdown constraint
                first_up_constraint_RL = np.array(
                    [RL_reference_foot[0][0], RL_reference_foot[0][1], RL_reference_foot[0][2] + 0.002]
                )
                first_up_constraint_RL[0:2] = h_R_w @ (first_up_constraint_RL[0:2] - base[0:2]) + 0.15

                first_low_constraint_RL = np.array(
                    [RL_reference_foot[0][0], RL_reference_foot[0][1], RL_reference_foot[0][2] - 0.002]
                )
                first_low_constraint_RL[0:2] = h_R_w @ (first_low_constraint_RL[0:2] - base[0:2]) - 0.15

                # RR first touchdown constraint
                first_up_constraint_RR = np.array(
                    [RR_reference_foot[0][0], RR_reference_foot[0][1], RR_reference_foot[0][2] + 0.002]
                )
                first_up_constraint_RR[0:2] = h_R_w @ (first_up_constraint_RR[0:2] - base[0:2]) + 0.15

                first_low_constraint_RR = np.array(
                    [RR_reference_foot[0][0], RR_reference_foot[0][1], RR_reference_foot[0][2] - 0.002]
                )
                first_low_constraint_RR[0:2] = h_R_w @ (first_low_constraint_RR[0:2] - base[0:2]) - 0.15

            # we stack all the constraint we have for now
            up_constraint_FL = np.vstack((stance_up_constraint_FL, first_up_constraint_FL))
            up_constraint_FR = np.vstack((stance_up_constraint_FR, first_up_constraint_FR))
            up_constraint_RL = np.vstack((stance_up_constraint_RL, first_up_constraint_RL))
            up_constraint_RR = np.vstack((stance_up_constraint_RR, first_up_constraint_RR))

            low_constraint_FL = np.vstack((stance_low_constraint_FL, first_low_constraint_FL))
            low_constraint_FR = np.vstack((stance_low_constraint_FR, first_low_constraint_FR))
            low_constraint_RL = np.vstack((stance_low_constraint_RL, first_low_constraint_RL))
            low_constraint_RR = np.vstack((stance_low_constraint_RR, first_low_constraint_RR))

            # If there are more than 1 nominal foothold per leg, we create additional constraints
            # We do not expect more than two reference footholds...

            # FL second touchdown constraint
            if FL_reference_foot.shape[0] == 2:
                second_up_constraint_FL = np.array(
                    [FL_reference_foot[1][0], FL_reference_foot[1][1], FL_reference_foot[1][2] + 0.002]
                )
                second_up_constraint_FL[0:2] = h_R_w @ (second_up_constraint_FL[0:2] - base[0:2]) + 0.15

                second_low_constraint_FL = np.array(
                    [FL_reference_foot[1][0], FL_reference_foot[1][1], FL_reference_foot[1][2] - 0.002]
                )
                second_low_constraint_FL[0:2] = h_R_w @ (second_low_constraint_FL[0:2] - base[0:2]) - 0.15

                up_constraint_FL = np.vstack((up_constraint_FL, second_up_constraint_FL))
                low_constraint_FL = np.vstack((low_constraint_FL, second_low_constraint_FL))

            # FR second touchdown constraint
            if FR_reference_foot.shape[0] == 2:
                second_up_constraint_FR = np.array(
                    [FR_reference_foot[1][0], FR_reference_foot[1][1], FR_reference_foot[1][2] + 0.002]
                )
                second_up_constraint_FR[0:2] = h_R_w @ (second_up_constraint_FR[0:2] - base[0:2]) + 0.15

                second_low_constraint_FR = np.array(
                    [FR_reference_foot[1][0], FR_reference_foot[1][1], FR_reference_foot[1][2] - 0.002]
                )
                second_low_constraint_FR[0:2] = h_R_w @ (second_low_constraint_FR[0:2] - base[0:2]) - 0.15

                up_constraint_FR = np.vstack((up_constraint_FR, second_up_constraint_FR))
                low_constraint_FR = np.vstack((low_constraint_FR, second_low_constraint_FR))

            # RL second touchdown constraint
            if RL_reference_foot.shape[0] == 2:
                second_up_constraint_RL = np.array(
                    [RL_reference_foot[1][0], RL_reference_foot[1][1], RL_reference_foot[1][2] + 0.002]
                )
                second_up_constraint_RL[0:2] = h_R_w @ (second_up_constraint_RL[0:2] - base[0:2]) + 0.15

                second_low_constraint_RL = np.array(
                    [RL_reference_foot[1][0], RL_reference_foot[1][1], RL_reference_foot[1][2] - 0.002]
                )
                second_low_constraint_RL[0:2] = h_R_w @ (second_low_constraint_RL[0:2] - base[0:2]) - 0.15

                up_constraint_RL = np.vstack((up_constraint_RL, second_up_constraint_RL))
                low_constraint_RL = np.vstack((low_constraint_RL, second_low_constraint_RL))

            # RR second touchdown constraint
            if RR_reference_foot.shape[0] == 2:
                second_up_constraint_RR = np.array(
                    [RR_reference_foot[1][0], RR_reference_foot[1][1], RR_reference_foot[1][2] + 0.002]
                )
                second_up_constraint_RR[0:2] = h_R_w @ (second_up_constraint_RR[0:2] - base[0:2]) + 0.15

                second_low_constraint_RR = np.array(
                    [RR_reference_foot[1][0], RR_reference_foot[1][1], RR_reference_foot[1][2] - 0.002]
                )
                second_low_constraint_RR[0:2] = h_R_w @ (second_low_constraint_RR[0:2] - base[0:2]) - 0.15

                up_constraint_RR = np.vstack((up_constraint_RR, second_up_constraint_RR))
                low_constraint_RR = np.vstack((low_constraint_RR, second_low_constraint_RR))

            # We pass all the constraints (foothold and friction conte) to acados.
            # Then one regarding friction are precomputed
            ub_friction = self.constr_uh_friction
            lb_friction = self.constr_lh_friction

            # If the foothold is in swing, the idx of the constraint start from 1
            idx_constraint = np.array([0, 0, 0, 0])
            if FL_contact_sequence[0] == 0:
                idx_constraint[0] = 1
            if FR_contact_sequence[0] == 0:
                idx_constraint[1] = 1
            if RL_contact_sequence[0] == 0:
                idx_constraint[2] = 1
            if RR_contact_sequence[0] == 0:
                idx_constraint[3] = 1

            for j in range(0, self.horizon):
                # take the constraint for the current timestep
                ub_foot_FL = up_constraint_FL[idx_constraint[0]]
                lb_foot_FL = low_constraint_FL[idx_constraint[0]]

                ub_foot_FR = up_constraint_FR[idx_constraint[1]]
                lb_foot_FR = low_constraint_FR[idx_constraint[1]]

                ub_foot_RL = up_constraint_RL[idx_constraint[2]]
                lb_foot_RL = low_constraint_RL[idx_constraint[2]]

                ub_foot_RR = up_constraint_RR[idx_constraint[3]]
                lb_foot_RR = low_constraint_RR[idx_constraint[3]]

                # Concatenate the friction and foothold constraint
                ub_foot = copy.deepcopy(np.concatenate((ub_foot_FL, ub_foot_FR, ub_foot_RL, ub_foot_RR)))
                lb_foot = copy.deepcopy(np.concatenate((lb_foot_FL, lb_foot_FR, lb_foot_RL, lb_foot_RR)))
                if self.use_foothold_constraints:
                    ub_total = np.concatenate((ub_friction, ub_foot))
                    lb_total = np.concatenate((lb_friction, lb_foot))
                else:
                    ub_total = ub_friction
                    lb_total = lb_friction

                # Constraints for the support polygon depending on the leg in stance
                # all disabled at the beginning!!
                if self.use_stability_constraints:
                    ub_support_FL_FR = ACADOS_INFTY
                    lb_support_FL_FR = -ACADOS_INFTY

                    ub_support_FR_RR = ACADOS_INFTY
                    lb_support_FR_RR = -ACADOS_INFTY

                    ub_support_RR_RL = ACADOS_INFTY
                    lb_support_RR_RL = -ACADOS_INFTY

                    ub_support_RL_FL = ACADOS_INFTY
                    lb_support_RL_FL = -ACADOS_INFTY

                    ub_support_FL_RR = ACADOS_INFTY
                    lb_support_FL_RR = -ACADOS_INFTY

                    ub_support_FR_RL = ACADOS_INFTY
                    lb_support_FR_RL = -ACADOS_INFTY

                    # We have 4 cases for the stability constraint: trot, pace, crawl, full stance
                    if (
                        FL_contact_sequence[j] == 1
                        and FR_contact_sequence[j] == 1
                        and RL_contact_sequence[j] == 1
                        and RR_contact_sequence[j] == 1
                    ):
                        # FULL STANCE TODO
                        ub_support_FL_FR = ACADOS_INFTY
                        lb_support_FL_FR = -ACADOS_INFTY

                        ub_support_FR_RR = ACADOS_INFTY
                        lb_support_FR_RR = -ACADOS_INFTY

                        ub_support_RR_RL = ACADOS_INFTY
                        lb_support_RR_RL = -ACADOS_INFTY

                        ub_support_RL_FL = ACADOS_INFTY
                        lb_support_RL_FL = -ACADOS_INFTY

                        ub_support_FL_RR = ACADOS_INFTY
                        lb_support_FL_RR = -ACADOS_INFTY

                        ub_support_FR_RL = ACADOS_INFTY
                        lb_support_FR_RL = -ACADOS_INFTY

                    elif np.array_equal(FL_contact_sequence, RR_contact_sequence) and np.array_equal(
                        FR_contact_sequence, RL_contact_sequence
                    ):
                        # TROT
                        stability_margin = config.mpc_params['trot_stability_margin']
                        if FL_contact_sequence[j] == 1 and FR_contact_sequence[j] == 0:
                            ub_support_FL_RR = 0 + stability_margin
                            lb_support_FL_RR = 0 - stability_margin

                        if FR_contact_sequence[j] == 1 and FL_contact_sequence[j] == 0:
                            ub_support_FR_RL = 0 + stability_margin
                            lb_support_FR_RL = 0 - stability_margin

                    elif np.array_equal(FL_contact_sequence, RL_contact_sequence) and np.array_equal(
                        FR_contact_sequence, RR_contact_sequence
                    ):
                        # PACE
                        stability_margin = config.mpc_params['pace_stability_margin']
                        if FL_contact_sequence[j] == 1 and FR_contact_sequence[j] == 0:
                            ub_support_RL_FL = 0 + stability_margin
                            lb_support_RL_FL = 0 - stability_margin

                        if FR_contact_sequence[j] == 1 and FL_contact_sequence[j] == 0:
                            ub_support_FR_RR = 0 + stability_margin
                            lb_support_FR_RR = 0 - stability_margin

                    else:
                        # CRAWL BACKDIAGONALCRAWL ONLY
                        stability_margin = config.mpc_params["crawl_stability_margin"]

                        if FL_contact_sequence[j] == 1:
                            if FR_contact_sequence[j] == 1:
                                ub_support_FL_FR = -0.0 - stability_margin
                                lb_support_FL_FR = -ACADOS_INFTY
                            else:
                                ub_support_FL_RR = ACADOS_INFTY
                                lb_support_FL_RR = 0.0 + stability_margin

                        if FR_contact_sequence[j] == 1:
                            if RR_contact_sequence[j] == 1:
                                ub_support_FR_RR = ACADOS_INFTY
                                lb_support_FR_RR = 0.0 + stability_margin
                            else:
                                ub_support_FR_RL = ACADOS_INFTY
                                lb_support_FR_RL = 0.0 + stability_margin

                        if RR_contact_sequence[j] == 1:
                            if RL_contact_sequence[j] == 1:
                                ub_support_RR_RL = ACADOS_INFTY
                                lb_support_RR_RL = 0.0 + stability_margin
                            else:
                                ub_support_FL_RR = -0.0 - stability_margin
                                lb_support_FL_RR = -ACADOS_INFTY

                        if RL_contact_sequence[j] == 1:
                            if FL_contact_sequence[j] == 1:
                                ub_support_RL_FL = -0.0 - stability_margin
                                lb_support_RL_FL = -ACADOS_INFTY
                            else:
                                ub_support_FR_RL = -0.0 - stability_margin
                                lb_support_FR_RL = -ACADOS_INFTY

                    ub_support = np.array(
                        [
                            ub_support_FL_FR,
                            ub_support_FR_RR,
                            ub_support_RR_RL,
                            ub_support_RL_FL,
                            ub_support_FL_RR,
                            ub_support_FR_RL,
                        ]
                    )
                    lb_support = np.array(
                        [
                            lb_support_FL_FR,
                            lb_support_FR_RR,
                            lb_support_RR_RL,
                            lb_support_RL_FL,
                            lb_support_FL_RR,
                            lb_support_FR_RL,
                        ]
                    )

                    ub_total = np.concatenate((ub_total, ub_support))
                    lb_total = np.concatenate((lb_total, lb_support))

                # No friction constraint at the end! we don't have u_N
                if j == self.horizon:
                    if self.use_foothold_constraints:
                        if self.use_stability_constraints:
                            ub_total = np.concatenate((ub_foot, ub_support))
                            lb_total = np.concatenate((lb_foot, lb_support))
                        else:
                            ub_total = ub_foot
                            lb_total = lb_foot
                    else:
                        if self.use_stability_constraints:
                            ub_total = ub_support
                            lb_total = lb_support
                        else:
                            continue

                # Only friction costraints at the beginning
                if j == 0:
                    self.acados_ocp_solver.constraints_set(j, "uh", ub_friction)
                    self.acados_ocp_solver.constraints_set(j, "lh", lb_friction)
                if j > 0:
                    self.acados_ocp_solver.constraints_set(j, "uh", ub_total)
                    self.acados_ocp_solver.constraints_set(j, "lh", lb_total)

                # save the constraint for logging
                self.upper_bound[j] = ub_total.tolist()
                self.lower_bound[j] = lb_total.tolist()

                # ugly procedure to update the idx of the constraint
                if j >= 1:
                    if FL_contact_sequence[j] == 0 and FL_contact_sequence[j - 1] == 1:
                        if idx_constraint[0] < up_constraint_FL.shape[0] - 1:
                            idx_constraint[0] += 1

                    if FR_contact_sequence[j] == 0 and FR_contact_sequence[j - 1] == 1:
                        if idx_constraint[1] < up_constraint_FR.shape[0] - 1:
                            idx_constraint[1] += 1

                    if RL_contact_sequence[j] == 0 and RL_contact_sequence[j - 1] == 1:
                        if idx_constraint[2] < up_constraint_RL.shape[0] - 1:
                            idx_constraint[2] += 1

                    if RR_contact_sequence[j] == 0 and RR_contact_sequence[j - 1] == 1:
                        if idx_constraint[3] < up_constraint_RR.shape[0] - 1:
                            idx_constraint[3] += 1

        except:
            print("###WARNING: error in setting the constraints")

        return

    # Method to perform the centering of the states and the reference around (0, 0, 0)
    def perform_scaling(self, state, reference, constraint=None):
        self.initial_base_position = copy.deepcopy(state["position"])
        reference = copy.deepcopy(reference)
        state = copy.deepcopy(state)

        reference["ref_position"] = reference["ref_position"] - state["position"]
        reference["ref_foot_FL"] = reference["ref_foot_FL"] - state["position"]
        reference["ref_foot_FR"] = reference["ref_foot_FR"] - state["position"]
        reference["ref_foot_RL"] = reference["ref_foot_RL"] - state["position"]
        reference["ref_foot_RR"] = reference["ref_foot_RR"] - state["position"]

        state["foot_FL"] = state["foot_FL"] - state["position"]
        state["foot_FR"] = state["foot_FR"] - state["position"]
        state["foot_RL"] = state["foot_RL"] - state["position"]
        state["foot_RR"] = state["foot_RR"] - state["position"]
        state["position"] = np.array([0, 0, 0])

        return state, reference, constraint

    # Main loop for computing batched control
    def compute_batch_control(
        self,
        state,
        reference,
        contact_sequence,
        constraint=None,
        external_wrenches=np.zeros((6,)),
        inertia=config.inertia.reshape((9,)),
        mass=config.mass,
    ):
        start = time.time()

        costs = []

        # Perform the scaling of the states and the reference
        state, reference, constraint = self.perform_scaling(state, reference, constraint)

        # Fill reference (self.states_dim+self.inputs_dim)
        idx_ref_foot_to_assign = np.array([0, 0, 0, 0])
        yref = np.zeros(shape=(self.states_dim + self.inputs_dim,))
        yref[0:3] = reference["ref_position"]
        yref[3:6] = reference["ref_linear_velocity"]
        yref[6:9] = reference["ref_orientation"]
        yref[9:12] = reference["ref_angular_velocity"]
        yref[12:15] = reference["ref_foot_FL"][idx_ref_foot_to_assign[0]]
        yref[15:18] = reference["ref_foot_FR"][idx_ref_foot_to_assign[1]]
        yref[18:21] = reference["ref_foot_RL"][idx_ref_foot_to_assign[2]]
        yref[21:24] = reference["ref_foot_RR"][idx_ref_foot_to_assign[3]]

        yref_N = np.zeros(shape=(self.states_dim,))
        yref_N[0:3] = reference["ref_position"]
        yref_N[3:6] = reference["ref_linear_velocity"]
        yref_N[6:9] = reference["ref_orientation"]
        yref_N[9:12] = reference["ref_angular_velocity"]

        # Set initial state constraint. We teleported the robot foothold
        # to the previous optimal foothold. This is done to avoid the optimization
        # of a foothold that is not considered at all at touchdown! In any case,
        # the height cames always from the VFA
        FL_contact_sequence = contact_sequence[0][0]
        FR_contact_sequence = contact_sequence[0][1]
        RL_contact_sequence = contact_sequence[0][2]
        RR_contact_sequence = contact_sequence[0][3]
        if FL_contact_sequence[0] == 0:
            state["foot_FL"] = reference["ref_foot_FL"][0]

        if FR_contact_sequence[0] == 0:
            state["foot_FR"] = reference["ref_foot_FR"][0]

        if RL_contact_sequence[0] == 0:
            state["foot_RL"] = reference["ref_foot_RL"][0]

        if RR_contact_sequence[0] == 0:
            state["foot_RR"] = reference["ref_foot_RR"][0]

        # Fill stance param, friction and stance proximity
        # (stance proximity will disable foothold optimization near a stance!!)
        mu = config.mpc_params["mu"]

        state_acados = np.concatenate(
            (
                state["position"],
                state["linear_velocity"],
                state["orientation"],
                state["angular_velocity"],
                state["foot_FL"],
                state["foot_FR"],
                state["foot_RL"],
                state["foot_RR"],
                self.integral_errors,
            )
        )  # .reshape((self.states_dim, 1))

        for n in range(self.batch):
            # Take the array of the contact sequence and split it in 4 arrays,
            # one for each leg
            FL_contact_sequence = contact_sequence[n][0]
            FR_contact_sequence = contact_sequence[n][1]
            RL_contact_sequence = contact_sequence[n][2]
            RR_contact_sequence = contact_sequence[n][3]

            for j in range(self.horizon):
                # Update the idx_ref_foot_to_assign. Every time there is a change in the contact phase
                # between 1 and 0, it means that the leg go into swing and a new reference is needed!!!
                if j > 1 and j < self.horizon - 1:
                    if FL_contact_sequence[j + 1] == 0 and FL_contact_sequence[j] == 1:
                        if reference['ref_foot_FL'].shape[0] > idx_ref_foot_to_assign[0] + 1:
                            idx_ref_foot_to_assign[0] += 1
                    if FR_contact_sequence[j + 1] == 0 and FR_contact_sequence[j] == 1:
                        if reference['ref_foot_FR'].shape[0] > idx_ref_foot_to_assign[1] + 1:
                            idx_ref_foot_to_assign[1] += 1
                    if RL_contact_sequence[j + 1] == 0 and RL_contact_sequence[j] == 1:
                        if reference['ref_foot_RL'].shape[0] > idx_ref_foot_to_assign[2] + 1:
                            idx_ref_foot_to_assign[2] += 1
                    if RR_contact_sequence[j + 1] == 0 and RR_contact_sequence[j] == 1:
                        if reference['ref_foot_RR'].shape[0] > idx_ref_foot_to_assign[3] + 1:
                            idx_ref_foot_to_assign[3] += 1

                # Setting the reference to acados
                self.batch_solver.ocp_solvers[n].set(j, "yref", yref)

                # Fill last step horizon reference (self.states_dim - no control action!!)
            yref_N[12:15] = reference["ref_foot_FL"][idx_ref_foot_to_assign[0]]
            yref_N[15:18] = reference["ref_foot_FR"][idx_ref_foot_to_assign[1]]
            yref_N[18:21] = reference["ref_foot_RL"][idx_ref_foot_to_assign[2]]
            yref_N[21:24] = reference["ref_foot_RR"][idx_ref_foot_to_assign[3]]

            # Setting the reference to acados
            self.batch_solver.ocp_solvers[n].set(self.horizon, "yref", yref_N)

            # Set the parameters to  acados
            for j in range(self.horizon):
                param = np.array(
                    [
                        FL_contact_sequence[j],
                        FR_contact_sequence[j],
                        RL_contact_sequence[j],
                        RR_contact_sequence[j],
                        mu,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        state["position"][0],
                        state["position"][1],
                        state["position"][2],
                        state["orientation"][2],
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        inertia[0],
                        inertia[1],
                        inertia[2],
                        inertia[3],
                        inertia[4],
                        inertia[5],
                        inertia[6],
                        inertia[7],
                        inertia[8],
                        mass,
                    ]
                )
                self.batch_solver.ocp_solvers[n].set(j, "p", param)

            # Set initial state constraint acados, converting first the dictionary to np array
            self.batch_solver.ocp_solvers[n].set(0, "lbx", state_acados)
            self.batch_solver.ocp_solvers[n].set(0, "ubx", state_acados)

        t_elapsed2 = time.time() - start

        # Solve the batched ocp
        t0 = time.time()
        self.batch_solver.solve()
        t_elapsed = time.time() - t0

        print("time_python: ", t_elapsed2)
        print("time_solver: ", t_elapsed)

        for n in range(self.batch):
            cost_single_qp = self.batch_solver.ocp_solvers[n].get_cost()
            if n != 0:
                cost_single_qp += (
                    3 * (config.mpc_params["step_freq_available"][n] - config.mpc_params["step_freq_available"][0]) ** 2
                )
            costs.append(cost_single_qp)

        best_freq_index = np.argmin(costs)
        best_freq = config.mpc_params["step_freq_available"][best_freq_index]

        print("costs: ", costs)
        print("best_freq: ", best_freq)

        return costs, best_freq
