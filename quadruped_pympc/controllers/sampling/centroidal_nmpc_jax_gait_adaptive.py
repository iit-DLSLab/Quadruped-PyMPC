import numpy as np
np.set_printoptions(precision=3, suppress=True)

import os
os.environ['XLA_FLAGS'] = '--xla_gpu_triton_gemm_any=True'

import jax
import jax.numpy as jnp
from jax import random

import sys
import copy

from quadruped_pympc.helpers.periodic_gait_generator_jax import PeriodicGaitGeneratorJax
from .centroidal_model_jax import Centroidal_Model_JAX
import quadruped_pympc.config as config

dtype_general = 'float32'
jax.config.update("jax_default_matmul_precision", "float32")


class Sampling_MPC:
    """This is a small class that implements a sampling based control law"""

    def __init__(
        self,
    ):

        device = config.mpc_params["device"]
        self.num_parallel_computations = config.mpc_params["num_parallel_computations"]
        self.sampling_method = config.mpc_params["sampling_method"]
        self.control_parametrization = config.mpc_params["control_parametrization"]
        self.num_sampling_iterations = config.mpc_params["num_sampling_iterations"]
        self.dt = config.mpc_params["dt"]
        self.horizon = config.mpc_params["horizon"]

        self.state_dim = 24
        self.control_dim = 24
        self.reference_dim = self.state_dim

        self.max_sampling_forces_x = 10
        self.max_sampling_forces_y = 10
        self.max_sampling_forces_z = 30

        if device == "gpu":
            try:
                self.device = jax.devices("gpu")[0]
            except:
                self.device = jax.devices("cpu")[0]
                print("GPU not available, using CPU")
        else:
            self.device = jax.devices('cpu')[0]


        if self.control_parametrization == "linear_spline":
            # Along the horizon, we have 2 splines per control input (3 forces)
            # Each spline has 2 parameters, but one is shared between the two splines
            self.num_spline = config.mpc_params['num_splines']
            self.num_control_parameters_single_leg = (self.num_spline + 1) * 3

            # In totale we have 4 legs
            self.num_control_parameters = self.num_control_parameters_single_leg * 4

            # We have 4 different spline functions, one for each leg
            self.spline_fun_FL = self.compute_linear_spline
            self.spline_fun_FR = self.compute_linear_spline
            self.spline_fun_RL = self.compute_linear_spline
            self.spline_fun_RR = self.compute_linear_spline


        elif self.control_parametrization == "cubic_spline":
            # Along the horizon, we have 1 splines per control input (3 forces)
            # Each spline has 3 parameters
            self.num_spline = config.mpc_params['num_splines']
            self.num_control_parameters_single_leg = 4 * 3 * self.num_spline

            # In totale we have 4 legs
            self.num_control_parameters = self.num_control_parameters_single_leg * 4

            # We have 4 different spline functions, one for each leg
            self.spline_fun_FL = self.compute_cubic_spline
            self.spline_fun_FR = self.compute_cubic_spline
            self.spline_fun_RL = self.compute_cubic_spline
            self.spline_fun_RR = self.compute_cubic_spline

        else:
            # We have 1 parameters for every 3 force direction (x,y,z)...for each time horizon!!
            self.num_control_parameters_single_leg = self.horizon * 3

            # In totale we have 4 legs
            self.num_control_parameters = self.num_control_parameters_single_leg * 4

            # We have 4 different spline functions, one for each leg
            self.spline_fun_FL = self.compute_zero_order_spline
            self.spline_fun_FR = self.compute_zero_order_spline
            self.spline_fun_RL = self.compute_zero_order_spline
            self.spline_fun_RR = self.compute_zero_order_spline

        if self.sampling_method == 'random_sampling':
            self.compute_control = self.compute_control_random_sampling
            self.sigma_random_sampling = config.mpc_params['sigma_random_sampling']
        elif self.sampling_method == 'mppi':
            self.compute_control = self.compute_control_mppi
            self.sigma_mppi = config.mpc_params['sigma_mppi']
        elif self.sampling_method == 'cem_mppi':
            self.compute_control = self.compute_control_cem_mppi
            self.sigma_cem_mppi = (
                jnp.ones(self.num_control_parameters, dtype=dtype_general) * config.mpc_params['sigma_cem_mppi']
            )
        else:
            # return error and stop execution
            print("Error: sampling method not recognized")
            sys.exit(1)

        self.jitted_compute_control = jax.jit(self.compute_control, device=self.device)

        # Initialize the robot model
        self.robot = Centroidal_Model_JAX(self.dt, self.device)

        # Initialize the cost function matrices
        self.Q = jnp.identity(self.state_dim, dtype=dtype_general) * 0
        self.Q = self.Q.at[0, 0].set(0.0)
        self.Q = self.Q.at[1, 1].set(0.0)
        self.Q = self.Q.at[2, 2].set(1500)  # com_z
        self.Q = self.Q.at[3, 3].set(200)  # com_vel_x
        self.Q = self.Q.at[4, 4].set(200)  # com_vel_y
        self.Q = self.Q.at[5, 5].set(200)  # com_vel_z
        self.Q = self.Q.at[6, 6].set(500)  # base_angle_roll
        self.Q = self.Q.at[7, 7].set(500)  # base_angle_pitch
        self.Q = self.Q.at[8, 8].set(0.0)  # base_angle_yaw
        self.Q = self.Q.at[9, 9].set(20)  # base_angle_rates_x
        self.Q = self.Q.at[10, 10].set(20)  # base_angle_rates_y
        self.Q = self.Q.at[11, 11].set(50)  # base_angle_rates_z

        self.R = jnp.identity(self.control_dim, dtype=dtype_general)
        self.R = self.R.at[0, 0].set(0.0)  # foot_pos_x_FL
        self.R = self.R.at[1, 1].set(0.0)  # foot_pos_y_FL
        self.R = self.R.at[2, 2].set(0.0)  # foot_pos_z_FL
        self.R = self.R.at[3, 3].set(0.0)  # foot_pos_x_FR
        self.R = self.R.at[4, 4].set(0.0)  # foot_pos_y_FR
        self.R = self.R.at[5, 5].set(0.0)  # foot_pos_z_FR
        self.R = self.R.at[6, 6].set(0.0)  # foot_pos_x_RL
        self.R = self.R.at[7, 7].set(0.0)  # foot_pos_y_RL
        self.R = self.R.at[8, 8].set(0.0)  # foot_pos_z_RL
        self.R = self.R.at[9, 9].set(0.0)  # foot_pos_x_RR
        self.R = self.R.at[10, 10].set(0.0)  # foot_pos_y_RR
        self.R = self.R.at[11, 11].set(0.0)  # foot_pos_z_RR

        self.R = self.R.at[12, 12].set(0.1)  # foot_force_x_FL
        self.R = self.R.at[13, 13].set(0.1)  # foot_force_y_FL
        self.R = self.R.at[14, 14].set(0.001)  # foot_force_z_FL
        self.R = self.R.at[15, 15].set(0.1)  # foot_force_x_FR
        self.R = self.R.at[16, 16].set(0.1)  # foot_force_y_FR
        self.R = self.R.at[17, 17].set(0.001)  # foot_force_z_FR
        self.R = self.R.at[18, 18].set(0.1)  # foot_force_x_RL
        self.R = self.R.at[19, 19].set(0.1)  # foot_force_y_RL
        self.R = self.R.at[20, 20].set(0.001)  # foot_force_z_RL
        self.R = self.R.at[21, 21].set(0.1)  # foot_force_x_RR
        self.R = self.R.at[22, 22].set(0.1)  # foot_force_y_RR
        self.R = self.R.at[23, 23].set(0.001)  # foot_force_z_RR

        # mu is the friction coefficient
        self.mu = config.mpc_params["mu"]

        # maximum allowed z contact forces
        self.f_z_max = config.mpc_params['grf_max']
        self.f_z_min = config.mpc_params['grf_min']

        self.best_control_parameters = jnp.zeros((self.num_control_parameters,), dtype=dtype_general)
        self.master_key = jax.random.PRNGKey(42)
        self.initial_random_parameters = jax.random.uniform(
            key=self.master_key,
            minval=-self.max_sampling_forces_z,
            maxval=self.max_sampling_forces_z,
            shape=(self.num_parallel_computations, self.num_control_parameters),
        )

        # Initialize the periodic gait generator and jit it!
        self.pgg = PeriodicGaitGeneratorJax(duty_factor=0.65, step_freq=1.65, horizon=self.horizon, mpc_dt=self.dt)
        _, _ = self.pgg.compute_contact_sequence(simulation_dt=0.002, t=self.pgg.get_t(), step_freq=1.65)
        self.jitted_compute_contact_sequence = jax.jit(self.pgg.compute_contact_sequence, device=self.device)
        self.step_freq_delta = jnp.array(config.mpc_params['step_freq_available'])


        # jitting the vmap function!
        self.vectorized_rollout = jax.vmap(self.compute_rollout, in_axes=(None, None, None, 0, 0), out_axes=0)
        self.jit_vectorized_rollout = jax.jit(self.vectorized_rollout, device=self.device)


    def compute_linear_spline(self, parameters, step, horizon_leg):
        """
        Compute the linear spline parametrization of the GRF (N splines)
        """

        # Adding the last boundary for the case when step is exactly self.horizon
        chunk_boundaries = jnp.linspace(0, self.horizon, self.num_spline + 1)
        # Find the chunk index by checking in which interval the step falls
        index = jnp.max(jnp.where(step >= chunk_boundaries, jnp.arange(self.num_spline + 1), 0))
        
        tau = step / (horizon_leg/self.num_spline)
        tau = tau - 1*index

        q = (tau - 0.0) / (1.0 - 0.0)

        shift = self.num_spline + 1
        f_x = (1 - q) * parameters[index + 0] + q * parameters[index + 1]
        f_y = (1 - q) * parameters[index + shift] + q * parameters[index + shift + 1]
        f_z = (1 - q) * parameters[index + shift * 2] + q * parameters[index + shift * 2 + 1]

        return f_x, f_y, f_z


    def compute_cubic_spline(self, parameters, step, horizon_leg):
        """
        Compute the cubic spline parametrization of the GRF (N splines)
        """

        # Adding the last boundary for the case when step is exactly self.horizon
        chunk_boundaries = jnp.linspace(0, self.horizon, self.num_spline + 1)
        # Find the chunk index by checking in which interval the step falls
        index = jnp.max(jnp.where(step >= chunk_boundaries, jnp.arange(self.num_spline + 1), 0))
        
        tau = step / (horizon_leg/self.num_spline)
        tau = tau - 1*index

        q = (tau - 0.0) / (1.0 - 0.0)

        start_index = 10 * index

        q = (tau - 0.0) / (1.0 - 0.0)
        a = 2 * q * q * q - 3 * q * q + 1
        b = (q * q * q - 2 * q * q + q) * 1.0
        c = -2 * q * q * q + 3 * q * q
        d = (q * q * q - q * q) * 1.0

        phi = (1.0 / 2.0) * (
            ((parameters[start_index + 2] - parameters[start_index + 1]) / 1.0)
            + ((parameters[start_index + 1] - parameters[start_index + 0]) / 1.0)
        )
        phi_next = (1.0 / 2.0) * (
            ((parameters[start_index + 3] - parameters[start_index + 2]) / 1.0)
            + ((parameters[start_index + 2] - parameters[start_index + 1]) / 1.0)
        )
        f_x = a * parameters[start_index + 1] + b * phi + c * parameters[start_index + 2] + d * phi_next

        phi = (1.0 / 2.0) * (
            ((parameters[start_index + 6] - parameters[start_index + 5]) / 1.0)
            + ((parameters[start_index + 5] - parameters[start_index + 4]) / 1.0)
        )
        phi_next = (1.0 / 2.0) * (
            ((parameters[start_index + 7] - parameters[start_index + 6]) / 1.0)
            + ((parameters[start_index + 6] - parameters[start_index + 5]) / 1.0)
        )
        f_y = a * parameters[start_index + 5] + b * phi + c * parameters[start_index + 6] + d * phi_next

        phi = (1.0 / 2.0) * (
            ((parameters[start_index + 10] - parameters[start_index + 9]) / 1.0)
            + ((parameters[start_index + 9] - parameters[start_index + 8]) / 1.0)
        )
        phi_next = (1.0 / 2.0) * (
            ((parameters[start_index + 11] - parameters[start_index + 10]) / 1.0)
            + ((parameters[start_index + 10] - parameters[start_index + 9]) / 1.0)
        )
        f_z = a * parameters[start_index + 9] + b * phi + c * parameters[start_index + 10] + d * phi_next

        return f_x, f_y, f_z
    

    def compute_zero_order_spline(self, parameters, step, horizon_leg):
        """
        Compute the zero-order control input
        """

        index = jnp.int16(step)
        f_x = parameters[index]
        f_y = parameters[index + self.horizon]
        f_z = parameters[index + self.horizon * 2]
        return f_x, f_y, f_z

    def enforce_force_constraints(
        self, f_x_FL, f_y_FL, f_z_FL, f_x_FR, f_y_FR, f_z_FR, f_x_RL, f_y_RL, f_z_RL, f_x_RR, f_y_RR, f_z_RR
    ):
        """
        Enforce the friction cone and the force limits constraints
        """

        # Enforce push-only of the ground!
        f_z_FL = jax.numpy.where(f_z_FL > self.f_z_min, f_z_FL, self.f_z_min)
        f_z_FR = jax.numpy.where(f_z_FR > self.f_z_min, f_z_FR, self.f_z_min)
        f_z_RL = jax.numpy.where(f_z_RL > self.f_z_min, f_z_RL, self.f_z_min)
        f_z_RR = jax.numpy.where(f_z_RR > self.f_z_min, f_z_RR, self.f_z_min)

        # Enforce maximum force per leg!
        f_z_FL = jax.numpy.where(f_z_FL < self.f_z_max, f_z_FL, self.f_z_max)
        f_z_FR = jax.numpy.where(f_z_FR < self.f_z_max, f_z_FR, self.f_z_max)
        f_z_RL = jax.numpy.where(f_z_RL < self.f_z_max, f_z_RL, self.f_z_max)
        f_z_RR = jax.numpy.where(f_z_RR < self.f_z_max, f_z_RR, self.f_z_max)

        # Enforce friction cone
        # ( f_{\text{min}} \leq f_z \leq f_{\text{max}} )
        # ( -\mu f_{\text{z}} \leq f_x \leq \mu f_{\text{z}} )
        # ( -\mu f_{\text{z}} \leq f_y \leq \mu f_{\text{z}} )

        f_x_FL = jax.numpy.where(f_x_FL > -self.mu * f_z_FL, f_x_FL, -self.mu * f_z_FL)
        f_x_FL = jax.numpy.where(f_x_FL < self.mu * f_z_FL, f_x_FL, self.mu * f_z_FL)
        f_y_FL = jax.numpy.where(f_y_FL > -self.mu * f_z_FL, f_y_FL, -self.mu * f_z_FL)
        f_y_FL = jax.numpy.where(f_y_FL < self.mu * f_z_FL, f_y_FL, self.mu * f_z_FL)

        f_x_FR = jax.numpy.where(f_x_FR > -self.mu * f_z_FR, f_x_FR, -self.mu * f_z_FR)
        f_x_FR = jax.numpy.where(f_x_FR < self.mu * f_z_FR, f_x_FR, self.mu * f_z_FR)
        f_y_FR = jax.numpy.where(f_y_FR > -self.mu * f_z_FR, f_y_FR, -self.mu * f_z_FR)
        f_y_FR = jax.numpy.where(f_y_FR < self.mu * f_z_FR, f_y_FR, self.mu * f_z_FR)

        f_x_RL = jax.numpy.where(f_x_RL > -self.mu * f_z_RL, f_x_RL, -self.mu * f_z_RL)
        f_x_RL = jax.numpy.where(f_x_RL < self.mu * f_z_RL, f_x_RL, self.mu * f_z_RL)
        f_y_RL = jax.numpy.where(f_y_RL > -self.mu * f_z_RL, f_y_RL, -self.mu * f_z_RL)
        f_y_RL = jax.numpy.where(f_y_RL < self.mu * f_z_RL, f_y_RL, self.mu * f_z_RL)

        f_x_RR = jax.numpy.where(f_x_RR > -self.mu * f_z_RR, f_x_RR, -self.mu * f_z_RR)
        f_x_RR = jax.numpy.where(f_x_RR < self.mu * f_z_RR, f_x_RR, self.mu * f_z_RR)
        f_y_RR = jax.numpy.where(f_y_RR > -self.mu * f_z_RR, f_y_RR, -self.mu * f_z_RR)
        f_y_RR = jax.numpy.where(f_y_RR < self.mu * f_z_RR, f_y_RR, self.mu * f_z_RR)

        return f_x_FL, f_y_FL, f_z_FL, f_x_FR, f_y_FR, f_z_FR, f_x_RL, f_y_RL, f_z_RL, f_x_RR, f_y_RR, f_z_RR

    def compute_rollout(self, initial_state, reference, timing, control_parameters, step_frequency):
        """Calculate cost of a rollout of the dynamics given random parameters
        Args:
            initial_state (np.array): actual state of the robot
            reference (np.array): desired state of the robot
            control_parameters (np.array): parameters for the controllers
            parameters (np.array): parameters for the simplified dynamics
        Returns:
            (float): cost of the rollout
        """

        state = initial_state
        cost = jnp.float32(0.0)
        n_ = jnp.array([-1, -1, -1, -1])

        contact_sequence, _ = self.jitted_compute_contact_sequence(
            simulation_dt=0.002, t=timing, step_freq=step_frequency
        )

        FL_num_of_contact = jnp.sum(contact_sequence[0]) + 1
        FR_num_of_contact = jnp.sum(contact_sequence[1]) + 1
        RL_num_of_contact = jnp.sum(contact_sequence[2]) + 1
        RR_num_of_contact = jnp.sum(contact_sequence[3]) + 1

        def iterate_fun(n, carry):
            cost, state, reference, n_ = carry

            n_ = n_.at[0].set(n_[0] + 1 * contact_sequence[0][n])
            n_ = n_.at[1].set(n_[1] + 1 * contact_sequence[1][n])
            n_ = n_.at[2].set(n_[2] + 1 * contact_sequence[2][n])
            n_ = n_.at[3].set(n_[3] + 1 * contact_sequence[3][n])

            f_x_FL, f_y_FL, f_z_FL = self.spline_fun_FL(
                control_parameters[0 : self.num_control_parameters_single_leg], n_[0], FL_num_of_contact
            )
            f_x_FR, f_y_FR, f_z_FR = self.spline_fun_FR(
                control_parameters[self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2],
                n_[1],
                FR_num_of_contact,
            )
            f_x_RL, f_y_RL, f_z_RL = self.spline_fun_RL(
                control_parameters[
                    self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
                ],
                n_[2],
                RL_num_of_contact,
            )
            f_x_RR, f_y_RR, f_z_RR = self.spline_fun_RR(
                control_parameters[
                    self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
                ],
                n_[3],
                RR_num_of_contact,
            )

            # The sampling over f_z is a delta over gravity compensation (only for the leg in stance!)
            number_of_legs_in_stance = (
                contact_sequence[0][n] + contact_sequence[1][n] + contact_sequence[2][n] + contact_sequence[3][n]
            )
            reference_force_stance_legs = (self.robot.mass * 9.81) / number_of_legs_in_stance

            f_z_FL = reference_force_stance_legs + f_z_FL
            f_z_FR = reference_force_stance_legs + f_z_FR
            f_z_RL = reference_force_stance_legs + f_z_RL
            f_z_RR = reference_force_stance_legs + f_z_RR

            # Foot in swing (contact sequence = 0) have zero force
            f_x_FL = f_x_FL * contact_sequence[0][n] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
            f_y_FL = f_y_FL * contact_sequence[0][n] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
            f_z_FL = f_z_FL * contact_sequence[0][n]

            f_x_FR = f_x_FR * contact_sequence[1][n] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
            f_y_FR = f_y_FR * contact_sequence[1][n] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
            f_z_FR = f_z_FR * contact_sequence[1][n]

            f_x_RL = f_x_RL * contact_sequence[2][n] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
            f_y_RL = f_y_RL * contact_sequence[2][n] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
            f_z_RL = f_z_RL * contact_sequence[2][n]

            f_x_RR = f_x_RR * contact_sequence[3][n] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
            f_y_RR = f_y_RR * contact_sequence[3][n] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
            f_z_RR = f_z_RR * contact_sequence[3][n]

            # Enforce force constraints
            f_x_FL, f_y_FL, f_z_FL, f_x_FR, f_y_FR, f_z_FR, f_x_RL, f_y_RL, f_z_RL, f_x_RR, f_y_RR, f_z_RR = (
                self.enforce_force_constraints(
                    f_x_FL, f_y_FL, f_z_FL, f_x_FR, f_y_FR, f_z_FR, f_x_RL, f_y_RL, f_z_RL, f_x_RR, f_y_RR, f_z_RR
                )
            )

            input = jnp.array(
                [
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    f_x_FL,
                    f_y_FL,
                    f_z_FL,
                    f_x_FR,
                    f_y_FR,
                    f_z_FR,
                    f_x_RL,
                    f_y_RL,
                    f_z_RL,
                    f_x_RR,
                    f_y_RR,
                    f_z_RR,
                ],
                dtype=dtype_general,
            )

            # Integrate the dynamics
            current_contact = jnp.array(
                [contact_sequence[0][n], contact_sequence[1][n], contact_sequence[2][n], contact_sequence[3][n]],
                dtype=dtype_general,
            )
            state_next = self.robot.integrate_jax(state, input, current_contact, n)

            # Calculate cost regulation state
            state_error = state_next - reference[0 : self.state_dim]
            error_cost = state_error.T @ self.Q @ state_error

            input_for_cost = jnp.array(
                [
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    jnp.float32(0),
                    f_x_FL,
                    f_y_FL,
                    f_z_FL - reference_force_stance_legs,
                    f_x_FR,
                    f_y_FR,
                    f_z_FR - reference_force_stance_legs,
                    f_x_RL,
                    f_y_RL,
                    f_z_RL - reference_force_stance_legs,
                    f_x_RR,
                    f_y_RR,
                    f_z_RR - reference_force_stance_legs,
                ],
                dtype=dtype_general,
            )

            # Calculate cost regulation input
            # error_cost += input_for_cost.T@self.R@input_for_cost

            # saturate in the case of NaN
            # state_next = jnp.where(jnp.isnan(state_next), 100, state_next)
            # error_cost = jnp.where(jnp.isnan(error_cost), 1000, error_cost)
            # cost = jnp.where(jnp.isnan(cost), 10000, cost)

            return (cost + error_cost, state_next, reference, n_)

        carry = (cost, state, reference, n_)
        cost, state, reference, n_ = jax.lax.fori_loop(0, self.horizon, iterate_fun, carry)

        cost += (step_frequency - 1.3) * 100 * (step_frequency - 1.3)
        return cost

    def with_newkey(self):
        newkey, subkey = jax.random.split(self.master_key)
        self.master_key = newkey
        return self

    def get_key(self):
        return self.master_key

    def with_newsigma(self, sigma):
        self.sigma_cem_mppi = sigma
        return self

    def get_sigma(self):
        return self.sigma_cem_mppi

    def shift_solution(self, best_control_parameters, step):
        """
        This function shift the control parameter ahed
        """

        best_control_parameters = np.array(best_control_parameters)
        FL_control = copy.deepcopy(best_control_parameters[0 : self.num_control_parameters_single_leg])
        FR_control = copy.deepcopy(
            best_control_parameters[self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2]
        )
        RL_control = copy.deepcopy(
            best_control_parameters[
                self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
            ]
        )
        RR_control = copy.deepcopy(
            best_control_parameters[
                self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
            ]
        )

        FL_control_temp = copy.deepcopy(FL_control)
        FL_control[0], FL_control[2], FL_control[4] = self.spline_fun_FL(FL_control_temp, step)
        # FL_control[1], FL_control[3], FL_control[5] = controller.spline_fun_FL(FL_control_temp, controller.horizon+step)

        FR_control_temp = copy.deepcopy(FR_control)
        FR_control[0], FR_control[2], FR_control[4] = self.spline_fun_FR(FR_control_temp, step)
        # FR_control[1], FR_control[3], FR_control[5] = controller.spline_fun_FR(FR_control_temp, controller.horizon+step)

        RL_control_temp = copy.deepcopy(RL_control)
        RL_control[0], RL_control[2], RL_control[4] = self.spline_fun_RL(RL_control_temp, step)
        # RL_control[1], RL_control[3], RL_control[5] = controller.spline_fun_RL(RL_control_temp, controller.horizon+step)

        RR_control_temp = copy.deepcopy(RR_control)
        RR_control[0], RR_control[2], RR_control[4] = self.spline_fun_RR(RR_control_temp, step)
        # RR_control[1], RR_control[3], RR_control[5] = controller.spline_fun_RR(RR_control_temp, controller.horizon+step)

        best_control_parameters[0 : self.num_control_parameters_single_leg] = FL_control
        best_control_parameters[self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2] = (
            FR_control
        )
        best_control_parameters[
            self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
        ] = RL_control
        best_control_parameters[
            self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
        ] = RR_control

        return best_control_parameters

    def prepare_state_and_reference(
        self, state_current, reference_state, current_contact, previous_contact, mpc_frequency=100
    ):
        """
        This function jaxify the current state and reference for further processing.
        """

        # Shift the previous solution ahead
        if config.mpc_params['shift_solution']:
            index_shift = 1.0 / mpc_frequency
            self.best_control_parameters = self.shift_solution(self.best_control_parameters, index_shift)

        state_current_jax = np.concatenate(
            (
                state_current["position"],
                state_current["linear_velocity"],
                state_current["orientation"],
                state_current["angular_velocity"],
                state_current["foot_FL"],
                state_current["foot_FR"],
                state_current["foot_RL"],
                state_current["foot_RR"],
            )
        ).reshape((24,))

        if current_contact[0] == 0.0:
            state_current_jax[12:15] = reference_state["ref_foot_FL"].reshape((3,))
        if current_contact[1] == 0.0:
            state_current_jax[15:18] = reference_state["ref_foot_FR"].reshape((3,))
        if current_contact[2] == 0.0:
            state_current_jax[18:21] = reference_state["ref_foot_RL"].reshape((3,))
        if current_contact[3] == 0.0:
            state_current_jax[21:24] = reference_state["ref_foot_RR"].reshape((3,))

        reference_state_jax = np.concatenate(
            (
                reference_state["ref_position"],
                reference_state["ref_linear_velocity"],
                reference_state["ref_orientation"],
                reference_state["ref_angular_velocity"],
                reference_state["ref_foot_FL"].reshape((3,)),
                reference_state["ref_foot_FR"].reshape((3,)),
                reference_state["ref_foot_RL"].reshape((3,)),
                reference_state["ref_foot_RR"].reshape((3,)),
            )
        ).reshape((24,))

        self.best_control_parameters = np.array(self.best_control_parameters)

        if previous_contact[0] == 1 and current_contact[0] == 0:
            self.best_control_parameters[0 : self.num_control_parameters_single_leg] = 0.0
        if previous_contact[1] == 1 and current_contact[1] == 0:
            self.best_control_parameters[
                self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2
            ] = 0.0
        if previous_contact[2] == 1 and current_contact[2] == 0:
            self.best_control_parameters[
                self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
            ] = 0.0
        if previous_contact[3] == 1 and current_contact[3] == 0:
            self.best_control_parameters[
                self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
            ] = 0.0

        return state_current_jax, reference_state_jax

    def compute_control_random_sampling(
        self,
        state,
        reference,
        contact_sequence,
        best_control_parameters,
        key,
        timing,
        nominal_step_frequency,
        optimize_swing,
    ):
        """
        This function computes the control parameters by sampling from a Gaussian and a uniform distribution.
        """

        # Generate random parameters

        # The first control parameters is the old best one, so we add zero noise there
        additional_random_parameters = self.initial_random_parameters * 0.0

        # FIRST GAUSSIAN
        num_sample_gaussian_1 = (1 + int(self.num_parallel_computations / 3)) - 1
        sigma_gaussian_1 = self.sigma_random_sampling[0]
        additional_random_parameters = additional_random_parameters.at[
            1 : 1 + int(self.num_parallel_computations / 3)
        ].set(sigma_gaussian_1 * jax.random.normal(key=key, shape=(num_sample_gaussian_1, self.num_control_parameters)))

        # SECOND GAUSSIAN
        num_sample_gaussian_2 = (1 + int(self.num_parallel_computations / 3) * 2) - (
            1 + int(self.num_parallel_computations / 3)
        )
        sigma_gaussian_2 = self.sigma_random_sampling[1]
        additional_random_parameters = additional_random_parameters.at[
            1 + int(self.num_parallel_computations / 3) : 1 + int(self.num_parallel_computations / 3) * 2
        ].set(sigma_gaussian_2 * jax.random.normal(key=key, shape=(num_sample_gaussian_2, self.num_control_parameters)))

        # UNIFORM
        max_sampling_forces = self.sigma_random_sampling[2]
        num_samples_uniform = int(self.num_parallel_computations) - (1 + int(self.num_parallel_computations / 3) * 2)
        additional_random_parameters = additional_random_parameters.at[
            1 + int(self.num_parallel_computations / 3) * 2 : int(self.num_parallel_computations)
        ].set(
            jax.random.uniform(
                key=key,
                minval=-max_sampling_forces,
                maxval=max_sampling_forces,
                shape=(num_samples_uniform, self.num_control_parameters),
            )
        )

        # Add sampling to the best old control parameters
        control_parameters_vec = best_control_parameters + additional_random_parameters

        # Sampling step frequency
        available_freq_increment = jnp.where(optimize_swing, self.step_freq_delta, nominal_step_frequency)
        # available_freq_increment = self.step_freq_delta

        # step_frequencies_vec = jax.random.choice(key, available_freq_increment, shape=(self.num_parallel_computations, ))*optimize_swing + nominal_step_frequency
        step_frequencies_vec = jax.random.choice(key, available_freq_increment, shape=(self.num_parallel_computations,))

        # Do rollout
        costs = self.jit_vectorized_rollout(state, reference, timing, control_parameters_vec, step_frequencies_vec)

        # Saturate the cost in case of NaN or inf
        costs = jnp.where(jnp.isnan(costs), 1000000, costs)
        costs = jnp.where(jnp.isinf(costs), 1000000, costs)

        # Take the best found control parameters
        best_index = jnp.nanargmin(costs)
        best_cost = costs.take(best_index)
        best_control_parameters = control_parameters_vec[best_index]
        best_step_frequency = step_frequencies_vec[best_index]

        # and redistribute it to each leg
        best_control_parameters_FL = best_control_parameters[0 : self.num_control_parameters_single_leg]
        best_control_parameters_FR = best_control_parameters[
            self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2
        ]
        best_control_parameters_RL = best_control_parameters[
            self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
        ]
        best_control_parameters_RR = best_control_parameters[
            self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
        ]

        # Compute the GRF associated to the best parameter
        fx_FL, fy_FL, fz_FL = self.spline_fun_FL(best_control_parameters_FL, 0.0, 1)
        fx_FR, fy_FR, fz_FR = self.spline_fun_FR(best_control_parameters_FR, 0.0, 1)
        fx_RL, fy_RL, fz_RL = self.spline_fun_RL(best_control_parameters_RL, 0.0, 1)
        fx_RR, fy_RR, fz_RR = self.spline_fun_RR(best_control_parameters_RR, 0.0, 1)

        # Add the gravity compensation to the stance legs and put to zero
        # the GRF of the swing legs
        number_of_legs_in_stance = (
            contact_sequence[0][0] + contact_sequence[1][0] + contact_sequence[2][0] + contact_sequence[3][0]
        )
        reference_force_stance_legs = (self.robot.mass * 9.81) / number_of_legs_in_stance

        fz_FL = reference_force_stance_legs + fz_FL
        fz_FR = reference_force_stance_legs + fz_FR
        fz_RL = reference_force_stance_legs + fz_RL
        fz_RR = reference_force_stance_legs + fz_RR

        fx_FL = fx_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FL = fy_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FL = fz_FL * contact_sequence[0][0] 

        fx_FR = fx_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FR = fy_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FR = fz_FR * contact_sequence[1][0]

        fx_RL = fx_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RL = fy_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RL = fz_RL * contact_sequence[2][0]

        fx_RR = fx_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RR = fy_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RR = fz_RR * contact_sequence[3][0]

        # Enforce force constraints
        fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR = (
            self.enforce_force_constraints(
                fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR
            )
        )

        nmpc_GRFs = jnp.array([fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR])
        nmpc_footholds = jnp.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # Compute predicted state for IK
        input = jnp.array(
            [
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                fx_FL,
                fy_FL,
                fz_FL,
                fx_FR,
                fy_FR,
                fz_FR,
                fx_RL,
                fy_RL,
                fz_RL,
                fx_RR,
                fy_RR,
                fz_RR,
            ],
            dtype=dtype_general,
        )
        current_contact = jnp.array(
            [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]],
            dtype=dtype_general,
        )
        nmpc_predicted_state = self.robot.integrate_jax(state, input, current_contact, 0)

        return (
            nmpc_GRFs,
            nmpc_footholds,
            nmpc_predicted_state,
            best_control_parameters,
            best_cost,
            best_step_frequency,
            costs,
        )

    def compute_control_mppi(
        self,
        state,
        reference,
        contact_sequence,
        best_control_parameters,
        key,
        timing,
        nominal_step_frequency,
        optimize_swing,
    ):
        """
        This function computes the control parameters by applying MPPI.
        """

        # The first control parameters is the old best one, so we add zero noise there
        additional_random_parameters = self.initial_random_parameters * 0.0

        # GAUSSIAN
        num_sample_gaussian_1 = self.num_parallel_computations - 1
        additional_random_parameters = additional_random_parameters.at[1 : self.num_parallel_computations].set(
            self.sigma_mppi * jax.random.normal(key=key, shape=(num_sample_gaussian_1, self.num_control_parameters))
        )

        control_parameters_vec = best_control_parameters + additional_random_parameters

        # Sampling step frequency
        available_freq_increment = self.step_freq_delta
        step_frequencies_vec = jax.random.choice(
            key, available_freq_increment, shape=(self.num_parallel_computations,)
        )  # *optimize_swing + nominal_step_frequency

        # Do rollout
        costs = self.jit_vectorized_rollout(state, reference, timing, control_parameters_vec, step_frequencies_vec)

        # Saturate the cost in case of NaN or inf
        costs = jnp.where(jnp.isnan(costs), 1000000, costs)
        costs = jnp.where(jnp.isinf(costs), 1000000, costs)

        # Take the best found control parameters
        best_index = jnp.nanargmin(costs)
        best_cost = costs.take(best_index)

        # Compute MPPI update
        beta = best_cost
        temperature = 1.0
        exp_costs = jnp.exp((-1.0 / temperature) * (costs - beta))
        denom = np.sum(exp_costs)
        weights = exp_costs / denom
        weighted_inputs = weights[:, jnp.newaxis, jnp.newaxis] * additional_random_parameters.reshape(
            (self.num_parallel_computations, self.num_control_parameters, 1)
        )
        best_control_parameters += jnp.sum(weighted_inputs, axis=0).reshape((self.num_control_parameters,))
        best_step_frequency = step_frequencies_vec[best_index]

        # And redistribute it to each leg
        best_control_parameters_FL = best_control_parameters[0 : self.num_control_parameters_single_leg]
        best_control_parameters_FR = best_control_parameters[
            self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2
        ]
        best_control_parameters_RL = best_control_parameters[
            self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
        ]
        best_control_parameters_RR = best_control_parameters[
            self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
        ]

        # Compute the GRF associated to the best parameter
        fx_FL, fy_FL, fz_FL = self.spline_fun_FL(best_control_parameters_FL, 0.0, 1)
        fx_FR, fy_FR, fz_FR = self.spline_fun_FR(best_control_parameters_FR, 0.0, 1)
        fx_RL, fy_RL, fz_RL = self.spline_fun_RL(best_control_parameters_RL, 0.0, 1)
        fx_RR, fy_RR, fz_RR = self.spline_fun_RR(best_control_parameters_RR, 0.0, 1)

        # Add the gravity compensation to the stance legs and put to zero
        # the GRF of the swing legs
        number_of_legs_in_stance = (
            contact_sequence[0][0] + contact_sequence[1][0] + contact_sequence[2][0] + contact_sequence[3][0]
        )
        reference_force_stance_legs = (self.robot.mass * 9.81) / number_of_legs_in_stance

        fz_FL = reference_force_stance_legs + fz_FL
        fz_FR = reference_force_stance_legs + fz_FR
        fz_RL = reference_force_stance_legs + fz_RL
        fz_RR = reference_force_stance_legs + fz_RR

        fx_FL = fx_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FL = fy_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FL = fz_FL * contact_sequence[0][0] 

        fx_FR = fx_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FR = fy_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FR = fz_FR * contact_sequence[1][0]

        fx_RL = fx_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RL = fy_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RL = fz_RL * contact_sequence[2][0]

        fx_RR = fx_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RR = fy_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RR = fz_RR * contact_sequence[3][0]

        # Enforce force constraints
        fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR = (
            self.enforce_force_constraints(
                fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR
            )
        )

        nmpc_GRFs = jnp.array([fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR])
        nmpc_footholds = jnp.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # Compute predicted state for IK
        input = jnp.array(
            [
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                fx_FL,
                fy_FL,
                fz_FL,
                fx_FR,
                fy_FR,
                fz_FR,
                fx_RL,
                fy_RL,
                fz_RL,
                fx_RR,
                fy_RR,
                fz_RR,
            ],
            dtype=dtype_general,
        )
        current_contact = jnp.array(
            [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]],
            dtype=dtype_general,
        )
        nmpc_predicted_state = self.robot.integrate_jax(state, input, current_contact, 0)

        return (
            nmpc_GRFs,
            nmpc_footholds,
            nmpc_predicted_state,
            best_control_parameters,
            best_cost,
            best_step_frequency,
            costs,
        )

    def compute_control_cem_mppi(
        self,
        state,
        reference,
        contact_sequence,
        best_control_parameters,
        key,
        sigma,
        timing,
        nominal_step_frequency,
        optimize_swing,
    ):
        """
        This function computes the control parameters by applying CEM-MPPI.
        """

        # Generate random parameters
        # The first control parameters is the old best one, so we add zero noise there
        additional_random_parameters = self.initial_random_parameters * 0.0

        # GAUSSIAN
        num_sample_gaussian_1 = self.num_parallel_computations - 1

        # jax.debug.breakpoint()
        additional_random_parameters = additional_random_parameters.at[1 : self.num_parallel_computations].set(
            jax.random.normal(key=key, shape=(num_sample_gaussian_1, self.num_control_parameters)) * sigma
        )

        control_parameters_vec = best_control_parameters + additional_random_parameters

        # Sampling step frequency
        available_freq_increment = jnp.array([0.0, 0.2, 0.4])
        step_frequencies_vec = (
            jax.random.choice(key, available_freq_increment, shape=(self.num_parallel_computations,)) * optimize_swing
            + nominal_step_frequency
        )

        # Do rollout
        costs = self.jit_vectorized_rollout(state, reference, timing, control_parameters_vec, step_frequencies_vec)

        # Saturate the cost in case of NaN or inf
        costs = jnp.where(jnp.isnan(costs), 1000000, costs)
        costs = jnp.where(jnp.isinf(costs), 1000000, costs)

        # Take the best found control parameters
        best_index = jnp.nanargmin(costs)
        best_cost = costs.take(best_index)

        # Compute MPPI update
        beta = best_cost
        temperature = 1.0
        exp_costs = jnp.exp((-1.0 / temperature) * (costs - beta))
        denom = np.sum(exp_costs)
        weights = exp_costs / denom
        weighted_inputs = weights[:, jnp.newaxis, jnp.newaxis] * additional_random_parameters.reshape(
            (self.num_parallel_computations, self.num_control_parameters, 1)
        )
        best_control_parameters += jnp.sum(weighted_inputs, axis=0).reshape((self.num_control_parameters,))
        best_step_frequency = step_frequencies_vec[best_index]

        # And redistribute it to each leg
        best_control_parameters_FL = best_control_parameters[0 : self.num_control_parameters_single_leg]
        best_control_parameters_FR = best_control_parameters[
            self.num_control_parameters_single_leg : self.num_control_parameters_single_leg * 2
        ]
        best_control_parameters_RL = best_control_parameters[
            self.num_control_parameters_single_leg * 2 : self.num_control_parameters_single_leg * 3
        ]
        best_control_parameters_RR = best_control_parameters[
            self.num_control_parameters_single_leg * 3 : self.num_control_parameters_single_leg * 4
        ]

        # Compute the GRF associated to the best parameter
        fx_FL, fy_FL, fz_FL = self.spline_fun_FL(best_control_parameters_FL, 0.0, 1)
        fx_FR, fy_FR, fz_FR = self.spline_fun_FR(best_control_parameters_FR, 0.0, 1)
        fx_RL, fy_RL, fz_RL = self.spline_fun_RL(best_control_parameters_RL, 0.0, 1)
        fx_RR, fy_RR, fz_RR = self.spline_fun_RR(best_control_parameters_RR, 0.0, 1)

        # Add the gravity compensation to the stance legs and put to zero
        # the GRF of the swing legs
        number_of_legs_in_stance = (
            contact_sequence[0][0] + contact_sequence[1][0] + contact_sequence[2][0] + contact_sequence[3][0]
        )
        reference_force_stance_legs = (self.robot.mass * 9.81) / number_of_legs_in_stance

        fz_FL = reference_force_stance_legs + fz_FL
        fz_FR = reference_force_stance_legs + fz_FR
        fz_RL = reference_force_stance_legs + fz_RL
        fz_RR = reference_force_stance_legs + fz_RR

        fx_FL = fx_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FL = fy_FL * contact_sequence[0][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FL = fz_FL * contact_sequence[0][0] 

        fx_FR = fx_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_FR = fy_FR * contact_sequence[1][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_FR = fz_FR * contact_sequence[1][0]

        fx_RL = fx_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RL = fy_RL * contact_sequence[2][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RL = fz_RL * contact_sequence[2][0]

        fx_RR = fx_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_x)
        fy_RR = fy_RR * contact_sequence[3][0] / (self.max_sampling_forces_z/self.max_sampling_forces_y)
        fz_RR = fz_RR * contact_sequence[3][0]

        # Enforce force constraints
        fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR = (
            self.enforce_force_constraints(
                fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR
            )
        )

        nmpc_GRFs = jnp.array([fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR])
        nmpc_footholds = jnp.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # Compute predicted state for IK
        input = jnp.array(
            [
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                jnp.float32(0),
                fx_FL,
                fy_FL,
                fz_FL,
                fx_FR,
                fy_FR,
                fz_FR,
                fx_RL,
                fy_RL,
                fz_RL,
                fx_RR,
                fy_RR,
                fz_RR,
            ],
            dtype=dtype_general,
        )
        current_contact = jnp.array(
            [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]],
            dtype=dtype_general,
        )
        nmpc_predicted_state = self.robot.integrate_jax(state, input, current_contact, 0)

        indexes = jnp.argsort(costs)[:200]
        elite = additional_random_parameters[indexes]
        new_sigma_cem_mppi = jnp.cov(elite, rowvar=False) + np.eye(self.num_control_parameters) * 1e-8
        new_sigma_cem_mppi = jnp.diag(new_sigma_cem_mppi)

        new_sigma_cem_mppi = jnp.where(new_sigma_cem_mppi > 3, 3, new_sigma_cem_mppi)
        new_sigma_cem_mppi = jnp.where(new_sigma_cem_mppi < 0.5, 0.5, new_sigma_cem_mppi)

        return (
            nmpc_GRFs,
            nmpc_footholds,
            nmpc_predicted_state,
            best_control_parameters,
            best_cost,
            costs,
            new_sigma_cem_mppi,
        )

    def reset(self):
        print("Resetting the controller")
