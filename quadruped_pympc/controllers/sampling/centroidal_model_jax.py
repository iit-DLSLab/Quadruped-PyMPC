# Description: This file contains the class Centroidal_Model that defines the
# prediction model used by the MPC

# Authors: Giulio Turrisi -

import time

import jax
import jax.numpy as jnp
from jax import jit, random

from quadruped_pympc import config

dtype_general = 'float32'
jax.config.update("jax_default_matmul_precision", "float32")


# Class that defines the prediction model of the NMPC
class Centroidal_Model_JAX:
    def __init__(self, dt, device) -> None:
        """
        This method initializes the foothold generator Centroidal_Model, which creates
        the prediction model of the NMPC.
        """

        self.dt = dt

        if device == "gpu":
            try:
                self.device = jax.devices("gpu")[0]
            except:
                self.device = jax.devices("cpu")[0]
                print("GPU not available, using CPU")
        else:
            self.device = jax.devices("cpu")[0]

        # Mass and Inertia robot dependant
        self.mass = config.mass
        self.inertia = jnp.array(config.inertia)

        # Nonuniform discretization
        if config.mpc_params['use_nonuniform_discretization']:
            time_steps_fine_grained = jnp.tile(
                config.mpc_params['dt_fine_grained'], config.mpc_params['horizon_fine_grained']
            )
            self.dts = jnp.concatenate(
                (
                    time_steps_fine_grained,
                    jnp.tile(self.dt, config.mpc_params['horizon'] - config.mpc_params['horizon_fine_grained']),
                )
            )
        else:
            self.dts = jnp.tile(self.dt, config.mpc_params["horizon"])

        # We precompute the inverse of the inertia
        self.inertia_inv = self.calculate_inverse(self.inertia)

        # State and input dimensions
        self.state_dim = 12
        self.input_dim = 24

        # this is useful to manage the fact that df depends on self.mass and self.inertia which jit does not really like
        # by compiling it here we ensure that the parameters will always stay the same which is what jit likes
        vectorized_integrate_jax = jax.vmap(self.integrate_jax, in_axes=(None, 0, None), out_axes=0)
        self.compiled_integrate_jax = jit(vectorized_integrate_jax, device=self.device)

    def calculate_inverse(self, A):
        a11 = A[0, 0]
        a12 = A[0, 1]
        a13 = A[0, 2]
        a21 = A[1, 0]
        a22 = A[1, 1]
        a23 = A[1, 2]
        a31 = A[2, 0]
        a32 = A[2, 1]
        a33 = A[2, 2]

        # Calculate the determinant DET of A
        DET = a11 * (a33 * a22 - a32 * a23) - a21 * (a33 * a12 - a32 * a13) + a31 * (a23 * a12 - a22 * a13)

        # Calculate the inverse of A
        return (
            jnp.array(
                [
                    [(a33 * a22 - a32 * a23), -(a33 * a12 - a32 * a13), (a23 * a12 - a22 * a13)],
                    [-(a33 * a21 - a31 * a23), (a33 * a11 - a31 * a13), -(a23 * a11 - a21 * a13)],
                    [(a32 * a21 - a31 * a22), -(a32 * a11 - a31 * a12), (a22 * a11 - a21 * a12)],
                ]
            )
            / DET
        )

    def fd(self, states: jnp.ndarray, inputs: jnp.ndarray, contact_status: jnp.ndarray):
        """
        This method computes the state derivative of the system.
        """

        def skew(v):
            return jnp.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

        # Extracting variables for clarity
        foot_position_fl, foot_position_fr, foot_position_rl, foot_position_rr = jnp.split(states[12:], 4)
        foot_force_fl, foot_force_fr, foot_force_rl, foot_force_rr = jnp.split(inputs[12:], 4)
        com_position = states[:3]
        stanceFL, stanceFR, stanceRL, stanceRR = contact_status[:4]

        # Compute linear_com_vel
        linear_com_vel = states[3:6]

        # Compute linear_com_acc
        temp = (
            jnp.dot(foot_force_fl, stanceFL)
            + jnp.dot(foot_force_fr, stanceFR)
            + jnp.dot(foot_force_rl, stanceRL)
            + jnp.dot(foot_force_rr, stanceRR)
        )
        gravity = jnp.array([jnp.float32(0), jnp.float32(0), jnp.float32(-9.81)])
        linear_com_acc = jnp.dot(jnp.float32(1) / self.mass, temp) + gravity

        # Compute euler_rates_base and angular_acc_base
        w = states[9:12]
        roll, pitch, yaw = states[6:9]

        conj_euler_rates = jnp.array(
            [
                [jnp.float32(1), jnp.float32(0), -jnp.sin(pitch)],
                [jnp.float32(0), jnp.cos(roll), jnp.cos(pitch) * jnp.sin(roll)],
                [jnp.float32(0), -jnp.sin(roll), jnp.cos(pitch) * jnp.cos(roll)],
            ]
        )

        temp2 = jnp.dot(skew(foot_position_fl - com_position), foot_force_fl) * stanceFL
        temp2 += jnp.dot(skew(foot_position_fr - com_position), foot_force_fr) * stanceFR
        temp2 += jnp.dot(skew(foot_position_rl - com_position), foot_force_rl) * stanceRL
        temp2 += jnp.dot(skew(foot_position_rr - com_position), foot_force_rr) * stanceRR

        euler_rates_base = jnp.dot(self.calculate_inverse(conj_euler_rates), w)

        # FINAL angular_acc_base STATE (4)
        # Z Y X rotations!
        b_R_w = jnp.array(
            [
                [jnp.cos(pitch) * jnp.cos(yaw), jnp.cos(pitch) * jnp.sin(yaw), -jnp.sin(pitch)],
                [
                    jnp.sin(roll) * jnp.sin(pitch) * jnp.cos(yaw) - jnp.cos(roll) * jnp.sin(yaw),
                    jnp.sin(roll) * jnp.sin(pitch) * jnp.sin(yaw) + jnp.cos(roll) * jnp.cos(yaw),
                    jnp.sin(roll) * jnp.cos(pitch),
                ],
                [
                    jnp.cos(roll) * jnp.sin(pitch) * jnp.cos(yaw) + jnp.sin(roll) * jnp.sin(yaw),
                    jnp.cos(roll) * jnp.sin(pitch) * jnp.sin(yaw) - jnp.sin(roll) * jnp.cos(yaw),
                    jnp.cos(roll) * jnp.cos(pitch),
                ],
            ]
        )

        angular_acc_base = -jnp.dot(self.inertia_inv, jnp.dot(skew(w), jnp.dot(self.inertia, w))) + jnp.dot(
            self.inertia_inv, jnp.dot(b_R_w, temp2)
        )

        # Returning the results
        return jnp.concatenate([linear_com_vel, linear_com_acc, euler_rates_base, angular_acc_base])

    def integrate_jax(self, state, inputs, contact_status, n):
        """
        This method computes the forward evolution of the system.
        """
        fd = self.fd(state, inputs, contact_status)

        # Simple euler!
        dt = self.dts[n]
        new_state = state[0:12] + fd * dt

        return jnp.concatenate([new_state, state[12:]])


if __name__ == "__main__":
    model_jax = Centroidal_Model_JAX(dt=jnp.float32(0.04), device="gpu")

    # all stance, friction, stance proximity
    param = jnp.array([1.0, 1.0, 1.0, 1.0, 0.6, 0.0, 0.0, 0.0, 0.0], dtype=dtype_general)

    state = jnp.array(
        [
            0.0,
            0.0,
            0.0,  # com position
            0.0,
            0.0,
            0.0,  # com velocity
            0.0,
            0.0,
            0.0,  # euler angles
            0.0,
            0.0,
            0.0,
        ],
        dtype=dtype_general,
    )  # euler rates

    input = jnp.array(
        [
            0.0,
            0.0,
            0.0,  # foot position fl
            0.0,
            0.0,
            0.0,  # foot position fr
            0.0,
            0.0,
            0.0,  # foot position rl
            0.0,
            0.0,
            0.0,  # foot position rr
            0.0,
            0.0,
            0.0,  # foot force fl
            0.0,
            0.0,
            0.0,  # foot force fr
            0.0,
            0.0,
            0.0,  # foot force rl
            0.0,
            0.0,
            0.0,
        ],
        dtype=dtype_general,
    )  # foot force rr

    # test fd
    acc = model_jax.fd(state, input, param)

    # test integrated
    print("testing SINGLE integration PYTHON")
    time_start = time.time()
    state_integrated = model_jax.integrate_jax(state, input, param)
    print("computation time: ", time.time() - time_start)

    # test compiled integrated
    print("\ntesting SINGLE integration COMPILED-FIRST TIME")
    compiled_integrated_jax_single = jit(model_jax.integrate_jax, device=model_jax.device)
    time_start = time.time()
    state_integrated = compiled_integrated_jax_single(state, input, param).block_until_ready()
    print("computation time: ", time.time() - time_start)

    print("\ntesting SINGLE integration COMPILED-SECOND TIME")
    time_start = time.time()
    state_integrated = compiled_integrated_jax_single(state, input, param).block_until_ready()
    print("computation time: ", time.time() - time_start)

    print("\ntesting MULTIPLE integration COMPILED-FIRST TIME")
    threads = 10000

    key = random.PRNGKey(42)
    input_vec = random.randint(key, (model_jax.input_dim * threads,), minval=-2, maxval=2).reshape(
        threads, model_jax.input_dim
    )

    time_start = time.time()
    model_jax.compiled_integrate_jax(state, input_vec, param).block_until_ready()
    print("computation time: ", time.time() - time_start)

    print("\ntesting MULTIPLE integration SECOND TIME")
    time_start = time.time()
    key = random.PRNGKey(50)
    input_vec = random.randint(key, (model_jax.input_dim * threads,), minval=-2, maxval=2).reshape(
        threads, model_jax.input_dim
    )
    time_start = time.time()
    model_jax.compiled_integrate_jax(state, input_vec, param).block_until_ready()
    print("computation time: ", time.time() - time_start)
