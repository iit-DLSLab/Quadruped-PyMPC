"""Checks of the OT-MPC sampling method (CPU, small number of samples)."""
import unittest
from unittest.mock import patch

import jax
import jax.numpy as jnp
import numpy as np
from numpy.testing import assert_allclose

from quadruped_pympc import config as cfg
from tests.test_feedback_mppi import _problem


class OTMPCTests(unittest.TestCase):
    def make_mpc(self, **kwargs):
        params = dict(type='sampling', sampling_method='ot_mpc', optimize_step_freq=False, device='cpu',
                      num_parallel_computations=128, gradient_refinement_steps=0, ot_mpc_num_particles=8)
        params.update(kwargs)
        with patch.dict(cfg.mpc_params, params):
            from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC
            return Sampling_MPC()

    def test_sinkhorn_marginals(self):
        mpc = self.make_mpc(ot_mpc_sinkhorn_iterations=500)
        rng = np.random.default_rng(0)
        log_q = jnp.log(jnp.full(5, 0.2))
        log_p = jax.nn.log_softmax(jnp.array(rng.normal(size=12)))
        cost = jnp.array(rng.uniform(size=(5, 12)))
        coupling = jnp.exp(mpc.sinkhorn(log_q, log_p, cost, 0.1))
        assert_allclose(coupling.sum(axis=1), jnp.exp(log_q), atol=1e-5)
        assert_allclose(coupling.sum(axis=0), jnp.exp(log_p), atol=1e-5)

    def test_large_epsilon_recovers_mppi_average(self):
        # With epsilon -> inf the coupling is q p^T and every barycenter is the importance-weighted average
        mpc = self.make_mpc()
        rng = np.random.default_rng(1)
        log_q = jnp.log(jnp.full(4, 0.25))
        log_p = jax.nn.log_softmax(jnp.array(rng.normal(size=20)))
        proposals = jnp.array(rng.normal(size=(20, 3)))
        cost = jnp.array(rng.uniform(size=(4, 20)))
        barycenters = jax.nn.softmax(mpc.sinkhorn(log_q, log_p, cost, 1e6), axis=1) @ proposals
        assert_allclose(barycenters, jnp.tile(jnp.exp(log_p) @ proposals, (4, 1)), atol=1e-5)

    def test_compute_control(self):
        mpc = self.make_mpc()
        state, reference, contact_sequence = _problem(mpc)
        out = mpc.jitted_compute_control(state, reference, contact_sequence, mpc.particles,
                                         jax.random.PRNGKey(0), np.ones(4) * 0.5, 1.4, 0)
        grfs, parameters, best_cost, particles = out[0], out[3], out[4], out[-1]
        self.assertEqual(particles.shape, mpc.particles.shape)
        assert_allclose(mpc.compute_first_stage_grfs(parameters, contact_sequence), grfs, atol=1e-4)
        # The applied solution is the lowest-cost particle
        particle_costs = mpc.vectorized_rollout(state, reference, particles, contact_sequence)
        assert_allclose(best_cost, particle_costs.min(), rtol=1e-4)
        assert_allclose(parameters, particles[jnp.argmin(particle_costs)], atol=1e-5)

    def test_iterations_decrease_cost(self):
        mpc = self.make_mpc()
        state, reference, contact_sequence = _problem(mpc)
        initial_cost = mpc.vectorized_rollout(state, reference, mpc.particles, contact_sequence).min()
        particles, key = mpc.particles, jax.random.PRNGKey(0)
        for _ in range(10):
            key, subkey = jax.random.split(key)
            out = mpc.jitted_compute_control(state, reference, contact_sequence, particles, subkey,
                                             np.ones(4) * 0.5, 1.4, 0)
            particles = out[-1]
        self.assertLess(float(out[4]), float(initial_cost))

    def test_shift_particles(self):
        mpc = self.make_mpc()
        shifted = mpc.shift_solution(mpc.particles, 2.0)
        self.assertEqual(shifted.shape, mpc.particles.shape)
        assert_allclose(shifted[3], mpc.shift_solution(mpc.particles[3], 2.0), atol=1e-5)


if __name__ == '__main__':
    unittest.main()
