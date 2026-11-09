"""Checks of the Feedback-MPPI gain of the sampling mpc (CPU, small number of samples)."""
import unittest
from unittest.mock import patch

import jax
import jax.numpy as jnp
import numpy as np
from numpy.testing import assert_allclose

from quadruped_pympc import config as cfg


def _problem(mpc):
    rng = np.random.default_rng(0)
    feet = np.array([[0.2, 0.15, 0.0], [0.2, -0.15, 0.0], [-0.2, 0.15, 0.0], [-0.2, -0.15, 0.0]])
    state = np.concatenate(([0.0, 0.0, 0.3], rng.normal(size=3) * 0.2, rng.normal(size=3) * 0.05,
                            rng.normal(size=3) * 0.2, feet.reshape(-1))).astype(np.float32)
    reference = np.concatenate(([0.0, 0.0, 0.3], [0.3, 0.0, 0.0], np.zeros(6), feet.reshape(-1))).astype(np.float32)
    contact_sequence = np.ones((4, mpc.horizon), dtype=np.float32)
    contact_sequence[1, :3] = 0.0  # FR in swing at the beginning
    contact_sequence[2, :3] = 0.0  # RL in swing at the beginning
    return jnp.array(state), jnp.array(reference), jnp.array(contact_sequence)


class FeedbackMPPITests(unittest.TestCase):
    def make_mpc(self, method):
        params = dict(type='sampling', sampling_method=method, optimize_step_freq=False, device='cpu',
                      num_parallel_computations=64, gradient_refinement_steps=0, use_riccati_feedback=True)
        with patch.dict(cfg.mpc_params, params):
            from quadruped_pympc.controllers.sampling.centroidal_nmpc_jax import Sampling_MPC
            return Sampling_MPC()

    def test_gain_matches_autodiff_of_mppi_update(self):
        mpc = self.make_mpc('mppi')
        state, reference, contact_sequence = _problem(mpc)
        theta = jnp.zeros(mpc.num_control_parameters)
        noise = jax.random.normal(jax.random.PRNGKey(1), (64, mpc.num_control_parameters)) * 3.0
        temperature = 50.0

        def mppi_grf(x0):
            costs = mpc.vectorized_rollout(x0, reference, theta + noise, contact_sequence)
            weights = jax.nn.softmax(-costs / temperature)
            return mpc.compute_first_stage_grfs(theta + weights @ noise, contact_sequence)

        costs = mpc.vectorized_rollout(state, reference, theta + noise, contact_sequence)
        weights = jax.nn.softmax(-costs / temperature)
        costs_gradient = jax.vmap(jax.grad(mpc.compute_rollout), in_axes=(None, None, 0, None))(
            state, reference, theta + noise, contact_sequence
        )
        gain = mpc.compute_feedback_gain(
            costs_gradient, weights, noise, temperature, theta + weights @ noise, contact_sequence
        )

        expected = jax.jacfwd(mppi_grf)(state)
        self.assertEqual(gain.shape, (12, 24))
        assert_allclose(gain, expected, rtol=1e-3, atol=1e-3 * float(jnp.abs(expected).max()))
        # Swing legs have no force, so no gain
        assert_allclose(gain[3:9], 0.0)

    def test_first_stage_grfs_match_compute_control(self):
        for method in ('mppi', 'cem_mppi'):
            with self.subTest(method=method):
                mpc = self.make_mpc(method)
                state, reference, contact_sequence = _problem(mpc)
                args = (state, reference, contact_sequence, mpc.best_control_parameters, jax.random.PRNGKey(0))
                if method == 'cem_mppi':
                    args += (mpc.sigma_cem_mppi,)
                out = mpc.jitted_compute_control(*args, np.ones(4) * 0.5, 1.4, 0)
                grfs, parameters, gain = out[0], out[3], out[-1]
                assert_allclose(mpc.compute_first_stage_grfs(parameters, contact_sequence), grfs, atol=1e-4)
                self.assertEqual(gain.shape, (12, 24))
                self.assertTrue(np.isfinite(np.array(gain)).all())


if __name__ == '__main__':
    unittest.main()
