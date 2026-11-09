import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc.helpers.riccati_feedback import apply_riccati_feedback, state_to_vector

LEGS = ("FL", "FR", "RL", "RR")


def _state(rng):
    return dict(
        position=rng.normal(size=3),
        linear_velocity=rng.normal(size=3),
        orientation=rng.normal(size=3) * 0.1,
        angular_velocity=rng.normal(size=3),
        foot_FL=rng.normal(size=3),
        foot_FR=rng.normal(size=3),
        foot_RL=rng.normal(size=3),
        foot_RR=rng.normal(size=3),
        joint_FL=np.zeros(3),
    )


def test_no_state_error_keeps_mpc_grfs():
    rng = np.random.default_rng(0)
    state = _state(rng)
    grfs = LegsAttr(**{leg: np.array([1.0, -2.0, 40.0]) for leg in LEGS})
    K = rng.normal(size=(12, 24)) * 100
    out = apply_riccati_feedback(grfs, K, state_to_vector(state), state, np.ones(4), 0.5, 0.0, 150.0)
    for leg in LEGS:
        np.testing.assert_allclose(out[leg], grfs[leg])


def test_swing_legs_friction_and_yaw_wrap():
    rng = np.random.default_rng(1)
    state = _state(rng)
    x_mpc = state_to_vector(state)
    grfs = LegsAttr(**{leg: np.array([0.0, 0.0, 40.0]) for leg in LEGS})
    contact = np.array([1, 0, 0, 1])

    # A yaw error of 2*pi is no error at all
    K = rng.normal(size=(12, 24)) * 100
    state_wrapped = dict(state, orientation=state["orientation"] + np.array([0.0, 0.0, 2 * np.pi]))
    out = apply_riccati_feedback(grfs, K, x_mpc, state_wrapped, contact, 0.5, 0.0, 150.0)
    np.testing.assert_allclose(out.FL, grfs.FL, atol=1e-9)
    np.testing.assert_array_equal(out.FR, np.zeros(3))
    np.testing.assert_array_equal(out.RL, np.zeros(3))

    # A large correction stays inside the friction pyramid and the normal force limits
    state_moved = dict(state, linear_velocity=state["linear_velocity"] + 10.0)
    out = apply_riccati_feedback(grfs, K, x_mpc, state_moved, contact, 0.5, 0.0, 150.0)
    for leg in ("FL", "RR"):
        assert 0.0 <= out[leg][2] <= 150.0
        assert np.all(np.abs(out[leg][0:2]) <= 0.5 * out[leg][2] + 1e-12)


def test_setpoint_interpolation():
    rng = np.random.default_rng(2)
    state = _state(rng)
    x_mpc = state_to_vector(state)
    x_next = x_mpc + rng.normal(size=24) * 0.1
    grfs = LegsAttr(**{leg: np.array([1.0, -2.0, 40.0]) for leg in LEGS})
    K = rng.normal(size=(12, 24)) * 100

    # A state halfway along the predicted motion is no error at alpha = 0.5
    x_half = x_mpc + 0.5 * (x_next - x_mpc)
    state_half = dict(state, position=x_half[0:3], linear_velocity=x_half[3:6], orientation=x_half[6:9],
                      angular_velocity=x_half[9:12], foot_FL=x_half[12:15], foot_FR=x_half[15:18],
                      foot_RL=x_half[18:21], foot_RR=x_half[21:24])
    out = apply_riccati_feedback(grfs, K, x_mpc, state_half, np.ones(4), 0.5, 0.0, 150.0, x_next, 0.5)
    for leg in LEGS:
        np.testing.assert_allclose(out[leg], grfs[leg], atol=1e-9)
