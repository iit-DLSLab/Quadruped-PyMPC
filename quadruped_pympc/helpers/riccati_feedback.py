"""Close the loop on the MPC GRFs at the whole body control rate with the Riccati gain of the MPC.

GRF = GRF_mpc + K (x_now - x_mpc), see 'use_riccati_feedback' in the config.
"""

import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

_LEGS = ("FL", "FR", "RL", "RR")


def state_to_vector(state: dict) -> np.ndarray:
    """(24,) state in the same order as the MPC: com position, linear velocity, euler angles, angular velocity, feet."""
    return np.concatenate(
        (
            state["position"],
            state["linear_velocity"],
            state["orientation"],
            state["angular_velocity"],
            state["foot_FL"],
            state["foot_FR"],
            state["foot_RL"],
            state["foot_RR"],
        )
    )


def apply_riccati_feedback(
    nmpc_GRFs: LegsAttr,
    K: np.ndarray,
    x_mpc: np.ndarray,
    state_current: dict,
    current_contact: np.ndarray,
    mu: float,
    grf_min: float,
    grf_max: float,
) -> LegsAttr:
    """Correct the MPC GRFs with the state error accumulated since the MPC solution.

    Args:
        nmpc_GRFs (LegsAttr): GRFs of the last MPC solution, world frame
        K (np.ndarray): (12, 24) Riccati gain of the first stage, d(GRF)/d(state)
        x_mpc (np.ndarray): (24,) state used by the MPC for that solution
        state_current (dict): current state, as the one passed to the MPC
        current_contact (np.ndarray): current contact state of the legs
        mu (float): friction coefficient
        grf_min (float): min normal force of a leg in stance
        grf_max (float): max normal force of a leg in stance

    Returns:
        LegsAttr: corrected GRFs, zero for the legs in swing and inside the friction pyramid for the legs in stance
    """
    dx = state_to_vector(state_current) - x_mpc
    dx[8] = (dx[8] + np.pi) % (2 * np.pi) - np.pi  # yaw wrap
    # A swing foot follows its swing trajectory, which is not the foot motion of the MPC model:
    # only the stance feet error is fed back
    for leg_id in range(4):
        if current_contact[leg_id] == 0:
            dx[12 + 3 * leg_id : 15 + 3 * leg_id] = 0.0

    grf = np.concatenate([nmpc_GRFs[leg] for leg in _LEGS]) + K @ dx

    out = {}
    for leg_id, leg in enumerate(_LEGS):
        if current_contact[leg_id] == 0:
            out[leg] = np.zeros(3)
            continue
        f = grf[3 * leg_id : 3 * leg_id + 3]
        f[2] = np.clip(f[2], grf_min, grf_max)
        f[0:2] = np.clip(f[0:2], -mu * f[2], mu * f[2])
        out[leg] = f
    return LegsAttr(**out)
