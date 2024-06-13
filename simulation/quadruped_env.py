from __future__ import annotations

import time
from collections import namedtuple
from typing import Any

import gymnasium as gym
import scipy.spatial.transform
from gymnasium import spaces
import numpy as np
import mujoco.viewer
import mujoco
from pathlib import Path
import os

from scipy.spatial.transform import Rotation

from helpers.other import render_vector

#  This will be the order convention for the legs signals.
LegsSignal = namedtuple('LegSignals', ['FR', 'FL', 'RR', 'RL'])


class QuadrupedEnv(gym.Env):
    metadata = {'render.modes': ['human'], 'version': 1}

    def __init__(self,
                 robot='mini_cheetah',
                 scene='flat',
                 sim_dt=0.002,
                 base_vel_command_type: str = 'forward',
                 base_vel_range: tuple[float, float] = (0.00, 0.8),
                 n_robots=1,
                 ):
        super(QuadrupedEnv, self).__init__()

        self.base_vel_command_type = base_vel_command_type
        self.base_vel_range = base_vel_range
        if n_robots > 1:
            raise NotImplementedError("Multiple robots not supported yet. (@Giovanny please!) ")

        # Define the model and data
        dir_path = os.path.dirname(os.path.realpath(__file__))
        base_path = Path(dir_path) / 'robot_model' / robot
        model_file_path = base_path / f'scene_{scene}.xml'
        assert model_file_path.exists(), f"Model file not found: {model_file_path.absolute().resolve()}"

        # Load the robot and scene to mujoco
        self.mjModel = mujoco.MjModel.from_xml_path(str(model_file_path))
        self.mjData = mujoco.MjData(self.mjModel)
        # Set the simulation step size (dt)
        self.mjModel.opt.timestep = sim_dt

        # Action space: Torque values for each joint
        tau_low, tau_high = self.mjModel.actuator_forcerange[:, 0], self.mjModel.actuator_forcerange[:, 1]
        is_act_lim = [np.inf if not lim else 1.0 for lim in self.mjModel.actuator_forcelimited]
        self.action_space = spaces.Box(
            shape=(self.mjModel.nu,),
            low=np.asarray([tau if not lim else -np.inf for tau, lim in zip(tau_low, is_act_lim)]),
            high=np.asarray([tau if not lim else np.inf for tau, lim in zip(tau_high, is_act_lim)]),
            dtype=np.float32)

        # Observation space: Position, velocity, orientation, and foot positions
        obs_dim = self.mjModel.nq + self.mjModel.nv
        # Get limits from the model
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        self.viewer = None
        self.step_num = 0
        self._ref_base_velocity = None
        self._geom_ids = {}

    def step(self, action) -> tuple[np.ndarray, float, bool, bool, dict]:
        # Apply action (torque) to the robot
        self.mjData.ctrl[:] = action
        mujoco.mj_step(self.mjModel, self.mjData)

        # Get observation
        obs = self._get_obs()

        # Compute reward (simplified)
        reward = self._compute_reward()

        # Check if done (simplified, usually more complex)
        is_terminated = self._is_done()
        is_truncated = False
        # Info dictionary
        info = dict(time=self.mjData.time, step_num=self.step_num)

        self.step_num += 1
        return obs, reward, is_terminated, is_truncated, info

    def reset(self, qpos=None, qvel=None, seed: int | None = None, options: dict[str, Any] | None = None):
        # Reset relevant variables
        self.step_num = 0
        self.mjData.time = 0.0
        self.mjData.ctrl = 0.0  # Reset control signals
        self.mjData.qfrc_applied = 0.0

        if seed is not None: np.random.seed(seed)  # Set seed for reproducibility

        # Reset the robot state ----------------------------------------------------------------------------------------
        if qpos is None and qvel is None:  # Random initialization around xml keyframe 0
            mujoco.mj_resetDataKeyframe(self.mjModel, self.mjData, 0)
            # Add white noise to the joint-space position and velocity
            q_pos_amp = 20 * np.pi / 180
            q_vel_amp = 0.1
            self.mjData.qpos[7:] += np.random.uniform(-q_pos_amp, q_pos_amp, self.mjModel.nq - 7)
            self.mjData.qvel[6:] += np.random.uniform(-q_vel_amp, q_vel_amp, self.mjModel.nv - 6)
        else:
            self.mjData.qpos = qpos
            self.mjData.qvel = qvel

        # Reset the desired base velocity command ----------------------------------------------------------------------
        if self.base_vel_command_type == 'forward':
            base_vel_norm = np.random.uniform(*self.base_vel_range)
            base_heading_vel_vec = np.array([1, 0, 0])  # Move in the "forward" direction
        elif self.base_vel_command_type == 'random_orientation':
            base_vel_norm = np.random.uniform(*self.base_vel_range)
            heading_angle = np.random.uniform(-np.pi, np.pi)
            base_heading_vel_vec = np.array([np.cos(heading_angle), np.sin(heading_angle), 0])
        else:
            raise ValueError(f"Invalid base velocity command type: {self.base_vel_command_type}")

        self._ref_base_velocity = base_vel_norm * base_heading_vel_vec
        return self._get_obs()

    def render(self, mode='human'):
        X_B = self.base_configuration
        r_B = X_B[:3, 3]
        dr_B = self.mjData.qvel[0:3]
        ref_dr_B = self.target_base_velocity
        R_heading = self.heading_orientation_SO3
        ref_vec_pos, vec_pos = r_B + [0, 0, 0.1], r_B + [0, 0, 0.15]
        ref_vel_vec_color, vel_vec_color = np.array([1, 0.5, 0, .7]), np.array([0, 1, 1, .7])
        ref_vec_scale, vec_scale = np.linalg.norm(ref_dr_B) / 1.0, np.linalg.norm(dr_B) / 1.0
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(
                self.mjModel, self.mjData, show_left_ui=False, show_right_ui=False
                )
            mujoco.mjv_defaultFreeCamera(self.mjModel, self.viewer.cam)
            # self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            # self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
            # Define markers for visualization of desired and current base velocity
            self._geom_ids['ref_dr_B_vec'] = render_vector(
                viewer=self.viewer, vector=ref_dr_B, pos=ref_vec_pos, scale=ref_vec_scale, color=ref_vel_vec_color
                )
            self._geom_ids['dr_B_vec'] = render_vector(
                viewer=self.viewer, vector=dr_B, pos=vec_pos, scale=vec_scale, color=vel_vec_color,
                )
        else:
            # Update the reference and current base velocity markers
            render_vector(self.viewer, ref_dr_B, ref_vec_pos, ref_vec_scale, ref_vel_vec_color,
                          geom_id=self._geom_ids['ref_dr_B_vec'])
            render_vector(self.viewer, dr_B, vec_pos, vec_scale, vel_vec_color,
                          geom_id=self._geom_ids['dr_B_vec'])
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    @property
    def hip_positions(self):
        FL_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
        FR_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
        RL_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
        RR_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')
        return LegsSignal(
            FR=self.mjData.body_xpos[FR_hip_id],
            FL=self.mjData.body_xpos[FL_hip_id],
            RR=self.mjData.body_xpos[RR_hip_id],
            RL=self.mjData.body_xpos[RL_hip_id],
            )

    @property
    def base_configuration(self):
        """Robot base configuration (homogenous transformation matrix) in world reference frame """
        com_pos = self.mjData.qpos[0:3]  # world frame
        quat_wxyz = self.mjData.qpos[3:7]  # world frame (wxyz) mujoco convention
        quat_xyzw = np.roll(quat_wxyz, -1)  # SciPy convention (xyzw)
        X_B = np.eye(4)
        X_B[0:3, 0:3] = Rotation.from_quat(quat_xyzw).as_matrix()
        X_B[0:3, 3] = com_pos
        return X_B

    @property
    def target_base_velocity(self):
        """
        Returns the target base velocity (3,) in the world reference frame.
        """
        R_B_heading = self.heading_orientation_SO3
        ref_dr_B = R_B_heading @ self._ref_base_velocity.reshape(3, 1)
        return ref_dr_B.squeeze()

    @property
    def heading_orientation_SO3(self):
        """
        Returns a SO(3) rotation matrix that aligns with the robot's base heading orientation and the world z axis.
        """
        X_B = self.base_configuration
        R_B = X_B[0:3, 0:3]
        euler_xyz = Rotation.from_matrix(R_B).as_euler('xyz')
        # Rotation aligned with the base orientation and the vertical axis
        R_B_horizontal = Rotation.from_euler('xyz', euler_xyz * [0, 0, 1]).as_matrix()
        return R_B_horizontal

    @property
    def applied_join_torques(self):
        return self.mjData.qfrc_applied

    @property
    def simulation_dt(self):
        return self.mjModel.opt.timestep

    @property
    def simulation_time(self):
        return self.mjData.time

    def _get_obs(self):
        # Observation includes position, velocity, orientation, and foot positions
        qpos = self.mjData.qpos
        qvel = self.mjData.qvel
        return np.concatenate([qpos, qvel])

    def _compute_reward(self):
        # Example reward function (to be defined based on the task)
        # Reward could be based on distance traveled, energy efficiency, etc.
        return 0

    def _is_done(self):
        # Example termination condition (to be defined based on the task)
        return False


# Example usage:
if __name__ == '__main__':
    env = QuadrupedEnv()
    sim_dt = env.simulation_dt

    obs = env.reset()
    env.render()
    run_t0 = time.time()

    for _ in range(10000):
        action = env.action_space.sample() * 50  # Sample random action
        obs, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            break

        # When rendering simulation try to approximate real-time
        if env.simulation_time > (time.time() - run_t0):
            time.sleep(max(0.0, env.simulation_time - (time.time() - run_t0) - (5 * sim_dt)))
        env.render()

    env.close()
