from __future__ import annotations

import itertools
import os
import time
import warnings
from collections import namedtuple
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces
from mujoco import MjData, MjModel
from scipy.spatial.transform import Rotation

from utils.math_utils import homogenous_transform
from utils.mujoco_utils.visual import render_vector
from utils.quadruped_utils import LegsAttr, extract_mj_joint_info

BASE_OBS = ['base_pos', 'base_lin_vel', 'base_ang_vel', 'base_ori_euler_xyz', 'base_ori_quat_wxyz', 'base_ori_SO3']
GEN_CORDS_OBS = ['qpos', 'qvel', 'tau_applied', 'qpos_js', 'qvel_js']
FEET_OBS = ['feet_pos', 'feet_pos_base', 'feet_vel', 'feet_vel_base']
ALL_OBS = BASE_OBS + GEN_CORDS_OBS + FEET_OBS


class QuadrupedEnv(gym.Env):
    """A simple quadruped environment for testing model-based controllers and imitation learning algorithms.

    To deal with different quadruped robots, which might have different joint naming and ordering conventions, this
    environment uses the `LegsAttr` dataclass to store attributes associated with the legs of a quadruped robot. This
    dataclass uses the naming convention FL, FR, RL, RR to represent the Front-Left, Front-Right, Rear-Right, and
    Rear-Left legs, respectively.
    """

    metadata = {'render.modes': ['human'], 'version': 0}

    def __init__(self,
                 robot: str,
                 legs_joint_names: LegsAttr,  # Joint names associated to each of the four legs
                 scene='flat',
                 sim_dt=0.002,
                 base_vel_command_type: str = 'forward',
                 base_lin_vel_range: tuple[float, float] = (0.28, 0.3),
                 base_ang_vel_range: tuple[float, float] = (-np.pi * 3 / 4, np.pi * 3 / 4),
                 feet_geom_name: LegsAttr = LegsAttr(FL='FL', FR='FR', RL='RL', RR='RR'),
                 state_obs_names: list[str] = ('qpos', 'qvel', 'tau_applied', 'feet_pos_base', 'feet_vel_base'),
                 legs_order: tuple[str] = ('FL', 'FR', 'RL', 'RR'),
                 ):
        """Initialize the quadruped environment.

        Args:
        ----
            robot: (str) The name of the robot model to be used.
            legs_joint_names: (LegsAttr) The joint names associated with each of the four legs.
            scene: (str) The scene to be used in the simulation.
            sim_dt: (float) The simulation time step.
            base_vel_command_type: (str) The type of base velocity command. Either 'forward' or 'random_orientation'.
            base_vel_range: (tuple) The range of the base velocity command.
            feet_geom_name: (LegsAttr) The name of the geometry associated with each of the four feet.
            feet_body_name: (LegsAttr) The name of the body associated with each of the four feet.
            state_obs_names: (list) The names of the state observations to be included in the observation space.
        """
        super(QuadrupedEnv, self).__init__()

        self.base_vel_command_type = base_vel_command_type
        self.base_lin_vel_range, self.base_ang_vel_range = base_lin_vel_range, base_ang_vel_range
        self.legs_order = legs_order

        # Define the model and data
        dir_path = os.path.dirname(os.path.realpath(__file__))
        base_path = Path(dir_path) / 'robot_model' / robot
        model_file_path = base_path / f'scene_{scene}.xml'
        assert model_file_path.exists(), f"Model file not found: {model_file_path.absolute().resolve()}"

        # Load the robot and scene to mujoco
        try:
            self.mjModel: MjModel = mujoco.MjModel.from_xml_path(str(model_file_path))
        except ValueError as e:
            raise ValueError(f"Error loading the scene {model_file_path}:") from e
        self.mjData: MjData = mujoco.MjData(self.mjModel)
        # Set the simulation step size (dt)
        self.mjModel.opt.timestep = sim_dt

        # Identify the legs DoF indices/address in the qpos and qvel arrays ___________________________________________
        assert legs_joint_names is not None, "Please provide the joint names associated with each of the four legs."
        self.joint_info = extract_mj_joint_info(self.mjModel)
        self.legs_qpos_idx = LegsAttr(None, None, None, None)  # Indices of legs joints in qpos vector
        self.legs_qvel_idx = LegsAttr(None, None, None, None)  # Indices of legs joints in qvel vector
        self.legs_tau_idx = LegsAttr(None, None, None, None)  # Indices of legs actuators in gen forces vector
        # Ensure the joint names of the robot's legs' joints are in the model. And store the qpos and qvel indices
        for leg_name in ["FR", "FL", "RR", "RL"]:
            qpos_idx, qvel_idx, tau_idx = [], [], []
            leg_joints = legs_joint_names[leg_name]
            for joint_name in leg_joints:
                assert joint_name in self.joint_info, f"Joint {joint_name} not found in {list(self.joint_info.keys())}"
                qpos_idx.extend(self.joint_info[joint_name].qpos_idx)
                qvel_idx.extend(self.joint_info[joint_name].qvel_idx)
                tau_idx.extend(self.joint_info[joint_name].tau_idx)
            self.legs_qpos_idx[leg_name] = qpos_idx
            self.legs_qvel_idx[leg_name] = qvel_idx
            self.legs_tau_idx[leg_name] = tau_idx

        # Action space: Torque values for each joint _________________________________________________________________
        tau_low, tau_high = self.mjModel.actuator_forcerange[:, 0], self.mjModel.actuator_forcerange[:, 1]
        is_act_lim = [np.inf if not lim else 1.0 for lim in self.mjModel.actuator_forcelimited]
        self.action_space = spaces.Box(
            shape=(self.mjModel.nu,),
            low=np.asarray([tau if not lim else -np.inf for tau, lim in zip(tau_low, is_act_lim)]),
            high=np.asarray([tau if not lim else np.inf for tau, lim in zip(tau_high, is_act_lim)]),
            dtype=np.float32)

        # Observation space: _______________________________________
        # Get the Env observation gym.Space, and a dict with the indices of each observation in the state vector
        self.observation_space, self.state_obs_idx = self._configure_observation_space(state_obs_names)
        self.state_obs_names = state_obs_names

        # Check the provided feet geometry and body names are within the model. These are used to compute the Jacobians
        # of the feet positions.
        self._feet_geom_id = LegsAttr(None, None, None, None)
        self._feet_body_id = LegsAttr(None, None, None, None)
        _all_geoms = [mujoco.mj_id2name(self.mjModel, i, mujoco.mjtObj.mjOBJ_GEOM) for i in range(self.mjModel.ngeom)]
        for lef_name in ["FR", "FL", "RR", "RL"]:
            foot_geom_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_GEOM, feet_geom_name[lef_name])
            assert foot_geom_id != -1, f"Foot GEOM {feet_geom_name[lef_name]} not found in {_all_geoms}."
            self._feet_geom_id[lef_name] = foot_geom_id
            foot_body_id, foot_body_name = self._get_geom_body_info(geom_id=foot_geom_id)
            self._feet_body_id[lef_name] = foot_body_id

        self.viewer = None
        self.step_num = 0
        # Reference base velocity in "Horizontal" frame (see heading_orientation_SO3)
        self._ref_base_lin_vel_H, self._base_ang_yaw_dot = None, None
        # Store the ids of visual aid geometries
        self._geom_ids = {}

    def step(self, action) -> tuple[np.ndarray, float, bool, bool, dict]:
        """Apply the action to the robot, evolve the simulation, and return the observation, reward, and termination.

        Args:
        ----
            action: (np.ndarray) The desired joint-space torques to apply to each of the robot's DoF actuators.

        Returns:
        -------
            np.ndarray: The state observation.
            float: The reward.
            bool: Whether the episode is terminated.
            bool: Whether the episode is truncated.
            dict: Additional information.
        """
        # Apply action (torque) to the robot
        self.mjData.ctrl = action
        mujoco.mj_step(self.mjModel, self.mjData)

        # Get observation
        obs = self._get_obs()

        # Compute reward (simplified)
        reward = self._compute_reward()

        # Check if done (simplified, usually more complex)
        invalid_contact, contact_info = self._check_for_invalid_contacts()
        is_terminated = invalid_contact  # and ...
        is_truncated = False
        # Info dictionary
        info = dict(time=self.mjData.time, step_num=self.step_num, invalid_contacts=contact_info)

        self.step_num += 1
        return obs, reward, is_terminated, is_truncated, info

    def reset(self,
              qpos: np.ndarray = None,
              qvel: np.ndarray = None,
              seed: int | None = None,
              options: dict[str, Any] | None = None) -> np.ndarray:
        """Reset the environment.

        Args:
        ----
            qpos: (np.ndarray) Initial joint positions. If None, random initialization around keyframe 0.
            qvel: (np.ndarray) Initial joint velocities. If None, random initialization around keyframe 0.
            seed: (int) Seed for reproducibility.
            options: (dict) Additional options for the reset.

        Returns:
        -------
            np.ndarray: The initial observation.
        """
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
            # Random orientation
            ori_xyzw = Rotation.from_euler('xyz',
                                           [np.random.uniform(-5 * np.pi / 180, 5 * np.pi / 180),
                                            np.random.uniform(-5 * np.pi / 180, 5 * np.pi / 180),
                                            np.random.uniform(-np.pi, np.pi)]).as_quat(canonical=True)
            ori_wxyz = np.roll(ori_xyzw, 1)
            self.mjData.qpos[3:7] = ori_wxyz
            # Random xy position withing a 2 x 2 square
            self.mjData.qpos[0:2] = np.random.uniform(-2, 2, 2)

            # Perform a forward dynamics computation to update the contact information
            mujoco.mj_step1(self.mjModel, self.mjData)
            # Check if the robot is in contact with the ground
            contact_state, contacts = self.feet_contact_state
            while np.any(contact_state.to_list()):
                all_contacts = list(itertools.chain(*contacts.to_list()))
                max_penetration_distance = np.max([np.abs(contact.dist) for contact in all_contacts])
                self.mjData.qpos[2] += max_penetration_distance * 1.1
                mujoco.mj_step1(self.mjModel, self.mjData)
                contact_state, contacts = self.feet_contact_state
        else:
            self.mjData.qpos = qpos
            self.mjData.qvel = qvel

        # Reset the accelerations to zero
        self.mjData.qacc[:] = 0
        self.mjData.qacc_warmstart[:] = 0
        # This ensures all registers/arrays are updated
        mujoco.mj_step(self.mjModel, self.mjData)

        # Reset the desired base velocity command
        # ----------------------------------------------------------------------
        if 'forward' in self.base_vel_command_type:
            base_vel_norm = np.random.uniform(*self.base_lin_vel_range)
            base_heading_vel_vec = np.array([1, 0, 0])  # Move in the "forward" direction
        elif 'random' in self.base_vel_command_type:
            base_vel_norm = np.random.uniform(*self.base_lin_vel_range)
            heading_angle = np.random.uniform(-np.pi, np.pi)
            base_heading_vel_vec = np.array([np.cos(heading_angle), np.sin(heading_angle), 0])
        else:
            raise ValueError(f"Invalid base linear velocity command type: {self.base_vel_command_type}")

        if 'rotate' in self.base_vel_command_type:
            self._base_ang_yaw_dot = np.random.uniform(*self.base_ang_vel_range)
        else:
            self._base_ang_yaw_dot = 0.0

        self._ref_base_lin_vel_H = base_vel_norm * base_heading_vel_vec

        return self._get_obs()

    def render(self, mode='human', key_callback: mujoco.viewer.KeyCallbackType = None):
        X_B = self.base_configuration
        r_B = X_B[:3, 3]
        dr_B = self.mjData.qvel[0:3]
        ref_base_lin_vel_B, _ = self.target_base_vel()
        ref_vec_pos, vec_pos = r_B + [0, 0, 0.1], r_B + [0, 0, 0.15]
        ref_vel_vec_color, vel_vec_color = np.array([1, 0.5, 0, .7]), np.array([0, 1, 1, .7])
        ref_vec_scale, vec_scale = np.linalg.norm(ref_base_lin_vel_B) / 1.0, np.linalg.norm(dr_B) / 1.0

        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(
                self.mjModel, self.mjData, show_left_ui=False, show_right_ui=False, key_callback=key_callback,
                )
            mujoco.mjv_defaultFreeCamera(self.mjModel, self.viewer.cam)
            # self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            # self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
            # Define markers for visualization of desired and current base velocity
            self._geom_ids['ref_dr_B_vec'] = render_vector(
                self.viewer, vector=ref_base_lin_vel_B, pos=ref_vec_pos, scale=ref_vec_scale, color=ref_vel_vec_color
                )
            self._geom_ids['dr_B_vec'] = render_vector(
                self.viewer, vector=dr_B, pos=vec_pos, scale=vec_scale, color=vel_vec_color,
                )
        else:
            # Update the reference and current base velocity markers
            render_vector(self.viewer, ref_base_lin_vel_B, ref_vec_pos, ref_vec_scale, ref_vel_vec_color,
                          geom_id=self._geom_ids['ref_dr_B_vec'])
            render_vector(self.viewer, dr_B, vec_pos, vec_scale, vel_vec_color,
                          geom_id=self._geom_ids['dr_B_vec'])

        self._update_camera_target(self.viewer.cam, r_B)
        self.viewer.sync()

    def close(self):
        """Close the viewer."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def hip_positions(self, frame='world') -> LegsAttr:
        """Get the hip positions in the specified frame.

        Args:
        ----
            frame:  Either 'world' or 'base'. The reference frame in which the hip positions are computed.

        Returns:
        -------
            LegsAttr: A dictionary-like object with:
                - FR: (3,) position of the FR hip in the specified frame.
                - FL: (3,) position of the FL hip in the specified frame.
                - RR: (3,) position of the RR hip in the specified frame.
                - RL: (3,) position of the RL hip in the specified frame.
        """
        if frame == 'world':
            R = np.eye(3)
        elif frame == 'base':
            R = self.base_configuration[0:3, 0:3]
        else:
            raise ValueError(f"Invalid frame: {frame} != 'world' or 'base'")
        # TODO: this should not be hardcoded.
        FL_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
        FR_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
        RL_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
        RR_hip_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')
        return LegsAttr(
            FR=R.T @ self.mjData.body(FR_hip_id).xpos,
            FL=R.T @ self.mjData.body(FL_hip_id).xpos,
            RR=R.T @ self.mjData.body(RR_hip_id).xpos,
            RL=R.T @ self.mjData.body(RL_hip_id).xpos,
            )

    def target_base_vel(self):
        """Returns the target base linear (3,) and angular (3,) velocity in the world reference frame."""
        if self._ref_base_lin_vel_H is None:
            raise RuntimeError("Please call env.reset() before accessing the target base velocity.")
        R_B_heading = self.heading_orientation_SO3
        ref_base_lin_vel = (R_B_heading @ self._ref_base_lin_vel_H.reshape(3, 1)).squeeze()
        ref_base_ang_vel = np.array([0., 0., self._base_ang_yaw_dot])
        return ref_base_lin_vel, ref_base_ang_vel

    def get_base_inertia(self) -> np.ndarray:
        """Returns the reflected rotational inertia of the robot's base at the current configuration in world frame.

        Args:
        ----
            model: The MuJoCo model.
            data: The MuJoCo data.

        Returns:
        -------
            np.ndarray: The reflected rotational inertia matrix in the world frame.
        """
        # Initialize the full mass matrix
        mass_matrix = np.zeros((self.mjModel.nv, self.mjModel.nv))
        mujoco.mj_fullM(self.mjModel, mass_matrix, self.mjData.qM)

        # Extract the 3x3 rotational inertia matrix of the base (assuming the base has 6 DoFs)
        inertia_B_at_qpos = mass_matrix[3:6, 3:6]

        return inertia_B_at_qpos

    def feet_pos(self, frame='world') -> LegsAttr:
        """Get the feet positions in the specified frame.

        Args:
        ----
            frame: Either 'world' or 'base'. The reference frame in which the feet positions are computed.

        Returns:
        -------
            LegsAttr: A dictionary-like object with:
                - FR: (3,) position of the FR foot in the specified frame.
                - FL: (3,) position of the FL foot in the specified frame.
                - RR: (3,) position of the RR foot in the specified frame.
                - RL: (3,) position of the RL foot in the specified frame.
        """
        if frame == 'world':
            X = np.eye(4)
        elif frame == 'base':
            X = self.base_configuration
        else:
            raise ValueError(f"Invalid frame: {frame} != 'world' or 'base'")

        return LegsAttr(
            FR=homogenous_transform(self.mjData.geom_xpos[self._feet_geom_id.FR], X.T),
            FL=homogenous_transform(self.mjData.geom_xpos[self._feet_geom_id.FL], X.T),
            RR=homogenous_transform(self.mjData.geom_xpos[self._feet_geom_id.RR], X.T),
            RL=homogenous_transform(self.mjData.geom_xpos[self._feet_geom_id.RL], X.T),
            )

    # LegsAttr(**{leg_name: feet_jac[leg_name] @ env.mjData.qvel for leg_name in legs_order})
    def feet_vel(self, frame='world') -> LegsAttr:
        """Get the feet velocities in the specified frame.

        The feet velocities are computed using the Jacobians of the feet positions and the joint velocities.
        Args:
        ----
            frame: Either 'world' or 'base'. The reference frame in which the feet velocities are computed.

        Returns:
        -------
            LegsAttr: A dictionary-like object with:
                - FR: (3,) velocity of the FR foot in the specified frame.
                - FL: (3,) velocity of the FL foot in the specified frame.
                - RR: (3,) velocity of the RR foot in the specified frame.
                - RL: (3,) velocity of the RL foot in the specified frame.
        """
        feet_jac = self.feet_jacobians(frame=frame)
        return LegsAttr(**{leg_name: feet_jac[leg_name] @ self.mjData.qvel for leg_name in self.legs_order})

    def feet_jacobians(self, frame: str = 'world', return_rot_jac: bool = False) -> LegsAttr | tuple[LegsAttr, ...]:
        """Compute the Jacobians of the feet positions.

        This function computes the translational and rotational Jacobians of the feet positions. Each feet position is
        defined as the position of the geometry corresponding to each foot, passed in the `feet_geom_name` argument of
        the constructor. The body to which each feet point/geometry is attached to is assumed to be the one passed in
        the `feet_body_name` argument of the constructor.

        The Jacobians returned can be used to compute the relationship between joint velocities and feet velocities,
        such that if r_dot_FL is the velocity of the FL foot in the world frame, then:
        r_dot_FL = J_FL @ qvel, where J_FL in R^{3 x mjModel.nv} is the Jacobian of the FL foot position.

        Args:
        ----
            frame: Either 'world' or 'base'. The reference frame in which the Jacobians are computed.
            return_rot_jac: Whether to compute the rotational Jacobians. If False, only the translational Jacobians
                are computed.

        Returns:
        -------
            If `return_rot_jac` is False:
            LegsAttr: A dictionary-like object with:
                - FR: (3, mjModel.nv) Jacobian of the FR foot position in the specified frame.
                - FL: (3, mjModel.nv) Jacobian of the FL foot position in the specified frame.
                - RR: (3, mjModel.nv) Jacobian of the RR foot position in the specified frame.
                - RL: (3, mjModel.nv) Jacobian of the RL foot position in the specified frame.
            If `return_rot_jac` is True:
            tuple: A tuple with two LegsAttr objects:
                - The first LegsAttr object contains the translational Jacobians as described above.
                - The second LegsAttr object contains the rotational Jacobians.
        """
        if frame == 'world':
            R = np.eye(3)
        elif frame == 'base':
            R = self.base_configuration[0:3, 0:3]
        else:
            raise ValueError(f"Invalid frame: {frame} != 'world' or 'base'")
        feet_trans_jac = LegsAttr(*[np.zeros((3, self.mjModel.nv)) for _ in range(4)])
        feet_rot_jac = LegsAttr(*[np.zeros((3, self.mjModel.nv)) if not return_rot_jac else None for _ in range(4)])
        feet_pos = self.feet_pos(frame='world')  # Mujoco mj_jac expects the point in global coordinates.

        for leg_name in ["FR", "FL", "RR", "RL"]:
            mujoco.mj_jac(m=self.mjModel,
                          d=self.mjData,
                          jacp=feet_trans_jac[leg_name],
                          jacr=feet_rot_jac[leg_name],
                          point=feet_pos[leg_name],  # Point in global coordinates
                          body=self._feet_body_id[leg_name]  # Body to which `point` is attached to.
                          )
            feet_trans_jac[leg_name] = R.T @ feet_trans_jac[leg_name]
            if return_rot_jac:
                feet_rot_jac[leg_name] = R.T @ feet_rot_jac[leg_name]

        return feet_trans_jac if not return_rot_jac else (feet_trans_jac, feet_rot_jac)

    @property
    def feet_contact_state(self) -> [LegsAttr, LegsAttr]:
        """Returns the boolean contact state of the feet.

        This function considers only contacts between the feet and the ground.

        Returns
        -------
            LegsAttr: A dictionary-like object with:
                - FL: (bool) True if the FL foot is in contact with the ground.
                - FR: (bool) True if the FR foot is in contact with the ground.
                - RL: (bool) True if the RL foot is in contact with the ground.
                - RR: (bool) True if the RR foot is in contact with the ground.
            LegsAttr: A dictionary-like object with:
                - FL: list[MjContact] A list of contact objects associated with the FL foot.
                - FR: list[MjContact] A list of contact objects associated with the FR foot.
                - RL: list[MjContact] A list of contact objects associated with the RL foot.
                - RR: list[MjContact] A list of contact objects associated with the RR foot.
        """
        contact_state = LegsAttr(FL=False, FR=False, RL=False, RR=False)
        legs_contacts = LegsAttr(FL=[], FR=[], RL=[], RR=[])
        for contact in self.mjData.contact:
            # Get body IDs from geom IDs
            body1_id = self.mjModel.geom_bodyid[contact.geom1]
            body2_id = self.mjModel.geom_bodyid[contact.geom2]

            if 0 in [body1_id, body2_id]:  # World body ID is 0
                second_id = body2_id if body1_id == 0 else body1_id
                if second_id in self._feet_body_id.to_list():  # Check if contact occurs with anything but the feet
                    for leg_name in ["FL", "FR", "RL", "RR"]:
                        if second_id == self._feet_body_id[leg_name]:
                            contact_state[leg_name] = True
                            legs_contacts[leg_name].append(contact)

        return contact_state, legs_contacts

    @property
    def base_configuration(self):
        """Robot base configuration (homogenous transformation matrix) in world reference frame."""
        com_pos = self.mjData.qpos[0:3]  # world frame
        quat_wxyz = self.mjData.qpos[3:7]  # world frame (wxyz) mujoco convention
        quat_xyzw = np.roll(quat_wxyz, -1)  # SciPy convention (xyzw)
        X_B = np.eye(4)
        X_B[0:3, 0:3] = Rotation.from_quat(quat_xyzw).as_matrix()
        X_B[0:3, 3] = com_pos
        return X_B

    @property
    def joint_space_state(self):
        """Returns the joint-space state (qpos, qvel) of the robot."""
        return self.mjData.qpos[7:], self.mjData.qvel[6:]

    @property
    def base_pos(self):
        """Returns the base position (3,) in the world reference frame."""
        return self.mjData.qpos[0:3]

    @property
    def base_lin_vel(self):
        """Returns the base linear velocity (3,) in the world reference frame."""
        return self.mjData.qvel[0:3]

    @property
    def base_ang_vel(self):
        """Returns the base angular velocity (3,) in the world reference frame."""
        return self.mjData.qvel[3:6]

    @property
    def base_ori_euler_xyz(self):
        """Returns the base orientation in Euler XYZ angles (roll, pitch, yaw) in the world reference frame."""
        quat_wxyz = self.mjData.qpos[3:7]
        quat_xyzw = np.roll(quat_wxyz, -1)
        return Rotation.from_quat(quat_xyzw).as_euler('xyz')

    @property
    def heading_orientation_SO3(self):
        """Returns a SO(3) matrix that aligns with the robot's base heading orientation and the world z axis."""
        X_B = self.base_configuration
        R_B = X_B[0:3, 0:3]
        euler_xyz = Rotation.from_matrix(R_B).as_euler('xyz')
        # Rotation aligned with the base orientation and the vertical axis
        R_B_horizontal = Rotation.from_euler('xyz', euler_xyz * [0, 0, 1]).as_matrix()
        return R_B_horizontal

    @property
    def applied_joint_torques(self):
        """Returns the true joint torques (self.mjModel.nu) used in evolving the physics simulation.

        Differs from action if actuator hasnon-ideal dynamics.
        """
        return self.mjData.qfrc_applied[6:]

    @property
    def simulation_dt(self):
        """Returns the simulation dt in seconds."""
        return self.mjModel.opt.timestep

    @property
    def simulation_time(self):
        """Returns the simulation time in seconds."""
        return self.mjData.time

    def extract_obs_from_state(self, state_like_array: np.ndarray) -> namedtuple:
        """ Extracts the state observation from a state-like array.

        Parameters
        ----------
        state_like_array : np.ndarray
            The state-like array to extract the state observation from. The shape of
            the array should be (..., self.observation_space.shape).

        Returns
        -------
        namedtuple
            A namedtuple with the entries corresponding to each of the state observation
            names, and their corresponding (..., obs_name_dimension) arrays.

        Example
        -------
        >>> env = QuadrupedEnv(state_obs_names=['base_pos', 'qpos_js'])
        >>> state = np.random.rand(env.observation_space.shape[0])
        >>> state_obs = env.extract_obs_from_state(state)
        >>> print(state_obs.base_pos)
        >>> print(state_obs.qpos_js)
        """
        state_obs_shape = self.observation_space.shape
        assert state_like_array.shape[-len(state_obs_shape):] == self.observation_space.shape, \
            f"Invalid state vector shape: {state_like_array.shape}, expected: (...{self.observation_space.shape})"
        StateObs = namedtuple('StateObs', self.state_obs_names)
        return StateObs(*[state_like_array[..., idx] for idx in self.state_obs_idx.values()])

    def _compute_reward(self):
        # Example reward function (to be defined based on the task)
        # Reward could be based on distance traveled, energy efficiency, etc.
        return 0

    def _check_for_invalid_contacts(self) -> [bool, dict]:
        """Env termination occurs when a contact is detected on the robot's base."""
        invalid_contacts = {}
        invalid_contact_detected = False
        for contact in self.mjData.contact:
            # Get body IDs from geom IDs
            body1_id = self.mjModel.geom_bodyid[contact.geom1]
            body2_id = self.mjModel.geom_bodyid[contact.geom2]

            if 0 in [body1_id, body2_id]:  # World body ID is 0
                second_id = body2_id if body1_id == 0 else body1_id
                if second_id not in self._feet_body_id.to_list():  # Check if contact occurs with anything but the feet
                    # Get body names from body IDs
                    body1_name = mujoco.mj_id2name(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, body1_id)
                    body2_name = mujoco.mj_id2name(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, body2_id)
                    invalid_contacts[f"{body1_name}:{body1_id}_{body2_name}:{body2_id}"] = contact
                    invalid_contact_detected = True
            else:  # Contact between two bodies of the robot
                pass  # Do nothing for now

        return invalid_contact_detected, invalid_contacts  # No invalid contact detected

    def _get_geom_body_info(self, geom_name: str = None, geom_id: int = None) -> [int, str]:
        """Returns the body ID and name associated with the geometry name or ID."""
        assert geom_name is not None or geom_id is not None, "Please provide either the geometry name or ID."
        if geom_name is not None:
            geom_id = mujoco.mj_name2id(self.mjModel, mujoco.mjtObj.mjOBJ_GEOM, geom_name)

        body_id = self.mjModel.geom_bodyid[geom_id]
        body_name = mujoco.mj_id2name(self.mjModel, mujoco.mjtObj.mjOBJ_BODY, body_id)

        return body_id, body_name

    # Function to update the camera position
    def _update_camera_target(self, cam, target_point: np.ndarray):
        cam.lookat[:] = target_point  # Update the camera lookat point to the target point
        # Potentially do other fancy stuff.
        pass

    def _get_obs(self):
        """Returns the state observation based on the specified state observation names."""
        obs = []
        for obs_name in self.state_obs_names:
            # Generalized position, velocity, and force (torque) spaces
            if obs_name == 'qpos':
                obs.append(self.mjData.qpos)
            elif obs_name == 'qvel':
                obs.append(self.mjData.qvel)
            elif obs_name == 'tau_applied':
                obs.append(self.applied_joint_torques)
            # Joint-space position and velocity spaces
            elif obs_name == 'qpos_js':
                obs.append(self.mjData.qpos[7:])
            elif obs_name == 'qvel_js':
                obs.append(self.mjData.qvel[6:])
            # Base position and velocity configurations (in world frame)
            elif obs_name == 'base_pos':
                obs.append(self.base_pos)
            elif obs_name == 'base_lin_vel':
                obs.append(self.base_lin_vel)
            elif obs_name == 'base_ang_vel':
                obs.append(self.base_ang_vel)
            elif obs_name == 'base_ori_euler_xyz':
                obs.append(self.base_ori_euler_xyz)
            elif obs_name == 'base_ori_quat_wxyz':
                obs.append(self.mjData.qpos[3:7])
            elif obs_name == 'base_ori_SO3':
                obs.append(self.base_configuration[0:3, 0:3].flatten())
            # Feet positions and velocities
            elif 'feet_pos' in obs_name:  # feet_pos:frame := feet_pos:world or feet_pos:base
                frame = 'world' if 'base' not in obs_name else 'base'
                obs.append(np.concatenate(self.feet_pos(frame).to_list(order=self.legs_order), axis=0))
            elif 'feet_vel' in obs_name:  # feet_vel:frame := feet_vel:world or feet_vel:base
                frame = 'world' if 'base' not in obs_name else 'base'
                obs.append(np.concatenate(self.feet_vel(frame).to_list(order=self.legs_order), axis=0))
            elif obs_name == 'contact_state':
                contact_state, _ = self.feet_contact_state
                obs.append(np.array(contact_state.to_list()))
            else:
                raise ValueError(f"Invalid observation name: {obs_name}, available obs: {ALL_OBS}")

        state_obs = np.concatenate(obs, axis=0)
        assert state_obs.shape == self.observation_space.shape, \
            f"Invalid state observation shape: {state_obs.shape}, expected: {self.observation_space.shape}"
        return state_obs

    def _configure_observation_space(self, state_obs_names: list[str]) -> [spaces.Space, dict[str, slice]]:
        """ Configures the observation space for the environment based on the provided state observation names.

           Args:
               state_obs_names (list[str]): A list of state observation names based on which the observation space is
               configured.

           Returns:
               gym.Space: The environment state observation space.
               dict: A dictionary mapping each state observation name to its indices in the observation space.
       """
        obs_dim, last_idx = 0, 0

        obs_lim_min, obs_lim_max = [], []
        qpos_lim_min, qpos_lim_max = self.mjModel.jnt_range[:, 0], self.mjModel.jnt_range[:, 1]
        tau_lim_min, tau_lim_max = self.mjModel.actuator_ctrlrange[:, 0], self.mjModel.actuator_ctrlrange[:, 1]

        obs_idx = {k: None for k in state_obs_names}
        for obs_name in state_obs_names:
            # Generalized position, velocity, and force (torque) spaces
            if obs_name == 'qpos':
                obs_dim += self.mjModel.nq
                obs_lim_max.extend([np.inf] * 7 + qpos_lim_max[1:].tolist())  # Ignore the base position
                obs_lim_min.extend([-np.inf] * 7 + qpos_lim_min[1:].tolist())  # Ignore the base position
            elif obs_name == 'qvel':
                obs_dim += self.mjModel.nv
                obs_lim_max.extend([np.inf] * self.mjModel.nv)
                obs_lim_min.extend([-np.inf] * self.mjModel.nv)
            elif obs_name == 'tau_applied':
                obs_dim += self.mjModel.nu
                obs_lim_max.extend(tau_lim_max)
                obs_lim_min.extend(tau_lim_min)
            # Joint-space position and velocity spaces
            elif obs_name == 'qpos_js':  # Joint space position configuration
                obs_dim += self.mjModel.nq - 7
                obs_lim_max.extend(qpos_lim_max)
                obs_lim_min.extend(qpos_lim_min)
            elif obs_name == 'qvel_js':  # Joint space velocity configuration
                obs_dim += self.mjModel.nv - 6
                obs_lim_max.extend([np.inf] * (self.mjModel.nv - 6))
                obs_lim_min.extend([-np.inf] * (self.mjModel.nv - 6))
            # Base position and velocity configurations (in world frame)
            elif obs_name == 'base_pos':
                if "qpos" in state_obs_names:
                    warnings.warn("base_pos is redundant with additional obs qpos. base_pos = qpos[0:3]")
                obs_dim += 3
                obs_lim_max = [np.inf] * 3
                obs_lim_min = [-np.inf] * 3
            elif obs_name == 'base_lin_vel':
                if "qvel" in state_obs_names:
                    warnings.warn("base_lin_vel is redundant with additional obs qvel. base_lin_vel = qvel[0:3]")
                obs_dim += 3
                obs_lim_max = [np.inf] * 3
                obs_lim_min = [-np.inf] * 3
            elif obs_name == 'base_ang_vel':
                if "qvel" in state_obs_names:
                    warnings.warn("base_ang_vel is redundant with additional obs qvel. base_ang_vel = qvel[3:6]")
                obs_dim += 3
                obs_lim_max = [np.inf] * 3
                obs_lim_min = [-np.inf] * 3
            elif obs_name == 'base_ori_euler_xyz':
                if "qpos" in state_obs_names:
                    warnings.warn(
                        "base_ori_euler_xyz is redundant with additional obs qpos. base_ori_euler_xyz = qpos[3:6]")
                obs_dim += 3
                obs_lim_max = [np.inf] * 3
                obs_lim_min = [-np.inf] * 3
            elif obs_name == 'base_ori_quat_wxyz':
                if "qpos" in state_obs_names:
                    warnings.warn(
                        "base_ori_quat_wxyz is redundant with additional obs qpos. base_ori_quat_wxyz = qpos[3:7]")
                obs_dim += 4
                obs_lim_max = [np.inf] * 4
                obs_lim_min = [-np.inf] * 4
            elif obs_name == 'base_ori_SO3':
                if "qpos" in state_obs_names:
                    warnings.warn("base_ori_SO3 is redundant with additional obs qpos. base_ori_SO3 = qpos[3:7]")
                obs_dim += 9
                obs_lim_max = [np.inf] * 9
                obs_lim_min = [-np.inf] * 9
            # Feet positions and velocities
            elif 'feet_pos' in obs_name:  # feet_pos:frame := feet_pos:world or feet_pos:base
                obs_dim += 12
                obs_lim_max.extend([np.inf] * 12)
                obs_lim_min.extend([-np.inf] * 12)
            elif 'feet_vel' in obs_name:  # feet_vel:frame := feet_vel:world or feet_vel:base
                obs_dim += 12
                obs_lim_max.extend([np.inf] * 12)
                obs_lim_min.extend([-np.inf] * 12)
            elif obs_name == 'contact_state':
                obs_dim += 4
                obs_lim_max.extend([1] * 4)
                obs_lim_min.extend([0] * 4)
            else:
                raise ValueError(f"Invalid observation name: {obs_name}, available obs: {ALL_OBS}")

            obs_idx[obs_name] = range(last_idx, obs_dim)
            last_idx = obs_dim

        obs_lim_min = np.array(obs_lim_min)
        obs_lim_max = np.array(obs_lim_max)
        observation_space = spaces.Box(low=obs_lim_min, high=obs_lim_max, shape=(obs_dim,), dtype=np.float32)
        return observation_space, obs_idx


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
