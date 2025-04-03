import numpy as np

np.set_printoptions(precision=3, suppress=True)
from numpy.linalg import norm, solve
import time
import casadi as cs

# import example_robot_data as robex
import copy
import os
import time

import gym_quadruped

# Mujoco magic
import mujoco
import mujoco.viewer

# Adam and Liecasadi magic

import gym_quadruped
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
gym_quadruped_path = os.path.dirname(gym_quadruped.__file__)


from quadruped_pympc import config as cfg

from gym_quadruped.quadruped_env import QuadrupedEnv

from quadruped_pympc import config as cfg

IT_MAX = 5
DT = 1e-2
damp = 1e-4
damp_matrix = damp * np.eye(12)


# Class for solving a generic inverse kinematics problem
class InverseKinematicsNumeric:
    def __init__(self) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:

        """

        robot_name = cfg.robot
        hip_height = cfg.hip_height
        robot_leg_joints = cfg.robot_leg_joints
        robot_feet_geom_names = cfg.robot_feet_geom_names
        # scene_name = cfg.simulation_params['scene']
        # simulation_dt = cfg.simulation_params['dt']

        # Create the quadruped robot environment -----------------------------------------------------------
        self.env = QuadrupedEnv(
            robot=robot_name,
            hip_height=hip_height,
            legs_joint_names=robot_leg_joints,  # Joint names of the legs DoF
            feet_geom_name=robot_feet_geom_names,  # Geom/Frame id of feet
        )

    def compute_solution(
        self,
        q: np.ndarray,
        FL_foot_target_position: np.ndarray,
        FR_foot_target_position: np.ndarray,
        RL_foot_target_position: np.ndarray,
        RR_foot_target_position: np.ndarray,
    ) -> np.ndarray:
        """
        This method computes the forward kinematics from initial joint angles and desired foot target positions.

        Args:
            q (np.ndarray): The initial joint angles.
            FL_foot_target_position (np.ndarray): The desired position of the front-left foot.
            FR_foot_target_position (np.ndarray): The desired position of the front-right foot.
            RL_foot_target_position (np.ndarray): The desired position of the rear-left foot.
            RR_foot_target_position (np.ndarray): The desired position of the rear-right foot.

        Returns:
            np.ndarray: The joint angles that achieve the desired foot positions.
        """

        # Set the initial states
        self.env.mjData.qpos = q
        mujoco.mj_fwdPosition(self.env.mjModel, self.env.mjData)

        for j in range(IT_MAX):
            feet_pos = self.env.feet_pos(frame='world')

            FL_foot_actual_pos = feet_pos.FL
            FR_foot_actual_pos = feet_pos.FR
            RL_foot_actual_pos = feet_pos.RL
            RR_foot_actual_pos = feet_pos.RR

            err_FL = FL_foot_target_position - FL_foot_actual_pos
            err_FR = FR_foot_target_position - FR_foot_actual_pos
            err_RL = RL_foot_target_position - RL_foot_actual_pos
            err_RR = RR_foot_target_position - RR_foot_actual_pos

            # Compute feet jacobian
            feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)

            J_FL = feet_jac.FL[:, 6:]
            J_FR = feet_jac.FR[:, 6:]
            J_RL = feet_jac.RL[:, 6:]
            J_RR = feet_jac.RR[:, 6:]

            total_jac = np.vstack((J_FL, J_FR, J_RL, J_RR))
            total_err = np.hstack((err_FL, err_FR, err_RL, err_RR))
            # breakpoint()

            # Solve the IK problem
            dq = total_jac.T @ np.linalg.solve(total_jac @ total_jac.T + damp_matrix, total_err)

            # Integrate joint velocities to obtain joint positions.
            q_joint = self.env.mjData.qpos.copy()[7:]
            q_joint += dq * DT

            print("joint step", self.env.mjData.qpos)
            self.env.mjData.qpos[7:] = q_joint
            # mujoco.mj_fwdPosition(self.env.mjModel, self.env.mjData)
            # mujoco.mj_kinematics(self.env.mjModel, self.env.mjData)

        return q_joint


if __name__ == "__main__":
    if cfg.robot == 'go2':
        xml_filename = gym_quadruped_path + '/robot_model/go2/go2.xml'
    if cfg.robot == 'go1':
        xml_filename = gym_quadruped_path + '/robot_model/go1/go1.xml'
    elif cfg.robot == 'aliengo':
        xml_filename = gym_quadruped_path + '/robot_model/aliengo/aliengo.xml'
    elif cfg.robot == 'hyqreal':
        xml_filename = gym_quadruped_path + '/robot_model/hyqreal/hyqreal.xml'
    elif cfg.robot == 'mini_cheetah':
        xml_filename = gym_quadruped_path + '/robot_model/mini_cheetah/mini_cheetah.xml'

    ik = InverseKinematicsNumeric()

    # Check consistency in mujoco
    m = mujoco.MjModel.from_xml_path(xml_filename)
    d = mujoco.MjData(m)

    random_q_joint = np.random.rand(12)
    d.qpos[7:] = random_q_joint

    # random quaternion
    rand_quat = np.random.rand(4)
    rand_quat = rand_quat / np.linalg.norm(rand_quat)
    d.qpos[3:7] = rand_quat

    mujoco.mj_fwdPosition(m, d)

    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "FL")
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "FR")
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "RL")
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "RR")
    FL_foot_target_position = d.geom_xpos[FL_id]
    FR_foot_target_position = d.geom_xpos[FR_id]
    RL_foot_target_position = d.geom_xpos[RL_id]
    RR_foot_target_position = d.geom_xpos[RR_id]

    print("FL foot target position: ", FL_foot_target_position)
    print("FR foot target position: ", FR_foot_target_position)
    print("RL foot target position: ", RL_foot_target_position)
    print("RR foot target position: ", RR_foot_target_position)

    initial_q = copy.deepcopy(d.qpos)
    initial_q[7:] = np.random.rand(12)

    ik.env.mjData.qpos = initial_q
    mujoco.mj_fwdPosition(ik.env.mjModel, ik.env.mjData)
    feet = ik.env.feet_pos(frame="world")

    print("joints start position: ", initial_q)
    print("FL foot start position", feet.FL)
    print("FR foot start position", feet.FR)
    print("RL foot start position", feet.RL)
    print("RR foot start position", feet.RR)

    initial_time = time.time()
    solution = ik.compute_solution(
        initial_q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position
    )
    print("time: ", time.time() - initial_time)

    print("\n")
    print("DESIRED SOLUTION")
    foot_position_FL = d.geom_xpos[FL_id]
    foot_position_FR = d.geom_xpos[FR_id]
    foot_position_RL = d.geom_xpos[RL_id]
    foot_position_RR = d.geom_xpos[RR_id]
    print("joints desired: ", d.qpos)
    print("FL foot desired position: ", foot_position_FL)
    print("FR foot desired position: ", foot_position_FR)
    print("RL foot desired position:  ", foot_position_RL)
    print("RR foot desired position: ", foot_position_RR)

    print("\n")
    print("MUJOCO IK SOLUTION")
    ik.env.mjData.qpos[7:] = solution
    mujoco.mj_fwdPosition(ik.env.mjModel, ik.env.mjData)
    feet = ik.env.feet_pos(frame="world")

    print("joints solution: ", ik.env.mjData.qpos)
    print("FL foot solution position", feet.FL)
    print("FR foot solution position", feet.FR)
    print("RL foot solution position", feet.RL)
    print("RR foot solution position", feet.RR)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while True:
            viewer.sync()
