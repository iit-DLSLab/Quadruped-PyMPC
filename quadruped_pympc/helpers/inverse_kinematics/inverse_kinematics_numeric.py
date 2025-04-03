import numpy as np

np.set_printoptions(precision=3, suppress=True)
from numpy.linalg import norm, solve
import time

import casadi as cs

# import example_robot_data as robex
import copy

# Mujoco magic
import mujoco
import mujoco.viewer
from adam import Representations

# Adam and Liecasadi magic
from adam.casadi import KinDynComputations
from liecasadi import SO3

import gym_quadruped
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
gym_quadruped_path = os.path.dirname(gym_quadruped.__file__)


from quadruped_pympc import config


# Class for solving a generic inverse kinematics problem
class InverseKinematicsNumeric:
    def __init__(self) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:

        """

        if config.robot == 'go2':
            urdf_filename = gym_quadruped_path + '/robot_model/go2/go2.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/go2/go2.xml'
        if config.robot == 'go1':
            urdf_filename = gym_quadruped_path + '/robot_model/go1/go1.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/go1/go1.xml'
        elif config.robot == 'aliengo':
            urdf_filename = gym_quadruped_path + '/robot_model/aliengo/aliengo.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/aliengo/aliengo.xml'
        elif config.robot == 'b2':
            urdf_filename = gym_quadruped_path + '/robot_model/b2/b2.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/b2/b2.xml'
        elif config.robot == 'hyqreal':
            urdf_filename = gym_quadruped_path + '/robot_model/hyqreal/hyqreal.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/hyqreal/hyqreal.xml'
        elif config.robot == 'mini_cheetah':
            urdf_filename = gym_quadruped_path + '/robot_model/mini_cheetah/mini_cheetah.urdf'
            xml_filename = gym_quadruped_path + '/robot_model/mini_cheetah/mini_cheetah.xml'

        joint_list = [
            'FL_hip_joint',
            'FL_thigh_joint',
            'FL_calf_joint',
            'FR_hip_joint',
            'FR_thigh_joint',
            'FR_calf_joint',
            'RL_hip_joint',
            'RL_thigh_joint',
            'RL_calf_joint',
            'RR_hip_joint',
            'RR_thigh_joint',
            'RR_calf_joint',
        ]

        self.kindyn = KinDynComputations(urdfstring=urdf_filename, joints_name_list=joint_list)
        self.kindyn.set_frame_velocity_representation(representation=Representations.MIXED_REPRESENTATION)

        self.forward_kinematics_FL_fun = self.kindyn.forward_kinematics_fun("FL_foot")
        self.forward_kinematics_FR_fun = self.kindyn.forward_kinematics_fun("FR_foot")
        self.forward_kinematics_RL_fun = self.kindyn.forward_kinematics_fun("RL_foot")
        self.forward_kinematics_RR_fun = self.kindyn.forward_kinematics_fun("RR_foot")

        self.jacobian_FL_fun = self.kindyn.jacobian_fun("FL_foot")
        self.jacobian_FR_fun = self.kindyn.jacobian_fun("FR_foot")
        self.jacobian_RL_fun = self.kindyn.jacobian_fun("RL_foot")
        self.jacobian_RR_fun = self.kindyn.jacobian_fun("RR_foot")

        q = cs.SX.sym('q', 12 + 7)
        FL_foot_target_position = cs.SX.sym('FL_foot_target_position', 3)
        FR_foot_target_position = cs.SX.sym('FR_foot_target_position', 3)
        RL_foot_target_position = cs.SX.sym('RL_foot_target_position', 3)
        RR_foot_target_position = cs.SX.sym('RR_foot_target_position', 3)
        ik = self.compute_solution(
            q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position
        )
        self.fun_compute_solution = cs.Function(
            'fun_ik',
            [q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position],
            [ik],
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

        eps = 1e-2
        IT_MAX = 5
        DT = 1e-2
        damp = 1e-2
        damp_matrix = damp * np.eye(12)

        i = 0

        err_FL = cs.SX.zeros(3, 1)
        err_FR = cs.SX.zeros(3, 1)
        err_RL = cs.SX.zeros(3, 1)
        err_RR = cs.SX.zeros(3, 1)
        err = cs.SX.zeros(3)

        q_joint = q[7:]
        quaternion = q[3:7]
        quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        R = SO3.from_quat(quaternion).as_matrix()
        # H = cs.DM.eye(4)
        H = cs.SX.eye(4)

        H[0:3, 0:3] = R
        H[0:3, 3] = q[0:3]

        while i <= IT_MAX:
            FL_foot_actual_pos = self.forward_kinematics_FL_fun(H, q_joint)[0:3, 3]
            FR_foot_actual_pos = self.forward_kinematics_FR_fun(H, q_joint)[0:3, 3]
            RL_foot_actual_pos = self.forward_kinematics_RL_fun(H, q_joint)[0:3, 3]
            RR_foot_actual_pos = self.forward_kinematics_RR_fun(H, q_joint)[0:3, 3]

            err_FL = FL_foot_target_position - FL_foot_actual_pos
            err_FR = FR_foot_target_position - FR_foot_actual_pos
            err_RL = RL_foot_target_position - RL_foot_actual_pos
            err_RR = RR_foot_target_position - RR_foot_actual_pos

            err = err_FL + err_FR + err_RL + err_RR
            norm_err = cs.norm_2(err)
            # if(norm_err < eps):
            #    success = True
            #    break

            J_FL = self.jacobian_FL_fun(H, q_joint)[0:3, 6:]
            J_FR = self.jacobian_FR_fun(H, q_joint)[0:3, 6:]
            J_RL = self.jacobian_RL_fun(H, q_joint)[0:3, 6:]
            J_RR = self.jacobian_RR_fun(H, q_joint)[0:3, 6:]

            total_jac = cs.vertcat(J_FL, J_FR, J_RL, J_RR)
            total_err = 100.0 * cs.vertcat(err_FL, err_FR, err_RL, err_RR)
            damped_pinv = cs.inv(total_jac.T @ total_jac + damp_matrix) @ total_jac.T
            v = damped_pinv @ total_err
            q_joint = q_joint + DT * v

            i += 1

        return q_joint


if __name__ == "__main__":
    if config.robot == 'go2':
        urdf_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/go2/go2.urdf'
        xml_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/go2/go2.xml'
    if config.robot == 'go1':
        urdf_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/go1/go1.urdf'
        xml_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/go1/go1.xml'
    elif config.robot == 'aliengo':
        urdf_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.urdf'
        xml_filename = dir_path + '/../../gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.xml'
    elif config.robot == 'hyqreal':
        urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.urdf'
        xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.xml'
    elif config.robot == 'mini_cheetah':
        urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.urdf'
        xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.xml'

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

    mujoco.mj_step(m, d)

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
    quaternion = d.qpos[3:7]
    quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
    R = SO3.from_quat(quaternion).as_matrix()
    H = cs.DM.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = d.qpos[0:3]
    print("FL foot start position", ik.forward_kinematics_FL_fun(H, initial_q[7:])[0:3, 3])
    print("FR foot start position", ik.forward_kinematics_FR_fun(H, initial_q[7:])[0:3, 3])
    print("RL foot start position", ik.forward_kinematics_RL_fun(H, initial_q[7:])[0:3, 3])
    print("RR foot start position", ik.forward_kinematics_RR_fun(H, initial_q[7:])[0:3, 3])

    initial_time = time.time()
    solution = ik.fun_compute_solution(
        initial_q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position
    )
    print("time: ", time.time() - initial_time)

    print("\n")
    print("MUJOCO SOLUTION")
    foot_position_FL = d.geom_xpos[FL_id]
    foot_position_FR = d.geom_xpos[FR_id]
    foot_position_RL = d.geom_xpos[RL_id]
    foot_position_RR = d.geom_xpos[RR_id]
    print("joints: ", d.qpos[7:])
    print("FL foot position: ", foot_position_FL)
    print("FR foot position: ", foot_position_FR)
    print("RL foot position:  ", foot_position_RL)
    print("RR foot position: ", foot_position_RR)

    print("\n")
    print("ADAM SOLUTION")
    print("joints: ", solution)
    quaternion = d.qpos[3:7]
    quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
    R = SO3.from_quat(quaternion).as_matrix()
    H = cs.SX.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = d.qpos[0:3]
    print("FL foot position", ik.forward_kinematics_FL_fun(H, solution)[0:3, 3])
    print("FR foot position", ik.forward_kinematics_FR_fun(H, solution)[0:3, 3])
    print("RL foot position", ik.forward_kinematics_RL_fun(H, solution)[0:3, 3])
    print("RR foot position", ik.forward_kinematics_RR_fun(H, solution)[0:3, 3])

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while True:
            viewer.sync()
