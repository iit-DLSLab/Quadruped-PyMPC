import numpy as np
# import example_robot_data as robex
import mujoco

# from pinocchio import casadi as cpin

import os

from utils.quadruped_utils import LegsAttr

dir_path = os.path.dirname(os.path.realpath(__file__))

import sys

sys.path.append(dir_path + '/../')

# Parameters for both MPC and simulation
import config


# Class for the generation of the reference footholds
class FootholdReferenceGenerator:
    def __init__(self, stance_time: np.float64) -> None:
        """
        This method initializes the foothold generator class, which computes
        the reference foothold for the nonlinear MPC.

        Args:
            stance_time: The user-defined time of the stance phase.
        """

        self.stance_time = stance_time

    def compute_footholds_reference(self,
                                    com_position,
                                    rpy_angles,
                                    linear_com_velocity,
                                    desired_linear_com_velocity,
                                    hips_position: LegsAttr,
                                    com_height: np.float64,
                                    lift_off_positions: LegsAttr) -> LegsAttr:
        """
        Starting from the robot hips position, com actual and desired velocities, this method
        computes the reference footholds for the nonlinear MPC.

        TODO: Docstring of assumed order of legs, Expected frames of reference, dimensions of the expected arguments
        Args:
            linear_com_velocity: The linear velocity of the center of mass.
            desired_linear_com_velocity: The desired linear velocity of the center of mass.
            hips_position: The position of the hips in the WORLD frame. The order of the legs is assumed to be
                [FL, FR, RL, RR].
            com_height: The height of the center of mass.
        """

        # we want to move the feet in the direction of the desired velocity 
        delta_vel = (self.stance_time / 2.) * desired_linear_com_velocity
        # delta_vel = (self.stance_time/2.)*linear_com_velocity

        # we perform a hip-centric step in the horizontal frame
        hip_pos_FL, hip_pos_FR, hip_pos_RL, hip_pos_RR = hips_position.to_list(order=['FL', 'FR', 'RL', 'RR'])

        yaw = rpy_angles[2]
        h_R_w = np.array([np.cos(yaw), np.sin(yaw),
                          -np.sin(yaw), np.cos(yaw)])
        h_R_w = h_R_w.reshape((2, 2))

        hip_pos_FL[0:2] = h_R_w @ (hip_pos_FL[0:2] - com_position[0:2])
        hip_pos_FR[0:2] = h_R_w @ (hip_pos_FR[0:2] - com_position[0:2])
        hip_pos_RL[0:2] = h_R_w @ (hip_pos_RL[0:2] - com_position[0:2])
        hip_pos_RR[0:2] = h_R_w @ (hip_pos_RR[0:2] - com_position[0:2])

        # enlarging the posture depending on the robot
        # TODO: Should avoid relying on access to config. This seems to be a generic function.
        if (config.robot == 'mini_cheetah'):
            hip_pos_FL[1] += 0.07
            hip_pos_FR[1] -= 0.07
            hip_pos_RL[1] += 0.07
            hip_pos_RR[1] -= 0.07
        else:
            hip_pos_FL[1] += 0.1
            hip_pos_FR[1] -= 0.1
            hip_pos_RL[1] += 0.1
            hip_pos_RR[1] -= 0.1

        # we want to compensate for the error in the velocity
        linear_com_velocity_horizontal_frame = h_R_w @ linear_com_velocity
        compensation = np.sqrt(com_height / 9.81) * (linear_com_velocity_horizontal_frame - desired_linear_com_velocity)
        compensation = np.where(compensation > 0.05, 0.05, compensation)
        compensation = np.where(compensation < -0.05, -0.05, compensation)

        # we compute the reference footholds
        footholds_reference_FL = hip_pos_FL[0:2] + delta_vel + compensation
        footholds_reference_FR = hip_pos_FR[0:2] + delta_vel + compensation
        footholds_reference_RL = hip_pos_RL[0:2] + delta_vel + compensation
        footholds_reference_RR = hip_pos_RR[0:2] + delta_vel + compensation

        # we rotate them back in the world frame
        footholds_reference_FL[0:2] = h_R_w.T @ footholds_reference_FL[0:2] + com_position[0:2]
        footholds_reference_FR[0:2] = h_R_w.T @ footholds_reference_FR[0:2] + com_position[0:2]
        footholds_reference_RL[0:2] = h_R_w.T @ footholds_reference_RL[0:2] + com_position[0:2]
        footholds_reference_RR[0:2] = h_R_w.T @ footholds_reference_RR[0:2] + com_position[0:2]

        # we should rotate them considering the terrain estimator maybe
        # or we can just do exteroceptive height adjustement...
        # for now we substract 0.02cm to have a clear touch down
        reference_foot_FL = np.array(
            [footholds_reference_FL[0], footholds_reference_FL[1], lift_off_positions.FL[2] - 0.02])
        reference_foot_FR = np.array(
            [footholds_reference_FR[0], footholds_reference_FR[1], lift_off_positions.FR[2] - 0.02])
        reference_foot_RL = np.array(
            [footholds_reference_RL[0], footholds_reference_RL[1], lift_off_positions.RL[2] - 0.02])
        reference_foot_RR = np.array(
            [footholds_reference_RR[0], footholds_reference_RR[1], lift_off_positions.RR[2] - 0.02])

        return LegsAttr(FR=reference_foot_FR, FL=reference_foot_FL, RR=reference_foot_RR, RL=reference_foot_RL)


if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path('./../simulation/unitree_go1/scene.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    FL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
    FR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
    RL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
    RR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')

    hip_pos = np.array(([d.body(FL_hip_id).xpos],
                        [d.body(FR_hip_id).xpos],
                        [d.body(RL_hip_id).xpos],
                        [d.body(RR_hip_id).xpos]))

    print("hip_pos", hip_pos)

    stance_time = 0.5
    linear_com_velocity = np.array([0.1, 0.0, 0.0])
    desired_linear_com_velocity = np.array([0.1, 0.0, 0.0])
    com_height = d.qpos[2]

    foothold_generator = FootholdReferenceGenerator(stance_time)
    footholds_reference = foothold_generator.compute_footholds_reference(linear_com_velocity[0:2],
                                                                         desired_linear_com_velocity[0:2],
                                                                         hip_pos, com_height)
    print("iniztial hip_pos: ", hip_pos)
    print("footholds_reference: ", footholds_reference)
