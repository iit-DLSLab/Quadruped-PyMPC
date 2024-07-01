import os

import numpy as np
import pinocchio as pin
from adam import Representations
from adam.casadi import KinDynComputations
from liecasadi import SE3

dir_path = os.path.dirname(os.path.realpath(__file__))

import sys

sys.path.append(dir_path + '/../')

# Parameters for both MPC and simulation
from quadruped_pympc import config


class SrbInertiaComputation:
    def __init__(self, ) -> None:
        if (config.robot == 'go2'):
            urdf_filename = dir_path + '/../simulation/robot_model/go2/go2.urdf'
        elif (config.robot == 'aliengo'):
            urdf_filename = dir_path + '/../simulation/robot_model/aliengo/aliengo.urdf'
        elif (config.robot == 'hyqreal'):
            urdf_filename = dir_path + '/../simulation/robot_model/hyqreal/hyqreal.urdf'
        elif (config.robot == 'mini_cheetah'):
            urdf_filename = dir_path + '/../simulation/robot_model/mini_cheetah/mini_cheetah.urdf'

        self.use_pinocchio = True

        if (self.use_pinocchio):
            self.robot_full = pin.buildModelFromUrdf(urdf_filename)

            # Create a list of joints to lock
            jointsToLock = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']

            # Get the ID of all existing joints
            self.jointsToLockIDs = []
            for jn in jointsToLock:
                # print("jn", jn)
                if self.robot_full.existJointName(jn):
                    self.jointsToLockIDs.append(self.robot_full.getJointId(jn))
                else:
                    print('Warning: joint ' + str(jn) + ' does not belong to the model!')
        else:
            self.kindyn = KinDynComputations(urdfstring=urdf_filename)
            self.kindyn.set_frame_velocity_representation(representation=Representations.BODY_FIXED_REPRESENTATION)
            self.mass_mass_fun = self.kindyn.mass_matrix_fun()

    def compute_inertia(self, q):

        if (self.use_pinocchio):
            if (config.robot == 'aliengo'):
                robot_reduced = pin.buildReducedModel(self.robot_full, self.jointsToLockIDs, q[3:])
            else:
                robot_reduced = pin.buildReducedModel(self.robot_full, self.jointsToLockIDs, q[7:])
            inertia = robot_reduced.inertias[0].inertia
            # mass = robot_reduced.inertias[0].mass
        else:
            base_pose = q[0:3]
            base_quaternion = q[3:7]
            base_quaternion_xyzw = [base_quaternion[1], base_quaternion[2], base_quaternion[3], base_quaternion[0]]
            joint_position = q[7:]

            H = SE3.from_position_quaternion(base_pose, base_quaternion_xyzw).as_matrix()
            inertia = self.mass_mass_fun(H, joint_position)[3:6, 3:6]

        return np.array(inertia)


if __name__ == "__main__":
    srb_inertia_computation = SrbInertiaComputation()
    q = np.array([0, 0, 0, 0, 0.0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])
    inertia = srb_inertia_computation.compute_inertia(q)
    print(inertia)
