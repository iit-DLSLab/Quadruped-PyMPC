import pinocchio as pin
import numpy as np 
import time

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../')


# Parameters for both MPC and simulation
import config



class SrbInertiaComputation:
    def __init__(self, ) -> None:
        if(config.robot == 'go2'):
            urdf_filename = dir_path + '/../simulation/robot_model/go2/go2.urdf' 
        elif(config.robot == 'aliengo'):
            urdf_filename = dir_path + '/../simulation/robot_model/aliengo/aliengo.urdf'
        elif(config.robot == 'hyqreal'):
            urdf_filename = dir_path + '/../simulation/robot_model/hyqreal/hyqreal.urdf'
        elif(config.robot == 'mini_cheetah'):
            urdf_filename = dir_path + '/../simulation/robot_model/mini_cheetah/mini_cheetah.urdf'
        self.robot_full = pin.buildModelFromUrdf(urdf_filename)

        # Create a list of joints to lock
        jointsToLock = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 
                        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 
                        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        
        # Get the ID of all existing joints
        self.jointsToLockIDs = []
        for jn in jointsToLock:
            #print("jn", jn)
            if self.robot_full.existJointName(jn):
                self.jointsToLockIDs.append(self.robot_full.getJointId(jn))
            else:
                print('Warning: joint ' + str(jn) + ' does not belong to the model!')


    def compute_inertia(self, q) -> None:
        robot_reduced = pin.buildReducedModel(self.robot_full, self.jointsToLockIDs, q)
        #mass = robot_reduced.inertias[0].mass
        return robot_reduced.inertias[0].inertia
    

if __name__ == "__main__":
    srb_inertia_computation = SrbInertiaComputation()
    q = np.array([0, 0, 0, 0, 0.0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])
    inertia = srb_inertia_computation.compute_inertia(q)
    print(inertia)