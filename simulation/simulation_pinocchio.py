# Description: This script is used to simulate the full model of the robot in mujoco

# Authors: Giulio Turrisi -

# TODO

import pinocchio as pin
#from pinocchio import casadi as cpin
import numpy as np 

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../')
sys.path.append(dir_path + '/../helpers/')
sys.path.append(dir_path + '/../helpers/swing_generators/')
sys.path.append(dir_path + '/../gradient/')
sys.path.append(dir_path + '/../gradient/input_rates/')
sys.path.append(dir_path + '/../gradient/nominal/')
sys.path.append(dir_path + '/../sampling/')

# Parameters for both MPC and simulation
import config

# You should change here to set up your own URDF file or just pass it as an argument of this example.
#urdf_filename = dir_path + '/robot_model/aliengo/aliengo.urdf"
if(config.robot == 'go2'):
    urdf_filename = dir_path + '/robot_model/unitree_go2/"go2.urdf' 
elif(config.robot == 'aliengo'):
    urdf_filename = dir_path + '/robot_model/aliengo/aliengo.urdf'
elif(config.robot == 'hyqreal'):
    urdf_filename = dir_path + '/robot_model/hyqreal/hyqreal.urdf'
else:
    print("Robot not found")
    exit()

# Load the urdf model
robot_full = pin.buildModelFromUrdf(urdf_filename)
#initial_config = robot_full.q0

# Sample a random configuration
#initial_config = pin.randomConfiguration(robot_full)
if(config.robot == 'go2'):
    initial_config = np.array([0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])
if(config.robot == 'aliengo'):
    initial_config = np.array([0, 0, 0, 0, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])
if(config.robot == 'hyqreal'):
    initial_config = np.array([0.0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])

# Create a list of joints to lock
jointsToLock = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']

# Get the ID of all existing joints
jointsToLockIDs = []
for jn in jointsToLock:
    print("jn", jn)
    if robot_full.existJointName(jn):
        jointsToLockIDs.append(robot_full.getJointId(jn))
    else:
        print('Warning: joint ' + str(jn) + ' does not belong to the model!')

robot_reduced = pin.buildReducedModel(robot_full, jointsToLockIDs, initial_config)
print("Reduded model: ", robot_reduced)



#centroidal = robot_reduced.centroidal(robot_reduced.q0, robot_reduced.v0)
#Ig = centroidal[2] #centroidal rigid inertia
#mass = Ig.mass
#inertia = Ig.inertia
mass = robot_reduced.inertias[0].mass
inertia = robot_reduced.inertias[0].inertia


breakpoint()
