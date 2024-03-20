import numpy as np
from numpy.linalg import norm
import time
import unittest
import casadi as cs
#import example_robot_data as robex
import mujoco

import pinocchio as pin
#from pinocchio import casadi as cpin

class TerrainEstimator:
    def __init__(self, ) -> None:
        

        self.terrain_roll = 0
        self.terrain_pitch = 0



    def compute_terrain_estimion(self, legs_stance_status, foot_height) -> np.ndarray:
        

 



if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path('./../simulation/unitree_go1/scene.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    FL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
    FR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
    RL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
    RR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')

    hip_pos = np.array(([d.body(FL_hip_id).xpos,
                        d.body(FR_hip_id).xpos,
                        d.body(RL_hip_id).xpos,
                        d.body(RR_hip_id).xpos]))

    stance_time = 0.5
    linear_com_velocity = np.array([0.1, 0.0, 0.0])
    desired_linear_com_velocity = np.array([0.1, 0.0, 0.0])
    com_height = d.qpos[2]

    foothold_generator = FootHoldReferenceGenerator(stance_time)
    footholds_reference = foothold_generator.compute_footholds_reference(linear_com_velocity, desired_linear_com_velocity, 
                                                                         hip_pos, com_height)
    print("iniztial hip_pos: ", hip_pos)
    print("footholds_reference: ", footholds_reference)
  