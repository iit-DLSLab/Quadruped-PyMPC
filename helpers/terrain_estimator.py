import numpy as np
from numpy.linalg import norm
import time
import unittest
import casadi as cs
#import example_robot_data as robex
import mujoco

import pinocchio as pin

from quadruped_utils import LegsAttr


#from pinocchio import casadi as cpin

class TerrainEstimator:
    def __init__(self, ) -> None:
        
        self.terrain_roll = 0
        self.terrain_pitch = 0



    def compute_terrain_estimation(self,
                                   base_position: np.ndarray,
                                   yaw: float,
                                   lift_foot_position: LegsAttr) -> np.ndarray:
        """
        TODO: Docstring and documentation of the expected order of the feet positions
        """

        
        # Compute roll and pitch for each foot position
        roll = 0
        pitch = 0
        
        # Rotation matrix R_yaw
        h_R_w = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Extracting 3-element segments from liftoff_position_z_ and x_op_
        seg0 = lift_foot_position.FL
        seg3 = lift_foot_position.FR
        seg6 = lift_foot_position.RL
        seg9 = lift_foot_position.RR
        
        
        # Calculating differences
        front_difference = h_R_w @ (seg0 - base_position) - h_R_w @ (seg3 - base_position)
        back_difference = h_R_w @ (seg6 - base_position) - h_R_w @ (seg9 - base_position)
        left_difference = h_R_w @ (seg0 - base_position) - h_R_w @ (seg6 - base_position)
        right_difference = h_R_w @ (seg3 - base_position) - h_R_w @ (seg9 - base_position)



        # Calculating pitch and roll
        pitch = (np.arctan(np.abs(left_difference[2]) / np.abs(left_difference[0] + 0.001)) + 
                np.arctan(np.abs(right_difference[2]) / np.abs(right_difference[0] + 0.001))) * 0.5

        roll = (np.arctan(np.abs(front_difference[2]) / np.abs(front_difference[1] + 0.001)) + 
                np.arctan(np.abs(back_difference[2]) / np.abs(back_difference[1] + 0.001 ))) * 0.5


        # Adjusting signs of pitch and roll
        if (front_difference[2] * 0.5 + back_difference[2] * 0.5) < 0:
            roll = -roll
        if (left_difference[2] * 0.5 + right_difference[2] * 0.5) > 0:
            pitch = -pitch


        self.terrain_roll = roll
        self.terrain_pitch = pitch
        return self.terrain_roll, self.terrain_pitch
 




  