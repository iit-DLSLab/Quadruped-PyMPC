import numpy as np
np.set_printoptions(precision=3, suppress = True)
from numpy.linalg import norm
import time
import unittest
import casadi as cs
#import example_robot_data as robex

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path)
sys.path.append(dir_path + '/../../')

#from jnrh2023.utils.meshcat_viewer_wrapper import MeshcatVisualizer

# Mujoco magic
import mujoco
import mujoco.viewer

# Pinocchio magic
import pinocchio as pin
from pinocchio import casadi as cpin

import copy



# Class for solving a generic inverse kinematics problem
class InverseKinematicsQP:
    def __init__(self, robot: pin.robot_wrapper.RobotWrapper, use_viewer: bool = None) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:
            robot: The robot model.
            use_viewer: Whether to use the Meshcat viewer.
        """
        self.robot = robot
        self.model = self.robot.model
        self.data = self.robot.data
        
        # generate the casadi graph
        cmodel = cpin.Model(self.model)
        cdata = cmodel.createData()
        cq = cs.SX.sym("q", self.model.nq, 1)
        
        # precompute the forward kinematics graph
        cpin.framesForwardKinematics(cmodel, cdata, cq)

        # initialize the viewer if requested
        self.use_viewer = use_viewer        
        """if(self.use_viewer):
            # Open the viewer
            self.viz = MeshcatVisualizer(self.robot)
            self.viz.display(self.robot.q0)
            self.viewer = self.viz.viewer
            time.sleep(10)"""
        

        # takes the ID of the feet, and generate a casadi function for a generic forward kinematics
        self.FL_foot_id = self.model.getFrameId("FL_foot_fixed")
        self.FL_foot_position = cs.Function("FL_foot_pos", [cq], [cdata.oMf[self.FL_foot_id].translation])

        self.FR_foot_id = self.model.getFrameId("FR_foot_fixed")
        self.FR_foot_position = cs.Function("FR_foot_pos", [cq], [cdata.oMf[self.FR_foot_id].translation])

        self.RL_foot_id = self.model.getFrameId("RL_foot_fixed")
        self.RL_foot_position = cs.Function("RL_foot_pos", [cq], [cdata.oMf[self.RL_foot_id].translation])

        self.RR_foot_id = self.model.getFrameId("RR_foot_fixed")
        self.RR_foot_position = cs.Function("RR_foot_pos", [cq], [cdata.oMf[self.RR_foot_id].translation])


        # create the NLP for computing the forward kinematics
        self.create_nlp_ik()
    


    def create_nlp_ik(self) -> None:
        """
        This method creates the NLP for the forward kinematics problem and sets up the necessary variables.
        """
        # create NLP
        self.opti = cs.Opti()
        

        # casadi param to be updated at each request
        self.base_pose = self.opti.parameter(7) #7 is the number of DoF of the base, position + quaternion
        self.FL_foot_target_position = self.opti.parameter(3) #3 is the number of DoF of the foot, position
        self.FR_foot_target_position = self.opti.parameter(3) #3 is the number of DoF of the foot, position
        self.RL_foot_target_position = self.opti.parameter(3) #3 is the number of DoF of the foot, position
        self.RR_foot_target_position = self.opti.parameter(3) #3 is the number of DoF of the foot, position


        # define the configuration variables (base pose + joints))   
        self.var_q = self.opti.variable(self.model.nq) 


        # define the cost function (it's parametric!!)
        totalcost = cs.sumsqr(self.FL_foot_position(self.var_q) - self.FL_foot_target_position) +\
                    cs.sumsqr(self.FR_foot_position(self.var_q) - self.FR_foot_target_position) +\
                    cs.sumsqr(self.RL_foot_position(self.var_q) - self.RL_foot_target_position) +\
                    cs.sumsqr(self.RR_foot_position(self.var_q) - self.RR_foot_target_position)
        self.opti.minimize(totalcost)
        

        # define the solver
        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt", p_opts, s_opts)

        
        
        # define the parametric constraints for the base, it's fixed, only the leg can move!
        self.opti.subject_to(self.var_q[0:7] == self.base_pose)


        # if use_viewer is yes, you can see the different solution iteration by iteration
        # in the browser
        if(self.use_viewer):
            self.opti.callback(lambda i: self.callback(self.opti.debug.value(self.var_q)))
            


    def compute_solution(self, q: np.ndarray, FL_foot_target_position: np.ndarray, FR_foot_target_position: np.ndarray, 
                   RL_foot_target_position: np.ndarray, RR_foot_target_position: np.ndarray) -> np.ndarray:
        """
        This method computes the inverse kinematics from initial joint angles and desired foot target positions.

        Args:
            q (np.ndarray): The initial joint angles.
            FL_foot_target_position (np.ndarray): The desired position of the front-left foot.
            FR_foot_target_position (np.ndarray): The desired position of the front-right foot.
            RL_foot_target_position (np.ndarray): The desired position of the rear-left foot.
            RR_foot_target_position (np.ndarray): The desired position of the rear-right foot.

        Returns:
            np.ndarray: The joint angles that achieve the desired foot positions.
        """
        
        #print("initial state", q)
        # set the value for the constraints
        self.opti.set_value(self.base_pose, q[0:7])

        # set initial guess
        self.opti.set_initial(self.var_q, q)

        # set the value for the target
        self.opti.set_value(self.FL_foot_target_position, FL_foot_target_position)
        self.opti.set_value(self.FR_foot_target_position, FR_foot_target_position)
        self.opti.set_value(self.RL_foot_target_position, RL_foot_target_position)
        self.opti.set_value(self.RR_foot_target_position, RR_foot_target_position)

        # Caution: in case the solver does not converge, we are picking the candidate values
        # at the last iteration in opti.debug, and they are NO guarantee of what they mean.
        try:
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)
            #print("final q: \n", sol_q)
            return sol_q
        except:
            print("ERROR in convergence, plotting debug info.")
            sol_q = self.opti.debug.value(self.var_q)



    def callback(self, q: np.ndarray) -> None:
        """
        This method is called by the solver at each iteration (if use_viewer is TRUE) 
        and displays the current joint angles and foot positions.
        """
        pin.framesForwardKinematics(self.model, self.data, q)
        #transform_frame_to_world = self.data.oMf[self.FL_foot_id]
        #self.viewer["target"].set_transform(self.transform_target_to_world.np)
        #self.viewer["current"].set_transform(transform_frame_to_world.np)
        self.viz.display(q)
        print("q: \n", q)
        time.sleep(0.5)


if __name__ == "__main__":
    robot = robex.load("go1")
    ik = InverseKinematicsQP(robot, use_viewer=False)

    FL_foot_target_position = np.array([0.1, 0, -0.02])
    FR_foot_target_position = np.array([-0.08, 0, 0])
    RL_foot_target_position = np.array([-0.12, 0, 0.06])
    RR_foot_target_position = np.array([0, 0.2, 0])



    initial_time = time.time()
    solution = ik.compute_solution(robot.q0, FL_foot_target_position, FR_foot_target_position, 
                               RL_foot_target_position, RR_foot_target_position)
    print("time: ", time.time() - initial_time)


    # Check consistency in mujoco
    m = mujoco.MjModel.from_xml_path('./../simulation/robot_model/unitree_go1/scene.xml')
    d = mujoco.MjData(m)
    d.qpos[2] = robot.q0[2]
    d.qpos[7:] = robot.q0[7:]


    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')

    
    joint_FL = solution[7:10]
    joint_FR = solution[10:13]
    joint_RL = solution[13:16]
    joint_RR = solution[16:19]
    
    d.qpos[7:] = np.concatenate((joint_FR, joint_FL, joint_RR, joint_RL))
    mujoco.mj_step(m, d)
    print("\n")
    print("MUJOCO SOLUTION")
    foot_position_FL = d.geom_xpos[FL_id]
    foot_position_FR = d.geom_xpos[FR_id]
    foot_position_RL = d.geom_xpos[RL_id]
    foot_position_RR = d.geom_xpos[RR_id]
    print("joints: ", np.concatenate((joint_FL, joint_FR, joint_RL, joint_RR)))
    print("FL foot position: ", foot_position_FL)
    print("FR foot position: ", foot_position_FR)
    print("RL foot position:  ", foot_position_RL)
    print("RR foot position: ", foot_position_RR)
    
    print("\n")
    print("PINOCCHIO SOLUTION")
    print("joints: ", solution[7:])
    print("FL foot position", ik.FL_foot_position(solution))
    print("FR foot position", ik.FR_foot_position(solution))
    print("RL foot position", ik.RL_foot_position(solution))
    print("RR foot position", ik.RR_foot_position(solution))

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while True:
            viewer.sync()