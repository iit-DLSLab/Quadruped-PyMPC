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

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../')

import config

# Class for solving a generic inverse kinematics problem
class InverseKinematicsQP:
    def __init__(self, model, use_viewer = None) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:
            robot: The robot model.
            use_viewer: Whether to use the Meshcat viewer.
        """
        self.model = model
        self.data = model.createData()
        
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



    if(config.robot == 'go2'):
        urdf_filename = dir_path + '/../simulation/robot_model/go2/go2.urdf' 
        xml_filename = dir_path + '/../simulation/robot_model/go2/scene_flat.xml'
    elif(config.robot == 'aliengo'):
        urdf_filename = dir_path + '/../simulation/robot_model/aliengo/aliengo.urdf'
        xml_filename = dir_path + '/../simulation/robot_model/aliengo/scene_flat.xml'
    elif(config.robot == 'hyqreal'):
        urdf_filename = dir_path + '/../simulation/robot_model/hyqreal/hyqreal.urdf'
        xml_filename = dir_path + '/../simulation/robot_model/hyqreal/scene_flat.xml'
    elif(config.robot == 'mini_cheetah'):
        urdf_filename = dir_path + '/../simulation/robot_model/mini_cheetah/mini_cheetah.urdf'
        xml_filename = dir_path + '/../simulation/robot_model/mini_cheetah/scene.xml'
    

    # Load the urdf model
    model = pin.buildModelFromUrdf(urdf_filename)
    
    
    ik = InverseKinematicsQP(model=model, use_viewer=False)   


    # Check consistency in mujoco
    m = mujoco.MjModel.from_xml_path(xml_filename)
    d = mujoco.MjData(m)

    random_q_joint = np.random.rand(12,)
    d.qpos[7:] = random_q_joint
    mujoco.mj_step(m, d)

    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')
    FL_foot_target_position = d.geom_xpos[FL_id]
    FR_foot_target_position = d.geom_xpos[FR_id]
    RL_foot_target_position = d.geom_xpos[RL_id]
    RR_foot_target_position = d.geom_xpos[RR_id]


    

    initial_q = copy.deepcopy(d.qpos)
    initial_q[7:] = np.random.rand(12,)
    initial_time = time.time()
    solution = ik.compute_solution(initial_q, FL_foot_target_position, FR_foot_target_position, 
                               RL_foot_target_position, RR_foot_target_position)
    print("time: ", time.time() - initial_time)



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
    print("PINOCCHIO SOLUTION")
    print("joints: ", solution[7:])
    print("FL foot position", ik.FL_foot_position(solution))
    print("FR foot position", ik.FR_foot_position(solution))
    print("RL foot position", ik.RL_foot_position(solution))
    print("RR foot position", ik.RR_foot_position(solution))



    with mujoco.viewer.launch_passive(m, d) as viewer:
        while True:
            viewer.sync()