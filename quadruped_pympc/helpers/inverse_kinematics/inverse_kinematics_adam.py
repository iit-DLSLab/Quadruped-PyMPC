import numpy as np
np.set_printoptions(precision=3, suppress = True)
from numpy.linalg import norm, solve
import time
import casadi as cs
#import example_robot_data as robex
import copy

# Mujoco magic
import mujoco
import mujoco.viewer

# Adam and Liecasadi magic
from adam.casadi import KinDynComputations
from adam import Representations
from liecasadi import SO3

# Pinocchio magic
#import pinocchio as pin
#from pinocchio import casadi as cpin

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path)
sys.path.append(dir_path + '/../')

from quadruped_pympc import config 




# Class for solving a generic inverse kinematics problem
class InverseKinematicsAdam:
    def __init__(self,) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:

        """

        if(config.robot == 'go2'):
            urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/go2/go2.urdf' 
            xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/go2/go2.xml'
        elif(config.robot == 'aliengo'):
            #urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.urdf'
            urdf_filename = '/home/iit.local/gturrisi/personal_ws_home/gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.urdf'
            xml_filename = '/home/iit.local/gturrisi/personal_ws_home/gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.xml'
        elif(config.robot == 'hyqreal'):
            urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.urdf'
            xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.xml'
        elif(config.robot == 'mini_cheetah'):
            urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.urdf'
            xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.xml'

        self.use_adam = True

        if(self.use_adam):
            joint_list = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
                        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']

            
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
        
        else:
            self.model = pin.buildModelFromUrdf(urdf_filename)
            #self.model = pin.buildModelFromXML(xml_filename)
            self.data = self.model.createData()
            
            # generate the casadi graph
            cmodel = cpin.Model(self.model)
            cdata = cmodel.createData()
            cq = cs.SX.sym("q", self.model.nq, 1)
            
            # precompute the forward kinematics graph
            cpin.framesForwardKinematics(cmodel, cdata, cq)

            # takes the ID of the feet, and generate a casadi function for a generic forward kinematics
            self.FL_foot_id = self.model.getFrameId("FL_foot_fixed")
            self.FR_foot_id = self.model.getFrameId("FR_foot_fixed")
            self.RL_foot_id = self.model.getFrameId("RL_foot_fixed")
            self.RR_foot_id = self.model.getFrameId("RR_foot_fixed")
            #self.FL_foot_id = self.model.getJointId("FL_foot_fixed")
            
            self.forward_kinematics_FL_fun = cs.Function("FR_foot_pos", [cq], [cdata.oMf[self.FR_foot_id].translation])
            self.forward_kinematics_FR_fun = cs.Function("FL_foot_pos", [cq], [cdata.oMf[self.FL_foot_id].translation])
            self.forward_kinematics_RL_fun = cs.Function("RR_foot_pos", [cq], [cdata.oMf[self.RR_foot_id].translation])
            self.forward_kinematics_RR_fun = cs.Function("RL_foot_pos", [cq], [cdata.oMf[self.RL_foot_id].translation])

            



    def compute_solution(self, q: np.ndarray, FL_foot_target_position: np.ndarray, FR_foot_target_position: np.ndarray, 
                   RL_foot_target_position: np.ndarray, RR_foot_target_position: np.ndarray) -> np.ndarray:
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
        IT_MAX = 4000
        DT = 1e-2
        damp = 1e-2
        damp_matrix = damp * np.eye(12)
        
        i=0


        err_FL = cs.SX.zeros(3,1)
        err_FR = cs.SX.zeros(3,1)
        err_RL = cs.SX.zeros(3,1)
        err_RR = cs.SX.zeros(3,1)
        err = cs.SX.zeros(3)

        q_joint = q[7:]
        quaternion = q[3:7]
        quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        R = SO3.from_quat(quaternion).as_matrix()
        H = cs.DM.eye(4)
        H[0:3, 0:3] = R
        H[0:3, 3] = q[0:3]

        initial_time = time.time()
        
        while True:            

            
            time_fk = time.time()
            
            if(self.use_adam):
                FL_foot_actual_pos = self.forward_kinematics_FL_fun(H, q_joint)[0:3, 3]
                FR_foot_actual_pos = self.forward_kinematics_FR_fun(H, q_joint)[0:3, 3]
                RL_foot_actual_pos = self.forward_kinematics_RL_fun(H, q_joint)[0:3, 3]
                RR_foot_actual_pos = self.forward_kinematics_RR_fun(H, q_joint)[0:3, 3]
            else:
                pin.forwardKinematics(self.model, self.data, q)
                pin.computeJointJacobians(self.model, self.data, q)
                FL_foot_actual_pos = self.forward_kinematics_FL_fun(q)[0:3]
                FR_foot_actual_pos = self.forward_kinematics_FR_fun(q)[0:3]
                RL_foot_actual_pos = self.forward_kinematics_RL_fun(q)[0:3]
                RR_foot_actual_pos = self.forward_kinematics_RR_fun(q)[0:3]


            err_FL = (FL_foot_target_position - FL_foot_actual_pos)
            err_FR = (FR_foot_target_position - FR_foot_actual_pos)
            err_RL = (RL_foot_target_position - RL_foot_actual_pos)
            err_RR = (RR_foot_target_position - RR_foot_actual_pos)
            
            time_fk = time.time() - time_fk
            

            err = err_FL + err_FR + err_RL + err_RR
            norm_err = cs.norm_2(err)
            if(norm_err < eps):
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            

            
            time_getJ = time.time()
            if(self.use_adam):
                J_FL = self.jacobian_FL_fun(H, q_joint)[0:3, 6:]
                J_FR = self.jacobian_FR_fun(H, q_joint)[0:3, 6:]
                J_RL = self.jacobian_RL_fun(H, q_joint)[0:3, 6:]
                J_RR = self.jacobian_RR_fun(H, q_joint)[0:3, 6:]
            else:
                J_FL = pin.getFrameJacobian(self.model, self.data, self.FL_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
                J_FR = pin.getFrameJacobian(self.model, self.data, self.FR_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
                J_RL = pin.getFrameJacobian(self.model, self.data, self.RL_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
                J_RR = pin.getFrameJacobian(self.model, self.data, self.RR_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
            time_getJ = time.time() - time_getJ


            time_solve = time.time()
            #v_FL = J_FL.T@cs.solve(J_FL@J_FL.T + damp_matrix, err_FL)
            #v _FR = J_FR.T@cs.solve(J_FR@J_FR.T + damp_matrix, err_FR)
            #v_RL = J_RL.T@cs.solve(J_RL@J_RL.T + damp_matrix, err_RL)
            #v_RR = J_RR.T@cs.solve(J_RR@J_RR.T + damp_matrix, err_RR)
            total_jac = cs.vertcat(J_FL, J_FR, J_RL, J_RR)
            total_err = 100.0 * cs.vertcat(err_FL, err_FR, err_RL, err_RR)
            damped_pinv = cs.inv(total_jac.T@total_jac + damp_matrix)@total_jac.T
            v =  damped_pinv @ total_err

            # damped_pinv_FL = cs.inv(J_FL.T@J_FL + damp_matrix)@J_FL.T
            # damped_pinv_FR = cs.inv(J_FR.T@J_FR + damp_matrix)@J_FR.T
            # damped_pinv_RL = cs.inv(J_RL.T@J_RL + damp_matrix)@J_RL.T
            # damped_pinv_RR = cs.inv(J_RR.T@J_RR + damp_matrix)@J_RR.T
            # #err_concat = cs.vertcat(err_FL, err_FR, err_RL, err_RR)
            # v_FL =  damped_pinv_FL@err_FL
            # v_FR =  damped_pinv_FR@err_FR
            # v_RL =  damped_pinv_RL@err_RL
            # v_RR =  damped_pinv_RR@err_RR
            # v = v_FL + v_FR + v_RL + v_RR
                        
            
            q_joint = q_joint + DT * v

            time_solve = time.time() - time_solve
            #print("time fk: ", time_fk)
            #print("time getJ: ", time_getJ)
            #print("time solve: ", time_solve)
            #print("norm_err: ", norm_err)
            


            #if not i % 10:
            #    print("q_joint: \n", q_joint)
            #    print('%d: error = %s' % (i, err.T))
            i += 1
  
        if success:
            print("Convergence achieved in iteration: ", i)
            print("in time: ", time.time() - initial_time)
        else:
            print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

        return q_joint





if __name__ == "__main__":

    if(config.robot == 'go2'):
        urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/go2/go2.urdf' 
        xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/go2/go2.xml'
    elif(config.robot == 'aliengo'):
        #urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.urdf'
        urdf_filename = '/home/iit.local/gturrisi/personal_ws_home/gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.urdf'
        xml_filename = '/home/iit.local/gturrisi/personal_ws_home/gym-quadruped/gym_quadruped/robot_model/aliengo/aliengo.xml'
    elif(config.robot == 'hyqreal'):
        urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.urdf'
        xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.xml'
    elif(config.robot == 'mini_cheetah'):
        urdf_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.urdf'
        xml_filename = dir_path + '/../../../gym-quadruped/gym_quadruped/robot_model/mini_cheetah/mini_cheetah.xml'
    
    
    ik = InverseKinematicsAdam()   

    #FL_foot_target_position = np.array([0.1, 0, -0.4])
    #FR_foot_target_position = np.array([-0.08, 0, -0.4])
    #RL_foot_target_position = np.array([-0.12, 0, -0.4])
    #RR_foot_target_position = np.array([0, 0.2, -0.4])


    # Check consistency in mujoco
    m = mujoco.MjModel.from_xml_path(xml_filename)
    d = mujoco.MjData(m)

    random_q_joint = np.random.rand(12,)
    d.qpos[7:] = random_q_joint

    # random quaternion
    rand_quat = np.random.rand(4,)
    rand_quat = rand_quat / np.linalg.norm(rand_quat)
    d.qpos[3:7] = rand_quat


    mujoco.mj_step(m, d)

    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')
    FL_foot_target_position = d.geom_xpos[FL_id]
    FR_foot_target_position = d.geom_xpos[FR_id]
    RL_foot_target_position = d.geom_xpos[RL_id]
    RR_foot_target_position = d.geom_xpos[RR_id]

    print("FL foot target position: ", FL_foot_target_position)
    print("FR foot target position: ", FR_foot_target_position)
    print("RL foot target position: ", RL_foot_target_position)
    print("RR foot target position: ", RR_foot_target_position)

    # print forward  kinematics from ik.forward_kinematics_FL_fun
    quaternion = d.qpos[3:7]
    quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
    R = SO3.from_quat(quaternion).as_matrix()
    H = cs.DM.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = d.qpos[0:3]
    print("FL foot position", ik.forward_kinematics_FL_fun(H, random_q_joint)[0:3, 3])
    print("FR foot position", ik.forward_kinematics_FR_fun(H, random_q_joint)[0:3, 3])
    print("RL foot position", ik.forward_kinematics_RL_fun(H, random_q_joint)[0:3, 3])
    print("RR foot position", ik.forward_kinematics_RR_fun(H, random_q_joint)[0:3, 3])

    # print(cacca)


    initial_time = time.time()
    initial_q = copy.deepcopy(d.qpos)
    initial_q[7:] = np.random.rand(12,)
    solution = ik.compute_solution(initial_q, FL_foot_target_position, FR_foot_target_position, 
                               RL_foot_target_position, RR_foot_target_position)
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