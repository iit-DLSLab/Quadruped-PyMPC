import numpy as np
np.set_printoptions(precision=3, suppress = True)
from numpy.linalg import norm, solve
import time
import casadi as cs
#import example_robot_data as robex



# Mujoco magic
import mujoco
import mujoco.viewer

# Pinocchio magic
import pinocchio as pin
from pinocchio import casadi as cpin

import copy



# Class for solving a generic inverse kinematics problem
class InverseKinematicsPin:
    def __init__(self, model, use_viewer = None) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:
            model: The robot model.
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
        #self.FL_foot_id = self.model.getJointId("FL_foot_fixed")
        self.FL_foot_position = cs.Function("FL_foot_pos", [cq], [cdata.oMf[self.FL_foot_id].translation])

        self.FR_foot_id = self.model.getFrameId("FR_foot_fixed")
        #self.FR_foot_id = self.model.getJointId("FR_foot_fixed")
        self.FR_foot_position = cs.Function("FR_foot_pos", [cq], [cdata.oMf[self.FR_foot_id].translation])

        self.RL_foot_id = self.model.getFrameId("RL_foot_fixed")
        #self.RL_foot_id = self.model.getJointId("RL_foot_fixed")
        self.RL_foot_position = cs.Function("RL_foot_pos", [cq], [cdata.oMf[self.RL_foot_id].translation])

        self.RR_foot_id = self.model.getFrameId("RR_foot_fixed")
        #self.RR_foot_id = self.model.getJointId("RR_foot_fixed")
        self.RR_foot_position = cs.Function("RR_foot_pos", [cq], [cdata.oMf[self.RR_foot_id].translation])





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

        eps = 1e-3
        IT_MAX = 1000
        DT = 1e-0
        damp = 1e-12
        damp_matrix = damp * np.eye(6)

        i=0
        err_FL = np.zeros(6).reshape(6,1)
        err_FR = np.zeros(6).reshape(6,1)
        err_RL = np.zeros(6).reshape(6,1)
        err_RR = np.zeros(6).reshape(6,1)
        err = np.zeros(6).reshape(6,1)
        while True:

            pin.forwardKinematics(self.model, self.data, q)
            pin.computeJointJacobians(self.model, self.data, q)


            time_fk = time.time()
            err_FL[0:3] = (self.FL_foot_position(q) - FL_foot_target_position)
            err_FR[0:3] = (self.FR_foot_position(q) - FR_foot_target_position)
            err_RL[0:3] = (self.RL_foot_position(q) - RL_foot_target_position)
            err_RR[0:3] = (self.RR_foot_position(q) - RR_foot_target_position)
            time_fk = time.time() - time_fk
            #print("time fk: ", time_fk)

            #iMd = self.data.oMi[self.FL_foot_id].actInv(FL_foot_target_position)
            #err = pin.log(iMd).vector  # in joint frame
            err = err_FL + err_FR + err_RL + err_RR

            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            #breakpoint()
            # pin.ReferenceFrame.LOCAL_WORLD_ALIGNED, pin.ReferenceFrame.LOCAL_WORLD, pin.ReferenceFrame.WORLD

            time_getJ = time.time()
            J_FL = pin.getFrameJacobian(self.model, self.data, self.FL_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
            J_FR = pin.getFrameJacobian(self.model, self.data, self.FR_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
            J_RL = pin.getFrameJacobian(self.model, self.data, self.RL_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
            J_RR = pin.getFrameJacobian(self.model, self.data, self.RR_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # in joint frame
            time_getJ = time.time() - time_getJ
            #print("time getJ: ", time_getJ)

            #J_FL = pin.getJointJacobian(self.model, self.data, self.FL_foot_id, pin.ReferenceFrame.WORLD)
            #J_FR = pin.getJointJacobian(self.model, self.data, self.FR_foot_id, pin.ReferenceFrame.WORLD)
            #J_RL = pin.getJointJacobian(self.model, self.data, self.RL_foot_id, pin.ReferenceFrame.WORLD)
            #J_RR = pin.getJointJacobian(self.model, self.data, self.RR_foot_id, pin.ReferenceFrame.WORLD)
            #J = np.concatenate((J_FL, J_FR, J_RL, J_RR), axis=1)
            #J = -np.dot(pin.Jlog6(iMd.inverse()), J)

            time_solve = time.time()

            v_FL = - J_FL.T@solve(J_FL@J_FL.T + damp_matrix, err_FL)
            v_FR = - J_FR.T@solve(J_FR@J_FR.T + damp_matrix, err_FR)
            v_RL = - J_RL.T@solve(J_RL@J_RL.T + damp_matrix, err_RL)
            v_RR = - J_RR.T@solve(J_RR@J_RR.T + damp_matrix, err_RR)
            v = v_FL + v_FR + v_RL + v_RR

            #v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model,q,v*DT)

            time_solve = time.time() - time_solve

            #print("time solve: ", time_solve)
            #print("time total: ", time_solve + time_fk + time_getJ)

            #print("q: \n", q)
            if not i % 10:
                print('%d: error = %s' % (i, err.T))
            i += 1

        if success:
            print("Convergence achieved!")
        else:
            print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

        return q



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

    # Load the urdf model
    urdf_filename = "../../../gym-quadruped/gym_quadruped/robot_model/hyqreal/hyqreal.urdf"
    model = pin.buildModelFromUrdf(urdf_filename)


    ik = InverseKinematicsPin(model=model, use_viewer=False)

    FL_foot_target_position = np.array([0.1, 0, -0.4])
    FR_foot_target_position = np.array([-0.08, 0, -0.4])
    RL_foot_target_position = np.array([-0.12, 0, -0.4])
    RR_foot_target_position = np.array([0, 0.2, -0.4])


    q0 = pin.neutral(ik.model)
    initial_time = time.time()
    solution = ik.compute_solution(q0, FL_foot_target_position, FR_foot_target_position,
                               RL_foot_target_position, RR_foot_target_position)
    print("time: ", time.time() - initial_time)



    # Check consistency in mujoco
    m = mujoco.MjModel.from_xml_path('./../simulation/robot_model/hyqreal/scene.xml')
    d = mujoco.MjData(m)
    #d.qpos[2] = q0[2]
    d.qpos[7:] = q0#[7:]


    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')


    joint_FL = solution[0:3]
    joint_FR = solution[3:6]
    joint_RL = solution[6:9]
    joint_RR = solution[9:12]

    d.qpos[7:] = np.concatenate((joint_FL, joint_FR, joint_RL, joint_RR))
    #d.qpos[7:] = np.concatenate((joint_FR, joint_FL, joint_RR, joint_RL))
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


    print("\n")
    print("DESIRED SOLUTION")
    print("FL foot position", FL_foot_target_position)
    print("FR foot position", FR_foot_target_position)
    print("RL foot position", RL_foot_target_position)
    print("RR foot position", RR_foot_target_position)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while True:
            viewer.sync()