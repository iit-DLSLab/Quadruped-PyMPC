import numpy as np

np.set_printoptions(precision=3, suppress=True)
from numpy.linalg import norm
import time

import casadi as cs
# import example_robot_data as robex

import os

dir_path = os.path.dirname(os.path.realpath(__file__))

import sys

sys.path.append(dir_path)
sys.path.append(dir_path + "/../../")

# from jnrh2023.utils.meshcat_viewer_wrapper import MeshcatVisualizer

# Mujoco magic

import mujoco
import mujoco.viewer

# Pinocchio magic
import pinocchio as pin
from pinocchio import casadi as cpin



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
        self.base_pose = self.opti.parameter(7)  # 7 is the number of DoF of the base, position + quaternion
        self.FL_foot_target_position = self.opti.parameter(3)  # 3 is the number of DoF of the foot, position
        self.FR_foot_target_position = self.opti.parameter(3)  # 3 is the number of DoF of the foot, position
        self.RL_foot_target_position = self.opti.parameter(3)  # 3 is the number of DoF of the foot, position
        self.RR_foot_target_position = self.opti.parameter(3)  # 3 is the number of DoF of the foot, position

        # define the configuration variables (base pose + joints))
        self.var_q = self.opti.variable(self.model.nq)

        # define the cost function (it's parametric!!)
        totalcost = (
            cs.sumsqr(self.FL_foot_position(self.var_q) - self.FL_foot_target_position)
            + cs.sumsqr(self.FR_foot_position(self.var_q) - self.FR_foot_target_position)
            + cs.sumsqr(self.RL_foot_position(self.var_q) - self.RL_foot_target_position)
            + cs.sumsqr(self.RR_foot_position(self.var_q) - self.RR_foot_target_position)
        )
        self.opti.minimize(totalcost)

        # define the solver
        p_opts = dict(print_time=False, verbose=False)
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt", p_opts, s_opts)

        # define the parametric constraints for the base, it's fixed, only the leg can move!
        self.opti.subject_to(self.var_q[0:7] == self.base_pose)

        # if use_viewer is yes, you can see the different solution iteration by iteration
        # in the browser
        if self.use_viewer:
            self.opti.callback(lambda i: self.callback(self.opti.debug.value(self.var_q)))

    def compute_solution(
        self,
        q: np.ndarray,
        FL_foot_target_position: np.ndarray,
        FR_foot_target_position: np.ndarray,
        RL_foot_target_position: np.ndarray,
        RR_foot_target_position: np.ndarray,
    ) -> np.ndarray:
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

        # print("initial state", q)
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
            # print("final q: \n", sol_q)
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
        # transform_frame_to_world = self.data.oMf[self.FL_foot_id]
        # self.viewer["target"].set_transform(self.transform_target_to_world.np)
        # self.viewer["current"].set_transform(transform_frame_to_world.np)
        self.viz.display(q)
        print("q: \n", q)
        time.sleep(0.5)


if __name__ == "__main__":
    from pathlib import Path

    import gym_quadruped
    from quadruped_pympc import config as cfg

    # All distances are in meters and all foot targets are in world coordinates.
    # Start from the nominal pose, keep the base fixed, and move the feet slightly.
    legs = ("FL", "FR", "RL", "RR")
    # Draw independent offsets for each foot once per run, then keep targets fixed.
    # Use default_rng(42) instead to reproduce the same example on every launch.
    rng = np.random.default_rng()
    # Displacements around home: x and y +/-10 cm, z +1 to +10 cm.
    offsets = rng.uniform(low=[-0.10, -0.10, 0.01], high=[0.10, 0.10, 0.10], size=(4, 3))
    colors = ([1, 0.2, 0.2, 0.7], [0.2, 1, 0.2, 0.7],
              [0.2, 0.4, 1, 0.7], [1, 0.8, 0.1, 0.7])
    model_path = Path(gym_quadruped.__file__).parent / "robot_model" / cfg.robot_cfg.mjcf_filename
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[2] = 0.6  # Suspend the robot so every foot is clearly visible.
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    initial_q = data.qpos.copy()  # xyz, quaternion wxyz, then the 12 joint angles.
    foot_ids = [model.geom(cfg.robot_feet_geom_names[leg]).id for leg in legs]
    initial_feet = data.geom_xpos[foot_ids].copy()
    # Copy target positions: they must not change when forward kinematics updates data.
    targets = initial_feet + offsets
    # Despite its historical name, this solver minimizes a nonlinear foot-position
    # error with IPOPT, constraining the base pose and optimizing the joint angles.
    pin_model = pin.buildModelFromMJCF(str(model_path))
    for leg, foot_id in zip(legs, foot_ids):
        # Pinocchio imports body frames, but not MuJoCo foot geometry centers.
        # Attach a frame at each desired contact point, expressed in its parent body.
        body_name = model.body(model.geom_bodyid[foot_id]).name
        parent_id = pin_model.getFrameId(body_name)
        parent = pin_model.frames[parent_id]
        placement = parent.placement * pin.SE3(np.eye(3), model.geom_pos[foot_id])
        pin_model.addFrame(pin.Frame(f"{leg}_foot_fixed", parent.parentJoint,
                                     parent_id, placement, pin.FrameType.OP_FRAME))
    robot = pin.RobotWrapper(pin_model)
    ik = InverseKinematicsQP(robot, use_viewer=False)

    # Pinocchio uses quaternion xyzw; MuJoCo uses wxyz. Joint order may also differ,
    # so transfer joint values by name instead of relying on hard-coded leg slices.
    q_pin = pin.neutral(pin_model)
    q_pin[:3] = initial_q[:3]
    q_pin[3:7] = initial_q[[4, 5, 6, 3]]
    joint_mapping = []
    for names in cfg.robot_leg_joints.values():
        for name in names:
            mj_index = model.joint(name).qposadr[0]
            pin_index = pin_model.joints[pin_model.getJointId(name)].idx_q
            joint_mapping.append((mj_index, pin_index))
            q_pin[pin_index] = initial_q[mj_index]
    started = time.perf_counter()
    solution = ik.compute_solution(q_pin, *targets)
    elapsed = time.perf_counter() - started
    if solution is None:
        raise RuntimeError("IK did not converge; no solution to display.")
    for mj_index, pin_index in joint_mapping:
        data.qpos[mj_index] = solution[pin_index]

    # Recompute forward kinematics at the IK solution to measure its accuracy.
    # Do not step the dynamics: this demo displays configurations, not torque control.
    mujoco.mj_forward(model, data)
    final_feet = data.geom_xpos[foot_ids].copy()
    print(f"IK solve time: {elapsed * 1000:.2f} ms")
    for leg, target, before, after in zip(legs, targets, initial_feet, final_feet):
        print(f"{leg}: target {target}, error {np.linalg.norm(target - before) * 1000:.2f}"
              f" -> {np.linalg.norm(target - after) * 1000:.2f} mm")
    print("Target colors: FL red, FR green, RL blue, RR yellow. Close the window to stop.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = initial_q[:3]
        viewer.cam.distance = 1.6
        viewer.cam.azimuth, viewer.cam.elevation = 135, -20
        with viewer.lock():
            # Visual-only spheres: these are the DESIRED positions, not geom_xpos
            # at the solution. They stay fixed even when there is an IK residual.
            viewer.user_scn.ngeom = 0
            for i, (target, color) in enumerate(zip(targets, colors)):
                mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                   [0.025] * 3, target, np.eye(3).ravel(), color)
                viewer.user_scn.ngeom += 1
        while viewer.is_running():
            viewer.sync()
            time.sleep(1.0 / 60.0)
