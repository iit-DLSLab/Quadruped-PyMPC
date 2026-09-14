import numpy as np

np.set_printoptions(precision=3, suppress=True)
import time

import casadi as cs

# Mujoco magic
import mujoco
import mujoco.viewer
from adam import Representations

# Adam and Liecasadi magic
from adam.casadi import KinDynComputations
from liecasadi import SO3

# Class for solving a generic inverse kinematics problem
class InverseKinematicsNumeric:
    def __init__(self, mujoco_model: mujoco.MjModel) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:
            mujoco_model: MuJoCo model with FL_foot, FR_foot,
                RL_foot and RR_foot body frames.
        """

        joint_list = [
            'FL_hip_joint',
            'FL_thigh_joint',
            'FL_calf_joint',
            'FR_hip_joint',
            'FR_thigh_joint',
            'FR_calf_joint',
            'RL_hip_joint',
            'RL_thigh_joint',
            'RL_calf_joint',
            'RR_hip_joint',
            'RR_thigh_joint',
            'RR_calf_joint',
        ]

        self.kindyn = KinDynComputations.from_mujoco_model(mujoco_model, joints_name_list=joint_list)
        self.kindyn.set_frame_velocity_representation(representation=Representations.MIXED_REPRESENTATION)

        self.forward_kinematics_FL_fun = self.kindyn.forward_kinematics_fun("FL_foot")
        self.forward_kinematics_FR_fun = self.kindyn.forward_kinematics_fun("FR_foot")
        self.forward_kinematics_RL_fun = self.kindyn.forward_kinematics_fun("RL_foot")
        self.forward_kinematics_RR_fun = self.kindyn.forward_kinematics_fun("RR_foot")

        self.jacobian_FL_fun = self.kindyn.jacobian_fun("FL_foot")
        self.jacobian_FR_fun = self.kindyn.jacobian_fun("FR_foot")
        self.jacobian_RL_fun = self.kindyn.jacobian_fun("RL_foot")
        self.jacobian_RR_fun = self.kindyn.jacobian_fun("RR_foot")

        q = cs.SX.sym('q', 12 + 7)
        FL_foot_target_position = cs.SX.sym('FL_foot_target_position', 3)
        FR_foot_target_position = cs.SX.sym('FR_foot_target_position', 3)
        RL_foot_target_position = cs.SX.sym('RL_foot_target_position', 3)
        RR_foot_target_position = cs.SX.sym('RR_foot_target_position', 3)
        ik = self.compute_solution(
            q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position
        )
        self.fun_compute_solution = cs.Function(
            'fun_ik',
            [q, FL_foot_target_position, FR_foot_target_position, RL_foot_target_position, RR_foot_target_position],
            [ik],
        )

    def compute_solution(
        self,
        q: np.ndarray,
        FL_foot_target_position: np.ndarray,
        FR_foot_target_position: np.ndarray,
        RL_foot_target_position: np.ndarray,
        RR_foot_target_position: np.ndarray,
    ) -> np.ndarray:
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
        IT_MAX = 5
        DT = 1e-2
        damp = 1e-2
        damp_matrix = damp * np.eye(12)

        i = 0

        err_FL = cs.SX.zeros(3, 1)
        err_FR = cs.SX.zeros(3, 1)
        err_RL = cs.SX.zeros(3, 1)
        err_RR = cs.SX.zeros(3, 1)
        err = cs.SX.zeros(3)

        q_joint = q[7:]
        quaternion = q[3:7]
        quaternion = np.array([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        R = SO3.from_quat(quaternion).as_matrix()
        # H = cs.DM.eye(4)
        H = cs.SX.eye(4)

        H[0:3, 0:3] = R
        H[0:3, 3] = q[0:3]

        while i <= IT_MAX:
            FL_foot_actual_pos = self.forward_kinematics_FL_fun(H, q_joint)[0:3, 3]
            FR_foot_actual_pos = self.forward_kinematics_FR_fun(H, q_joint)[0:3, 3]
            RL_foot_actual_pos = self.forward_kinematics_RL_fun(H, q_joint)[0:3, 3]
            RR_foot_actual_pos = self.forward_kinematics_RR_fun(H, q_joint)[0:3, 3]

            err_FL = FL_foot_target_position - FL_foot_actual_pos
            err_FR = FR_foot_target_position - FR_foot_actual_pos
            err_RL = RL_foot_target_position - RL_foot_actual_pos
            err_RR = RR_foot_target_position - RR_foot_actual_pos

            err = err_FL + err_FR + err_RL + err_RR
            norm_err = cs.norm_2(err)
            # if(norm_err < eps):
            #    success = True
            #    break

            J_FL = self.jacobian_FL_fun(H, q_joint)[0:3, 6:]
            J_FR = self.jacobian_FR_fun(H, q_joint)[0:3, 6:]
            J_RL = self.jacobian_RL_fun(H, q_joint)[0:3, 6:]
            J_RR = self.jacobian_RR_fun(H, q_joint)[0:3, 6:]

            total_jac = cs.vertcat(J_FL, J_FR, J_RL, J_RR)
            total_err = 100.0 * cs.vertcat(err_FL, err_FR, err_RL, err_RR)
            damped_pinv = cs.inv(total_jac.T @ total_jac + damp_matrix) @ total_jac.T
            v = damped_pinv @ total_err
            q_joint = q_joint + DT * v

            i += 1

        return q_joint


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
    # ADAM needs named foot body frames. Add massless fixed frames exactly at
    # the foot geometry centers, so ADAM and MuJoCo refer to the same points.
    spec = mujoco.MjSpec.from_file(str(model_path))
    for leg in legs:
        geom = spec.geom(cfg.robot_feet_geom_names[leg])
        geom.parent.add_body(name=f"{leg}_foot", pos=geom.pos)
    model = spec.compile()
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
    # Build the symbolic FK/Jacobians from the same model shown in the viewer.
    # The CasADi function performs a fixed number of damped least-squares updates.
    ik = InverseKinematicsNumeric(mujoco_model=model)
    started = time.perf_counter()
    solution = ik.fun_compute_solution(initial_q, *targets)
    elapsed = time.perf_counter() - started
    data.qpos[7:] = np.asarray(solution).reshape(12)

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
