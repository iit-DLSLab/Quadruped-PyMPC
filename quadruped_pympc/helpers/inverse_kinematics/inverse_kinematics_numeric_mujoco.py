import numpy as np

np.set_printoptions(precision=3, suppress=True)
from numpy.linalg import norm, solve
import time
import casadi as cs

# import example_robot_data as robex
import copy
import os
import time

import gym_quadruped

# Mujoco magic
import mujoco
import mujoco.viewer

# Adam and Liecasadi magic

import gym_quadruped
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
gym_quadruped_path = os.path.dirname(gym_quadruped.__file__)


from quadruped_pympc import config as cfg

from gym_quadruped.quadruped_env import QuadrupedEnv

from quadruped_pympc import config as cfg

IT_MAX = 5
DT = 1e-2
TOLERANCE = 2e-3  # [m] stop iterating when every foot is closer than this to its target
damp = 1e-3
damp_matrix = damp * np.eye(12)


# Class for solving a generic inverse kinematics problem
class InverseKinematicsNumeric:
    def __init__(self) -> None:
        """
        This method initializes the inverse kinematics solver class.

        Args:

        """

        robot_name = cfg.robot

        # Create the quadruped robot environment ---------------------
        self.env = QuadrupedEnv(
            robot=robot_name,
        )

    def _update_kinematics(self):
        # Only what the feet positions and Jacobians need, instead of the full mj_fwdPosition
        mujoco.mj_kinematics(self.env.mjModel, self.env.mjData)
        mujoco.mj_comPos(self.env.mjModel, self.env.mjData)

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

        # Set the initial states
        self.env.mjData.qpos = q
        self._update_kinematics()

        q_joint = self.env.mjData.qpos[7:].copy()
        for j in range(IT_MAX):
            feet_pos = self.env.feet_pos(frame='world')

            FL_foot_actual_pos = feet_pos.FL
            FR_foot_actual_pos = feet_pos.FR
            RL_foot_actual_pos = feet_pos.RL
            RR_foot_actual_pos = feet_pos.RR

            err_FL = FL_foot_target_position - FL_foot_actual_pos
            err_FR = FR_foot_target_position - FR_foot_actual_pos
            err_RL = RL_foot_target_position - RL_foot_actual_pos
            err_RR = RR_foot_target_position - RR_foot_actual_pos

            # Already close enough to the targets, no need for other iterations
            if max(norm(err_FL), norm(err_FR), norm(err_RL), norm(err_RR)) < TOLERANCE:
                break

            # Compute feet jacobian
            feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)
        
            J_FL = feet_jac.FL[:, 6:]
            J_FR = feet_jac.FR[:, 6:]
            J_RL = feet_jac.RL[:, 6:]
            J_RR = feet_jac.RR[:, 6:]

            total_jac = np.vstack((J_FL, J_FR, J_RL, J_RR))
            total_err = 100*np.hstack((err_FL, err_FR, err_RL, err_RR))

            # Solve the damped normal equations directly; only one RHS is needed.
            dq = np.linalg.solve(total_jac.T @ total_jac + damp_matrix, total_jac.T @ total_err)

            # Integrate joint velocities to obtain joint positions.
            q_joint = self.env.mjData.qpos[7:].copy()
            q_joint += dq * DT
            self.env.mjData.qpos[7:] = q_joint

            # The kinematics after the last iteration is not needed
            if j < IT_MAX - 1:
                self._update_kinematics()
            #mujoco.mj_kinematics(self.env.mjModel, self.env.mjData)
            #mujoco.mj_step(self.env.mjModel, self.env.mjData)

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
    # This solver uses MuJoCo foot Jacobians and damped least-squares updates.
    ik = InverseKinematicsNumeric()
    started = time.perf_counter()
    solution = ik.compute_solution(initial_q.copy(), *targets)
    elapsed = time.perf_counter() - started
    # The numeric solver returns only joint angles; preserve the original base pose.
    data.qpos[7:] = solution

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
