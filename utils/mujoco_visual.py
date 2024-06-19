import mujoco
import numpy as np
from mujoco.viewer import Handle
from scipy.spatial.transform import Rotation

from helpers.swing_trajectory_controller import SwingTrajectoryController
from utils.quadruped_utils import LegsAttr


def cross2(a: np.ndarray, b: np.ndarray) -> np.ndarray:  # See https://github.com/microsoft/pylance-release/issues/3277
    return np.cross(a, b)


def render_vector(viewer: Handle,
                  vector: np.ndarray,
                  pos: np.ndarray,
                  scale: float,
                  color: np.ndarray,
                  geom_id: int = -1) -> int:
    """
    Function to render a vector in the Mujoco viewer.

    Args:
        viewer (Handle): The Mujoco viewer.
        vector (np.ndarray): The vector to render.
        pos (np.ndarray): The position of the base of vector.
        scale (float): The scale of the vector.
        color (np.ndarray): The color of the vector.
        geom_id (int, optional): The id of the geometry. Defaults to -1.
    Returns:
        int: The id of the geometry.
    """
    if geom_id < 0:
        # Instantiate a new geometry
        geom = mujoco.MjvGeom()
        geom.type = mujoco.mjtGeom.mjGEOM_ARROW
        viewer.user_scn.ngeom += 1
        geom_id = viewer.user_scn.ngeom - 1

    geom = viewer.user_scn.geoms[geom_id]

    # Define the a rotation matrix with the Z axis aligned with the vector direction
    vec_z = vector.squeeze() / np.linalg.norm(vector + 1e-5)
    # Define any orthogonal to z vector as the X axis using the Gram-Schmidt process
    rand_vec = np.random.rand(3)
    vec_x = rand_vec - (np.dot(rand_vec, vec_z) * vec_z)
    vec_x = vec_x / np.linalg.norm(vec_x)
    # Define the Y axis as the cross product of X and Z
    vec_y = cross2(vec_z, vec_x)

    ori_mat = Rotation.from_matrix(np.array([vec_x, vec_y, vec_z]).T).as_matrix()
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=np.asarray([0.01, 0.01, scale]),
        pos=pos,
        mat=ori_mat.flatten(),
        rgba=color
        )

    return geom_id


def render_sphere(viewer: Handle,
                  position: np.ndarray,
                  diameter: float,
                  color: np.ndarray,
                  geom_id: int = -1) -> int:
    """
    Function to render a sphere in the Mujoco viewer.

    Args:
        viewer (Handle): The Mujoco viewer.
        position (np.ndarray): The position of the sphere.
        diameter (float): The diameter of the sphere.
        color (np.ndarray): The color of the sphere.
        geom_id (int, optional): The id of the geometry. Defaults to -1.
    Returns:
        int: The id of the geometry.
    """
    if geom_id < 0:
        # Instantiate a new geometry
        geom = mujoco.MjvGeom()
        geom.type = mujoco.mjtGeom.mjGEOM_SPHERE
        viewer.user_scn.ngeom += 1
        geom_id = viewer.user_scn.ngeom - 1

    geom = viewer.user_scn.geoms[geom_id]

    # Initialize the geometry
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.asarray([diameter / 2] * 3),  # Radius is half the diameter
        mat=np.eye(3).flatten(),
        pos=position,
        rgba=color
        )

    return geom_id


def render_line(viewer: Handle,
                initial_point: np.ndarray,
                target_point: np.ndarray,
                width: float,
                color: np.ndarray,
                geom_id: int = -1) -> int:
    """
    Function to render a line in the Mujoco viewer.

    Args:
        viewer (Handle): The Mujoco viewer.
        initial_point (np.ndarray): The initial point of the line.
        target_point (np.ndarray): The target point of the line.
        width (float): The width of the line.
        color (np.ndarray): The color of the line.
        geom_id (int, optional): The id of the geometry. Defaults to -1.
    Returns:
        int: The id of the geometry.
    """
    if geom_id < 0:
        # Instantiate a new geometry
        geom = mujoco.MjvGeom()
        geom.type = mujoco.mjtGeom.mjGEOM_LINE
        viewer.user_scn.ngeom += 1
        geom_id = viewer.user_scn.ngeom - 1

    geom = viewer.user_scn.geoms[geom_id]

    # Define the a rotation matrix with the Z axis aligned with the line direction
    vector = target_point - initial_point
    vec_z = vector.squeeze() / np.linalg.norm(vector + 1e-5)
    # Define any orthogonal to z vector as the X axis using the Gram-Schmidt process
    rand_vec = np.random.rand(3)
    vec_x = rand_vec - (np.dot(rand_vec, vec_z) * vec_z)
    vec_x = vec_x / np.linalg.norm(vec_x)
    # Define the Y axis as the cross product of X and Z
    vec_y = cross2(vec_z, vec_x)

    ori_mat = Rotation.from_matrix(np.array([vec_x, vec_y, vec_z]).T).as_matrix()
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_LINE,
        size=np.asarray([width, 0.1, np.linalg.norm(vector)]),
        pos=initial_point,
        mat=ori_mat.flatten(),
        rgba=color
        )

    return geom_id


def plot_swing_mujoco(viewer: Handle,
                      swing_traj_controller: SwingTrajectoryController,
                      swing_period,
                      swing_time: LegsAttr,
                      lift_off_positions: LegsAttr,
                      nmpc_footholds: LegsAttr,
                      ref_feet_pos: LegsAttr,
                      geom_ids: LegsAttr = None,
                      ):
    """
       Function to plot the desired foot swing trajectory in Mujoco.
       Args:
           viewer (Handle): The Mujoco viewer.
           swing_traj_controller (SwingTrajectoryController): The swing trajectory controller.
           swing_period: The swing period.
           swing_time (LegsAttr): The swing time for each leg.
           lift_off_positions (LegsAttr): The lift-off positions for each leg.
           nmpc_footholds (LegsAttr): The footholds for each leg.
           ref_feet_pos (LegsAttr): The reference feet positions for each leg.
           geom_ids (LegsAttr, optional): The geometry ids for each leg. Defaults to None.

       Returns:
           LegsAttr: The geometry ids for each leg trajectory
       """

    # Plot desired foot swing trajectory in Mujoco.
    NUM_TRAJ_POINTS = 6

    if geom_ids is None:
        geom_ids = LegsAttr(FL=[], FR=[], RL=[], RR=[])
        # Instantiate a new geometry
        for leg_id, leg_name in enumerate(['FL', 'FR', 'RL', 'RR']):
            viewer.user_scn.ngeom += NUM_TRAJ_POINTS
            geom_ids[leg_name] = list(range(viewer.user_scn.ngeom - NUM_TRAJ_POINTS - 1, viewer.user_scn.ngeom - 1))

    # viewer.user_scn.ngeom = 1
    # We first draw the trajectory of the feet
    des_foot_traj = LegsAttr(FL=[], FR=[], RL=[], RR=[])
    for leg_id, leg_name in enumerate(['FL', 'FR', 'RL', 'RR']):
        # TODO: This function should be vectorized rather than queried sequentially
        for point_idx, foot_swing_time in enumerate(np.linspace(swing_time[leg_name], swing_period, NUM_TRAJ_POINTS)):
            ref_foot_pos, _, _ = swing_traj_controller.swing_generator.compute_trajectory_references(
                foot_swing_time,
                lift_off_positions[leg_name],
                nmpc_footholds[leg_name])
            des_foot_traj[leg_name].append(ref_foot_pos.squeeze())

        for point_idx in range(NUM_TRAJ_POINTS - 1):
            render_line(viewer=viewer,
                        initial_point=des_foot_traj[leg_name][point_idx],
                        target_point=des_foot_traj[leg_name][point_idx + 1],
                        width=0.1,
                        color=np.array([1, 0, 0, 1]),
                        geom_id=geom_ids[leg_name][point_idx]
                        )

        # Add a sphere at the the ref_feet_pos
        render_sphere(viewer=viewer,
                      position=ref_feet_pos[leg_name],
                      diameter=0.04,
                      color=np.array([0, 1, 0, .5]),
                      geom_id=geom_ids[leg_name][-1]
                      )
    return geom_ids
