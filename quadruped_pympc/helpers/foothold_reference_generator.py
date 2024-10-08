import collections

import mujoco
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr


# Class for the generation of the reference footholds
# TODO: @Giulio Should we convert this to a single function instead of a class? Stance time, can be passed as argument
class FootholdReferenceGenerator:

    def __init__(self, stance_time: float, lift_off_positions: LegsAttr, vel_moving_average_length=20,
                 hip_height: float = None) -> None:
        """This method initializes the foothold generator class, which computes
        the reference foothold for the nonlinear MPC.

        Args:
        ----
            stance_time: The user-defined time of the stance phase.
        """
        self.base_vel_hist = collections.deque(maxlen=vel_moving_average_length)
        self.stance_time = stance_time
        self.hip_height = hip_height
        self.lift_off_positions = lift_off_positions

        # The footholds are wrt the hip position, so if we want to change
        # the default foothold, we need to use a variable to add an offset
        self.hip_offset = 0.1

    def compute_footholds_reference(self,
                                    com_position: np.ndarray,
                                    base_ori_euler_xyz: np.ndarray,
                                    base_xy_lin_vel: np.ndarray,
                                    ref_base_xy_lin_vel: np.ndarray,
                                    hips_position: LegsAttr,
                                    com_height_nominal: np.float32) -> LegsAttr:
        """Compute the reference footholds for a quadruped robot, using simple geometric heuristics.

        TODO: This function should be adapted to:
           1. Use the terrain slope estimation
           2. Use the desired base angular (yaw_dot) velocity to compensate for the error in the velocity.
            Similar to the linear velocity compensation.

        TODO: This function should be vectorized so operations are not done for each leg separately.

        Args:
        ----
            com_position: (3,) The position of the center of mass of the robot.
            base_ori_euler_xyz: (3,) The orientation of the base in euler angles.
            base_xy_lin_vel: (2,) The [x,y] linear velocity of the base in world frame.
            ref_base_xy_lin_vel: (2,) The desired [x,y] linear velocity of the base in world frame.
            hips_position: (LegsAttr) The position of the hips of the robot in world frame.

        Returns:
        -------
            ref_feet: (LegsAttr) The reference footholds for the robot in world frame.
        """
        assert base_xy_lin_vel.shape == (2,) and ref_base_xy_lin_vel.shape == (2,), \
            f"Expected shape (2,):=[x_dot, y_dot], got {base_xy_lin_vel.shape} and {ref_base_xy_lin_vel.shape}."

        # Get the rotation matrix to transform from world to horizontal frame (hip-centric)
        yaw = base_ori_euler_xyz[2]
        R_W2H = np.array([np.cos(yaw), np.sin(yaw),
                          -np.sin(yaw), np.cos(yaw)])
        R_W2H = R_W2H.reshape((2, 2))

        # Compute desired and error velocity compensation values for all legs
        base_lin_vel_H = R_W2H @ base_xy_lin_vel
        ref_base_lin_vel_H = R_W2H @ ref_base_xy_lin_vel

        # Moving average of the base velocity
        self.base_vel_hist.append(base_lin_vel_H)
        base_vel_mvg = np.mean(list(self.base_vel_hist), axis=0)
        # Compensation due to average velocity
        #delta_ref_H = (self.stance_time / 2.) * base_vel_mvg
        
        # Compensation due to desired velocity
        delta_ref_H = (self.stance_time / 2.) * ref_base_lin_vel_H
        delta_ref_H = np.clip(delta_ref_H, -self.hip_height * 1.5, self.hip_height * 1.5)
        vel_offset = np.concatenate((delta_ref_H, np.zeros(1)))


        # Compensation for the error in velocity tracking
        error_compensation = np.sqrt(com_height_nominal/9.81)*(base_vel_mvg - ref_base_lin_vel_H) 
        error_compensation = np.where(error_compensation > 0.05, 0.05, error_compensation)
        error_compensation = np.where(error_compensation < -0.05, -0.05, error_compensation)
        error_compensation = np.concatenate((error_compensation, np.zeros(1)))



        # Reference footholds in the horizontal frame
        ref_feet = LegsAttr(*[np.zeros(3) for _ in range(4)])

        # Reference feet positions are computed from the hips x,y position in the hip-centric/Horizontal frame
        ref_feet.FL[0:2] = R_W2H @ (hips_position.FL[0:2] - com_position[0:2])
        ref_feet.FR[0:2] = R_W2H @ (hips_position.FR[0:2] - com_position[0:2])
        ref_feet.RL[0:2] = R_W2H @ (hips_position.RL[0:2] - com_position[0:2])
        ref_feet.RR[0:2] = R_W2H @ (hips_position.RR[0:2] - com_position[0:2])
        # Offsets are introduced to account for x,y offsets from nominal hip and feet positions.
        # Offsets to the Y axis result in wider/narrower stance (+y values lead to wider stance in left/right)
        # Offsets to the X axis result in spread/crossed legs (+x values lead to spread legs in front/back)
        # TODO: This should not be hardcoded, should be a property of the robot cofiguration and passed as argment
        #  to this function, not loaded from the config file.
        ref_feet.FL[1] += self.hip_offset
        ref_feet.FR[1] -= self.hip_offset
        ref_feet.RL[1] += self.hip_offset
        ref_feet.RR[1] -= self.hip_offset

        

        # Add the velocity compensation and desired velocity to the feet positions
        ref_feet += vel_offset + error_compensation  # Add offset to all feet

        # Reference footholds in world frame
        ref_feet.FL[0:2] = R_W2H.T @ ref_feet.FL[:2] + com_position[0:2]
        ref_feet.FR[0:2] = R_W2H.T @ ref_feet.FR[:2] + com_position[0:2]
        ref_feet.RL[0:2] = R_W2H.T @ ref_feet.RL[:2] + com_position[0:2]
        ref_feet.RR[0:2] = R_W2H.T @ ref_feet.RR[:2] + com_position[0:2]

        # TODO: we should rotate them considering the terrain estimator maybe
        #   or we can just do exteroceptive height adjustement...
        for leg_id in ['FL', 'FR', 'RL', 'RR']:
            ref_feet[leg_id][2] = self.lift_off_positions[leg_id][2]# - 0.02

        return ref_feet

    def update_lift_off_positions(self, previous_contact, current_contact, feet_pos, legs_order):
        for leg_id, leg_name in enumerate(legs_order):
            # Set lif-offs
            if previous_contact[leg_id] == 1 and current_contact[leg_id] == 0:
                self.lift_off_positions[leg_name] = feet_pos[leg_name]


if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path('./../simulation/unitree_go1/scene.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    FL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_hip')
    FR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_hip')
    RL_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_hip')
    RR_hip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_hip')

    hip_pos = np.array(([d.body(FL_hip_id).xpos],
                        [d.body(FR_hip_id).xpos],
                        [d.body(RL_hip_id).xpos],
                        [d.body(RR_hip_id).xpos]))

    print("hip_pos", hip_pos)

    stance_time = 0.5
    linear_com_velocity = np.array([0.1, 0.0, 0.0])
    desired_linear_com_velocity = np.array([0.1, 0.0, 0.0])
    com_height = d.qpos[2]

    foothold_generator = FootholdReferenceGenerator(stance_time)
    footholds_reference = foothold_generator.compute_footholds_reference(linear_com_velocity[0:2],
                                                                         desired_linear_com_velocity[0:2],
                                                                         hip_pos, com_height)
    print("iniztial hip_pos: ", hip_pos)
    print("footholds_reference: ", footholds_reference)
