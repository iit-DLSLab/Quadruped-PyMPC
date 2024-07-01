import numpy as np


class TerrainEstimator:
    def __init__(self, ) -> None:

        self.terrain_roll = 0
        self.terrain_pitch = 0
        self.terrain_height = 0

    def compute_terrain_estimation(self,
                                   base_position: np.ndarray,
                                   yaw: float,
                                   feet_pos: dict,
                                   current_contact: np.ndarray) -> [float, float]:
        """Compute the estimated roll and pitch of the terrain based on the positions of the robot's feet.

        Parameters
        ----------
            base_position : (np.ndarray) The position of the robot's base in the world frame.
            yaw : (float) The yaw angle of the robot in radians.
            lifted_foot_positions : (dict) The positions of the robot's feet in the world frame. This is an instance of
            the dict class, which has attributes for the positions of the front left (FL), front right (FR), rear left
            (RL),and rear right (RR) feet.
            current_contact: (np.ndarray) The contact state of the feet. This is a 4-element array

        Returns
        -------
        roll : (float) The estimated roll of the terrain in radians.
        pitch : (float) The estimated pitch of the terrain in radians.

        Notes
        -----
        This function assumes that the robot is on a planar terrain and that the feet positions are measured in the
        world frame.
        """
        # Compute roll and pitch for each foot position
        # Rotation matrix R_yaw
        R_W2H = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
            ])

        # Extracting 3-element segments from liftoff_position_z_ and x_op_
        seg0 = feet_pos['FL']
        seg3 = feet_pos['FR']
        seg6 = feet_pos['RL']
        seg9 = feet_pos['RR']

        # Calculating differences
        # TODO: Feet position in base frame?
        front_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg3 - base_position)
        back_difference = R_W2H @ (seg6 - base_position) - R_W2H @ (seg9 - base_position)
        left_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg6 - base_position)
        right_difference = R_W2H @ (seg3 - base_position) - R_W2H @ (seg9 - base_position)

        # Calculating pitch and roll
        # TODO: Docstring
        pitch = (np.arctan(np.abs(left_difference[2]) / np.abs(left_difference[0] + 0.001)) +
                 np.arctan(np.abs(right_difference[2]) / np.abs(right_difference[0] + 0.001))) * 0.5

        roll = (np.arctan(np.abs(front_difference[2]) / np.abs(front_difference[1] + 0.001)) +
                np.arctan(np.abs(back_difference[2]) / np.abs(back_difference[1] + 0.001))) * 0.5

        # Adjusting signs of pitch and roll TODO: Adjusting what and for what?
        if (front_difference[2] * 0.5 + back_difference[2] * 0.5) < 0:
            roll = -roll
        if (left_difference[2] * 0.5 + right_difference[2] * 0.5) > 0:
            pitch = -pitch

        self.terrain_roll = self.terrain_roll * 0.8 + roll * 0.2
        self.terrain_pitch = self.terrain_pitch * 0.8 + pitch * 0.2

        # Update the reference height given the foot in contact
        z_foot_FL = feet_pos['FL'][2]
        z_foot_FR = feet_pos['FR'][2]
        z_foot_RL = feet_pos['RL'][2]
        z_foot_RR = feet_pos['RR'][2]
        number_foot_in_contact = current_contact[0] + \
                                 current_contact[1] + \
                                 current_contact[2] + \
                                 current_contact[3]
        if (number_foot_in_contact != 0):
            z_foot_mean_temp = (z_foot_FL * current_contact[0] + \
                                z_foot_FR * current_contact[1] + \
                                z_foot_RL * current_contact[2] + \
                                z_foot_RR * current_contact[3]) / number_foot_in_contact
            self.terrain_height = self.terrain_height * 0.6 + z_foot_mean_temp * 0.4

        return self.terrain_roll, self.terrain_pitch, self.terrain_height
