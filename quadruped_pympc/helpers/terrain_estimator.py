import numpy as np


class TerrainEstimator:
    def __init__(self) -> None:
        self.terrain_roll = 0
        self.terrain_pitch = 0
        self.terrain_height = 0
        self.robot_height = 0

        self.roll_activated = False
        self.pitch_activated = True

    def compute_terrain_estimation(
        self, base_position: np.ndarray, yaw: float, feet_pos: dict, current_contact: np.ndarray
    ) -> [float, float]:
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
        R_W2H = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Extracting 3-element segments from liftoff_position_z_ and x_op_
        seg0 = feet_pos["FL"]
        seg3 = feet_pos["FR"]
        seg6 = feet_pos["RL"]
        seg9 = feet_pos["RR"]

        # Calculating differences
        # TODO: Feet position in base frame?
        front_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg3 - base_position)
        back_difference = R_W2H @ (seg6 - base_position) - R_W2H @ (seg9 - base_position)
        left_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg6 - base_position)
        right_difference = R_W2H @ (seg3 - base_position) - R_W2H @ (seg9 - base_position)

        # Calculating pitch and roll
        # TODO: Docstring
        pitch = (
            np.arctan(np.abs(left_difference[2]) / np.abs(left_difference[0] + 0.001))
            + np.arctan(np.abs(right_difference[2]) / np.abs(right_difference[0] + 0.001))
        ) * 0.5

        roll = (
            np.arctan(np.abs(front_difference[2]) / np.abs(front_difference[1] + 0.001))
            + np.arctan(np.abs(back_difference[2]) / np.abs(back_difference[1] + 0.001))
        ) * 0.5

        # Adjusting signs of pitch and roll TODO: Adjusting what and for what?
        if (front_difference[2] * 0.5 + back_difference[2] * 0.5) < 0:
            roll = -roll
        if (left_difference[2] * 0.5 + right_difference[2] * 0.5) > 0:
            pitch = -pitch


        if self.roll_activated:
            self.terrain_roll = self.terrain_roll * 0.99 + roll * 0.01
        else:
            self.terrain_roll = 0.0
        
        if self.pitch_activated:
            self.terrain_pitch = self.terrain_pitch * 0.99 + pitch * 0.01
        else:
            self.terrain_pitch = 0.0

        # Update the reference height given the foot in contact
        z_foot_FL = feet_pos["FL"][2]
        z_foot_FR = feet_pos["FR"][2]
        z_foot_RL = feet_pos["RL"][2]
        z_foot_RR = feet_pos["RR"][2]
        """number_foot_in_contact = current_contact[0] + \
                                 current_contact[1] + \
                                 current_contact[2] + \
                                 current_contact[3]
        if (number_foot_in_contact != 0):
            z_foot_mean_temp = (z_foot_FL * current_contact[0] + \
                                z_foot_FR * current_contact[1] + \
                                z_foot_RL * current_contact[2] + \
                                z_foot_RR * current_contact[3]) / number_foot_in_contact
            self.terrain_height = self.terrain_height * 0.6 + z_foot_mean_temp * 0.4"""

        z_foot_mean_temp = (z_foot_FL + z_foot_FR + z_foot_RL + z_foot_RR) / 4
        #self.terrain_height = self.terrain_height * 0.2 + (base_position[2] - z_foot_mean_temp) * 0.8
        self.terrain_height = self.terrain_height * 0.2 + (z_foot_mean_temp) * 0.8


        feet_to_base_FL = base_position[2] - feet_pos["FL"][2]
        feet_to_base_FR = base_position[2] - feet_pos["FR"][2]
        feet_to_base_RL = base_position[2] - feet_pos["RL"][2]
        feet_to_base_RR = base_position[2] - feet_pos["RR"][2] 
        feet_to_base_mean = (feet_to_base_FL + feet_to_base_FR + feet_to_base_RL + feet_to_base_RR) / 4
        self.robot_height = self.robot_height * 0.2 + (feet_to_base_mean) * 0.8


        return self.terrain_roll, self.terrain_pitch, self.terrain_height, self.robot_height


if __name__ == "__main__":
    # Illustrate the estimator with noisy feet on a piecewise planar terrain.
    import matplotlib.pyplot as plt

    estimator = TerrainEstimator()
    estimator.roll_activated = True  # Disabled by default; enable it for the demo.
    rng = np.random.default_rng(7)

    # Time is illustrative: the estimator filters once per call, without a dt.
    # At 100 Hz, the angle filter (0.99 old + 0.01 new) takes about 1 s to respond.
    time = np.arange(1600) * 0.01
    phase = np.where(time < 2.0, 0, np.where(time < 9.0, 1, 2))
    true_roll = np.deg2rad(np.array([0.0, 8.0, -5.0])[phase])
    true_pitch = np.deg2rad(np.array([0.0, -12.0, 7.0])[phase])
    true_height = np.array([0.0, 0.08, 0.03])[phase]
    clearance = 0.35  # Vertical base-to-terrain distance, not normal distance.

    names = ("FL", "FR", "RL", "RR")
    feet_xy = np.array([[0.30, 0.18], [0.30, -0.18], [-0.30, 0.18], [-0.30, -0.18]])
    yaw = np.deg2rad(30.0)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    origin = np.array([0.5, -0.2])
    world_xy = feet_xy @ rotation.T + origin
    contact = np.ones(4)
    estimates = np.empty((len(time), 4))

    # The heading frame has the base yaw, but no roll/pitch. Match the estimator's
    # slope conventions: z = h - tan(pitch)*x_heading + tan(roll)*y_heading.
    # These are directional slope angles, not a full Euler-angle decomposition.
    # All feet stay on the plane: current_contact is currently unused by the
    # estimator, so lifting a foot would also contaminate angles and mean height.
    for i in range(len(time)):
        feet_z = (
            true_height[i]
            - np.tan(true_pitch[i]) * feet_xy[:, 0]
            + np.tan(true_roll[i]) * feet_xy[:, 1]
        )
        measured_feet = np.column_stack((world_xy, feet_z + rng.normal(0.0, 0.002, 4)))
        feet = dict(zip(names, measured_feet))
        base = np.r_[origin, true_height[i] + clearance]
        estimates[i] = estimator.compute_terrain_estimation(base, yaw, feet, contact)

    fig = plt.figure(figsize=(13, 9), layout="constrained")
    ax_3d = fig.add_subplot(2, 2, 1, projection="3d")
    x, y = np.meshgrid(np.linspace(-0.4, 0.4, 15), np.linspace(-0.28, 0.28, 15))
    grid_world = np.stack((x, y), axis=-1) @ rotation.T + origin
    z = true_height[-1] - np.tan(true_pitch[-1]) * x + np.tan(true_roll[-1]) * y
    ax_3d.plot_surface(grid_world[..., 0], grid_world[..., 1], z, alpha=0.35, color="tab:green")
    ax_3d.scatter(*measured_feet.T, color="tab:blue", label="Measured feet")
    for name, position in feet.items():
        ax_3d.text(*position, name)
    ax_3d.scatter(*base, color="tab:red", label="Base")
    ax_3d.set(xlabel="World x [m]", ylabel="World y [m]", zlabel="World z [m]",
              title="Final terrain and feet (yaw = 30 deg)")
    ax_3d.legend()

    ax_roll = fig.add_subplot(2, 2, 2)
    ax_pitch = fig.add_subplot(2, 2, 3, sharex=ax_roll)
    for ax, reference, column, name in (
        (ax_roll, true_roll, 0, "Roll"),
        (ax_pitch, true_pitch, 1, "Pitch"),
    ):
        ax.plot(time, np.rad2deg(reference), "k--", label="True slope")
        ax.plot(time, np.rad2deg(estimates[:, column]), label="Filtered estimate")
        ax.set(title=f"{name}: filter convergence", xlabel="Time [s]", ylabel="Angle [deg]")
        ax.grid(alpha=0.3)
        ax.legend()

    ax_height = fig.add_subplot(2, 2, 4, sharex=ax_roll)
    ax_height.plot(time, true_height, "--", color="tab:green", label="True terrain height")
    ax_height.plot(time, estimates[:, 2], color="tab:green", label="Estimated terrain height")
    ax_height.axhline(clearance, linestyle="--", color="tab:purple", label="True base clearance")
    ax_height.plot(time, estimates[:, 3], color="tab:purple", label="Estimated base clearance")
    ax_height.set(title="Heights: faster filter response", xlabel="Time [s]", ylabel="Height [m]")
    ax_height.grid(alpha=0.3)
    ax_height.legend()
    fig.suptitle("TerrainEstimator demo — 100 Hz, foot-height noise: 2 mm")

    plt.show()
    plt.close(fig)
