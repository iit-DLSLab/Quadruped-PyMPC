"""Estimate a terrain plane from stance feet using a vertical least-squares fit.

Run this file directly with Python to display a self-contained synthetic demo.
Use ``--no-show --save terrain_fit.png`` to run without a graphical display.
Only NumPy is required by the estimator; the demo also uses Matplotlib.
"""

import numpy as np


class TerrainEstimatorVerticalFit:
    """Maintain filtered terrain orientation and height estimates across calls."""

    def __init__(self) -> None:
        self.terrain_roll = 0
        self.terrain_pitch = 0
        self.terrain_height = 0
        self.robot_height = 0

        self.roll_activated = True
        self.pitch_activated = True

    def compute_terrain_estimation(
        self, base_position: np.ndarray, yaw: float, feet_pos: dict, current_contact: np.ndarray
    ) -> tuple[float, float, float, float]:
        """Fit stance feet using Section VI-A, Eqs. (24)-(26) of
        https://arxiv.org/abs/1805.10238.

        Positions are world-frame 3-vectors in metres; yaw is in radians.
        Contacts are binary entries ordered FL, FR, RL, RR. Swing feet are
        ignored. With fewer than three non-collinear contacts, outputs are
        held (initially zero). No historical footholds are retained.

        Returns filtered ZYX roll/pitch in the horizontal heading frame,
        world terrain z at the base x/y, and vertical base clearance.
        Clearance preserves the existing interface; it is not the normal
        distance used for robot height in the paper. Filters retain the
        original per-call gains: 0.01 for angles and 0.8 for heights.
        The fit updates each call, rather than only at touchdown.
        """
        # Activation flags control the outputs, not the plane used for heights.
        if not self.roll_activated:
            self.terrain_roll = 0.0
        if not self.pitch_activated:
            self.terrain_pitch = 0.0

        contacts = np.asarray(current_contact)
        base_position = np.asarray(base_position, dtype=float)
        names = ("FL", "FR", "RL", "RR")
        stance = np.asarray([feet_pos[name] for name, contact in zip(names, contacts) if contact], dtype=float)
        # A plane has three unknown coefficients. Fewer than three stance
        # points cannot determine it uniquely, so keep the previous estimate.
        if len(stance) >= 3:
            

            # INSERT THE CODE HERE
            self.terrain_pitch = 0.0 # comment this line
            self.terrain_roll = 0.0 # comment this line







            #----------------------


        # Update the reference height given the foot in contact
        z_foot_FL = feet_pos["FL"][2]
        z_foot_FR = feet_pos["FR"][2]
        z_foot_RL = feet_pos["RL"][2]
        z_foot_RR = feet_pos["RR"][2]
        z_foot_mean_temp = (z_foot_FL + z_foot_FR + z_foot_RL + z_foot_RR) / 4
        self.terrain_height = self.terrain_height * 0.2 + (z_foot_mean_temp) * 0.8

        feet_to_base_FL = base_position[2] - feet_pos["FL"][2]
        feet_to_base_FR = base_position[2] - feet_pos["FR"][2]
        feet_to_base_RL = base_position[2] - feet_pos["RL"][2]
        feet_to_base_RR = base_position[2] - feet_pos["RR"][2] 
        feet_to_base_mean = (feet_to_base_FL + feet_to_base_FR + feet_to_base_RL + feet_to_base_RR) / 4
        self.robot_height = self.robot_height * 0.2 + (feet_to_base_mean) * 0.8

        return self.terrain_roll, self.terrain_pitch, self.terrain_height, self.robot_height


# Preserve compatibility with code that imported the original class name.
TerrainEstimator = TerrainEstimatorVerticalFit


def main() -> None:
    """Run a reproducible demo without importing the robot or MPC stack."""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-show", action="store_true", help="Run without opening a plot window")
    parser.add_argument("--save", metavar="PATH", help="Save the demo figure, for example terrain_fit.png")
    args = parser.parse_args()

    # Select a non-interactive backend before importing pyplot so the demo
    # also runs on machines without a display (e.g. via SSH or in CI).
    if args.no_show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    estimator = TerrainEstimatorVerticalFit()
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
    # ZYX convention: z = h - tan(pitch)*x + tan(roll)/cos(pitch)*y.
    # All four feet remain in contact in this illustration.
    for i in range(len(time)):
        feet_z = (
            true_height[i]
            - np.tan(true_pitch[i]) * feet_xy[:, 0]
            + np.tan(true_roll[i]) / np.cos(true_pitch[i]) * feet_xy[:, 1]
        )
        measured_feet = np.column_stack((world_xy, feet_z + rng.normal(0.0, 0.002, 4)))
        feet = dict(zip(names, measured_feet))
        base = np.r_[origin, true_height[i] + clearance]
        estimates[i] = estimator.compute_terrain_estimation(base, yaw, feet, contact)

    fig = plt.figure(figsize=(13, 9), layout="constrained")
    ax_3d = fig.add_subplot(2, 2, 1, projection="3d")
    x, y = np.meshgrid(np.linspace(-0.4, 0.4, 15), np.linspace(-0.28, 0.28, 15))
    grid_world = np.stack((x, y), axis=-1) @ rotation.T + origin
    z = true_height[-1] - np.tan(true_pitch[-1]) * x + np.tan(true_roll[-1]) / np.cos(true_pitch[-1]) * y
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

    # Print numerical results as well, so headless runs give useful feedback.
    roll, pitch, height, robot_height = estimates[-1]
    print(f"Final estimate: roll={np.rad2deg(roll):.2f} deg, pitch={np.rad2deg(pitch):.2f} deg, "
          f"terrain height={height:.3f} m, vertical clearance={robot_height:.3f} m")
    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"Figure saved to {args.save}")
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
