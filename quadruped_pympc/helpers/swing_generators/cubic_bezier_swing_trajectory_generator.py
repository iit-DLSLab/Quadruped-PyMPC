import matplotlib.pyplot as plt
import numpy as np


class SwingTrajectoryGenerator:
    def __init__(self, step_height: float, swing_period: float) -> None:
        self.step_height = step_height
        self.swing_period = swing_period
        self.half_swing_period = swing_period / 2
        self.bezier_time_factor = 1 / (swing_period / 2)

    def plot_trajectory_3d(self, curve_points: np.ndarray) -> None:
        curve_points = np.array(curve_points)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2])
        ax.legend()

        plt.title("3D Bézier Curve")
        plt.show()

    def plot_trajectory_references(self, tp, fp, vp, ap):
        # Convert lists to NumPy arrays for easier plotting
        time_points = np.array(tp)
        footPosDes_points = np.array(fp)
        footVelDes_points = np.array(vp)
        footAccDes_points = np.array(ap)

        # Create subplots for position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(8, 12))

        # Plot position
        for i in range(3):
            axs[0].plot(time_points, footPosDes_points[:, i], label=f"Position {i + 1}")
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('Position')
        axs[0].legend()

        # Plot velocity
        for i in range(3):
            axs[1].plot(time_points, footVelDes_points[:, i], label=f"Velocity {i + 1}")
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Velocity')
        axs[1].legend()

        # Plot acceleration
        for i in range(3):
            axs[2].plot(time_points, footAccDes_points[:, i], label=f"Acceleration {i + 1}")
        axs[2].set_xlabel('Time')
        axs[2].set_ylabel('Acceleration')
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    def compute_control_points(
        self, swing_time: float, lift_off: np.ndarray, touch_down: np.ndarray
    ) -> (np.array, np.array, np.array, np.array):
        cp1, cp2, cp3, cp4 = None, None, None, None
        middle_point = 0.5 * (lift_off + touch_down)

        if swing_time <= self.half_swing_period:
            cp1 = lift_off
            cp2 = lift_off
            cp3 = np.array([[lift_off[0], lift_off[1], self.step_height]])
            cp4 = np.array([[middle_point[0], middle_point[1], self.step_height]])
        else:
            cp1 = np.array([[middle_point[0], middle_point[1], self.step_height]])
            cp2 = np.array([[touch_down[0], touch_down[1], self.step_height]])
            cp3 = np.array([[touch_down[0], touch_down[1], touch_down[2]]])
            cp4 = np.array([[touch_down[0], touch_down[1], touch_down[2]]])

        # Keep the endpoint of each half instead of wrapping it back to zero.
        local_time = swing_time if swing_time <= self.half_swing_period else swing_time - self.half_swing_period
        return local_time, cp1, cp2, cp3, cp4

    def compute_trajectory_references(
        self, swing_time: float, lift_off: np.ndarray, touch_down: np.ndarray, early_stance_hitmoment = -1, early_stance_hitpoint = None
    ) -> (np.ndarray, np.ndarray, np.ndarray):
        bezier_swing_time, cp1, cp2, cp3, cp4 = self.compute_control_points(
            swing_time, lift_off.squeeze(), touch_down.squeeze()
        )

        desired_foot_position = (
            (1 - self.bezier_time_factor * bezier_swing_time) ** 3 * cp1
            + 3
            * (self.bezier_time_factor * bezier_swing_time)
            * (1 - self.bezier_time_factor * bezier_swing_time) ** 2
            * cp2
            + 3
            * (self.bezier_time_factor * bezier_swing_time) ** 2
            * (1 - self.bezier_time_factor * bezier_swing_time)
            * cp3
            + (self.bezier_time_factor * bezier_swing_time) ** 3 * cp4
        )

        desired_foot_velocity = (
            3 * (1 - self.bezier_time_factor * bezier_swing_time) ** 2 * (cp2 - cp1)
            + 6
            * (1 - self.bezier_time_factor * bezier_swing_time)
            * (self.bezier_time_factor * bezier_swing_time)
            * (cp3 - cp2)
            + 3 * (self.bezier_time_factor * bezier_swing_time) ** 2 * (cp4 - cp3)
        )

        desired_foot_acceleration = 6 * (1 - self.bezier_time_factor * bezier_swing_time) * (
            cp3 - 2 * cp2 + cp1
        ) + 6 * (self.bezier_time_factor * bezier_swing_time) * (cp4 - 2 * cp3 + cp2)

        # Convert derivatives with respect to normalized Bézier time to seconds.
        desired_foot_velocity *= self.bezier_time_factor
        desired_foot_acceleration *= self.bezier_time_factor**2

        return (
            desired_foot_position.reshape((3,)),
            desired_foot_velocity.reshape((3,)),
            desired_foot_acceleration.reshape((3,)),
        )


if __name__ == "__main__":
    # Example swing: distances are in metres and times are in seconds.
    # Change these parameters to inspect a different step or landing position.
    step_height = 0.08
    swing_period = 0.9
    simulation_dt = 0.002
    lift_off = np.array([0.0, 0.0, 0.0])
    touch_down = np.array([0.1, -0.2, 0.0])
    trajectory_generator = SwingTrajectoryGenerator(step_height=step_height, swing_period=swing_period)

    # Include both lift-off (t = 0) and touchdown (t = swing_period).
    # Use the same timestamps for evaluation and plotting to avoid a sample delay.
    num_intervals = int(np.ceil(swing_period / simulation_dt))
    time_points = np.linspace(0.0, swing_period, num_intervals + 1)
    references = np.array([
        trajectory_generator.compute_trajectory_references(
            swing_time=foot_swing_time, lift_off=lift_off, touch_down=touch_down
        )
        for foot_swing_time in time_points
    ])
    # Shape: (time samples, reference kind [position, velocity, acceleration], xyz).
    position_points = references[:, 0, :]

    # Keep the spatial curve and all time references in one window.
    fig = plt.figure(figsize=(13, 8), constrained_layout=True)
    grid = fig.add_gridspec(3, 2)
    ax_3d = fig.add_subplot(grid[:, 0], projection="3d")
    ax_3d.plot(*position_points.T, label="Swing trajectory")
    ax_3d.scatter(*lift_off, color="tab:green", label="Lift-off")
    ax_3d.scatter(*touch_down, color="tab:red", label="Touchdown")
    ax_3d.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title="Foot trajectory")
    ax_3d.legend()

    # Plot Cartesian components with consistent colours across the three panels.
    time_axes = []
    for reference_index, ylabel in enumerate(("Position [m]", "Velocity [m/s]", "Acceleration [m/s²]")):
        ax = fig.add_subplot(grid[reference_index, 1], sharex=time_axes[0] if time_axes else None)
        for component, label in enumerate(("x", "y", "z")):
            ax.plot(time_points, references[:, reference_index, component], label=label)
        ax.set_ylabel(ylabel)
        ax.set_xlim(0.0, swing_period)
        ax.grid(alpha=0.3)
        ax.legend(loc="best")
        time_axes.append(ax)
    time_axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"Bézier swing — duration {swing_period:g} s, step height {step_height:g} m")
    plt.show()
