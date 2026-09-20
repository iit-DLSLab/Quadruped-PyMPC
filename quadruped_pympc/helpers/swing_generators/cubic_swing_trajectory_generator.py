import matplotlib.pyplot as plt
import numpy as np


class SwingTrajectoryGenerator:
    """Generate a Cartesian swing using two cubic Hermite polynomials.

    Each segment occupies half of the swing: lift-off to apex, then apex to
    touchdown. The segments share position and velocity at the apex (C1
    continuity). All positions must use the same coordinate frame, with z
    pointing upward; distances are in metres and times are in seconds.
    """

    def __init__(self, step_height: float, swing_period: float) -> None:
        """Set the absolute apex z coordinate and the total swing duration.

        Despite its name, step_height is not an offset from lift-off. Choose
        it above both endpoint heights to obtain an upward swing with a true
        maximum at the apex.
        """
        # A positive duration is required to normalize time and compute derivatives.
        if not np.isfinite(swing_period) or swing_period <= 0:
            raise ValueError("swing_period must be finite and positive")
        self.step_height = step_height
        self.swing_period = swing_period
        self.half_swing_period = swing_period / 2

    def plot_trajectory_3d(self, curve_points: np.ndarray) -> None:
        """Display sampled Cartesian positions supplied as an (N, 3) array."""
        curve_points = np.array(curve_points)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2])
        ax.legend()

        plt.title("3D cubic swing")
        plt.show()

    def plot_trajectory_references(self, tp, fp, vp, ap):
        """Plot N timestamps and their (N, 3) position/velocity/acceleration arrays."""
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
    ) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return the Hermite boundary data for the active half of the swing.

        swing_time is elapsed time since lift-off. Endpoint inputs may be flat,
        row or column vectors containing three Cartesian coordinates.
        Returns (local_time, start_position, end_position, start_velocity,
        end_velocity), with velocities expressed in metres per second.
        These are Hermite boundary conditions, not Bezier control points.
        """
        # Normalize input shapes so every vector operation produces a (3,) array.
        lift_off = np.asarray(lift_off, dtype=float).reshape(3)
        touch_down = np.asarray(touch_down, dtype=float).reshape(3)

        # EXERCISE 1.1-1.4---------------------------


        #--------------------------------------------


        # Both segments use exactly the same apex position and velocity.
        # At the knot itself, select the first segment (left-hand acceleration).
        if swing_time <= self.half_swing_period:
            return swing_time, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
        # Reset the local clock for the descending segment, without wrapping the touchdown time.
        return swing_time - self.half_swing_period, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)

    def compute_trajectory_references(
        self, swing_time: float, lift_off: np.ndarray, touch_down: np.ndarray,
        early_stance_hitmoment=-1, early_stance_hitpoint=None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate two C1 cubic Hermite segments and their time derivatives.

        Velocity is zero at lift-off and touchdown, and its z component is
        zero at the apex. Vertical acceleration may jump at the apex when
        endpoint heights differ. Early-contact arguments are accepted for
        controller compatibility; this generator does not replan on contact.
        Times outside the swing interval hold the corresponding endpoint.

        Returns three (3,) arrays: position [m], velocity [m/s], and
        acceleration [m/s^2], all expressed in the input coordinate frame.
        """
        # Shift time to the beginning of the active segment: tau = t - t_s.
        # First half: t_s = 0; second half: t_s = swing_period/2.
        # T = t_f - t_s is the SEGMENT duration, not the total swing period.
        tau, q_s, q_f, q_dot_s, q_dot_f = self.compute_control_points(
            swing_time, lift_off, touch_down
        )
        # EXERCISE 1.1-1.4 --------------
        position = np.zeros(3)
        velocity = np.zeros(3)
        acceleration = np.zeros(3)



        # --------------------------------

        # C1 continuity does not require matching vertical acceleration at the
        # apex: with equal half durations, it matches only for equal endpoint z.
        # Outside the swing, the clamped position is held stationary. At exactly
        # lift-off or touchdown, retain the cubic's one-sided acceleration.
        if swing_time < 0.0 or swing_time > self.swing_period:
            velocity = np.zeros(3)
            acceleration = np.zeros(3)
        return position, velocity, acceleration


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
    fig.suptitle(f"Cubic Hermite swing — duration {swing_period:g} s, step height {step_height:g} m")
    plt.show()
