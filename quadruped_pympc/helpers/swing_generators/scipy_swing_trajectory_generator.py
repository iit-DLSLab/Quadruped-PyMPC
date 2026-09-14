import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

from quadruped_pympc import config as cfg

class SwingTrajectoryGenerator:
    def __init__(self, step_height: float, swing_period: float) -> None:
        self.step_height = step_height
        self.swing_period = swing_period
        self.half_swing_period = swing_period / 2
        self.bezier_time_factor = 1 / (swing_period / 2)

        # Stored swing-trajectory properties
        self.stepHeight = step_height
        self.reflex_next_steps_height_enhancement = False

        if(cfg.simulation_params['visual_foothold_adaptation'] == 'blind'):
            self.blind_locomotion = True
        else:
            self.blind_locomotion = False

        self.reflex_max_step_height = cfg.simulation_params['reflex_max_step_height']

    def createCurve(self, x0, xf, early_stance_hitmoment = -1):

        scaling_factor = 1.5
        
        if early_stance_hitmoment != -1:# and early_stance_hitmoment < self.swing_period*0.9:
            reflex_maximum_height = self.reflex_max_step_height
            
            p1 = x0.copy()
            p1[:2] = x0[:2] - 0.01 * (xf[:2]-x0[:2])
            p1 += np.array([0., 0., self.stepHeight / scaling_factor])
            p2 = 0.5 * (x0 + xf) + np.array([0., 0., reflex_maximum_height])
            p3 = 0.2*x0 + 0.8*xf + np.array([0.0, 0.0, reflex_maximum_height / scaling_factor])

            x = np.array([x0[0], p1[0], p2[0], p3[0], p3[0]])
            y = np.array([x0[1], p1[1], p2[1], p3[1], p3[1]])
            if(self.blind_locomotion):
                z = np.array([x0[2], p1[2], p2[2], p3[2], xf[2] + reflex_maximum_height / (scaling_factor+0.5)])
            else:
                z = np.array([x0[2], p1[2], p2[2], p3[2], xf[2]])

            updated_swing_period = self.swing_period - early_stance_hitmoment
            t = np.array([early_stance_hitmoment, early_stance_hitmoment+updated_swing_period/4, early_stance_hitmoment+updated_swing_period/2, early_stance_hitmoment+updated_swing_period*3/4, self.swing_period])

        else:
            if(self.reflex_next_steps_height_enhancement):
                temp_step_height = self.reflex_max_step_height
            else:
                temp_step_height = self.stepHeight
            p1 = x0 + np.array([0., 0., temp_step_height / scaling_factor])
            p2 = 0.5 * (x0 + xf) + np.array([0., 0., temp_step_height])
            p3 = xf + np.array([0.0, 0.0, temp_step_height / scaling_factor])

            x = np.array([x0[0], p1[0], p2[0], p3[0], xf[0]])
            y = np.array([x0[1], p1[1], p2[1], p3[1], xf[1]])
            z = np.array([x0[2], p1[2], p2[2], p3[2], xf[2]])

            t = np.array([0, self.half_swing_period/2, self.half_swing_period, self.half_swing_period*3/2, self.half_swing_period*2])
        


        self._curve_x = CubicSpline(t, x, bc_type=["clamped", "clamped"])
        self._curve_y = CubicSpline(t, y, bc_type=["clamped", "clamped"])
        self._curve_z = CubicSpline(t, z, bc_type=["clamped", "clamped"])

        # self._curve_x = Akima1DInterpolator(t, x)
        # self._curve_y = Akima1DInterpolator(t, y)
        # self._curve_z = Akima1DInterpolator(t, z)

        """dxdt = np.array([0, 0, 0])
        dydt = np.array([0, 0, 0])
        dzdt = np.array([0, 0, 0])
        scaling_factor = 1
        
        p2 = 0.5 * (x0 + xf) + np.array([0., 0., self.stepHeight / scaling_factor])
        
        x = np.array([x0[0], p2[0], xf[0]])
        y = np.array([x0[1], p2[1], xf[1]])
        z = np.array([x0[2], p2[2], xf[2]])
        t = np.array([0, self.half_swing_period, self.half_swing_period*2])
        self._curve_x = CubicHermiteSpline(t, x, dxdt)
        self._curve_y = CubicHermiteSpline(t, y, dydt)
        self._curve_z = CubicHermiteSpline(t, z, dzdt)"""

        self._curve_x_vel = self._curve_x.derivative()
        self._curve_y_vel = self._curve_y.derivative()
        self._curve_z_vel = self._curve_z.derivative()

        self._curve_x_acc = self._curve_x_vel.derivative()
        self._curve_y_acc = self._curve_y_vel.derivative()
        self._curve_z_acc = self._curve_z_vel.derivative()

    def compute_trajectory_references(
        self, swing_time: float, lift_off: np.array, touch_down: np.array, early_stance_hitmoment = -1, early_stance_hitpoint = None) -> (np.array, np.array, np.array):
        if early_stance_hitpoint is not None:
            self.createCurve(early_stance_hitpoint, touch_down, early_stance_hitmoment)
            # self.plot_current_curve(hitmoment)
        else:
            self.createCurve(lift_off, touch_down)

        position_x = self._curve_x(swing_time)
        position_y = self._curve_y(swing_time)
        position_z = self._curve_z(swing_time)

        position = np.array([position_x, position_y, position_z])

        velocity_x = self._curve_x_vel(swing_time)
        velocity_y = self._curve_y_vel(swing_time)
        velocity_z = self._curve_z_vel(swing_time)

        velocity = np.array([velocity_x, velocity_y, velocity_z])

        acceleration_x = self._curve_x_acc(swing_time)
        acceleration_y = self._curve_y_acc(swing_time)
        acceleration_z = self._curve_z_acc(swing_time)

        acceleration = np.array([acceleration_x, acceleration_y, acceleration_z])

        return position, velocity, acceleration

    def plot_trajectory_3d(self, curve_points: np.array) -> None:
        curve_points = np.array(curve_points)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2])
        ax.legend()

        plt.title("3D Curve")
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
    fig.suptitle(f"SciPy swing — duration {swing_period:g} s, step height {step_height:g} m")
    plt.show()
