import numpy as np
import matplotlib.pyplot as plt

import ndcurves
# importing tools to plot bezier curves
#from ndcurves.plot import plotBezier
import copy


class SwingTrajectoryGenerator:
    def __init__(self,
                 step_height: float, 
                 swing_period: float) -> None:
        self.step_height = step_height
        self.swing_period = swing_period
        self.half_swing_period = swing_period / 2
        self.bezier_time_factor = 1 / (swing_period / 2)


        # Stored swing-trajectory properties
        dt = 0.002
        self._dt = copy.deepcopy(dt)
        self._stepHeight = step_height
        self._N = int(self.swing_period / self._dt)

        # Define the initial and terminal curve constraints
        self._constraints = ndcurves.curve_constraints(3)
        self._constraints.init_vel = np.array([[0., 0., 0.]]).T
        self._constraints.end_vel = np.array([[0., 0., 0.]]).T
        self._constraints.init_acc = np.array([[0., 0., 0.]]).T
        self._constraints.end_acc = np.array([[0., 0., 0.]]).T


    def createBezierCurve(self, x0, xf):

        scaling_factor = 0.7105
        p1 = x0 + np.array([0., 0., self._stepHeight / scaling_factor])
        p2 = 0.5 * (x0 + xf) + np.array([0., 0., self._stepHeight / scaling_factor])
        p3 = xf + np.array([0., 0., self._stepHeight / scaling_factor])
        self._curve = ndcurves.bezier(
            np.array([x0, p1, p2, p3, xf]).T, self._constraints, 0., (self._N - 1) * self._dt)
        
    def compute_trajectory_references(self, k, lift_off, touch_down):
        
        self.createBezierCurve(lift_off, touch_down)
        position = self._curve(k * self._dt)
        velocity = self._curve.derivate(k * self._dt, 1)

        return position, velocity, 0


    def plot_trajectory_3d(self,
                           curve_points: np.array) -> None:
        curve_points = np.array(curve_points)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2])
        ax.legend()

        plt.title('3D Bézier Curve')
        plt.show()

    def plot_trajectory_references(self, tp, fp, vp):
        # Convert lists to NumPy arrays for easier plotting
        time_points = np.array(tp)
        footPosDes_points = np.array(fp)
        footVelDes_points = np.array(vp)
        
        # Create subplots for position, velocity, and acceleration
        fig, axs = plt.subplots(2, 1, figsize=(8, 12))

        # Plot position
        for i in range(3):
            axs[0].plot(time_points, footPosDes_points[:, i], label=f"Position {i+1}")
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('Position')
        axs[0].legend()

        # Plot velocity
        for i in range(3):
            axs[1].plot(time_points, footVelDes_points[:, i], label=f"Velocity {i+1}")
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Velocity')
        axs[1].legend()



        plt.tight_layout()
        plt.show()







# Example:
if __name__ == "__main__":
    step_height = 0.08
    swing_period = 0.9
    trajectory_generator = SwingTrajectoryGenerator(step_height=step_height ,swing_period=swing_period, position_gain_fb = 0, velocity_gain_fb = 0)

    #trajectory_generator.createBezierCurve(np.array([0,0,0]), np.array([0.3,0,0]))
    
    # Generate trajectory points
    time_points = []
    position_points = []
    velocity_points = []
    #acceleration_points = []
    i = 0
    for foot_swing_time in np.arange(0.000001, swing_period, 0.002):
        desired_foot_position, desired_foot_velocity, _ = trajectory_generator.compute_trajectory_references(i, np.array([0,0,0]), np.array([0.3,0,0]))
        i += 1
                                                                                                                                     
        time_points.append(i*0.002)
        position_points.append(desired_foot_position.squeeze())
        velocity_points.append(desired_foot_velocity.squeeze())
        
    # Plot the generated trajectory
    trajectory_generator.plot_trajectory_3d(np.array(position_points))
    trajectory_generator.plot_trajectory_references(time_points, position_points, velocity_points)
    