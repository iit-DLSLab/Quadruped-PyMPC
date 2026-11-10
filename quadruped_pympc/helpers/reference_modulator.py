import numpy as np

from quadruped_pympc import config as cfg


class ReferenceModulator:
    """Modify the references given to the MPC.

    - Velocity modulator: the reference velocities are set to zero if a foot is too far from its nominal stance.
    - Integrators: integral action on com height, linear velocities, roll, pitch and yaw rate, to compensate for
      model mismatch.
      The integral of the tracking error is subtracted from the reference, so the MPC is asked for more where it lags.
    """

    def __init__(self):

        self.use_velocity_modulator = cfg.locomotion_params['use_velocity_modulator']
        self.hip_offset_x = cfg.hip_offset_x
        self.hip_offset_y = cfg.hip_offset_y

        if cfg.robot == "aliengo":
            self.max_distance = 0.2
        elif cfg.robot == "go1" or cfg.robot == "go2":
            self.max_distance = 0.2
        else:
            self.max_distance = 0.2

        self.use_integrators = cfg.locomotion_params['use_integrators']
        self.integrator_gain = cfg.locomotion_params['integrator_gain']
        self.integrator_cap = np.array(cfg.locomotion_params['integrator_cap'], dtype=float)
        # z [m], x_vel, y_vel (heading frame), z_vel [m/s], roll, pitch [rad], yaw_rate [rad/s]
        self.integral_errors = np.zeros((7,))

        # Offsets on the height and pitch references (e.g. from the ROS 2 console). They are added before
        # the integrators, otherwise the integrators would compensate them
        self.ref_height_offset = 0.0
        self.ref_pitch_offset = 0.0

    def modulate_velocities(self, ref_base_lin_vel, ref_base_ang_vel, feet_pos, hip_pos, base_yaw):
        """Limit velocities using displacement from the nominal offset stance.

        Feet and hips are in world coordinates; offsets use the horizontal heading frame.
        """
        if not self.use_velocity_modulator:
            return ref_base_lin_vel, ref_base_ang_vel

        c, s = np.cos(base_yaw), np.sin(base_yaw)
        R_W2H = np.array([[c, s], [-s, c]])
        distances = []
        for leg, sign_x, sign_y in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)):
            foot_to_hip = R_W2H @ (feet_pos[leg][:2] - hip_pos[leg][:2])
            offset = np.array([sign_x * self.hip_offset_x, sign_y * self.hip_offset_y])
            distances.append(np.linalg.norm(foot_to_hip - offset))

        if(ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01):
            # If the robot is not moving, we don't need to modulate the velocities
            return ref_base_lin_vel, ref_base_ang_vel

        if any(distance > self.max_distance for distance in distances):
            ref_base_lin_vel = ref_base_lin_vel * 0.0
            ref_base_ang_vel = ref_base_ang_vel * 0.0

        return ref_base_lin_vel, ref_base_ang_vel

    def integrate_references(
        self, state, ref_position, ref_linear_velocity, ref_orientation, ref_angular_velocity, dt
    ):
        """Apply the offsets and the integral action on height, linear velocities, roll, pitch and yaw rate.

        The error is the one between the state and the reference before the integral offset, so each
        integral grows until the tracking error is zero or its cap is reached (anti-windup).

        Args:
            state (dict): state of the robot used in the mpc (position, linear_velocity, orientation in world frame,
                angular_velocity in base frame)
            ref_position (np.ndarray): reference com position in world frame
            ref_linear_velocity (np.ndarray): reference linear velocity in world frame
            ref_orientation (np.ndarray): reference euler angles (roll, pitch, yaw)
            ref_angular_velocity (np.ndarray): reference angular velocity, compared with the one of the state
            dt (float): time since the last call

        Returns:
            ref_position, ref_linear_velocity, ref_orientation, ref_angular_velocity (np.ndarray): modified copies
            of the references
        """
        ref_position = np.array(ref_position, dtype=float)
        ref_linear_velocity = np.array(ref_linear_velocity, dtype=float)
        ref_orientation = np.array(ref_orientation, dtype=float)
        ref_angular_velocity = np.array(ref_angular_velocity, dtype=float)

        ref_position[2] += self.ref_height_offset
        ref_orientation[1] += self.ref_pitch_offset

        if not self.use_integrators:
            # Restart from zero when they are enabled again (e.g. from the console)
            self.integral_errors[:] = 0.0
            return ref_position, ref_linear_velocity, ref_orientation, ref_angular_velocity

        # The horizontal velocities are integrated in the heading frame, so the integral follows the robot yaw
        yaw = state["orientation"][2]
        c, s = np.cos(yaw), np.sin(yaw)
        R_W2H = np.array([[c, s], [-s, c]])

        error = np.zeros((7,))
        error[0] = state["position"][2] - ref_position[2]
        error[1:3] = R_W2H @ (state["linear_velocity"][0:2] - ref_linear_velocity[0:2])
        error[3] = state["linear_velocity"][2] - ref_linear_velocity[2]
        error[4:6] = state["orientation"][0:2] - ref_orientation[0:2]
        error[6] = state["angular_velocity"][2] - ref_angular_velocity[2]

        self.integral_errors = np.clip(
            self.integral_errors + self.integrator_gain * error * dt, -self.integrator_cap, self.integrator_cap
        )

        ref_position[2] -= self.integral_errors[0]
        ref_linear_velocity[0:2] -= R_W2H.T @ self.integral_errors[1:3]
        ref_linear_velocity[2] -= self.integral_errors[3]
        ref_orientation[0:2] -= self.integral_errors[4:6]
        ref_angular_velocity[2] -= self.integral_errors[6]

        return ref_position, ref_linear_velocity, ref_orientation, ref_angular_velocity

    def reset(self):
        self.integral_errors = np.zeros((7,))
