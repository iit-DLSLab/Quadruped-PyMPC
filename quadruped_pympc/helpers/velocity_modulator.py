import numpy as np

from quadruped_pympc import config as cfg


class VelocityModulator:
    def __init__(self):
        
        self.activated = cfg.simulation_params['velocity_modulator']
        self.hip_offset_x = cfg.hip_offset_x
        self.hip_offset_y = cfg.hip_offset_y

        if cfg.robot == "aliengo":
            self.max_distance = 0.2
        elif cfg.robot == "go1" or cfg.robot == "go2":
            self.max_distance = 0.2
        else:
            self.max_distance = 0.2

    def modulate_velocities(self, ref_base_lin_vel, ref_base_ang_vel, feet_pos, hip_pos, base_yaw):
        """Limit velocities using displacement from the nominal offset stance.

        Feet and hips are in world coordinates; offsets use the horizontal heading frame.
        """
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
