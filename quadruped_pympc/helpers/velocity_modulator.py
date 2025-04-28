import numpy as np

from quadruped_pympc import config as cfg


class VelocityModulator:
    def __init__(self):
        
        self.activated = cfg.simulation_params['velocity_modulator']

        if cfg.robot == "aliengo":
            self.max_distance = 0.2
        elif cfg.robot == "go1" or cfg.robot == "go2":
            self.max_distance = 0.2
        else:
            self.max_distance = 0.2

    def modulate_velocities(self, ref_base_lin_vel, ref_base_ang_vel, feet_pos, hip_pos):
        distance_FL_to_hip_xy = np.sqrt(
            np.square(feet_pos.FL[0] - hip_pos.FL[0]) + np.square(feet_pos.FL[1] - hip_pos.FL[1])
        )
        distance_FR_to_hip_xy = np.sqrt(
            np.square(feet_pos.FR[0] - hip_pos.FR[0]) + np.square(feet_pos.FR[1] - hip_pos.FR[1])
        )
        distance_RL_to_hip_xy = np.sqrt(
            np.square(feet_pos.RL[0] - hip_pos.RL[0]) + np.square(feet_pos.RL[1] - hip_pos.RL[1])
        )
        distance_RR_to_hip_xy = np.sqrt(
            np.square(feet_pos.RR[0] - hip_pos.RR[0]) + np.square(feet_pos.RR[1] - hip_pos.RR[1])
        )

        if (
            distance_FL_to_hip_xy > self.max_distance
            or distance_FR_to_hip_xy > self.max_distance
            or distance_RL_to_hip_xy > self.max_distance
            or distance_RR_to_hip_xy > self.max_distance
        ):
            ref_base_lin_vel = ref_base_lin_vel * 0.0
            ref_base_ang_vel = ref_base_ang_vel * 0.0

        return ref_base_lin_vel, ref_base_ang_vel
