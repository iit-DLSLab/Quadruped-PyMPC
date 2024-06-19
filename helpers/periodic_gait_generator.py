import copy

import numpy as np

from utils.quadruped_utils import Gait


class PeriodicGaitGenerator:

    def __init__(self, duty_factor, step_freq, gait_type: Gait, horizon):
        self.duty_factor = duty_factor
        self.step_freq = step_freq
        self.horizon = horizon
        self.gait_type = gait_type

        self.reset()

    def reset(self):
        # Choose of the gait, this represent the delay of each leg
        if self.gait_type == Gait.TROT:
            # self.delta = [0.0, 0.5, 0.5, 0.0]
            self.phase_offset = [0.5, 1.0, 1.0, 0.5]
        elif self.gait_type == Gait.PACE:
            self.phase_offset = [0.8, 0.3, 0.8, 0.3]
        elif self.gait_type == Gait.BOUNDING:
            self.phase_offset = [0.5, 0.5, 0.0, 0.0]
        elif self.gait_type == Gait.CIRCULARCRAWL:
            self.phase_offset = [0.0, 0.25, 0.75, 0.5]
        elif self.gait_type == Gait.BFDIAGONALCRAWL:
            self.phase_offset = [0.0, 0.25, 0.5, 0.75]
        elif self.gait_type == Gait.BACKDIAGONALCRAWL:
            self.phase_offset = [0.0, 0.5, 0.75, 0.25]
        elif self.gait_type == Gait.FRONTDIAGONALCRAWL:
            self.phase_offset = [0.5, 1.0, 0.75, 1.25]
        else:
            self.phase_offset = [0.0, 0.5, 0.5, 0.0]

        self.phase_signal = self.phase_offset
        self.init = [False] * len(self.phase_offset)
        self.n_contact = len(self.phase_offset)
        self.time_before_switch_freq = 0

    def run(self, dt, new_step_freq):
        contact = np.zeros(self.n_contact)
        for leg in range(self.n_contact):

            # Phase signal is between 0 and 1
            if self.phase_signal[leg] >= 1.0:
                self.phase_signal[leg] = 0

            # Increase time by dt
            # self.t[leg] += dt*self.step_freq
            self.phase_signal[leg] += dt * new_step_freq

            # If we are still in init, we check if the delay of the leg
            # is not surpassed. If not, the contact needs to be still 1
            # otherwise we lift off
            if self.init[leg]:
                if self.phase_signal[leg] < self.phase_offset[leg]:
                    contact[leg] = 1
                else:
                    self.init[leg] = False
                    contact[leg] = 1
                    self.phase_signal[leg] = 0
            else:
                # During the gait, we check if the time is below the duty factor
                # if so, the contact is 1, otherwise it is 0
                if self.phase_signal[leg] < self.duty_factor:
                    contact[leg] = 1
                else:
                    contact[leg] = 0

        return contact

    def set(self, t, init):
        self.phase_signal = t
        self.init = init

    def get_t(self):
        return self.phase_signal

    def compute_contact_sequence(self, mpc_dt, simulation_dt):
        t_init = copy.deepcopy(self.phase_signal)
        init_init = copy.deepcopy(self.init)

        contact_sequence = np.zeros((self.n_contact, self.horizon))
        for i in range(self.horizon):
            contact_sequence[:, i] = self.run(mpc_dt, self.step_freq)
        self.set(t_init, init_init)
        self.time_before_switch_freq += simulation_dt
        return contact_sequence
