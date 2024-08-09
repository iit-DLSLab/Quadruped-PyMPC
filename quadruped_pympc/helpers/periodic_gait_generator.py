import numpy as np

from quadruped_pympc.helpers.quadruped_utils import GaitType


class PeriodicGaitGenerator:

    def __init__(self, duty_factor, step_freq, gait_type: GaitType, horizon):
        self.duty_factor = duty_factor
        self.step_freq = step_freq
        self.horizon = horizon
        self.gait_type = gait_type

        # Private variables
        self._phase_signal, self._init = None, None
        self.reset()

    def reset(self):
        # Choose of the gait, this represent the delay of each leg
        if self.gait_type == GaitType.TROT.value:
            self.phase_offset = [0.5, 1.0, 1.0, 0.5]
        elif self.gait_type == GaitType.PACE.value:
            self.phase_offset = [0.8, 0.3, 0.8, 0.3]
        elif self.gait_type == GaitType.BOUNDING.value:
            self.phase_offset = [0.5, 0.5, 0.0, 0.0]
        elif self.gait_type == GaitType.CIRCULARCRAWL.value:
            self.phase_offset = [0.0, 0.25, 0.75, 0.5]
        elif self.gait_type == GaitType.BFDIAGONALCRAWL.value:
            self.phase_offset = [0.0, 0.25, 0.5, 0.75]
        elif self.gait_type == GaitType.BACKDIAGONALCRAWL.value:
            self.phase_offset = [0.0, 0.5, 0.75, 0.25]
        elif self.gait_type == GaitType.FRONTDIAGONALCRAWL.value:
            self.phase_offset = [0.5, 1.0, 0.75, 1.25]
        else:
            self.phase_offset = [0.0, 0.5, 0.5, 0.0]

        # Set random gait_signal respecting the gait phase offset between legs
        self._phase_signal = (np.asarray(self.phase_offset) + np.random.rand()) % 1.0
        self._init = [False] * len(self.phase_offset)
        self.n_contact = len(self.phase_offset)
        self.time_before_switch_freq = 0

    def run(self, dt, new_step_freq):
        contact = np.zeros(self.n_contact)
        for leg in range(self.n_contact):

            # Increase time by dt
            # self.t[leg] += dt*self.step_freq
            self._phase_signal[leg] += dt * new_step_freq

            # Phase signal is between 0 and 1
            self._phase_signal[leg] = self._phase_signal[leg] % 1.0

            # If we are still in init, we check if the delay of the leg
            # is not surpassed. If not, the contact needs to be still 1
            # otherwise we lift off
            if self._init[leg]:
                if self._phase_signal[leg] < self.phase_offset[leg]:
                    contact[leg] = 1
                else:
                    self._init[leg] = False
                    contact[leg] = 1
                    self._phase_signal[leg] = 0
            else:
                # During the gait, we check if the time is below the duty factor
                # if so, the contact is 1, otherwise it is 0
                if self._phase_signal[leg] < self.duty_factor:
                    contact[leg] = 1
                else:
                    contact[leg] = 0

        return contact

    def set_phase_signal(self, phase_signal: np.ndarray, init: np.ndarray | None = None):
        assert len(phase_signal) == len(self._phase_signal)

        self._phase_signal = phase_signal

        if init is not None:
            assert len(init) == len(self._init)
            self._init = init
        else:
            self._init = [False for _ in range(len(self._phase_signal))]

    @property
    def phase_signal(self):
        return np.array(self._phase_signal)

    def compute_contact_sequence(self, contact_sequence_dts, contact_sequence_lenghts):
        # TODO: This function can be vectorized and computed with numpy vectorized operations
        if (self.gait_type == GaitType.FULL_STANCE.value):
            contact_sequence = np.ones((4, self.horizon * 2))
            return contact_sequence

        else:
            t_init = np.array(self._phase_signal)
            init_init = np.array(self._init)

            contact_sequence = np.zeros((self.n_contact, self.horizon))

            # the first value is simply the current predicted contact by the timer
            contact_sequence[:, 0] = self.run(0.0, self.step_freq)
            
            # contact_sequence_dts contains a list of dt (usefull for nonuniform sampling)
            # contact_sequence_lenghts contains the number of steps for each dt
            j = 0
            for i in range(1, self.horizon):
                if(i >= contact_sequence_lenghts[j]):
                    j += 1
                dt = contact_sequence_dts[j]
                contact_sequence[:, i] = self.run(dt, self.step_freq)
            self.set_phase_signal(t_init, init_init)
            return contact_sequence

