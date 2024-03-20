import copy
import numpy as np

class PeriodicGaitGenerator:
    def __init__(self, duty_factor, step_freq, p_gait, horizon):
        self.duty_factor = duty_factor
        self.step_freq = step_freq
        self.horizon = horizon
        
        # Choose of the gait, this represent the delay of each leg
        if p_gait == Gait.TROT:
            #self.delta = [0.0, 0.5, 0.5, 0.0]
            self.delta = [0.5, 1.0, 1.0, 0.5]
        elif p_gait == Gait.PACE:
            self.delta = [0.8, 0.3, 0.8, 0.3]
        elif p_gait == Gait.BOUNDING:
            self.delta = [0.5, 0.5, 0.0, 0.0]
        elif p_gait == Gait.CIRCULARCRAWL:
            self.delta = [0.0, 0.25, 0.75, 0.5]
        elif p_gait == Gait.BFDIAGONALCRAWL:
            self.delta = [0.0, 0.25, 0.5, 0.75]
        elif p_gait == Gait.BACKDIAGONALCRAWL:
            self.delta = [0.0, 0.5, 0.75, 0.25]
        elif p_gait == Gait.FRONTDIAGONALCRAWL:
            self.delta = [0.5, 1.0, 0.75, 1.25]
        else:
            self.delta = [0.0, 0.5, 0.5, 0.0]

        # Initialize variables
        #self.t = np.zeros(len(self.delta))
        #self.init = [True]*len(self.delta)
        
        self.t = self.delta
        self.init = [False]*len(self.delta)
        self.n_contact = len(self.delta)

        self.new_step_freq = step_freq
        self.time_before_switch_freq = 0
        

    def run(self, dt, new_step_freq):
        contact = np.zeros(self.n_contact)
        for leg in range(self.n_contact):

            # Restart condition
            if self.t[leg] >= 1.0:
                self.t[leg] = 0 

            # Increase time by dt
            #self.t[leg] += dt*self.step_freq
            self.t[leg] += dt*new_step_freq


            # If we are still in init, we check if the delay of the leg
            # is not surpassed. If not, the contact needs to be still 1
            # otherwise we lift off
            if self.init[leg]:
                if self.t[leg] < self.delta[leg]:
                    contact[leg] = 1
                else:
                    self.init[leg] = False
                    contact[leg] = 1
                    self.t[leg] = 0
            else:
                # During the gait, we check if the time is below the duty factor
                # if so, the contact is 1, otherwise it is 0
                if self.t[leg] < self.duty_factor:
                    contact[leg] = 1
                else:
                    contact[leg] = 0



        return contact
    
    def set_new_step_freq(self, new_step_freq):
        self.new_step_freq = new_step_freq
        

    def set(self, t, init):
        self.t = t
        self.init = init


    def get_t(self):
        return self.t

    
    def compute_contact_sequence(self, mpc_dt, simulation_dt):
        t_init = copy.deepcopy(self.t)
        init_init = copy.deepcopy(self.init)

            
        
        contact_sequence = np.zeros((self.n_contact, self.horizon))
        for i in range(self.horizon):
            #if(i > 2):
            #    new_step_freq = self.new_step_freq
            #else:
            #    new_step_freq = self.step_freq
            #contact_sequence[:, i] = self.run(mpc_dt, new_step_freq)
            contact_sequence[:, i] = self.run(mpc_dt, self.step_freq)
        
        self.set(t_init, init_init)
        #self.run(simulation_dt)
        self.time_before_switch_freq += simulation_dt
        
        #if(self.time_before_switch_freq >= 0.08):
        #    self.step_freq = self.new_step_freq
        #    self.time_before_switch_freq = 0
        return contact_sequence
    

class Gait:
    TROT = 0
    PACE = 1
    BOUNDING = 2
    CIRCULARCRAWL = 3
    BFDIAGONALCRAWL = 4
    BACKDIAGONALCRAWL = 5
    FRONTDIAGONALCRAWL = 6
    FULL_STANCE = 7