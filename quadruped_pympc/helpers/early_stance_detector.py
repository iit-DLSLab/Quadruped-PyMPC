import numpy as np
from quadruped_pympc import config as cfg
from gym_quadruped.utils.quadruped_utils import LegsAttr




class EarlyStanceDetector:
    def __init__(self, legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR')):
        self.legs_order = legs_order
        self.early_stance = LegsAttr(FL=False, FR=False, RR=False, RL=False )
        self.hitmoments = LegsAttr(FL=-1.0, FR=-1.0, RR=-1.0, RL=-1.0) # swing time of the last contact moment
        self.hitpoints = LegsAttr(FL=None, FR=None, RR=None, RL=None)

        self.activated = True

        if(cfg.mpc_params['type'] == 'sampling'):
            self.activated = False # TO FIX

        self.early_stance_time_threshold = 0.07
        self.relative_tracking_error_threshold = 0.5
        self.absolute_min_distance_error_threshold = 0.1

        self.gait_cycles_after_step_height_enanchement = -1
        self.use_height_enhancement = False
        self.max_gait_cycles_height_enhancement = 6


    def update_detection(self, feet_pos: LegsAttr, des_feet_pos: LegsAttr, 
                         lift_off: LegsAttr, touch_down: LegsAttr, 
                         swing_time: list, swing_period: float, 
                         current_contact, previous_contact,
                         stc):
        """ 
        Update the early stance detector.
        
        Parameters:
            feet_pos : Current feet positions.
            des_feet_pos : Desired feet positions.
            lift_off_positions : Lift off positions of all legs.
            touch_down_positions : Touch down positions of all legs.
            swing_time : Time of the current swing phase.
            swing_period : Duration of the swing phase.
            current_contact : Current contact state of the legs.
        """
        if not self.activated:
            for leg_id,leg_name in enumerate(self.legs_order):
                self.early_stance[leg_name] = False
                self.hitmoments[leg_name] = -1.0
                self.hitpoints[leg_name] = None
        else: 
            for leg_id,leg_name in enumerate(self.legs_order):
                disp = touch_down[leg_name] - lift_off[leg_name]
                #if swing_time[leg_id] < EARLY_STANCE_TIME_THRESHOLD or swing_time[leg_id] > swing_period - EARLY_STANCE_TIME_THRESHOLD:
                if current_contact[leg_id] == 1:
                    self.early_stance[leg_name] = False  # reset early stance in "planned" stance
                    continue
                elif self.early_stance[leg_name] == False and swing_time[leg_id] > swing_period - self.early_stance_time_threshold:
                    self.early_stance[leg_name] = False # avoid early stance in the last 10% of the swing phase
                    continue
                else:
                    local_disp = (des_feet_pos[leg_name] - feet_pos[leg_name]).squeeze()
                    #print(f"Leg {leg_name} local_disp: {np.linalg.norm(local_disp)} disp: {np.linalg.norm(disp)}")
                    #print("norm(local_disp)/norm(disp): ", np.linalg.norm(local_disp)/np.linalg.norm(disp))
                    if self.early_stance[leg_name] == False:
                        #if np.arccos(np.dot(disp, local_disp) / (np.linalg.norm(disp) * np.linalg.norm(local_disp))) < np.pi/3: 
                        if (np.linalg.norm(local_disp)/np.linalg.norm(disp)) > self.relative_tracking_error_threshold and np.linalg.norm(local_disp) > self.absolute_min_distance_error_threshold:
                            self.hitpoints[leg_name] = feet_pos[leg_name].copy()
                            self.hitmoments[leg_name] = swing_time[leg_id]
                            self.early_stance[leg_name] = True  # acos( disp dot local_disp / |disp| |local_disp|) < 60°
                            self.gait_cycles_after_step_height_enanchement = 0
                            break
                        else:
                            self.early_stance[leg_name] = False
                            self.hitmoments[leg_name] = -1.0
                            self.hitpoints[leg_name] = None
                
                if self.early_stance[leg_name] == False:
                    self.hitmoments[leg_name] = -1.0
                    self.hitpoints[leg_name] = None

        if(self.use_height_enhancement):
            for leg_id,leg_name in enumerate(self.legs_order):
                if current_contact[leg_id] == 1 and previous_contact[leg_id] == 0 and self.gait_cycles_after_step_height_enanchement >= 0:
                    self.gait_cycles_after_step_height_enanchement += 1

                    break
            
            if(self.gait_cycles_after_step_height_enanchement >= 0 and self.gait_cycles_after_step_height_enanchement < self.max_gait_cycles_height_enhancement):
                stc.swing_generator.step_height_enhancement = True 
            else:
                stc.swing_generator.step_height_enhancement = False
                self.gait_cycles_after_step_height_enanchement = -1
        

