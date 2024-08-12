import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

try:
    from virall.vfa.vfa import VFA
except ImportError:
    print("VFA not installed, not open source yet")

class VisualFootholdAdaptation:

    def __init__(self, legs_order, adaptation_strategy='height'):
        self.footholds_adaptation = LegsAttr(FL=np.array([0, 0, 0]), 
                                             FR=np.array([0, 0, 0]), 
                                             RL=np.array([0, 0, 0]), 
                                             RR=np.array([0, 0, 0]))
        self.initialized = False

        self.adaptation_strategy = adaptation_strategy

        if(self.adaptation_strategy == 'vfa'):
            self.vfa_evaluators = LegsAttr(FL=None, FR=None, RL=None, RR=None)
            for leg_id, leg_name in enumerate(legs_order):
                self.vfa_evaluators[leg_name] = VFA(leg=leg_name)


    def update_footholds_adaptation(self, update_footholds_adaptation):
        self.footholds_adaptation = update_footholds_adaptation
        self.initialized = True

    
    def reset(self):
        self.initialized = False


    def get_footholds_adapted(self, reference_footholds):
        # If the adaptation is not initialized, return the reference footholds
        if(self.initialized == False):
            self.footholds_adaptation = reference_footholds
            return reference_footholds
        else:
            return self.footholds_adaptation
        

    def compute_adaptation(self, legs_order, reference_footholds, hip_positions, heightmaps,
                    forward_vel, base_orientation, base_orientation_rate):
        
        for leg_id, leg_name in enumerate(legs_order):
            if(heightmaps[leg_name].data is None):
                return False
        
        
        if(self.adaptation_strategy == 'height'):
            for leg_id, leg_name in enumerate(legs_order):
                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if(height_adjustment is not None):
                    reference_footholds[leg_name][2] = height_adjustment
        

        elif(self.adaptation_strategy == 'vfa'):
            gait_phases = 0.0 #for now
            for leg_id, leg_name in enumerate(legs_order):
                # Transform the heightmap in hip frame
                
                heightmap = heightmaps[leg_name].data
                hip_position = hip_positions[leg_name]
                
                heightmap_hip_frame = heightmap[:,:,0,2] - hip_position[2]
                

                convex_data, \
                _, \
                safe_map, \
                info = self.vfa_evaluators[leg_name](heightmap_data=heightmap_hip_frame,
                                                base_linear_velocity=forward_vel,
                                                base_orientation=base_orientation,
                                                base_orientation_rate=base_orientation_rate,
                                                gait_phase=gait_phases,
                                                return_info=True
                                                )
                
                best_foothold_id = convex_data[0][0]
                best_convex_area_vertices_id = [convex_data[1][0].x1, convex_data[1][0].y2]

                r = round(best_foothold_id.item()/heightmaps[leg_name].n)
                c = round(best_foothold_id.item()%heightmaps[leg_name].n)

                if(r >= heightmaps[leg_name].n):
                    r = heightmaps[leg_name].n -1
                if(c >= heightmaps[leg_name].n):
                    c = heightmaps[leg_name].n -1
                
                reference_footholds[leg_name][0:2] = heightmap[r,c,0,:][0:2]

                
                height_adjustment = heightmaps[leg_name].get_height(reference_footholds[leg_name])
                if(height_adjustment is not None):
                    reference_footholds[leg_name][2] = height_adjustment

                #print("Safe map: ", safe_map)
            
        
        self.update_footholds_adaptation(reference_footholds)
        
        return True