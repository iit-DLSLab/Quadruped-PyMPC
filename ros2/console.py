import readline
import readchar
import time
import numpy as np

# Config imports
from quadruped_pympc import config as cfg

class Console():
    def __init__(self, controller_node):
        self.controller_node = controller_node

        # Walking and Stopping
        self.walking = False

        # Go Up and Go Down motion
        self.isDown = True
        self.height_delta = -cfg.simulation_params['ref_z']

        # Pitch Up and Pitch Down
        self.pitch_delta = 0

        # Init gain Aliengo
        self.controller_node.wb_interface.stc.position_gain_fb = 100
        self.controller_node.wb_interface.stc.velocity_gain_fb = 10
        self.controller_node.wb_interface.stc.use_feedback_linearization = False
        self.controller_node.wb_interface.stc.use_friction_compensation = False

        # Step Height holder to keep track of the step height
        self.step_height_holder = cfg.simulation_params['step_height']


        # Autocomplete setup
        self.commands = [
            "stw", "ooo", "narrowStance", "wideStance", "setGaitTimer", 
            "setup", "goUp", "goDown", "help", "ictp"
        ]
        readline.set_completer(self.complete)
        readline.parse_and_bind("tab: complete")


    def complete(self, text, state):
        options = [cmd for cmd in self.commands if cmd.startswith(text)]
        if state < len(options):
            print(options[state])
            return options[state]
        else:
            return None


    def interactive_command_line(self, ):
        self.print_all_commands()
        while True:
            input_string = input(">>> ")
            try:
                if(input_string == "stw"):
                    if(self.isDown == True):
                        print("The robot is down, please go up before starting to walk")
                    else:
                        if(self.walking):
                            print("The robot is already walking")
                        print("Starting Walking")
                        self.walking = True
                        self.controller_node.wb_interface.pgg.gait_type = self.controller_node.wb_interface.pgg.previous_gait_type
                        self.controller_node.wb_interface.pgg.reset()
                

                elif(input_string == "ooo"):
                    print("Stopping Walking")
                    self.walking = False
                    self.controller_node.wb_interface.pgg.gait_type = 7 # FULL_STANCE
                

                elif(input_string == "narrowStance"):
                    print("Narrow Stance")
                    self.controller_node.wb_interface.frg.hip_offset -= 0.03 
                

                elif(input_string == "wideStance"):
                    print("Wide Stance")
                    self.controller_node.wb_interface.frg.hip_offset += 0.03 


                elif(input_string == "setGaitTimer"):
                    print("Press one of the following numbers to set the gait type")
                    print("0: TROT")
                    print("1: PACE")
                    print("2: BOUNDING")
                    print("3: CIRCULARCRAWL")
                    print("4: BFDIAGONALCRAWL")
                    print("5: BACKDIAGONALCRAWL")
                    print("6: FRONTDIAGONALCRAWL")
                    print("7: FULL_STANCE")

                    if(self.walking):
                        print("Please stop the robot before changing the gait type")
                        continue
                    
                    gait_type = int(input("Gait Type: >>> "))
                    if(gait_type == 7):
                        gait_name = "full_stance"
                    elif(gait_type == 0):
                        gait_name = "trot"
                    elif(gait_type == 1):
                        gait_name = "pace"
                    elif(gait_type == 2):
                        gait_name = "bound"
                    elif(gait_type == 3):
                        gait_name = "crawl"
                    elif(gait_type == 4):
                        gait_name = "crawl"
                    elif(gait_type == 5):
                        gait_name = "crawl"
                    elif(gait_type == 6):
                        gait_name = "crawl"


                    if(gait_type >= 0 and gait_type <= 7):
                        gait_params = cfg.simulation_params['gait_params'][gait_name]
                        gait_type, duty_factor, step_frequency = gait_params['type'], gait_params['duty_factor'], gait_params['step_freq']
                        
                        
                        self.controller_node.wb_interface.pgg.step_freq = step_frequency
                        self.controller_node.wb_interface.pgg.duty_factor = duty_factor
                        self.controller_node.wb_interface.pgg.gait_type = gait_type
                        self.controller_node.wb_interface.pgg.previous_gait_type = gait_type
                        self.controller_node.wb_interface.pgg.reset()
                        
                        self.controller_node.wb_interface.frg.stance_time = (1 / self.controller_node.wb_interface.pgg.step_freq) * self.controller_node.wb_interface.pgg.duty_factor
                        swing_period = (1 - self.controller_node.wb_interface.pgg.duty_factor) * (1 / self.controller_node.wb_interface.pgg.step_freq)
                        self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(step_height=self.step_height_holder, swing_period=swing_period)
                        
                    else:
                        print("Invalid Gait Type")

                
                elif(input_string == "setupGaitTimer"):
                    
                    print("Current Step Frequency: ", self.controller_node.wb_interface.pgg.step_freq)
                    temp = input("Step Frequency: >>> ")
                    if(temp != ""):
                        temp = max(0.4, min(float(temp), 2.0))
                        self.controller_node.wb_interface.pgg.step_freq = temp
                        self.controller_node.wb_interface.frg.stance_time = (1 / self.controller_node.wb_interface.pgg.step_freq) * self.controller_node.wb_interface.pgg.duty_factor
                        swing_period = (1 - self.controller_node.wb_interface.pgg.duty_factor) * (1 / self.controller_node.wb_interface.pgg.step_freq)
                        self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(step_height=self.step_height_holder, swing_period=swing_period)

                    
                    print("Current Duty Factor: ", self.controller_node.wb_interface.pgg.duty_factor)
                    temp = input("Duty Factor: >>> ")
                    if(temp != ""):
                        temp = max(0.4, min(float(temp), 0.9))
                        self.controller_node.wb_interface.pgg.duty_factor = temp
                        self.controller_node.wb_interface.frg.stance_time = (1 / self.controller_node.wb_interface.pgg.step_freq) * self.controller_node.wb_interface.pgg.duty_factor
                        swing_period = (1 - self.controller_node.wb_interface.pgg.duty_factor) * (1 / self.controller_node.wb_interface.pgg.step_freq)
                        self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(step_height=self.step_height_holder, swing_period=swing_period)

                    print("Start and Stop Gait: ", self.controller_node.wb_interface.pgg.start_and_stop_activated)
                    temp = input("Start and Stop Gait: >>> ")
                    if(temp != ""):
                        if(temp == "True"):
                            self.controller_node.wb_interface.pgg.start_and_stop_activated = True
                        elif(temp == "False"):
                            self.controller_node.wb_interface.pgg.start_and_stop_activated = False

                elif(input_string == "setupLegsGains"):

                    print("Current Gains Swing Kp: ", self.controller_node.wb_interface.stc.position_gain_fb)
                    temp = input("Gains Swing Kp: >>> ")
                    if(temp != ""):
                        self.controller_node.wb_interface.stc.position_gain_fb = float(temp)
                    
                    
                    print("Current Gains Swing Kd: ", self.controller_node.wb_interface.stc.velocity_gain_fb)
                    temp = input("Gains Swing Kd: >>> ")
                    if(temp != ""):
                        self.controller_node.wb_interface.stc.velocity_gain_fb = float(temp)

                    print("Current Gain Stance Kp: ", self.controller_node.impedence_joint_position_gain)
                    temp = input("Gain Stance Kp: >>> ")
                    if(temp != ""):
                        self.controller_node.impedence_joint_position_gain = np.ones(12)*float(temp)
                    
                    print("Current Gain Stance Kd: ", self.controller_node.impedence_joint_velocity_gain)
                    temp = input("Gain Stance Kd: >>> ")
                    if(temp != ""):
                        self.controller_node.impedence_joint_velocity_gain = np.ones(12)*float(temp)

                elif(input_string == "setupGeneral"):
                    
                    print("Current Base Height: ", cfg.simulation_params['ref_z'] + self.height_delta)
                    height_temp = input("CoM Height: >>> ")
                    if(height_temp != ""):
                        height_delta_temp = float(height_temp) - cfg.simulation_params['ref_z']
                        min_value = -0.1
                        max_value = 0.1
                        self.height_delta = max(min_value, min(height_delta_temp, max_value))


                    print("Current Step Height: ", self.controller_node.wb_interface.stc.swing_generator.step_height)
                    step_height_temp = input("Step Height: >>> ")
                    if(step_height_temp != ""):
                        self.step_height_holder = max(0.05, min(float(step_height_temp), 0.25))
                        swing_period_temp =  self.controller_node.wb_interface.stc.swing_period
                        self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(self.step_height_holder, swing_period_temp)
                    
                    
                    print("Use FeedbackLin: ", self.controller_node.wb_interface.stc.use_feedback_linearization)
                    temp = input("Use FeedbackLin: >>> ")
                    if(temp != ""):
                        if(temp == "True"):
                            self.controller_node.wb_interface.stc.use_feedback_linearization = True
                        elif(temp == "False"):
                            self.controller_node.wb_interface.stc.use_feedback_linearization = False

                    
                    print("Use Friction Comp. only: ", self.controller_node.wb_interface.stc.use_friction_compensation)
                    temp = input("Use Friction Comp. only: >>> ")
                    if(temp != ""):
                        if(temp == "True"):
                            self.controller_node.wb_interface.stc.use_friction_compensation = True
                        elif(temp == "False"):
                            self.controller_node.wb_interface.stc.use_friction_compensation = False
                    
                    

                    print("Use Integrators in MPC: ", self.controller_node.srbd_controller_interface.controller.use_integrators)
                    temp = input("Use Integrators in MPC: >>> ")
                    if(temp != ""):
                        if(temp == "True"):
                            self.controller_node.srbd_controller_interface.controller.use_integrators = True
                        elif(temp == "False"):
                            self.controller_node.srbd_controller_interface.controller.use_integrators = False

                    
                    print("Com Offset: ", self.controller_node.wb_interface.frg.com_pos_offset_b)
                    temp = input("Set CoM offset x: >>> ")
                    if(temp != ""):
                        temp = max(-0.1, min(float(temp), 0.1))
                        self.controller_node.wb_interface.frg.com_pos_offset_b[0] = float(temp)
                    temp = input("Set CoM offset y: >>> ")
                    if(temp != ""):
                        temp = max(-0.1, min(float(temp), 0.1))
                        self.controller_node.wb_interface.frg.com_pos_offset_b[1] = float(temp)
                    temp = input("Set CoM offset z: >>> ")
                    if(temp != ""):
                        temp = max(-0.1, min(float(temp), 0.1))
                        self.controller_node.wb_interface.frg.com_pos_offset_b[2] = float(temp)

                    print("Use Reflexes: ", self.controller_node.wb_interface.esd.activated)
                    temp = input("Use Reflexes: >>> ")
                    if(temp != ""):
                        if(temp == "True"):
                            self.controller_node.wb_interface.esd.activated = True
                        elif(temp == "False"):
                            self.controller_node.wb_interface.esd.activated = False


                    
                
                elif(input_string == "goUp"):
                    print("Going Up")
                    if(self.walking):
                        print("Please stop the robot before going down")
                        continue
                    if(not self.isDown):
                        print("The robot is already up")
                        continue
                    
                    start_time = time.time()
                    time_motion = 5.
                    initial_height = -cfg.simulation_params['ref_z']
                    while(time.time() - start_time < time_motion):
                        time_diff = time.time() - start_time
                        self.height_delta = initial_height + (cfg.simulation_params['ref_z'] * time_diff / time_motion)
                        time.sleep(0.01)
                        
                    self.height_delta = 0
                    self.isDown = False


                elif(input_string == "goDown"):
                    print("Going Down")
                    if(self.walking):
                        print("Please stop the robot before going down")
                        continue
                    if(self.isDown):
                        print("The robot is already down")
                        continue

                    start_time = time.time()
                    time_motion = 5.
                    initial_height = 0
                    while(time.time() - start_time < time_motion):
                        time_diff = time.time() - start_time
                        self.height_delta = initial_height - cfg.simulation_params['ref_z']*time_diff/time_motion
                        time.sleep(0.01)

                    self.height_delta = -cfg.simulation_params['ref_z']
                    self.isDown = True
                
                
                elif(input_string == "help"):
                    self.print_all_commands()

                
                elif(input_string == "ictp"):
                    print("Interactive Keyboard Control")
                    print("w: Move Forward")
                    print("s: Move Backward")
                    print("a: Move Left")
                    print("d: Move Right")
                    print("q: Rotate Left")
                    print("e: Rotate Right")
                    print("0: Stop")
                    print("1: Pitch Up")
                    print("2: Reset Pitch")
                    print("3: Pitch Down")
                    print("Press any other key to exit")
                    while True:
                        command = readchar.readkey()
                        if(command == "w"):
                            self.controller_node.env._ref_base_lin_vel_H[0] += 0.1
                            print("w")
                        elif(command == "s"):
                            self.controller_node.env._ref_base_lin_vel_H[0] -= 0.1
                            print("s")
                        elif(command == "a"):
                            self.controller_node.env._ref_base_lin_vel_H[1] += 0.1
                            print("a")
                        elif(command == "d"):
                            self.controller_node.env._ref_base_lin_vel_H[1] -= 0.1
                            print("d")
                        elif(command == "q"):
                            self.controller_node.env._ref_base_ang_yaw_dot += 0.1
                            print("q")
                        elif(command == "e"):
                            self.controller_node.env._ref_base_ang_yaw_dot -= 0.1
                            print("e")
                        elif(command == "0"):
                            self.controller_node.env._ref_base_lin_vel_H[0] = 0
                            self.controller_node.env._ref_base_lin_vel_H[1] = 0
                            self.controller_node.env._ref_base_ang_yaw_dot = 0 
                            print("0")
                        elif(command == "1"):
                            self.pitch_delta -= 0.1
                            print("1")
                        elif(command == "2"):
                            self.pitch_delta = 0
                            print("2")
                        elif(command == "3"):
                            self.pitch_delta += 0.1
                            print("3")
                        else:
                            self.controller_node.env._ref_base_lin_vel_H[0] = 0
                            self.controller_node.env._ref_base_lin_vel_H[1] = 0
                            self.controller_node.env._ref_base_ang_yaw_dot = 0 
                            break
            except Exception as e:
                print("Error: ", e)
                print("Invalid Command")
                self.print_all_commands()


    def print_all_commands(self):
        print("\nAvailable Commands")
        print("help: Display all available messages")
        print("stw: Start Walking")
        print("ooo: Stop Walking")
        print("ictp: Interactive Keyboard Control")
        print("########################")
        print("narrowstance: Narrow Stance")
        print("widestance: Wide Stance")
        print("goUp: The robot goes up")
        print("goDown: The robot goes down")
        print("########################")
        print("setGaitTimer: Set the gait type")
        print("setupGaitTimer: Setup the gait timer")
        print("setupLegsGains: Setup the leg gains")
        print("setupGeneral: Setup general parameters\n")