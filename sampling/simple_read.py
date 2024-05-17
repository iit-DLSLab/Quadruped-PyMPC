import numpy as np

with open('./data_sb_controller.npy', 'rb') as f:
    state = np.load(f)
    reference = np.load(f)
    input = np.load(f)
    external_disturbance = np.load(f)



# HOW DID I SAVED THING??
# temp_state =  np.zeros((1, 49))
# temp_state[0, 0] = state_current["position"][0]   WORLD
# temp_state[0, 1] = state_current["position"][1]   WORLD
# temp_state[0, 2] = state_current["position"][2]   WORLD
# temp_state[0, 3:6] = state_current["linear_velocity"][0:3] WORLD
# temp_state[0, 6] = state_current["orientation"][0] WORLD
# temp_state[0, 7] = state_current["orientation"][1] WORLD
# temp_state[0, 8] = state_current["orientation"][2] WORLD
# temp_state[0, 9] = state_current["angular_velocity"][0] BASE
# temp_state[0, 10] = state_current["angular_velocity"][1] BASE
# temp_state[0, 11] = state_current["angular_velocity"][2] BASE
# temp_state[0, 12:15] = state_current["foot_FL"] WORLD
# temp_state[0, 15:18] = state_current["foot_FR"] WORLD
# temp_state[0, 18:21] = state_current["foot_RL"] WORLD
# temp_state[0, 21:24] = state_current["foot_RR"] WORLD
# temp_state[0, 24:24+12] = d.qpos[6:18]
# temp_state[0, 36:36+12] = d.qvel[6:18]
# temp_state[0, 48] = pgg.step_freq

# temp_state[0, 49] = optimize_swing #we optimize the step freq only at a certain point during the swing (this will be 1)
# temp_state[0, 50] = current_contact[0] #FL, 0 or 1 (in contact if 1)
# temp_state[0, 51] = current_contact[1] #FR, 0 or 1 (in contact if 1)
# temp_state[0, 52] = current_contact[2] #RL, 0 or 1 (in contact if 1)
# temp_state[0, 53] = current_contact[3] #RR, 0 or 1 (in contact if 1)
# temp_state[0, 54] = swing_time[0]/swing_period #swing phase FL
# temp_state[0, 55] = swing_time[1]/swing_period #swing phase FR
# temp_state[0, 56] = swing_time[2]/swing_period #swing phase RL
# temp_state[0, 57] = swing_time[3]/swing_period #swing phase RR
# data_state.append(copy.deepcopy(temp_state))


# temp_ref = np.zeros((1, 22))
# temp_ref[0, 0] = reference_state["ref_position"][2]
# temp_ref[0, 1:4] = reference_state["ref_linear_velocity"][0:3] WORLD - from a base one that is always costant here (0.4, 0, 0), but then is rotated in world
# temp_ref[0, 4] = reference_state["ref_orientation"][0]
# temp_ref[0, 5] = reference_state["ref_orientation"][1]
# temp_ref[0, 6] = reference_state["ref_angular_velocity"][0]
# temp_ref[0, 7] = reference_state["ref_angular_velocity"][1]
# temp_ref[0, 8] = reference_state["ref_angular_velocity"][2]
# temp_ref[0, 9:12] = reference_state["ref_FL_foot"]
# temp_ref[0, 12:15] = reference_state["ref_FR_foot"]
# temp_ref[0, 15:18] = reference_state["ref_RL_foot"]
# temp_ref[0, 18:21] = reference_state["ref_RR_foot"]
# temp_ref[0, 21] = 1.3 #nominal step frequency
# data_reference.append(copy.deepcopy(temp_ref))


# temp_input = np.zeros((1, 24))
# temp_input[0, 0:3] = nmpc_GRFs[0:3] WORLD
# temp_input[0, 3:6] = nmpc_GRFs[3:6] WORLD
# temp_input[0, 6:9] = nmpc_GRFs[6:9] WORLD
# temp_input[0, 9:12] = nmpc_GRFs[9:12] WORLD
# temp_input[0, 12:15] = tau_FL
# temp_input[0, 15:18] = tau_FR
# temp_input[0, 18:21] = tau_RL
# temp_input[0, 21:24] = tau_RR
# data_input.append(copy.deepcopy(temp_input))


# temp_disturbance = np.zeros((1, 7))
# temp_disturbance[0, 0:6] = disturbance_wrench WORLD (only x-y will be 0. Sample from -80N, 80N)
# temp_disturbance[0, 6] = start_disturbance_boolean WORLD
# data_external_disturbance.append(copy.deepcopy(temp_disturbance))