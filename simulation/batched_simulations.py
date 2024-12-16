# Description: This script is used to simulate multiple MPC runs in parallel, 
# mainly to test the performance of the MPC controller over time.

# Authors:
# Giulio Turrisi

import multiprocessing
from multiprocessing import Process
import time
import numpy as np
import matplotlib.pyplot as plt
# Import the simulation module that will run mujoco
import simulation

# Number of processes to run
NUM_PROCESSES = 5
NUM_EPISODES = 20


if __name__ == '__main__':
    render = False
    render_only_first = True
    output_simulations_tracking = None
    output_simulations_success_rate = None
    base_vel_command_type = "forward+rotate"
    
        
    # Run the simulation in parallel
    processes = []
    manager = multiprocessing.Manager()
    return_dict = manager.dict()
    for i in range(NUM_PROCESSES):
        if(render_only_first and i == 0):
            render = True
        else:
            render = False
        p = Process(target=simulation.run_simulation, args=(i, NUM_EPISODES, return_dict, i*NUM_PROCESSES, base_vel_command_type, render))
        p.start()
        time.sleep(2)
        processes.append(p)

    for p in processes:
        p.join()


    # Concatenate the data from the processes
    for proc_j in range(NUM_PROCESSES):
        for ep_i in range(NUM_EPISODES):
            if(output_simulations_tracking is None):
                output_simulations_tracking = return_dict['process'+str(proc_j)+'_ctrl_state_history_ep'+str(ep_i)]
            else:
                output_simulations_tracking = np.concatenate((output_simulations_tracking, return_dict['process'+str(proc_j)+'_ctrl_state_history_ep'+str(ep_i)]))

            if(output_simulations_success_rate is None):
                output_simulations_success_rate = np.array([return_dict['process'+str(proc_j)+'_success_rate_ep'+str(ep_i)]])
            else:
                output_simulations_success_rate = np.concatenate((output_simulations_success_rate, np.array([return_dict['process'+str(proc_j)+'_success_rate_ep'+str(ep_i)]])))

    
    print("Tracking data mean: ", np.mean(output_simulations_tracking, axis=0))
    print("Tracking data std: ", np.std(output_simulations_tracking, axis=0))
    print("Success rate: ", np.sum(output_simulations_success_rate)/len(output_simulations_success_rate), "%")
    

    