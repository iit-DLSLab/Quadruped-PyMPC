# Description: This script is used to simulate multiple MPC runs in parallel,
# mainly to test the performance of the MPC controller over time.

# Authors:
# Giulio Turrisi

import multiprocessing
import time
from multiprocessing import Process

import numpy as np

# Import the simulation module that will run mujoco
import simulation

import os
import pathlib
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Number of processes to run
NUM_PROCESSES = 4
NUM_EPISODES = 20
NUM_SECONDS_EPISODE = 2
REF_BASE_LIN_VEL=(-1.0, 1.0)
REF_BASE_ANG_VEL=(-0.2, 0.2)
FRICTION_COEFF=(0.5, 1.0)
BASE_VEL_COMMAND_TYPE="forward+rotate"


if __name__ == "__main__":
    from quadruped_pympc import config as cfg
    qpympc_cfg = cfg
    render = False
    render_only_first = False
    output_simulations_tracking = None
    output_simulations_success_rate = None

    # Run the simulation in parallel
    processes = []
    manager = multiprocessing.Manager()
    return_dict = manager.dict()
    for i in range(NUM_PROCESSES):
        if render_only_first and i == 0:
            render = True
        
        storage_path = pathlib.Path(__file__).parent.parent / ("datasets_" + "proc_" + str(i))
        
        p = Process(
            target=simulation.run_simulation,
            args=(qpympc_cfg, i, NUM_EPISODES, NUM_SECONDS_EPISODE, REF_BASE_LIN_VEL, REF_BASE_ANG_VEL, FRICTION_COEFF, BASE_VEL_COMMAND_TYPE, i * NUM_PROCESSES, render, storage_path),
        )
        p.start()
        time.sleep(5)
        processes.append(p)

    for p in processes:
        p.join()

    """# Concatenate the data from the processes
    for proc_j in range(NUM_PROCESSES):
        for ep_i in range(NUM_EPISODES):
            if output_simulations_tracking is None:
                output_simulations_tracking = return_dict[
                    "process" + str(proc_j) + "_ctrl_state_history_ep" + str(ep_i)
                ]
            else:
                output_simulations_tracking = np.concatenate(
                    (
                        output_simulations_tracking,
                        return_dict["process" + str(proc_j) + "_ctrl_state_history_ep" + str(ep_i)],
                    )
                )

            if output_simulations_success_rate is None:
                output_simulations_success_rate = np.array(
                    [return_dict["process" + str(proc_j) + "_success_rate_ep" + str(ep_i)]]
                )
            else:
                output_simulations_success_rate = np.concatenate(
                    (
                        output_simulations_success_rate,
                        np.array([return_dict["process" + str(proc_j) + "_success_rate_ep" + str(ep_i)]]),
                    )
                )

    print("Tracking data mean: ", np.mean(output_simulations_tracking, axis=0))
    print("Tracking data std: ", np.std(output_simulations_tracking, axis=0))
    print("Success rate: ", np.sum(output_simulations_success_rate) / len(output_simulations_success_rate), "%")"""
