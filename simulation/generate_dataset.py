# Authors:
# Giulio Turrisi, Daniel Ordonez
import itertools
import os
import pathlib
import sys
from pprint import pprint
import numpy as np

from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.data.h5py import H5Reader

# TODO: Remove horrible hack
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from simulation import run_simulation

if __name__ == "__main__":
    from quadruped_pympc import config as cfg

    qpympc_cfg = cfg
    # Custom changes to the config here:

    storage_path = pathlib.Path(__file__).parent.parent / "datasets"
    data_path = run_simulation(
        qpympc_cfg=qpympc_cfg,
        num_seconds_per_episode=50,
        num_episodes=1,
        ref_base_lin_vel=(-0.5, 0.5),
        ref_base_ang_vel=(-0.3, 0.3),
        base_vel_command_type="human",
        render=True,
        recording_path=storage_path,
    )

    dataset = H5Reader(file_path=data_path)

    print(f"Number of trajectories: {dataset.len()}")
    print("Parameters used to generate the data")
    pprint(dataset.env_hparams)
    # We can reconstruct the environment used to reproduce the dataset using
    reproduced_env = QuadrupedEnv(**dataset.env_hparams)

    # We can also list the observations in the dataset
    obs_names = dataset.env_hparams["state_obs_names"]

    print(f"\n\n{str(list(itertools.chain(['-'] * 100)))}\n\n")
    print(
        f"Dataset stored at: {data_path} \n"
        f"Number of trajectories: {dataset.len()} \n"
        f"Dataset observations names: \n{obs_names}"
    )


    # We can also convert the dataset to a npy file
    n_trajs = dataset.n_trajectories
    desired_fps = 50.0
    actual_fps = 1.0 / reproduced_env.mjModel.opt.timestep
    skipping_factor = int(actual_fps / desired_fps)
    for traj_id in range(n_trajs):
        
        obs_t = {obs_name: dataset.recordings[obs_name][traj_id][::skipping_factor] for obs_name in reproduced_env.state_obs_names}
        
        base_pos = obs_t['base_pos']
        base_ori_quat_xyzw = np.roll(obs_t['base_ori_quat_wxyz'],-1)
        qpos_js = obs_t['qpos_js']
        
        data = {
            "joints_list": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"],
            "joint_positions": [row for row in qpos_js],
            'root_position': [row for row in base_pos],
            'root_quaternion': [row for row in base_ori_quat_xyzw],
            "fps": desired_fps,
        }
        # Save the data to an .npy file
        output_file = f"datasets/traj_{traj_id}.npy"
        np.save(output_file, data)


    # And since we use the same names as QuadrupedEnv, we can get the group representations for free
    from gym_quadruped.utils.quadruped_utils import configure_observation_space_representations

    obs_reps = configure_observation_space_representations(robot_name=dataset.env_hparams["robot"], obs_names=obs_names)

    for obs_name in obs_names:
        print(f"Obs: {obs_name} \n Representation: {obs_reps[obs_name]}")
