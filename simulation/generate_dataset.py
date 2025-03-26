# Authors:
# Giulio Turrisi, Daniel Ordonez
import itertools
import os
import pathlib
import sys
from pprint import pprint

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
        num_seconds_per_episode=1,
        num_episodes=5,
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

    # And since we use the same names as QuadrupedEnv, we can get the group representations for free
    from gym_quadruped.utils.quadruped_utils import configure_observation_space_representations

    obs_reps = configure_observation_space_representations(robot_name=dataset.env_hparams["robot"], obs_names=obs_names)

    for obs_name in obs_names:
        print(f"Obs: {obs_name} \n Representation: {obs_reps[obs_name]}")
