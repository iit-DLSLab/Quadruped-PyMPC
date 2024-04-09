## Overview
This repo contains a model predictive controller based on the **single rigid body model** and written entirely in **Python**. It cames in two flavours: gradient-based via [acados](https://docs.acados.org/about_acados/index.html#:~:text=acados%20is%20a%20software%20package,Moritz%20Diehl) or sampling-based via [jax](https://github.com/google/jax). The controller is tested on real robots and is compatible with [Mujoco](https://mujoco.org/).  


Features gradient-based mpc:
- less than 5ms computation on an intel i7-13700H cpu 
- optional integrators for model mismatch
- optional smoothing for the ground reaction forces 
- optional foothold optimization
- optional [real-time iteration](http://cse.lab.imtlucca.it/~bemporad/publications/papers/ijc_rtiltv.pdf) or [advanced-step real-time iteration](https://arxiv.org/pdf/2403.07101.pdf)
- optional zero moment point/center of mass constraints


Features sampling-based mpc:
- 10000 parallel rollouts in less than 2ms on an nvidia 4050 mobile gpu!
- optional step frequency adaptation for enhancing robustness
- implements different strategies: [random sampling](https://arxiv.org/pdf/2212.00541.pdf), [mppi](https://sites.gatech.edu/acds/mppi/), or [cemppi](https://arxiv.org/pdf/2203.16633.pdf) 
- different control parametrizations: zero-order, linear splines or cubic splines (see [mujoco-mpc](https://arxiv.org/pdf/2212.00541.pdf))


<table >
    <tr>
        <th colspan="3" align="center">Experiments with Aliengo</th>
    </tr>
    <tr>
        <td align="left"><img src="./gifs/trot.gif"/></td>
        <td align="center"><img src="./gifs/pace.gif"/></td>
        <td align="right"><img src="./gifs/crawl.gif"/></td>
    </tr>
    <tr>
        <th colspan="3" align="center">Simulations with Go2</th>
    </tr>
    <tr>
        <td align="left"><img src="./gifs/trot_mujoco.gif"/></td>
        <td align="center"><img src="./gifs/pace_mujoco.gif"/></td>
        <td align="right"><img src="./gifs/crawl_mujoco.gif"/></td>
    </tr>
</table>


## Dependencies
Gradient-based MPC: It uses [CasADI](https://web.casadi.org/) to define the model and [acados](https://docs.acados.org/about_acados/index.html#:~:text=acados%20is%20a%20software%20package,Moritz%20Diehl) to solve the optimal control problem. Sampling-based MPC: [jax](https://github.com/google/jax) for both. The simulation environment is based on [Mujoco](https://mujoco.org/). Pinocchio support is still experimental and for now not really used.

## Installation

1. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64)

2. create an environment using the file in the folder [installation/mamba](https://github.com/iit-DLSLab/Quadruped-PyMPC/tree/main/installation/mamba) choosing the file depending on your gpu (nvidia or integrated):

    `conda env create -f mamba_environment.yml`


3. clone the other submodules:

    `git submodule update --init --recursive`
    
4. activate the conda environment

    `conda activate quadruped_pympc_env`

5. go inside the folder acados and compile it pressing:
    
    ```
    mkdir build
    cd build
    cmake -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=ON ..
    make install -j4
    pip install -e ./../interfaces/acados_template
    ```

6. inside the file .bashrc, given your **path_to_acados**, put:
    
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path_to_acados/lib"
    export ACADOS_SOURCE_DIR="/path_to_acados"
    ```

The first time you run the simulation with acados, in the terminal you will be asked to install tera_render. You should accept to proceed.

## How to run

1. activate the conda environment
   
   ```
   conda env create -f mamba_environment.yml
   ```

2. go in the folder [simulation](https://github.com/iit-DLSLab/Quadruped-PyMPC/tree/main/simulation) and press
   
   ```
   python3 simulation_mujoco.py
   ```

In the file [config.py](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/config.py), you can set up the robot, the mpc type (gradient, sampling..), its proprierties (real time iteration, sampling type, foothold optimization..), and other simulation params (reference, gait type..). 



## Citing this work

If you find the work useful, please consider citing:

```
@article{turrisi2024sampling,
  title={On the Benefits of GPU Sample-Based Stochastic Predictive Controllers for Legged Locomotion},
  author={Giulio Turrisi, Valerio Modugno, Lorenzo Amatucci, Dimitrios Kanoulas, Claudio Semini},
  booktitle={arXiv preprint arXiv:2403.11383},
  year={2024}
}
```

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi).