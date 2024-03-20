## Overview
This repo contains a model predictive controller based on the **single rigid body model** and written entirely in **Python**. It cames in two flavours: gradient-based via [acados](https://docs.acados.org/about_acados/index.html#:~:text=acados%20is%20a%20software%20package,Moritz%20Diehl) or sampling-based via [jax](https://github.com/google/jax). The controller is tested on real robots and is compatible with [Mujoco](https://mujoco.org/).  


Features gradient-based mpc:
- less than 10ms computation on an intel i7-13700H cpu 
- optional integrators for model mismatch
- optional smoothing for the ground reaction forces 
- optional foothold optimization
- optional real time iteration
- optional zero moment point/center of mass constraints


Features sampling-based mpc:
- less than 10ms computation on an nvidia 4050 mobile gpu 
- 10000 parallel rollouts in less than 2ms!
- optional step frequency adapation for enhancing robustness
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

2. create an environment using the file in the folder installation/:

    `conda env create -f mamba_environment.yml`


3. clone the other submodules:

    `git submodule update --init --recursive`
    
4. activate the conda environment

    `conda activate quadruped_pympc_env`

5. go inside acados and create a build folder (mkdir build). Then press:
    
    ```
    cd build
    cmake -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=ON ..
    make install -j4
    pip install -e ./../interfaces/acados_template
    ```

6. inside the file .bashrc, put:
    
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path_to_acados/lib"
    export ACADOS_SOURCE_DIR="/path_to_acados"
    ```

    
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