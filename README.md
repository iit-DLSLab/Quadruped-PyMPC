<div style="display: flex; justify-content: space-around;">
  <img src="./gifs/aliengo_trot.gif" alt="Trot" width="30%">
  <img src="./gifs/hyqreal_pace.gif" alt="Pace" width="30%">
  <img src="./gifs/go2_bound.gif" alt="Bound" width="30%">
</div>


## Overview
This repo contains a model predictive controller based on the **single rigid body model** and written in **Python**. It comes in two flavours: gradient-based via [acados](https://github.com/acados/acados) or sampling-based via [jax](https://github.com/google/jax). The controller is tested on real robots and is compatible with [Mujoco](https://mujoco.org/). 


Features gradient-based mpc:
- less than 5ms computation on an intel i7-13700H cpu 
- optional integrators for model mismatch
- optional smoothing for the ground reaction forces 
- optional foothold optimization
- optional [real-time iteration](http://cse.lab.imtlucca.it/~bemporad/publications/papers/ijc_rtiltv.pdf) or [advanced-step real-time iteration](https://arxiv.org/pdf/2403.07101.pdf)
- optional zero moment point/center of mass constraints
- optional lyapunov-based criteria


Features sampling-based mpc:
- 10000 parallel rollouts in less than 2ms on an nvidia 4050 mobile gpu!
- optional step frequency adaptation for enhancing robustness
- implements different strategies: [random sampling](https://arxiv.org/pdf/2212.00541.pdf), [mppi](https://sites.gatech.edu/acds/mppi/), or [cemppi](https://arxiv.org/pdf/2203.16633.pdf) 
- different control parametrizations: zero-order, linear splines or cubic splines (see [mujoco-mpc](https://arxiv.org/pdf/2212.00541.pdf))

## Installation

See [here](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README_install.md)

## Citing this work

If you find the work useful, please consider citing one of our works: 

#### [On the Benefits of GPU Sample-Based Stochastic Predictive Controllers for Legged Locomotion (IROS-2024)](https://arxiv.org/abs/2403.11383):
```
@INPROCEEDINGS{turrisi2024sampling,
  author={Turrisi, Giulio and Modugno, Valerio and Amatucci, Lorenzo and Kanoulas, Dimitrios and Semini, Claudio},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={On the Benefits of GPU Sample-Based Stochastic Predictive Controllers for Legged Locomotion}, 
  year={2024},
  pages={13757-13764},
  doi={10.1109/IROS58592.2024.10801698}}
```
#### [Adaptive Non-Linear Centroidal MPC With Stability Guarantees for Robust Locomotion of Legged Robots (RAL-2025)](https://arxiv.org/abs/2409.01144):
```
@ARTICLE{elobaid2025adaptivestablempc,
  author={Elobaid, Mohamed and Turrisi, Giulio and Rapetti, Lorenzo and Romualdi, Giulio and Dafarra, Stefano and Kawakami, Tomohiro and Chaki, Tomohiro and Yoshiike, Takahide and Semini, Claudio and Pucci, Daniele},
  journal={IEEE Robotics and Automation Letters}, 
  title={Adaptive Non-Linear Centroidal MPC With Stability Guarantees for Robust Locomotion of Legged Robots}, 
  year={2025},
  volume={10},
  number={3},
  pages={2806-2813},
  doi={10.1109/LRA.2025.3536296}}
```

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi) and [Daniel Ordonez](https://github.com/Danfoa).
