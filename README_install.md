## Dependencies
Gradient-based MPC: It uses [CasADI](https://web.casadi.org/) to define the model and [acados](https://github.com/acados/acados) to solve the optimal control problem. Sampling-based MPC: [jax](https://github.com/google/jax) for both. The simulation environment is based on [Mujoco](https://mujoco.org/).

## Installation

1. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64 or arm64 depending on your platform)

2. create an environment using the file in the folder [installation](https://github.com/iit-DLSLab/Quadruped-PyMPC/tree/main/installation) choosing between **nvidia and integrated gpu**, either **with or without ros** (to run the simulation, you don't need ros!):

    `conda env create -f mamba_environment.yml`


3. clone the other submodules:

    `git submodule update --init --recursive`
    
4. activate the conda environment

    `conda activate quadruped_pympc_env`

5. go inside the folder acados and compile it pressing:
    
    ```
    mkdir build
    cd build
    cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
    make install -j4
    pip install -e ./../interfaces/acados_template
    ```

6. inside the file .bashrc, given your **path_to_acados**, put:
    
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path_to_acados/lib"
    export ACADOS_SOURCE_DIR="/path_to_acados"
    ```

    Notice that if you are using Mac, you should modify the file .zshrc adding
    
    ```
    export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path_to_acados/lib"
    export ACADOS_SOURCE_DIR="/path_to_acados"
    ```

The first time you run the simulation with acados, in the terminal you will be asked to install tera_render. You should accept to proceed.


7. go to Quadruped-PyMPC initial folder and install it:

    ```
    pip install -e .
    ```

## How to run - Simulation

1. activate the conda environment
   
   ```
   conda activate quadruped_pympc_env
   ```

2. go in the main Quadruped-PyMPC [folder](https://github.com/iit-DLSLab/Quadruped-PyMPC) and press
   
   ```
   python3 simulation/simulation.py
   ```

In the file [config.py](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/quadruped_pympc/config.py), you can set up the robot, the mpc type (gradient, sampling..), its proprierties (real time iteration, sampling type, foothold optimization..), and other simulation params (reference, gait type..). 

3. you can interact with the simulation with your mouse to add disturbances, or with the keyboard by pressing
```
arrow up, arrow down -> add positive or negative forward velocity
arrow left, arrow right -> add positive or negative yaw velocity
ctrl -> set zero all velocities
```

## How to run - ROS2
During the installation procedure, use the file **mamba_environment_ros2.yml**. Then:

1. activate the conda environment
   
   ```
   conda activate quadruped_pympc_ros2_env
   ```

2. go in the folder [ros2/msgs_ws](https://github.com/iit-DLSLab/Quadruped-PyMPC/tree/main/ros2/msgs_ws) and compile the messages
  
  ```
  colcon build
  source install/setup.bash
  ```

3. you can run now the script

  ```
  python3 ros2/run_controller.py
  ```

4. if you want to test the above node with a simulator, for example to test ros2 delay, you can run 

  ```
  python3 ros2/run_simulator.py
  ```

5. joystick

  ```
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
  ```

6. general commands from terminal available [here](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/335779f1eecc315511b8036d84d986a66f2450c5/ros2/console.py#L358)

7. for a real-robot deployment, use a nice [state estimator](https://github.com/iit-DLSLab/muse) 
