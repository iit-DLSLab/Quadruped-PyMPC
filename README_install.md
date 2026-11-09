## Dependencies
Gradient-based MPC: It uses [CasADI](https://web.casadi.org/) to define the model and [acados](https://github.com/acados/acados) to solve the optimal control problem. Sampling-based MPC: [jax](https://github.com/google/jax) for both. The simulation environment is based on [Mujoco](https://mujoco.org/).

You can install this repo with Pixi (preferred) or Docker (that in any case will use pixi). 

---

### Pixi installation

1. install [pixi](https://pixi.sh/latest/)

2. create the environment, choosing the ROS 2 distro (**lyrical** or **humble**) and whether you want the **cuda** build:

```bash
pixi shell                       # lyrical, no cuda (or pixi shell -e lyrical-no-cuda)
pixi shell -e lyrical-cuda       # lyrical, cuda
pixi shell -e humble-no-cuda     # humble, no cuda
pixi shell -e humble-cuda        # humble, cuda
```


### Docker installation

1. install [Docker](https://docs.docker.com/engine/install/) (and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) if you want the **nvidia** image)

2. from the Quadruped-PyMPC folder, build the image choosing between **integrated gpu or nvidia** and the ROS 2 distro (**lyrical** or **humble**):

```bash
docker build -f docker/integrated_gpu/dockerfile --build-arg ROS_DISTRO=humble -t quadruped_pympc:humble .         # integrated gpu

docker build -f docker/nvidia/dockerfile --build-arg ROS_DISTRO=humble -t quadruped_pympc:humble-cuda .           # nvidia
```

3. run the container from the Quadruped-PyMPC folder, which is mounted in `/workspace`:

```bash
# integrated gpu
docker run -it --rm --network host --ipc host -e DISPLAY -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro \
  -v "$(pwd)":/workspace quadruped_pympc:humble

# nvidia
docker run -it --rm --gpus all --network host --ipc host -e DISPLAY \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro \
  -v "$(pwd)":/workspace quadruped_pympc:humble-cuda
```

Quadruped-PyMPC is already built inside the image, and the pixi environment is activated in every new terminal.



### How to run - Simulation

1. activate your pixi, conda or docker environment
   

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

### How to run - ROS2

1. activate your pixi, conda or docker environment
   

2. you can run now the script

    ```
    python3 ros2/run_controller.py
    ```

3. if you want to test the above node with a simulator, for example to test ros2 delay, you can run 

    ```
    python3 ros2/run_simulator.py
    ```

4. joystick

    ```
    ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
    ```

6. general commands from terminal available [here](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/335779f1eecc315511b8036d84d986a66f2450c5/ros2/console.py#L358)

7. for a real-robot deployment, use a nice [state estimator](https://github.com/iit-DLSLab/muse)
