"""This file includes all of the configuration parameters for the MPC controllers
and of the internal simulations that can be launch from the folder /simulation.
"""
import numpy as np
from quadruped_pympc.helpers.quadruped_utils import GaitType

# These are used both for a real experiment and a simulation -----------
# These are the only attributes needed per quadruped, the rest can be computed automatically ----------------------
robot = 'aliengo'  # 'aliengo', 'go1', 'go2', 'b2', 'hyqreal1', 'hyqreal2', 'mini_cheetah', 'spot'  # TODO: Load from robot_descriptions.py

from gym_quadruped.robot_cfgs import RobotConfig, get_robot_config
robot_cfg: RobotConfig = get_robot_config(robot_name=robot)
robot_leg_joints = robot_cfg.leg_joints
robot_feet_geom_names = robot_cfg.feet_geom_names
qpos0_js = robot_cfg.qpos0_js
hip_height = robot_cfg.hip_height

# ----------------------------------------------------------------------------------------------------------------
if (robot == 'go1'):
    mass = 12.019
    inertia = np.array([[1.58460467e-01, 1.21660000e-04, -1.55444692e-02],
                        [1.21660000e-04, 4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05, 5.24474661e-01]])

elif (robot == 'go2'):
    mass = 15.019
    inertia = np.array([[1.58460467e-01, 1.21660000e-04, -1.55444692e-02],
                        [1.21660000e-04, 4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05, 5.24474661e-01]])

elif (robot == 'aliengo'):
    mass = 24.637
    inertia = np.array([[0.2310941359705289, -0.0014987128245817424, -0.021400468992761768],
                        [-0.0014987128245817424, 1.4485084687476608, 0.0004641447134275615],
                        [-0.021400468992761768, 0.0004641447134275615, 1.503217877350808]])

elif (robot == 'b2'):
    mass = 83.49
    inertia = np.array([[0.2310941359705289, -0.0014987128245817424, -0.021400468992761768],
                        [-0.0014987128245817424, 1.4485084687476608, 0.0004641447134275615],
                        [-0.021400468992761768, 0.0004641447134275615, 1.503217877350808]])


elif (robot == 'hyqreal1'):
    mass = 108.40
    inertia = np.array([[4.55031444e+00, 2.75249434e-03, -5.11957307e-01],
                        [2.75249434e-03, 2.02411774e+01, -7.38560592e-04],
                        [-5.11957307e-01, -7.38560592e-04, 2.14269772e+01]])
    
elif (robot == 'hyqreal2'):
    mass = 126.69
    inertia = np.array([[4.55031444e+00, 2.75249434e-03, -5.11957307e-01],
                        [2.75249434e-03, 2.02411774e+01, -7.38560592e-04],
                        [-5.11957307e-01, -7.38560592e-04, 2.14269772e+01]])
    
elif (robot == 'mini_cheetah'):
    mass = 12.5
    inertia = np.array([[1.58460467e-01, 1.21660000e-04, -1.55444692e-02],
                        [1.21660000e-04, 4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05, 5.24474661e-01]])

elif (robot == 'spot'):
    mass = 50.34
    inertia = np.array([[0.2310941359705289, -0.0014987128245817424, -0.021400468992761768],
                        [-0.0014987128245817424, 1.4485084687476608, 0.0004641447134275615],
                        [-0.021400468992761768, 0.0004641447134275615, 1.503217877350808]])


gravity_constant = 9.81 # Exposed in case of different gravity conditions
# ----------------------------------------------------------------------------------------------------------------

mpc_params = {
    # 'nominal' optimized directly the GRF
    # 'input_rates' optimizes the delta GRF
    # 'sampling' is a gpu-based mpc that samples the GRF
    # 'collaborative' optimized directly the GRF and has a passive arm model inside
    # 'lyapunov' optimized directly the GRF and has a Lyapunov-based stability constraint
    # 'kinodynamic' sbrd with joints - experimental
    'type':                                    'nominal',

    # print the mpc info
    'verbose':                                 False,

    # horizon is the number of timesteps in the future that the mpc will optimize
    # dt is the discretization time used in the mpc
    'horizon':                                 12,
    'dt':                                      0.02,

    # GRF limits for each single leg
    "grf_max":                                 mass * gravity_constant,
    "grf_min":                                 0,
    'mu':                                      0.5,

    # this is used to have a smaller dt near the start of the horizon
    'use_nonuniform_discretization':           False,
    'horizon_fine_grained':                    2,
    'dt_fine_grained':                         0.01,

    # if this is true, we optimize the step frequency as well
    # for the sampling controller, this is done in the rollout
    # for the gradient-based controller, this is done with a batched version of the ocp
    'optimize_step_freq':                      False,
    'step_freq_available':                     [1.4, 2.0, 2.4],

    # ----- START properties only for the gradient-based mpc -----

    # this is used if you want to manually warm start the mpc
    'use_warm_start':                          False,

    # this enables integrators for height, linear velocities, roll and pitch
    'use_integrators':                         False,
    'alpha_integrator':                        0.1,
    'integrator_cap':                          [0.5, 0.2, 0.2, 0.0, 0.0, 1.0],

    # if this is off, the mpc will not optimize the footholds and will
    # use only the ones provided in the reference
    'use_foothold_optimization':               True,

    # this is set to false automatically is use_foothold_optimization is false
    # because in that case we cannot chose the footholds and foothold
    # constraints do not any make sense
    'use_foothold_constraints':                False,

    # works with all the mpc types except 'sampling'. In sim does not do much for now,
    # but in real it minizimes the delay between the mpc control and the state
    'use_RTI':                                 False,
    # If RTI is used, we can set the advance RTI-step! (Standard is the simpler RTI)
    # See https://arxiv.org/pdf/2403.07101.pdf
    'as_rti_type':                             "Standard",  # "AS-RTI-A", "AS-RTI-B", "AS-RTI-C", "AS-RTI-D", "Standard"
    'as_rti_iter':                             1,  # > 0, the higher the better, but slower computation!

    # This will force to use DDP instead of SQP, based on https://arxiv.org/abs/2403.10115.
    # Note that RTI is not compatible with DDP, and no state costraints for now are considered
    'use_DDP':                                 False,

    # this is used only in the case 'use_RTI' is false in a single mpc feedback loop.
    # More is better, but slower computation!
    'num_qp_iterations':                       1,

    # this is used to speeding up or robustify acados' solver (hpipm).
    'solver_mode':                             'balance',  # balance, robust, speed, crazy_speed


    # these is used only for the case 'input_rates', using as GRF not the actual state
    # of the robot of the predicted one. Can be activated to compensate
    # for the delay in the control loop on the real robot
    'use_input_prediction':                    False,

    # ONLY ONE CAN BE TRUE AT A TIME (only gradient)
    'use_static_stability':                    False,
    'use_zmp_stability':                       False,
    'trot_stability_margin':                   0.04,
    'pace_stability_margin':                   0.1,
    'crawl_stability_margin':                  0.04,  # in general, 0.02 is a good value

    # this is used to compensate for the external wrenches
    # you should provide explicitly this value in compute_control
    'external_wrenches_compensation':          True,
    'external_wrenches_compensation_num_step': 15,

    # this is used only in the case of collaborative mpc, to
    # compensate for the external wrench in the prediction (only collaborative)
    'passive_arm_compensation':                True,


    # Gain for Lyapunov-based MPC
    'K_z1': np.array([1, 1, 10]),
    'K_z2': np.array([1, 4, 10]),
    'residual_dynamics_upper_bound': 30,
    'use_residual_dynamics_decay': False,

    # ----- END properties for the gradient-based mpc -----


    # ----- START properties only for the sampling-based mpc -----

    # this is used only in the case 'sampling'.
    'sampling_method':                         'random_sampling',  # 'random_sampling', 'mppi', 'cem_mppi'
    'control_parametrization':                 'cubic_spline', # 'cubic_spline', 'linear_spline', 'zero_order'
    'num_splines':                             2,  # number of splines to use for the control parametrization
    'num_parallel_computations':               10000,  # More is better, but slower computation!
    'num_sampling_iterations':                 1,  # More is better, but slower computation!
    'device':                                  'gpu',  # 'gpu', 'cpu'
    # convariances for the sampling methods
    'sigma_cem_mppi':                          3,
    'sigma_mppi':                              3,
    'sigma_random_sampling':                   [0.2, 3, 10],
    'shift_solution':                          False,

    # ----- END properties for the sampling-based mpc -----
    }
# -----------------------------------------------------------------------

simulation_params = {
    'swing_generator':             'scipy',  # 'scipy', 'explicit'
    'swing_position_gain_fb':      500,
    'swing_velocity_gain_fb':      10,
    'impedence_joint_position_gain':  10.0,
    'impedence_joint_velocity_gain':  2.0,

    'step_height':                 0.2 * hip_height,  

    # Visual Foothold adapatation
    "visual_foothold_adaptation":  'blind', #'blind', 'height', 'vfa'

    # this is the integration time used in the simulator
    'dt':                          0.002,

    'gait':                        'trot',  # 'trot', 'pace', 'crawl', 'bound', 'full_stance'
    'gait_params':                 {'trot': {'step_freq': 1.4, 'duty_factor': 0.65, 'type': GaitType.TROT.value},
                                    'crawl': {'step_freq': 0.5, 'duty_factor': 0.8, 'type': GaitType.BACKDIAGONALCRAWL.value},
                                    'pace': {'step_freq': 1.4, 'duty_factor': 0.7, 'type': GaitType.PACE.value},
                                    'bound': {'step_freq': 1.8, 'duty_factor': 0.65, 'type': GaitType.BOUNDING.value},
                                    'full_stance': {'step_freq': 2, 'duty_factor': 0.65, 'type': GaitType.FULL_STANCE.value},
                                   },

    # This is used to activate or deactivate the reflexes upon contact detection
    'reflex_trigger_mode':       'tracking', # 'tracking', 'geom_contact', False
    'reflex_max_step_height':    0.5 * hip_height,  # this is the maximum step height that the robot can do if reflexes are enabled
    'reflex_next_steps_height_enhancement': False,
    'velocity_modulator': True,

    # velocity mode: human will give you the possibility to use the keyboard, the other are
    # forward only random linear-velocity, random will give you random linear-velocity and yaw-velocity
    'mode':                        'human',  # 'human', 'forward', 'random'
    'ref_z':                       hip_height,


    # the MPC will be called every 1/(mpc_frequency*dt) timesteps
    # this helps to evaluate more realistically the performance of the controller
    'mpc_frequency':               100,

    'use_inertia_recomputation':   True,

    'scene':                       'flat',  # flat, random_boxes, random_pyramids, perlin

    }
# -----------------------------------------------------------------------
