"""This file includes all of the configuration parameters for the MPC controllers
and of the internal simulations that can be launch from the folder /simulation.
"""
import numpy as np
from quadruped_pympc.helpers.quadruped_utils import GaitType

# These are used both for a real experiment and a simulation -----------
# These are the only attributes needed per quadruped, the rest can be computed automatically ----------------------
robot = 'go2'  # 'aliengo', 'go1', 'go2', 'a2', 'b2', 'hyqreal1', 'hyqreal2', 'mini_cheetah', 'spot', 'pegasus'

from gym_quadruped.robot_cfgs import RobotConfig, get_robot_config
robot_cfg: RobotConfig = get_robot_config(robot_name=robot)
robot_leg_joints = robot_cfg.leg_joints
robot_feet_geom_names = robot_cfg.feet_geom_names
qpos0_js = robot_cfg.qpos0_js
hip_height = robot_cfg.hip_height

# ----------------------------------------------------------------------------------------------------------------
# Nominal hip-to-foot offsets (m): +x front/-x rear, +y left/-y right, in the horizontal heading frame.
# Mass and SRBD inertia (whole robot, about its CoM, expressed in the base frame) in the 'home' keyframe of the gym_quadruped MJCF (foot collision bodies excluded)
if (robot == 'go1'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 12.743
    inertia = np.array([[ 1.268390e-01, -4.377506e-04, -1.432618e-02],
                        [-4.377506e-04,  3.940320e-01, -9.140054e-05],
                        [-1.432618e-02, -9.140054e-05,  4.228811e-01]])

elif (robot == 'go2'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 15.206
    inertia = np.array([[ 1.702904e-01,  1.216598e-04, -1.647458e-02],
                        [ 1.216598e-04,  4.837366e-01, -3.120047e-05],
                        [-1.647458e-02, -3.120047e-05,  5.353721e-01]])

elif (robot == 'a2'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 40.071
    inertia = np.array([[ 7.689768e-01,  6.960592e-04, -9.620317e-02],
                        [ 6.960592e-04,  2.048805e+00,  2.700440e-06],
                        [-9.620317e-02,  2.700440e-06,  2.428280e+00]])

elif (robot == 'aliengo'):
    hip_offset_x = 0.0
    hip_offset_y = 0.083
    mass = 24.638
    inertia = np.array([[ 2.325974e-01, -1.025321e-03, -1.614396e-02],
                        [-1.025321e-03,  8.951498e-01, -6.527086e-04],
                        [-1.614396e-02, -6.527086e-04,  9.195693e-01]])

elif (robot == 'b2'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 83.498
    inertia = np.array([[ 1.654483e+00, -1.608186e-02, -2.646546e-01],
                        [-1.608186e-02,  7.004132e+00, -3.650494e-03],
                        [-2.646546e-01, -3.650494e-03,  7.561386e+00]])

elif (robot == 'hyqreal1'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 107.573
    inertia = np.array([[ 4.540910e+00,  5.145509e-03, -5.106845e-01],
                        [ 5.145509e-03,  2.018049e+01, -8.456211e-04],
                        [-5.106845e-01, -8.456211e-04,  2.136024e+01]])

elif (robot == 'hyqreal2'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 126.694
    inertia = np.array([[ 4.893050e+00,  2.716032e-05, -1.849255e-01],
                        [ 2.716032e-05,  1.781918e+01, -6.545602e-03],
                        [-1.849255e-01, -6.545602e-03,  1.846831e+01]])

elif (robot == 'mini_cheetah'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 12.473
    inertia = np.array([[ 9.811458e-02,  3.536596e-04,  2.504832e-04],
                        [ 3.536596e-04,  2.804790e-01, -2.740048e-05],
                        [ 2.504832e-04, -2.740048e-05,  3.522035e-01]])

elif (robot == 'spot'):
    hip_offset_x = 0.0
    hip_offset_y = 0.095
    mass = 50.340
    inertia = np.array([[ 6.609257e-01,  8.818121e-05, -1.394429e-01],
                        [ 8.818121e-05,  2.057524e+00,  8.727701e-05],
                        [-1.394429e-01,  8.727701e-05,  2.189357e+00]])

elif (robot == 'pegasus'):
    hip_offset_x = 0.112
    hip_offset_y = 0.148
    mass = 83.077
    inertia = np.array([[ 3.666069e+00, -1.496914e-02, -2.939266e-01],
                        [-1.496914e-02,  1.108444e+01, -9.354534e-04],
                        [-2.939266e-01, -9.354534e-04,  1.270721e+01]])


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

    # If True, the Riccati feedback gain of the first stage is taken from acados (hpipm), and the whole body
    # controller corrects the MPC GRFs at every control step with GRF = GRF_mpc + K (x_now - x_mpc), closing
    # the loop without waiting for the next MPC solution. Only for the 'nominal' mpc, not with 'use_DDP'.
    # For the 'sampling' mpc (mppi and cem_mppi) the gain is computed as in Feedback-MPPI
    # (https://arxiv.org/abs/2506.14855), differentiating the MPPI weights through the rollouts
    'use_riccati_feedback':                    False,

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

    # this is used to speequanto manca?ding up or robustify acados' solver (hpipm).
    'solver_mode':                             'balance',  # balance, robust, fast, crazy_speed


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
    'sampling_method':                         'ot_mpc',  # 'random_sampling', 'mppi', 'cem_mppi', 'ot_mpc'
    'control_parametrization':                 'cubic_spline', # 'cubic_spline', 'linear_spline', 'zero_order'
    'num_splines':                             2,  # number of splines to use for the control parametrization
    'num_parallel_computations':               10000,  # More is better, but slower computation!
    'num_sampling_iterations':                 1,  # More is better, but slower computation!
    'device':                                  'gpu',  # 'gpu', 'cpu'
    # convariances for the sampling methods
    'sigma_cem_mppi':                          20.0,
    'sigma_cem_mppi_reset_every':              2,  # reset the cem_mppi covariance every k mpc calls (1 = always)
    'sigma_mppi':                              20.0,
    'temperature_mppi':                        0.03,  # relative to the cost spread. Lower = greedier (better tracking), higher = smoother (more robust)
    'sigma_random_sampling':                   [1.0, 8.0, 20.0],
    # OT-MPC (https://arxiv.org/abs/2605.02147): particles move toward the low-cost proposals they are coupled with
    # by entropic optimal transport, instead of the global MPPI average. It uses also temperature_mppi
    'sigma_ot_mpc':                            20.0,
    'ot_mpc_num_particles':                    16,  # candidate solutions kept across mpc calls
    'ot_mpc_epsilon':                          0.05,  # entropic regularization, relative to the median transport cost
    'ot_mpc_step_size':                        0.7,  # relaxation of the barycentric update, in (0, 1]
    'ot_mpc_exploration':                      0.1,  # fraction of proposals sampled around zero instead of the particles
    'ot_mpc_sinkhorn_iterations':              30,
    'shift_solution':                          False,
    # refine the sampled solution with a few Adam steps on the gradient of the rollout cost (0 = off).
    # The refined solution is used only if it decreases the cost
    'gradient_refinement_steps':               2,
    'gradient_refinement_lr':                  0.5,  # [N] approximately the change of each parameter per step

    # ----- END properties for the sampling-based mpc -----
    }
# -----------------------------------------------------------------------

simulation_params = {
    'swing_generator':             'hermite',  # 'scipy', 'hermite', 'bezier'
    'swing_position_gain_fb':      500,
    'swing_velocity_gain_fb':      10,
    'impedence_joint_position_gain':  10.0,
    'impedence_joint_velocity_gain':  2.0,

    # Joint friction compensation: viscous damping (qfrc_passive) and Coulomb friction (model frictionloss)
    'use_friction_compensation':   True,
    'friction_compensation_vel_eps': 0.1,  # [rad/s] velocity (above the deadband) at which the smoothed Coulomb sign saturates
    'friction_compensation_vel_deadband': 0.4,  # [rad/s] no Coulomb compensation below this velocity, set above the joint velocity noise
    'friction_compensation_ratio': 0.8,  # fraction of the friction (viscous and Coulomb) that is compensated, in [0, 1]

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

    'scene':                       'random_boxes',  # flat, random_boxes, random_pyramids, perlin

    }
# -----------------------------------------------------------------------
