"""This file includes all of the configuration parameters for the MPC controllers
and of the internal simulations that can be launch from the folder /simulation.
"""
import numpy as np
from quadruped_pympc.helpers.quadruped_utils import GaitType

# These are used both for a real experiment and a simulation -----------
# These are the only attributes needed per quadruped, the rest can be computed automatically ----------------------
robot = 'aliengo'  # 'go2', 'aliengo', 'hyqreal', 'mini_cheetah'  aliengo_arm # TODO: Load from robot_descriptions.py
robot_leg_joints = dict(FL=['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',],  # TODO: Make configs per robot.
                        FR=['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',],
                        RL=['RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',],
                        RR=['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',])
robot_feet_geom_names = dict(FL='FL', FR='FR', RL='RL', RR='RR')
qpos0_js = None  # Zero joint-space configuration. If None it will be extracted from the URDF.

# ----------------------------------------------------------------------------------------------------------------
if (robot == 'go2'):
    mass = 15.019
    inertia = np.array([[1.58460467e-01, 1.21660000e-04, -1.55444692e-02],
                        [1.21660000e-04, 4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05, 5.24474661e-01]])
    urdf_filename = "go2.urdf"
    hip_height = 0.28

elif (robot == 'aliengo'):
    mass = 24.637
    inertia = np.array([[0.2310941359705289, -0.0014987128245817424, -0.021400468992761768],
                        [-0.0014987128245817424, 1.4485084687476608, 0.0004641447134275615],
                        [-0.021400468992761768, 0.0004641447134275615, 1.503217877350808]])

    urdf_filename = "aliengo.urdf"
    hip_height = 0.35

elif (robot == 'hyqreal'):
    mass = 108.40
    inertia = np.array([[4.55031444e+00, 2.75249434e-03, -5.11957307e-01],
                        [2.75249434e-03, 2.02411774e+01, -7.38560592e-04],
                        [-5.11957307e-01, -7.38560592e-04, 2.14269772e+01]])
    urdf_filename = "hyqreal.urdf"
    robot_leg_joints = dict(FL=['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',],
                            FR=['rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',],
                            RL=['lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',],
                            RR=['rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint',])
    hip_height = 0.5
elif (robot == 'mini_cheetah'):
    mass = 12.5
    inertia = np.array([[1.58460467e-01, 1.21660000e-04, -1.55444692e-02],
                        [1.21660000e-04, 4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05, 5.24474661e-01]])
    urdf_filename = "mini_cheetah.urdf"
    hip_height = 0.225
    qpos0_js = np.concatenate((np.array([0, -np.pi/2, 0] * 2), np.array([0, np.pi/2, 0] * 2)))


mpc_params = {
    # 'nominal' optimized directly the GRF
    # 'input_rates' optimizes the delta GRF
    # 'sampling' is a gpu-based mpc that samples the GRF
    # 'collaborative' optimized directly the GRF and has a passive arm model inside
    'type':                                    'sampling',  # 'nominal', 'input_rates', 'sampling', 'collaborative'
    # horizon is the number of timesteps in the future that the mpc will optimize
    # dt is the discretization time used in the mpc
    'horizon':                                 12,
    'dt':                                      0.02,
    'mu':                                      0.5,

    # GRF limits for each single leg
    "grf_max":                                 mass * 9.81,
    "grf_min":                                 0,
    
    # if this is true, we optimize the step frequency as well
    # for the sampling controller, this is done in the rollout
    # for the gradient-based controller, this is done with a batched version of the ocp
    'optimize_step_freq':                      True,
    'step_freq_available':                     [1.4, 2.0, 2.4],
    'use_nonuniform_discretization':           False,

    }
mpc_nominal_params = {

    # ----- START properties only for the gradient-based mpc -----
    "grf_max":                                 mass * 9.81,
    "grf_min":                                 0,
    # this is used if you want to manually warm start the mpc
    'use_warm_start':                          False,

    # this enables integrators for height, linear velocities, roll and pitch
    'use_integrators':                         True,
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
    'as_rti_type':                             "AS-RTI-A",  # "AS-RTI-A", "AS-RTI-B", "AS-RTI-C", "AS-RTI-D", "Standard"
    'as_rti_iter':                             1,  # > 0, the higher the better, but slower computation!

    # This will force to use DDP instead of SQP, based on https://arxiv.org/abs/2403.10115.
    # Note that RTI is not compatible with DDP, and no state costraints for now are considered
    'use_DDP':                                 False,

    # this is used only in the case 'use_RTI' is false in a single mpc feedback loop.
    # More is better, but slower computation!
    'num_qp_iterations':                       1,

    # this is used to speeding up or robustify acados' solver (hpipm).
    'solver_mode':                             'balance',  # balance, robust, speed, crazy_speed

    # this is used to have a smaller dt near the start of the horizon
    'use_nonuniform_discretization':           False,
    'horizon_fine_grained':                    2,
    'dt_fine_grained':                         0.01,

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

    # ----- END properties for the gradient-based mpc -----    
}
mpc_sampling_params = {
    # ----- START properties only for the sampling-based mpc -----
    # this is used only in the case 'sampling'.
    'sampling_method':                         'random_sampling',  # 'random_sampling', 'mppi', 'cem_mppi'
    'control_parametrization':                 'cubic_spline_1',
    # 'cubic_spline_1', 'cubic_spline_2', 'linear_spline_1', 'linear_spline_2', 'zero_order'
    'num_parallel_computations':               10000,  # More is better, but slower computation!
    'num_sampling_iterations':                 1,  # More is better, but slower computation!
    # convariances for the sampling methods
    'sigma_cem_mppi':                          3,
    'sigma_mppi':                              3,
    'sigma_random_sampling':                   [0.2, 3, 10],
    'shift_solution':                          False,
}
# -----------------------------------------------------------------------

simulation_params = {
    
    'swing_generator':             'scipy',  # 'scipy', 'explicit', 'ndcurves'
    'swing_position_gain_fb':      5000,
    'swing_velocity_gain_fb':      100,
    'swing_integral_gain_fb':      0,
    'step_height':                 0.3 * hip_height,  # 0.05 go2

    # this is the integration time used in the simulator
    'dt':                          0.002,

    'gait':                        'trot',  # 'trot', 'pace', 'crawl', 'bound', 'full_stance'
    'gait_params':                 {'trot': {'step_freq': 1.4, 'duty_factor': 0.65, 'type': GaitType.TROT.value},
                                    'crawl': {'step_freq': 0.7, 'duty_factor': 0.9, 'type': GaitType.BACKDIAGONALCRAWL.value},
                                    'pace': {'step_freq': 2, 'duty_factor': 0.7, 'type': GaitType.PACE.value},
                                    'bound': {'step_freq': 4, 'duty_factor': 0.65, 'type': GaitType.BOUNDING.value},
                                    'full_stance': {'step_freq': 2, 'duty_factor': 0.65, 'type': GaitType.FULL_STANCE.value},
                                   },

    # velocity mode: human will give you the possibility to use the keyboard, the other are
    # forward only random linear-velocity, random will give you random linear-velocity and yaw-velocity
    'mode':                        'human',  # 'human', 'forward', 'random'
    'ref_z':                       hip_height,


    # the MPC will be called every 1/(mpc_frequency*dt) timesteps
    # this helps to evaluate more realistically the performance of the controller
    'mpc_frequency':               100,

    'use_inertia_recomputation':   True,

    'use_print_debug':             False,
    'use_visualization_debug':     True,

    'use_kind_of_real_time':       True,

    'scene':                       'flat',  # flat, rough, stairs, suspend_stairs, slope, perlin, image

    }
# -----------------------------------------------------------------------
