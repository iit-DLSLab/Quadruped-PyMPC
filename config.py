"""
This file includes all of the configuration parameters for the MPC controllers 
and of the internal simulations that can be launch from the folder /simulation.
"""
import numpy as np


# These are used both for a real experiment and a simulation -----------
robot = 'aliengo' #'go2', 'aliengo', 'hyqreal'

if(robot == 'go2'):
    mass = 15.019
    inertia = np.array([[ 1.58460467e-01,  1.21660000e-04, -1.55444692e-02],
                        [ 1.21660000e-04,  4.68645637e-01, -3.12000000e-05],
                        [-1.55444692e-02, -3.12000000e-05,  5.24474661e-01]])
    urdf_filename = "go2.urdf"

elif(robot == 'aliengo'):
    mass = 24.637
    inertia = np.array([[ 0.2310941359705289, -0.0014987128245817424, -0.021400468992761768],
                        [-0.0014987128245817424, 1.4485084687476608, 0.0004641447134275615],
                        [-0.021400468992761768, 0.0004641447134275615, 1.503217877350808]])
    
    urdf_filename = "aliengo.urdf"

elif(robot == 'hyqreal'):
    mass = 108.40 
    inertia = np.array([[ 4.55031444e+00,  2.75249434e-03, -5.11957307e-01],
                        [ 2.75249434e-03,  2.02411774e+01, -7.38560592e-04],
                        [-5.11957307e-01, -7.38560592e-04,  2.14269772e+01]])
    urdf_filename = "hyqreal.urdf"
    


mpc_params = {
    # 'nominal' optimized directly the GRF
    # 'input_rates' optimizes the delta GRF
    # 'sampling' is a gpu-based mpc that samples the GRF
    # 'collaborative' optimized directly the GRF and has a passive arm model inside 
    'type': 'nominal',
    
    # horizon is the number of timesteps in the future that the mpc will optimize
    # dt is the discretization time used in the mpc
    'horizon': 12,
    'dt': 0.02, 

    # GRF limits for each single leg
    "grf_max": mass*9.81,
    "grf_min": 0,
    'mu': 0.5,
    

    # ----- START properties only for the gradient-based mpc -----
    
    # this is used if you want to manually warm start the mpc 
    'use_warm_start': False, 

    # this enables integrators for height, linear velocities, roll and pitch 
    'use_integrators': False,
    'alpha_integrator': 0.1,
    'integrator_cap': [0.5, 0.2, 0.2, 0.0, 0.0, 1.0],

    # if this is off, the mpc will not optimize the footholds and will
    # use only the ones provided in the reference
    'use_foothold_optimization': True,
    
    # this is set to false automatically is use_foothold_optimization is false
    # because in that case we cannot chose the footholds and foothold
    # constraints do not any make sense
    'use_foothold_constraints': False,

    # works with all the mpc types except 'sampling'. In sim does not do much for now,
    # but in real it minizimes the delay between the mpc control and the state
    'use_RTI': False,
    # If RTI is used, we can set the advance RTI-step! (Standard is the simpler RTI)
    # See https://arxiv.org/pdf/2403.07101.pdf
    'as_rti_type': "AS-RTI-A",  # "AS-RTI-A", "AS-RTI-B", "AS-RTI-C", "AS-RTI-D", "Standard"
    'as_rti_iter': 1, # > 0, the higher the better, but slower computation!

    # this is used only in the case 'use_RTI' is false in a single mpc feedback loop. 
    # More is better, but slower computation!
    'num_qp_iterations': 1,

    # this is used to limit the number of interior point iterations and choose
    # "speed" in hpipm. This gives a more costant solution time. 
    'prioritize_speed': True,


    # this is used to have a smaller dt near the start of the horizon 
    'use_nonuniform_discretization': False,
    'horizon_fine_grained': 2,
    'dt_fine_grained': 0.01,


    # these is used only for the case 'input_rates'
    # for some reason, real-exp works only with this enabled
    # while simulation works only with this disabled.....
    'use_input_prediction': False,

    # ONLY ONE CAN BE TRUE AT A TIME (only gradient)
    'use_static_stability': False,
    'use_zmp_stability': False,
    'trot_stability_margin': 0.04,
    'pace_stability_margin': 0.1,
    'crawl_stability_margin': 0.04, #in general, 0.02 is a good value
    
    
    # this is used to compensate for the external wrenches
    # you should provide explicitly this value in compute_control 
    'external_wrenches_compensation': True,
    'external_wrenches_compensation_num_step': 15,

    # this is used only in the case of collaborative mpc, to 
    # compensate for the external wrench in the prediction (only collaborative)
    'passive_arm_compensation': True,

    # ----- END properties for the gradient-based mpc -----



    # ----- START properties only for the sampling-based mpc -----

    # this is used only in the case 'sampling'. 
    'sampling_method': 'random_sampling', #'random_sampling', 'mppi', 'cem_mppi'
    'control_parametrization': 'cubic_spline', #'cubic_spline', 'linear_spline_1', 'linear_spline_2', 'zero_order'
    'num_parallel_computations': 10000, # More is better, but slower computation!
    'num_sampling_iterations': 3, # More is better, but slower computation!
    # convariances for the sampling methods
    'sigma_cem_mppi': 3, 
    'sigma_mppi': 3,
    'sigma_random_sampling': [0.2, 3, 10],
    'shift_solution': False,
    
    # if this is true, sampling will be done for the step frequency as well
    'optimize_step_freq': True,
    #'step_freq_delta': [0.0, 0.7, 1.0]
    'step_freq_delta': [1.3, 2.0, 2.4]

    # ----- END properties for the sampling-based mpc -----
    
}
#-----------------------------------------------------------------------


# This is only used if a simulation in the folder /simulation is run ---
if(robot == 'go2'):
    ref_z = 0.28
elif(robot == 'aliengo'):
    ref_z = 0.35
elif(robot == 'hyqreal'):
    ref_z = 0.5

simulation_params = {
    'swing_generator': 'explicit', #'scipy', 'explicit', 'ndcurves'
    'swing_position_gain_fb': 5000,
    'swing_velocity_gain_fb': 100,
    'swing_integral_gain_fb': 0,
    'step_height': 0.05, #0.05 go2

    # this is the integration time used in the simulator
    'dt': 0.002,

    'gait': 'trot', #'trot', 'pace', 'crawl', 'bound', 'full_stance'
    
    # ref_x_dot, ref_y_dot, ref_yaw_dot are in the horizontal frame
    'ref_x_dot': .3,
    'ref_y_dot': 0.,
    'ref_yaw_dot': 0.0,
    'ref_z': ref_z, 
    'ref_pitch': 0.0,
    'ref_roll': 0,

    # the MPC will be called every 1/(mpc_frequency*dt) timesteps
    # this helps to evaluate more realistically the performance of the controller
    'mpc_frequency': 200, 


    'use_external_disturbances': True,
    'external_disturbances_bound': [18, 18, 0, 18, 18, 18], #fx, fy, fz, mx, my, mz

    'use_print_debug': False,
    'use_visualization_debug': True,

    'use_kind_of_real_time': True,

}
#-----------------------------------------------------------------------