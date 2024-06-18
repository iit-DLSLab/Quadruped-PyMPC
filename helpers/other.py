import numpy as np
from scipy.signal import savgol_filter
import config
import copy


# Function to convert from quaternion to euler angles
def euler_from_quaternion(quaternion):
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # return -roll, -pitch, yaw
    return roll, pitch, yaw


# Function to filter the linear and angular velocities
def filter_state(signal_in, filter_type="mean"):
    # simple mean filter
    if (filter_type == "mean"):
        signal_out = signal_in[-1] * 0.8 + signal_in[-1 - 1] * 0.2
    elif (filter_type == "savgol"):
        signal_out = savgol_filter(signal_in, window_length=5, polyorder=2)
        signal_out = signal_out[-1]
    elif (filter_type == "none"):
        signal_out = signal_in[-1]
    return signal_out


def plot_state_matplotlib(queue, state_dict):
    state = np.concatenate((state_dict["position"], state_dict["linear_velocity"],
                            state_dict["orientation"], state_dict["angular_velocity"],
                            state_dict["foot_FL"], state_dict["foot_FR"],
                            state_dict["foot_RL"], state_dict["foot_RR"])).reshape((24,))


def check_zmp_constraint_satisfaction(state, contact_status, forces):
    base_w = copy.deepcopy(state['position'])
    base_vel_w = copy.deepcopy(state['linear_velocity'])

    FL = copy.deepcopy(state['foot_FL'])
    FR = copy.deepcopy(state['foot_FR'])
    RL = copy.deepcopy(state['foot_RL'])
    RR = copy.deepcopy(state['foot_RR'])

    yaw = copy.deepcopy(state['orientation'][2])
    h_R_w = np.zeros((2, 2))
    h_R_w[0, 0] = np.cos(yaw)
    h_R_w[0, 1] = np.sin(yaw)
    h_R_w[1, 0] = -np.sin(yaw)
    h_R_w[1, 1] = np.cos(yaw)

    FL[0:2] = h_R_w @ (FL[0:2] - base_w[0:2])
    FR[0:2] = h_R_w @ (FR[0:2] - base_w[0:2])
    RL[0:2] = h_R_w @ (RL[0:2] - base_w[0:2])
    RR[0:2] = h_R_w @ (RR[0:2] - base_w[0:2])

    foot_force_fl = forces[0:3]  # @self.centroidal_model.param[0]
    foot_force_fr = forces[3:6]  # @self.centroidal_model.param[1]
    foot_force_rl = forces[6:9]  # @self.centroidal_model.param[2]
    foot_force_rr = forces[9:12]  # @self.centroidal_model.param[3]
    temp = foot_force_fl + foot_force_fr + foot_force_rl + foot_force_rr
    gravity = np.array([0, 0, -9.81])
    linear_com_acc = (1 / config.mass) * temp + gravity

    if (config.mpc_params['use_zmp_stability']):
        gravity_z = 9.81
        robotHeight = base_w[2]
        zmp = base_w[0:2] - linear_com_acc[0:2] * (robotHeight / gravity_z)
        zmp = h_R_w @ (zmp - base_w[0:2])
        x = zmp[0]
        y = zmp[1]
    else:
        x = 0.0
        y = 0.0

    y_FL = FL[1]
    y_FR = FR[1]
    y_RL = RL[1]
    y_RR = RR[1]

    x_FL = FL[0]
    x_FR = FR[0]
    x_RL = RL[0]
    x_RR = RR[0]

    # LF - RF : x < (x2 - x1) (y - y1) / (y2 - y1) + x1
    # RF - RH: y > (y2 - y1) (x - x1) / (x2 - x1) + y1
    # RH - LH : x > (x2 - x1) (y - y1) / (y2 - y1) + x1
    # LH - LF: y < (y2 - y1) (x - x1) / (x2 - x1) + y1

    # FL and FR cannot stay at the same x! #constrint should be less than zero
    constraint_FL_FR = x - (x_FR - x_FL) * (y - y_FL) / (y_FR - y_FL + 0.001) - x_FL

    # FR and RR cannot stay at the same y! #constraint should be bigger than zero
    constraint_FR_RR = y - (y_RR - y_FR) * (x - x_FR) / (x_RR - x_FR + 0.001) - y_FR

    # RL and RR cannot stay at the same x! #constraint should be bigger than zero
    constraint_RR_RL = x - (x_RL - x_RR) * (y - y_RR) / (y_RL - y_RR + 0.001) - x_RR

    # FL and RL cannot stay at the same y! #constraint should be less than zero
    constraint_RL_FL = y - (y_FL - y_RL) * (x - x_RL) / (x_FL - x_RL + 0.001) - y_RL

    # the diagonal stuff can be at any point...
    constraint_FL_RR = y - (y_RR - y_FL) * (x - x_FL) / (x_RR - x_FL + 0.001) - y_FL  # bigger than zero
    constraint_FR_RL = y - (y_RL - y_FR) * (x - x_FR) / (x_RL - x_FR + 0.001) - y_FR  # bigger than zero

    FL_contact = contact_status[0]
    FR_contact = contact_status[1]
    RL_contact = contact_status[2]
    RR_contact = contact_status[3]

    violation = 0

    if (FL_contact == 1):
        if (FR_contact == 1):
            # ub_support_FL_FR = -0.0
            # lb_support_FL_FR = -1000
            if (constraint_FL_FR > 0):
                violation = violation + 1

        else:
            # ub_support_FL_RR = 1000
            # lb_support_FL_RR = 0.0
            if (constraint_FL_RR < 0):
                violation = violation + 1

    if (FR_contact == 1):
        if (RR_contact == 1):
            # ub_support_FR_RR = 1000
            # lb_support_FR_RR = 0.0
            if (constraint_FR_RR < 0):
                violation = violation + 1

        else:
            ub_support_FR_RL = 1000
            lb_support_FR_RL = 0.0
            # if(constraint_FR_RL < 0):
            # violation = violation + 1
            # TOCHECK

    if (RR_contact == 1):
        if (RL_contact == 1):
            # ub_support_RR_RL = 1000
            # lb_support_RR_RL = 0.0
            if (constraint_RR_RL < 0):
                violation = violation + 1

        else:
            # ub_support_FL_RR = -0.0
            # lb_support_FL_RR = -1000
            if (constraint_FL_RR > 0):
                violation = violation + 1 * 0

    if (RL_contact == 1):
        if (FL_contact == 1):
            # ub_support_RL_FL = -0.0
            # lb_support_RL_FL = -1000
            if (constraint_RL_FL > 0):
                violation = violation + 1
        else:
            # ub_support_FR_RL = -0.0
            # lb_support_FR_RL = -1000
            if (constraint_FR_RL > 0):
                violation = violation + 1

    if (FL_contact == 1 and FR_contact == 1 and RL_contact == 1 and RR_contact == 1):
        violation = 0

    if (violation >= 1):
        return True
    else:
        return False
