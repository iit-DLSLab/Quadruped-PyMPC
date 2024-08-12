import os
import numpy as np

class SwingTrajectoryController:
    def __init__(self,
                 step_height: float,
                 swing_period: float,
                 position_gain_fb: np.ndarray,
                 velocity_gain_fb: np.ndarray,
                 generator: str) -> None:

        self.generator = generator

        if (self.generator == "ndcurves"):
            from .swing_generators.ndcurves_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        elif (self.generator == "scipy"):
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)

        self.position_gain_fb = position_gain_fb
        self.velocity_gain_fb = velocity_gain_fb
        self.swing_period = swing_period
        self.swing_time = [0, 0, 0, 0]

    def regenerate_swing_trajectory_generator(self, step_height: float, swing_period: float) -> None:
        if (self.generator == "ndcurves"):
            from .swing_generators.ndcurves_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        elif (self.generator == "scipy"):
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        self.swing_period = swing_period

    def compute_swing_control(self,
                              leg_id,
                              q_dot,
                              J,
                              J_dot,
                              lift_off,
                              touch_down,
                              foot_pos,
                              foot_vel,
                              h,
                              mass_matrix):
        """TODO: Docstring.

        Args:
        ----
            model:
            q:
            q_dot:
            J:
            J_dot:
            lift_off:
            touch_down:
            swing_time:
            foot_pos:
            foot_vel:
            h:
            mass_matrix:

        Returns:
        -------

        """
        # Compute trajectory references
        des_foot_pos, des_foot_vel, des_foot_acc = self.swing_generator.compute_trajectory_references(
            self.swing_time[leg_id],
            lift_off,
            touch_down
            )

        err_pos = des_foot_pos - foot_pos
        err_pos = err_pos.reshape((3,))

        err_vel = des_foot_vel - foot_vel
        err_vel = err_vel.reshape((3,))

        accelleration = des_foot_acc + \
                        self.position_gain_fb * (err_pos) + \
                        self.velocity_gain_fb * (err_vel)

        accelleration = accelleration.reshape((3,))

        # Compute inertia matrix in task space.
        # Mass Matrix and centrifugal missing
        tau_swing = J.T @ (self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel))
        tau_swing += mass_matrix @ np.linalg.pinv(J) @ (accelleration - J_dot @ q_dot) + h

        return tau_swing, des_foot_pos, des_foot_vel

    def update_swing_time(self, current_contact, legs_order, dt):
        for leg_id, leg_name in enumerate(legs_order):
            # Swing time reset
            if current_contact[leg_id] == 0:
                if self.swing_time[leg_id] < self.swing_period:
                    self.swing_time[leg_id] = self.swing_time[leg_id] + dt
            else:
                self.swing_time[leg_id] = 0

    def check_apex_condition(self, current_contact, interval=0.02):
        optimize_swing = 0
        for leg_id in range(4):
            # Swing time check
            if (current_contact[leg_id] == 0):
                if ((self.swing_time[leg_id] > (self.swing_period / 2.) - interval) and \
                        (self.swing_time[leg_id] < (self.swing_period / 2.) + interval)):
                    optimize_swing = 1
        return optimize_swing

    def check_full_stance_condition(self, current_contact):
        stance = 1
        # If one leg is not in stance, the robot is not in full stance
        for leg_id in range(4):
            if (current_contact[leg_id] == 0):
                stance = 0
        return stance


# Example:
if __name__ == "__main__":
    import copy

    import mujoco
    import mujoco.viewer
    from swing_trajectory_controller import SwingTrajectoryController
    import matplotlib.pyplot as plt

    from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco

    # Mujoco model and data
    m = mujoco.MjModel.from_xml_path('./../external/mujoco_menagerie/unitree_go2/scene.xml')
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    simulation_dt = 0.002

    # Get the ids of the contact points, useful for computing
    # contact jacobian later
    FL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FL')
    FR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'FR')
    RL_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RL')
    RR_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'RR')

    FL_geom = m.geom("FL")
    FR_geom = m.geom("FR")
    RL_geom = m.geom("RL")
    RR_geom = m.geom("RR")

    FL_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FL_calf')
    FR_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'FR_calf')
    RL_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RL_calf')
    RR_calf_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'RR_calf')

    # Jacobian matrices
    jac_foot_FL = np.zeros((3, m.nv))
    jac_foot_FR = np.zeros((3, m.nv))
    jac_foot_RL = np.zeros((3, m.nv))
    jac_foot_RR = np.zeros((3, m.nv))

    # Previous jacobian matrices
    jac_foot_FL_prev = np.zeros((3, m.nv))
    jac_foot_FR_prev = np.zeros((3, m.nv))
    jac_foot_RL_prev = np.zeros((3, m.nv))
    jac_foot_RR_prev = np.zeros((3, m.nv))

    # Derivative of the jacobian matrices
    jac_foot_FL_dot = np.zeros((3, m.nv))
    jac_foot_FR_dot = np.zeros((3, m.nv))
    jac_foot_RL_dot = np.zeros((3, m.nv))
    jac_foot_RR_dot = np.zeros((3, m.nv))

    # Previous foot positions
    position_foot_FL_prev = np.zeros((3,))
    position_foot_FR_prev = np.zeros((3,))
    position_foot_RL_prev = np.zeros((3,))
    position_foot_RR_prev = np.zeros((3,))

    # Torque vectors
    tau_FL = np.zeros((m.nv, 1))
    tau_FR = np.zeros((m.nv, 1))
    tau_RL = np.zeros((m.nv, 1))
    tau_RR = np.zeros((m.nv, 1))

    swing_time = [0, 0, 0, 0]
    lift_off_positions = [np.zeros((3, 1)) for _ in range(4)]
    step_frequency = 1.5
    duty_factor = 0.5
    step_height = 0.08  # + 0.3
    # swing_period = (1 - duty_factor) * (1 / step_frequency)
    swing_period = 1
    position_gain_fb = 1500
    velocity_gain_fb = 10
    stc = SwingTrajectoryController(step_height=step_height, swing_period=swing_period,
                                    position_gain_fb=position_gain_fb, velocity_gain_fb=velocity_gain_fb,
                                    generator="scipy")

    d.qpos[2] = 0.6
    mujoco.mj_step(m, d)
    position_foot_FL = d.geom_xpos[FL_id]
    position_foot_FL_prev = copy.deepcopy(position_foot_FL)
    lift_off_positions[0] = copy.deepcopy(position_foot_FL)
    desired_touch_down_FL = copy.deepcopy(position_foot_FL)
    desired_touch_down_FL[0] += 0.15

    position_foot_FR = d.geom_xpos[FR_id]
    position_foot_FR_prev = copy.deepcopy(position_foot_FR)
    lift_off_positions[1] = copy.deepcopy(position_foot_FR)
    desired_touch_down_FR = copy.deepcopy(position_foot_FR)
    desired_touch_down_FR[0] += 0.15

    position_foot_RL = d.geom_xpos[RL_id]
    position_foot_RL_prev = copy.deepcopy(position_foot_RL)
    lift_off_positions[2] = copy.deepcopy(position_foot_RL)
    desired_touch_down_RL = copy.deepcopy(position_foot_RL)
    desired_touch_down_RL[0] += 0.15

    position_foot_RR = d.geom_xpos[RR_id]
    position_foot_RR_prev = copy.deepcopy(position_foot_RR)
    lift_off_positions[3] = copy.deepcopy(position_foot_RR)
    desired_touch_down_RR = copy.deepcopy(position_foot_RR)
    desired_touch_down_RR[0] += 0.15

    # Plot stuff
    swing_FL_evolution = np.zeros((6, 100))
    swing_FL_evolution_reference = np.zeros((6, 100))
    x = np.linspace(0, 100, 100)
    fig, (ax_x, ax_y, ax_z, ax_x_dot, ax_y_dot, ax_z_dot) = plt.subplots(6, sharex=True)

    (ln_x,) = ax_x.plot(x, np.sin(x), animated=True)
    ax_x.set_title('LF swing tracking')
    ax_x.set_ylabel('x')
    ax_x.set_ylim(-0.1, 0.5)

    (ln_y,) = ax_y.plot(x, np.sin(x), animated=True)
    ax_y.set_ylabel('y')
    ax_y.set_ylim(-0.1, 0.5)

    (ln_z,) = ax_z.plot(x, np.sin(x), animated=True)
    ax_z.set_ylabel('z')
    ax_z.set_ylim(0.2, 0.5)

    (ln_x_dot,) = ax_x_dot.plot(x, np.sin(x), animated=True)
    ax_x_dot.set_ylabel('x_dot')

    (ln_y_dot,) = ax_y_dot.plot(x, np.sin(x), animated=True)
    ax_y_dot.set_ylabel('y_dot')

    (ln_z_dot,) = ax_z_dot.plot(x, np.sin(x), animated=True)
    ax_z_dot.set_ylabel('z_dot')
    ax_z_dot.set_ylim(-0.2, 0.2)

    plt.show(block=False)
    plt.pause(0.1)
    bg = fig.canvas.copy_from_bbox(fig.bbox)
    ax_x.draw_artist(ln_x)
    ax_y.draw_artist(ln_y)
    ax_z.draw_artist(ln_z)
    ax_x_dot.draw_artist(ln_x_dot)
    ax_y_dot.draw_artist(ln_y_dot)
    ax_z_dot.draw_artist(ln_z_dot)
    fig.canvas.blit(fig.bbox)

    # Main simulation loop ------------------------------------------------------------------
    with mujoco.viewer.launch_passive(m, d) as viewer:
        i = 0
        while True:
            d.qpos[2] = 0.6
            d.qvel[0:6] = 0.0

            # Compute the jacobian of the contact points
            mujoco.mj_jac(m, d, jac_foot_FL, None, d.geom_xpos[FL_id], FL_calf_id)
            mujoco.mj_jac(m, d, jac_foot_FR, None, d.geom_xpos[FR_id], FR_calf_id)
            mujoco.mj_jac(m, d, jac_foot_RL, None, d.geom_xpos[RL_id], RL_calf_id)
            mujoco.mj_jac(m, d, jac_foot_RR, None, d.geom_xpos[RR_id], RR_calf_id)

            # Compute jacobian derivatives of the contact points
            jac_foot_FL_dot = (jac_foot_FL - jac_foot_FL_prev) / simulation_dt
            jac_foot_FR_dot = (jac_foot_FR - jac_foot_FR_prev) / simulation_dt
            jac_foot_RL_dot = (jac_foot_RL - jac_foot_RL_prev) / simulation_dt
            jac_foot_RR_dot = (jac_foot_RR - jac_foot_RR_prev) / simulation_dt

            # Update previous jacobians
            jac_foot_FL_prev = copy.deepcopy(jac_foot_FL)
            jac_foot_FR_prev = copy.deepcopy(jac_foot_FR)
            jac_foot_RL_prev = copy.deepcopy(jac_foot_RL)
            jac_foot_RR_prev = copy.deepcopy(jac_foot_RR)

            position_foot_FL = d.geom_xpos[FL_id]
            position_foot_FR = d.geom_xpos[FR_id]
            position_foot_RL = d.geom_xpos[RL_id]
            position_foot_RR = d.geom_xpos[RR_id]

            velocity_foot_FL = (position_foot_FL - position_foot_FL_prev) / simulation_dt
            velocity_foot_FR = (position_foot_FR - position_foot_FR_prev) / simulation_dt
            velocity_foot_RL = (position_foot_RL - position_foot_RL_prev) / simulation_dt
            velocity_foot_RR = (position_foot_RR - position_foot_RR_prev) / simulation_dt

            # Update previous foot positions
            position_foot_FL_prev = copy.deepcopy(position_foot_FL)
            position_foot_FR_prev = copy.deepcopy(position_foot_FR)
            position_foot_RL_prev = copy.deepcopy(position_foot_RL)
            position_foot_RR_prev = copy.deepcopy(position_foot_RR)

            # Compute the reference for the swing trajectory

            if swing_time[0] > swing_period:
                swing_time[0] = 0
            else:
                swing_time[0] = swing_time[0] + simulation_dt

            # The swing controller is in the end-effector space. For its computation,
            # we save for simplicity joints position and velocities
            joints_pos_FL = d.qpos[6:9]
            joints_vel_FL = d.qvel[6:9]

            joints_pos_FR = d.qpos[9:12]
            joints_vel_FR = d.qvel[9:12]

            joints_pos_RL = d.qpos[12:15]
            joints_vel_RL = d.qvel[12:15]

            joints_pos_RR = d.qpos[15:18]
            joints_vel_RR = d.qvel[15:18]

            print("d.qpos", d.qpos[6:])

            # centrifugal, coriolis, gravity
            h_FL = d.qfrc_bias[6:9]
            h_FR = d.qfrc_bias[9:12]
            h_RL = d.qfrc_bias[12:15]
            h_RR = d.qfrc_bias[15:18]

            # and inertia matrix
            mass_matrix = np.zeros((m.nv, m.nv))
            mujoco.mj_fullM(m, mass_matrix, d.qM)
            mass_matrix_FL = mass_matrix[6:9, 6:9]

            # breakpoint()

            # If the foot is not in stance, we can calculate the swing controller
            tau_FL, \
                desired_swing_foot_position_FL, \
                desired_swing_foot_velocity_FL = stc.compute_swing_control(m,
                                                                           joints_pos_FL,
                                                                           joints_vel_FL,
                                                                           jac_foot_FL[0:3, 6:9],
                                                                           jac_foot_FL_dot[0:3, 6:9],
                                                                           lift_off_positions[0],
                                                                           desired_touch_down_FL,
                                                                           swing_time[0],
                                                                           position_foot_FL,
                                                                           velocity_foot_FL,
                                                                           h_FL,
                                                                           mass_matrix_FL)

            tau_FR, \
                desired_swing_foot_position_FR, \
                desired_swing_foot_velocity_FR = stc.compute_swing_control(m,
                                                                           joints_pos_FR,
                                                                           joints_vel_FR,
                                                                           jac_foot_FR[0:3, 9:12],
                                                                           jac_foot_FR_dot[0:3, 9:12],
                                                                           lift_off_positions[1],
                                                                           desired_touch_down_FR,
                                                                           swing_time[0],
                                                                           position_foot_FR,
                                                                           velocity_foot_FR,
                                                                           h_FR,
                                                                           mass_matrix_FL)

            tau_RL, \
                desired_swing_foot_position_RL, \
                desired_swing_foot_velocity_RL = stc.compute_swing_control(m,
                                                                           joints_pos_RL,
                                                                           joints_vel_RL,
                                                                           jac_foot_RL[0:3, 12:15],
                                                                           jac_foot_RL_dot[0:3, 12:15],
                                                                           lift_off_positions[2],
                                                                           desired_touch_down_RL,
                                                                           swing_time[0],
                                                                           position_foot_RL,
                                                                           velocity_foot_RL,
                                                                           h_RL,
                                                                           mass_matrix_FL)

            tau_RR, \
                desired_swing_foot_position_RR, \
                desired_swing_foot_velocity_RR = stc.compute_swing_control(m,
                                                                           joints_pos_RR,
                                                                           joints_vel_RR,
                                                                           jac_foot_RR[0:3, 15:18],
                                                                           jac_foot_RR_dot[0:3, 15:18],
                                                                           lift_off_positions[3],
                                                                           desired_touch_down_RR,
                                                                           swing_time[0],
                                                                           position_foot_RR,
                                                                           velocity_foot_RR,
                                                                           h_RR,
                                                                           mass_matrix_FL)

            print("joint pos: \n", joints_pos_FL)
            print("joint vel: \n", joints_vel_FL)
            print("mass matrix: \n", mass_matrix_FL)
            print("h: \n", h_FL)
            print("jac: \n", jac_foot_FL[0:3, 6:9])
            print("jac dot: \n", jac_foot_FL_dot[0:3, 6:9])
            print("lift off: \n", lift_off_positions[0])
            print("touch down: \n", desired_swing_foot_position_FL)
            print("swing time: \n", swing_time[0])
            print("foot pos: \n", position_foot_FL)
            print("foot vel: \n", velocity_foot_FL)

            nmpc_footholds = [desired_touch_down_FL, desired_swing_foot_position_FR,
                              desired_swing_foot_position_RL, desired_swing_foot_position_RR]

            # Set control and mujoco step ----------------------------------------------------------------------
            tau = np.concatenate((tau_FR, tau_FL, tau_RR, tau_RL))
            print("tau: \n", tau)
            d.ctrl = tau.reshape(m.nu, )
            mujoco.mj_step(m, d)

            # Plot
            if (i % 5 == 0):
                plot_swing_mujoco(mujoco, stc, swing_period, lift_off_positions, nmpc_footholds, viewer)

                desired_swing_foot_position_FL = desired_swing_foot_position_FL.reshape((3,))
                desired_swing_foot_velocity_FL = desired_swing_foot_velocity_FL.reshape((3,))
                swing_FL_evolution_new = np.array([position_foot_FL[0],
                                                   position_foot_FL[1],
                                                   position_foot_FL[2],
                                                   velocity_foot_FL[0],
                                                   velocity_foot_FL[1],
                                                   velocity_foot_FL[2]])
                swing_FL_evolution_new = swing_FL_evolution_new.reshape((6, 1))
                first, swing_FL_evolution = swing_FL_evolution[:, 0], swing_FL_evolution[:, 1:]
                swing_FL_evolution = np.concatenate((swing_FL_evolution, swing_FL_evolution_new), axis=1)

                swing_FL_evolution_reference_new = np.array([desired_swing_foot_position_FL[0],
                                                             desired_swing_foot_position_FL[1],
                                                             desired_swing_foot_position_FL[2],
                                                             desired_swing_foot_velocity_FL[0],
                                                             desired_swing_foot_velocity_FL[1],
                                                             desired_swing_foot_velocity_FL[2]])
                swing_FL_evolution_reference_new = swing_FL_evolution_reference_new.reshape((6, 1))
                first, swing_FL_evolution_reference = swing_FL_evolution_reference[:, 0], swing_FL_evolution_reference[
                                                                                          :, 1:]
                swing_FL_evolution_reference = np.concatenate(
                    (swing_FL_evolution_reference, swing_FL_evolution_reference_new), axis=1)

                fig.canvas.restore_region(bg)
                ln_x.set_ydata(swing_FL_evolution[0][:])
                ln_y.set_ydata(swing_FL_evolution[1][:])
                ln_z.set_ydata(swing_FL_evolution[2][:])
                ln_x_dot.set_ydata(swing_FL_evolution[3][:])
                ln_y_dot.set_ydata(swing_FL_evolution[4][:])
                ln_z_dot.set_ydata(swing_FL_evolution_reference[5][:])

                ax_x.draw_artist(ln_x)
                ax_y.draw_artist(ln_y)
                ax_z.draw_artist(ln_z)
                ax_x_dot.draw_artist(ln_x_dot)
                ax_y_dot.draw_artist(ln_y_dot)
                ax_z_dot.draw_artist(ln_z_dot)

                fig.canvas.blit(fig.bbox)
                fig.canvas.flush_events()

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
            # ---------------------------------------------------------------------------------------------------

            i = i + 1

            print("############")
