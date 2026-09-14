import numpy as np


class SwingTrajectoryController:
    def __init__(
        self,
        step_height: float,
        swing_period: float,
        position_gain_fb: np.ndarray,
        velocity_gain_fb: np.ndarray,
        generator: str,
    ) -> None:
        self.generator = generator

        if self.generator == "scipy":
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)

        self.position_gain_fb = position_gain_fb
        self.velocity_gain_fb = velocity_gain_fb
        self.swing_period = swing_period
        self.swing_time = [0, 0, 0, 0]

        self.use_feedback_linearization = True
        self.use_friction_compensation = True

        self.rising_edge_detected = False

    def regenerate_swing_trajectory_generator(self, step_height: float, swing_period: float) -> None:
        if self.generator == "scipy":
            from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        
        else:
            from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
            self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
        
        self.swing_period = swing_period

    def compute_swing_control_cartesian_space(
        self, leg_id, q_dot, J, J_dot, lift_off, touch_down, foot_pos, foot_vel, passive_force, h, mass_matrix, early_stance_hitmoments, early_stance_hitpoints
    ):
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
            self.swing_time[leg_id], lift_off, touch_down, early_stance_hitmoments, early_stance_hitpoints
        )

        err_pos = des_foot_pos - foot_pos
        err_pos = err_pos.reshape((3,))

        err_vel = des_foot_vel - foot_vel
        err_vel = err_vel.reshape((3,))

        accelleration = des_foot_acc + self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel)

        accelleration = accelleration.reshape((3,))

        # Compute inertia matrix in task space.
        # Mass Matrix and centrifugal missing
        tau_swing = J.T @ (self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel))
        if self.use_feedback_linearization:
            tau_swing += mass_matrix @ np.linalg.pinv(J) @ (accelleration - J_dot @ q_dot) + h
    

        return tau_swing, des_foot_pos, des_foot_vel

    def compute_swing_control_joint_space(
        self, nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc, qpos, qvel, legs_mass_matrix, legs_qfrc_bias, legs_qfrc_passive
    ):
        error_position = nmpc_joints_pos - qpos
        error_position = error_position.reshape((3,))

        error_velocity = nmpc_joints_vel - qvel
        error_velocity = error_velocity.reshape((3,))

        accelleration = nmpc_joints_acc
        accelleration = accelleration.reshape((3,))
        

        tau_swing = self.position_gain_fb * error_position + self.velocity_gain_fb * error_velocity
        # Feedback linearization
        if self.use_feedback_linearization:
            tau_swing += (
                legs_mass_matrix
                @ (accelleration + self.position_gain_fb * error_position + self.velocity_gain_fb * error_velocity)
                + legs_qfrc_bias
            )
        
        return tau_swing, None, None

    def update_swing_time(self, current_contact, legs_order, dt):
        for leg_id, leg_name in enumerate(legs_order):
            # Swing time reset
            if current_contact[leg_id] == 0:
                if self.swing_time[leg_id] < self.swing_period:
                    self.swing_time[leg_id] = self.swing_time[leg_id] + dt
            else:
                self.swing_time[leg_id] = 0

    def check_apex_condition(self, current_contact, interval=0.02):
        apex = 0
        for leg_id in range(4):
            # Swing time check
            if current_contact[leg_id] == 0:
                if (self.swing_time[leg_id] > (self.swing_period / 2.0) - interval) and (
                    self.swing_time[leg_id] < (self.swing_period / 2.0) + interval
                ):
                    apex = 1
        return apex

    def check_full_stance_condition(self, current_contact):
        stance = 1
        # If one leg is not in stance, the robot is not in full stance
        for leg_id in range(4):
            if current_contact[leg_id] == 0:
                stance = 0
        return stance

    def check_touch_down_condition(self, current_contact, previous_contact, contact_sequence, lookahead=3):
        """
        Detect when all feet have just made contact with the ground (rising edge).
        Uses a lookahead mechanism to ensure stability and avoid transient states.
        """
        # Rising edge detection (transition from at least one foot in swing to all feet in stance)
        if np.all(current_contact == 1) and not np.all(previous_contact == 1):
            self.rising_edge_detected = True

        # Wait until first "n lookahead" columns in contact sequence are all in stance contact
        stable_stance = np.all(contact_sequence[:, 0:lookahead] == 1)
        next_leg_lift = not np.all(contact_sequence[:, lookahead] == 1)

        if self.rising_edge_detected and stable_stance and next_leg_lift:
            self.rising_edge_detected = False
            return 1 # Signal to trigger optimization
        else:
            return 0



if __name__ == "__main__":
    import time
    from pathlib import Path

    import gym_quadruped
    import mujoco
    import mujoco.viewer
    from gym_quadruped.robot_cfgs import get_robot_config

    # Package import also supports launching this file directly after installation.
    from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController

    # Change these settings to explore the two generators and the swing geometry.
    robot, leg = "go2", "FL"
    generator = "explicit"  # "explicit" (Bezier) or "scipy" (cubic spline)
    # Duration [s], vertical control-point height [m], forward displacement [m].
    swing_period, step_height, step_length = 1.0, 0.08, 0.08
    dt = 0.002
    # Integrate dynamics at 500 Hz, but refresh the viewer approximately at 60 Hz.
    steps_per_frame = round(1.0 / (60 * dt))

    # Remove the floating joint: the base is physically fixed in the air,
    # while the leg joints remain dynamic and are driven by controller torques.
    robot_cfg = get_robot_config(robot)
    model_path = Path(gym_quadruped.__file__).parent / "robot_model" / robot_cfg.mjcf_filename
    spec = mujoco.MjSpec.from_file(str(model_path))
    # The original pose contains base xyz, a quaternion, then the joint angles.
    home = np.array(list(spec.keys)[0].qpos)
    free_joint = next(j for j in spec.joints if j.type == mujoco.mjtJoint.mjJNT_FREE)
    free_joint.parent.pos = [0.0, 0.0, 0.8]
    free_joint.parent.quat = home[3:7]
    spec.delete(free_joint)
    # Floating-base keyframes are no longer compatible with the fixed model.
    for key in list(spec.keys):
        spec.delete(key)
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1],
                            rgba=[0.3, 0.3, 0.3, 1])
    model = spec.compile()
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    # With the free joint removed, qpos and qvel contain only the 12 leg joints.
    data.qpos[:] = home[7:]
    # Refresh foot positions and dynamics after assigning the initial pose.
    mujoco.mj_forward(model, data)
    home_joints = data.qpos.copy()

    # Select the three velocity coordinates belonging to the controlled leg.
    # These indices select its Jacobian columns and its inertia matrix block.
    joint_ids = [model.joint(name).id for name in robot_cfg.leg_joints[leg]]
    dofs = model.jnt_dofadr[joint_ids]
    foot_id = model.geom(robot_cfg.feet_geom_names[leg]).id
    foot_origin = data.geom_xpos[foot_id].copy()
    # This reference frame is only translated: its axes remain world-aligned.
    # Therefore velocities and Jacobians need no rotation or translation correction.
    lift_off, touch_down = np.zeros(3), np.array([step_length, 0.0, 0.0])
    # Translate into coordinates relative to the initial foot: the explicit
    # generator uses step_height as an absolute z control-point coordinate.
    controller = SwingTrajectoryController(
        step_height, swing_period, np.full(3, 400.0), np.full(3, 40.0), generator
    )
    # Sample the same generator used by the controller to display its spatial curve.
    # Position, velocity and acceleration references are recomputed during control.
    curve = np.array([
        controller.swing_generator.compute_trajectory_references(t, lift_off, touch_down)[0].reshape(3)
        + foot_origin for t in np.linspace(0.0, swing_period, 80)
    ])
    jacobian = np.zeros((3, model.nv))
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_jacGeom(model, data, jacobian, None, foot_id)
    # Initialize with the actual Jacobian to avoid a spurious derivative at startup.
    previous_jacobian = jacobian[:, dofs].copy()
    # Map motor torques by joint ID instead of assuming a particular leg order.
    # The Go2 model uses direct joint torque motors with unit transmission ratios.
    actuator_dofs = model.jnt_dofadr[model.actuator_trnid[:, 0]]

    print("Close the window to stop. Green: swing curve; red: desired foot.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        viewer.cam.distance = 1.8
        viewer.cam.azimuth, viewer.cam.elevation = 135, -20
        while viewer.is_running():
            frame_start = time.perf_counter()
            for _ in range(steps_per_frame):
                mujoco.mj_forward(model, data)
                # Alternate forward and backward swings with matching endpoints.
                cycle = int(data.time / swing_period)
                # Slot 0 is the demo's single swing clock; dofs selects the actual leg.
                controller.swing_time[0] = data.time % swing_period
                start, end = (lift_off, touch_down) if cycle % 2 == 0 else (touch_down, lift_off)
                # Foot velocity is J*q_dot. Its acceleration is J*q_ddot + J_dot*q_dot.
                # Estimate J_dot by finite differences between consecutive physics steps.
                mujoco.mj_jacGeom(model, data, jacobian, None, foot_id)
                J = jacobian[:, dofs].copy()
                J_dot = (J - previous_jacobian) / dt
                previous_jacobian = J.copy()
                # Expand MuJoCo's inertia storage into a dense joint-space matrix.
                # qfrc_bias contains gravity, Coriolis and centrifugal terms.
                mujoco.mj_fullM(model, data, mass)
                # The controller combines the generator's feedforward acceleration
                # with position/velocity feedback, then computes the three joint torques.
                # No early contact is expected because the robot is suspended.
                swing_torque, desired_position, _ = controller.compute_swing_control_cartesian_space(
                    leg_id=0, q_dot=data.qvel[dofs], J=J, J_dot=J_dot,
                    lift_off=start, touch_down=end,
                    foot_pos=data.geom_xpos[foot_id] - foot_origin,
                    foot_vel=jacobian @ data.qvel, passive_force=data.qfrc_passive[dofs],
                    h=data.qfrc_bias[dofs], mass_matrix=mass[np.ix_(dofs, dofs)],
                    early_stance_hitmoments=-1, early_stance_hitpoints=None,
                )
                # Hold the other legs at home with joint PD and gravity compensation.
                torque = 40.0 * (home_joints - data.qpos) - 4.0 * data.qvel + data.qfrc_bias
                # Replace only the selected leg's holding torques with swing control.
                torque[dofs] = swing_torque
                data.ctrl[:] = torque[actuator_dofs]
                # Integrate the commanded dynamics for dt seconds.
                mujoco.mj_step(model, data)

            # Visualize the generator curve and its current reference in world coordinates.
            with viewer.lock():
                # These spheres are visual overlays and do not create contacts or forces.
                viewer.user_scn.ngeom = 0
                points = [*curve, desired_position.reshape(3) + foot_origin]
                for i, point in enumerate(points):
                    radius = 0.003 if i < len(curve) else 0.012
                    color = [0, 1, 0, 0.6] if i < len(curve) else [1, 0, 0, 1]
                    mujoco.mjv_initGeom(viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                       [radius] * 3, point, np.eye(3).ravel(), color)
                    viewer.user_scn.ngeom += 1
            viewer.sync()
            # Pace the demo in real time; the physics time step remains fixed at dt.
            time.sleep(max(0.0, steps_per_frame * dt - (time.perf_counter() - frame_start)))
