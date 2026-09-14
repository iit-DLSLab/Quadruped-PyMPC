import os

# Fail-safe: if not explicitly chosen otherwise,
# ROS 2 communicates only on the local machine.
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

print(
    "ROS 2 network mode:",
    "LOCALHOST" if os.environ["ROS_LOCALHOST_ONLY"] == "1" else "NETWORK",
)

import sys
import shlex
import subprocess
from pathlib import Path

dir_path = Path(__file__).resolve().parent
sys.path.append(str(dir_path / ".."))

ros_ws = dir_path / "msgs_ws"
setup_bash = ros_ws / "install" / "setup.bash"

if not setup_bash.exists():
    print("Building the msgs first...")
    subprocess.run(["colcon", "build"], cwd=ros_ws, check=True)

if os.environ.get("QUADRUPED_PYMPC_ROS2_SOURCED") != "1":
    print("Sourcing ROS2 workspace and restarting script...")
    cmd = (
        f"source {shlex.quote(str(setup_bash))} && "
        "export QUADRUPED_PYMPC_ROS2_SOURCED=1 && "
        f"exec {shlex.quote(sys.executable)} "
        + " ".join(shlex.quote(arg) for arg in [str(Path(__file__).resolve()), *sys.argv[1:]])
    )
    os.execv("/bin/bash", ["bash", "-c", cmd])

import rclpy 
from rclpy.node import Node 
from dls2_interface.msg import BaseState, BaseStateDebug, BlindState, ControlSignal, TimeDebug
from unitree_go.msg import LowState

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import mujoco

# Gym and Simulation related imports
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.quadruped_utils import LegsAttr


# Config imports
from quadruped_pympc import config as cfg


USE_SCHEDULER = True # Use the scheduler to compute the control signal
SCHEDULER_FREQ = 500 # Frequency of the scheduler
RENDER_FREQ = 30

# Shell for the controllers ----------------------------------------------
class Simulator_Node(Node):
    def __init__(self):
        super().__init__('Simulator_Node')

        # Subscribers and Publishers
        self.publisher_base_state = self.create_publisher(BaseState,"/base_state", 1)
        self.publisher_base_state_debug = self.create_publisher(BaseStateDebug, "/base_state_debug", 1)
        self.publisher_blind_state = self.create_publisher(BlindState,"/blind_state_legged", 1)
        self.publisher_low_state = self.create_publisher(LowState,"/lowstate", 1)
        self.subscriber_control_signal = self.create_subscription(ControlSignal,"/control_signal_legged", self.get_control_signal_callback, 1)

        self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_simulator_step_callback)


        # Mujoco env
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=1.0/SCHEDULER_FREQ,
            base_vel_command_type="human"
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
        

        self.last_render_time = time.time()
        self.publish_count = 0
        self.env.render()  
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False

        # Torque vector
        self.desired_tau = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])

        # Desired PD 
        self.desired_joints_position = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
        self.desired_joints_velocity = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])

        self.foot_geom_ids = {
            leg: mujoco.mj_name2id(self.env.mjModel, mujoco.mjtObj.mjOBJ_GEOM, leg)
            for leg in ["FL", "FR", "RL", "RR"]
        }

        self.joint_names = [
            mujoco.mj_id2name(self.env.mjModel, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            for joint_id in range(1, self.env.mjModel.njnt)
        ]


    def get_control_signal_callback(self, msg):

        torques = np.array(msg.joints_torques)

        self.desired_tau.FL = torques[0:3]
        self.desired_tau.FR = torques[3:6]
        self.desired_tau.RL = torques[6:9]
        self.desired_tau.RR = torques[9:12]

        joints_position = np.array(msg.joints_position)

        self.desired_joints_position.FL = joints_position[0:3]
        self.desired_joints_position.FR = joints_position[3:6]
        self.desired_joints_position.RL = joints_position[6:9]
        self.desired_joints_position.RR = joints_position[9:12]



    def compute_simulator_step_callback(self):

        action = np.zeros(self.env.mjModel.nu)
        action[self.env.legs_tau_idx.FL] = self.desired_tau.FL.reshape(-1)
        action[self.env.legs_tau_idx.FR] = self.desired_tau.FR.reshape(-1)
        action[self.env.legs_tau_idx.RL] = self.desired_tau.RL.reshape(-1)
        action[self.env.legs_tau_idx.RR] = self.desired_tau.RR.reshape(-1)
        self.env.step(action=action)

        base_lin_vel = self.env.base_lin_vel(frame='world')
        base_ang_vel = self.env.base_ang_vel(frame='base')
        base_pos = self.env.base_pos
        base_lin_vel_body = self.env.base_lin_vel(frame="base")

        base_state_msg = BaseState()
        base_state_msg.pose.position = base_pos
        base_state_msg.pose.orientation = np.roll(self.env.mjData.qpos[3:7],-1)
        base_state_msg.velocity.linear = base_lin_vel
        base_state_msg.velocity.angular = base_ang_vel
        self.publisher_base_state.publish(base_state_msg)

        base_state_debug_msg = BaseStateDebug()
        base_state_debug_msg.frame_id = "world"
        base_state_debug_msg.sequence_id = self.publish_count
        base_state_debug_msg.timestamp = float(self.env.mjData.time)
        base_state_debug_msg.robot_name = cfg.robot
        base_state_debug_msg.linear_velocity_base = (
            np.asarray(base_lin_vel_body, dtype=np.float32).reshape(3).tolist()
        )
        self.publisher_base_state_debug.publish(base_state_debug_msg)
        self.publish_count += 1

        blind_state_msg = BlindState()
        blind_state_msg.joints_position = self.env.mjData.qpos[7:].tolist()
        blind_state_msg.joints_velocity = self.env.mjData.qvel[6:].tolist()
        self.publisher_blind_state.publish(blind_state_msg)


        # Compute feet contact forces ------------------------------------------------
        feet_GRF = {leg: np.zeros(3) for leg in ["FL", "FR", "RL", "RR"]}
        leg_from_geom_id = {geom_id: leg for leg, geom_id in self.foot_geom_ids.items()}
        for contact_id in range(self.env.mjData.ncon):
            contact = self.env.mjData.contact[contact_id]
            # Contact geometry is named FL/FR/RL/RR in the robot XML. Match
            # those geom ids directly instead of looking at their parent body.
            foot_geom_id = next(
                (geom_id for geom_id in (contact.geom1, contact.geom2) if geom_id in leg_from_geom_id),
                None,
            )
            if foot_geom_id is not None:
                force_contact = np.zeros(6)
                mujoco.mj_contactForce(self.env.mjModel, self.env.mjData, contact_id, force_contact)
                rotation_contact = contact.frame.reshape(3, 3)
                feet_GRF[leg_from_geom_id[foot_geom_id]] += rotation_contact.T @ force_contact[:3]


        # Publish Low State of Unitree ------------------------------------------------
        # FR, FL, RR, RL convention to follow the unitree standard msgs
        lowstate_msg = LowState()
        for i in range(3):
            lowstate_msg.motor_state[i].q = self.env.mjData.qpos[10+i]
            lowstate_msg.motor_state[i].dq = self.env.mjData.qvel[9+i]
        for i in range(3):
            lowstate_msg.motor_state[i+3].q = self.env.mjData.qpos[7+i]
            lowstate_msg.motor_state[i+3].dq = self.env.mjData.qvel[6+i]
        for i in range(3):
            lowstate_msg.motor_state[i+6].q = self.env.mjData.qpos[16+i]
            lowstate_msg.motor_state[i+6].dq = self.env.mjData.qvel[15+i]
        for i in range(3):
            lowstate_msg.motor_state[i+9].q = self.env.mjData.qpos[13+i]
            lowstate_msg.motor_state[i+9].dq = self.env.mjData.qvel[12+i]

        lowstate_msg.foot_force = np.array([abs(feet_GRF["FR"][2]), abs(feet_GRF["FL"][2]), abs(feet_GRF["RR"][2]), abs(feet_GRF["RL"][2])], dtype=np.int16)
        lowstate_msg.imu_state.accelerometer = self.env.mjData.sensordata[0:3].astype(np.float32)
        lowstate_msg.imu_state.gyroscope = self.env.mjData.sensordata[3:6].astype(np.float32)
        lowstate_msg.imu_state.quaternion = np.roll(np.array(self.env.mjData.sensordata[9:13].astype(np.float32)), -1)
        self.publisher_low_state.publish(lowstate_msg)

        # Render only at a certain frequency -----------------------------------------------------------------
        if time.time() - self.last_render_time > 1.0 / RENDER_FREQ:
            self.env.render()
            self.last_render_time = time.time()


def main():
    print('Hello from the gym_quadruped simulator.')
    rclpy.init()

    simulator_node = Simulator_Node()

    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
