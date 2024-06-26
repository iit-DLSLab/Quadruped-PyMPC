from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import mujoco
import numpy as np


@dataclass
class LegsAttr:
    """Dataclass to store attributes associated with the legs of a quadruped robot.

    This class is useful to deal with different leg's ordering and naming conventions, given by different robots,
    vendors, labs. This should allow you to be flexible enough so that code made with a given convention can be
    easily used with another one. :)

    Attributes
    ----------
    FR : Any object/scalar/vector/tensor/feature associated with the Front Right leg
    FL : Any object/scalar/vector/tensor/feature associated with the Front Left  leg
    RR : Any object/scalar/vector/tensor/feature associated with the Rear  Right leg
    RL : Any object/scalar/vector/tensor/feature associated with the Rear  Left  leg

    Examples
    --------
    >>> feet_pos = LegsAttr(FR=[1, 3, 5], FL=[2, 4, 6], RR=[7, 9, 11], RL=[8, 10, 12])
    >>> feet_pos["FR"] = [0.1, 0.1, 0.2]  # Set the value of the FR attribute
    >>> feet_pos.RR = [0.3, 0.1, 0.2]     # Set the value of the RR attribute
    >>> b = feet_pos["FR"]  # feet_pos.FR Get the value of the FR attribute
    >>> # Get the (4, 3) numpy array of the feet positions in the order FR, FL, RR, RL
    >>> import numpy as np
    >>> a = np.array([feet_pos.to_list(order=['FR', 'FL', 'RR', 'RL'])])
    >>> # Basic arithmetic operations are supported
    >>> c: LegsAttr = feet_pos + feet_pos
    >>> assert c.FR == feet_pos.FR + feet_pos.FR
    >>> d: LegsAttr = feet_pos - feet_pos
    >>> assert d.FL == feet_pos.FL - feet_pos.FL
    >>> e: LegsAttr = feet_pos / 2
    >>> assert e.RR == (feet_pos.RR / 2)
    """

    FR: Any
    FL: Any
    RR: Any
    RL: Any

    order = ['FL', 'FR', 'RL', 'RR']

    def to_list(self, order=None):
        """Return a list of the leg's attributes in the order specified (or self.order if order=None)."""
        order = order if order is not None else self.order
        return [getattr(self, leg) for leg in order]

    def __getitem__(self, key):
        """Get the value of the attribute associated with the leg key."""
        assert key in self.order, f"Key {key} is not a valid leg label. Expected any of {self.order}"
        return getattr(self, key)

    def __setitem__(self, key, value):
        """Set the value of the attribute associated with the leg key."""
        setattr(self, key, value)

    def __iter__(self):
        """Iterate over the legs attributes in the order self.order."""
        return iter(self.to_list())

    def __add__(self, other):
        """Add the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, LegsAttr):
            return LegsAttr(FR=self.FR + other.FR, FL=self.FL + other.FL, RR=self.RR + other.RR, RL=self.RL + other.RL)
        elif isinstance(other, type(self.FR)):
            return LegsAttr(FR=self.FR + other, FL=self.FL + other, RR=self.RR + other, RL=self.RL + other)
        else:
            raise TypeError("Unsupported operand type for +: 'LegsAttr' and '{}'".format(type(other)))

    def __sub__(self, other):
        """Subtract the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, LegsAttr):
            return LegsAttr(FR=self.FR - other.FR, FL=self.FL - other.FL, RR=self.RR - other.RR, RL=self.RL - other.RL)
        elif isinstance(other, type(self.FR)):
            return LegsAttr(FR=self.FR - other, FL=self.FL - other, RR=self.RR - other, RL=self.RL - other)
        else:
            raise TypeError("Unsupported operand type for -: 'LegsAttr' and '{}'".format(type(other)))

    def __truediv__(self, other):
        """Divide the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, type(self.FR)) or isinstance(other, (int, float)):
            return LegsAttr(FR=self.FR / other, FL=self.FL / other, RR=self.RR / other, RL=self.RL / other)
        else:
            raise TypeError("Unsupported operand type for /: 'LegsAttr' and '{}'".format(type(other)))

    def __matmul__(self, other):
        """Matrix multiplication of the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, LegsAttr):
            return LegsAttr(FR=self.FR @ other.FR, FL=self.FL @ other.FL, RR=self.RR @ other.RR, RL=self.RL @ other.RL)
        elif isinstance(other, type(self.FR)):
            return LegsAttr(FR=self.FR @ other, FL=self.FL @ other, RR=self.RR @ other, RL=self.RL @ other)
        else:
            raise TypeError("Unsupported operand type for @: 'LegsAttr' and '{}'".format(type(other)))

    def __str__(self):
        """Return a string representation of the legs attributes."""
        return f"{', '.join([f'{leg}={getattr(self, leg)}' for leg in self.order])}"

    def __repr__(self):
        """Return a string representation of the legs attributes."""
        return self.__str__()


@dataclass
class JointInfo:
    """Dataclass to store information about the joints of a robot.

    Attributes
    ----------
    name : (str) The name of the joint.
    type : (int) The type of the joint.
    body_id : (int) The body id of the joint.
    nq : (int) The number of generalized coordinates.
    nv : (int) The number of generalized velocities.
    qpos_idx : (tuple) The indices of the joint's generalized coordinates.
    qvel_idx : (tuple) The indices of the joint's generalized velocities.
    tau_idx: (tuple) The indices of the joint's in the generalized forces vector.
    range : list(min, max) The range of the joint's generalized coordinates.
    """

    name: str
    type: int
    body_id: int
    nq: int
    nv: int
    qpos_idx: tuple
    qvel_idx: tuple
    range: list
    tau_idx: tuple = field(default_factory=tuple)
    actuator_id: int = field(default=-1)

    def __str__(self):
        """Return a string representation of the joint information."""
        return f"{', '.join([f'{key}={getattr(self, key)}' for key in self.__dict__.keys()])}"


class GaitType(Enum):
    """Enumeration class to represent the different gaits that a quadruped robot can perform."""

    TROT = 0
    PACE = 1
    BOUNDING = 2
    CIRCULARCRAWL = 3
    BFDIAGONALCRAWL = 4
    BACKDIAGONALCRAWL = 5
    FRONTDIAGONALCRAWL = 6
    FULL_STANCE = 7


def extract_mj_joint_info(model: mujoco.MjModel) -> OrderedDict[str, JointInfo]:
    """Returns the joint-space information of the model.

    Thanks to the obscure Mujoco API, this function tries to do the horrible hacks to get the joint information
    we need to do a minimum robotics project with a rigid body system.

    Returns
    -------
        A dictionary with the joint names as keys and the JointInfo namedtuple as values.
            each JointInfo namedtuple contains the following fields:
            - name: The joint name.
            - type: The joint type (mujoco.mjtJoint).
            - body_id: The body id to which the joint is attached.
            - range: The joint range.
            - nq: The number of joint position variables.
            - nv: The number of joint velocity variables.
            - qpos_idx: The indices of the joint position variables in the qpos array.
            - qvel_idx: The indices of the joint velocity variables in the qvel array.
    """
    joint_info = OrderedDict()
    for joint_id in range(model.njnt):
        # Get the starting index of the joint name in the model.names string
        name_start_index = model.name_jntadr[joint_id]
        # Extract the joint name from the model.names bytes and decode it
        joint_name = model.names[name_start_index:].split(b'\x00', 1)[0].decode('utf-8')
        joint_type = model.jnt_type[joint_id]
        qpos_idx_start = model.jnt_qposadr[joint_id]
        qvel_idx_start = model.jnt_dofadr[joint_id]

        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            joint_nq, joint_nv = 7, 6
        elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
            joint_nq, joint_nv = 4, 3
        elif joint_type == mujoco.mjtJoint.mjJNT_SLIDE:
            joint_nq, joint_nv = 1, 1
        elif joint_type == mujoco.mjtJoint.mjJNT_HINGE:
            joint_nq, joint_nv = 1, 1
        else:
            raise RuntimeError(f"Unknown mujoco joint type: {joint_type} available {mujoco.mjtJoint}")

        qpos_idx = np.arange(qpos_idx_start, qpos_idx_start + joint_nq)
        qvel_idx = np.arange(qvel_idx_start, qvel_idx_start + joint_nv)

        joint_info[joint_name] = JointInfo(
            name=joint_name,
            type=joint_type,
            body_id=model.jnt_bodyid[joint_id],
            range=model.jnt_range[joint_id],
            nq=joint_nq,
            nv=joint_nv,
            qpos_idx=qpos_idx,
            qvel_idx=qvel_idx)

    # Iterate over all actuators
    current_dim = 0
    for acutator_idx in range(model.nu):
        name_start_index = model.name_actuatoradr[acutator_idx]
        act_name = model.names[name_start_index:].split(b'\x00', 1)[0].decode('utf-8')
        mj_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
        # Get the joint index associated with the actuator
        joint_id = model.actuator_trnid[mj_actuator_id, 0]
        # Get the joint name from the joint index
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)

        # Add the actuator indx to the joint_info
        joint_info[joint_name].actuator_id = mj_actuator_id
        joint_info[joint_name].tau_idx = tuple(range(current_dim, current_dim + joint_info[joint_name].nv))
        current_dim += joint_info[joint_name].nv
    return joint_info
