from dataclasses import dataclass
from typing import Any


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
        else:
            raise TypeError("Unsupported operand type for +: 'LegsAttr' and '{}'".format(type(other)))

    def __sub__(self, other):
        """Subtract the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, LegsAttr):
            return LegsAttr(FR=self.FR - other.FR, FL=self.FL - other.FL, RR=self.RR - other.RR, RL=self.RL - other.RL)
        else:
            raise TypeError("Unsupported operand type for -: 'LegsAttr' and '{}'".format(type(other)))

    def __truediv__(self, other):
        """Divide the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, (int, float)):
            return LegsAttr(FR=self.FR / other, FL=self.FL / other, RR=self.RR / other, RL=self.RL / other)
        else:
            raise TypeError("Unsupported operand type for /: 'LegsAttr' and '{}'".format(type(other)))

    def __matmul__(self, other):
        """Matrix multiplication of the attributes of the legs with the attributes of the other LegsAttr object."""
        if isinstance(other, LegsAttr):
            return LegsAttr(FR=self.FR @ other.FR, FL=self.FL @ other.FL, RR=self.RR @ other.RR, RL=self.RL @ other.RL)
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

    def __str__(self):
        """Return a string representation of the joint information."""
        return f"{', '.join([f'{key}={getattr(self, key)}' for key in self.__dict__.keys()])}"
