from dataclasses import dataclass
from typing import Any


@dataclass
class LegsAttr:
    """
    Dataclass to store attributes associated with the legs of a quadruped robot.

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
    """
    FR: Any
    FL: Any
    RR: Any
    RL: Any

    order = ['FL', 'FR', 'RL', 'RR']

    def to_list(self, order=None):
        order = order if order is not None else self.order
        return [getattr(self, leg) for leg in order]

    def __getitem__(self, key):
        assert key in self.order, f"Key {key} is not a valid leg label. Expected any of {self.order}"
        return getattr(self, key)

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def __iter__(self):
        return iter(self.to_list())

    def __str__(self):
        return f"{', '.join([f'{leg}={getattr(self, leg)}' for leg in self.order])}"

    def __repr__(self):
        return self.__str__()