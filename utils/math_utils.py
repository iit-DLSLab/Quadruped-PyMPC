import numpy as np


def skew(x):
    """
    Skew symmetric matrix from a 3D vector
    """
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])