#!/usr/bin/python3

import numpy as np
from numpy.linalg import inv
from typing import List
from tools.normalize_angle import normalize_angle

def v2t(pose: List[float]) -> np.ndarray:
    """ Takes as input the vector form of the robot pose and outputs
    the corresponding homogeneous transformation.
    """
    x,y,theta = pose
    theta = normalize_angle(theta)

    M = np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0.,0.,1.]
        ])
    return M

def t2v(M: np.ndarray) -> List[float]:
    """ Takes as input a homogeneous transformation representing the
    robot pose in 2-D space and outputs the corresponding compact vector.
    """
    theta = np.arccos(M[0,0])
    pose = [M[0,2],M[1,2],theta]
    return pose
