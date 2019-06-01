#!/usr/bin/python3

import math
from typing import List
from tools.normalize_angle import normalize_angle
from tools.dataobjects import Odometry


def motion_command(xp: List[float], u: Odometry) -> List[float]:
    """ Updates the robot pose according to the motion model. """
    x = [0.0] * 3
    # normalize theta
    xp[2] = normalize_angle(xp[2])

    # update based on motion model
    x[0] = xp[0] + u.t * math.cos(xp[2] + u.r1)
    x[1] = xp[1] + u.t * math.sin(xp[2] + u.r1)
    x[2] = xp[2] + u.r1 + u.r2
    return x
