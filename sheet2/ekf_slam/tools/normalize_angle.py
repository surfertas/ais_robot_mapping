#!/usr/bin/python3

import numpy as np


def normalize_angle(phi: float) -> float:
    """ Normalize phi to be between -pi and pi """
    while phi > np.pi:
        phi = phi - 2 * np.pi

    while phi < -np.pi:
        phi = phi + 2 * np.pi

    return phi
