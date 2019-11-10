#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2
from typing import List
from tools.dataobjects import Sensor, Landmark

ndarray = np.ndarray


def draw_ellipse(x, a, b, color):
    return


def draw_prob_ellipse(mu, cov, alpha, color):
    # TODO: Incomplete
    # Calculate unscaled half axes
    sxx = cov[0,0]
    syy = cov[1,1]
    sxy = cov[0,1]
    a = np.sqrt(0.5*(sxx+syy+np.sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy)))
    b = np.sqrt(0.5*(sxx+syy-np.sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy)))
    print(np.isreal(a), np.isreal(b))

    a = np.real(a) if not np.isreal(a) else a
    b = np.real(b) if not np.isreal(b) else b

    a = a*np.sqrt(chi2.ppf(alpha, 2))
    b = b*np.sqrt(chi2.ppf(alpha, 2))

    if sxx < syy:
        a,b = b,a

    angle = 0.0
    if sxx != syy:
        angle = 0.5*np.arctan(2*sxy/(sxx-syy))
    elif sxy > 0:
        angle = np.pi/4
    elif sxy < 0:
        angle = -np.pi/4

    mu[2] = angle
    draw_ellipse(mu, a,b,color)


def plot_state(mu: ndarray, sigma: ndarray,  landmarks: List[Landmark], timestep: int, observed_landmarks: List[bool], z: List[Sensor]):
    """
    Visualizes the robot in the map and the observations made at this timestep.

    Note:
        Run `ffmpeg -r 10 -start_number 0 -i 'odom_%d.png' -b 500000  odom.mp4`
        from the /plots directory to convert images to video.
    """
    (xs, ys) = zip(*[(l.x, l.y) for l in landmarks])
    fig = plt.figure()
    plt.scatter(xs, ys)

    for reading in z:
        # subtract by 1 as landmark data is index starting from 1
        id = reading.sid - 1
        mX = mu[3+2*id]
        mY = mu[3+2*id+1]
        plt.plot([mu[0], mX], [mu[1], mY], 'ro-')

    fig.savefig(f'./plots/odom_{timestep}.png', dpi=fig.dpi)
