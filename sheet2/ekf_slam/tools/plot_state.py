#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from typing import List
from tools.dataobjects import Sensor, Landmark

ndarray = np.ndarray

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
        #print(id, mX,mY)
        plt.plot([mu[0], mX], [mu[1], mY], 'ro-')

    fig.savefig(f'./plots/odom_{timestep}.png', dpi=fig.dpi)
