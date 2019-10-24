
import math
import numpy as np
from typing import Tuple

from tools.normalize_angle import normalize_angle
from tools.dataobjects import Odometry


ndarray = np.ndarray

def prediction_step(mu: ndarray, sigma: ndarray, u: Odometry, motion_noise=0.1) -> Tuple[ndarray, ndarray]:
    """ Updates the belief concerning the robot pose according to the motion model,

    Params:
        mu: 2N+3 x 1 vector representing the state mean
        sigma: 2N+3 x 2N+3 covariance matrix
        u: odometry reading (r1, t, r2)
    """

    N = int((mu.shape[0]-3)/2)
    tcos = u.t * math.cos(mu[2] + u.r1)
    tsin = u.t * math.sin(mu[2] + u.r1)

    # Update state with motion model
    mu[0] = mu[0] + tcos
    mu[1] = mu[1] + tsin
    mu[2] = mu[2] + u.r1 + u.r2

    # Normalize angle after noise-free motion model update.
    mu[2] = normalize_angle(mu[2])

    mu_bar = mu

    # Compute the 3x3 Jacobian Gx of the motion model
    G_low = np.eye(3) + np.array([
                    [0., 0., -tsin],
                    [0., 0., tcos],
                    [0., 0., 0.]])

    # Construct the full Jacobian
    Gt = np.block([
        [G_low, np.zeros((3,2*N))],
        [np.zeros((2*N,3)), np.eye(2*N)]
    ])

    # Make sure shape of Gt is (2N+3, 2N+3)
    assert(Gt.shape[0]==(2*N+3))
    assert(Gt.shape[1]==(2*N+3))

    # Construct the process noise
    R = np.zeros(sigma.shape)
    Rx = np.array([
        [motion_noise, 0., 0.],
        [0., motion_noise, 0.],
        [0.,0., motion_noise/10.]
    ])
    R[0:3,0:3] = Rx


    sigma_bar = np.dot(np.dot(Gt, sigma), Gt.T) + R

    return mu_bar, sigma_bar
