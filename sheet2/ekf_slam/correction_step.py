
import math
import numpy as np
from numpy.linalg import inv

from typing import List
from tools.dataobjects import Sensor
from tools.normalize_angle import normalize_angle


ndarray = np.ndarray

def correction_step(mu: ndarray, sigma: ndarray, z: List[Sensor] , observed_landmarks: List[bool]):
    """ Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model

    # The employed sensor model measures the range and bearing of a landmark

    Params:
        mu: 2N+3 x 1 vector representing the state mean. The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
            The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
        sigma: 2N+3 x 2N+3 is the covariance matrix
        z: struct array containing the landmark observations. Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
        observed_landmarks: The list indicates which landmarks have been observed at some point by the robot.
                            observed_landmarks[j] is false if the landmark with id = j has never been observed before.

    """
    # Number of measurements in this time step.
    m = len(z)
    # Total number of landmarks
    N = len(observed_landmarks)
    # Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
    # expected_Z: vectorized form of all expected measurements in the same form.

    Z = []
    expected_Z = []
    # H will be 2m x 3+2N dimensions.
    H = np.array([])

    for i in range(m):
        # Need to subtract by 1 as python index starts from 0 (as opposed to 1 in matlab)
        lid = z[i].sid - 1
        (xi, yi) = 2*lid+3, 2*lid+4

        if not observed_landmarks[lid]:
            # Initialize its pose (x,y) in mu based on the measurement and the current robot pose (x,y, theta)
            mu[xi] = mu[0] + (z[i].srange*math.cos(z[i].bearing+mu[2]))
            mu[yi] = mu[1] + (z[i].srange*math.sin(z[i].bearing+mu[2]))
            observed_landmarks[lid] = True

        # Add the landmark measurement to the Z
        Z.append(z[i].srange)
        Z.append(z[i].bearing)
        # Use the current estimate of the landmark pose to compute the corresponding expected measuremen
        # pg.35 http://ais.informatik.uni-freiburg.de/teaching/ws17/mapping/pdf/slam05-ekf-slam.pdf
        dx = mu[xi] - mu[0]
        dy = mu[yi] - mu[1]
        q = dx*dx + dy*dy

        # TODO: do we need to normalize the angle???
        expected_Z.append(np.sqrt(q))
        expected_Z.append(normalize_angle(math.atan2(dy,dx)-mu[2]))

        # Compute the Jacobian Hi of the measurement function h for this observation.
        sqq = np.sqrt(q)

        # Dimension 2x5
        H_low = (1./q) * np.array([
                            [-sqq*dx, -sqq*dy, 0., sqq*dx, sqq*dy],
                            [dy,-dx, -1.,-dy, dx]])

        # Map Jacobian to high dimensional space pg. 40.
        Fxj = np.zeros((5, 3+2*N))
        robot_pose = np.eye(3)
        landmark_pose = np.eye(2)
        Fxj[:3,:3] = robot_pose
        Fxj[3:, 3+2*lid: 3+2*lid+2] = landmark_pose
        # Hi Dimension  2x3+2*N
        Hi = np.dot(H_low, Fxj)
        # Stack
        H = Hi if len(H)==0 else np.block([[H],[Hi]])

        # Construct the sensor noise matrix Q.
        Q   = np.eye(2*(i+1))*0.01

        # Compute the Kalman gain with dimensions (3+2N, 2m)
        # Note: l = i+1
        # K =  S * H.T * ((H*S*H.T)+Q)^-1
        # (3+2N, 2l) = (3+2N,3+2N)*(3+2N,2l)*INV((2l,3+2N)*(3+2N, 3+2N)*(3+2N,2l)+(2l,2l))
        K = np.dot(sigma, np.dot(H.T, inv(np.dot(H, np.dot(sigma, H.T)) + Q)))

        # Compute the difference between the expected and recorded measurements.
        innovation = np.array(Z)-np.array(expected_Z)

        # Normalize the bearings after subtracting!
        for i,obs in enumerate(innovation):
            if not i%2==0:
                innovation[i] = normalize_angle(innovation[i])
        # Update
        # (3+2N,2l)*(2l,1)
        mu += np.dot(K,innovation).flatten()
        sigma -= np.dot(np.dot(K,H), sigma)

    # Normalize theta in the robot pose.
    mu[2] = normalize_angle(mu[2])

    return mu, sigma, observed_landmarks
