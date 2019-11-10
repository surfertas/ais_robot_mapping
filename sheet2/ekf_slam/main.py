
import numpy as np

from tools.plot_state import plot_state
from tools.read_data import read_data
from tools.read_world import read_world
from prediction_step import prediction_step
from correction_step import correction_step


if __name__=="__main__":

    # Read world data, i.e. landmarks
    landmarks = read_world('./data/world.dat')
    # Read sensor readings, i.e. odometry and range-bearing sensor
    data = read_data('./data/sensor_data.dat')

    n_landmarks = len(landmarks)
    # observed_landmarks is a vector that keeps track of which landmarks have been observed so far.
    # observed_landmarks[i] will be true if the landmark with id = i has been observed at some point by the robot
    observed_landmarks = [False]*n_landmarks

    # Initialize belief:
    # mu: 2N+3x1 vector representing the mean of the normal distribution
    # The first 3 components of mu correspond to the pose of the robot,
    # and the landmark poses (xi, yi) are stacked in ascending id order.
    # sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
    mu = np.zeros(2*n_landmarks+3)
    rob_sigma = np.zeros((3,3))
    rob_map_sigma = np.zeros((3,2*n_landmarks))
    map_sigma = 1e10*np.eye(2*n_landmarks)

    # Construct a (3+3*n_landmarks, 3+3*n_landmarks) matrix
    sigma = np.block([
                [rob_sigma, rob_map_sigma],
                [rob_map_sigma.T, map_sigma]])

    for t, sample in enumerate(data.timestep):
            odom, sensor = sample

            mu_bar, sigma_bar = prediction_step(mu, sigma, odom)

            mu, sigma, observed_landmarks = correction_step(mu_bar, sigma_bar, sensor, observed_landmarks)

            plot_state(mu, sigma, landmarks, t, observed_landmarks, sensor)
            #print(observed_landmarks)
