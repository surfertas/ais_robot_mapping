#!/usr/bin/python3

# This script runs the main loop and calls all the required
# functions in the correct order.
#
# You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should however not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions you
# should read their documentation which tells you the expected dimensions.
from motion_command import motion_command
from tools.plot_state import plot_state
from tools.read_data import read_data
from tools.read_world import read_world

if __name__ == "__main__":
    # Read world data, i.e. landmarks
    landmarks = read_world('./data/world.dat')

    # Read sensor readings, i.e. odometry and range-bearing sensor
    data = read_data('./data/sensor_data.dat')

    # Initialize belief
    # x is the robot pose [x,y,theta]
    x = [0.0] * 3

    # Iterate over odometry commands and update the robot pose
    # according to the motion model
    for t, sample in enumerate(data.timestep):
        odom, sensor = sample

        # Update the poes of the robot based on the motion model
        x = motion_command(x, odom)

        # Generate visualization plots of the current state#
        plot_state(x, landmarks, t, sensor)

        print("Current robot pose:")
        print(f'x = {x}')

    # Display the final state estimate
    print("Final robot pose:")
    print(f'x={x}')
