#!/usr/bin/python3

import pytest
from robot_mapping.tools.read_data import read_data


def test_read_data():
    data = read_data('../robot_mapping/data/sensor_data.dat')
    sample = data.timestep[0]
    assert(sample[0].r1 == 0.100692392654)
    assert(sample[0].t == 0.100072845247)
    assert(sample[0].r2 == 0.000171392857486)


def test_read_data_size():
    data = read_data('../robot_mapping/data/sensor_data.dat')
    assert(len(data) == 330)
