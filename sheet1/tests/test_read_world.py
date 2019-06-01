#!/usr/bin/python3

import pytest
from robot_mapping.tools.read_world import read_world


def test_read_world():
    landmarks = read_world('../robot_mapping/data/world.dat')
    assert(landmarks[0].x == 2)
    assert(landmarks[0].y == 1)


def test_read_world_size():
    landmarks = read_world('../robot_mapping/data/world.dat')
    assert(len(landmarks) == 9)
