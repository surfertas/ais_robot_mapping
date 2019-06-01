#!/usr/bin/python3

import pytest
import numpy as np
from robot_mapping.homogenous_transformation import v2t, t2v

def test_v2t():
    pose = [1.,2.,0.523599]
    M = v2t(pose)
    assert(M.shape[0]==3)
    assert(M.shape[1]==3)
    assert(M[0,0]==0.8660252915835662)
    assert(M[1,1]==0.8660252915835662)

def test_t2v():
    ht = np.array([
        [ 0.86602529, -0.50000019,  1.],
        [ 0.50000019,  0.86602529,  2.],
        [ 0.,          0.,          1.]])

    pose = t2v(ht)
    x,y,theta = pose
    assert(x == 1.)
    assert(y == 2.)
    assert(theta == 0.5235990031671314)