#!/usr/bin/python3

import pytest
from robot_mapping.motion_command import motion_command
from robot_mapping.tools.dataobjects import Odometry


def test_motion_command_zero():
    xp = [0.0, 0.0, 0.0]
    u = Odometry(r1=0.100692392654, t=0.100072845247, r2=0.000171392857486)
    x = motion_command(xp, u)
    assert(x[0] == 0.09956595655676025)
    assert(x[1] == 0.010059555197290845)
    assert(x[2] == 0.100863785511486)


def test_motion_command_nonzero():
    xp = [0.09956595655676025, 0.010059555197290845, 0.100863785511486]
    u = Odometry(r1=0.0993660353102, t=0.0999680211112, r2=-0.000241341506349)
    x = motion_command(xp, u)
    assert(x[0] == 0.19753670593862271)
    assert(x[1] == 0.029942651295374236)
    assert(x[2] == 0.199988479315337)
