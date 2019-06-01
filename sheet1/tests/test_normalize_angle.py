import pytest
from robot_mapping.tools.normalize_angle import normalize_angle


def test_normalize_angle_zero():
    phi = 0.0
    norm_phi = normalize_angle(phi)
    assert(norm_phi == phi)


def test_normalize_angle_greater():
    phi = 4.5
    norm_phi = normalize_angle(phi)
    assert(norm_phi == -1.7831853071795862)


def test_normalize_angle_less():
    phi = -3.5
    norm_phi = normalize_angle(phi)
    assert(norm_phi == 2.7831853071795862)
