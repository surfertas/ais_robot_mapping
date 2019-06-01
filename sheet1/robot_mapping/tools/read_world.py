#!/usr/bin/python3

from typing import List, Tuple
from tools.dataobjects import Landmark


def read_world(filename: str) -> List[Landmark]:
    """ Reads the world definition and returns a structure of landmarks. """
    landmarks = []
    with open(filename, 'r') as f:
        for line in f:
            fields = line.split(' ')
            landmarks.append(Landmark(*[int(field) for field in fields]))
    return landmarks
