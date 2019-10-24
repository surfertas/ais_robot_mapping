#!/usr/bin/python3

from typing import List, Tuple
from dataclasses import dataclass, field


@dataclass
class Odometry:
    r1: float = 0.0
    t: float = 0.0
    r2: float = 0.0

    def __str__(self) -> str:
        return f'rot1={self.r1} trans={self.t} rot2={self.r2}'


@dataclass
class Sensor:
    sid: int
    srange: float
    bearing: float

    def __str__(self) -> str:
        return f'id={self.sid} range={self.srange} bearing={self.bearing}'


@dataclass
class Data:
    timestep: List[Tuple[Odometry, List[Sensor]]] = field(default_factory=lambda: [])
    first: bool = True

    def __len__(self):
        return len(self.timestep)


@dataclass
class Landmark:
    lid: int
    x: int
    y: int

    def __repr__(self) -> str:
        return f'id={self.lid} x={self.x} y={self.y}'
