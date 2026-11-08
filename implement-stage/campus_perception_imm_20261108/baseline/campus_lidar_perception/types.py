"""ROS-independent measurements. Length follows shape_yaw, never velocity yaw."""
from dataclasses import dataclass
import math


@dataclass
class Detection:
    x: float
    y: float
    z: float = 0.0
    length: float = 0.5
    width: float = 0.5
    height: float = 0.5
    yaw: float = 0.0
    label: str = 'UNKNOWN'
    score: float = 1.0
    source: str = 'geometry'

    def validate(self):
        values = (self.x, self.y, self.z, self.length, self.width,
                  self.height, self.yaw, self.score)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('non-finite detection')
        if min(self.length, self.width, self.height) <= 0 or not 0 <= self.score <= 1:
            raise ValueError('invalid detection dimensions/score')
        return self


def planner_label(label):
    label = label.upper()
    if label == 'PEDESTRIAN':
        return 'pedestrian'
    if label in ('BICYCLE', 'MOTORCYCLE'):
        return 'bicycle'
    if label in ('CAR', 'TRUCK', 'BUS', 'VEHICLE'):
        return 'vehicle'
    return 'unknown'
