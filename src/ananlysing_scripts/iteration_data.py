from enum import Enum

from src.ananlysing_scripts.camera_script import ArucoInfo

import numpy as np
import time


class State(Enum):
    STOP = 0

    ROTATING = 1

    MOVING2TARGET = 2

    GETTING_CLOSER2ARUCO = 3

    SCANNING_OBSTACLE = 4

    GETTING_OVER_AN_OBSTACLE_FORWARD = 5

    GETTING_OVER_AN_OBSTACLE_SCANNING = 6


class SonarInfo:
    front: float
    right: float
    back: float
    left: float

    def __init__(self, front, right, back, left):
        self.front = front
        self.right = right
        self.back = back
        self.left = left

    def __str__(self):
        return f"{self.front}, {self.right}, {self.back}, {self.left}"


class IterationData:
    iterationStartTime: float

    cameraImage: np.ndarray = None

    gyroData: float = 0

    rotated: float = 0

    sonarData: SonarInfo = SonarInfo(-1, -1, -1, -1)

    arucoResult: ArucoInfo = None

    isRotating: bool = False

    def __init__(self):
        self.iterationStartTime = time.time()
