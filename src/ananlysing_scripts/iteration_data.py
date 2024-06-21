from enum import Enum

from src.constants import SONAR_DIST_NOTHING
from src.ananlysing_scripts.camera_script import ArucoResult

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

    ARUCO_PARKING = 7

    ARUCO_PARKING_END = 8

    ARUCO_PARKING_CORRECT = 9

    ARUCO_PARKING_END_CLOSER = 10

    ARUCO_PARKING_END_FINALLY = 11


class SonarInfo:
    front: float
    right: float
    back: float
    left: float

    def __init__(self, front=SONAR_DIST_NOTHING, right=SONAR_DIST_NOTHING,
                 back=SONAR_DIST_NOTHING, left=SONAR_DIST_NOTHING):
        self.front = front
        self.right = right
        self.back = back
        self.left = left

    def setData(self, index, data):
        match index:
            case 0:
                self.front = data
            case 1:
                self.right = data
            case 2:
                self.back = data
            case 3:
                self.left = data

    def __str__(self):
        return f"{self.front}, {self.right}, {self.back}, {self.left}"


class IterationData:
    iterationStartTime: float

    cameraImage: np.ndarray = None

    gyroData: float = 0

    rotated: float = 0

    sonarData: SonarInfo = SonarInfo(-1, -1, -1, -1)

    hasPit = False

    arucoResult: ArucoResult = None

    isRotating: bool = False

    def __init__(self):
        self.iterationStartTime = time.time()
