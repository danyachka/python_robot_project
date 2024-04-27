import numpy as np
import time


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

    gyroData: [float, float, float] = [0, 0, 0]

    sonarData: SonarInfo = SonarInfo(-1, -1, -1, -1)

    arucoResult = None

    isRotating: bool = False

    def __init__(self):
        self.iterationStartTime = time.time()
