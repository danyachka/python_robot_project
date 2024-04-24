import time
import numpy as np

from src.logger import log


class IterationData:
    iterationStartTime: float

    cameraImage: np.ndarray

    gyroData: [float, float, float]

    sonarData = []

    arucoResult = None

    isRotating: bool

    def __init__(self):
        self.iterationStartTime = time.time()


class StepListener:

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        pass


class RotationListener(StepListener):
    currentRotatedAngle = 0

    def __init__(self, executor, angle):
        self.executor = executor
        self.angle = angle

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        current = iterationData.gyroData[2]

        iterationDuration = iterationData.iterationStartTime - previousData.iterationStartTime
        self.currentRotatedAngle += current * iterationDuration

        log(f'RotationAngle = {self.currentRotatedAngle}', "RotationListener")

        if abs(self.currentRotatedAngle) > abs(self.angle):
            self.currentRotatedAngle = 0
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeListener(self)
