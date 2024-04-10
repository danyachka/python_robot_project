import time
import numpy as np


class IterationData:
    cameraImage: np.ndarray

    gyroData: [float, float, float]

    sonarData = None

    arucoResult = None

    isRotating: bool


class StepListener:

    def onStep(self, iterationData: IterationData):
        pass


class RotationListener(StepListener):
    currentRotatedAngle = 0
    measuredTime = time.time()

    def __init__(self, executor, angle):
        self.executor = executor
        self.angle = angle

    def onStep(self, iterationData: IterationData):
        current = iterationData.gyroData[2]

        currentTime = time.time()

        self.currentRotatedAngle += current * (currentTime - self.measuredTime)
        self.measuredTime = currentTime
        print(f'RotationAngle = {self.currentRotatedAngle}')

        if abs(self.currentRotatedAngle) > abs(self.angle):
            self.currentRotatedAngle = 0
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeListener(self)
