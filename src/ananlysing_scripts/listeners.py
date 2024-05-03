from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State

from src.logger import log


class StepListener:

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        pass


class GyroListener:

    def onStep(self, gyroData, absoluteAngle, directionAngle, dt):
        pass


class RotationListener(GyroListener):

    def __init__(self, executor, angleToRotate):
        self.executor = executor
        self.angleToRotate = angleToRotate

        log(f"Angle to rotate = {angleToRotate}", "RotationListener")

    def onStep(self, gyroData, absoluteAngle, directionAngle, dt):

        log(f'RotationAngle = {absoluteAngle}, subtraction = {abs(self.angleToRotate - absoluteAngle)}'
            , "RotationListener")

        if abs(self.angleToRotate - absoluteAngle) < 1.5:
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeGyroListener(self)
            self.executor.analyser.onRotationEnd()
