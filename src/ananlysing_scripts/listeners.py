from src import constants
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State

from src.logger import log


class StepListener:

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        pass


class ArucoCloserListener(StepListener):

    analyser = None

    def __init__(self, analyser):
        self.analyser = analyser

    def endListening(self):
        self.analyser.onGotClose2Aruco()
        self.analyser.removeListener(self)

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.currentArucoId == -1:
            self.endListening()

        if constants.ARUCO_DISTANCE < iterationData.sonarData.front < constants.SONAR_MAX_DIST:
            return

        if constants.SONAR_FAKE_MIN < iterationData.sonarData.front < constants.ARUCO_DISTANCE:
            self.endListening()
            return

        self.endListening()


class GyroListener:

    def onStep(self, gyroData, absoluteAngle, directionAngle, dt):
        pass


class RotationListener(GyroListener):
    toState = None

    def __init__(self, executor, angleToRotate, toState):
        self.executor = executor
        self.angleToRotate = angleToRotate
        self.toState = toState

        log(f"Angle to rotate = {angleToRotate}", "RotationListener")

    def onStep(self, gyroData, absoluteAngle, directionAngle, dt):

        log(f'RotationAngle = {absoluteAngle}, subtraction = {abs(self.angleToRotate - absoluteAngle)}'
            , "RotationListener")

        if abs(self.angleToRotate - absoluteAngle) < 1.5:
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeGyroListener(self)
            self.executor.analyser.onRotationEnd(self.toState)
