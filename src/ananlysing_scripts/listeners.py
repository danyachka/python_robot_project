import math

from src import constants
from src.ananlysing_scripts.camera_script import rad2Deg
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

        # Check if robot is close enough
        if constants.SONAR_FAKE_MIN < iterationData.sonarData.front < constants.ARUCO_DISTANCE:
            self.endListening()
            return

        # Rotate if angle is missing
        arucoResult = iterationData.arucoResult
        for i, arucoId in enumerate(arucoResult.ids):
            if arucoId != self.analyser.currentArucoId:
                continue

            if arucoId != self.analyser.finishId:
                angle = self.analyser.arucoDict[arucoId]
            else:
                angle = 0
            self.analyser.currentArucoDirectionAngle = arucoResult.angles[i] + angle

            angleToRotate = rad2Deg(math.atan((constants.imageW / 2 - arucoResult.centers[i]) *
                                              math.tan(constants.CAMERA_ANGLE / 2) / (constants.imageW / 2)))

            isMissingDirection = abs(angleToRotate) > constants.MAX_MISSING_ANGLE
            if isMissingDirection:
                self.analyser.rotate(angle=angleToRotate, stateAfterRotation=State.GETTING_CLOSER2ARUCO)
                self.analyser.removeListener(self)
                return


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
