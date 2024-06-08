import math

from src import constants
from src.ananlysing_scripts.camera_script import rad2Deg
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State

from src.logger import log
from src.utils import isAngleClose, angleToCoords


class StepListener:

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        pass


class Moving2TargetListener(StepListener):
    analyser = None

    def __init__(self, analyser):
        super().__init__()
        self.analyser = analyser

    def endListening(self):
        self.analyser.onObstacleFound()

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.state != State.MOVING2TARGET:
            return

        if self.analyser.arucoInfo.isValid():
            return

        # Check if robot is close enough
        if constants.SONAR_FAKE_MIN < iterationData.sonarData.front < constants.OBSTACLE_DISTANCE:
            self.analyser.onObstacleFound()
            return

        if not isAngleClose(self.analyser.absoluteAngle, self.analyser.currentArucoDirectionAngle,
                            constants.MAX_MISSING_ANGLE_ON_MOVING):
            self.analyser.rotate(toRotate=self.analyser.currentArucoDirectionAngle,
                                 stateAfterRotation=State.MOVING2TARGET)
            return


class ArucoCloserListener(StepListener):
    analyser = None

    def __init__(self, analyser):
        super().__init__()
        self.analyser = analyser

    def endListening(self):
        self.analyser.onGotClose2Aruco()
        self.analyser.removeListener(self)

    def isCloseEnough(self, iterationData: IterationData) -> bool:
        if self.analyser.arucoInfo.distance < constants.ARUCO_DISTANCE:
            return True

        checkDist = constants.ARUCO_DISTANCE_CHECKING
        arucoInfo = self.analyser.arucoInfo
        if arucoInfo.isValid():
            checkDist /= math.cos(math.degrees(arucoInfo.angle - 180))

        if checkDist > constants.ARUCO_DISTANCE_CHECKING * 1.4:
            checkDist = constants.ARUCO_DISTANCE_CHECKING * 1.4

        return ((iterationData.sonarData.front < checkDist and
                self.analyser.arucoInfo.distance < constants.ARUCO_CAMERA_DISTANCE) or
                self.analyser.arucoInfo.distance < checkDist)

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if not self.analyser.arucoInfo.isValid():
            self.endListening()
            return

        # Check if robot is close enough
        if self.isCloseEnough(iterationData):
            self.endListening()
            return

        # Rotate if angle is missing
        arucoResult = iterationData.arucoResult
        for i, arucoId in enumerate(arucoResult.ids):
            if arucoId != self.analyser.arucoInfo.id:
                continue

            if arucoId != self.analyser.finishId:
                angle = self.analyser.arucoDict[arucoId]
            else:
                angle = 0

            self.analyser.arucoInfo.normal = arucoResult.normals[i]
            self.analyser.arucoInfo.angle = arucoResult.angles[i]
            self.analyser.arucoInfo.center = arucoResult.centers[i]
            self.analyser.arucoInfo.distance = arucoResult.distances[i]

            if self.analyser.arucoInfo.distance > constants.ARUCO_CAMERA_DISTANCE:
                self.analyser.currentArucoDirectionAngle = angleToCoords(arucoResult.angles[i] + angle)

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
        super().__init__()
        self.executor = executor
        self.angleToRotate = angleToRotate
        self.toState = toState

        log(f"Angle to rotate = {angleToRotate}", "RotationListener", isImportant=True)

    def onStep(self, gyroData, absoluteAngle, directionAngle, dt):
        log(f'RotationAngle = {absoluteAngle}, subtraction = {abs(self.angleToRotate - absoluteAngle)}'
            , "RotationListener")

        if isAngleClose(self.angleToRotate, absoluteAngle, checkAngle=3):
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeGyroListener(self)
            self.executor.analyser.onRotationEnd(self.toState)
