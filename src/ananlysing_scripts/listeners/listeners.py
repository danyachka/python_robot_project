import math

from src import constants
from src.ananlysing_scripts.camera_script import rad2Deg
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State

from src.logger import log


def isAngleClose(angle1, angle2, checkAngle=1.5):
    return abs(getDeltaAngle(angle1, angle2)) < checkAngle


def angleToCoords(angle):
    if angle < 0:
        angle = 360 + angle
    elif angle >= 360:
        angle = angle % 360
    return angle


def getDeltaAngle(absolute, other):
    da = 0.
    if absolute < 180:
        if absolute <= other <= absolute + 180:
            da = other - absolute
        else:
            if other < absolute:
                da = absolute - other
            else:
                da = absolute + (360 - other)
            da *= -1
    else:  # > 180
        if absolute - 180 <= other <= absolute:
            da = other - absolute
        else:
            if other > absolute:
                da = other - absolute
            else:
                da = other + (360 - absolute)

    return da


if __name__ == '__main__':
    print(getDeltaAngle(90, 45))
    print(getDeltaAngle(90, 330))
    print(getDeltaAngle(90, 210))
    print(getDeltaAngle(250, 45))


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
        if self.analyser.lastMeasuredArucoDistance < constants.ARUCO_DISTANCE:
            return True

        checkDist = constants.ARUCO_DISTANCE_CHECKING
        arucoInfo = self.analyser.arucoInfo
        if arucoInfo.isValid():
            checkDist /= math.cos(math.degrees(arucoInfo.angle - 180))

        if checkDist > constants.ARUCO_DISTANCE_CHECKING * 1.4:
            checkDist = constants.ARUCO_DISTANCE_CHECKING * 1.4

        return (iterationData.sonarData.front < checkDist and
                self.analyser.lastMeasuredArucoDistance < constants.ARUCO_CAMERA_DISTANCE)

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

            self.analyser.lastMeasuredArucoDistance = arucoResult.distances[i]

            if self.analyser.lastMeasuredArucoDistance > constants.ARUCO_CAMERA_DISTANCE:
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
