import math

from src import constants
from src.ananlysing_scripts.camera_script import rad2Deg
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State

from src.logger import log


def isAngleClose(angle1, angle2):
    return abs(angle1 - angle2) < 1.5


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


class ObstacleScanningListener(StepListener):
    analyser = None

    anglesMap: dict[float, float] = {}

    counter = 0

    goLeft = True

    rotationLevel = 1

    def __init__(self, analyser):
        self.analyser = analyser

    def goOverAnObstacle(self, angle):
        self.analyser.removeListener(self)

        angle = self.analyser.currentArucoDirectionAngle + angle
        angle = angleToCoords(angle)

        if not isAngleClose(self.analyser.absoluteAngle, angle):
            self.analyser.rotate(toRotate=angle, stateAfterRotation=State.GETTING_OVER_AN_OBSTACLE)

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.state != State.SCANNING_OBSTACLE:
            self.analyser.removeListener(self)
            return

        # wait
        self.counter += 1

        if self.counter < constants.WAIT_TICKS_ON_SONAR:
            return

        dAlpha = getDeltaAngle(self.analyser.currentArucoDirectionAngle, self.analyser.absoluteAngle)

        sonarData = iterationData.sonarData.front
        if sonarData is None:
            self.goOverAnObstacle(dAlpha)
            return

        self.anglesMap[dAlpha] = sonarData
        self.counter = 0

        # if all angles checked
        if (not self.goLeft) and dAlpha < -constants.MAX_ANGLE_TO_CHECK_ON_OBSTACLE:
            # TODO: figure out what to do ant this point
            self.analyser.removeListener(self)
            self.analyser.state = State.STOP
            return

        if self.goLeft:
            self.analyser.rotate(toRotate=angleToCoords(
                self.analyser.currentArucoDirectionAngle + self.rotationLevel * constants.SHOT_ANGLE_ON_OBSTACLE),
                stateAfterRotation=State.SCANNING_OBSTACLE)
            self.goLeft = False
        else:
            self.analyser.rotate(toRotate=angleToCoords(
                self.analyser.currentArucoDirectionAngle - self.rotationLevel * constants.SHOT_ANGLE_ON_OBSTACLE),
                stateAfterRotation=State.SCANNING_OBSTACLE)
            self.goLeft = True
            self.rotationLevel += 1


class Moving2TargetListener(StepListener):
    analyser = None

    def __init__(self, analyser):
        self.analyser = analyser

    def endListening(self):
        self.analyser.onObstacleFound()
        self.analyser.removeListener(self)

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.state != State.MOVING2TARGET:
            self.endListening()
            return

        if self.analyser.currentArucoId != -1:
            self.endListening()
            return

        # Check if robot is close enough
        if constants.SONAR_FAKE_MIN < iterationData.sonarData.front < constants.OBSTACLE_DISTANCE:
            self.endListening()
            return


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

        if isAngleClose(self.angleToRotate, absoluteAngle):
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeGyroListener(self)
            self.executor.analyser.onRotationEnd(self.toState)
