import math

from src import constants
from src.ananlysing_scripts.iteration_data import State, IterationData
from src.ananlysing_scripts.listeners.listeners import angleToCoords, getDeltaAngle, isAngleClose, StepListener
from src.ananlysing_scripts.listeners.obstacle_avoiding_listener import ObstacleAvoidingListener


class ObstacleScanningListener(StepListener):
    analyser = None

    anglesMap: dict[float, float] = {}

    counter = 0

    goLeft = True

    rotationLevel = 1

    goOver = False

    def __init__(self, analyser):
        super().__init__()
        self.analyser = analyser

    def goOverAnObstacle(self, angle):
        angle = self.analyser.currentArucoDirectionAngle + angle
        angle = angleToCoords(angle)

        if not isAngleClose(self.analyser.absoluteAngle, angle):
            self.analyser.rotate(toRotate=angle, stateAfterRotation=State.SCANNING_OBSTACLE)
            self.goOver = True
        else:
            self.startGoingOverAnObstacle()

    def startGoingOverAnObstacle(self):
        self.analyser.removeListener(self)

        isOnLeftSide = getDeltaAngle(self.analyser.absoluteAngle, self.analyser.currentArucoDirectionAngle) < 0

        newListener = ObstacleAvoidingListener(self.analyser, isOnLeftSide)
        self.analyser.registerListener(newListener)
        newListener.moveForward()

    def calcMinObstacleDistance(self, angle) -> float:
        d = 1.4 * constants.OBSTACLE_DISTANCE
        # if not isAngleClose(angle, 0):
        #     d = constants.OBSTACLE_DISTANCE / math.cos(angle)
        #
        # if d > 1.2 * constants.OBSTACLE_DISTANCE:
        #     d = 1.2 * constants.OBSTACLE_DISTANCE
        return d

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.state == State.ROTATING:
            return
        elif self.analyser.state != State.SCANNING_OBSTACLE:
            self.analyser.removeListener(self)
            return

        if self.goOver:
            self.startGoingOverAnObstacle()
            return

        # wait
        self.counter += 1

        if self.counter < constants.WAIT_TICKS_ON_SONAR:
            return

        dAlpha = getDeltaAngle(self.analyser.currentArucoDirectionAngle, self.analyser.absoluteAngle)

        sonarData = iterationData.sonarData.front
        if sonarData > self.calcMinObstacleDistance(dAlpha):
            self.goOverAnObstacle(dAlpha)
            return

        self.anglesMap[dAlpha] = sonarData
        self.counter = 0

        # if all angles checked
        if dAlpha < -constants.MAX_ANGLE_TO_CHECK_ON_OBSTACLE:
            self.goOverAnObstacle(dAlpha)
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
