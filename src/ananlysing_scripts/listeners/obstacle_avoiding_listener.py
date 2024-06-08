from src import constants
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State
from src.ananlysing_scripts.listeners.listeners import StepListener
from src.logger import logBlue, log
from src.utils import isAngleClose, getDeltaAngle, angleToCoords


class ObstacleAvoidingListener(StepListener):

    analyser = None

    angleStarted: float
    angleEnd: float

    isOnLeftSide: bool = True

    rotateCloser: float
    rotateOuter: float

    isCloserChecked = False
    previousDistances = []

    counter = 0

    forceMoveForward = False

    def __init__(self, analyser, isOnLeftSide):
        super().__init__()
        self.analyser = analyser

        self.angleStarted = analyser.absoluteAngle
        deltaAngle = getDeltaAngle(analyser.currentArucoDirectionAngle, self.angleStarted)
        self.angleEnd = angleToCoords(analyser.currentArucoDirectionAngle - deltaAngle)

        self.isOnLeftSide = isOnLeftSide

        if isOnLeftSide:
            self.rotateCloser = -constants.ROTATE_ON_AVOIDING
        else:
            self.rotateCloser = constants.ROTATE_ON_AVOIDING

        self.rotateOuter = -self.rotateCloser

    def endListening(self):
        self.analyser.removeListener(self)

        log(f"Obstacle avoiding ended", self.__class__.__name__, isImportant=True)

        if not isAngleClose(self.analyser.currentArucoDirectionAngle, self.analyser.absoluteAngle):
            self.analyser.rotate(toRotate=self.analyser.currentArucoDirectionAngle,
                                 stateAfterRotation=State.MOVING2TARGET)
        else:
            self.analyser.state = State.MOVING2TARGET
            self.analyser.hardwareExecutor.setSpeed(constants.MOVEMENT_SPEED)

    def getObstacleSideData(self, iterationData: IterationData, innerSide=True) -> float:
        if self.isOnLeftSide and innerSide:
            return iterationData.sonarData.left
        else:
            return iterationData.sonarData.right

    def checkForEnd(self):
        angle = getDeltaAngle(self.analyser.absoluteAngle, self.angleEnd)

        isEnd: bool
        if self.isOnLeftSide:
            isEnd = angle > 0
        else:
            isEnd = angle < 0

        return isEnd

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        logBlue(f'Starting new step: {self.analyser.state}, c = {self.counter}')
        if self.analyser.state == State.ROTATING:
            return

        if self.checkForEnd():
            self.endListening()
            return

        if self.analyser.state == State.GETTING_OVER_AN_OBSTACLE_FORWARD:
            self.onMovingFroward()
            return

        if self.analyser.state != State.GETTING_OVER_AN_OBSTACLE_SCANNING:
            self.analyser.removeListener(self)
            return

        log(f"Counted, counter = {self.counter}", self.__class__.__name__)
        # Skip if it hasn't waited
        if self.counter < constants.WAIT_TICKS_ON_SONAR:
            self.counter += 1
            return
        self.counter = 0

        if self.forceMoveForward:
            self.moveForward()
            return

        noObstacle = iterationData.sonarData.front > constants.OBSTACLE_DISTANCE
        sideSonarData = self.getObstacleSideData(iterationData)
        # Rotate closer if it hasn't
        if (not self.isCloserChecked) and (sideSonarData > constants.OBSTACLE_SIDE_DISTANCE):
            if noObstacle:
                self.analyser.rotate(angle=self.rotateCloser,
                                     stateAfterRotation=State.GETTING_OVER_AN_OBSTACLE_SCANNING)
            self.isCloserChecked = True
            return

        # Check for obstacles
        if noObstacle:
            self.moveForward()
            return

        if len(self.previousDistances) > 0:
            subtracted = iterationData.sonarData.front - self.previousDistances[-1]

            if constants.OBSTACLE_SUBTRACT_DISTANCE < subtracted:
                self.moveForward()
                return
            elif subtracted < -1 * constants.OBSTACLE_SUBTRACT_DISTANCE:
                self.forceMoveForward = True
                self.analyser.rotate(angle=self.rotateCloser,
                                     stateAfterRotation=State.GETTING_OVER_AN_OBSTACLE_SCANNING)
                return

        if self.getObstacleSideData(iterationData, False) > constants.OBSTACLE_SIDE_DISTANCE:
            self.analyser.rotate(angle=self.rotateOuter,
                                 stateAfterRotation=State.GETTING_OVER_AN_OBSTACLE_SCANNING)
            self.previousDistances.append(iterationData.sonarData.front)
        else:
            self.moveForward()

    def moveForward(self):
        self.analyser.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)
        self.analyser.state = State.GETTING_OVER_AN_OBSTACLE_FORWARD

        logBlue("Moving forward", self.__class__.__name__)

        self.previousDistances = []
        self.isCloserChecked = False
        self.counter = 0

        self.forceMoveForward = False

    def onMovingFroward(self):
        self.counter += 1

        if self.counter >= constants.WAIT_TICKS_ON_FORWARD:
            self.analyser.hardwareExecutor.setSpeed(0)

            self.counter = 0
            self.analyser.state = State.GETTING_OVER_AN_OBSTACLE_SCANNING
