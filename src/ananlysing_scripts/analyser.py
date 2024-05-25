import math
import time

from src import constants
from src.ananlysing_scripts.listeners.listeners import (angleToCoords, getDeltaAngle, StepListener, GyroListener,
                                                        ArucoCloserListener, Moving2TargetListener)
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State
from src.ananlysing_scripts.listeners.obstacle_scanning_listener import ObstacleScanningListener
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoInfo, rad2Deg

from src.logger import log, logBlue, logError

tag = "Iteration"


class Analyser:
    # rotateLeft = True

    state: State = State.MOVING2TARGET

    currentArucoId: int = -1
    arucoDict: dict[int, float]
    finishId: int = -1
    scannedArucoIds: list = []
    scannedArucoIdsSet = set()

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []
    __gyroListeners: [GyroListener] = []

    previousData: IterationData = IterationData()
    iterationData: IterationData = IterationData()

    arucoDetector: ArucoDetector

    absoluteAngle: float = 0.
    currentArucoDirectionAngle: float = 0.

    gyroTimeStamp = time.time()

    def __init__(self, executor: HardwareExecutorModel, arucoDict: dict[int, float], finishId: int):
        self.hardwareExecutor = executor
        self.arucoDict = arucoDict
        self.finishId = finishId

        self.__listeners.append(Moving2TargetListener(self))

        self.arucoDetector = ArucoDetector(self.hardwareExecutor.cameraMatrix, self.hardwareExecutor.distCfs)

    def onIteration(self):
        logBlue(f"Starting next step, state = {self.state}, {self.scannedArucoIds}, "
                f"{len(self.__gyroListeners)} and {len(self.__listeners)}", tag)
        self.previousData = self.iterationData
        self.iterationData = IterationData()

        self.iterationData.cameraImage = self.hardwareExecutor.readImage()
        self.iterationData.arucoResult = self.arucoDetector.onImage(self.iterationData.cameraImage)
        self.iterationData.arucoResult.calcAngles(self.absoluteAngle)

        if self.iterationData.arucoResult.isFound:
            self.onArucoFound()

        self.iterationData.sonarData = self.hardwareExecutor.readSonarData()
        log(f"Sonar read points = {self.iterationData.sonarData}", tag)

        self.notifyListeners(self.iterationData, self.previousData)

    def onArucoFound(self):
        arucoResult: ArucoInfo = self.iterationData.arucoResult

        # usual handling
        if self.state in {State.ROTATING, State.STOP, State.GETTING_CLOSER2ARUCO,
                          State.SCANNING_OBSTACLE, State.GETTING_OVER_AN_OBSTACLE_SCANNING,
                          State.GETTING_OVER_AN_OBSTACLE_FORWARD}:
            return

        for i, arucoId in enumerate(arucoResult.ids):
            if arucoId in self.scannedArucoIdsSet:
                continue

            if arucoId not in self.arucoDict and arucoId != self.finishId:
                continue

            if arucoId != self.finishId:
                angle = self.arucoDict[arucoId]
            else:
                angle = 0
            self.currentArucoId = arucoId

            angleToRotate = rad2Deg(math.atan((constants.imageW / 2 - arucoResult.centers[i]) *
                                              math.tan(constants.CAMERA_ANGLE / 2) / (constants.imageW / 2)))

            print(angleToRotate)
            self.currentArucoDirectionAngle = arucoResult.angles[i] + angle
            self.rotate(angle=angleToRotate, stateAfterRotation=State.GETTING_CLOSER2ARUCO)

            self.scannedArucoIds.append(arucoId)
            self.scannedArucoIdsSet.add(arucoId)

            return

    def onGyroIteration(self):
        currentTime = time.time()
        # dt = currentTime - self.gyroTimeStamp
        dt = constants.gyro_dt

        self.iterationData.gyroData = self.hardwareExecutor.readGyro()
        rotated = dt * self.iterationData.gyroData[2]

        self.iterationData.rotated += rotated
        self.absoluteAngle += rotated
        if self.absoluteAngle < 0:
            self.absoluteAngle = 360 + self.absoluteAngle
        if self.absoluteAngle >= 360:
            self.absoluteAngle = self.absoluteAngle % 360

        log(f"Gyro data = {self.iterationData.gyroData}, angle = {self.absoluteAngle}", tag)

        self.__notifyGyroListeners(self.iterationData.gyroData, dt)
        self.gyroTimeStamp = currentTime

    def rotate(self, *, angle=0., toRotate=0., stateAfterRotation):
        if self.state == State.ROTATING:
            logError("Trying to start rotation during ROTATING state", tag)
            return

        self.state = State.ROTATING
        if toRotate == 0:
            toRotate = self.absoluteAngle + angle

        toRotate = angleToCoords(toRotate)

        left: bool = getDeltaAngle(self.absoluteAngle, toRotate) > 0

        self.hardwareExecutor.rotate(toRotate, left, stateAfterRotation)

    def onRotationEnd(self, toState):
        self.state = toState

        match toState:
            case State.MOVING2TARGET:
                self.registerListener(Moving2TargetListener(self))
                self.hardwareExecutor.setSpeed(constants.MOVEMENT_SPEED)
            case State.GETTING_CLOSER2ARUCO:
                self.registerListener(ArucoCloserListener(self))
                self.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)

    def onGotClose2Aruco(self):
        if self.state != State.GETTING_CLOSER2ARUCO:
            return

        if self.currentArucoId == self.finishId:
            self.hardwareExecutor.setSpeed(0)
            self.state = State.STOP
            return

        self.currentArucoId = -1
        self.rotate(toRotate=self.currentArucoDirectionAngle, stateAfterRotation=State.MOVING2TARGET)

    def onObstacleFound(self):
        self.state = State.SCANNING_OBSTACLE
        self.registerListener(ObstacleScanningListener(self))

    def registerListener(self, listener):
        self.__listeners.append(listener)

    def removeListener(self, listener):
        self.__listeners.remove(listener)

    def notifyListeners(self, iterationData: IterationData, previousData: IterationData):
        for listener in self.__listeners:
            listener.onStep(iterationData, previousData)

    def registerGyroListener(self, listener):
        self.__gyroListeners.append(listener)

    def removeGyroListener(self, listener):
        self.__gyroListeners.remove(listener)

    def __notifyGyroListeners(self, gyroData, dt):
        for listener in self.__gyroListeners:
            listener.onStep(gyroData, self.absoluteAngle, self.currentArucoDirectionAngle, dt)
