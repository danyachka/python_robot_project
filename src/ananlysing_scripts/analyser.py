import math
import time

from src import constants
from src.ananlysing_scripts.listeners import StepListener, GyroListener, ArucoCloserListener
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoInfo, rad2Deg

from src.logger import log, logBlue, logError

tag = "Iteration"


class Analyser:
    # rotateLeft = True

    state: State = State.MOVING2TARGET

    currentArucoId: int = -1
    arucoDict: dict[int, float]
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

    def __init__(self, executor: HardwareExecutorModel, arucoDict: dict[int, float]):
        self.hardwareExecutor = executor
        self.arucoDict = arucoDict

        self.arucoDetector = ArucoDetector(self.hardwareExecutor.cameraMatrix, self.hardwareExecutor.distCfs)

    def onIteration(self):
        logBlue(f"Starting next step, state = {self.state}, {self.scannedArucoIds}, {len(self.__gyroListeners)}", tag)
        self.previousData = self.iterationData
        self.iterationData = IterationData()

        self.iterationData.cameraImage = self.hardwareExecutor.readImage()
        self.iterationData.arucoResult = self.arucoDetector.onImage(self.iterationData.cameraImage)

        if self.iterationData.arucoResult.isFound:
            self.onArucoFound()

        self.iterationData.sonarData = self.hardwareExecutor.readSonarData()
        log(f"Sonar read points = {self.iterationData.sonarData}", tag)

        self.notifyListeners(self.iterationData, self.previousData)

    def onArucoFound(self):
        # placeHolder
        if self.state != State.MOVING2TARGET:
            return

        for i, arucoId in enumerate(self.iterationData.arucoResult.ids):
            if arucoId in self.scannedArucoIdsSet:
                continue

            angle = self.arucoDict[arucoId]
            if angle is None:
                continue
            self.currentArucoId = arucoId

            angleToRotate = rad2Deg(math.atan((constants.imageW / 2 - self.iterationData.arucoResult.centers[i]) *
                                              math.tan(constants.CAMERA_ANGLE / 2) / (constants.imageW / 2)))

            print(angleToRotate)
            self.currentArucoDirectionAngle = self.iterationData.arucoResult.angles[i] + angle
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

        if toRotate < 0:
            toRotate = 360 + toRotate
        if toRotate >= 360:
            toRotate = toRotate % 360

        left: bool
        if self.absoluteAngle < 180:
            left = self.absoluteAngle < toRotate < self.absoluteAngle + 180
        else:
            left = not (self.absoluteAngle - 180 < toRotate < self.absoluteAngle)

        self.hardwareExecutor.rotate(toRotate, left, stateAfterRotation)

    def onRotationEnd(self, toState):
        self.state = toState

        match toState:
            case State.MOVING2TARGET:
                self.hardwareExecutor.setSpeed(constants.MOVEMENT_SPEED)
            case State.GETTING_CLOSER2ARUCO:
                self.registerListener(ArucoCloserListener(self))
                self.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)

    def onGotClose2Aruco(self):
        if self.state != State.GETTING_CLOSER2ARUCO:
            return

        self.rotate(toRotate=self.currentArucoDirectionAngle, stateAfterRotation=State.MOVING2TARGET)

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
