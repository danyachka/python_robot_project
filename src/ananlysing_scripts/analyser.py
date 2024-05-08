import time

from src import constants
from src.ananlysing_scripts.listeners import StepListener, GyroListener
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo, State
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoInfo

from src.logger import log, logBlue, logError

tag = "Iteration"


class Analyser:
    rotateLeft = True

    state: State = State.MOVING_TO_TARGET
    scannedArucoIds: list = []
    scannedArucoIdsSet = set()

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []
    __gyroListeners: [GyroListener] = []

    previousData: IterationData = IterationData()
    iterationData: IterationData = IterationData()

    arucoDetector: ArucoDetector

    absoluteAngle: float = 0
    currentDirectionAngle: float = 0

    gyroTimeStamp = time.time()

    def __init__(self, executor):
        self.hardwareExecutor = executor

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
        if self.state != State.MOVING_TO_TARGET:
            return

        for arucoId in self.iterationData.arucoResult.ids:
            if arucoId in self.scannedArucoIdsSet:
                continue

            self.scannedArucoIds.append(arucoId)
            self.scannedArucoIdsSet.add(arucoId)

            # if self.rotateLeft:
            #     self.rotate(15)
            #     self.rotateLeft = False
            # else:
            #     self.rotate(-15)
            #     self.rotateLeft = True

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

    def rotate(self, angle):
        if self.state == State.ROTATING:
            logError("Trying to start rotation during ROTATING state", tag)
            return

        self.state = State.ROTATING
        toRotate = self.absoluteAngle + angle
        if toRotate < 0:
            toRotate = 360 + toRotate
        if toRotate >= 360:
            toRotate = toRotate % 360

        self.hardwareExecutor.rotate(toRotate, angle)

    def onRotationEnd(self):
        self.state = State.MOVING_TO_TARGET

        # placeHolder
        self.scannedArucoIds = []
        self.scannedArucoIdsSet = set()

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
            listener.onStep(gyroData, self.absoluteAngle, self.currentDirectionAngle, dt)
