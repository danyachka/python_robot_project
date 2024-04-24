import numpy as np

from src.ananlysing_scripts.listeners import StepListener, IterationData
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import onImage, ArucoInfo

from src.logger import logRated, log

tag = "Iteration"


class Analyser:

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []

    previousData: IterationData = IterationData()

    def __init__(self, executor):
        self.hardwareExecutor = executor

    def onIteration(self):
        iterationData = IterationData()

        iterationData.gyroData = self.hardwareExecutor.readGyro()
        logRated(f"Gyro data = {iterationData.gyroData}", tag)

        image = self.hardwareExecutor.readImage()
        iterationData.cameraImage = image
        arucoResult: ArucoInfo = onImage(image)

        iterationData.arucoResult = arucoResult

        if arucoResult.isFound:
            self.hardwareExecutor.setSpeed(0)

        iterationData.sonarData = self.hardwareExecutor.readSonarData()
        logRated(f"Sonar read points = {iterationData.sonarData}", tag)

        self.__notifyListeners(iterationData, self.previousData)

        self.previousData = iterationData

    def registerListener(self, listener):
        self.__listeners.append(listener)

    def removeListener(self, listener):
        self.__listeners.remove(listener)

    def __notifyListeners(self, iterationData: IterationData, previousData: IterationData):
        for listener in self.__listeners:
            listener.onStep(iterationData, previousData)
