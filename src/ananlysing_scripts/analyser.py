import numpy as np

from src.ananlysing_scripts.listeners import StepListener
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoInfo

from src.logger import log, log

tag = "Iteration"


class Analyser:

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []

    previousData: IterationData = IterationData()

    arucoDetector: ArucoDetector

    def __init__(self, executor):
        self.hardwareExecutor = executor

        self.arucoDetector = ArucoDetector()

    def onIteration(self):
        iterationData = IterationData()

        iterationData.gyroData = self.hardwareExecutor.readGyro()
        log(f"Gyro data = {iterationData.gyroData}", tag)

        iterationData.cameraImage = self.hardwareExecutor.readImage()
        iterationData.arucoResult = self.arucoDetector.onImage(iterationData.cameraImage)

        if iterationData.arucoResult.isFound:
            self.hardwareExecutor.setSpeed(0)

        iterationData.sonarData = self.hardwareExecutor.readSonarData()
        log(f"Sonar read points = {iterationData.sonarData}", tag)

        self.__notifyListeners(iterationData, self.previousData)

        self.previousData = iterationData

    def registerListener(self, listener):
        self.__listeners.append(listener)

    def removeListener(self, listener):
        self.__listeners.remove(listener)

    def __notifyListeners(self, iterationData: IterationData, previousData: IterationData):
        for listener in self.__listeners:
            listener.onStep(iterationData, previousData)
