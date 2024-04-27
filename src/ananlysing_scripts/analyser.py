from src.ananlysing_scripts.listeners import StepListener
from src.ananlysing_scripts.iteration_data import IterationData, SonarInfo
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoInfo

from src.logger import log

tag = "Iteration"


class Analyser:

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []

    previousData: IterationData = IterationData()
    iterationData: IterationData = IterationData()

    arucoDetector: ArucoDetector

    absoluteAngle: float = 0

    currentDirectionAngle: float = 0

    def __init__(self, executor):
        self.hardwareExecutor = executor

        self.arucoDetector = ArucoDetector()

    def onIteration(self):
        self.iterationData = IterationData()

        self.iterationData.cameraImage = self.hardwareExecutor.readImage()
        self.iterationData.arucoResult = self.arucoDetector.onImage(self.iterationData.cameraImage)

        if self.iterationData.arucoResult.isFound:
            self.hardwareExecutor.setSpeed(0)

        self.iterationData.sonarData = self.hardwareExecutor.readSonarData()
        log(f"Sonar read points = {self.iterationData.sonarData}", tag)

        self.previousData = self.iterationData

    def onGyroIteration(self):
        self.iterationData.gyroData = self.hardwareExecutor.readGyro()
        log(f"Gyro data = {self.iterationData.gyroData}", tag)

        self.__notifyGyroListeners(self.iterationData, self.previousData)

    def registerGyroListener(self, listener):
        self.__listeners.append(listener)

    def removeGyroListener(self, listener):
        self.__listeners.remove(listener)

    def __notifyGyroListeners(self, iterationData: IterationData, previousData: IterationData):
        for listener in self.__listeners:
            listener.onStep(iterationData, previousData)
