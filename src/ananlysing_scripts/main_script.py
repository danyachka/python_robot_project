import numpy as np

from src.ananlysing_scripts.listeners import StepListener, IterationData
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import onImage, ArucoInfo


class Analyser:

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []

    def __init__(self, executor):
        self.hardwareExecutor = executor

    def onIteration(self):
        iterationData = IterationData()
        # image = self.hardwareExecutor.readImage()
        # iterationData.cameraImage = image
        # arucoResult: ArucoInfo = onImage(image)
        #
        # iterationData.arucoResult = arucoResult

        # if arucoResult.isFound:
        #     self.hardwareExecutor.setSpeed(0)

        gyroData = self.hardwareExecutor.readGyro()
        print(gyroData)

        iterationData.gyroData = gyroData

        self.__notifyListeners(iterationData)

    def registerListener(self, listener):
        self.__listeners.append(listener)

    def removeListener(self, listener):
        self.__listeners.remove(listener)

    def __notifyListeners(self, iterationData: IterationData):
        for listener in self.__listeners:
            listener.onStep(iterationData)
