from src.ananlysing_scripts.listeners import StepListener
from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import onImage, ArucoInfo


class Analyser:

    hardwareExecutor: HardwareExecutorModel
    __listeners: [StepListener] = []

    def __init__(self, executor):
        self.hardwareExecutor = executor

    def onIteration(self):
        self.__notifyListeners()

        # image = self.hardwareExecutor.readImage()
        # arucoResult: ArucoInfo = onImage(image)
        #
        # if arucoResult.isFound:
        #     self.hardwareExecutor.setSpeed(0)

        gyroData = self.hardwareExecutor.readGyro()
        print(gyroData)

    def registerListener(self, listener):
        self.__listeners.append(listener)

    def removeListener(self, listener):
        self.__listeners.remove(listener)

    def __notifyListeners(self):
        for listener in self.__listeners:
            listener.onStep()
