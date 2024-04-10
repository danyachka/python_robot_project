from src.execution_scripts.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import onImage, ArucoInfo


class Analyser:

    hardwareExecutor: HardwareExecutorModel

    def __init__(self, executor):
        self.hardwareExecutor = executor

    def analyse(self):
        image = self.hardwareExecutor.readImage()
        arucoResult: ArucoInfo = onImage(image)

        if arucoResult.isFound:
            self.hardwareExecutor.setSpeed(0)

        gyroData = self.hardwareExecutor.readGyro()
        print(gyroData)

