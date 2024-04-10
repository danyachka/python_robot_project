class StepListener:

    def onStep(self):
        pass


class RotationListener(StepListener):
    currentRotatedAngle = 0

    def __init__(self, executor, angle):
        self.executor = executor
        self.angle = angle

    def onStep(self):
        current = self.executor.readGyro()[2]

        self.currentRotatedAngle += current
        print(f'RotationAngle = {self.currentRotatedAngle}')

        if self.currentRotatedAngle > self.angle:
            self.currentRotatedAngle = 0
            self.executor.isRotating = False

            self.executor.setLeftSpeed(0)
            self.executor.setRightSpeed(0)

            self.executor.analyser.removeListener(self)
