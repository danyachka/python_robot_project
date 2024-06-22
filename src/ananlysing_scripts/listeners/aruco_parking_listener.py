from src import constants
from src.ananlysing_scripts.camera_script import fakeArucoInfo
from src.ananlysing_scripts.iteration_data import IterationData, State
from src.ananlysing_scripts.listeners.listeners import StepListener
from src.execution_scripts.executor.sonar_reading_model import SonarReadingModel
from src.utils import getDeltaAngle


class ParkingListener(StepListener):
    analyser = None

    turnedLeft: bool

    checkSonar = True

    counter = 0

    def __init__(self, analyser, turnedLeft):
        super().__init__()

        self.analyser = analyser
        self.turnedLeft = turnedLeft
        self.analyser.hardwareExecutor.sonarReadModel = SonarReadingModel(True, True, False, True)

    def getSonarData(self, iterationData: IterationData):
        if self.turnedLeft:
            return iterationData.sonarData.right

        return iterationData.sonarData.left

    def stop(self):

        angle = -90 if self.turnedLeft else 90

        self.analyser.hardwareExecutor.sonarReadModel = SonarReadingModel(True, False, False, False)

        self.analyser.rotate(angle=angle, stateAfterRotation=State.ARUCO_PARKING_CORRECT)
        self.analyser.removeListener(self)

    def onStep(self, iterationData: IterationData, previousData: IterationData):
        if self.analyser.state != State.ARUCO_PARKING:
            return

        self.counter += 1

        if self.counter > constants.WAIT_TICKS_ON_PARKING:
            self.stop()
            return

        distance = self.getSonarData(iterationData)

        if distance > constants.ARUCO_DISTANCE_PARKING or (not self.checkSonar):
            return

        self.checkSonar = False
        self.counter = constants.WAIT_TICKS_ON_PARKING - constants.WAIT_TICKS_ON_PARKING_KEEP_MOVING


class GettingCloseListener(StepListener):

    analyser = None

    counter = 0

    def __init__(self, analyser):
        super().__init__()
        self.analyser = analyser

    def stop(self):
        self.analyser.setArucoInfo(fakeArucoInfo)

        self.analyser.removeListener(self)

        self.analyser.rotate(toRotate=self.analyser.currentArucoDirectionAngle,
                             stateAfterRotation=State.MOVING2TARGET)

    def onStep(self, iterationData: IterationData, previousData: IterationData):

        if self.analyser.state != State.ARUCO_PARKING_END:
            self.stop()
            return

        self.counter += 1

        if self.counter > constants.WAIT_TICKS_ON_PARKING_END:
            self.stop()
