import math
import time

import numpy as np

from src import constants
from src.ananlysing_scripts.listeners.aruco_parking_listener import ParkingListener, GettingCloseListener
from src.ananlysing_scripts.listeners.listeners import (StepListener, GyroListener,
                                                        ArucoCloserListener, Moving2TargetListener, Callback,
                                                        TicksListener)
from src.ananlysing_scripts.iteration_data import IterationData, State
from src.ananlysing_scripts.listeners.obstacle_scanning_listener import ObstacleScanningListener
from src.execution_scripts.emulation import emulation_tools
from src.execution_scripts.executor.hardware_executor import HardwareExecutorModel

from src.ananlysing_scripts.camera_script import ArucoDetector, ArucoResult, rad2Deg, ArucoInfo, fakeArucoInfo

from src.logger import log, logBlue, logError
from src.utils import angleToCoords, getDeltaAngle

tag = "Iteration"


class Analyser:
    # rotateLeft = True

    state: State = State.MOVING2TARGET

    arucoDict: dict[int, float]
    finishId: int = -1
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
    arucoInfo: ArucoInfo = fakeArucoInfo

    gyroTimeStamp = 0
    gyro_dt = constants.gyro_dt

    def __init__(self, executor: HardwareExecutorModel, arucoDict: dict[int, float], finishId: int):
        self.hardwareExecutor = executor
        self.arucoDict = arucoDict
        self.finishId = finishId

        self.__listeners.append(Moving2TargetListener(self))

        self.arucoDetector = ArucoDetector(self.hardwareExecutor.cameraMatrix, self.hardwareExecutor.distCfs)

    def onIteration(self):
        logBlue(f"Starting next step, state = {self.state}, {self.scannedArucoIds}, "
                f"{len(self.__gyroListeners)} and {len(self.__listeners)}", tag, isImportant=True)
        log(f"Gyro Hz = {1/self.gyro_dt}, abs = {self.absoluteAngle}, direction = {self.currentArucoDirectionAngle}",
            isImportant=True)
        self.previousData = self.iterationData
        self.iterationData = IterationData()

        self.iterationData.cameraImage = self.hardwareExecutor.readImage()
        self.iterationData.arucoResult = self.arucoDetector.onImage(self.iterationData.cameraImage)
        self.iterationData.arucoResult.calcAngles(self.absoluteAngle)

        if self.iterationData.arucoResult.isFound:
            self.onArucoFound()

        self.iterationData.sonarData = self.hardwareExecutor.readSonarData()
        log(f"Sonar read points = {self.iterationData.sonarData}", tag, isImportant=True)

        self.iterationData.hasPit = self.hardwareExecutor.readInfraScannerData()
        log(f"Bottom scanner data = {self.iterationData.hasPit}", tag)

        self.notifyListeners(self.iterationData, self.previousData)

    def onArucoFound(self):
        arucoResult: ArucoResult = self.iterationData.arucoResult

        # usual handling
        if self.state in {State.ROTATING, State.STOP, State.GETTING_CLOSER2ARUCO,
                          State.ARUCO_PARKING, State.ARUCO_PARKING_END}:
            return

        arucoId = -1
        index = -1
        for i, curArucoId in enumerate(arucoResult.ids):
            if curArucoId in self.scannedArucoIdsSet:
                continue

            if curArucoId not in self.arucoDict and curArucoId != self.finishId:
                continue

            if arucoResult.normals[i] is None:
                continue

            arucoId = curArucoId
            index = i

        if arucoId == -1:
            return

        if arucoId != self.finishId:
            angle = self.arucoDict[arucoId]
        else:
            angle = 0

        arucoId = int(arucoId)

        arucoInfo = ArucoInfo(arucoId, arucoResult.normals[index], arucoResult.angles[index],
                              arucoResult.centers[index], arucoResult.distances[index])
        self.arucoInfo = arucoInfo

        angleToRotate = rad2Deg(math.atan((constants.imageW / 2 - arucoInfo.center) *
                                          math.tan(constants.CAMERA_ANGLE / 2) / (constants.imageW / 2)))

        print(angleToRotate)
        self.currentArucoDirectionAngle = angleToCoords(arucoInfo.angle + angle)

        self.rotate(angle=angleToRotate, stateAfterRotation=State.GETTING_CLOSER2ARUCO)

        self.scannedArucoIds.append(arucoId)
        self.scannedArucoIdsSet.add(arucoId)

    def onGyroIteration(self):
        currentTime = time.time()
        # dt = currentTime - self.gyroTimeStamp
        self.gyro_dt = currentTime - self.gyroTimeStamp
        if self.gyro_dt == 0 or self.gyroTimeStamp == 0:
            self.gyro_dt = constants.gyro_dt

        emulation_tools.last_dt = self.gyro_dt

        self.iterationData.gyroData = self.hardwareExecutor.readGyro(self.gyro_dt)
        rotated = self.iterationData.gyroData

        self.iterationData.rotated += rotated
        self.absoluteAngle += rotated
        if self.absoluteAngle < 0:
            self.absoluteAngle = 360 + self.absoluteAngle
        if self.absoluteAngle >= 360:
            self.absoluteAngle = self.absoluteAngle % 360

        log(f"Gyro data = {self.iterationData.gyroData}, angle = {self.absoluteAngle}, "
            f"direction = {self.currentArucoDirectionAngle}", tag)

        self.__notifyGyroListeners(self.iterationData.gyroData, self.gyro_dt)
        self.gyroTimeStamp = currentTime

    def rotate(self, *, angle=0., toRotate=0., stateAfterRotation):
        if self.state == State.ROTATING:
            logError("Trying to start rotation during ROTATING state", tag)
            return

        self.hardwareExecutor.setSpeed(0)
        self.state = State.ROTATING

        class RotationStartCallback(Callback):
            def __init__(self, analyser, a, tR):
                self.analyser = analyser
                self.angle = a
                self.toRotate = tR

            def call(self):
                if self.toRotate == 0:
                    self.toRotate = self.analyser.absoluteAngle + self.angle

                self.toRotate = angleToCoords(self.toRotate)

                left: bool = getDeltaAngle(self.analyser.absoluteAngle, self.toRotate) > 0

                self.analyser.hardwareExecutor.rotate(self.toRotate, left, stateAfterRotation)

            def keepListening(self) -> bool:
                return True

        callBack = RotationStartCallback(self, angle, toRotate)
        self.registerListener(TicksListener(self, constants.WAIT_TICKS_ON_ROTATION, callBack))

    def onRotationEnd(self, toState):

        class RotationCallback(Callback):
            def __init__(self, analyser):
                self.analyser = analyser

            def call(self):
                self.analyser.state = toState

                match toState:
                    case State.MOVING2TARGET:
                        self.analyser.hardwareExecutor.setSpeed(constants.MOVEMENT_SPEED)
                    case State.GETTING_CLOSER2ARUCO:
                        self.analyser.registerListener(ArucoCloserListener(self.analyser))
                        self.analyser.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)
                    case State.ARUCO_PARKING_END:
                        self.analyser.getCloser2Aruco()
                    case State.ARUCO_PARKING:
                        self.analyser.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)
                    case State.ARUCO_PARKING_CORRECT:
                        angleToRotate = rad2Deg(math.atan((constants.imageW / 2 - self.analyser.arucoInfo.center) *
                                                          math.tan(constants.CAMERA_ANGLE / 2) / (
                                                                      constants.imageW / 2)))
                        self.analyser.rotate(angle=angleToRotate, stateAfterRotation=State.ARUCO_PARKING_END)

            def keepListening(self) -> bool:
                return True

        callBack = RotationCallback(self)
        self.registerListener(TicksListener(self, constants.WAIT_TICKS_ON_ROTATION, callBack))

    def onGotClose2Aruco(self):
        if self.state != State.GETTING_CLOSER2ARUCO:
            return

        dl = getDeltaAngle(self.absoluteAngle, self.arucoInfo.angle)
        log(f"Delta angle is: {dl}", tag, isImportant=True)

        if abs(180 - abs(dl)) < constants.MISSING_ANGLE_PARKING:
            self.onRotationEnd(State.ARUCO_PARKING_CORRECT)
            return

        isRight = dl > 0

        rotateAngle = angleToCoords(self.arucoInfo.angle + (-90 if isRight else 90))
        log(f"Parking toRotate is: {rotateAngle}", tag, isImportant=True)

        self.rotate(toRotate=rotateAngle, stateAfterRotation=State.ARUCO_PARKING)

        self.registerListener(ParkingListener(self, isRight))

    def getCloser2Aruco(self):

        class ParkingEndedCallback(Callback):
            def __init__(self, analyser):
                self.analyser = analyser

            def call(self):
                self.analyser.rotate(toRotate=self.analyser.currentArucoDirectionAngle,
                                     stateAfterRotation=State.MOVING2TARGET)

        class ParkingCallback(Callback):
            def __init__(self, analyser):
                self.analyser = analyser

            def call(self):
                self.analyser.hardwareExecutor.setSpeed(0)
                self.analyser.state = State.ARUCO_PARKING_END_FINALLY
                self.analyser.registerListener(
                    TicksListener(self.analyser, constants.WAIT_TICKS_ON_PARKING_FINALLY_ENDED,
                                  ParkingEndedCallback(self.analyser)))

            def keepListening(self) -> bool:
                return self.analyser.iterationData.sonarData.front > constants.ARUCO_DISTANCE

        callBack = ParkingCallback(self)
        self.state = State.ARUCO_PARKING_END_CLOSER
        self.hardwareExecutor.setSpeed(constants.LOW_MOVEMENT_SPEED)
        self.registerListener(TicksListener(self, constants.WAIT_TICKS_ON_PARKING_END_CLOSER, callBack))

    def onPitFound(self):
        self.hardwareExecutor.setSpeed(0)

        class CallbackPit(Callback):
            def __init__(self, analyser):
                self.analyser = analyser

            def call(self):
                self.analyser.onObstacleFound(shotAngle=2 * constants.SHOT_ANGLE_ON_OBSTACLE)

            def keepListening(self) -> bool:
                return self.analyser.iterationData.sonarData.back > constants.OBSTACLE_BACK_DISTANCE

        callBack = CallbackPit(self)
        self.registerListener(TicksListener(self, constants.WAIT_TICKS_ON_GOING_BACK_PIT, callBack))
        self.hardwareExecutor.setSpeed(-constants.LOW_MOVEMENT_SPEED)

    def onObstacleFound(self, shotAngle=constants.SHOT_ANGLE_ON_OBSTACLE):
        self.state = State.SCANNING_OBSTACLE
        self.hardwareExecutor.setSpeed(0)

        self.registerListener(ObstacleScanningListener(self, shotAngle))

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
