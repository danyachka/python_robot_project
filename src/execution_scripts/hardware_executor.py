import time

import numpy as np
import sim
import cv2 as cv

from abc import ABCMeta, abstractmethod, abstractproperty

from src.ananlysing_scripts.listeners import RotationListener
from src.execution_scripts.emulation_tools import GyroscopeEmulator
from src.logger import logRated, log

tag = "HardwareExecutor"


# How to create interfaces (abstract classes)?
class HardwareExecutorModel:
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def readImage(self) -> np.ndarray:
        pass

    @abstractmethod
    def readGyro(self) -> [float, float, float]:
        pass

    @abstractmethod
    def readSonarData(self) -> list:
        pass

    @abstractmethod
    def readInfraScannerData(self):
        pass

    @abstractmethod
    def setSpeed(self, speed) -> None:
        pass

    @abstractmethod
    def setRightSpeed(self, speed) -> None:
        pass

    @abstractmethod
    def setLeftSpeed(self, speed) -> None:
        pass

    @abstractmethod
    def rotate(self, angle) -> None:
        pass


# Should execute commands in simulation
class HardwareExecutorEmulator(HardwareExecutorModel):
    clientId = 0
    analyser = None

    frontLeftWheel = None
    frontRightWheel = None
    backLeftWheel = None
    backRightWheel = None

    camera_handle = None

    robot_handle = None
    gyroscopeEmulator: GyroscopeEmulator

    sonarHandle = None

    isRotating: bool = False

    def __init__(self, clientId):
        super().__init__()
        self.clientId = clientId

        res, self.frontLeftWheel = sim.simxGetObjectHandle(
            clientId, './front_left_wheel', sim.simx_opmode_oneshot_wait)
        log(f'frontLeftWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.frontRightWheel = sim.simxGetObjectHandle(
            clientId, './front_right_wheel', sim.simx_opmode_oneshot_wait)
        log(f'frontRightWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.backLeftWheel = sim.simxGetObjectHandle(
            clientId, './back_right_wheel', sim.simx_opmode_oneshot_wait)
        log(f'backLeftWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.backRightWheel = sim.simxGetObjectHandle(
            clientId, './back_left_wheel', sim.simx_opmode_oneshot_wait)
        log(f'backRightWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.camera_handle = sim.simxGetObjectHandle(clientId, '/robot/camera', sim.simx_opmode_oneshot_wait)
        res_stream, resolution, image = sim.simxGetVisionSensorImage(self.clientId, self.camera_handle, 0, sim.simx_opmode_streaming)
        log(f'Camera handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}', tag)

        res, self.robot_handle = sim.simxGetObjectHandle(clientId, '/robot', sim.simx_opmode_oneshot_wait)
        log(f'Robot handel - {res is sim.simx_return_ok}', tag)
        self.gyroscopeEmulator = GyroscopeEmulator()

        res, self.sonarHandle = sim.simxGetObjectHandle(clientId, '/robot/sensor', sim.simx_opmode_oneshot_wait)
        log(f'Sonar handel - {res is sim.simx_return_ok}', tag)

        self.readGyro()

    def setAnalyser(self, analyser):
        self.analyser = analyser

    def readImage(self) -> np.ndarray:
        res, resolution, image = sim.simxGetVisionSensorImage(
            self.clientId, self.camera_handle, 0, sim.simx_opmode_buffer)

        if res == sim.simx_return_ok:
            # Convert the image to a format usable by OpenCV
            time_0 = time.time_ns() / 1_000_000

            image = np.asarray(image)
            # image = np.array(image, dtype=np.uint8)

            time_1 = time.time_ns() / 1_000_000
            image = image.astype(np.uint8).reshape([resolution[1], resolution[0], 3])
            time_2 = time.time_ns() / 1_000_000
            image = np.flip(image, 0)
            image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
            time_3 = time.time_ns() / 1_000_000
            logRated(f"{time_1 - time_0}, {time_2 - time_1}, {time_3 - time_2}", tag)
            return image
        else:
            log("Failed to capture image.", tag)
            return None

    def readGyro(self) -> [float, float, float]:
        res, euler_angles = (
            sim.simxGetObjectOrientation(self.clientId, self.robot_handle, -1, sim.simx_opmode_blocking))

        return self.gyroscopeEmulator.update(euler_angles)

    def readSonarData(self) -> list:
        err, detectionState, detectedPoints, detectedObjectHandle, detectedSurfaceNormalVector = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle, sim.simx_opmode_blocking))

        if err == sim.simx_return_ok:
            if detectionState:
                return detectedPoints
            else:
                return None
        else:
            log("Error reading proximity sensor")
            return None

    def readInfraScannerData(self):
        pass

    def setRightSpeed(self, speed) -> None:
        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.frontRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        logRated(f'frontRightWheel: {error is sim.simx_return_ok}', "wheel")

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        logRated(f'backRightWheel: {error is sim.simx_return_ok}', "wheel")

    def setLeftSpeed(self, speed) -> None:
        error = (sim.simxSetJointTargetVelocity
                 (self.clientId, self.frontLeftWheel, speed, sim.simx_opmode_oneshot_wait))
        logRated(f'frontLeftWheel: {error is sim.simx_return_ok}', "wheel")

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backLeftWheel, speed, sim.simx_opmode_oneshot_wait)
        logRated(f'backLeftWheel: {error is sim.simx_return_ok}', "wheel")

    def setSpeed(self, speed) -> None:
        self.setLeftSpeed(speed)
        self.setRightSpeed(speed)

    def rotate(self, angle) -> None:
        self.setLeftSpeed(0)

        self.isRotating = True

        listener = RotationListener(self, angle)

        v = 1
        if angle > 0:
            self.setLeftSpeed(-v)
            self.setRightSpeed(v)
        else:
            self.setLeftSpeed(v)
            self.setRightSpeed(-v)

        self.analyser.registerListener(listener)


# Should execute commands with actual hardware
class HardwareExecutor(HardwareExecutorModel):

    def __init__(self):
        super().__init__()
        pass

    def readImage(self) -> np.ndarray:
        pass

    def readGyro(self) -> [float, float, float]:
        pass

    def readSonarData(self) -> list:
        pass

    def readInfraScannerData(self):
        pass

    def setSpeed(self, speed) -> None:
        pass

    def setRightSpeed(self, speed) -> None:
        pass

    def setLeftSpeed(self, speed) -> None:
        pass

    def rotate(self, angle) -> None:
        pass
