import math
import time

import numpy as np
import sim
import cv2 as cv

from abc import ABCMeta, abstractmethod

from src.ananlysing_scripts.iteration_data import SonarInfo
from src.ananlysing_scripts.listeners import RotationListener
from src.execution_scripts.emulation.emulation_tools import GyroscopeEmulator
from src.logger import log, logError

tag = "HardwareExecutor"


# How to create interfaces (abstract classes)?
class HardwareExecutorModel:
    __metaclass__ = ABCMeta

    clientId = 0

    analyser = None

    @abstractmethod
    def __init__(self, clientId):
        self.clientId = clientId

    def setAnalyser(self, analyser):
        self.analyser = analyser

    @abstractmethod
    def readImage(self) -> np.ndarray:
        pass

    @abstractmethod
    def readGyro(self) -> [float, float, float]:
        pass

    @abstractmethod
    def readSonarData(self) -> SonarInfo:
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
    def rotate(self, toRotate, degrees) -> None:
        pass


# Should execute commands in simulation
class HardwareExecutorEmulator(HardwareExecutorModel):
    frontLeftWheel = None
    frontRightWheel = None
    backLeftWheel = None
    backRightWheel = None

    camera_handle = None

    robot_handle = None
    gyroscopeEmulator: GyroscopeEmulator

    sonarHandle_f = None
    sonarHandle_r = None
    sonarHandle_b = None
    sonarHandle_l = None

    isRotating: bool = False

    cameraMatrix = [[1.27358341e+03, 0.00000000e+00, 3.00942489e+02],
                    [0.00000000e+00, 1.26663343e+03, 2.33213114e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

    distCfs = [[7.43051630e-02, -5.16983657e+00, -1.01402024e-03, -2.80294514e-04, 9.13089594e+01]]

    def __init__(self, clientId):
        super().__init__(clientId)

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
        res_stream, resolution, image = sim.simxGetVisionSensorImage(
            self.clientId, self.camera_handle, 0, sim.simx_opmode_streaming)
        log(f'Camera handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}', tag)

        res, self.robot_handle = sim.simxGetObjectHandle(clientId, '/robot', sim.simx_opmode_oneshot_wait)
        res_stream, euler_angles = sim.simxGetObjectOrientation(
            self.clientId, self.robot_handle, -1, sim.simx_opmode_streaming)
        self.gyroscopeEmulator = GyroscopeEmulator()
        log(f'Robot handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}', tag)

        # Sonars
        res, self.sonarHandle_f = sim.simxGetObjectHandle(clientId, './sensor_front', sim.simx_opmode_oneshot_wait)
        res_stream, _, _, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_f, sim.simx_opmode_streaming))
        log(f'Sonar_f handel - {res is sim.simx_return_ok}, {res_stream == sim.simx_return_ok}, '
            f'{self.sonarHandle_f}', tag)

        res, self.sonarHandle_r = sim.simxGetObjectHandle(clientId, './sensor_right', sim.simx_opmode_oneshot_wait)
        res_stream, _, _, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_r, sim.simx_opmode_streaming))
        log(f'Sonar_r handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}, '
            f'{self.sonarHandle_r}', tag)

        res, self.sonarHandle_b = sim.simxGetObjectHandle(clientId, './sensor_back', sim.simx_opmode_oneshot_wait)
        res_stream, _, _, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_b, sim.simx_opmode_streaming))
        log(f'Sonar_b handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}, '
            f'{self.sonarHandle_b}', tag)

        res, self.sonarHandle_l = sim.simxGetObjectHandle(clientId, '/robot/sensor_left', sim.simx_opmode_oneshot_wait)
        res_stream, _, _, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_l, sim.simx_opmode_streaming))
        log(f'Sonar_l handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}, '
            f'{self.sonarHandle_l}', tag)

        self.readGyro()

    def readImage(self) -> np.ndarray:
        res, resolution, image = sim.simxGetVisionSensorImage(
            self.clientId, self.camera_handle, 0, sim.simx_opmode_buffer)

        if res == sim.simx_return_ok:
            time_0 = time.time_ns() / 1_000_000

            image = np.asarray(image).astype(np.uint8)
            # image = np.array(image, dtype=np.uint8)

            time_1 = time.time_ns() / 1_000_000
            image = image.reshape([resolution[1], resolution[0], 3])
            time_2 = time.time_ns() / 1_000_000
            image = np.flip(image, 0)
            image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
            time_3 = time.time_ns() / 1_000_000
            log(f"Image preparing - {time_1 - time_0}, {time_2 - time_1}, {time_3 - time_2}", tag)
            return image
        else:
            logError("Failed to capture image", tag)
            return None

    def readGyro(self) -> [float, float, float]:
        res, euler_angles = (
            sim.simxGetObjectOrientation(self.clientId, self.robot_handle, -1, sim.simx_opmode_buffer))

        return self.gyroscopeEmulator.update(euler_angles)

    def readSonarData(self) -> SonarInfo:
        err1, _, front, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_f, sim.simx_opmode_buffer))
        err2, _, right, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_r, sim.simx_opmode_buffer))
        err3, _, back, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_b, sim.simx_opmode_buffer))
        err4, _, left, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_l, sim.simx_opmode_buffer))

        if err1 != sim.simx_return_ok:
            logError("Error reading proximity sensor (front)", tag)
            front = -1
        else:
            front = math.sqrt(front[0]**2 + front[1]**2 + front[2]**2)
            if front < 10**-16:
                front = -1

        if err2 != sim.simx_return_ok:
            logError("Error reading proximity sensor (right)", tag)
            right = -1
        else:
            right = math.sqrt(right[0]**2 + right[1]**2 + right[2]**2)
            if right < 10**-16:
                right = -1
            elif right == front:
                right = -1

        if err3 != sim.simx_return_ok:
            logError("Error reading proximity sensor (back)", tag)
            back = -1
        else:
            back = math.sqrt(back[0]**2 + back[1]**2 + back[2]**2)
            if back < 10**-16:
                back = -1
            elif back == right:
                back = -1

        if err4 != sim.simx_return_ok:
            logError("Error reading proximity sensor (left)", tag)
            left = -1
        else:
            left = math.sqrt(left[0]**2 + left[1]**2 + left[2]**2)
            if left < 10**-16:
                left = -1
            elif left == back:
                left = -1

        sonarData = SonarInfo(front, right, back, left)

        return sonarData

    def readInfraScannerData(self):
        pass

    def setRightSpeed(self, speed) -> None:
        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.frontRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        log(f'frontRightWheel: {error is sim.simx_return_ok}', tag)

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        log(f'backRightWheel: {error is sim.simx_return_ok}', tag)

    def setLeftSpeed(self, speed) -> None:
        error = (sim.simxSetJointTargetVelocity
                 (self.clientId, self.frontLeftWheel, speed, sim.simx_opmode_oneshot_wait))
        log(f'frontLeftWheel: {error is sim.simx_return_ok}', tag)

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backLeftWheel, speed, sim.simx_opmode_oneshot_wait)
        log(f'backLeftWheel: {error is sim.simx_return_ok}', tag)

    def setSpeed(self, speed) -> None:
        self.setLeftSpeed(speed)
        self.setRightSpeed(speed)

    def rotate(self, toRotate, degrees) -> None:

        self.setLeftSpeed(0)
        self.setRightSpeed(0)

        self.isRotating = True

        listener = RotationListener(self, toRotate)

        v = 0.5
        if degrees > 0:
            self.setLeftSpeed(-v)
            self.setRightSpeed(v)
        else:
            self.setLeftSpeed(v)
            self.setRightSpeed(-v)

        self.analyser.registerGyroListener(listener)


# Should execute commands with actual hardware
class HardwareExecutor(HardwareExecutorModel):

    def __init__(self, clientId):
        super().__init__(clientId)

    def readImage(self) -> np.ndarray:
        pass

    def readGyro(self) -> [float, float, float]:
        pass

    def readSonarData(self) -> SonarInfo:
        pass

    def readInfraScannerData(self):
        pass

    def setSpeed(self, speed) -> None:
        pass

    def setRightSpeed(self, speed) -> None:
        pass

    def setLeftSpeed(self, speed) -> None:
        pass

    def rotate(self, toRotate, degrees) -> None:
        pass
