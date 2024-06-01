import math
import time
from threading import Thread

import numpy as np
import cv2 as cv

from abc import ABCMeta, abstractmethod, ABC

from src.ananlysing_scripts.iteration_data import SonarInfo
from src.ananlysing_scripts.listeners.listeners import RotationListener
from src.execution_scripts.emulation.emulation_tools import GyroscopeEmulator
from src.logger import log, logError
from src import constants

if constants.isEmulation:
    import sim
else:
    from mpu6050 import mpu6050
    import RPi.GPIO as GPIO

tag = "HardwareExecutor"


class HardwareExecutorModel:
    __metaclass__ = ABCMeta

    analyser = None

    cameraMatrix = None

    distCfs = None

    gyroKalmanFilter = None

    @abstractmethod
    def __init__(self):
        self.__initializeKalmanFilter()

    def __initializeKalmanFilter(self):
        kf = cv.KalmanFilter(2, 1)
        kf.measurementMatrix = np.array([[0, 1]], np.float32)
        kf.transitionMatrix = np.array([[1, 0],
                                        [0, 1]], np.float32)
        kf.processNoiseCov = np.eye(2, dtype=np.float32) * 1e-5
        kf.measurementNoiseCov = np.eye(1, dtype=np.float32) * 1e-5
        kf.errorCovPost = np.eye(2, dtype=np.float32) * 1e-1
        kf.statePost = np.zeros((2, 1), np.float32)

        self.gyroKalmanFilter = kf

    def setAnalyser(self, analyser):
        self.analyser = analyser

    @abstractmethod
    def readImage(self) -> np.ndarray:
        pass

    def readGyro(self, dt) -> float:
        data = self.readRawGyro()[2]

        self.gyroKalmanFilter.transitionMatrix[0, 1] = dt
        self.gyroKalmanFilter.correct(np.array(data, np.float32))

        prediction = self.gyroKalmanFilter.predict()[0][0] * dt

        return prediction

    @abstractmethod
    def readRawGyro(self) -> [float, float, float]:
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
    def rotate(self, toRotate, degrees, toState) -> None:
        pass

    @abstractmethod
    def onDestroy(self) -> None:
        pass


# Should execute commands in simulation
class HardwareExecutorEmulator(HardwareExecutorModel, ABC):
    clientId = 0

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

    cameraMatrix = np.array([[772.15579521, 0., 318.24378241],
                             [0., 772.28979442, 237.61600108],
                             [0., 0., 1.]])

    distCfs = np.array([[-9.28420136e-03, 1.41950687e-01, -6.17753741e-04, -2.64559643e-04, -6.80797471e-01]])

    def __init__(self, clientId):
        super().__init__()

        self.clientId = clientId

        res, self.frontLeftWheel = sim.simxGetObjectHandle(
            clientId, './lf', sim.simx_opmode_oneshot_wait)
        log(f'frontLeftWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.frontRightWheel = sim.simxGetObjectHandle(
            clientId, './rf', sim.simx_opmode_oneshot_wait)
        log(f'frontRightWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.backLeftWheel = sim.simxGetObjectHandle(
            clientId, './lb', sim.simx_opmode_oneshot_wait)
        log(f'backLeftWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.backRightWheel = sim.simxGetObjectHandle(
            clientId, './rb', sim.simx_opmode_oneshot_wait)
        log(f'backRightWheel handel - {res is sim.simx_return_ok}', tag)

        res, self.camera_handle = sim.simxGetObjectHandle(clientId, './camera', sim.simx_opmode_oneshot_wait)
        res_stream, resolution, image = sim.simxGetVisionSensorImage(
            self.clientId, self.camera_handle, 0, sim.simx_opmode_streaming)
        log(f'Camera handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}', tag)

        res, self.robot_handle = sim.simxGetObjectHandle(clientId, 'robot', sim.simx_opmode_oneshot_wait)
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

        res, self.sonarHandle_l = sim.simxGetObjectHandle(clientId, './sensor_left', sim.simx_opmode_oneshot_wait)
        res_stream, _, _, _, _ = (
            sim.simxReadProximitySensor(self.clientId, self.sonarHandle_l, sim.simx_opmode_streaming))
        log(f'Sonar_l handel - {res is sim.simx_return_ok}, {res_stream is sim.simx_return_ok}, '
            f'{self.sonarHandle_l}', tag)

        self.readRawGyro()

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

    def readGyro(self, dt) -> float:

        data = self.readRawGyro()[2] * dt

        return data

    def readRawGyro(self) -> [float, float, float]:
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
            front = constants.SONAR_DIST_NOTHING
        else:
            front = math.sqrt(front[0] ** 2 + front[1] ** 2 + front[2] ** 2)
            if front < constants.SONAR_FAKE_MIN:
                front = constants.SONAR_DIST_NOTHING

        if err2 != sim.simx_return_ok:
            logError("Error reading proximity sensor (right)", tag)
            right = constants.SONAR_DIST_NOTHING
        else:
            right = math.sqrt(right[0] ** 2 + right[1] ** 2 + right[2] ** 2)
            if right < constants.SONAR_FAKE_MIN:
                right = constants.SONAR_DIST_NOTHING
            elif right == front:
                right = constants.SONAR_DIST_NOTHING

        if err3 != sim.simx_return_ok:
            logError("Error reading proximity sensor (back)", tag)
            back = constants.SONAR_DIST_NOTHING
        else:
            back = math.sqrt(back[0] ** 2 + back[1] ** 2 + back[2] ** 2)
            if back < constants.SONAR_FAKE_MIN:
                back = constants.SONAR_DIST_NOTHING
            elif back == right:
                back = constants.SONAR_DIST_NOTHING

        if err4 != sim.simx_return_ok:
            logError("Error reading proximity sensor (left)", tag)
            left = constants.SONAR_DIST_NOTHING
        else:
            left = math.sqrt(left[0] ** 2 + left[1] ** 2 + left[2] ** 2)
            if left < constants.SONAR_FAKE_MIN:
                left = constants.SONAR_DIST_NOTHING
            elif left == back:
                left = constants.SONAR_DIST_NOTHING

        sonarData = SonarInfo(front, right, back, left)

        return sonarData

    def readInfraScannerData(self):
        pass

    def setRightSpeed(self, speed) -> None:
        speed = speed
        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.frontRightWheel, speed, sim.simx_opmode_oneshot_wait)
        log(f'frontRightWheel: {error is sim.simx_return_ok}', tag)

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backRightWheel, speed, sim.simx_opmode_oneshot_wait)
        log(f'backRightWheel: {error is sim.simx_return_ok}', tag)

    def setLeftSpeed(self, speed) -> None:
        speed = -speed
        error = (sim.simxSetJointTargetVelocity
                 (self.clientId, self.frontLeftWheel, speed, sim.simx_opmode_oneshot_wait))
        log(f'frontLeftWheel: {error is sim.simx_return_ok}', tag)

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backLeftWheel, speed, sim.simx_opmode_oneshot_wait)
        log(f'backLeftWheel: {error is sim.simx_return_ok}', tag)

    def setSpeed(self, speed) -> None:
        self.setLeftSpeed(speed)
        self.setRightSpeed(speed)

    def rotate(self, toRotate, left, toState) -> None:

        self.setLeftSpeed(0)
        self.setRightSpeed(0)

        self.isRotating = True

        listener = RotationListener(self, toRotate, toState)

        v = constants.ROTATION_SPEED
        if left:
            self.setLeftSpeed(-v)
            self.setRightSpeed(v)
        else:
            self.setLeftSpeed(v)
            self.setRightSpeed(-v)

        self.analyser.registerGyroListener(listener)

    def onDestroy(self) -> None:
        pass


# Should execute commands with actual hardware
class HardwareExecutor(HardwareExecutorModel, ABC):
    cap = None

    gyroSensor = None

    sonarThread: Thread = None

    lastSonarData: SonarInfo = SonarInfo()

    TRIG = [18, 20, 14, 16]
    ECHO = [17, 21, 13, 15]

    gyro_bias = [-1.8526727175380198, 0.4408821747229168, -1.686442607025719]

    cameraMatrix = np.array([[582.86635316, 0., 321.49076078],
                             [0., 584.82488533, 234.52954564],
                             [0., 0., 1.]])

    distCfs = np.array([[-0.39671064,  0.0474044,   0.00244292, -0.00081249,  0.56456562]])

    def __init__(self):
        super().__init__()

        GPIO.setmode(GPIO.BCM)
        for i in range(len(self.TRIG)):
            GPIO.setup(self.TRIG[i], GPIO.OUT)
            GPIO.setup(self.ECHO[i], GPIO.IN)

        self.cap = cv.VideoCapture(0)

        self.gyroSensor = mpu6050(0x68)

        if not self.cap.isOpened():
            raise Exception("Failed to open camera")

    def readImage(self) -> np.ndarray:

        ret, image = self.cap.read()

        if not ret:
            logError("Failed to capture image", tag)
            return None

        return image

    def readGyro(self, dt) -> float:
        data = self.readRawGyro()[2] * dt

        return data

    def readRawGyro(self) -> [float, float, float]:
        data = self.gyroSensor.get_gyro_data()

        if data is None:
            return [0, 0, 0]

        return [data['x'] - self.gyro_bias[0],
                data['y'] - self.gyro_bias[1],
                data['z'] - self.gyro_bias[2]]

    def readSonarData(self) -> SonarInfo:
        temp = self.lastSonarData

        if self.sonarThread is not None:
            if self.sonarThread.is_alive():
                return temp

        self.lastSonarData = SonarInfo()
        self.sonarThread = Thread(target=self.__startSonarThread)
        self.sonarThread.start()

        return temp

    def __startSonarThread(self):
        self.__clearSonar()

        absoluteStartTime = time.time()
        startTimes = [0] * len(self.TRIG)

        while True:
            time.sleep(constants.SONAR_CHECK_ITERATION_SLEEP)
            iterTime = time.time()
            if iterTime - absoluteStartTime > constants.SONAR_MAX_SIGNAL_WAITING:
                return

            for i in range(len(self.TRIG)):
                start = startTimes[i]
                if start != 0 and start != -1:
                    duration = iterTime - start
                    if duration > constants.SONAR_MAX_DURATION:
                        startTimes[i] = -1

            isEnd = True
            for i in range(len(self.TRIG)):
                isEnd = isEnd and startTimes[i] == -1
            if isEnd:
                return

            # Check if started
            for i in range(len(self.TRIG)):
                start = startTimes[i]
                if start == 0 and start != -1 and GPIO.input(self.ECHO[i]) == 1:
                    startTimes[i] = iterTime

            # Check if found
            for i in range(len(self.TRIG)):
                start = startTimes[i]
                if start != 0 and start != -1 and GPIO.input(self.ECHO[i]) == 0:
                    distance = (iterTime - start) * constants.SOUND_SPEED / 2

                    if distance > constants.SONAR_MAX_DIST:
                        logError(f"Some how distance is more, then {constants.SONAR_MAX_DIST}. "
                                 f"{distance}, {iterTime}, {startTimes}", self.__class__.__name__)
                        startTimes[i] = -1
                        continue

                    self.lastSonarData.setData(i, distance)
                    startTimes[i] = -1

    def __clearSonar(self):
        for trig in self.TRIG:
            GPIO.output(trig, True)

        time.sleep(0.00001)

        for trig in self.TRIG:
            GPIO.output(trig, False)

    def readInfraScannerData(self):
        pass

    def setSpeed(self, speed) -> None:
        pass

    def setRightSpeed(self, speed) -> None:
        pass

    def setLeftSpeed(self, speed) -> None:
        pass

    def rotate(self, toRotate, degrees, toState) -> None:
        pass

    def onDestroy(self) -> None:
        self.cap.release()
