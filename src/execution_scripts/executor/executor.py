import time
from abc import ABC
from threading import Thread

import numpy as np
import cv2 as cv

from src import constants
from src.ananlysing_scripts.iteration_data import SonarInfo
from src.execution_scripts.executor.hardware_executor import HardwareExecutorModel
from src.logger import logError

if not constants.isEmulation:
    from mpu6050 import mpu6050
    import RPi.GPIO as GPIO


# Should execute commands with actual hardware
def setWheelSpeed(speed, IN1, IN2, pwm):
    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    pwm.ChangeDutyCycle(abs(speed))


class HardwareExecutor(HardwareExecutorModel, ABC):
    cap = None

    gyroSensor = None

    sonarThread: Thread = None

    lastSonarData: SonarInfo = SonarInfo()

    # right wheels
    IN1_r = 17
    IN2_r = 27
    ENA_r = 22
    pwm_r = None

    # left wheels
    IN1_l = 17
    IN2_l = 27
    ENA_l = 22
    pwm_l = None

    TRIG = [18, 20, 14, 16]
    ECHO = [17, 21, 13, 15]

    gyro_bias = [-1.8526727175380198, 0.4408821747229168, -1.686442607025719]

    cameraMatrix = np.array([[582.86635316, 0., 321.49076078],
                             [0., 584.82488533, 234.52954564],
                             [0., 0., 1.]])

    distCfs = np.array([[-0.39671064,  0.0474044,   0.00244292, -0.00081249,  0.56456562]])

    def __init__(self, GPIO=None):
        super().__init__()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for i in range(len(self.TRIG)):
            GPIO.setup(self.TRIG[i], GPIO.OUT)
            GPIO.setup(self.ECHO[i], GPIO.IN)

        self.cap = cv.VideoCapture(0)

        self.gyroSensor = mpu6050(0x68)

        if not self.cap.isOpened():
            raise Exception("Failed to open camera")

    def __setupWheels(self) -> None:
        GPIO.setup(self.IN1_r, GPIO.OUT)
        GPIO.setup(self.IN2_r, GPIO.OUT)
        GPIO.setup(self.ENA_r, GPIO.OUT)

        self.pwm_r = GPIO.PWM(self.ENA_r, 100)
        self.pwm_r.start(0)

        GPIO.setup(self.IN1_l, GPIO.OUT)
        GPIO.setup(self.IN2_l, GPIO.OUT)
        GPIO.setup(self.ENA_l, GPIO.OUT)

        self.pwm_l = GPIO.PWM(self.ENA_l, 100)
        self.pwm_l.start(0)

    def readImage(self) -> np.ndarray:

        ret, image = self.cap.read()

        if not ret:
            logError("Failed to capture image", self.__class__.__name__)
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

    def setRightSpeed(self, speed) -> None:
        setWheelSpeed(speed, self.IN1_r, self.IN2_r, self.pwm_r)

    def setLeftSpeed(self, speed) -> None:
        setWheelSpeed(speed, self.IN1_l, self.IN2_l, self.pwm_l)

    def onDestroy(self) -> None:
        self.cap.release()

        self.pwm_r.stop()
        self.pwm_l.stop()

        GPIO.cleanup()

