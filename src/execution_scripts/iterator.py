import sys
import threading
import json
import traceback
from sre_parse import State

import sim
import time
import cv2 as cv
from threading import Thread

from src import constants
from src.ananlysing_scripts.analyser import Analyser
from src.constants import gyro_dt, main_dt
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator, HardwareExecutor, HardwareExecutorModel

from src.logger import log, logBlue, logError, logCriticalError

tag = "Iterator"


class Iterator:
    isEmulation: bool

    clientId = 0

    simTime = 0

    analyser: Analyser

    executor: HardwareExecutorModel

    def __init__(self, isEmulation, arucoDict, finish):
        self.isEmulation = isEmulation

        if isEmulation:
            self.__startSimulation()

            self.executor = HardwareExecutorEmulator(self.clientId)
        else:
            self.executor = HardwareExecutor()

        self.analyser = Analyser(self.executor, arucoDict, finish)
        self.executor.setAnalyser(self.analyser)

    def __startSimulation(self):
        sim.simxFinish(-1)  # just in case, close all opened connections

        # В главном скрипте адрес и порт должны быть такими же
        self.clientId = sim.simxStart('127.0.0.1', 19999, True,
                                      True, 2000, 1)

        if self.clientId != -1:  # check if client connection successful
            logBlue('Connected to remote API server', tag)

        else:
            logError('Connection not successful', tag)
            sys.exit('Could not connect')

    def start(self):
        gyroThread = Thread(target=self.startGyroIteration, args=[], daemon=True)
        gyroThread.start()
        self.startMainIteration()

    def startMainIteration(self):
        try:
            constants.mainThreadId = threading.get_ident()

            isRotated = False
            frame_delay = main_dt
            while True:
                iterationStartTime = time.time()

                self.analyser.onIteration()

                # if not isRotated:
                #     isRotated = True
                #     self.analyser.rotate(angle=90, stateAfterRotation=2)

                # waite util next tick
                elapsed_time = time.time() - iterationStartTime
                if elapsed_time < frame_delay:
                    time.sleep(frame_delay - elapsed_time)

                iterationTime = time.time() - iterationStartTime
                logBlue(f'Iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n',
                        "Iterator (Main)", isImportant=True)
        except Exception as e:
            self.onException(e, True)

    def startGyroIteration(self):
        try:
            constants.gyroThreadId = threading.get_ident()

            frame_delay = gyro_dt
            while True:
                iterationStartTime = time.time()

                self.analyser.onGyroIteration()

                # waite util next tick
                elapsed_time = time.time() - iterationStartTime
                if elapsed_time < frame_delay:
                    time.sleep(frame_delay - elapsed_time)

                iterationTime = time.time() - iterationStartTime
                logBlue(f'Gyro iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}',
                        "Iterator (Gyro)", isImportant=False)
        except Exception as e:
            self.onException(e, False)

    def onDestroy(self):
        cv.destroyAllWindows()

        if self.isEmulation:
            sim.simxFinish(self.clientId)

    def onException(self, exception: Exception, isMainThread):
        trace = ''.join(traceback.format_exception(exception))
        resDict = {
            constants.ERROR_MESSAGE: trace,
            constants.IS_IN_MAIN: isMainThread,
            constants.CURRENT_DIRECTION: self.analyser.currentArucoDirectionAngle,
            constants.CURRENT_ID: self.analyser.currentArucoId,
            constants.CURRENT_ANGLE: self.analyser.absoluteAngle,
            constants.READ_IDS: self.analyser.scannedArucoIds,
            constants.STATE: str(self.analyser.state.name)
        }

        logCriticalError(trace, "Exception")

        text = json.dumps(resDict)

        file = open("data/exception_res.json", "w")
        file.write(text)
        file.close()

