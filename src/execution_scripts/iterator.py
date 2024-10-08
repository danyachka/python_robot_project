import sys
import threading
import json
import traceback

import numpy as np

import time
import cv2 as cv
from threading import Thread
from matplotlib import pyplot as plt
from src.execution_scripts.stream import stream

from src import constants
from src.ananlysing_scripts.analyser import Analyser
from src.constants import gyro_dt, main_dt
from src.execution_scripts.executor.hardware_executor import HardwareExecutorEmulator, HardwareExecutorModel
from src.execution_scripts.executor.executor import HardwareExecutor

from src.logger import logBlue, logError, logCriticalError

if constants.isEmulation:
    import sim

tag = "Iterator"


class Iterator:
    isReleased = False

    isEmulation: bool

    buildPlot: bool

    clientId = 0

    simTime = 0

    analyser: Analyser

    executor: HardwareExecutorModel

    def __init__(self, arucoDict, finish):
        self.isEmulation = constants.isEmulation
        self.buildPlot = constants.buildPlot

        if constants.buildPlot:
            self.plotData = np.zeros((150, 2))

        if constants.isEmulation:
            self.__startSimulation()

            self.executor = HardwareExecutorEmulator(self.clientId)
        else:
            self.executor = HardwareExecutor()

        self.analyser = Analyser(self.executor, arucoDict, finish)
        self.executor.setAnalyser(self.analyser)

        if constants.use_flask:
            self.__startStream()

    def __startStream(self):
        stream.analyser = self.analyser

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
        counter = 0
        startTime = time.time()

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

                if self.buildPlot:
                    if counter == len(self.plotData):
                        self.showPlot()
                        break

                    self.plotData[counter, 1] = self.analyser.absoluteAngle
                    self.plotData[counter, 0] = time.time() - startTime
                    counter += 1

                # waite util next tick
                elapsed_time = time.time() - iterationStartTime
                if elapsed_time < frame_delay:
                    time.sleep(frame_delay - elapsed_time)

                iterationTime = time.time() - iterationStartTime
                logBlue(f'Iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n',
                        "Iterator (Main)", isImportant=True)
        except Exception as e:
            self.onDestroy()
            self.onException(e, True)
        except KeyboardInterrupt:
            if not constants.use_flask:
                self.onDestroy()

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
            self.onDestroy()
            self.onException(e, False)

    def onDestroy(self):
        if self.isReleased:
            return
        self.isReleased = True

        if not constants.use_flask:
            cv.destroyAllWindows()

        logBlue("On destroy", self.__class__.__name__, isImportant=True)

        self.executor.onDestroy()

        logBlue("On destroy processed", self.__class__.__name__, isImportant=True)

        if self.isEmulation:
            sim.simxFinish(self.clientId)

    def onException(self, exception: Exception, isMainThread):
        trace = ''.join(traceback.format_exception(exception))

        print(type(self.analyser.arucoInfo.id))
        print(type(isMainThread))
        print(type(self.analyser.currentArucoDirectionAngle))
        print(type(self.analyser.arucoInfo.id))
        print(type(self.analyser.absoluteAngle))
        print(type(self.analyser.state.name))
        resDict = {
            constants.ERROR_MESSAGE: trace,
            constants.IS_IN_MAIN: isMainThread,
            constants.CURRENT_DIRECTION: self.analyser.currentArucoDirectionAngle,
            constants.CURRENT_ID: self.analyser.arucoInfo.id,
            constants.CURRENT_ANGLE: self.analyser.absoluteAngle,
            constants.READ_IDS: self.analyser.scannedArucoIds,
            constants.STATE: str(self.analyser.state.name)
        }

        logCriticalError(trace, "Exception")

        text = json.dumps(resDict)

        file = open("data/exception_res.json", "w")
        file.write(text)
        file.close()

        sys.exit()

    def showPlot(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.plotData, "-", color="blue")
        plt.title("Absolute angles")

        plt.show()

