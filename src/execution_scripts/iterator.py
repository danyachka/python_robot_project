import threading

import sim
import time
import cv2 as cv
from threading import Thread

from src import constants
from src.ananlysing_scripts.analyser import Analyser
from src.constants import gyro_dt, main_dt
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator, HardwareExecutor, HardwareExecutorModel

from src.logger import log, logBlue, logError


class Iterator:
    clientId = None
    simTime = 0

    analyser: Analyser

    executor: HardwareExecutorModel

    def __init__(self, sim_client_id, isEmulation):
        self.clientId = sim_client_id

        if isEmulation:
            self.executor = HardwareExecutorEmulator(sim_client_id)
        else:
            self.executor = HardwareExecutor(sim_client_id)

        self.analyser = Analyser(self.executor)
        self.executor.setAnalyser(self.analyser)

    def start(self):
        gyroThread = Thread(target=self.startGyroIteration, args=[])
        gyroThread.start()
        self.startMainIteration()

    def startMainIteration(self):
        constants.mainThreadId = threading.get_ident()

        isRotated = False
        frame_delay = main_dt
        while True:
            iterationStartTime = time.time()

            self.analyser.onIteration()

            # if not isRotated:
            #     isRotated = True
            #     executor.rotate(720)

            # waite util next tick
            elapsed_time = time.time() - iterationStartTime
            if elapsed_time < frame_delay:
                time.sleep(frame_delay - elapsed_time)

            iterationTime = time.time() - iterationStartTime
            logBlue(f'Iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n',
                    "Iterator (Main)")

    def startGyroIteration(self):
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
                    "Iterator (Gyro)")

    def onDestroy(self):
        cv.destroyAllWindows()
        sim.simxFinish(self.clientId)

