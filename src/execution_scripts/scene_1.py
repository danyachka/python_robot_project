import sim
import time
import cv2 as cv
from threading import Thread

from src.ananlysing_scripts.analyser import Analyser
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator

from src.logger import log, logBlue

clientId = None
simTime = 0


def onDestroy():
    cv.destroyAllWindows()
    sim.simxFinish(clientId)


def startMainIteration(analyser, executor):
    isRotated = False
    frame_delay = 0.1
    while True:
        iterationStartTime = time.time()

        analyser.onIteration()

        if not isRotated:
            isRotated = True
            executor.rotate(720)

        # waite util next tick
        elapsed_time = time.time() - iterationStartTime
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

        iterationTime = time.time() - iterationStartTime
        logBlue(f'Iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n', "Scene")


def startGyroIteration(analyser):
    frame_delay = 0.01
    while True:
        iterationStartTime = time.time()

        analyser.onGyroIteration()

        # waite util next tick
        elapsed_time = time.time() - iterationStartTime
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

        # iterationTime = time.time() - iterationStartTime
        # logBlue(f'Gyro iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n', "Scene, gyro thread")


def main(sim_client_id):
    global clientId
    global simTime

    clientId = sim_client_id

    executor: HardwareExecutorEmulator = HardwareExecutorEmulator(sim_client_id)
    analyser: Analyser = Analyser(executor)
    executor.setAnalyser(analyser)

    gyroThread = Thread(target=startGyroIteration, args=[analyser])
    gyroThread.start()
    startMainIteration(analyser, executor)

