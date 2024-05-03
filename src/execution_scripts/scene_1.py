import sim
import time
import cv2 as cv
from threading import Thread

from src.ananlysing_scripts.analyser import Analyser
from src.constants import gyro_dt, main_dt
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator

from src.logger import log, logBlue, logError

clientId = None
simTime = 0


def onDestroy():
    cv.destroyAllWindows()
    sim.simxFinish(clientId)


def startMainIteration(analyser, executor):
    isRotated = False
    frame_delay = main_dt
    while True:
        iterationStartTime = time.time()

        analyser.onIteration()

        # if not isRotated:
        #     isRotated = True
        #     executor.rotate(720)

        # waite util next tick
        elapsed_time = time.time() - iterationStartTime
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

        iterationTime = time.time() - iterationStartTime
        logBlue(f'Iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}\n', "Scene")


def startGyroIteration(analyser):
    isRotated = False

    frame_delay = gyro_dt
    while True:
        iterationStartTime = time.time()

        analyser.onGyroIteration()

        # if not isRotated:
        #     analyser.rotate(360)
        #     isRotated = True

        # waite util next tick
        elapsed_time = time.time() - iterationStartTime
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

        iterationTime = time.time() - iterationStartTime
        logBlue(f'Gyro iteration TPS = {1 / iterationTime if iterationTime != 0 else "infinity"}', "Scene, gyro")


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

