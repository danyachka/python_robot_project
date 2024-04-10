import sim
import numpy as np
import time
import cv2 as cv
from PIL import Image

import src.ananlysing_scripts.camera_script as camera_script
from src.ananlysing_scripts.main_script import Analyser
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator

clientId = None
simTime = 0


def onDestroy():
    cv.destroyAllWindows()
    sim.simxFinish(clientId)


def main(sim_client_id):
    global clientId
    global simTime

    clientId = sim_client_id

    executor: HardwareExecutorEmulator = HardwareExecutorEmulator(sim_client_id)
    analyser: Analyser = Analyser(executor)
    executor.setAnalyser(analyser)

    frame_delay = 0.05

    # executor.setRightSpeed(0.25)
    # executor.setLeftSpeed(-0.25)

    timeAt = 1
    isRotated = False

    # main loop
    while True:
        simTime += frame_delay
        print(f'\n\nSimulation time = {simTime}')
        errorCode, signalValue = sim.simxGetIntegerSignal(clientId, "simulationStopped", sim.simx_opmode_blocking)
        if errorCode == sim.simx_return_ok and signalValue == 1:
            print("Simulation stopped")
            onDestroy()
            break

        start_time = time.time()

        analyser.onIteration()

        if abs(simTime - timeAt) < 0.0001 and not isRotated:
            isRotated = True
            executor.rotate(360)

        # waite util next tick
        elapsed_time = time.time() - start_time
        if elapsed_time < frame_delay:
            print(f'Sleeping for {frame_delay - elapsed_time}s')
            time.sleep(frame_delay - elapsed_time)
