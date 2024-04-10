import sim
import numpy as np
import time
import cv2 as cv
from PIL import Image

import src.ananlysing_scripts.camera_script as camera_script
from src.ananlysing_scripts.main_script import Analyser
from src.execution_scripts.hardware_executor import HardwareExecutorEmulator

clientId = None


def onDestroy():
    cv.destroyAllWindows()
    sim.simxFinish(clientId)


def main(sim_client_id):
    global clientId

    clientId = sim_client_id

    executor: HardwareExecutorEmulator = HardwareExecutorEmulator(sim_client_id)
    analyser: Analyser = Analyser(executor)

    frame_delay = 0.05

    executor.setRightSpeed(0.25)
    executor.setLeftSpeed(-0.25)

    # main loop
    while True:
        errorCode, signalValue = sim.simxGetIntegerSignal(clientId, "simulationStopped", sim.simx_opmode_blocking)
        if errorCode == sim.simx_return_ok and signalValue == 1:
            print("Simulation stopped")
            onDestroy()
            break

        start_time = time.time()

        analyser.analyse()

        # waite util next tick
        elapsed_time = time.time() - start_time
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)
