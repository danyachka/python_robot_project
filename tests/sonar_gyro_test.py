import threading
import time

from src import constants
from src.execution_scripts.executor.executor import HardwareExecutor
from src.execution_scripts.executor.sonar_reading_model import SonarReadingModel
from src.logger import logBlue, log


def gyroThread(executor: HardwareExecutor):
    constants.gyroThreadId = threading.get_ident()

    gyroTimeStamp = 0
    frame_delay = constants.gyro_dt
    absoluteAngle = 0.0
    while True:
        iterationStartTime = time.time()

        currentTime = time.time()
        dt = currentTime - gyroTimeStamp
        if dt == 0 or gyroTimeStamp == 0:
            dt = constants.gyro_dt

        gyroData = executor.readGyro(dt)
        rotated = gyroData

        absoluteAngle += rotated
        if absoluteAngle < 0:
            absoluteAngle = 360 + absoluteAngle
        if absoluteAngle >= 360:
            absoluteAngle = absoluteAngle % 360

        log(f"Gyro data = {gyroData}, angle = {absoluteAngle}", "gyro_thread")

        # waite util next tick
        elapsed_time = time.time() - iterationStartTime
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

        gyroTimeStamp = currentTime


def mainThread(executor: HardwareExecutor):
    frame_delay = constants.main_dt
    try:
        while True:
            iterationStartTime = time.time()

            sonarData = executor.readSonarData()
            logBlue(f"Sonar read points = {sonarData}", "main_thread", isImportant=True)

            # waite util next tick
            elapsed_time = time.time() - iterationStartTime
            if elapsed_time < frame_delay:
                time.sleep(frame_delay - elapsed_time)

    except KeyboardInterrupt:
        executor.onDestroy()


if __name__ == '__main__':
    hardwareExecutor = HardwareExecutor(setupGyro=True,
                                        setupSonar=True,
                                        setupCam=False,
                                        setupWheels=False,
                                        setupLaser=False)

    hardwareExecutor.sonarReadModel = SonarReadingModel(True, True, True, True)

    gyroThread = threading.Thread(target=gyroThread, args=[hardwareExecutor], daemon=True)
    gyroThread.start()

    mainThread(hardwareExecutor)
