import sim
import numpy as np
import time
import cv2 as cv
from PIL import Image

import src.scripts.camera_script as camera_script
import src.scripts.main_script as main_script

clientId = 0

frontLeftWheel = None
frontRightWheel = None
backLeftWheel = None
backRightWheel = None


def read_camera_image(camera_handle) -> np.ndarray:
    """
    Read the image from the camera in CoppeliaSim.
    """
    # Capture the image
    res, resolution, image = sim.simxGetVisionSensorImage(clientId, camera_handle, 0, sim.simx_opmode_blocking)

    if res == sim.simx_return_ok:
        # Convert the image to a format usable by OpenCV
        image = np.array(image, dtype=np.uint8)
        image = image.reshape([resolution[1], resolution[0], 3])
        image = np.flip(image, 0)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        return image
    else:
        print("Failed to capture image.")
        return None


def setSpeed(speed):
    error = sim.simxSetJointTargetVelocity(clientId, frontLeftWheel, --speed, sim.simx_opmode_oneshot_wait)
    print(f'frontLeftWheel: {error is sim.simx_return_ok}')
    error = sim.simxSetJointTargetVelocity(clientId, frontRightWheel, -speed, sim.simx_opmode_oneshot_wait)
    print(f'frontRightWheel: {error is sim.simx_return_ok}')
    error = sim.simxSetJointTargetVelocity(clientId, backRightWheel, speed, sim.simx_opmode_oneshot_wait)
    print(f'backRightWheel: {error is sim.simx_return_ok}')
    error = sim.simxSetJointTargetVelocity(clientId, backLeftWheel, -speed, sim.simx_opmode_oneshot_wait)
    print(f'backLeftWheel: {error is sim.simx_return_ok}')


def onDestroy():
    cv.destroyAllWindows()
    sim.simxFinish(clientId)
    print("Simulation stopped")


def main(sim_client_id):
    global clientId
    global frontLeftWheel
    global frontRightWheel
    global backRightWheel
    global backLeftWheel

    clientId = sim_client_id

    res1, frontLeftWheel = sim.simxGetObjectHandle(clientId, './front_left_wheel', sim.simx_opmode_oneshot_wait)
    print(frontLeftWheel)
    res2, frontRightWheel = sim.simxGetObjectHandle(clientId, './front_right_wheel', sim.simx_opmode_oneshot_wait)
    print(frontRightWheel)
    res3, backLeftWheel = sim.simxGetObjectHandle(clientId, './back_right_wheel', sim.simx_opmode_oneshot_wait)
    print(backLeftWheel)
    res4, backRightWheel = sim.simxGetObjectHandle(clientId, './back_left_wheel', sim.simx_opmode_oneshot_wait)
    print(backRightWheel)

    res, camera_handle = sim.simxGetObjectHandle(clientId, './camera', sim.simx_opmode_oneshot_wait)
    print(f'Camera handel - {res is sim.simx_return_ok}')

    setSpeed(0.2)

    frame_delay = 0.05

    # main loop
    while True:
        if sim.simxGetIntegerSignal(clientId, 'simulation_stop_signal', sim.simx_opmode_buffer)[1] == 1:
            onDestroy()
            break

        start_time = time.time()

        # Read the image from the camera
        img: np.ndarray = read_camera_image(camera_handle)
        camera_script.onImage(img)

        # waite util next tick
        elapsed_time = time.time() - start_time
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)
