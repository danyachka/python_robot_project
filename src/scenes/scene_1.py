import sim
import numpy as np
import time

import src.scripts.camera_script as camera_script
import src.scripts.main_script as main_script


clientId = 0


def read_camera_image(client_id, camera_handle):
    """
    Read the image from the camera in CoppeliaSim.
    """
    # Capture the image
    res, resolution, image = sim.simxGetVisionSensorImage(client_id, camera_handle, 0, sim.simx_opmode_buffer)

    if res == sim.simx_return_ok:
        # Convert the image to a format usable by OpenCV
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img = np.flip(img, 0)
        return img
    else:
        print("Failed to capture image.")
        return None


def main(sim_client_id):
    global clientId

    clientId = sim_client_id

    frontLeftWheel = sim.simxGetObjectHandle(clientId, './front_left_wheel', sim.simx_opmode_blocking)
    frontRightWheel = sim.simxGetObjectHandle(clientId, './front_right_wheel', sim.simx_opmode_blocking)
    backLeftWheel = sim.simxGetObjectHandle(clientId, './back_right_wheel', sim.simx_opmode_blocking)
    backRightWheel = sim.simxGetObjectHandle(clientId, './back_left_wheel', sim.simx_opmode_blocking)

    res, camera_handle = sim.simxGetObjectHandle(clientId, 'camera', sim.simx_opmode_oneshot_wait)

    frame_delay = 0.05

    # main loop
    while True:
        if sim.simxGetIntegerSignal(clientId, 'simulation_stop_signal', sim.simx_opmode_buffer)[1] == 1:
            break

        start_time = time.time()

        # Read the image from the camera
        img = read_camera_image(clientId, camera_handle)
        camera_script.onImage(img)

        # waite util next tick
        elapsed_time = time.time() - start_time
        if elapsed_time < frame_delay:
            time.sleep(frame_delay - elapsed_time)

