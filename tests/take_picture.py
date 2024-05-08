import sim
import numpy as np
import cv2 as cv


def main():
    sim.simxFinish(-1)

    clientID = sim.simxStart('127.0.0.1', 19999, True,
                             True, 5000, 1)

    res, camera_handle = sim.simxGetObjectHandle(clientID, '/robot/camera', sim.simx_opmode_oneshot_wait)
    res_stream, resolution, image = sim.simxGetVisionSensorImage(clientID, camera_handle, 0,
                                                                 sim.simx_opmode_oneshot_wait)

    image = np.asarray(image).astype(np.uint8)

    image = image.reshape([resolution[1], resolution[0], 3])
    image = np.flip(image, 0)
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)

    cv.imwrite("../data/chess_emulation/3.jpg", image)


if __name__ == '__main__':
    main()
