from pathlib import Path

import sim
import numpy as np
import cv2 as cv


def getImageEmu() -> np.ndarray:
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
    return image


def getImageReal() -> np.ndarray:
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Не удалось открыть камеру")
        exit()

    ret, image = cap.read()
    cap.release()

    return image


def main():
    i = input("Number of image: ")
    image = getImageEmu()

    # /data/chess_real/1...n
    path = Path(__file__).parent.parent.joinpath("data").joinpath("chess_emulation").joinpath(f"{i}.jpg")
    cv.imwrite(str(path), image)


if __name__ == '__main__':
    main()
