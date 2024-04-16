from PIL import Image
import numpy as np
import cv2 as cv
import cv2.aruco as aruco


class ArucoInfo:

    def __init__(self, angle, isFound):
        self.angle = angle
        self.isFound = isFound


def onImage(image: np.ndarray) -> ArucoInfo:
    if image is None:
        return ArucoInfo(0, False)

    # arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    # arucoParams = aruco.DetectorParameters_create()
    # (corners, ids, rejected) = aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    #
    # # Проверяем, был ли обнаружен хотя бы один ArUco маркер
    # if len(corners) > 0:
    #     # Наложение на изображение квадратов вокруг обнаруженных маркеров
    #     aruco.drawDetectedMarkers(image, corners, ids)
    #     print("Detected ArUco marker IDs:", ids.flatten())

    cv.imshow("Camera image", image)
    cv.waitKey(1)

    print(f'Image\'s been processed')

    return ArucoInfo(0, False)
