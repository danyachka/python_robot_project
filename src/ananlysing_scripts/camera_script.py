from pathlib import Path

import numpy as np
import cv2 as cv
from cv2 import aruco

from src.logger import log, logError

tag = "Camera"


class ArucoInfo:

    def __init__(self, angle, isFound):
        self.angle = angle
        self.isFound = isFound


class ArucoDetector:
    detector: aruco.ArucoDetector

    def __init__(self):
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        arucoParams = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(arucoDict, arucoParams)

    def onImage(self, image: np.ndarray) -> ArucoInfo:
        if image is None:
            return ArucoInfo(0, False)

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        image = aruco.drawDetectedMarkers(image, corners, ids)

        # rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)
        # if ids is not None:
        #     for i in range(ids.size):
        #         aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)

        if len(corners) > 0:
            aruco.drawDetectedMarkers(image, corners, ids)
            log("Detected ArUco marker IDs:" + str(ids.flatten()), tag)
        else:
            logError("No aruco detected", tag)

        cv.imshow("Camera image", image)
        cv.waitKey(1)

        log(f'Image\'s been processed', tag)

        return ArucoInfo(0, False)


if __name__ == '__main__':
    path = Path(__file__).parent.parent.parent.joinpath("data/aruco.png")

    img = cv.imread(str(path))

    arucoDetector = ArucoDetector()

    arucoDetector.onImage(img)
