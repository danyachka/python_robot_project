from pathlib import Path

import numpy as np
import cv2 as cv
from cv2 import aruco

from src.logger import log, logError

tag = "Camera"


class ArucoInfo:

    ids: np.ndarray
    isFound: bool

    def __init__(self, arucoIds, isFound):
        self.ids = arucoIds
        self.isFound = isFound


class ArucoDetector:
    detector: aruco.ArucoDetector

    cameraMatrix = None

    distCfs = None

    def __init__(self, cameraMatrix, distCfs):
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        arucoParams = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(arucoDict, arucoParams)
        self.cameraMatrix = cameraMatrix
        self.distCfs = distCfs

    def onImage(self, image: np.ndarray) -> ArucoInfo:
        if image is None:
            return ArucoInfo([], False)

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(image)

        if len(corners) > 0:
            aruco.drawDetectedMarkers(image, corners, ids)
            log("Detected ArUco marker IDs:" + str(ids.flatten()), tag)

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, self.cameraMatrix, self.distCfs)
            if ids is not None:
                for i in range(ids.size):
                    aruco.drawAxis(image, self.cameraMatrix, self.distCfs, rvecs[i], tvecs[i], 0.1)

        else:
            logError("No aruco detected", tag)

        cv.imshow("Camera image", image)
        cv.waitKey(1)

        log(f'Image\'s been processed', tag)

        if len(corners) > 0:
            return ArucoInfo(ids.flatten(), True)
        else:
            return ArucoInfo([], False)
