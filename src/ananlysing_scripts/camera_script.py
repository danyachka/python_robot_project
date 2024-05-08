from pathlib import Path

import numpy as np
import cv2 as cv
from cv2 import aruco

from src.logger import log, logError

tag = "Camera"


class ArucoInfo:
    ids: np.ndarray
    isFound: bool

    normals: list

    def __init__(self, arucoIds, normals, isFound):
        self.ids = arucoIds
        self.normals = normals
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
            return ArucoInfo([], [], False)

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(image)

        normals = []
        if len(corners) > 0:
            aruco.drawDetectedMarkers(image, corners, ids)
            log("Detected ArUco marker IDs:" + str(ids.flatten()), tag)

            for i in range(len(corners)):
                normals.append(self.__getOrientation(image, corners[i]))

        else:
            logError("No aruco detected", tag)

        cv.imshow("Camera image", image)
        cv.waitKey(1)

        log(f'Image\'s been processed', tag)

        if len(corners) > 0:
            return ArucoInfo(ids.flatten(), normals, True)
        else:
            return ArucoInfo([], [], False)

    def __getOrientation(self, image, corners) -> np.ndarray:
        side = 10
        marker_points_3d = np.array([[-side / 2, side / 2, 0],
                                     [side / 2, side / 2, 0],
                                     [side / 2, -side / 2, 0],
                                     [-side / 2, -side / 2, 0]],
                                    dtype=np.float32)

        success, rVecs, tVecs = cv.solvePnP(marker_points_3d, corners, self.cameraMatrix, self.distCfs,
                                            flags=cv.SOLVEPNP_IPPE_SQUARE)

        if success:
            R, _ = cv.Rodrigues(rVecs)

            normal = R[:, 2]

            endPoint, _ = cv.projectPoints(np.array([(0.0, 0.0, 5.0)]),
                                           rVecs, tVecs, self.cameraMatrix, self.distCfs)

            point1 = (int(np.average(corners[0, :, 0])), int(np.average(corners[0, :, 1])))
            point2 = (int(endPoint[0][0][0]), int(endPoint[0][0][1]))

            cv.line(image, point1, point2, (0, 0, 255), 2)

            return normal
        else:
            return None
