import math

import numpy as np
import cv2 as cv
from cv2 import aruco

from src.logger import log, logError

tag = "Camera"


def rad2Deg(rad):
    return rad * 180 / math.pi


class ArucoInfo:
    ids: np.ndarray
    isFound: bool

    normals: list

    angles: list[float]

    centers: list[int]

    def __init__(self, arucoIds, normals, isFound, centers):
        self.ids = arucoIds
        self.normals = normals
        self.isFound = isFound
        self.centers = centers

        self.__calcAngles()
        log(f"Detected ArUco marker IDs:{self.ids.flatten()}, normals: {normals}, angles: {self.angles}", "ArucoInfo")

    def __calcAngles(self):
        self.angles = []

        for normal in self.normals:
            if normal is None:
                self.angles.append(None)
                continue

            dx = normal[0]
            dz = normal[2]

            angle: float
            if dz > 0 > dx:
                # sec quarter
                angle = rad2Deg(math.atan(-dx / dz))
            elif dz < 0 and dx < 0:
                # third quarter
                angle = rad2Deg(math.atan(dz / dx)) + 90
            elif dz < 0 < dx:
                # forth quarter
                angle = rad2Deg(math.atan(dx / (-dz))) + 180
            else:
                # first quarter
                angle = rad2Deg(math.atan(dz / dx)) + 270

            self.angles.append(angle)


class ArucoDetector:
    detector: aruco.ArucoDetector

    cameraMatrix = None

    distCfs = None

    isTest: bool

    def __init__(self, cameraMatrix, distCfs, isTest=False):
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        arucoParams = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(arucoDict, arucoParams)
        self.cameraMatrix = cameraMatrix
        self.distCfs = distCfs
        self.isTest = isTest

    def onImage(self, image: np.ndarray) -> ArucoInfo:
        if image is None:
            return ArucoInfo(np.array([]), [], False, [])

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(image)

        normals = []
        centers = []
        if len(corners) > 0:
            aruco.drawDetectedMarkers(image, corners, ids)

            for i in range(len(corners)):
                center = abs(corners[i][0][0][0] + corners[i][0][2][0]) / 2
                # cv.line(image, (int(center), 0), (int(center), 479), (0, 0, 255))
                centers.append(center)
                normals.append(self.__getOrientation(image, corners[i]))

            log("Found ArUco marker IDs:" + str(ids.flatten()), tag)
        else:
            logError("No aruco detected", tag)

        cv.imshow("Camera image", image)
        if self.isTest:
            cv.waitKey(10000)
        else:
            cv.waitKey(1)

        log(f'Image\'s been processed', tag)

        if len(corners) > 0:
            return ArucoInfo(ids.flatten(), normals, True, centers)
        else:
            return ArucoInfo(np.array([]), [], False, [])

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
