import math
from dataclasses import dataclass

import numpy as np
import cv2 as cv
from cv2 import aruco

from src import constants
from src.logger import log, logError
from src.utils import angleToCoords

tag = "Camera"


def rad2Deg(rad):
    return rad * 180 / math.pi


@dataclass
class ArucoInfo:
    id: int

    normal: list

    angle: float

    center: int

    distance: float

    def isValid(self):
        return self.id != -1


fakeArucoInfo = ArucoInfo(-1, [0, 0, 0], 0, 0, float('inf'))


class ArucoResult:
    ids: np.ndarray
    isFound: bool

    normals: list

    angles: list[float]

    centers: list[int]

    distances = list[float]

    def __init__(self, arucoIds, normals, isFound, centers, distances):
        self.ids = arucoIds
        self.normals = normals
        self.isFound = isFound
        self.centers = centers
        self.distances = distances

    def calcAngles(self, absoluteAngle):
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

            angle = angleToCoords(absoluteAngle + angle)

            self.angles.append(angle)

        log(f"Detected ArUco marker IDs:{self.ids.flatten()}, normals: {self.normals}, "
            f"angles: {self.angles}, distances: {self.distances}", "ArucoInfo", isImportant=True)


class ArucoDetector:
    detector: aruco.ArucoDetector

    cameraMatrix = None

    distCfs = None

    isTest: bool

    def __init__(self, cameraMatrix, distCfs, isTest=False):
        arucoDict = aruco.getPredefinedDictionary(constants.arucoDictionary)
        arucoParams = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(arucoDict, arucoParams)
        self.cameraMatrix = cameraMatrix
        self.distCfs = distCfs
        self.isTest = isTest

    def onImage(self, image: np.ndarray) -> ArucoResult:
        if image is None:
            return ArucoResult(np.array([]), [], False, [], [])

        # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(image)

        normals = []
        centers = []
        distances = []
        if len(corners) > 0:
            aruco.drawDetectedMarkers(image, corners, ids)

            for i in range(len(corners)):
                center = abs(corners[i][0][0][0] + corners[i][0][2][0]) / 2
                # cv.line(image, (int(center), 0), (int(center), 479), (0, 0, 255))
                centers.append(center)

                normal, distance = self.__getOrientation(image, corners[i])

                normals.append(normal)
                distances.append(distance)

            log("Found ArUco marker IDs:" + str(ids.flatten()), tag)
        else:
            logError("No aruco detected", tag)

        if self.isTest:
            cv.imshow("Camera image test", image)
            cv.waitKey(5_000)
        elif not constants.use_flask:
            cv.imshow("Camera image", image)
            cv.waitKey(1)

        log(f'Image\'s been processed', tag)

        if len(corners) > 0:
            return ArucoResult(ids.flatten(), normals, True, centers, distances)
        else:
            return ArucoResult(np.array([]), [], False, [], [])

    def __getOrientation(self, image, corners) -> tuple[np.ndarray, float]:
        side = constants.ARUCO_SIDE_SIZE
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

            distance = math.sqrt(tVecs[0][0]**2 + tVecs[1][0]**2 + tVecs[2][0]**2) / constants.ARUCO_SIDE_SIZE_TO_METRIC

            return normal, distance
        else:
            return None, None
