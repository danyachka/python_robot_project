from pathlib import Path
import cv2 as cv
import numpy as np

from src.ananlysing_scripts.camera_script import ArucoDetector


def main():
    path = Path(__file__).parent.parent.joinpath("data/aruco.png")

    img = cv.imread(str(path))

    cameraMatrix = np.array([[1.27358341e+03, 0.00000000e+00, 3.00942489e+02],
                             [0.00000000e+00, 1.26663343e+03, 2.33213114e+02],
                             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    distCfs = np.array([[7.43051630e-02, -5.16983657e+00, -1.01402024e-03, -2.80294514e-04, 9.13089594e+01]])

    arucoDetector = ArucoDetector(cameraMatrix, distCfs)

    arucoDetector.onImage(img)


if __name__ == '__main__':
    main()
